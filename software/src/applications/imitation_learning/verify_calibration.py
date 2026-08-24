# -*- coding: utf-8 -*-
"""校准验证工具：对比 LeRobot 与 HomeBot 两套坐标系的关节读数

LeRobot 校准会把 homing offset 写入舵机寄存器。若校准流程中"行程中位"摆放的
是 HomeBot 零位姿态，两套坐标系应当重合——本工具分别用两套读法读取 6 个关节，
打印偏差表，用于校准后的验证。

注意：两套 SDK 不能同时打开同一串口，本工具采用"先后连接"方式读取。
运行方式:
    cd software/src
    python -m applications.imitation_learning verify-calibration --port COM23
"""
import time
from typing import Dict

from .joint_map import HOME2LEROBOT

# HomeBot 坐标换算（与 hal/arm/driver.py 一致）
ANGLE_OFFSET = 2048
ANGLE_SCALE = 4096.0 / 360.0


def _read_lerobot_degrees(port: str, baudrate: int) -> Dict[str, float]:
    """用 lerobot FeetechMotorsBus 读取校准后的关节角度（度）"""
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    from .robot import ARM_MOTOR_IDS

    motors = {
        name: Motor(sid, "sts3215",
                    MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.DEGREES)
        for name, sid in ARM_MOTOR_IDS.items()
    }
    bus = FeetechMotorsBus(port=port, motors=motors)
    bus.connect()
    try:
        if not bus.is_calibrated:
            raise RuntimeError("舵机未校准，请先运行 lerobot 校准流程")
        return bus.sync_read("Present_Position", list(ARM_MOTOR_IDS))
    finally:
        bus.disconnect(disable_torque=False)


def _read_homebot_degrees(port: str, baudrate: int) -> Dict[str, float]:
    """用 HomeBot FTServoBus 读取 raw 位置并按 HomeBot 公式换算为度"""
    from hal.ftservo_driver import FTServoBus
    from .robot import ARM_MOTOR_IDS

    bus = FTServoBus(port, baudrate)
    if not bus.connect():
        raise RuntimeError(f"无法连接串口 {port}")
    try:
        result = {}
        for name, sid in ARM_MOTOR_IDS.items():
            raw = bus.read_position(sid)
            if raw is None:
                raise RuntimeError(f"读取舵机 {sid} ({name}) 失败")
            result[name] = (raw - ANGLE_OFFSET) / ANGLE_SCALE
        return result
    finally:
        bus.disconnect()


def verify_calibration(port: str, baudrate: int = 1_000_000,
                       tolerance_deg: float = 2.0) -> bool:
    """对比两套坐标系读数，打印偏差表

    Args:
        port: 共用串口
        baudrate: 波特率
        tolerance_deg: 判定重合的最大允许偏差（度）

    Returns:
        是否所有关节偏差都在容忍范围内
    """
    lerobot_deg = _read_lerobot_degrees(port, baudrate)
    time.sleep(0.5)  # 串口释放
    homebot_raw_deg = _read_homebot_degrees(port, baudrate)

    # HomeBot 读数按 home 关节名，转到 lerobot 名对齐
    homebot_deg = {HOME2LEROBOT[k]: v for k, v in homebot_raw_deg.items()}

    print(f"\n{'关节':<15} {'LeRobot(°)':>12} {'HomeBot(°)':>12} {'偏差(°)':>10}")
    print("-" * 55)
    all_ok = True
    for name in lerobot_deg:
        if name == "gripper":
            # 夹爪单位不同（0-100 vs 度），换算后比较
            from .joint_map import gripper_lerobot_to_deg
            lr = gripper_lerobot_to_deg(lerobot_deg[name])
        else:
            lr = lerobot_deg[name]
        hb = homebot_deg[name]
        diff = lr - hb
        ok = abs(diff) <= tolerance_deg
        all_ok = all_ok and ok
        print(f"{name:<15} {lr:>12.2f} {hb:>12.2f} {diff:>+10.2f} {'OK' if ok else '超限'}")

    print("-" * 55)
    if all_ok:
        print(f"两套坐标系读数一致（容差 ±{tolerance_deg}°），校准对齐成功")
    else:
        print(f"存在超过 ±{tolerance_deg}° 的偏差，请重新校准（行程中位务必摆放在 HomeBot 零位姿态）")
    return all_ok
