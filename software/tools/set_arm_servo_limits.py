#!/usr/bin/env python3
"""
批量设置机械臂舵机位置限制为 0~4095

使用说明：
    cd software/src
    python -m tools.set_arm_servo_limits --port /dev/ttyFollower

注意：
    - 修改的是舵机 EPROM 中的 Min/Max Angle Limit
    - 执行前请确保机械臂处于安全位置
    - 修改后舵机可以转动到 0~4095 整个范围
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import argparse

from common.logging import get_logger
from configs.config import get_config
from hal.ftservo_driver import FTServoBus
from hal.scservo_sdk import sms_sts, PortHandler

logger = get_logger(__name__)

# SMS_STS 寄存器地址
SMS_STS_MIN_ANGLE_LIMIT_L = 9
SMS_STS_MIN_ANGLE_LIMIT_H = 10
SMS_STS_MAX_ANGLE_LIMIT_L = 11
SMS_STS_MAX_ANGLE_LIMIT_H = 12
SMS_STS_LOCK = 55


def read_angle_limits(packet_handler, servo_id: int) -> tuple:
    """读取舵机当前角度限制 (min, max)"""
    data, result, error = packet_handler.readTxRx(servo_id, SMS_STS_MIN_ANGLE_LIMIT_L, 4)
    if result != 0:
        return None
    min_limit = data[0] | (data[1] << 8)
    max_limit = data[2] | (data[3] << 8)
    return min_limit, max_limit


def write_angle_limits(packet_handler, servo_id: int, min_limit: int, max_limit: int) -> bool:
    """写入舵机角度限制"""
    txpacket = [
        min_limit & 0xFF,
        (min_limit >> 8) & 0xFF,
        max_limit & 0xFF,
        (max_limit >> 8) & 0xFF,
    ]
    result, error = packet_handler.writeTxRx(servo_id, SMS_STS_MIN_ANGLE_LIMIT_L, 4, txpacket)
    return result == 0


def unlock_eprom(packet_handler, servo_id: int) -> bool:
    """解锁 EPROM"""
    result, error = packet_handler.write1ByteTxRx(servo_id, SMS_STS_LOCK, 0)
    return result == 0


def lock_eprom(packet_handler, servo_id: int) -> bool:
    """锁定 EPROM"""
    result, error = packet_handler.write1ByteTxRx(servo_id, SMS_STS_LOCK, 1)
    return result == 0


def main():
    parser = argparse.ArgumentParser(description="批量设置机械臂舵机位置限制")
    parser.add_argument("--port", default=None, help="舵机串口，默认从配置读取")
    parser.add_argument("--baud", type=int, default=1000000, help="波特率，默认 1000000")
    parser.add_argument("--ids", type=int, nargs="+", default=None,
                        help="舵机ID列表，默认从配置读取机械臂关节ID")
    parser.add_argument("--min", type=int, default=0, help="最小位置限制，默认 0")
    parser.add_argument("--max", type=int, default=4095, help="最大位置限制，默认 4095")
    parser.add_argument("--dry-run", action="store_true", help="只读取当前限制，不写入")
    args = parser.parse_args()

    config = get_config()

    # 串口
    port = args.port or config.arm.serial_port
    baud = args.baud

    # 舵机ID
    if args.ids:
        servo_ids = args.ids
    else:
        servo_ids = [
            config.arm.base_id,
            config.arm.shoulder_id,
            config.arm.elbow_id,
            config.arm.wrist_flex_id,
            config.arm.wrist_roll_id,
            config.arm.gripper_id,
        ]

    print("=" * 60)
    print("机械臂舵机位置限制设置工具")
    print("=" * 60)
    print(f"串口: {port}")
    print(f"波特率: {baud}")
    print(f"目标舵机ID: {servo_ids}")
    if not args.dry_run:
        print(f"将设置限制范围: {args.min} ~ {args.max}")
    print()

    # 连接串口
    port_handler = PortHandler(port)
    if not port_handler.openPort():
        print(f"[错误] 无法打开串口: {port}")
        return 1
    if not port_handler.setBaudRate(baud):
        print(f"[错误] 无法设置波特率: {baud}")
        return 1

    packet_handler = sms_sts(port_handler)

    try:
        for servo_id in servo_ids:
            print(f"\n[ID:{servo_id:03d}] 处理中...")

            # 读取当前限制
            current = read_angle_limits(packet_handler, servo_id)
            if current is None:
                print(f"  [警告] 无法读取当前限制，跳过")
                continue
            print(f"  当前限制: {current[0]} ~ {current[1]}")

            if args.dry_run:
                continue

            # 解锁 EPROM
            if not unlock_eprom(packet_handler, servo_id):
                print(f"  [错误] 解锁 EPROM 失败")
                continue
            print(f"  EPROM 已解锁")

            # 写入新限制
            if not write_angle_limits(packet_handler, servo_id, args.min, args.max):
                print(f"  [错误] 写入限制失败")
                lock_eprom(packet_handler, servo_id)
                continue
            print(f"  已写入限制: {args.min} ~ {args.max}")

            # 锁定 EPROM
            if not lock_eprom(packet_handler, servo_id):
                print(f"  [警告] 锁定 EPROM 失败")
                continue
            print(f"  EPROM 已锁定")

            # 再次读取确认
            new_limits = read_angle_limits(packet_handler, servo_id)
            if new_limits:
                print(f"  验证限制: {new_limits[0]} ~ {new_limits[1]}")

        print("\n" + "=" * 60)
        if args.dry_run:
            print("干运行完成，未写入任何数据")
        else:
            print("设置完成")
        print("=" * 60)

    finally:
        port_handler.closePort()

    return 0


if __name__ == "__main__":
    sys.exit(main())
