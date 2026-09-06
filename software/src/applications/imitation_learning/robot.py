# -*- coding: utf-8 -*-
"""HomeBot LeRobot 机器人适配器

将 HomeBot（SO-101 机械臂 + 可选底盘）注册为 LeRobot 机器人（type="homebot"），
使 lerobot.record / lerobot.teleoperate 等官方工具链可直接使用。

校准约定（重要）：
- 机械臂采用 LeRobot 标准校准（homing offset 写入舵机，校准文件存
  ~/.cache/lerobot/calibration/robots/homebot/<id>.json）
- 执行校准时，必须先将机械臂摆放到 HomeBot 的零位姿态（各关节 0°），
  使 LeRobot 校准后的 0° 与 HomeBot 坐标系（0°=raw 2048）重合，
  这样 HomeBot 现有功能（web/手柄/语音/遥操作）的读数不受影响
- 校准后可运行 verify_calibration.py 验证两侧读数一致

用法（数采，需先安装 lerobot）：
    lerobot.record \\
        --robot.type=homebot \\
        --robot.port=COM23 \\
        --robot.chassis_type=omni3 \\
        --robot.cameras="{cam_front: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \\
        --teleop.type=so101_leader --teleop.port=COMxx ...
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from functools import cached_property

from .chassis_adapter import create_chassis_adapter
from .joint_map import LEROBOT_JOINTS

logger = logging.getLogger(__name__)

# 手臂关节名 -> 舵机 ID（与 HomeBot ArmConfig 一致：base=1 ... gripper=6）
ARM_MOTOR_IDS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

try:
    from lerobot.cameras import make_cameras_from_configs
    from lerobot.cameras.configs import CameraConfig
    from lerobot.types import RobotAction, RobotObservation
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
    from lerobot.robots import Robot, RobotConfig
    from lerobot.robots.utils import ensure_safe_goal_position
    from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

    LEROBOT_AVAILABLE = True
except ImportError as e:  # pragma: no cover - 取决于运行环境
    LEROBOT_AVAILABLE = False
    _LEROBOT_IMPORT_ERROR = e

    # 占位基类与空装饰器，使模块在 lerobot 未安装时也可被 import（实例化时才报错）
    class Robot:  # type: ignore
        pass

    class RobotConfig:  # type: ignore
        pass

    def _noop_decorator(fn):  # type: ignore
        return fn

    check_if_already_connected = _noop_decorator
    check_if_not_connected = _noop_decorator


if LEROBOT_AVAILABLE:

    @RobotConfig.register_subclass("homebot")
    @dataclass(kw_only=True)
    class HomeBotRobotConfig(RobotConfig):
        """HomeBot 机器人配置（lerobot draccus 注册类型 "homebot"）"""

        # 串口（机械臂与底盘轮舵机共用）
        port: str = "COM23"
        baudrate: int = 1_000_000

        # 手臂关节单位：True=度（推荐），False=归一化 -100~100
        use_degrees: bool = True

        # 安全限制：单步目标位置与当前位置的最大偏差（度），None=不限制
        max_relative_target: float | None = None

        num_read_retries: int = 3
        disable_torque_on_disconnect: bool = True

        # 底盘形态：none / omni3 / diff2
        chassis_type: str = "none"

        # 相机 {名称: CameraConfig}
        cameras: dict[str, CameraConfig] = field(default_factory=dict)

        # ---- omni3 底盘参数（与 ChassisConfig 默认值一致）----
        wheel_left_front_id: int = 9
        wheel_right_front_id: int = 8
        wheel_rear_id: int = 7
        chassis_radius: float = 0.18          # 轮心到中心距离 (m)
        chassis_max_linear_speed: float = 0.5  # m/s
        chassis_max_angular_speed: float = 1.0  # rad/s

        # 舵机 PID（参照 lerobot so_follower/lekiwi 推荐值）
        position_p_coefficient: int = 16
        position_i_coefficient: int = 0
        position_d_coefficient: int = 32

else:

    @dataclass(kw_only=True)
    class HomeBotRobotConfig(RobotConfig):  # type: ignore
        """占位配置类（lerobot 未安装）"""

        port: str = "COM23"


class HomeBotRobot(Robot):
    """HomeBot 机器人：SO-101 机械臂 + 可选底盘（omni3/diff2/none）+ 相机"""

    config_class = HomeBotRobotConfig
    name = "homebot"

    def __init__(self, config: HomeBotRobotConfig):
        if not LEROBOT_AVAILABLE:
            raise ImportError(
                f"HomeBotRobot 需要安装 lerobot（pip install lerobot 或 "
                f"pip install -r requirements-lerobot.txt）: {_LEROBOT_IMPORT_ERROR}"
            )
        super().__init__(config)
        self.config = config

        # 底盘适配器（决定是否注册轮电机）
        self.chassis = create_chassis_adapter(config.chassis_type, config)

        # 电机总线：手臂 6 关节 + 底盘轮电机，共用同一串口
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        motors = {
            name: Motor(sid, "sts3215", norm_mode_body)
            for name, sid in ARM_MOTOR_IDS.items() if name != "gripper"
        }
        motors["gripper"] = Motor(ARM_MOTOR_IDS["gripper"], "sts3215", MotorNormMode.RANGE_0_100)
        for wheel_name, (sid, model) in self.chassis.motor_specs.items():
            motors[wheel_name] = Motor(sid, model, MotorNormMode.RANGE_M100_100)

        self.bus = FeetechMotorsBus(
            port=config.port,
            motors=motors,
            calibration=self.calibration,
        )
        self.chassis.bind_bus(self.bus)
        self.cameras = make_cameras_from_configs(config.cameras)

    # ---------- features ----------

    @property
    def _arm_ft(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in LEROBOT_JOINTS}

    @property
    def _base_ft(self) -> dict[str, type]:
        if not self.chassis.has_base:
            return {}
        return {"x.vel": float, "y.vel": float, "theta.vel": float}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        # features 需在未连接时也可调用，以 config 为准
        return {
            cam: (cfg.height, cfg.width, 3)
            for cam, cfg in self.config.cameras.items()
        }

    @cached_property
    def observation_features(self) -> dict:
        return {**self._arm_ft, **self._base_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict:
        return {**self._arm_ft, **self._base_ft}

    # ---------- 连接与校准 ----------

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info("未找到校准文件或校准不匹配，进入校准流程")
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.chassis.connect()
        self.configure()
        logger.info("%s connected.", self)

    def calibrate(self) -> None:
        """LeRobot 标准校准流程

        注意：摆放行程中位时，请将机械臂摆到 HomeBot 的零位姿态
        （各关节 0°，即 rest_position 中 base/wrist_flex/wrist_roll=0 的姿态），
        使 LeRobot 坐标系与 HomeBot 坐标系重合。
        """
        if self.calibration:
            user_input = input(
                f"按回车使用 id={self.id} 的已有校准文件，或输入 'c' 回车重新校准: "
            )
            if user_input.strip().lower() != "c":
                logger.info("将已有校准写入舵机")
                self.bus.write_calibration(self.calibration)
                return

        logger.info("开始校准 %s", self)
        self.bus.disable_torque(list(ARM_MOTOR_IDS))
        for name in ARM_MOTOR_IDS:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)

        input("将机械臂摆放到【HomeBot 零位姿态】（各关节 0°）后按回车...")
        homing_offsets = self.bus.set_half_turn_homings(list(ARM_MOTOR_IDS))
        # 轮电机无位置含义
        for wheel_name in self.chassis.motor_specs:
            homing_offsets[wheel_name] = 0

        full_turn_motors = ["wrist_roll", *self.chassis.motor_specs.keys()]
        ranged_motors = [m for m in self.bus.motors if m not in full_turn_motors]
        print("依次全行程活动各关节（wrist_roll 除外），记录范围。完成后按回车...")
        range_mins, range_maxes = self.bus.record_ranges_of_motion(ranged_motors)
        for name in full_turn_motors:
            range_mins[name] = 0
            range_maxes[name] = 4095

        self.calibration = {}
        for name, motor in self.bus.motors.items():
            self.calibration[name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=homing_offsets[name],
                range_min=range_mins[name],
                range_max=range_maxes[name],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("校准已保存到", self.calibration_fpath)

    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for name in ARM_MOTOR_IDS:
                self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
                self.bus.write("P_Coefficient", name, self.config.position_p_coefficient)
                self.bus.write("I_Coefficient", name, self.config.position_i_coefficient)
                self.bus.write("D_Coefficient", name, self.config.position_d_coefficient)
                if name == "gripper":
                    # 限流保护夹爪（参照 so_follower）
                    self.bus.write("Max_Torque_Limit", name, 500)
                    self.bus.write("Protection_Current", name, 250)
                    self.bus.write("Overload_Torque", name, 25)

    # ---------- 观测与动作 ----------

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()
        arm_pos = self.bus.sync_read("Present_Position", list(ARM_MOTOR_IDS),
                                     num_retry=self.config.num_read_retries)
        obs = {f"{name}.pos": val for name, val in arm_pos.items()}

        if self.chassis.has_base:
            vx, vy, theta = self.chassis.read_velocity()
            obs.update({"x.vel": vx, "y.vel": vy, "theta.vel": theta})

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug("read state: %.1fms", dt_ms)

        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()

        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        arm_goal = {k: v for k, v in action.items() if k.endswith(".pos")}
        base_goal = {k: v for k, v in action.items() if k.endswith(".vel")}

        # 安全限幅：目标与当前位置偏差过大时截断
        if self.config.max_relative_target is not None:
            present = self.bus.sync_read("Present_Position", list(ARM_MOTOR_IDS),
                                         num_retry=self.config.num_read_retries)
            goal_present = {
                key: (g, present[key.removesuffix(".pos")]) for key, g in arm_goal.items()
            }
            arm_goal = ensure_safe_goal_position(goal_present, self.config.max_relative_target)

        if arm_goal:
            self.bus.sync_write("Goal_Position",
                                {k.removesuffix(".pos"): v for k, v in arm_goal.items()})

        if base_goal and self.chassis.has_base:
            self.chassis.set_velocity(
                base_goal.get("x.vel", 0.0),
                base_goal.get("y.vel", 0.0),
                base_goal.get("theta.vel", 0.0),
            )

        return {**arm_goal, **base_goal}

    @check_if_not_connected
    def disconnect(self) -> None:
        self.chassis.disconnect()
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()
        logger.info("%s disconnected.", self)
