# -*- coding: utf-8 -*-
"""底盘适配层：LeRobot 速度约定 ↔ HomeBot 两种底盘形态

LeRobot 侧约定（沿用 LeKiwi）：
- 观测/动作键: "x.vel"（m/s 前进）、"y.vel"（m/s 左移）、"theta.vel"（deg/s 逆时针）
- 轮子作为电机注册在同一个 FeetechMotorsBus 上，原始速度值读写

底盘形态：
- omni3: 三轮全向（main 分支，运动学与 LeKiwi 同构），轮舵机挂在共享串口总线上
- diff2: 双轮差动（homebot-navi 分支的 DiffChassisDriver，ESP32S3 控制器），
  无横向移动能力；该驱动在 main 分支上不存在，懒加载
- none: 纯机械臂模式

本模块不依赖 lerobot（运动学为纯函数，可独立测试）。
"""
import logging
import math
from abc import ABC, abstractmethod
from typing import Dict, Optional, Tuple

logger = logging.getLogger(__name__)

# 轮速原始值上限（ST3215 轮式模式速度寄存器范围）
WHEEL_MAX_RAW = 3250

# 三轮全向运动学矩阵参数：轮子安装方位角（度），从 X 轴正方向逆时针
# 与 hal/chassis/driver.py 的逆运动学一致：
#   v_lf   = -√3/2·vx - 0.5·vy - r·ω   （左前轮，240°）
#   v_rf   =  √3/2·vx - 0.5·vy - r·ω   （右前轮，120°）
#   v_rear =              vy - r·ω      （后轮，0°）
_OMNI3_WHEELS = ("wheel_left_front", "wheel_right_front", "wheel_rear")


def omni3_body_to_wheel_linear(vx: float, vy: float, omega_rads: float,
                               chassis_radius: float) -> Tuple[float, float, float]:
    """三轮全向正运动学输入的逆解：机体速度 → 各轮线速度 (m/s)

    Args:
        vx: 前进速度 (m/s)
        vy: 左移速度 (m/s)
        omega_rads: 角速度 (rad/s，逆时针为正)
        chassis_radius: 底盘半径（轮心到中心距离）(m)
    """
    sqrt3_2 = math.sqrt(3) / 2
    v_lf = -sqrt3_2 * vx - 0.5 * vy - chassis_radius * omega_rads
    v_rf = sqrt3_2 * vx - 0.5 * vy - chassis_radius * omega_rads
    v_rear = vy - chassis_radius * omega_rads
    return v_lf, v_rf, v_rear


def omni3_wheel_linear_to_body(v_lf: float, v_rf: float, v_rear: float,
                               chassis_radius: float) -> Tuple[float, float, float]:
    """各轮线速度 (m/s) → 机体速度 (vx, vy, omega_rads)

    正解矩阵 M = [[-√3/2, -0.5, -r], [√3/2, -0.5, -r], [0, 1, -r]]，闭式逆解：
      vx = (v_rf - v_lf) / √3
      ω  = -(v_lf + v_rf + v_rear) / (3r)
      vy = v_rear + r·ω
    """
    sqrt3 = math.sqrt(3)
    vx = (v_rf - v_lf) / sqrt3
    omega = -(v_lf + v_rf + v_rear) / (3.0 * chassis_radius)
    vy = v_rear + chassis_radius * omega
    return vx, vy, omega


def linear_to_wheel_raw(v_linear: float, max_linear: float, max_raw: int = WHEEL_MAX_RAW) -> int:
    """轮子线速度 (m/s) → 舵机原始速度值，与 HomeBot 底盘驱动的换算语义一致"""
    if max_linear <= 0:
        return 0
    raw = int(round(v_linear / max_linear * max_raw))
    return max(-max_raw, min(max_raw, raw))


def wheel_raw_to_linear(raw: int, max_linear: float, max_raw: int = WHEEL_MAX_RAW) -> float:
    """舵机原始速度值 → 轮子线速度 (m/s)"""
    if max_raw <= 0:
        return 0.0
    return raw / max_raw * max_linear


class ChassisAdapter(ABC):
    """底盘适配器抽象基类

    LeRobot 边界上的速度单位约定：vx/vy 为 m/s，theta 为 deg/s。
    """

    #: 是否有底盘（决定是否产生 x.vel/y.vel/theta.vel 键）
    has_base: bool = False

    @property
    def motor_specs(self) -> Dict[str, Tuple[int, str]]:
        """需要注册到 lerobot FeetechMotorsBus 的轮电机 {名称: (id, 型号)}"""
        return {}

    def bind_bus(self, bus) -> None:
        """注入 lerobot 电机总线（仅 motor_specs 非空的适配器需要）"""

    @abstractmethod
    def connect(self) -> None:
        """建立连接（共享总线模式下由 robot 统一 connect 后调用）"""

    @abstractmethod
    def set_velocity(self, vx: float, vy: float, theta_degs: float) -> None:
        """设置底盘速度（vx m/s 前进，vy m/s 左移，theta deg/s 逆时针）"""

    @abstractmethod
    def read_velocity(self) -> Tuple[float, float, float]:
        """读取当前底盘速度 (vx, vy, theta_degs)"""

    @abstractmethod
    def stop(self) -> None:
        """停止底盘"""

    def disconnect(self) -> None:
        """断开清理（默认先停车）"""
        self.stop()


class NullChassisAdapter(ChassisAdapter):
    """纯机械臂模式：无底盘"""

    has_base = False

    def connect(self) -> None:
        pass

    def set_velocity(self, vx: float, vy: float, theta_degs: float) -> None:
        if abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(theta_degs) > 1e-6:
            logger.warning("chassis_type=none，忽略底盘速度指令")

    def read_velocity(self) -> Tuple[float, float, float]:
        return 0.0, 0.0, 0.0

    def stop(self) -> None:
        pass


class Omni3ChassisAdapter(ChassisAdapter):
    """三轮全向底盘（main 分支形态，与 LeKiwi 同构）

    轮舵机注册在 lerobot FeetechMotorsBus 上（共享串口），速度经运动学换算后
    以原始值写入 Goal_Velocity。
    """

    has_base = True

    def __init__(self, left_front_id: int = 9, right_front_id: int = 8, rear_id: int = 7,
                 chassis_radius: float = 0.18, max_linear_speed: float = 0.5,
                 max_angular_speed: float = 1.0, max_raw: int = WHEEL_MAX_RAW):
        self._ids = {
            "wheel_left_front": left_front_id,
            "wheel_right_front": right_front_id,
            "wheel_rear": rear_id,
        }
        self._chassis_radius = chassis_radius
        self._max_linear = max_linear_speed
        self._max_angular = max_angular_speed  # rad/s
        self._max_raw = max_raw
        self._bus = None

    @property
    def motor_specs(self) -> Dict[str, Tuple[int, str]]:
        return {name: (sid, "sts3215") for name, sid in self._ids.items()}

    def bind_bus(self, bus) -> None:
        self._bus = bus

    def connect(self) -> None:
        """设置轮式模式（扭矩由 robot.configure 统一使能）"""
        from lerobot.motors.feetech import OperatingMode

        assert self._bus is not None, "bind_bus() 必须先于 connect() 调用"
        for name in self._ids:
            self._bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
        self.stop()

    def set_velocity(self, vx: float, vy: float, theta_degs: float) -> None:
        assert self._bus is not None, "bind_bus() 必须先于 set_velocity() 调用"
        omega = math.radians(theta_degs)
        # 限幅
        vx = max(-self._max_linear, min(self._max_linear, vx))
        vy = max(-self._max_linear, min(self._max_linear, vy))
        omega = max(-self._max_angular, min(self._max_angular, omega))

        wheel_linear = omni3_body_to_wheel_linear(vx, vy, omega, self._chassis_radius)
        raws = [linear_to_wheel_raw(v, self._max_linear, self._max_raw) for v in wheel_linear]
        self._bus.sync_write("Goal_Velocity", dict(zip(_OMNI3_WHEELS, raws)))

    def read_velocity(self) -> Tuple[float, float, float]:
        assert self._bus is not None, "bind_bus() 必须先于 read_velocity() 调用"
        raw = self._bus.sync_read("Present_Velocity", list(_OMNI3_WHEELS))
        wheel_linear = [
            wheel_raw_to_linear(raw[name], self._max_linear, self._max_raw)
            for name in _OMNI3_WHEELS
        ]
        vx, vy, omega = omni3_wheel_linear_to_body(*wheel_linear, self._chassis_radius)
        return vx, vy, math.degrees(omega)

    def stop(self) -> None:
        if self._bus is not None:
            self._bus.sync_write("Goal_Velocity", dict.fromkeys(_OMNI3_WHEELS, 0), num_retry=3)


class Diff2ChassisAdapter(ChassisAdapter):
    """双轮差动底盘（homebot-navi 分支形态，ESP32S3 控制器）

    DiffChassisDriver 位于 homebot-navi 分支（hal/chassis/diff_driver.py），
    main 分支合并前不可用——connect() 时懒加载并给出明确报错。
    差动底盘无横向移动能力：vy 非零时忽略并告警。
    该驱动使用独立的串口/协议通道，不注册到 lerobot 电机总线。
    """

    has_base = True

    def __init__(self, max_linear_speed: float = 0.5, max_angular_speed: float = 1.0):
        self._max_linear = max_linear_speed
        self._max_angular = max_angular_speed  # rad/s
        self._driver = None

    def connect(self) -> None:
        try:
            from hal.chassis.diff_driver import DiffChassisDriver
        except ImportError as e:
            raise RuntimeError(
                "双轮差动底盘驱动（hal/chassis/diff_driver.py）位于 homebot-navi 分支，"
                "待该分支合并到 main 后可用"
            ) from e
        self._driver = DiffChassisDriver()
        if not self._driver.initialize():
            raise RuntimeError("双轮差动底盘初始化失败")

    def set_velocity(self, vx: float, vy: float, theta_degs: float) -> None:
        assert self._driver is not None, "connect() 必须先于 set_velocity() 调用"
        if abs(vy) > 1e-6:
            logger.warning("差动底盘不支持横向移动，忽略 vy=%.3f", vy)
        vx = max(-self._max_linear, min(self._max_linear, vx))
        omega = max(-self._max_angular, min(self._max_angular, math.radians(theta_degs)))
        self._driver.move(vx, omega)

    def read_velocity(self) -> Tuple[float, float, float]:
        assert self._driver is not None, "connect() 必须先于 read_velocity() 调用"
        vx = getattr(self._driver, "_current_vx", 0.0)
        omega = getattr(self._driver, "_current_omega", 0.0)
        return vx, 0.0, math.degrees(omega)

    def stop(self) -> None:
        if self._driver is not None:
            self._driver.move(0.0, 0.0)

    def disconnect(self) -> None:
        if self._driver is not None:
            self.stop()
            close = getattr(self._driver, "close", None)
            if callable(close):
                close()


def create_chassis_adapter(chassis_type: str, config=None) -> ChassisAdapter:
    """底盘适配器工厂

    Args:
        chassis_type: "none" | "omni3" | "diff2"
        config: HomeBotRobotConfig（提供底盘参数），为 None 时用默认值
    """
    chassis_type = (chassis_type or "none").lower()
    if chassis_type == "none":
        return NullChassisAdapter()
    if chassis_type == "omni3":
        kwargs = {}
        if config is not None:
            kwargs = dict(
                left_front_id=config.wheel_left_front_id,
                right_front_id=config.wheel_right_front_id,
                rear_id=config.wheel_rear_id,
                chassis_radius=config.chassis_radius,
                max_linear_speed=config.chassis_max_linear_speed,
                max_angular_speed=config.chassis_max_angular_speed,
            )
        return Omni3ChassisAdapter(**kwargs)
    if chassis_type == "diff2":
        kwargs = {}
        if config is not None:
            kwargs = dict(
                max_linear_speed=config.chassis_max_linear_speed,
                max_angular_speed=config.chassis_max_angular_speed,
            )
        return Diff2ChassisAdapter(**kwargs)
    raise ValueError(f"未知底盘类型: {chassis_type}（可选: none / omni3 / diff2）")
