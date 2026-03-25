"""
底盘驱动 - 基于飞特 ST3215 舵机
实现三轮全向底盘的运动控制

从 configs.config.ChassisConfig 读取配置
"""
import math
import time
from typing import Dict, List, Optional, Tuple

from ..ftservo_driver import FTServoBus, ServoMode
from configs import ChassisConfig


class ChassisDriver:
    """
    底盘驱动器
    控制全向轮底盘的运动
    """

    def __init__(self, config: Optional[ChassisConfig] = None, bus: Optional[FTServoBus] = None):
        """
        初始化底盘驱动

        Args:
            config: 底盘配置，使用默认配置如果为None
            bus: 外部传入的舵机总线实例（用于共享串口），为None则自己创建
        """
        self.config = config or ChassisConfig()

        # 舵机总线 - 支持外部传入（共享模式）或自己创建（独立模式）
        if bus is not None:
            self.bus = bus
            self._shared_bus = True
        else:
            # 兼容 serial_port 和 port 两种属性名
            port = getattr(self.config, 'serial_port', None) or getattr(self.config, 'port', 'COM3')
            self.bus = FTServoBus(port, self.config.baudrate)
            self._shared_bus = False

        # ID映射
        self.wheel_ids = [
            self.config.left_front_id,
            self.config.right_front_id,
            self.config.rear_id,
        ]

        # 当前速度状态
        self._current_vx = 0.0
        self._current_vy = 0.0
        self._current_omega = 0.0

        self._initialized = False

    def initialize(self) -> bool:
        """
        初始化底盘
        - 连接串口（仅独立模式）
        - 设置轮式模式
        - 使能扭矩
        """
        print("[Chassis] Initializing...")

        # 如果是共享总线模式，检查总线是否已连接
        if self._shared_bus:
            if not self.bus.is_connected():
                print("[Chassis] Shared bus not connected")
                return False
            print("[Chassis] Using shared servo bus")
        else:
            # 独立模式，自己连接串口
            if not self.bus.connect():
                print("[Chassis] Failed to connect to servo bus")
                return False

        # 设置所有舵机为轮式模式
        for servo_id in self.wheel_ids:
            if not self.bus.set_wheel_mode(servo_id):
                print(f"[Chassis] Failed to set wheel mode for servo {servo_id}")
                return False
            time.sleep(0.01)  # 短暂延时

        # 使能扭矩
        self.bus.torque_enable()
        time.sleep(0.1)

        # 停止所有轮子
        self.stop()

        self._initialized = True
        print(f"[Chassis] Initialized with wheels: {self.wheel_ids}")
        return True

    def stop(self) -> None:
        """停止底盘运动"""
        self._current_vx = 0.0
        self._current_vy = 0.0
        self._current_omega = 0.0

        for servo_id in self.wheel_ids:
            self.bus.write_speed(servo_id, 0)

    def set_velocity(self, vx: float, vy: float, omega: float) -> bool:
        """
        设置底盘速度（全向轮逆运动学）

        Args:
            vx: X方向速度（前进为正）(m/s)
            vy: Y方向速度（左移为正）(m/s)
            omega: Z方向角速度（逆时针为正）(rad/s)

        Returns:
            是否设置成功
        """
        if not self._initialized:
            print("[Chassis] Not initialized")
            return False

        # 限制速度范围
        vx = max(-self.config.max_linear_speed, min(self.config.max_linear_speed, vx))
        vy = max(-self.config.max_linear_speed, min(self.config.max_linear_speed, vy))
        omega = max(-self.config.max_angular_speed, min(self.config.max_angular_speed, omega))

        # 保存当前速度
        self._current_vx = vx
        self._current_vy = vy
        self._current_omega = omega

        # 三轮全向轮逆运动学
        # 轮子速度 = (vx, vy, omega) -> 各轮速度
        wheel_speeds = self._inverse_kinematics(vx, vy, omega)
        # print(wheel_speeds)
        # 转换为舵机速度值并发送
        for i, servo_id in enumerate(self.wheel_ids):
            speed = wheel_speeds[i]
            servo_speed = self._wheel_speed_to_servo(speed)
            self.bus.write_speed(servo_id, servo_speed)

        return True

    def _inverse_kinematics(self, vx: float, vy: float, omega: float) -> List[float]:
        """
        三轮全向轮逆运动学
        计算各轮子的线速度

        布局 (从上方看，X轴向前，Y轴向左):
        - 左前轮: -60度方向 (从X轴逆时针60度)
        - 右前轮: 60度方向 (从X轴顺时针60度)
        - 后轮: 180度方向 (正后方)

        轮子速度公式: V_wheel = vx * sin(θ) + vy * cos(θ) + r * omega
        其中 θ 是轮子安装角度，从X轴正方向逆时针旋转

        Returns:
            [left_front_speed, right_front_speed, rear_speed] (m/s)
        """
        r = self.config.chassis_radius

        sqrt3_2 = math.sqrt(3) / 2

        v_left_front = -sqrt3_2 * vx - 0.5 * vy - r * omega

        v_right_front = sqrt3_2 * vx - 0.5 * vy - r * omega

        v_rear = vy - r * omega

        return [v_left_front, v_right_front, v_rear]

    def _wheel_speed_to_servo(self, wheel_speed: float) -> int:
        """
        将轮子线速度 (m/s) 转换为舵机速度值

        ST3215 速度值说明:
        - 值范围: -3250 ~ 3250
        - 单位: 约 0.0146 rpm / 单位
        - 转换为线速度: V(m/s) = speed * 0.0146 * (2*PI*R) / 60
        """
        # 最大舵机速度对应的线速度
        max_speed_value = self.config.default_wheel_speed
        max_linear_speed = self.config.max_linear_speed

        # 速度比例
        speed_ratio = wheel_speed / max_linear_speed if max_linear_speed > 0 else 0

        # 转换为舵机速度值
        servo_speed = int(speed_ratio * max_speed_value)

        # 限制范围
        return max(-3250, min(3250, servo_speed))

    def move_forward(self, distance: float, speed: float = 0.2) -> bool:
        """
        前进指定距离

        Args:
            distance: 距离 (m)，正数前进，负数后退
            speed: 速度 (m/s)
        """
        if distance == 0:
            return True

        direction = 1 if distance > 0 else -1
        duration = abs(distance) / speed

        self.set_velocity(direction * speed, 0, 0)
        time.sleep(duration)
        self.stop()

        return True

    def rotate(self, angle_deg: float, angular_speed: float = 90.0) -> bool:
        """
        旋转指定角度

        Args:
            angle_deg: 角度（度），正数逆时针
            angular_speed: 角速度（度/s）
        """
        if angle_deg == 0:
            return True

        direction = 1 if angle_deg > 0 else -1
        omega_rad = math.radians(angular_speed)
        duration = abs(angle_deg) / angular_speed

        self.set_velocity(0, 0, direction * omega_rad)
        time.sleep(duration)
        self.stop()

        return True

    def get_current_velocity(self) -> Tuple[float, float, float]:
        """获取当前速度 (vx, vy, omega)"""
        return (self._current_vx, self._current_vy, self._current_omega)

    def close(self) -> None:
        """关闭底盘驱动"""
        if not self._initialized:
            return  # 已经关闭，避免重复操作
        self.stop()
        time.sleep(0.1)
        self.bus.torque_disable()
        time.sleep(0.1)
        self.bus.disconnect()
        self._initialized = False
        print("[Chassis] Closed")


class OmniWheelKinematics:
    """
    全向轮运动学工具类
    用于速度计算和坐标转换
    """

    @staticmethod
    def world_to_robot(vx_world: float, vy_world: float, theta: float) -> Tuple[float, float]:
        """
        将世界坐标系速度转换为机器人坐标系速度

        Args:
            vx_world: 世界坐标系X速度
            vy_world: 世界坐标系Y速度
            theta: 机器人当前朝向（弧度）

        Returns:
            (vx_robot, vy_robot)
        """
        vx_robot = vx_world * math.cos(theta) + vy_world * math.sin(theta)
        vy_robot = -vx_world * math.sin(theta) + vy_world * math.cos(theta)
        return vx_robot, vy_robot

    @staticmethod
    def robot_to_world(vx_robot: float, vy_robot: float, theta: float) -> Tuple[float, float]:
        """
        将机器人坐标系速度转换为世界坐标系速度
        """
        vx_world = vx_robot * math.cos(theta) - vy_robot * math.sin(theta)
        vy_world = vx_robot * math.sin(theta) + vy_robot * math.cos(theta)
        return vx_world, vy_world
