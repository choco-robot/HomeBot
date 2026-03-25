"""
机械臂驱动 - 基于飞特 ST3215 舵机
实现多自由度机械臂的位置控制

机械臂关节定义 (以5DOF为例):
    J1: 基座旋转 (Waist)
    J2: 肩关节 (Shoulder)
    J3: 肘关节 (Elbow)
    J4: 腕关节1 (Wrist 1)
    J5: 腕关节2 / 夹爪 (Wrist 2 / Gripper)
"""
import time
from typing import Dict, List, Optional, Tuple, Union
from dataclasses import dataclass, field

from ..ftservo_driver import FTServoBus, ServoConfig, ServoState


@dataclass
class ArmConfig:
    """机械臂配置"""
    # 舵机ID映射 {joint_name: servo_id}
    joint_ids: Dict[str, int] = field(default_factory=lambda: {
        "waist": 4,      # 基座旋转
        "shoulder": 5,   # 肩关节
        "elbow": 6,      # 肘关节
        "wrist": 7,      # 腕关节
        "gripper": 8,    # 夹爪
    })

    # 关节角度限制 (度)
    joint_limits: Dict[str, Tuple[int, int]] = field(default_factory=lambda: {
        "waist": (-180, 180),
        "shoulder": (-90, 90),
        "elbow": (-120, 120),
        "wrist": (-90, 90),
        "gripper": (0, 90),
    })

    # 默认速度/加速度
    default_speed: int = 1000   # 位置模式下的运动速度
    default_acc: int = 50       # 加速度

    # 初始位置 (度)
    home_position: Dict[str, int] = field(default_factory=lambda: {
        "waist": 0,
        "shoulder": 0,
        "elbow": 90,
        "wrist": 0,
        "gripper": 45,
    })

    # 串口配置
    port: str = "/dev/ttyUSB0"
    baudrate: int = 1000000

    # 角度到位置值的转换系数
    # ST3215: 0-4095 对应 0-360度
    # 0度对应位置 2048
    angle_offset: int = 2048
    angle_scale: float = 4096 / 360  # 约 11.38 位置值/度


class ArmDriver:
    """
    机械臂驱动器
    控制多关节机械臂的位置
    """

    def __init__(self, config: Optional[ArmConfig] = None, bus: Optional[FTServoBus] = None):
        """
        初始化机械臂驱动

        Args:
            config: 机械臂配置
            bus: 外部传入的舵机总线实例（用于共享串口），为None则自己创建
        """
        self.config = config or ArmConfig()

        # 舵机总线 - 支持外部传入（共享模式）或自己创建（独立模式）
        if bus is not None:
            self.bus = bus
            self._shared_bus = True
        else:
            self.bus = FTServoBus(self.config.port, self.config.baudrate)
            self._shared_bus = False

        # 当前关节角度（度）
        self._current_angles: Dict[str, float] = {}

        # 当前夹爪状态
        self._gripper_open = False

        self._initialized = False

    def initialize(self, auto_home: bool = False) -> bool:
        """
        初始化机械臂
        - 连接串口（仅独立模式）
        - 设置位置模式
        - 使能扭矩
        - 读取当前位置
        - 可选：移动到初始位置
        
        Args:
            auto_home: 是否自动复位到 home 位置，默认 False
                      设为 True 时启动后会移动到配置的休息位置
        """
        print(f"[Arm] Initializing... (auto_home={auto_home})")

        # 如果是共享总线模式，检查总线是否已连接
        if self._shared_bus:
            if not self.bus.is_connected():
                print("[Arm] Shared bus not connected")
                return False
            print("[Arm] Using shared servo bus")
        else:
            # 独立模式，自己连接串口
            if not self.bus.connect():
                print("[Arm] Failed to connect to servo bus")
                return False

        # 使能扭矩
        self.bus.torque_enable()
        time.sleep(0.1)

        # 读取当前位置（从实际舵机读取）
        self._read_current_positions()

        # 根据配置决定是否移动到初始位置
        if auto_home:
            self.move_to_home()
        else:
            print(f"[Arm] 保持当前位置，不自动复位")
            print(f"[Arm] 当前角度: {self._current_angles}")

        self._initialized = True
        print(f"[Arm] Initialized with joints: {list(self.config.joint_ids.keys())}")
        return True

    def _read_current_positions(self) -> None:
        """读取当前所有关节位置"""
        positions = self.bus.sync_read_positions(list(self.config.joint_ids.values()))
        for joint_name, servo_id in self.config.joint_ids.items():
            if servo_id in positions:
                angle = self._pos_to_angle(positions[servo_id])
                self._current_angles[joint_name] = angle
            else:
                # 如果读取失败，使用默认值
                self._current_angles[joint_name] = self.config.home_position.get(joint_name, 0)

    def _angle_to_pos(self, angle: float) -> int:
        """
        将角度转换为舵机位置值

        Args:
            angle: 角度（度），0为中立位置

        Returns:
            位置值 (0-4095)
        """
        pos = int(self.config.angle_offset + angle * self.config.angle_scale)
        return max(0, min(4095, pos))

    def _pos_to_angle(self, pos: int) -> float:
        """
        将舵机位置值转换为角度
        """
        return (pos - self.config.angle_offset) / self.config.angle_scale

    def _clamp_angle(self, joint_name: str, angle: float) -> float:
        """限制关节角度在有效范围内"""
        if joint_name in self.config.joint_limits:
            min_angle, max_angle = self.config.joint_limits[joint_name]
            return max(min_angle, min(max_angle, angle))
        return angle

    def set_joint_angle(self, joint_name: str, angle: float,
                        speed: Optional[int] = None,
                        wait: bool = False) -> bool:
        """
        设置单个关节角度

        Args:
            joint_name: 关节名称
            angle: 目标角度（度）
            speed: 运动速度，None使用默认值
            wait: 是否等待运动完成

        Returns:
            是否设置成功
        """
        if not self._initialized:
            print("[Arm] Not initialized")
            return False

        if joint_name not in self.config.joint_ids:
            print(f"[Arm] Unknown joint: {joint_name}")
            return False

        # 限制角度范围
        angle = self._clamp_angle(joint_name, angle)

        # 获取舵机ID
        servo_id = self.config.joint_ids[joint_name]

        # 转换为目标位置
        position = self._angle_to_pos(angle)

        # 获取速度
        if speed is None:
            speed = self.config.default_speed

        # 发送指令
        success = self.bus.write_position(
            servo_id, position, speed, self.config.default_acc
        )

        if success:
            self._current_angles[joint_name] = angle

        if wait:
            self._wait_for_move(servo_id, position)

        return success

    def set_joint_angles(self, angles: Dict[str, float],
                         speed: Optional[int] = None,
                         wait: bool = False) -> bool:
        """
        同时设置多个关节角度 - 使用批量写入优化性能

        Args:
            angles: {joint_name: angle, ...}
            speed: 运动速度
            wait: 是否等待运动完成
        """
        if not self._initialized:
            return False

        if speed is None:
            speed = self.config.default_speed

        # 使用批量写入替代逐个写入，性能提升约6倍
        positions = {}  # {servo_id: (position, speed, acc), ...}
        servo_targets = []  # [(servo_id, target_position), ...]

        for joint_name, angle in angles.items():
            if joint_name not in self.config.joint_ids:
                print(f"[Arm] Unknown joint: {joint_name}")
                continue

            # 限制角度
            angle = self._clamp_angle(joint_name, angle)

            # 获取舵机ID和目标位置
            servo_id = self.config.joint_ids[joint_name]
            position = self._angle_to_pos(angle)

            # 添加到批量写入字典
            positions[servo_id] = (position, speed, self.config.default_acc)
            servo_targets.append((servo_id, position))
            self._current_angles[joint_name] = angle

        # 批量写入所有舵机（只有一次串口通信）
        all_success = True
        if positions:
            all_success = self.bus.sync_write_positions(positions)
            if not all_success:
                print(f"[Arm] Batch write failed for {len(positions)} servos")

        if wait and servo_targets:
            # 等待所有舵机运动完成
            time.sleep(0.1)
            for servo_id, target_pos in servo_targets:
                self._wait_for_move(servo_id, target_pos)

        return all_success

    def move_to_home(self, speed: Optional[int] = None) -> bool:
        """移动到初始位置"""
        print("[Arm] Moving to home position...")
        return self.set_joint_angles(self.config.home_position, speed, wait=True)

    def set_gripper(self, open_amount: float, speed: Optional[int] = None) -> bool:
        """
        控制夹爪

        Args:
            open_amount: 开合程度 (0.0-1.0)，0=完全关闭，1=完全打开
        """
        if "gripper" not in self.config.joint_ids:
            print("[Arm] No gripper configured")
            return False

        # 获取夹爪角度限制
        min_angle, max_angle = self.config.joint_limits.get("gripper", (0, 90))

        # 计算目标角度
        target_angle = min_angle + open_amount * (max_angle - min_angle)

        return self.set_joint_angle("gripper", target_angle, speed)

    def open_gripper(self, speed: Optional[int] = None) -> bool:
        """打开夹爪"""
        success = self.set_gripper(1.0, speed)
        if success:
            self._gripper_open = True
        return success

    def close_gripper(self, speed: Optional[int] = None) -> bool:
        """关闭夹爪"""
        success = self.set_gripper(0.0, speed)
        if success:
            self._gripper_open = False
        return success

    def _wait_for_move(self, servo_id: int, target_position: int, 
                        timeout: float = 5.0, tolerance: int = 20) -> bool:
        """
        等待舵机运动到目标位置
        
        Args:
            servo_id: 舵机ID
            target_position: 目标位置值
            timeout: 超时时间（秒）
            tolerance: 位置容差（编码值）
            
        Returns:
            是否成功到达目标位置
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_pos = self.bus.read_position(servo_id)
            if current_pos is not None:
                if abs(current_pos - target_position) <= tolerance:
                    return True
            time.sleep(0.05)
        return False

    def get_joint_angle(self, joint_name: str) -> Optional[float]:
        """获取当前关节角度"""
        if joint_name not in self.config.joint_ids:
            return None

        # 先尝试从缓存获取
        if joint_name in self._current_angles:
            return self._current_angles[joint_name]

        # 从舵机读取
        servo_id = self.config.joint_ids[joint_name]
        pos = self.bus.read_position(servo_id)
        if pos is not None:
            angle = self._pos_to_angle(pos)
            self._current_angles[joint_name] = angle
            return angle

        return None

    def get_all_joint_angles(self) -> Dict[str, float]:
        """获取所有关节当前角度"""
        self._read_current_positions()
        return self._current_angles.copy()

    def get_joint_states(self) -> Dict[str, Optional[ServoState]]:
        """获取所有关节状态"""
        states = {}
        for joint_name, servo_id in self.config.joint_ids.items():
            states[joint_name] = self.bus.get_state(servo_id)
        return states

    def enable_torque(self) -> bool:
        """使能所有关节扭矩"""
        return self.bus.torque_enable()

    def disable_torque(self) -> bool:
        """失能所有关节扭矩（可手动掰动）"""
        return self.bus.torque_disable()

    def emergency_stop(self) -> None:
        """紧急停止"""
        print("[Arm] Emergency stop!")
        # 可以在这里添加特定的停止逻辑
        self.disable_torque()

    def close(self) -> None:
        """关闭机械臂驱动"""
        if not self._initialized:
            return  # 已经关闭，避免重复操作
        print("[Arm] Closing...")
        self.move_to_home()
        time.sleep(0.5)
        self.disable_torque()
        time.sleep(0.1)
        # 仅独立模式需要断开总线
        if not self._shared_bus:
            self.bus.disconnect()
        self._initialized = False
        print("[Arm] Closed")


class ArmKinematics:
    """
    机械臂运动学工具类
    提供正逆运动学计算（简化版，针对特定构型）
    """

    def __init__(self, link_lengths: List[float] = None):
        """
        初始化运动学

        Args:
            link_lengths: 连杆长度 [L1, L2, L3, ...] (m)
        """
        self.link_lengths = link_lengths or [0.1, 0.1, 0.08]

    def forward_kinematics(self, joint_angles: List[float]) -> Tuple[float, float, float]:
        """
        正运动学：关节角度 -> 末端位置

        Args:
            joint_angles: [waist, shoulder, elbow, ...] (度)

        Returns:
            (x, y, z) 末端位置
        """
        import math

        if len(joint_angles) < 2:
            return (0.0, 0.0, 0.0)

        # 简化的2D平面运动学（针对平面机械臂）
        theta1 = math.radians(joint_angles[0])  # 基座旋转
        theta2 = math.radians(joint_angles[1])  # 肩关节
        theta3 = math.radians(joint_angles[2]) if len(joint_angles) > 2 else 0  # 肘关节

        L1, L2, L3 = self.link_lengths[:3]

        # 计算平面位置
        x_plane = (L1 * math.cos(theta2) +
                   L2 * math.cos(theta2 + theta3) +
                   L3 * math.cos(theta2 + theta3))
        z_plane = (L1 * math.sin(theta2) +
                   L2 * math.sin(theta2 + theta3) +
                   L3 * math.sin(theta2 + theta3))

        # 旋转到3D空间
        x = x_plane * math.cos(theta1)
        y = x_plane * math.sin(theta1)
        z = z_plane

        return (x, y, z)

    def inverse_kinematics_2dof(self, x: float, z: float) -> Optional[Tuple[float, float]]:
        """
        简化的2DOF逆运动学
        针对 shoulder + elbow 两个关节

        Returns:
            (shoulder_angle, elbow_angle) 或 None（无解）
        """
        import math

        L1, L2 = self.link_lengths[:2]

        # 距离
        d = math.sqrt(x*x + z*z)

        # 检查可达性
        if d > L1 + L2 or d < abs(L1 - L2):
            return None

        # 使用余弦定理计算角度
        cos_elbow = (L1*L1 + L2*L2 - d*d) / (2*L1*L2)
        elbow_angle = math.degrees(math.acos(max(-1, min(1, cos_elbow))))

        # 肩关节角度
        alpha = math.atan2(z, x)
        beta = math.acos((d*d + L1*L1 - L2*L2) / (2*d*L1))
        shoulder_angle = math.degrees(alpha + beta)

        return (shoulder_angle, elbow_angle)
