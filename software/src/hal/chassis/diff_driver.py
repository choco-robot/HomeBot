"""
双轮差动底盘驱动 - 兼容 ESP32S3-baseboard 控制器

通过 Feetech ST3215 兼容协议与 ESP32S3 通信，支持：
- 速度控制 move(vx, vz)
- 闭环直线位移 goStraight(dist, speed)
- 闭环旋转位移 turn(angle, speed)
- 编码器里程计读取
- IMU 姿态读取

从 configs.config.ChassisConfig 读取配置
"""
import math
import struct
import time
from typing import Dict, List, Optional, Tuple

from configs import ChassisConfig


class DiffChassisDriver:
    """
    双轮差动底盘驱动器
    通过串口与 ESP32S3-baseboard 控制器通信
    """

    # 寄存器地址（与 SMS/STS 协议兼容）
    REG_MODE = 33
    REG_TORQUE_ENABLE = 40
    REG_ACC = 41
    REG_GOAL_POSITION = 42
    REG_GOAL_SPEED = 46
    REG_PRESENT_POSITION = 56
    REG_PRESENT_SPEED = 58
    REG_PRESENT_VOLTAGE = 62
    REG_PRESENT_TEMPERATURE = 63
    REG_MOVING = 66

    # 里程计寄存器地址
    REG_ODOM_START = 0x72       # 里程计数据起始地址 (X, Y, Theta, Vx, Vz)
    REG_ODOM_CMD = 0x7F         # 里程计控制命令 (0x01=清零)

    # IMU 数据起始地址
    REG_IMU_START = 0x50
    REG_IMU_CMD = 0x70

    # 默认设备ID
    DEFAULT_CHASSIS_ID = 0x24
    DEFAULT_MOTOR_LEFT_ID = 0x21
    DEFAULT_MOTOR_RIGHT_ID = 0x22
    DEFAULT_IMU_ID = 0x23

    def __init__(self, config: Optional[ChassisConfig] = None, bus=None):
        """
        初始化双轮差动底盘驱动

        Args:
            config: 底盘配置
            bus: 外部传入的舵机总线实例（FTServoBus，用于共享串口模式）
        """
        self.config = config or ChassisConfig()
        self._bus = bus  # FTServoBus 实例或 None
        self._shared_bus = False

        # 设备ID（从配置读取，使用默认值回退）
        self.chassis_id = getattr(self.config, 'diff_chassis_id', self.DEFAULT_CHASSIS_ID)
        self.motor_left_id = getattr(self.config, 'diff_motor_left_id', self.DEFAULT_MOTOR_LEFT_ID)
        self.motor_right_id = getattr(self.config, 'diff_motor_right_id', self.DEFAULT_MOTOR_RIGHT_ID)
        self.imu_id = getattr(self.config, 'diff_imu_id', self.DEFAULT_IMU_ID)

        # 串口和协议处理器（延迟初始化）
        self._port = None
        self._protocol = None

        # 当前速度状态
        self._current_vx = 0.0
        self._current_vy = 0.0
        self._current_omega = 0.0

        self._initialized = False

    def initialize(self) -> bool:
        """
        初始化双轮底盘
        - 如果使用共享总线，复用外部 FTServoBus 的串口和协议处理器
        - 否则自己打开串口
        - PING 底盘虚拟设备确认在线
        """
        print("[DiffChassis] Initializing...")

        # 检查是否可以使用共享总线
        if self._bus is not None and hasattr(self._bus, 'is_connected') and self._bus.is_connected():
            # 复用共享总线
            self._port = self._bus.port_handler
            self._protocol = self._bus.packet_handler
            self._shared_bus = True
            print("[DiffChassis] Using shared servo bus")
        else:
            # 自己创建串口连接
            try:
                from ..scservo_sdk.port_handler import PortHandler
                from ..scservo_sdk.protocol_packet_handler import protocol_packet_handler
            except ImportError as e:
                print(f"[DiffChassis] Failed to import servo SDK: {e}")
                return False

            port_name = getattr(self.config, 'serial_port', None) or getattr(self.config, 'port', 'COM4')
            baudrate = getattr(self.config, 'baudrate', 1000000)

            self._port = PortHandler(port_name)
            if not self._port.setBaudRate(baudrate):
                print(f"[DiffChassis] Failed to set baudrate {baudrate}")
                return False

            if not self._port.openPort():
                print(f"[DiffChassis] Failed to open port {port_name}")
                return False

            # protocol_end=0 表示小端序（与 ESP32S3 协议一致）
            self._protocol = protocol_packet_handler(self._port, protocol_end=0)
            self._shared_bus = False

        # PING 底盘虚拟设备确认在线
        if not self._ping_device(self.chassis_id):
            print(f"[DiffChassis] Chassis (ID=0x{self.chassis_id:02X}) not responding")
            if not self._shared_bus and self._port:
                self._port.closePort()
            return False

        print(f"[DiffChassis] Chassis online (ID=0x{self.chassis_id:02X})")

        # 停止底盘
        self.stop()

        self._initialized = True
        print("[DiffChassis] Initialized")
        return True

    def _ping_device(self, device_id: int) -> bool:
        """PING 设备，检查是否在线"""
        if self._protocol is None:
            return False
        model, result, error = self._protocol.ping(device_id)
        return result == 0  # COMM_SUCCESS

    def stop(self) -> None:
        """停止底盘运动"""
        self._current_vx = 0.0
        self._current_vy = 0.0
        self._current_omega = 0.0
        if self._initialized:
            self._chassis_move(0, 0)

    def _chassis_move(self, vx_mmps: int, vz_degps: int) -> bool:
        """
        发送底盘速度指令 move(vx, vz)
        
        Args:
            vx_mmps: 线速度 (mm/s)，前进为正
            vz_degps: 角速度 (deg/s)，左转/CCW 为正
        """
        if not self._initialized or self._protocol is None:
            return False
        data = list(struct.pack('<hh', int(vx_mmps), int(vz_degps)))
        result, error = self._protocol.writeTxRx(self.chassis_id, self.REG_GOAL_SPEED, 4, data)
        return result == 0

    def set_velocity(self, vx: float, vy: float, omega: float) -> bool:
        """
        设置底盘速度

        Args:
            vx: X方向速度（前进为正）(m/s)
            vy: Y方向速度（左移为正）(m/s) — 双轮差动不支持，会记录警告并忽略
            omega: Z方向角速度（逆时针为正）(rad/s)

        Returns:
            是否设置成功
        """
        if not self._initialized:
            print("[DiffChassis] Not initialized")
            return False

        # 双轮差动底盘不支持横向移动
        if abs(vy) > 0.01:
            print(f"[DiffChassis] Warning: vy={vy:.3f} ignored, diff drive cannot move laterally")

        # 限制速度范围
        vx = max(-self.config.max_linear_speed, min(self.config.max_linear_speed, vx))
        omega = max(-self.config.max_angular_speed, min(self.config.max_angular_speed, omega))

        # 保存当前速度
        self._current_vx = vx
        self._current_vy = 0.0  # vy 始终为 0
        self._current_omega = omega

        # 单位转换: m/s -> mm/s, rad/s -> deg/s
        vx_mmps = int(vx * 1000)
        vz_degps = int(omega * 180.0 / math.pi)

        return self._chassis_move(vx_mmps, vz_degps)

    def move_forward(self, distance: float, speed: float = 0.2) -> bool:
        """
        前进指定距离
        
        使用 ESP32S3 的闭环 goStraight 指令，比 time.sleep 开环控制更精确。

        Args:
            distance: 距离 (m)，正数前进，负数后退
            speed: 速度 (m/s)，始终取正值，方向由 distance 决定
        """
        if distance == 0:
            return True

        if not self._initialized:
            print("[DiffChassis] Not initialized")
            return False

        dist_mm = int(distance * 1000)
        speed_mmps = int(abs(speed) * 1000)

        data = list(struct.pack('<hh', dist_mm, speed_mmps))
        result, error = self._protocol.writeTxRx(self.chassis_id, self.REG_GOAL_POSITION, 4, data)

        if result != 0:
            print(f"[DiffChassis] goStraight command failed: result={result}")
            return False

        # 等待到位
        timeout = abs(distance) / abs(speed) + 5.0
        return self._wait_for_idle(timeout=timeout)

    def rotate(self, angle_deg: float, angular_speed: float = 90.0) -> bool:
        """
        旋转指定角度
        
        使用 ESP32S3 的闭环 turn 指令，比 time.sleep 开环控制更精确。

        Args:
            angle_deg: 角度（度），正数逆时针/左转
            angular_speed: 旋转角速度（度/s），始终取正值
        """
        if angle_deg == 0:
            return True

        if not self._initialized:
            print("[DiffChassis] Not initialized")
            return False

        speed_degps = int(abs(angular_speed))

        data = list(struct.pack('<hh', int(angle_deg), speed_degps))
        result, error = self._protocol.writeTxRx(self.chassis_id, self.REG_ACC, 4, data)

        if result != 0:
            print(f"[DiffChassis] turn command failed: result={result}")
            return False

        # 等待到位
        timeout = abs(angle_deg) / abs(angular_speed) + 5.0
        return self._wait_for_idle(timeout=timeout)

    def _wait_for_idle(self, timeout: float = 15.0, poll_interval: float = 0.05) -> bool:
        """
        轮询底盘运动状态，直到到位或超时

        Args:
            timeout: 最大等待时间 (s)
            poll_interval: 轮询间隔 (s)

        Returns:
            True=到位, False=超时
        """
        t_start = time.time()
        while time.time() - t_start < timeout:
            moving = self._read_moving()
            if moving is not None and moving == 0:
                return True
            time.sleep(poll_interval)
        print(f"[DiffChassis] Wait for idle timed out ({timeout:.1f}s)")
        return False

    def _read_moving(self) -> Optional[int]:
        """读取底盘 Moving 状态 (0=静止, 1=运动中)"""
        if not self._initialized or self._protocol is None:
            return None
        data, result, error = self._protocol.readTxRx(self.chassis_id, self.REG_MOVING, 1)
        if result == 0 and len(data) >= 1:
            return data[0]
        return None

    def read_encoder(self, motor_id: Optional[int] = None) -> Optional[int]:
        """
        读取编码器累积位置（低16位，有符号）

        Args:
            motor_id: 电机ID，默认读取左电机

        Returns:
            编码器位置值，失败返回 None
        """
        if not self._initialized or self._protocol is None:
            return None
        motor_id = motor_id or self.motor_left_id
        data, result, error = self._protocol.readTxRx(motor_id, self.REG_PRESENT_POSITION, 2)
        if result == 0 and len(data) >= 2:
            return struct.unpack('<h', bytes(data[:2]))[0]
        return None

    def read_motor_speed(self, motor_id: Optional[int] = None) -> Optional[int]:
        """
        读取电机当前转速（SMS 速度编码格式）

        Args:
            motor_id: 电机ID，默认读取左电机

        Returns:
            原始速度编码值，失败返回 None
        """
        if not self._initialized or self._protocol is None:
            return None
        motor_id = motor_id or self.motor_left_id
        data, result, error = self._protocol.readTxRx(motor_id, self.REG_PRESENT_SPEED, 2)
        if result == 0 and len(data) >= 2:
            return struct.unpack('<H', bytes(data[:2]))[0]
        return None

    def read_imu(self) -> Optional[Dict[str, float]]:
        """
        读取 IMU 姿态角

        Returns:
            {'yaw': deg, 'pitch': deg, 'roll': deg}，失败返回 None
        """
        if not self._initialized or self._protocol is None:
            return None
        # 从 0x50 读取 6 字节 (Yaw, Pitch, Roll)
        data, result, error = self._protocol.readTxRx(self.imu_id, self.REG_IMU_START, 6)
        if result == 0 and len(data) >= 6:
            yaw, pitch, roll = struct.unpack('<hhh', bytes(data[:6]))
            return {
                'yaw': yaw / 100.0,
                'pitch': pitch / 100.0,
                'roll': roll / 100.0,
            }
        return None

    def read_odometry(self) -> Optional[Dict[str, float]]:
        """
        从底盘读取编码器里程计位姿和实际速度
        
        ESP32S3-baseboard 底盘控制器内部维护编码器里程计，
        上位机可直接读取，无需自行积分。
        
        寄存器: 底盘 ID (0x24) 地址 0x72, 长度 10 字节
        数据格式: X, Y, Theta, Vx, Vz (各 2 字节, 小端有符号整数)
        
        Returns:
            {
                'x': 世界坐标系X位置 (m),
                'y': 世界坐标系Y位置 (m),
                'theta': 航向角 (rad),
                'vx': 当前线速度 (m/s),
                'vz': 当前角速度 (rad/s),
            }
            若底盘未返回有效数据则返回 None
        """
        if not self._initialized or self._protocol is None:
            return None
        
        data, result, error = self._protocol.readTxRx(
            self.chassis_id, self.REG_ODOM_START, 10
        )
        if result == 0 and len(data) >= 10:
            x_mm, y_mm, theta_raw, vx_mmps, vz_raw = struct.unpack('<hhhhh', bytes(data[:10]))
            return {
                'x': x_mm / 1000.0,                    # mm -> m
                'y': y_mm / 1000.0,                    # mm -> m
                'theta': math.radians(theta_raw / 100.0),  # 0.01° -> rad
                'vx': vx_mmps / 1000.0,                # mm/s -> m/s
                'vz': math.radians(vz_raw / 10.0),     # 0.1°/s -> rad/s
            }
        return None
    
    def reset_odometry(self) -> bool:
        """
        清零底盘编码器里程计
        
        向底盘 ID (0x24) 地址 0x7F 写入 0x01
        
        Returns:
            是否成功
        """
        if not self._initialized or self._protocol is None:
            return False
        result, error = self._protocol.writeTxRx(
            self.chassis_id, self.REG_ODOM_CMD, 1, [0x01]
        )
        return result == 0

    def read_imu_all(self) -> Optional[Dict[str, float]]:
        """
        读取全部 IMU 数据（角度 + 角速度 + 加速度）

        Returns:
            {
                'yaw': deg, 'pitch': deg, 'roll': deg,
                'yaw_rate': deg/s, 'pitch_rate': deg/s, 'roll_rate': deg/s,
                'acc_x': mg, 'acc_y': mg, 'acc_z': mg
            }
            失败返回 None
        """
        if not self._initialized or self._protocol is None:
            return None
        # 从 0x50 读取 24 字节（最大到 0x61）
        data, result, error = self._protocol.readTxRx(self.imu_id, self.REG_IMU_START, 18)
        if result == 0 and len(data) >= 18:
            values = struct.unpack('<hhhhhhhhh', bytes(data[:18]))
            return {
                'yaw': values[0] / 100.0,
                'pitch': values[1] / 100.0,
                'roll': values[2] / 100.0,
                'yaw_rate': values[3] / 100.0,
                'pitch_rate': values[4] / 100.0,
                'roll_rate': values[5] / 100.0,
                'acc_x': values[6],
                'acc_y': values[7],
                'acc_z': values[8],
            }
        return None

    def imu_clear_attitude(self) -> bool:
        """
        清零 IMU 姿态角（Yaw/Pitch/Roll 归零）

        Returns:
            是否成功
        """
        if not self._initialized or self._protocol is None:
            return False
        # 向任意电机 ID 的 0x70 地址写入 0x02
        result, error = self._protocol.writeTxRx(
            self.motor_left_id, self.REG_IMU_CMD, 1, [0x02]
        )
        return result == 0

    def is_moving(self) -> bool:
        """底盘是否在运动中"""
        moving = self._read_moving()
        return moving == 1 if moving is not None else False

    def read_voltage(self, motor_id: Optional[int] = None) -> Optional[float]:
        """
        读取电机电压 (V)
        兼容 BatteryDriver 接口

        Args:
            motor_id: 电机ID，默认读取左电机

        Returns:
            电压值 (V)，失败返回 None
        """
        if not self._initialized or self._protocol is None:
            return None
        motor_id = motor_id or self.motor_left_id
        data, result, error = self._protocol.readTxRx(motor_id, self.REG_PRESENT_VOLTAGE, 1)
        if result == 0 and len(data) >= 1:
            return data[0] / 10.0  # 原始值 ×10
        return None

    def read_temperature(self, motor_id: Optional[int] = None) -> Optional[int]:
        """
        读取电机温度 (°C)
        兼容 BatteryDriver 接口

        Args:
            motor_id: 电机ID，默认读取左电机

        Returns:
            温度值 (°C)，失败返回 None
        """
        if not self._initialized or self._protocol is None:
            return None
        motor_id = motor_id or self.motor_left_id
        data, result, error = self._protocol.readTxRx(motor_id, self.REG_PRESENT_TEMPERATURE, 1)
        if result == 0 and len(data) >= 1:
            return data[0]
        return None

    @property
    def bus(self):
        """
        兼容属性，返回 self 以支持 BatteryDriver
        使 ChassisService 中的 driver.bus 调用正常工作
        """
        return self

    def get_current_velocity(self) -> Tuple[float, float, float]:
        """获取当前命令速度 (vx, vy, omega)"""
        return (self._current_vx, self._current_vy, self._current_omega)

    def close(self) -> None:
        """关闭底盘驱动"""
        if not self._initialized:
            return
        self.stop()
        time.sleep(0.1)
        # 仅独立模式时才关闭串口（共享总线由外部管理）
        if not self._shared_bus and self._port:
            self._port.closePort()
        self._protocol = None
        self._port = None
        self._initialized = False
        print("[DiffChassis] Closed")
