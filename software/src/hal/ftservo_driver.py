"""
飞特串行总线舵机底层驱动封装
基于 ftservo-python-sdk (scservo_sdk)
支持 ST3215 等 STS 系列舵机
"""
import sys
import time
from typing import Dict, List, Optional, Tuple, Union
from dataclasses import dataclass
from enum import IntEnum

# 字节操作辅助函数（模块级别定义，确保始终可用）
def _SMS_STS_LOBYTE(value):
    return value & 0xFF

def _SMS_STS_HIBYTE(value):
    return (value >> 8) & 0xFF

try:
    from .scservo_sdk import *
    # 如果 SDK 中没有定义这些函数，使用我们的实现
    if 'SMS_STS_LOBYTE' not in dir():
        SMS_STS_LOBYTE = _SMS_STS_LOBYTE
    if 'SMS_STS_HIBYTE' not in dir():
        SMS_STS_HIBYTE = _SMS_STS_HIBYTE
except ImportError:
    # 模拟模式 - 用于开发和测试
    print("[FTServo] Warning: scservo_sdk not found, running in simulation mode")
    COMM_SUCCESS = 0
    BROADCAST_ID = 0xFE
    SMS_STS_TORQUE_ENABLE = 40
    
    # 使用我们的辅助函数
    SMS_STS_LOBYTE = _SMS_STS_LOBYTE
    SMS_STS_HIBYTE = _SMS_STS_HIBYTE

    class sms_sts:
        def __init__(self, port_handler):
            self.port = port_handler
            self._sim_positions = {}

        def WritePosEx(self, scs_id, position, speed, acc):
            self._sim_positions[scs_id] = position
            return COMM_SUCCESS, 0

        def ReadPos(self, scs_id):
            return self._sim_positions.get(scs_id, 2048), COMM_SUCCESS, 0

        def ReadSpeed(self, scs_id):
            return 0, COMM_SUCCESS, 0

        def ReadVoltage(self, scs_id):
            return 120, COMM_SUCCESS, 0  # 模拟12.0V

        def ReadTemperature(self, scs_id):
            return 25, COMM_SUCCESS, 0  # 模拟25°C

        def WheelMode(self, scs_id):
            return COMM_SUCCESS, 0

        def WriteSpec(self, scs_id, speed, acc):
            return COMM_SUCCESS, 0

        def RegWritePosEx(self, scs_id, position, speed, acc):
            return COMM_SUCCESS, 0

        def RegAction(self):
            return COMM_SUCCESS, 0

        def torque_enable(self, scs_id=-1):
            return COMM_SUCCESS, 0

        def torque_disable(self, scs_id=-1):
            return COMM_SUCCESS, 0

        def write1ByteTxRx(self, scs_id, address, value):
            return COMM_SUCCESS, 0

        def SyncWritePosEx(self, scs_id, position, speed, acc):
            return COMM_SUCCESS, 0

        def groupSyncWrite(self, scs_id, data):
            return COMM_SUCCESS, 0

        def disconnect(self):
            pass

    class PortHandler:
        def __init__(self, port_name):
            self.port_name = port_name
            self._open = False

        def openPort(self):
            self._open = True
            return True

        def closePort(self):
            self._open = False

        def setBaudRate(self, baudrate):
            return True


class ServoMode(IntEnum):
    """舵机工作模式"""
    POSITION = 0  # 位置模式（默认）
    SPEED = 1     # 速度模式（轮式）
    TORQUE = 2    # 力矩模式（HLS系列支持）
    PWM = 3       # PWM模式


@dataclass
class ServoConfig:
    """舵机配置"""
    id: int
    name: str = ""
    mode: ServoMode = ServoMode.POSITION
    min_angle: int = 0        # 最小角度限制
    max_angle: int = 4095     # 最大角度限制
    default_speed: int = 1000 # 默认速度
    default_acc: int = 50     # 默认加速度


@dataclass
class ServoState:
    """舵机状态"""
    id: int
    position: int = 2048      # 当前位置 (0-4095)
    speed: int = 0            # 当前速度
    load: int = 0             # 当前负载
    voltage: float = 0.0      # 当前电压
    temperature: int = 0      # 当前温度
    moving: bool = False      # 是否正在运动


class FTServoBus:
    """
    飞特舵机总线管理器
    管理串口连接和舵机通信
    """

    DEFAULT_BAUDRATE = 1000000  # 默认波特率 1Mbps

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = DEFAULT_BAUDRATE):
        """
        初始化舵机总线

        Args:
            port: 串口设备路径，如 Linux '/dev/ttyUSB0'，Windows 'COM1'
            baudrate: 波特率，默认 1Mbps
        """
        self.port_name = port
        self.baudrate = baudrate
        self.port_handler: Optional[PortHandler] = None
        self.packet_handler: Optional[sms_sts] = None
        self._connected = False
        self._simulation = False

    def connect(self) -> bool:
        """连接舵机总线"""
        try:
            self.port_handler = PortHandler(self.port_name)

            if not self.port_handler.openPort():
                print(f"[FTServo] Failed to open port: {self.port_name}")
                return False

            if not self.port_handler.setBaudRate(self.baudrate):
                print(f"[FTServo] Failed to set baudrate: {self.baudrate}")
                return False

            self.packet_handler = sms_sts(self.port_handler)
            self._connected = True

            print(f"[FTServo] Connected to {self.port_name} @ {self.baudrate}bps")
            return True

        except Exception as e:
            print(f"[FTServo] Connection error: {e}")
            print(f"[FTServo] 硬件连接失败，请检查：")
            print(f"  1. 串口 {self.port_name} 是否正确")
            print(f"  2. 舵机电源是否开启")
            print(f"  3. 数据线是否连接正常")
            self._connected = False
            return False

    def disconnect(self) -> None:
        """断开连接"""
        if not self._connected:
            return  # 已经断开，避免重复操作
        if self.packet_handler and hasattr(self.packet_handler, 'disconnect'):
            self.packet_handler.disconnect()
        if self.port_handler:
            self.port_handler.closePort()
        self._connected = False
        print("[FTServo] Disconnected")

    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._connected

    def ping(self, servo_id: int) -> Tuple[bool, int]:
        """
        检测舵机是否存在

        Returns:
            (是否成功, 舵机型号)
        """
        if self._simulation:
            return True, 0

        model_number, comm_result, error = self.packet_handler.ping(servo_id)
        return comm_result == COMM_SUCCESS, model_number

    def write_position(self, servo_id: int, position: int,
                       speed: int = 0, acc: int = 0) -> bool:
        """
        设置舵机目标位置

        Args:
            servo_id: 舵机ID
            position: 目标位置 (0-4095)
            speed: 运动速度，0表示最大速度
            acc: 加速度
        """
        if not self._connected:
            return False

        # 限制位置范围
        position = max(0, min(4095, int(position)))

        if self._simulation:
            return True

        comm_result, error = self.packet_handler.WritePosEx(
            servo_id, position, speed, acc
        )

        if comm_result != COMM_SUCCESS:
            print(f"[FTServo] Write position failed for ID {servo_id}: {comm_result}")
            return False
        return True

    def read_position(self, servo_id: int) -> Optional[int]:
        """读取舵机当前位置"""
        if not self._connected:
            return None

        if self._simulation:
            return 2048

        position, comm_result, error = self.packet_handler.ReadPos(servo_id)

        if comm_result != COMM_SUCCESS:
            return None
        return position

    def sync_read_positions(self, servo_ids: List[int]) -> Dict[int, Optional[int]]:
        """同步读取多个舵机位置"""
        if not self._connected:
            return {sid: None for sid in servo_ids}

        positions = {}
        positions = self.packet_handler.SyncReadPos(servo_ids)
        return positions

    def set_wheel_mode(self, servo_id: int) -> bool:
        """设置舵机为轮式模式（连续旋转）"""
        if not self._connected:
            return False

        if self._simulation:
            return True

        comm_result, error = self.packet_handler.WheelMode(servo_id)

        if comm_result != COMM_SUCCESS:
            print(f"[FTServo] Set wheel mode failed for ID {servo_id}")
            return False
        return True

    def write_speed(self, servo_id: int, speed: int, acc: int = 50) -> bool:
        """
        设置舵机速度（轮式模式下）

        Args:
            servo_id: 舵机ID
            speed: 速度值，范围 -32767 ~ 32767
            acc: 加速度
        """
        if not self._connected:
            return False

        # 限制速度范围
        speed = max(-32767, min(32767, int(speed)))

        if self._simulation:
            return True

        comm_result, error = self.packet_handler.WriteSpec(servo_id, speed, acc)

        if comm_result != COMM_SUCCESS:
            print(f"[FTServo] Write speed failed for ID {servo_id}")
            return False
        return True

    def torque_enable(self, servo_id: int = -1) -> bool:
        """使能扭矩，servo_id=-1表示广播到所有舵机"""
        if not self._connected or self._simulation:
            return True

        if servo_id == -1:
            servo_id = BROADCAST_ID

        # 写入扭矩使能寄存器 (地址40)，值1表示使能
        comm_result, error = self.packet_handler.write1ByteTxRx(
            servo_id, SMS_STS_TORQUE_ENABLE, 1
        )
        return comm_result == COMM_SUCCESS

    def torque_disable(self, servo_id: int = -1) -> bool:
        """失能扭矩"""
        if not self._connected or self._simulation:
            return True

        if servo_id == -1:
            servo_id = BROADCAST_ID

        # 写入扭矩使能寄存器 (地址40)，值0表示失能
        comm_result, error = self.packet_handler.write1ByteTxRx(
            servo_id, SMS_STS_TORQUE_ENABLE, 0
        )
        return comm_result == COMM_SUCCESS

    def sync_write_positions(self, positions: Dict[int, Tuple[int, int, int]]) -> bool:
        """
        同步写入多个舵机位置

        Args:
            positions: {servo_id: (position, speed, acc), ...}
        """
        if not self._connected:
            return False

        if self._simulation:
            return True

        # 使用 GroupSyncWrite
        self.packet_handler.SyncWritePosEx(positions)

        return True

    def get_state(self, servo_id: int) -> Optional[ServoState]:
        """获取舵机状态"""
        if not self._connected or self._simulation:
            return ServoState(id=servo_id, position=2048)

        position, comm_result1, error1 = self.packet_handler.ReadPos(servo_id)
        speed, comm_result2, error2 = self.packet_handler.ReadSpeed(servo_id)

        if comm_result1 != COMM_SUCCESS:
            return None

        return ServoState(
            id=servo_id,
            position=position,
            speed=speed
        )

    def read_voltage(self, servo_id: int) -> Optional[float]:
        """
        读取舵机当前电压
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            电压值(伏特)，读取失败返回None
        """
        if not self._connected:
            return None
        
        if self._simulation:
            return 12.0  # 模拟模式返回默认电压
        
        voltage, comm_result, error = self.packet_handler.ReadVoltage(servo_id)
        
        if comm_result != COMM_SUCCESS:
            return None
        
        # 电压值是原始值，需要除以10得到实际电压(伏特)
        return voltage / 10.0 if voltage is not None else None

    def read_temperature(self, servo_id: int) -> Optional[int]:
        """
        读取舵机当前温度
        
        Args:
            servo_id: 舵机ID
            
        Returns:
            温度值(摄氏度)，读取失败返回None
        """
        if not self._connected:
            return None
        
        if self._simulation:
            return 25  # 模拟模式返回默认温度
        
        temp, comm_result, error = self.packet_handler.ReadTemperature(servo_id)
        
        if comm_result != COMM_SUCCESS:
            return None
        
        return temp


# 兼容旧接口的别名
FeetechMotorsBus = FTServoBus
