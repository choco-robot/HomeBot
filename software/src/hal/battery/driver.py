"""
电池驱动 - 通过飞特舵机总线读取电源电压

由于舵机与主控共用电源总线，可以通过读取任意舵机的电压值来获取电池电压。
通常读取第一个可用的舵机ID即可。
"""
from typing import Optional, List
from dataclasses import dataclass
from enum import Enum


class BatteryStatus(Enum):
    """电池状态枚举"""
    UNKNOWN = "unknown"           # 未知状态
    NORMAL = "normal"             # 正常
    LOW = "low"                   # 低电量
    CRITICAL = "critical"         # 电量严重不足
    CHARGING = "charging"         # 充电中


@dataclass
class BatteryState:
    """电池状态数据结构"""
    voltage: float = 0.0                  # 电压值 (V)
    percentage: float = 0.0               # 电量百分比 (0-100)
    status: BatteryStatus = BatteryStatus.UNKNOWN  # 电池状态
    temperature: Optional[int] = None     # 舵机温度 (°C)
    servo_id: int = 0                     # 读取电压的舵机ID
    is_valid: bool = False                # 数据是否有效


class BatteryDriver:
    """
    电池驱动
    通过舵机总线读取电源电压
    """
    
    # 默认电压阈值配置 (适用于3S锂电池: 11.1V标称, 12.6V满电, 9.0V截止)
    DEFAULT_FULL_VOLTAGE = 12.6    # 满电电压 (V)
    DEFAULT_LOW_VOLTAGE = 10.5     # 低电量阈值 (V)
    DEFAULT_CRITICAL_VOLTAGE = 9.5 # 严重低电量阈值 (V)
    DEFAULT_MIN_VOLTAGE = 9.0      # 最低工作电压 (V)
    
    def __init__(self, 
                 servo_bus=None,
                 servo_ids: Optional[List[int]] = None,
                 full_voltage: float = DEFAULT_FULL_VOLTAGE,
                 low_voltage: float = DEFAULT_LOW_VOLTAGE,
                 critical_voltage: float = DEFAULT_CRITICAL_VOLTAGE,
                 min_voltage: float = DEFAULT_MIN_VOLTAGE):
        """
        初始化电池驱动
        
        Args:
            servo_bus: 舵机总线实例 (FTServoBus)
            servo_ids: 用于读取电压的舵机ID列表，默认使用ID 1
            full_voltage: 满电电压 (V)
            low_voltage: 低电量阈值 (V)
            critical_voltage: 严重低电量阈值 (V)
            min_voltage: 最低工作电压 (V)
        """
        self._bus = servo_bus
        self._servo_ids = servo_ids or [1]  # 默认使用ID 1
        self._full_voltage = full_voltage
        self._low_voltage = low_voltage
        self._critical_voltage = critical_voltage
        self._min_voltage = min_voltage
        
        self._last_state = BatteryState()
    
    def set_servo_bus(self, servo_bus):
        """设置舵机总线实例"""
        self._bus = servo_bus
    
    def _voltage_to_percentage(self, voltage: float) -> float:
        """
        将电压转换为电量百分比
        使用简单的线性映射
        """
        if voltage >= self._full_voltage:
            return 100.0
        elif voltage <= self._min_voltage:
            return 0.0
        else:
            # 线性映射
            percentage = ((voltage - self._min_voltage) / 
                         (self._full_voltage - self._min_voltage)) * 100.0
            return max(0.0, min(100.0, percentage))
    
    def _determine_status(self, voltage: float) -> BatteryStatus:
        """根据电压确定电池状态"""
        if voltage <= 0:
            return BatteryStatus.UNKNOWN
        elif voltage < self._critical_voltage:
            return BatteryStatus.CRITICAL
        elif voltage < self._low_voltage:
            return BatteryStatus.LOW
        else:
            return BatteryStatus.NORMAL
    
    def read_state(self) -> BatteryState:
        """
        读取电池状态
        
        Returns:
            BatteryState 包含电压、电量百分比和状态
        """
        if self._bus is None:
            return BatteryState(is_valid=False)
        
        # 尝试从配置的舵机ID列表中读取电压
        for servo_id in self._servo_ids:
            try:
                voltage = self._bus.read_voltage(servo_id)
                if voltage is not None:
                    # 同时读取温度（可选）
                    temperature = self._bus.read_temperature(servo_id)
                    
                    percentage = self._voltage_to_percentage(voltage)
                    status = self._determine_status(voltage)
                    
                    state = BatteryState(
                        voltage=voltage,
                        percentage=percentage,
                        status=status,
                        temperature=temperature,
                        servo_id=servo_id,
                        is_valid=True
                    )
                    self._last_state = state
                    return state
                    
            except Exception as e:
                # 读取失败，尝试下一个舵机ID
                continue
        
        # 所有舵机ID都读取失败，返回无效状态
        return BatteryState(is_valid=False)
    
    def get_last_state(self) -> BatteryState:
        """获取上一次成功读取的电池状态"""
        return self._last_state
    
    def is_low_battery(self) -> bool:
        """检查是否低电量"""
        return (self._last_state.is_valid and 
                self._last_state.status in (BatteryStatus.LOW, BatteryStatus.CRITICAL))
    
    def is_critical_battery(self) -> bool:
        """检查是否电量严重不足"""
        return (self._last_state.is_valid and 
                self._last_state.status == BatteryStatus.CRITICAL)
