"""
舵机总线管理器 - 单例模式
管理共享的串口连接，供底盘和机械臂共同使用
"""
from typing import Optional
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from hal.ftservo_driver import FTServoBus


class ServoBusManager:
    """舵机总线单例管理器"""
    
    _instance: Optional['ServoBusManager'] = None
    _bus: Optional[FTServoBus] = None
    _initialized: bool = False
    _port: str = ""
    _baudrate: int = 0
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def initialize(self, port: str, baudrate: int) -> bool:
        """初始化串口总线（仅首次调用有效）"""
        if self._initialized and self._bus is not None:
            return True
            
        self._port = port
        self._baudrate = baudrate
        self._bus = FTServoBus(port, baudrate)
        
        if self._bus.connect():
            self._initialized = True
            print(f"[ServoBusManager] 串口总线已初始化: {port} @ {baudrate}bps")
            return True
        
        print(f"[ServoBusManager] 串口连接失败: {port}")
        return False
    
    def get_bus(self) -> Optional[FTServoBus]:
        """获取共享的总线实例"""
        return self._bus
    
    def is_initialized(self) -> bool:
        """检查是否已初始化"""
        return self._initialized and self._bus is not None and self._bus.is_connected()
    
    def get_port_info(self) -> tuple:
        """获取串口信息"""
        return self._port, self._baudrate
    
    def close(self):
        """关闭总线"""
        if self._bus:
            self._bus.disconnect()
            self._initialized = False
            self._bus = None
            ServoBusManager._instance = None
            print("[ServoBusManager] 串口总线已关闭")


def get_servo_bus() -> Optional[FTServoBus]:
    """获取舵机总线实例的快捷方式"""
    return ServoBusManager().get_bus()


def is_bus_ready() -> bool:
    """检查总线是否已准备好"""
    return ServoBusManager().is_initialized()
