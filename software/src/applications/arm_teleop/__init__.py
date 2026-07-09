"""
HomeBot 机械臂 WLAN 主从遥操作应用

通过读取本地主臂（SO101）关节角度，经 WLAN 实时同步到从端机械臂服务，
并支持动作录制与回放。
"""

from .app import ArmTeleopApp, build_master_arm_config, TeleopMode, create_default_app
from .master_reader import MasterArmReader
from .slave_client import SlaveArmClient
from .recorder import TrajectoryRecorder, TrajectoryPlayer
from .keyboard_input import KeyboardListener

__all__ = [
    "ArmTeleopApp",
    "build_master_arm_config",
    "create_default_app",
    "TeleopMode",
    "MasterArmReader",
    "SlaveArmClient",
    "TrajectoryRecorder",
    "TrajectoryPlayer",
    "KeyboardListener",
]
