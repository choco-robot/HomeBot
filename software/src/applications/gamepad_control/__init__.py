"""
HomeBot 游戏手柄控制应用

提供同时控制底盘和机械臂的功能：
- 底盘：左摇杆 + 扳机键
- 机械臂：右摇杆 + 十字键 + ABXY + 肩键

使用方法:
    >>> from applications.gamepad_control import GamepadControlApp
    >>> app = GamepadControlApp()
    >>> app.run()

命令行启动:
    cd software/src
    python -m applications.gamepad_control
"""

from .app import GamepadControlApp

__all__ = ["GamepadControlApp"]
