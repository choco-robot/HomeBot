"""
Xbox手柄驱动模块

提供对Xbox游戏手柄（Xbox 360/One/Series X|S）的完整支持：
- 按键状态读取
- 摇杆数据读取（含死区处理）
- 扳机键读取
- 震动控制
- 事件回调机制

使用方法:
    >>> from xbox_driver import XboxController, Button
    >>> 
    >>> controller = XboxController(0)  # 使用第一个手柄
    >>> state = controller.get_state()
    >>> 
    >>> if state.is_pressed(Button.A):
    ...     print("A键被按下")
    >>> 
    >>> x, y = state.get_left_stick()
    >>> print(f"左摇杆: ({x:.2f}, {y:.2f})")
"""

from .xinput_core import (
    # 核心类
    XInputDriver,
    XboxController,
    ControllerState,
    StickState,
    
    # 按键枚举
    ButtonFlags as Button,
    
    # 便捷函数
    get_connected_controllers,
    wait_for_connection,
    
    # 常量
    XINPUT_MAX_CONTROLLERS,
    XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE,
    XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE,
    XINPUT_GAMEPAD_TRIGGER_THRESHOLD,
)

__version__ = "1.0.0"
__all__ = [
    # 核心类
    "XInputDriver",
    "XboxController", 
    "ControllerState",
    "StickState",
    
    # 按键枚举
    "Button",
    
    # 便捷函数
    "get_connected_controllers",
    "wait_for_connection",
    
    # 常量
    "XINPUT_MAX_CONTROLLERS",
    "XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE",
    "XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE",
    "XINPUT_GAMEPAD_TRIGGER_THRESHOLD",
]
