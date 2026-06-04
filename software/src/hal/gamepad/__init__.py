"""
Xbox手柄驱动模块

提供对Xbox游戏手柄（Xbox 360/One/Series X|S）的完整支持：
- 按键状态读取
- 摇杆数据读取（含死区处理）
- 扳机键读取
- 震动控制反馈
- 事件回调机制

跨平台支持:
- Windows: 使用 XInput API（原生，性能最佳）
- macOS / Linux: 使用 pygame joystick（需安装 pygame）

使用方法:
    >>> from hal.gamepad import XboxController, Button
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

import sys

# 根据平台选择后端
if sys.platform == "win32":
    from .xinput_core import (
        XInputDriver as XboxController,
        ControllerState,
        StickState,
        ButtonFlags as Button,
        get_connected_controllers,
        wait_for_connection,
        XINPUT_MAX_CONTROLLERS,
        XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE,
        XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE,
        XINPUT_GAMEPAD_TRIGGER_THRESHOLD,
    )
else:
    from .pygame_backend import (
        PygameDriver as XboxController,
        ControllerState,
        StickState,
        ButtonFlags as Button,
        get_connected_controllers,
        wait_for_connection,
        XINPUT_MAX_CONTROLLERS,
        XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE,
        XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE,
        XINPUT_GAMEPAD_TRIGGER_THRESHOLD,
    )

__version__ = "1.1.0"
__all__ = [
    "XboxController",
    "ControllerState",
    "StickState",
    "Button",
    "get_connected_controllers",
    "wait_for_connection",
    "XINPUT_MAX_CONTROLLERS",
    "XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE",
    "XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE",
    "XINPUT_GAMEPAD_TRIGGER_THRESHOLD",
]
