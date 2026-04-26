"""
Xbox手柄跨平台驱动后端（macOS / Linux）
基于 pygame.joystick，API 与 xinput_core.py 保持一致
"""

import sys
import time
import threading
from enum import IntFlag
from dataclasses import dataclass
from typing import Optional, Callable, List

import pygame


# ==================== 常量定义 ====================

XINPUT_MAX_CONTROLLERS = 4
XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE = 7849
XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE = 8689
XINPUT_GAMEPAD_TRIGGER_THRESHOLD = 30


# ==================== 枚举定义 ====================

class ButtonFlags(IntFlag):
    """手柄按键标志位"""
    DPAD_UP = 0x0001
    DPAD_DOWN = 0x0002
    DPAD_LEFT = 0x0004
    DPAD_RIGHT = 0x0008
    START = 0x0010
    BACK = 0x0020
    LEFT_THUMB = 0x0040
    RIGHT_THUMB = 0x0080
    LEFT_SHOULDER = 0x0100
    RIGHT_SHOULDER = 0x0200
    A = 0x1000
    B = 0x2000
    X = 0x4000
    Y = 0x8000
    GUIDE = 0x0400


# ==================== 数据结构 ====================

@dataclass
class StickState:
    x: float
    y: float
    magnitude: float
    raw_x: int
    raw_y: int


@dataclass
class ControllerState:
    connected: bool
    packet_number: int
    buttons: set
    left_stick: StickState
    right_stick: StickState
    left_trigger: float
    right_trigger: float
    raw_buttons: int

    def is_pressed(self, button: ButtonFlags) -> bool:
        return button in self.buttons

    def get_left_stick(self) -> tuple[float, float]:
        return (self.left_stick.x, self.left_stick.y)

    def get_right_stick(self) -> tuple[float, float]:
        return (self.right_stick.x, self.right_stick.y)


# ==================== 按键映射 ====================

# pygame button index -> ButtonFlags (Xbox 360/One on macOS)
PYGAME_BUTTON_MAP = {
    0: ButtonFlags.A,
    1: ButtonFlags.B,
    2: ButtonFlags.X,
    3: ButtonFlags.Y,
    4: ButtonFlags.LEFT_SHOULDER,
    5: ButtonFlags.RIGHT_SHOULDER,
    6: ButtonFlags.BACK,
    7: ButtonFlags.START,
    8: ButtonFlags.GUIDE,
    9: ButtonFlags.LEFT_THUMB,
    10: ButtonFlags.RIGHT_THUMB,
}

#  hats[0] -> D-Pad
HAT_MAP = {
    (0, 1): ButtonFlags.DPAD_UP,
    (0, -1): ButtonFlags.DPAD_DOWN,
    (-1, 0): ButtonFlags.DPAD_LEFT,
    (1, 0): ButtonFlags.DPAD_RIGHT,
}


# ==================== 核心驱动类 ====================

class PygameDriver:
    """
    Xbox手柄跨平台驱动类（macOS/Linux）
    接口与 Windows XInputDriver 保持一致
    """

    def __init__(self, controller_index: int = 0):
        if not 0 <= controller_index < XINPUT_MAX_CONTROLLERS:
            raise ValueError(f"Controller index must be 0-{XINPUT_MAX_CONTROLLERS - 1}")

        self.controller_index = controller_index
        self._joystick: Optional[pygame.joystick.JoystickType] = None
        self._last_packet_number = 0

        self.left_deadzone = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE
        self.right_deadzone = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE
        self.trigger_threshold = XINPUT_GAMEPAD_TRIGGER_THRESHOLD

        self._button_press_callbacks: dict[ButtonFlags, List[Callable]] = {}
        self._button_release_callbacks: dict[ButtonFlags, List[Callable]] = {}
        self._state_change_callback: Optional[Callable] = None

        self._polling = False
        self._poll_thread: Optional[threading.Thread] = None
        self._poll_interval = 0.016
        self._previous_buttons: set = set()

        self._init_pygame()
        self._open_joystick()

    def _init_pygame(self):
        if not pygame.get_init():
            pygame.init()
        if not pygame.joystick.get_init():
            pygame.joystick.init()

    def _open_joystick(self):
        count = pygame.joystick.get_count()
        if self.controller_index >= count:
            return  # 留到 is_connected / get_state 时再处理
        self._joystick = pygame.joystick.Joystick(self.controller_index)
        self._joystick.init()

    def _apply_deadzone(self, x: float, y: float, deadzone: int) -> tuple[float, float, float]:
        raw_x = int(x * 32767)
        raw_y = int(-y * 32767)  # pygame Y轴向下为正，需翻转
        magnitude = (raw_x ** 2 + raw_y ** 2) ** 0.5

        if magnitude <= deadzone:
            return 0.0, 0.0, 0.0

        max_magnitude = 32767.0
        normalized_magnitude = min((magnitude - deadzone) / (max_magnitude - deadzone), 1.0)
        scale = normalized_magnitude / (magnitude / max_magnitude)
        normalized_x = (raw_x / max_magnitude) * scale
        normalized_y = (raw_y / max_magnitude) * scale

        return normalized_x, normalized_y, normalized_magnitude

    def _normalize_trigger(self, value: float) -> float:
        """pygame trigger axis 通常是 -1..1，映射到 0..1"""
        # 某些驱动下 trigger 是 0..1，这里做兼容处理
        if value < 0:
            value = (value + 1) / 2  # -1..1 -> 0..1
        normalized = int(value * 255)
        if normalized < self.trigger_threshold:
            return 0.0
        return min(normalized / 255.0, 1.0)

    def get_state(self) -> ControllerState:
        if self._joystick is None or not self._joystick.get_init():
            return ControllerState(
                connected=False,
                packet_number=0,
                buttons=set(),
                left_stick=StickState(0, 0, 0, 0, 0),
                right_stick=StickState(0, 0, 0, 0, 0),
                left_trigger=0.0,
                right_trigger=0.0,
                raw_buttons=0,
            )

        # 处理 pygame 事件（刷新设备状态）
        pygame.event.pump()

        buttons = set()
        raw_buttons = 0

        # 普通按键
        for i in range(self._joystick.get_numbuttons()):
            if self._joystick.get_button(i):
                flag = PYGAME_BUTTON_MAP.get(i)
                if flag:
                    buttons.add(flag)
                    raw_buttons |= flag

        # D-Pad (hat)
        for i in range(self._joystick.get_numhats()):
            hat = self._joystick.get_hat(i)
            flag = HAT_MAP.get(hat)
            if flag:
                buttons.add(flag)
                raw_buttons |= flag

        # 摇杆 (尝试两种常见映射)
        num_axes = self._joystick.get_numaxes()
        if num_axes >= 2:
            lx, ly = self._joystick.get_axis(0), self._joystick.get_axis(1)
        else:
            lx = ly = 0.0

        if num_axes >= 4:
            rx, ry = self._joystick.get_axis(2), self._joystick.get_axis(3)
            # LT/RT 可能在 axis 4,5
            if num_axes >= 6:
                lt = self._joystick.get_axis(4)
                rt = self._joystick.get_axis(5)
            elif num_axes >= 5:
                lt = self._joystick.get_axis(4)
                rt = 0.0
            else:
                lt = rt = 0.0
        else:
            rx = ry = 0.0
            lt = rt = 0.0

        # 另一种常见映射：axis 2 是左/右摇杆的混合，axis 4,5 是右摇杆
        # 如果检测到 axis 3 是 trigger（值范围接近 0..1），做自适应
        if num_axes == 5 and abs(ry) < 0.1:
            # 可能是 LX,LY,RX,RY,LT=RT 组合轴 或 LT,RT 分开在 axis 4,5
            pass

        left_x, left_y, left_m = self._apply_deadzone(lx, ly, self.left_deadzone)
        right_x, right_y, right_m = self._apply_deadzone(rx, ry, self.right_deadzone)

        self._last_packet_number += 1

        return ControllerState(
            connected=True,
            packet_number=self._last_packet_number,
            buttons=buttons,
            left_stick=StickState(
                x=left_x, y=left_y, magnitude=left_m,
                raw_x=int(lx * 32767), raw_y=int(-ly * 32767),
            ),
            right_stick=StickState(
                x=right_x, y=right_y, magnitude=right_m,
                raw_x=int(rx * 32767), raw_y=int(-ry * 32767),
            ),
            left_trigger=self._normalize_trigger(lt),
            right_trigger=self._normalize_trigger(rt),
            raw_buttons=raw_buttons,
        )

    def is_connected(self) -> bool:
        if self._joystick is None:
            self._init_pygame()
            self._open_joystick()
        return self._joystick is not None and self._joystick.get_init()

    def set_vibration(self, left_motor: float, right_motor: float):
        """震动功能在 pygame joystick 中并非所有平台都支持，尝试使用 rumble。"""
        if self._joystick is None:
            return
        duration = 300  # ms
        try:
            self._joystick.rumble(
                int(max(0.0, min(1.0, left_motor)) * 1.0),
                int(max(0.0, min(1.0, right_motor)) * 1.0),
                duration,
            )
        except AttributeError:
            pass  # 当前平台/驱动不支持 rumble

    def stop_vibration(self):
        if self._joystick is None:
            return
        try:
            self._joystick.stop_rumble()
        except AttributeError:
            pass

    def get_capabilities(self) -> Optional[dict]:
        return {
            "type": 1,
            "subtype": 1,
            "subtype_name": "Gamepad",
            "flags": 0,
            "has_vibration": hasattr(self._joystick, "rumble") if self._joystick else False,
        }

    # ==================== 事件回调机制 ====================

    def on_button_press(self, button: ButtonFlags, callback: Callable):
        if button not in self._button_press_callbacks:
            self._button_press_callbacks[button] = []
        self._button_press_callbacks[button].append(callback)

    def on_button_release(self, button: ButtonFlags, callback: Callable):
        if button not in self._button_release_callbacks:
            self._button_release_callbacks[button] = []
        self._button_release_callbacks[button].append(callback)

    def on_state_change(self, callback: Callable[[ControllerState], None]):
        self._state_change_callback = callback

    def _trigger_callbacks(self, current_buttons: set):
        pressed = current_buttons - self._previous_buttons
        for btn in pressed:
            if btn in self._button_press_callbacks:
                for cb in self._button_press_callbacks[btn]:
                    cb(btn)

        released = self._previous_buttons - current_buttons
        for btn in released:
            if btn in self._button_release_callbacks:
                for cb in self._button_release_callbacks[btn]:
                    cb(btn)

        self._previous_buttons = current_buttons

    def start_polling(self, interval: Optional[float] = None):
        if self._polling:
            return
        if interval is not None:
            self._poll_interval = interval
        self._polling = True
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def stop_polling(self):
        self._polling = False
        if self._poll_thread:
            self._poll_thread.join(timeout=1.0)
            self._poll_thread = None

    def _poll_loop(self):
        while self._polling:
            state = self.get_state()
            if state.connected:
                self._trigger_callbacks(state.buttons)
                if self._state_change_callback:
                    self._state_change_callback(state)
            time.sleep(self._poll_interval)


# ==================== 便捷函数 ====================

def get_connected_controllers() -> List[int]:
    if not pygame.get_init():
        pygame.init()
    if not pygame.joystick.get_init():
        pygame.joystick.init()
    return list(range(pygame.joystick.get_count()))


def wait_for_connection(controller_index: int = 0, timeout: Optional[float] = None) -> bool:
    start = time.time()
    while True:
        if not pygame.get_init():
            pygame.init()
        if not pygame.joystick.get_init():
            pygame.joystick.init()
        if controller_index < pygame.joystick.get_count():
            return True
        if timeout and (time.time() - start) > timeout:
            return False
        time.sleep(0.1)


# 兼容性别名
XboxController = PygameDriver
