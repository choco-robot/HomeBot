"""
Xbox 手柄 Linux 原生 joystick 后端

基于 /dev/input/js* 接口直接读取手柄事件，无需 pygame 等额外 Python 依赖。
API 与 xinput_core.py / pygame_backend.py 保持一致，确保应用层无感切换。

默认映射针对 Linux xpad 驱动下的 Xbox 360 / One / Series X|S 手柄：
- axes:  0=LX, 1=LY, 2=LT, 3=RX, 4=RY, 5=RT, 6=hatX, 7=hatY
- buttons: 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB, 6=BACK, 7=START, 8=GUIDE, 9=LS, 10=RS

权限说明：
- 读取 /dev/input/js* 需要当前用户属于 input 组，或设备文件有读权限。
- 可通过 `sudo usermod -aG input $USER` 添加权限，注销重新登录后生效。
"""

import os
import struct
import time
import threading
from enum import IntFlag
from dataclasses import dataclass
from typing import Optional, Callable, List, Dict, Tuple


# ==================== 常量定义 ====================

XINPUT_MAX_CONTROLLERS = 4
XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE = 7849
XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE = 8689
XINPUT_GAMEPAD_TRIGGER_THRESHOLD = 30


# ==================== 枚举定义 ====================

class ButtonFlags(IntFlag):
    """手柄按键标志位，与 Windows XInput 后端保持一致"""
    DPAD_UP = 0x0001
    DPAD_DOWN = 0x0002
    DPAD_LEFT = 0x0004
    DPAD_RIGHT = 0x0008
    START = 0x0010
    BACK = 0x0020
    LEFT_THUMB = 0x0040      # 左摇杆按下
    RIGHT_THUMB = 0x0080     # 右摇杆按下
    LEFT_SHOULDER = 0x0100   # LB
    RIGHT_SHOULDER = 0x0200  # RB
    A = 0x1000
    B = 0x2000
    X = 0x4000
    Y = 0x8000
    GUIDE = 0x0400           # Xbox按钮


# ==================== 数据结构 ====================

@dataclass
class StickState:
    """摇杆状态数据类"""
    x: float          # -1.0 到 1.0
    y: float          # -1.0 到 1.0
    magnitude: float  # 0.0 到 1.0
    raw_x: int        # 原始值 -32768 到 32767
    raw_y: int        # 原始值 -32768 到 32767


@dataclass
class ControllerState:
    """控制器完整状态"""
    connected: bool
    packet_number: int

    # 按键状态
    buttons: set

    # 摇杆状态
    left_stick: StickState
    right_stick: StickState

    # 扳机键
    left_trigger: float   # 0.0 到 1.0
    right_trigger: float  # 0.0 到 1.0

    # 原始数据
    raw_buttons: int

    def is_pressed(self, button: ButtonFlags) -> bool:
        """检查指定按键是否被按下"""
        return button in self.buttons

    def get_left_stick(self) -> tuple[float, float]:
        """获取左摇杆坐标 (x, y)"""
        return (self.left_stick.x, self.left_stick.y)

    def get_right_stick(self) -> tuple[float, float]:
        """获取右摇杆坐标 (x, y)"""
        return (self.right_stick.x, self.right_stick.y)


# ==================== Linux joystick 常量 ====================

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_FMT = "IhBB"  # time(ms), value(int16), type(uint8), number(uint8)
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)


# ==================== 默认按键/轴映射 ====================

# xpad 驱动下 Xbox 手柄的 axis 编号映射
DEFAULT_AXIS_MAP: Dict[int, str] = {
    0: "left_x",
    1: "left_y",
    2: "left_trigger",
    3: "right_x",
    4: "right_y",
    5: "right_trigger",
    6: "hat_x",
    7: "hat_y",
}

# xpad 驱动下 Xbox 手柄的 button 编号映射
DEFAULT_BUTTON_MAP: Dict[int, ButtonFlags] = {
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


# ==================== 核心驱动类 ====================

class LinuxJsDriver:
    """
    Xbox 手柄 Linux 原生 joystick 驱动类

    功能：
    - 连接状态检测
    - 按键状态读取
    - 摇杆数据读取（含死区处理）
    - 扳机键读取
    - 事件回调机制

    接口与 Windows XInputDriver / PygameDriver 保持一致。
    """

    def __init__(
        self,
        controller_index: int = 0,
        device_path: Optional[str] = None,
        axis_map: Optional[Dict[int, str]] = None,
        button_map: Optional[Dict[int, ButtonFlags]] = None,
    ):
        """
        初始化 Linux joystick 驱动

        Args:
            controller_index: 控制器索引 (0-3)，用于构造默认设备路径 /dev/input/js{N}
            device_path: 显式指定设备路径，如 /dev/input/js0；提供时优先于 controller_index
            axis_map: 自定义 axis 映射，默认使用 xpad 映射
            button_map: 自定义 button 映射，默认使用 xpad 映射
        """
        if not 0 <= controller_index < XINPUT_MAX_CONTROLLERS:
            raise ValueError(f"Controller index must be 0-{XINPUT_MAX_CONTROLLERS - 1}")

        self.controller_index = controller_index
        self.device_path = device_path or f"/dev/input/js{controller_index}"
        self.axis_map = axis_map if axis_map is not None else DEFAULT_AXIS_MAP.copy()
        self.button_map = button_map if button_map is not None else DEFAULT_BUTTON_MAP.copy()

        self._fd: Optional[int] = None
        self._last_packet_number = 0

        # 内部状态缓存
        self._axes: Dict[str, int] = {
            "left_x": 0,
            "left_y": 0,
            "right_x": 0,
            "right_y": 0,
            "left_trigger": 0,
            "right_trigger": 0,
            "hat_x": 0,
            "hat_y": 0,
        }
        self._buttons: Dict[int, bool] = {}

        # 死区设置
        self.left_deadzone = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE
        self.right_deadzone = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE
        self.trigger_threshold = XINPUT_GAMEPAD_TRIGGER_THRESHOLD

        # 回调函数
        self._button_press_callbacks: dict[ButtonFlags, List[Callable]] = {}
        self._button_release_callbacks: dict[ButtonFlags, List[Callable]] = {}
        self._state_change_callback: Optional[Callable] = None

        # 后台轮询
        self._polling = False
        self._poll_thread: Optional[threading.Thread] = None
        self._poll_interval = 0.016  # 约60Hz
        self._previous_buttons: set = set()

        self._open_device()

    def _open_device(self) -> bool:
        """尝试打开 joystick 设备文件"""
        self._close_device()
        if not os.path.exists(self.device_path):
            return False
        try:
            # 非阻塞读取，方便 get_state 时一次性消费所有事件
            self._fd = os.open(self.device_path, os.O_RDONLY | os.O_NONBLOCK)
            # 读取并丢弃初始化事件，建立当前状态快照
            self._drain_events()
            return True
        except OSError:
            self._fd = None
            return False

    def _close_device(self) -> None:
        """关闭设备文件"""
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None

    def _drain_events(self) -> None:
        """读取当前所有可用事件并更新内部状态"""
        if self._fd is None:
            return
        while True:
            try:
                data = os.read(self._fd, JS_EVENT_SIZE)
                if len(data) < JS_EVENT_SIZE:
                    break
                self._process_event(data)
            except BlockingIOError:
                break
            except OSError:
                self._close_device()
                break

    def _process_event(self, data: bytes) -> None:
        """解析单个 joystick 事件并更新内部状态"""
        _, value, event_type, number = struct.unpack(JS_EVENT_FMT, data)
        is_init = bool(event_type & JS_EVENT_INIT)
        event_type &= ~JS_EVENT_INIT

        if event_type == JS_EVENT_BUTTON:
            self._buttons[number] = bool(value)
        elif event_type == JS_EVENT_AXIS:
            axis_name = self.axis_map.get(number)
            if axis_name:
                self._axes[axis_name] = int(value)

    def _apply_deadzone(self, x: int, y: int, deadzone: int) -> tuple[float, float, float]:
        """
        应用摇杆死区处理

        Returns:
            (normalized_x, normalized_y, magnitude)
        """
        magnitude = (x ** 2 + y ** 2) ** 0.5

        if magnitude <= deadzone:
            return 0.0, 0.0, 0.0

        max_magnitude = 32767.0
        normalized_magnitude = min((magnitude - deadzone) / (max_magnitude - deadzone), 1.0)
        scale = normalized_magnitude / (magnitude / max_magnitude)
        normalized_x = (x / max_magnitude) * scale
        normalized_y = (y / max_magnitude) * scale

        return normalized_x, normalized_y, normalized_magnitude

    def _normalize_trigger(self, value: int) -> float:
        """归一化扳机键值（Linux js 触发器通常为 0..32767）"""
        # 统一映射到 0..255 后与 XInput 阈值比较
        normalized = int(max(0, value) * 255 / 32767)
        if normalized < self.trigger_threshold:
            return 0.0
        return min(normalized / 255.0, 1.0)

    def _build_button_set(self) -> Tuple[set, int]:
        """根据内部按钮和 hat 状态构造 ButtonFlags 集合及 raw_buttons"""
        buttons: set = set()
        raw_buttons = 0

        # 普通按键
        for idx, pressed in self._buttons.items():
            if not pressed:
                continue
            flag = self.button_map.get(idx)
            if flag:
                buttons.add(flag)
                raw_buttons |= int(flag)

        # D-Pad (hat)
        hat_x = self._axes.get("hat_x", 0)
        hat_y = self._axes.get("hat_y", 0)
        if hat_x < 0:
            buttons.add(ButtonFlags.DPAD_LEFT)
            raw_buttons |= int(ButtonFlags.DPAD_LEFT)
        elif hat_x > 0:
            buttons.add(ButtonFlags.DPAD_RIGHT)
            raw_buttons |= int(ButtonFlags.DPAD_RIGHT)
        if hat_y < 0:
            buttons.add(ButtonFlags.DPAD_UP)
            raw_buttons |= int(ButtonFlags.DPAD_UP)
        elif hat_y > 0:
            buttons.add(ButtonFlags.DPAD_DOWN)
            raw_buttons |= int(ButtonFlags.DPAD_DOWN)

        return buttons, raw_buttons

    def get_state(self) -> ControllerState:
        """
        获取当前控制器状态

        Returns:
            ControllerState 对象
        """
        # 如果设备未打开，尝试重新打开（支持热插拔恢复）
        if self._fd is None:
            self._open_device()

        if self._fd is not None:
            self._drain_events()

        if self._fd is None:
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

        self._last_packet_number += 1

        # Linux js 的 Y 轴向上为负，需翻转以匹配 XInput 坐标系（向上为正）
        left_raw_x = self._axes.get("left_x", 0)
        left_raw_y = -self._axes.get("left_y", 0)
        right_raw_x = self._axes.get("right_x", 0)
        right_raw_y = -self._axes.get("right_y", 0)

        left_x, left_y, left_m = self._apply_deadzone(left_raw_x, left_raw_y, self.left_deadzone)
        right_x, right_y, right_m = self._apply_deadzone(right_raw_x, right_raw_y, self.right_deadzone)

        buttons, raw_buttons = self._build_button_set()

        return ControllerState(
            connected=True,
            packet_number=self._last_packet_number,
            buttons=buttons,
            left_stick=StickState(
                x=left_x, y=left_y, magnitude=left_m,
                raw_x=left_raw_x, raw_y=left_raw_y,
            ),
            right_stick=StickState(
                x=right_x, y=right_y, magnitude=right_m,
                raw_x=right_raw_x, raw_y=right_raw_y,
            ),
            left_trigger=self._normalize_trigger(self._axes.get("left_trigger", 0)),
            right_trigger=self._normalize_trigger(self._axes.get("right_trigger", 0)),
            raw_buttons=raw_buttons,
        )

    def is_connected(self) -> bool:
        """检查控制器是否已连接"""
        if self._fd is None:
            self._open_device()
        return self._fd is not None

    def set_vibration(self, left_motor: float, right_motor: float):
        """
        设置手柄震动

        Args:
            left_motor: 左侧震动强度 (0.0 - 1.0)
            right_motor: 右侧震动强度 (0.0 - 1.0)

        注意：Linux 原生 joystick 接口不直接支持震动。
        当前版本保留 API 兼容性，实际震动通过底层驱动特定方式实现。
        后续可通过 evdev force feedback 事件扩展。
        """
        # TODO: 通过 /dev/input/event* 的 evdev FF_RUMBLE 事件实现震动
        # 目前先保持 API 一致，避免应用层异常
        pass

    def stop_vibration(self):
        """停止震动"""
        pass

    def get_capabilities(self) -> Optional[dict]:
        """获取控制器能力信息"""
        return {
            "type": 1,
            "subtype": 1,
            "subtype_name": "Gamepad",
            "flags": 0,
            "has_vibration": False,
        }

    # ==================== 事件回调机制 ====================

    def on_button_press(self, button: ButtonFlags, callback: Callable):
        """注册按键按下回调"""
        if button not in self._button_press_callbacks:
            self._button_press_callbacks[button] = []
        self._button_press_callbacks[button].append(callback)

    def on_button_release(self, button: ButtonFlags, callback: Callable):
        """注册按键释放回调"""
        if button not in self._button_release_callbacks:
            self._button_release_callbacks[button] = []
        self._button_release_callbacks[button].append(callback)

    def on_state_change(self, callback: Callable[[ControllerState], None]):
        """注册状态变化回调"""
        self._state_change_callback = callback

    def _trigger_callbacks(self, current_buttons: set):
        """触发回调函数"""
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
        """
        开始后台轮询

        Args:
            interval: 轮询间隔（秒），默认约 60Hz
        """
        if self._polling:
            return

        if interval is not None:
            self._poll_interval = interval

        self._polling = True
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def stop_polling(self):
        """停止后台轮询"""
        self._polling = False
        if self._poll_thread:
            self._poll_thread.join(timeout=1.0)
            self._poll_thread = None

    def _poll_loop(self):
        """后台轮询循环"""
        while self._polling:
            state = self.get_state()
            if state.connected:
                self._trigger_callbacks(state.buttons)
                if self._state_change_callback:
                    self._state_change_callback(state)
            time.sleep(self._poll_interval)


# ==================== 便捷函数 ====================

def get_connected_controllers() -> List[int]:
    """获取所有已连接的控制器索引列表"""
    connected = []
    for i in range(XINPUT_MAX_CONTROLLERS):
        path = f"/dev/input/js{i}"
        if os.path.exists(path):
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                os.close(fd)
                connected.append(i)
            except OSError:
                pass
    return connected


def wait_for_connection(controller_index: int = 0, timeout: Optional[float] = None) -> bool:
    """
    等待控制器连接

    Args:
        controller_index: 控制器索引
        timeout: 超时时间（秒），None 表示无限等待

    Returns:
        是否成功连接
    """
    path = f"/dev/input/js{controller_index}"
    start = time.time()
    while True:
        if os.path.exists(path):
            return True
        if timeout is not None and (time.time() - start) > timeout:
            return False
        time.sleep(0.1)


# ==================== 兼容性别名 ====================

XboxController = LinuxJsDriver
