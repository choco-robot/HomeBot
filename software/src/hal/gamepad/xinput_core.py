"""
Xbox手柄XInput驱动核心模块
基于Windows XInput API (XInput1_4.dll / XInput9_1_0.dll)
支持Xbox 360/One/Series X|S手柄
"""

import ctypes
import ctypes.wintypes
from enum import IntFlag
from dataclasses import dataclass
from typing import Optional, Callable, List
import time
import threading


# ==================== 常量定义 ====================

# XInput DLL名称（尝试多个版本）
XINPUT_DLLS = ['XInput1_4.dll', 'XInput9_1_0.dll', 'XInput1_3.dll']

# XInput常量
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
    LEFT_THUMB = 0x0040      # 左摇杆按下
    RIGHT_THUMB = 0x0080     # 右摇杆按下
    LEFT_SHOULDER = 0x0100   # LB
    RIGHT_SHOULDER = 0x0200  # RB
    A = 0x1000
    B = 0x2000
    X = 0x4000
    Y = 0x8000
    GUIDE = 0x0400           # Xbox按钮（部分DLL支持）


# ==================== 数据结构 ====================

class XINPUT_GAMEPAD(ctypes.Structure):
    """XInput游戏手柄原始数据结构"""
    _fields_ = [
        ("wButtons", ctypes.wintypes.WORD),
        ("bLeftTrigger", ctypes.c_ubyte),    # 必须使用无符号字节 (0-255)
        ("bRightTrigger", ctypes.c_ubyte),   # 使用 c_ubyte 代替 BYTE 避免符号问题
        ("sThumbLX", ctypes.wintypes.SHORT),
        ("sThumbLY", ctypes.wintypes.SHORT),
        ("sThumbRX", ctypes.wintypes.SHORT),
        ("sThumbRY", ctypes.wintypes.SHORT),
    ]


class XINPUT_STATE(ctypes.Structure):
    """XInput状态结构"""
    _fields_ = [
        ("dwPacketNumber", ctypes.wintypes.DWORD),
        ("Gamepad", XINPUT_GAMEPAD),
    ]


class XINPUT_VIBRATION(ctypes.Structure):
    """XInput震动结构"""
    _fields_ = [
        ("wLeftMotorSpeed", ctypes.wintypes.WORD),
        ("wRightMotorSpeed", ctypes.wintypes.WORD),
    ]


class XINPUT_BATTERY_INFORMATION(ctypes.Structure):
    """XInput电池信息"""
    _fields_ = [
        ("BatteryType", ctypes.wintypes.BYTE),
        ("BatteryLevel", ctypes.wintypes.BYTE),
    ]


class XINPUT_CAPABILITIES(ctypes.Structure):
    """XInput设备能力信息"""
    _fields_ = [
        ("Type", ctypes.wintypes.BYTE),
        ("SubType", ctypes.wintypes.BYTE),
        ("Flags", ctypes.wintypes.WORD),
        ("Gamepad", XINPUT_GAMEPAD),
        ("Vibration", XINPUT_VIBRATION),
    ]


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


# ==================== 核心驱动类 ====================

class XInputDriver:
    """
    Xbox手柄XInput驱动类
    
    功能：
    - 连接状态检测
    - 按键状态读取
    - 摇杆数据读取（含死区处理）
    - 扳机键读取
    - 震动控制
    - 事件回调机制
    """
    
    def __init__(self, controller_index: int = 0):
        """
        初始化XInput驱动
        
        Args:
            controller_index: 控制器索引 (0-3)
        """
        if not 0 <= controller_index < XINPUT_MAX_CONTROLLERS:
            raise ValueError(f"Controller index must be 0-{XINPUT_MAX_CONTROLLERS-1}")
        
        self.controller_index = controller_index
        self._xinput = None
        self._last_packet_number = 0
        
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
        
        self._load_xinput()
    
    def _load_xinput(self):
        """加载XInput DLL"""
        for dll_name in XINPUT_DLLS:
            try:
                self._xinput = ctypes.windll.LoadLibrary(dll_name)
                print(f"[XInput] Loaded {dll_name}")
                return
            except OSError:
                continue
        raise RuntimeError("Failed to load XInput DLL. Please ensure XInput is installed.")
    
    def _apply_deadzone(self, x: int, y: int, deadzone: int) -> tuple[float, float, float]:
        """
        应用摇杆死区处理
        
        Returns:
            (normalized_x, normalized_y, magnitude)
        """
        # 计算向量长度
        magnitude = (x ** 2 + y ** 2) ** 0.5
        
        if magnitude <= deadzone:
            return 0.0, 0.0, 0.0
        
        # 归一化
        max_magnitude = 32767.0
        normalized_x = x / max_magnitude
        normalized_y = y / max_magnitude
        normalized_magnitude = min(magnitude / max_magnitude, 1.0)
        
        # 应用死区补偿（平滑过渡）
        if magnitude > deadzone:
            adjusted_magnitude = (magnitude - deadzone) / (max_magnitude - deadzone)
            normalized_magnitude = min(adjusted_magnitude, 1.0)
            # 保持方向，调整幅度
            scale = normalized_magnitude / (magnitude / max_magnitude)
            normalized_x *= scale
            normalized_y *= scale
        
        return normalized_x, normalized_y, normalized_magnitude
    
    def _normalize_trigger(self, value: int) -> float:
        """归一化扳机键值"""
        if value < self.trigger_threshold:
            return 0.0
        return min(value / 255.0, 1.0)
    
    def get_state(self) -> ControllerState:
        """
        获取当前控制器状态
        
        Returns:
            ControllerState对象
        """
        state = XINPUT_STATE()
        result = self._xinput.XInputGetState(self.controller_index, ctypes.byref(state))
        
        if result != 0:  # ERROR_SUCCESS = 0
            # 控制器未连接
            return ControllerState(
                connected=False,
                packet_number=0,
                buttons=set(),
                left_stick=StickState(0, 0, 0, 0, 0),
                right_stick=StickState(0, 0, 0, 0, 0),
                left_trigger=0.0,
                right_trigger=0.0,
                raw_buttons=0
            )
        
        # 解析按键
        buttons = set()
        raw_buttons = state.Gamepad.wButtons
        for flag in ButtonFlags:
            if raw_buttons & flag:
                buttons.add(flag)
        
        # 处理摇杆死区
        lx, ly, lm = self._apply_deadzone(
            state.Gamepad.sThumbLX,
            state.Gamepad.sThumbLY,
            self.left_deadzone
        )
        rx, ry, rm = self._apply_deadzone(
            state.Gamepad.sThumbRX,
            state.Gamepad.sThumbRY,
            self.right_deadzone
        )
        
        return ControllerState(
            connected=True,
            packet_number=state.dwPacketNumber,
            buttons=buttons,
            left_stick=StickState(
                x=lx, y=ly, magnitude=lm,
                raw_x=state.Gamepad.sThumbLX,
                raw_y=state.Gamepad.sThumbLY
            ),
            right_stick=StickState(
                x=rx, y=ry, magnitude=rm,
                raw_x=state.Gamepad.sThumbRX,
                raw_y=state.Gamepad.sThumbRY
            ),
            left_trigger=self._normalize_trigger(state.Gamepad.bLeftTrigger),
            right_trigger=self._normalize_trigger(state.Gamepad.bRightTrigger),
            raw_buttons=raw_buttons
        )
    
    def is_connected(self) -> bool:
        """检查控制器是否已连接"""
        state = XINPUT_STATE()
        result = self._xinput.XInputGetState(self.controller_index, ctypes.byref(state))
        return result == 0
    
    def set_vibration(self, left_motor: float, right_motor: float):
        """
        设置手柄震动
        
        Args:
            left_motor: 左侧震动强度 (0.0 - 1.0)
            right_motor: 右侧震动强度 (0.0 - 1.0)
        """
        vibration = XINPUT_VIBRATION()
        vibration.wLeftMotorSpeed = int(max(0.0, min(1.0, left_motor)) * 65535)
        vibration.wRightMotorSpeed = int(max(0.0, min(1.0, right_motor)) * 65535)
        self._xinput.XInputSetState(self.controller_index, ctypes.byref(vibration))
    
    def stop_vibration(self):
        """停止震动"""
        self.set_vibration(0, 0)
    
    def get_capabilities(self) -> Optional[dict]:
        """获取控制器能力信息"""
        caps = XINPUT_CAPABILITIES()
        result = self._xinput.XInputGetCapabilities(
            self.controller_index, 0, ctypes.byref(caps)
        )
        if result != 0:
            return None
        
        subtype_names = {
            0: "Unknown",
            1: "Gamepad",
            2: "Wheel",
            3: "Arcade Stick",
            4: "Flight Stick",
            5: "Dance Pad",
            6: "Guitar",
            7: "Drum Kit",
            8: "Guitar Alternate",
            11: "Guitar Bass",
            19: "Arcade Pad",
        }
        
        return {
            "type": caps.Type,
            "subtype": caps.SubType,
            "subtype_name": subtype_names.get(caps.SubType, f"Unknown({caps.SubType})"),
            "flags": caps.Flags,
            "has_vibration": bool(caps.Flags & 0x0001),
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
        # 检测新按下的按键
        pressed = current_buttons - self._previous_buttons
        for btn in pressed:
            if btn in self._button_press_callbacks:
                for cb in self._button_press_callbacks[btn]:
                    cb(btn)
        
        # 检测释放的按键
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
            interval: 轮询间隔（秒），默认约60Hz
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
    driver = XInputDriver(0)
    for i in range(XINPUT_MAX_CONTROLLERS):
        driver.controller_index = i
        if driver.is_connected():
            connected.append(i)
    return connected


def wait_for_connection(controller_index: int = 0, timeout: Optional[float] = None) -> bool:
    """
    等待控制器连接
    
    Args:
        controller_index: 控制器索引
        timeout: 超时时间（秒），None表示无限等待
    
    Returns:
        是否成功连接
    """
    driver = XInputDriver(controller_index)
    start = time.time()
    while True:
        if driver.is_connected():
            return True
        if timeout and (time.time() - start) > timeout:
            return False
        time.sleep(0.1)


# ==================== 兼容性别名 ====================

XboxController = XInputDriver
