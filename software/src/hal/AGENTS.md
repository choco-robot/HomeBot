<!-- From: d:\develop\homebot\software\src\hal\AGENTS.md -->
# HomeBot 硬件抽象层 Agent 文档

本文档面向 AI 编程助手，提供 HAL 各驱动的速查、硬件参数和新增硬件适配指南。

> 根级项目文档参见 `d:\develop\homebot\AGENTS.md`
> 源码总览参见 `software/src/AGENTS.md`
> 配置体系速查参见 `software/src/configs/AGENTS.md`

---

## 模块速查

### HAL 清单

| 驱动 | 文件 | 主类 | 硬件类型 | 配置类 |
|------|------|------|---------|--------|
| 全向底盘 | `hal/chassis/driver.py` | `OmniChassisDriver` | 飞特 ST3215 舵机 ×3 | `ChassisConfig` |
| 差动底盘 | `hal/chassis/diff_driver.py` | `DiffChassisDriver` | ESP32S3 + SMS/STS 电机 | `ChassisConfig` |
| 机械臂 | `hal/arm/driver.py` | `ArmDriver` | 飞特 ST3215 舵机 ×6 | `ArmConfig` |
| 机械臂运动学 | `hal/arm/Kinematics.py` | `ArmKinematics` | — | `ArmConfig` |
| 摄像头 | `hal/camera/driver.py` | `CameraDriver` | USB 摄像头 (OpenCV) | `CameraConfig` |
| 音频 | `hal/audio/driver.py` | `AudioDriver` | 占位符 | — |
| 电池监测 | `hal/battery/driver.py` | `BatteryDriver` | 通过舵机电压读取 | `BatteryConfig` |
| 游戏手柄 | `hal/gamepad/` | `XInputDriver` / `PygameDriver` | Xbox 手柄 | `GamepadConfig` |
| 舵机总线 | `hal/ftservo_driver.py` | `FTServoBus` | ST3215 / STS 串口总线 | `ChassisConfig`/`ArmConfig` |

### 应用 → HAL → 配置映射

| HAL 模块 | 被哪些应用/服务使用 | 对应 Config Dataclass | 关键硬件参数 |
|----------|-------------------|----------------------|-------------|
| `chassis/driver.py` | ChassisService (omni) | `ChassisConfig` | LF=8, RF=7, Rear=9, r=0.08m, R=0.18m |
| `chassis/diff_driver.py` | ChassisService (diff) | `ChassisConfig` | Chassis=0x24, Left=0x21, Right=0x22, IMU=0x23 |
| `ftservo_driver.py` | ChassisService, ArmService | 共用串口 | Baud=1M, Pos 0-4095, Speed ±32767 |
| `arm/driver.py` | ArmService | `ArmConfig` (全局) | IDs 1-6, L1=115mm, L2=130mm |
| `arm/Kinematics.py` | remote_control, gamepad_control | `ArmConfig` | L1, L2 用于 IK/FK |
| `camera/driver.py` | VisionService | `CameraConfig` | Device 0, 1920×1080@30 |
| `battery/driver.py` | ChassisService | `BatteryConfig` | Servo IDs [1], 阈值 12.6/10.5/9.5/9.0V |
| `gamepad/` | gamepad_control | `GamepadConfig` | 死区、步长 |

---

## 1. 底盘驱动

### 1.1 全向底盘 (OmniChassisDriver)

**文件**: `hal/chassis/driver.py`

**职责**: 三轮全向底盘控制。速度 `(vx, vy, omega)` → 逆运动学 → 各轮速度 → 舵机轮式模式速度指令。

**关键方法**:
- `initialize()` → 连接总线、设置轮式模式、使能扭矩
- `set_velocity(vx, vy, omega)` → 钳制到配置限制 → 调用 `_inverse_kinematics()`
- `_inverse_kinematics(vx, vy, omega) → [v_left, v_right, v_rear]` (m/s)
- `_forward_kinematics(v_left, v_right, v_rear) → (vx, vy, omega)`
- `_wheel_speed_to_servo(wheel_speed) → int` — 物理模型或比例回退
- `read_wheel_speeds() → Dict[servo_id, speed]` — `sync_read_speeds()`
- `get_actual_velocity() → (vx, vy, omega)` — 编码器正运动学
- `move_forward(distance, speed)`, `rotate(angle_deg, angular_speed)` — 开环时间控制

**硬件参数** (`ChassisConfig`):
- `chassis_type = "omni"`（使用此驱动时）
- `left_front_id = 8`, `right_front_id = 7`, `rear_id = 9`
- `wheel_radius = 0.08` m, `chassis_radius = 0.18` m
- `max_linear_speed = 0.5` m/s, `max_angular_speed = 1.0` rad/s
- `serial_port`, `baudrate = 1000000`

**模拟/回退**: 依赖 `FTServoBus` 的模拟模式（SDK 未安装时自动进入）

---

### 1.2 差动底盘 (DiffChassisDriver)

**文件**: `hal/chassis/diff_driver.py`

**职责**: 双轮差动底盘，基于 ESP32S3-baseboard 控制器。支持闭环前进/转向、编码器里程计、IMU 读取。

**关键方法**:
- `initialize()` → PING 底盘虚拟设备 `0x24`
- `set_velocity(vx, vy, omega)` → 忽略 `vy`，转换为 `move(vx_mmps, vz_degps)`
- `move_forward(distance, speed)` → 写入 `REG_GOAL_POSITION` (0x42)，等待完成
- `rotate(angle_deg, angular_speed)` → 写入 `REG_ACC` (0x41)，等待完成
- `read_odometry() → {x, y, theta, vx, vz}` — 从 `REG_ODOM_START` (0x72) 读 10 字节
- `reset_odometry()` → 写 `0x01` 到 `REG_ODOM_CMD` (0x7F)
- `read_imu() → {yaw, pitch, roll}` — 从 `REG_IMU_START` (0x50) 读 6 字节
- `read_imu_all()` — 18 字节（含陀螺仪 + 加速度计）
- `read_encoder(motor_id)`, `read_motor_speed(motor_id)`, `read_voltage()`, `read_temperature()`

**寄存器定义**:
```python
chassis_id = 0x24
motor_left_id = 0x21
motor_right_id = 0x22
imu_id = 0x23
REG_GOAL_SPEED = 0x2E      # 46
REG_GOAL_POSITION = 0x2A   # 42
REG_ACC = 0x29             # 41
REG_ODOM_START = 0x72
REG_ODOM_CMD = 0x7F
REG_IMU_START = 0x50
REG_IMU_CMD = 0x70
```

**硬件参数** (`ChassisConfig`):
- `chassis_type = "diff"`（使用此驱动时）
- `wheel_track = 0.45` m, `wheel_diameter = 0.125` m, `max_rpm = 120.0`

**模拟/回退**: 需要真实 ESP32S3 控制器；支持共用 `FTServoBus` 串口句柄

---

## 2. 机械臂驱动

### 2.1 机械臂驱动器 (ArmDriver)

**文件**: `hal/arm/driver.py`

**职责**: 6-DOF 机械臂位置控制。关节角度 → 舵机位置转换、批量 sync-write、夹爪控制、回零位。

**关键方法**:
- `initialize(auto_home=False) → bool`
- `set_joint_angle(joint_name, angle, speed, wait) → bool`
- `set_joint_angles(angles_dict, speed, wait) → bool` — 使用 `sync_write_positions()` 批量写入
- `move_to_home(speed) → bool`
- `set_gripper(open_amount) → bool` (0=关闭, 1=打开)
- `open_gripper()`, `close_gripper()`
- `get_joint_angle(joint_name)`, `get_all_joint_angles()`
- `get_joint_states() → Dict[str, ServoState]`
- `emergency_stop()` — 关闭扭矩

**硬件参数**:
- 运行时实际使用 `configs.config.ArmConfig` 中的 IDs 1-6（base, shoulder, elbow, wrist_flex, wrist_roll, gripper）
- `angle_offset = 2048`, `angle_scale = 4096/360 ≈ 11.378`
- 位置范围: 0-4095（对应 0-360°）
- 关节限制: base (-180,180), shoulder (0,180), elbow (0,180), wrist_flex (-90,90), wrist_roll (-180,180), gripper (0,90)

**配置映射**: 全局 `ArmConfig` 覆盖本地默认值

**模拟/回退**: 继承 `FTServoBus` 模拟模式

---

### 2.2 机械臂运动学 (ArmKinematics)

**文件**: `hal/arm/Kinematics.py`

**职责**: 2-DOF 平面运动学（肩 + 肘）。前向/逆向运动学、腕部自动调平。

**关键方法**:
- `__init__(L1=120.0, L2=100.0)` — 连杆长度 mm
- `forward_kinematics(shoulder_angle, elbow_angle) → (r, z)`
- `inverse_kinematics(r, z, elbow_up=True) → Optional[(shoulder, elbow)]`
- `inverse_kinematics_all(r, z) → List[(s1,e1), (s2,e2)]`
- `compute_wrist_flex(shoulder, elbow, target_orientation=0.0) → wrist_flex`
- `is_reachable(r, z) → bool`
- `get_workspace_radius() → (min_r, max_r)`

**配置映射**: 使用 `ArmConfig.upper_arm_length` (L1=115.0) 和 `forearm_length` (L2=130.0)

---

## 3. 摄像头驱动 (CameraDriver)

**文件**: `hal/camera/driver.py`

**职责**: OpenCV VideoCapture 封装。

**关键方法**:
- `__init__(device=0)` → `cv2.VideoCapture(device)`
- `capture_frame()` → `cap.read()`，返回 BGR numpy 数组或 None
- `release()`

**配置映射**: `CameraConfig.device_id=0`, `width=1920`, `height=1080`, `fps=30`

---

## 4. 音频驱动 (AudioDriver)

**文件**: `hal/audio/driver.py`

**状态**: 占位符。`record(duration)` 和 `play(data)` 为空实现，返回空字节。

实际音频功能由 `services.speech_service.voice_engine` 通过 `sounddevice` 直接处理。

---

## 5. 电池监测 (BatteryDriver)

**文件**: `hal/battery/driver.py`

**职责**: 通过 `FTServoBus.read_voltage()` 读取舵机电压，映射为百分比和状态。

**关键方法**:
- `read_state() → BatteryState` — 按 `servo_ids` 列表逐个尝试
- `_voltage_to_percentage(voltage) → float` — 线性映射
- `_determine_status(voltage) → BatteryStatus` (NORMAL/LOW/CRITICAL)
- `is_low_battery()`, `is_critical_battery()`

**硬件参数**:
- `DEFAULT_FULL_VOLTAGE = 12.6` V（3S 锂电满电）
- `DEFAULT_LOW_VOLTAGE = 10.5` V
- `DEFAULT_CRITICAL_VOLTAGE = 9.5` V
- `DEFAULT_MIN_VOLTAGE = 9.0` V

**配置映射**: `BatteryConfig.servo_ids`, `full_voltage`, `low_voltage`, `critical_voltage`, `min_voltage`

---

## 6. 游戏手柄驱动 (hal/gamepad/)

**文件**: `__init__.py`, `xinput_core.py`, `pygame_backend.py`

**平台选择** (`__init__.py`):
- Windows → `XInputDriver`
- 其他平台 → `PygameDriver`

**关键方法** (两者通用接口):
- `get_state() → ControllerState`
- `is_connected() → bool`
- `set_vibration(left, right)`, `stop_vibration()`
- `start_polling(interval)`, `stop_polling()`
- `on_button_press(button, callback)`, `on_button_release(button, callback)`

**硬件参数**:
- `XINPUT_MAX_CONTROLLERS = 4`
- `XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE = 7849`
- `XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE = 8689`
- `XINPUT_GAMEPAD_TRIGGER_THRESHOLD = 30`

**模拟/回退**:
- Windows 缺少 XInput DLL → `RuntimeError`
- pygame 后端需安装 `pygame`；震动可能不支持所有平台

---

## 7. 飞特舵机总线 (FTServoBus)

**文件**: `hal/ftservo_driver.py`

**职责**: 底层串口通信封装。位置模式、速度模式（轮式）、同步读写、电压/温度读取。

**关键方法**:
- `connect() → bool` — 打开串口、设置波特率、创建 `sms_sts` 包处理器
- `write_position(servo_id, position, speed, acc) → bool`
- `sync_write_positions({sid: (pos, speed, acc), ...}) → bool`
- `read_position(servo_id) → Optional[int]`
- `sync_read_positions(servo_ids) → Dict[sid, pos]`
- `read_speed(servo_id) → Optional[int]`
- `sync_read_speeds(servo_ids) → Dict[sid, signed_speed]`
- `set_wheel_mode(servo_id) → bool`
- `write_speed(servo_id, speed, acc, quiet=False) → bool`
- `torque_enable(servo_id=-1)`, `torque_disable(servo_id=-1)` — `-1` 为广播
- `read_voltage(servo_id) → Optional[float]` (V，除以 10)
- `read_temperature(servo_id) → Optional[int]` (°C)
- `get_state(servo_id) → ServoState`

**硬件参数**:
- `DEFAULT_BAUDRATE = 1000000`
- 位置范围: 0-4095（对应 0-360°）
- 速度范围: -32767 ~ 32767

**模拟/回退（重要）**:
- 当 `scservo_sdk` 导入失败时，**自动进入完整模拟模式**
- 模拟 `PortHandler`、`sms_sts` 类，返回假数据：
  - 位置: 2048（中位）
  - 速度: 0
  - 电压: 12.0V
  - 温度: 25°C
- `COMM_SUCCESS = 0`, `BROADCAST_ID = 0xFE`

> **这对开发调试非常关键**：没有硬件时，服务层仍然可以正常启动和运行。

---

## 修改指南

### 新增底盘类型

1. 在 `hal/chassis/` 下创建新驱动文件（如 `mecanum_driver.py`）
2. 实现标准接口:
   - `initialize() -> bool`
   - `set_velocity(vx, vy, omega) -> bool`
   - `stop()`
   - `close()`
   - `get_actual_velocity() -> (vx, vy, omega)`（可选）
3. 在 `ChassisConfig` 中新增 `chassis_type` 枚举值
4. 在 `services/motion_service/chassis_service.py` 的 `RealChassisController` 中按 `config.chassis_type` 路由

### 新增机械臂关节

1. 修改 `ArmConfig` 中的舵机 ID 映射和 `joint_limits`
2. 修改 `ArmDriver` 的关节名 → ID 映射
3. 如需修改运动学维度，更新 `ArmKinematics`（当前为 2-DOF 平面）

### 新增传感器驱动

1. 在 `hal/` 下创建新目录（如 `hal/lidar/`）
2. 提供 `initialize()`, `read()`, `close()` 标准接口
3. 在 `configs/config.py` 中新增对应 Config dataclass
4. 在对应的服务层（如 `navigation/services/`）中创建发布服务

### 修改硬件参数

- **连杆长度** (`upper_arm_length`, `forearm_length`): 标记为 **"人工设置，AI勿动"**，修改后需重新校准机械臂
- **舵机 ID**: 修改后需确保机械臂物理接线对应
- **轮径/底盘半径**: 影响逆运动学精度，修改后建议重新测试里程计

---

## 人类文档索引

| 主题 | 对应人类文档 (`docs/`) |
|------|----------------------|
| 物料清单/硬件参数 | `HomeBot_BOM物料清单.md` |
| 工具脚本（设备枚举/校准） | `工具脚本使用指南.md` |
| 配置修改 | `配置修改说明.md` |
