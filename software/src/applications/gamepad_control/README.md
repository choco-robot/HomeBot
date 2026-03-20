# HomeBot 游戏手柄控制应用

使用 Xbox 游戏手柄同时控制底盘和机械臂。

## 功能特性

- **同时控制**：无需模式切换，左手控制底盘，右手控制机械臂
- **震动反馈**：紧急停止时手柄震动提示
- **紧急停止**：Back 键一键急停，锁定所有运动
- **复位功能**：Start 键复位底盘和机械臂
- **手腕水平**：B 键自动计算手腕角度保持末端水平

## 系统要求

- Windows 系统（使用 XInput API）
- Xbox 手柄（有线/无线适配器/蓝牙）
- 已启动底盘服务和机械臂服务

## 安装

游戏手柄驱动已包含在项目中，无需额外安装。

## 使用方法

### 启动服务

确保底盘服务和机械臂服务已启动：

```bash
# 终端 1: 启动底盘服务
cd software/src
python -m services.motion_service.chassis_service

# 终端 2: 启动机械臂服务
cd software/src
python -m services.motion_service.arm_service
```

### 启动游戏手柄控制

```bash
cd software/src
python -m applications.gamepad_control

# 指定手柄索引（如果连接了多个手柄）
python -m applications.gamepad_control --controller 1

# 显示详细日志
python -m applications.gamepad_control --verbose
```

## 控制映射

### 底盘控制（左手）

| 输入 | 功能 | 说明 |
|-----|------|------|
| 左摇杆 ↑↓ | 前进 / 后退 | 控制 vx（线速度） |
| 左摇杆 ←→ | 左转 / 右转 | 控制 vz（角速度） |
| LT（左扳机） | 向左平移 | 控制 vy（负方向） |
| RT（右扳机） | 向右平移 | 控制 vy（正方向） |

### 机械臂控制（右手）

| 输入 | 功能 | 控制关节 |
|-----|------|---------|
| 右摇杆 ←→ | 基座左右转 | base |
| 右摇杆 ↑↓ | 前伸 / 后缩 | elbow |
| 十字键 ↑ | 上升 | shoulder + |
| 十字键 ↓ | 下降 | shoulder - |
| 十字键 ← | 手腕左转 | wrist_roll + |
| 十字键 → | 手腕右转 | wrist_roll - |
| **Y 键** | 手腕上翻 | wrist_flex + |
| **A 键** | 手腕下翻 | wrist_flex - |
| **B 键** | 手腕一键水平 | wrist_flex = 180° - shoulder - elbow |
| **RB 键** | 夹爪打开 | gripper = 90° |
| **LB 键** | 夹爪关闭 | gripper = 0° |

### 系统控制

| 输入 | 功能 | 说明 |
|-----|------|------|
| **Back 键** | 紧急停止 | 锁定底盘和机械臂，需复位解锁 |
| **Start 键** | 复位 | 停止底盘，机械臂归位，解除急停 |

## 手腕一键水平原理

当按下 **B 键** 时，手腕角度自动计算为：

```
wrist_flex = 180° - shoulder - elbow
```

这样无论机械臂处于什么姿态，末端都能保持水平。

**示例**：
- shoulder = 30°, elbow = 45° → wrist_flex = 105°（末端水平）
- shoulder = 0°, elbow = 90° → wrist_flex = 90°（末端水平）

## 配置参数

在 `configs/config.py` 中修改 `GamepadConfig`：

```python
@dataclass
class GamepadConfig:
    # 底盘速度限制
    max_linear_speed: float = 0.5      # 最大线速度 (m/s)
    max_angular_speed: float = 1.0     # 最大角速度 (rad/s)
    
    # 机械臂步进角度
    arm_base_step: float = 3.0         # 基座步进 (度/帧)
    arm_elbow_step: float = 2.0        # 肘关节步进
    arm_shoulder_step: float = 2.0     # 肩关节步进
    arm_wrist_flex_step: float = 3.0   # 腕屈伸步进
    arm_wrist_roll_step: float = 3.0   # 腕旋转步进
    
    # 夹爪角度
    arm_gripper_open: float = 90.0     # 打开角度
    arm_gripper_close: float = 0.0     # 关闭角度
    
    # 通信地址
    chassis_service_addr: str = "tcp://localhost:5556"
    arm_service_addr: str = "tcp://localhost:5557"
```

## 故障排除

### 手柄无法连接

1. 检查手柄是否已配对（Windows 设置 → 蓝牙和其他设备）
2. 按 Xbox 按钮确认手柄已开机
3. 检查手柄电池电量

### 底盘/机械臂无响应

1. 确认底盘服务和机械臂服务已启动
2. 检查服务地址配置是否正确
3. 查看日志中的连接错误信息

### 摇杆漂移

在配置中调整死区值：

```python
left_stick_deadzone: float = 0.20   # 增大死区
right_stick_deadzone: float = 0.20
```

## 注意事项

1. **安全第一**：首次使用请在开阔地带测试，熟悉控制后再靠近障碍物
2. **紧急停止**：任何异常立即按 Back 键急停
3. **速度控制**：建议先用慢速熟悉，再逐渐提高速度
4. **机械臂限位**：注意机械臂关节角度限制，避免碰撞

## 文件结构

```
applications/gamepad_control/
├── __init__.py          # 包初始化
├── __main__.py          # 启动入口
├── app.py               # 主应用逻辑
└── README.md            # 本文档
```
