---
name: homebot
description: HomeBot 完整机器人控制器技能，集成了机器人底盘运动控制、机械臂关节控制、摄像头视觉画面捕获三个子功能，全部基于 ZeroMQ 局域网通信协议，完美匹配 HomeBot 项目服务端架构。
version: 1.0.0
metadata:
  openclaw:
    requires:
      bins:
      - python
      - pip
    install:
    - kind: pip
      requirements_file: requirements.txt
    emoji: "🤖"
---

# HomeBot Robot Controller

HomeBot 完整机器人控制器技能，集成三大功能模块：
- 🚗 **底盘控制**：前进/后退/转向，精确距离角度控制
- 🦾 **机械臂控制**：6自由度关节控制，夹爪控制，回原点
- 👁️ **视觉查询**：一键捕获机器人摄像头最新画面，回答"机器人看到了什么"

全部基于 ZeroMQ REQ-REP / PUB 局域网通信协议，完美匹配 HomeBot 项目服务端架构。

## 模块说明

| 模块 | 功能 | 默认端口 | 源文件 |
|------|------|----------|--------|
| `chassis` | 底盘运动控制 | 5556 | `chassis_control.py` |
| `arm` | 机械臂关节控制 | 5557 | `arm_control.py` |
| `vision` | 摄像头画面捕获 | 5560 | `video_subscriber.py` / `what_does_robot_see_workflow.py` |

## 配置

编辑 `scripts/robot_config.py` 修改机器人IP和默认端口配置：

```python
# 机器人IP地址
ROBOT_IP = "192.168.0.12"

# 各模块默认端口（与HomeBot服务端配置一致）
CHASSIS_PORT = 5556
ARM_PORT = 5557
VIDEO_PORT = 5560

# 默认参数
DEFAULT_SPEED = 0.3              # 默认线速度 m/s
DEFAULT_ANGULAR_SPEED = 0.5      # 默认角速度 rad/s
DEFAULT_ARM_PRIORITY = 2         # 默认机械臂控制优先级 (1=web, 2=voice, 3=auto, 4=emergency)
CAPTURE_TIMEOUT = 10             # 图像捕获超时（秒）
OUTPUT_DIR = "."                 # 图像保存目录
```

## 使用方法

### 环境安装

```bash
pip install -r skills/homebot/requirements.txt
```

---

## 1. 底盘控制 (Chassis)

精确控制机器人底盘运动，支持指定距离前进后退，指定角度左转右转，实时速度控制。

### 命令行使用

```bash
# 前进指定距离（厘米）
python skills/homebot/scripts/chassis_control.py forward 10

# 后退指定距离（厘米）
python skills/homebot/scripts/chassis_control.py backward 20

# 右转指定角度（度）
python skills/homebot/scripts/chassis_control.py right 90

# 左转指定角度（度）
python skills/homebot/scripts/chassis_control.py left 45

# 设置速度（线速度 m/s, 角速度 rad/s）
python skills/homebot/scripts/chassis_control.py velocity 0.2 0.0

# 紧急停止
python skills/homebot/scripts/chassis_control.py stop

# 交互式控制（w/a/s/d 键盘控制）
python skills/homebot/scripts/chassis_control.py interactive
```

### Python API

```python
from chassis_control import HomeBotController

bot = HomeBotController("192.168.0.12", 5556)
bot.connect()

# 前进 10cm
bot.forward_cm(10)

# 右转 90度
bot.right_deg(90)

# 停止
bot.stop()

bot.close()
```

---

## 2. 机械臂控制 (Arm)

6自由度机械臂控制，支持单个/多个关节角度设置，夹爪控制，回原点，优先级仲裁。

### 关节命名

| 关节名 | 说明 | 默认舵机ID |
|--------|------|-----------|
| `base` | 基座旋转 | 1 |
| `shoulder` | 肩关节 | 2 |
| `elbow` | 肘关节 | 3 |
| `wrist_flex` | 腕关节俯仰 | 4 |
| `wrist_roll` | 腕关节旋转 | 5 |
| `gripper` | 夹爪 | 6 |

### 优先级

| 优先级 | 控制源 | 说明 |
|--------|--------|------|
| 4 | emergency | 紧急停止（最高） |
| 3 | auto | 自动控制 |
| 2 | voice | 语音控制（默认） |
| 1 | web | 网页遥控 |

### 命令行使用

```bash
# 设置单个关节角度（度）
python skills/homebot/scripts/arm_control.py joint base 0

# 同时设置多个关节角度
python skills/homebot/scripts/arm_control.py joints "base:0,shoulder:10,elbow:45"

# 打开夹爪（90度）
python skills/homebot/scripts/arm_control.py gripper open

# 关闭夹爪（0度）
python skills/homebot/scripts/arm_control.py gripper close

# 设置夹爪角度（0-90度）
python skills/homebot/scripts/arm_control.py gripper 45

# 回原点（休息位置，由服务端配置）
python skills/homebot/scripts/arm_control.py home

# 获取当前所有关节角度
python skills/homebot/scripts/arm_control.py status

# 紧急停止
python skills/homebot/scripts/arm_control.py stop
```

### Python API

```python
from arm_control import HomeBotArmController

arm = HomeBotArmController(robot_ip="192.168.0.12", robot_port=5557)

# 设置单个关节
resp = arm.set_joint_angle("shoulder", 30.0)

# 同时设置多个关节
resp = arm.set_joint_angles({
    "base": 0,
    "shoulder": 20,
    "elbow": 45,
    "wrist_flex": 0
})

# 打开夹爪
resp = arm.open_gripper()

# 回原点
resp = arm.move_home()

arm.close()
```

---

## 3. 视觉查询 (Vision)

一键捕获机器人摄像头最新画面，工作流：获取最新帧 → 保存图片 → 返回文件路径。当用户问"机器人看到了什么"自动触发。

### 一键完整工作流

```bash
python skills/homebot/scripts/what_does_robot_see_workflow.py
```

输出：保存的JPEG图像文件路径（带时间戳命名）

### 视频订阅工具

```bash
# 获取单张图像
python skills/homebot/scripts/video_subscriber.py --ip 192.168.0.12 --port 5560

# 持续接收所有帧并保存到目录
python skills/homebot/scripts/video_subscriber.py --ip 192.168.0.12 --port 5560 --keep-receiving --output-dir ./frames
```

### 工作流集成

Picoclaw 自动触发：
- 用户提问："机器人看到了什么" / "what does the robot see"
- 自动执行捕获工作流
- 直接发送图片给用户

### Python API

```python
from what_does_robot_see_workflow import WhatDoesRobotSeeWorkflow

workflow = WhatDoesRobotSeeWorkflow()
image_path = workflow.capture()

if image_path:
    print(f"Image saved to: {image_path}")
```

---

## 通信协议

全部基于 ZeroMQ 协议，完全匹配 HomeBot 项目服务端配置：

| 服务 | 模式 | 默认端口 |
|------|------|----------|
| 底盘控制 | REQ-REP | 5556 |
| 机械臂控制 | REQ-REP | 5557 |
| 视频发布 | PUB | 5560 |

HomeBot 服务端配置示例：
```python
# config.py
class ZMQConfig:
    chassis_service_addr: str = "tcp://*:5556"
    arm_service_addr: str = "tcp://*:5557"
    vision_pub_addr: str = "tcp://*:5560"
```

## 依赖

- Python 3.x
- pyzmq >= 25.0.0
- Pillow >= 9.0.0

## 示例

- `scripts/dance.py` - 机械臂舞蹈动作示例
