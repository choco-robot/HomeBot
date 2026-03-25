---
name: homebot
description: HomeBot 完整机器人控制器技能，集成了机器人底盘运动控制、机械臂关节控制、摄像头视觉画面捕获三个子功能，全部基于 ZeroMQ 局域网通信协议，完美匹配 HomeBot 项目服务端架构。
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
- 👁️ **视觉查询**：一键捕获机器人摄像头画面，**自动调用火山引擎 LLM 分析图像内容**

全部基于 ZeroMQ REQ-REP / PUB 局域网通信协议，完美匹配 HomeBot 项目服务端架构。

> **🆕 新增功能**：视觉查询现已集成 [volcengine-vision](../volcengine-vision/) 技能，捕获图像后可自动调用火山引擎视觉大模型进行内容理解和描述。

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

一键捕获机器人摄像头最新画面，**集成火山引擎 LLM 自动分析图像内容**。工作流：获取最新帧 → 保存图片 → **调用火山引擎视觉模型分析** → 返回图片路径和分析结果。

### 一键完整工作流（捕获 + 分析）

```bash
# 捕获图像并自动分析
python skills/homebot/scripts/what_does_robot_see_workflow.py

# 仅捕获图像，不进行分析
python skills/homebot/scripts/what_does_robot_see_workflow.py --no-analysis

# 使用自定义提示词分析
python skills/homebot/scripts/what_does_robot_see_workflow.py --prompt "图中有几个人？他们在做什么？"

# 指定不同模型
python skills/homebot/scripts/what_does_robot_see_workflow.py --model doubao-vision-pro-250226
```

**输出示例：**
```
[INFO] 正在连接机器人 192.168.0.12:5560...
[OK] 图像捕获成功
[INFO] 保存位置: C:\...\homebot_capture_20260319_154530.jpg
[INFO] 文件大小: 45231 字节
[INFO] 正在使用火山引擎分析图片...
[INFO] 模型: doubao-vision-lite-250225

==================================================
图像路径: C:\...\homebot_capture_20260319_154530.jpg

==================================================
视觉分析结果:
==================================================
这张图片展示了一个室内场景，主要物体包括...

==================================================

--- RESULT ---
C:\...\homebot_capture_20260319_154530.jpg
```

### 视频订阅工具（仅捕获）

```bash
# 获取单张图像
python skills/homebot/scripts/video_subscriber.py --ip 192.168.0.12 --port 5560

# 持续接收所有帧并保存到目录
python skills/homebot/scripts/video_subscriber.py --ip 192.168.0.12 --port 5560 --keep-receiving --output-dir ./frames
```

### Python API

```python
from what_does_robot_see_workflow import WhatDoesRobotSeeWorkflow

# 完整工作流：捕获 + 分析
workflow = WhatDoesRobotSeeWorkflow(
    enable_analysis=True,
    prompt="描述图片中的主要物体",
    model="doubao-vision-lite-250225"
)

result = workflow.capture_and_analyze()
if result["success"]:
    print(f"图像路径: {result['image_path']}")
    print(f"分析结果: {result['analysis']}")

# 仅捕获图像
workflow = WhatDoesRobotSeeWorkflow(enable_analysis=False)
image_path = workflow.capture()

# 单独分析已有图片
analysis = workflow.analyze("path/to/image.jpg")
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
- volcenginesdkarkruntime >= 1.0.0（视觉分析功能需要）

## 示例

- `scripts/dance.py` - 机械臂舞蹈动作示例


## MCP 服务器支持 🚀

本技能现已内置 **Model Context Protocol (MCP)** 服务器，可直接配置给 Picoclaw/LLM 调用，让 AI 自动操控机器人！

### 功能封装

MCP 服务器封装了以下 9 个工具：

| 工具名称 | 功能描述 |
|---------|---------|
| `chassis_forward` | 机器人前进指定距离（厘米） |
| `chassis_backward` | 机器人后退指定距离（厘米） |
| `chassis_left` | 机器人左转指定角度（度数） |
| `chassis_right` | 机器人右转指定角度（度数） |
| `chassis_stop` | 紧急停止机器人底盘 |
| `arm_move_joint` | 移动机械臂指定关节到目标角度 |
| `arm_get_positions` | 获取机械臂所有关节当前位置 |
| `arm_stop` | 停止机械臂所有运动 |
| `robot_what_does_robot_see` | 捕获机器人画面并 AI 分析场景 |

### MCP 配置方法

在 Picoclaw 主配置文件 `config.yaml` 中添加：

```yaml
mcp:
  servers:
    homebot:
      command: "python"
      args: ["C:/Users/Administrator/.picoclaw/workspace/skills/homebot/mcp_homebot_server.py"]
```

### 依赖安装

安装 MCP 依赖：
```bash
pip install mcp
```

### 使用效果

配置完成后，LLM 即可**直接调用所有机器人控制工具**，自动完成：
- 根据自然语言指令控制机器人移动
- 调整机械臂位置
- 让机器人自动观察环境并报告场景

