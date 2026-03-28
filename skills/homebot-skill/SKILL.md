---
name: homebot
description: HomeBot 完整机器人控制器技能，集成底盘运动控制、机械臂关节控制、升降平台控制、摄像头视觉画面捕获与AI分析，全部基于 ZeroMQ 局域网通信协议。
version: 1.1.0
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

HomeBot 完整机器人控制器技能，集成功能模块：
- 🚗 **底盘控制**：前进/后退/转向，精确距离角度控制
- 🦾 **机械臂控制**：6自由度关节控制，夹爪控制，回原点
- ⬆️ **升降平台**：平台上升/下降控制
- 👁️ **视觉查询**：一键捕获机器人摄像头画面，**自动调用火山引擎 LLM 分析图像内容**

全部基于 ZeroMQ REQ-REP / PUB 局域网通信协议，完美匹配 HomeBot 项目服务端架构。

## 快速开始

### 1. 配置机器人连接

**方式一：环境变量（推荐，适合 OpenClaw 等 Agent）**

```bash
# Windows
set HOMEBOT_IP=192.168.1.13
set ARK_API_KEY=your_volcengine_api_key

# Linux/Mac
export HOMEBOT_IP=192.168.1.13
export ARK_API_KEY=your_volcengine_api_key
```

**方式二：修改配置文件**

编辑 `scripts/robot_config.py` 修改机器人 IP：
```python
ROBOT_IP = "192.168.1.13"  # 修改为你的机器人IP
```

### 2. 安装依赖

```bash
pip install -r requirements.txt
```

### 3. 测试连接

```bash
# 测试底盘连接
python scripts/chassis_control.py forward 10

# 测试机械臂连接
python scripts/arm_control.py status

# 测试升降平台
python scripts/arm_control.py lift up
python scripts/arm_control.py lift down

# 测试视觉捕获
python scripts/what_does_robot_see_workflow.py --no-analysis
```

---

## 环境变量配置

所有配置都可通过环境变量设置，优先级：**环境变量 > 配置文件默认值**

| 环境变量 | 说明 | 默认值 |
|---------|------|--------|
| `HOMEBOT_IP` | 机器人 IP 地址 | `192.168.1.13` |
| `HOMEBOT_CHASSIS_PORT` | 底盘服务端口 | `5556` |
| `HOMEBOT_ARM_PORT` | 机械臂服务端口 | `5557` |
| `HOMEBOT_VIDEO_PORT` | 视频流端口 | `5560` |
| `HOMEBOT_CAPTURE_TIMEOUT` | 图像捕获超时(秒) | `10.0` |
| `HOMEBOT_OUTPUT_DIR` | 图像保存目录 | `.` |
| `ARK_API_KEY` | 火山引擎 API Key（视觉分析） | - |
| `ARK_MODEL_ID` | 火山引擎模型 ID | `doubao-seed-2-0-lite-260215` |

### OpenClaw 配置示例

在 OpenClaw 配置中通过 `env` 设置环境变量：

```yaml
mcp:
  servers:
    homebot:
      command: "python"
      args: 
        - "{{skill_path}}/mcp_homebot_server.py"
      env:
        # 机器人连接配置
        HOMEBOT_IP: "192.168.1.13"
        HOMEBOT_CHASSIS_PORT: "5556"
        HOMEBOT_ARM_PORT: "5557"
        HOMEBOT_VIDEO_PORT: "5560"
        
        # 火山引擎视觉分析配置（可选）
        ARK_API_KEY: "your-api-key-here"
        ARK_MODEL_ID: "doubao-seed-2-0-lite-260215"
```

---

## 模块说明

| 模块 | 功能 | 默认端口 | 源文件 |
|------|------|----------|--------|
| `chassis` | 底盘运动控制 | 5556 | `chassis_control.py` |
| `arm` | 机械臂关节控制 | 5557 | `arm_control.py` |
| `lift` | 升降平台控制 | 5557 | `arm_control.py` |
| `vision` | 摄像头画面捕获+AI分析 | 5560 | `video_subscriber.py` / `what_does_robot_see_workflow.py` |
| `gestures` | 机械臂姿态动作（挥手/点头/摇头） | - | `arm_gestures.py` |

---

## 1. 底盘控制 (Chassis)

精确控制机器人底盘运动，支持指定距离前进后退，指定角度左转右转，实时速度控制。

### 命令行使用

```bash
# 前进指定距离（厘米）
python scripts/chassis_control.py forward 10

# 后退指定距离（厘米）
python scripts/chassis_control.py backward 20

# 右转指定角度（度）
python scripts/chassis_control.py right 90

# 左转指定角度（度）
python scripts/chassis_control.py left 45

# 设置速度（线速度 m/s, 角速度 rad/s）
python scripts/chassis_control.py velocity 0.2 0.0

# 紧急停止
python scripts/chassis_control.py stop

# 交互式控制（w/a/s/d 键盘控制）
python scripts/chassis_control.py interactive
```

### Python API

```python
from scripts.chassis_control import HomeBotChassisController
from scripts.robot_config import ROBOT_IP, CHASSIS_PORT

bot = HomeBotChassisController(ip=ROBOT_IP, port=CHASSIS_PORT)

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
python scripts/arm_control.py joint base 0

# 同时设置多个关节角度
python scripts/arm_control.py joints "base:0,shoulder:10,elbow:45"

# 打开夹爪（90度）
python scripts/arm_control.py gripper open

# 关闭夹爪（0度）
python scripts/arm_control.py gripper close

# 设置夹爪角度（0-90度）
python scripts/arm_control.py gripper 45

# 回原点（休息位置，由服务端配置）
python scripts/arm_control.py home

# 获取当前所有关节角度
python scripts/arm_control.py status

# 紧急停止
python scripts/arm_control.py stop

# 升降平台上升（默认20mm）
python scripts/arm_control.py lift up
python scripts/arm_control.py lift up --step 30

# 升降平台下降（默认20mm）
python scripts/arm_control.py lift down
python scripts/arm_control.py lift down --step 30
```

### Python API

```python
from scripts.arm_control import HomeBotArmController
from scripts.robot_config import ROBOT_IP, ARM_PORT

arm = HomeBotArmController(robot_ip=ROBOT_IP, robot_port=ARM_PORT)

# 设置单个关节
resp = arm.set_joint_angle("shoulder", 30.0)

# 同时设置多个关节
resp = arm.set_joint_angles({
    "base": 0,
    "shoulder": 20,
    "elbow": 45
})

# 打开夹爪
resp = arm.open_gripper()

# 回原点
resp = arm.move_home()

# 升降平台控制
resp = arm.lift_up(step=20.0)      # 上升
resp = arm.lift_down(step=20.0)    # 下降

arm.close()
```

---

## 3. 升降平台控制 (Lift Platform)

升降平台是机械臂的附加模块，通过机械臂服务控制。高度范围通常为 -200mm(最低) 到 0mm(最高)。

### 控制说明

| 方向 | direction 参数 | 高度变化 | 说明 |
|------|---------------|----------|------|
| 上升 | 1 | 增加（向0靠近）| 向最高点移动 |
| 下降 | -1 | 减少（向负值）| 向最低点移动 |

### 命令行使用

```bash
# 升降平台上升（默认20mm）
python scripts/arm_control.py lift up

# 升降平台上升指定步长
python scripts/arm_control.py lift up --step 50

# 升降平台下降（默认20mm）
python scripts/arm_control.py lift down

# 升降平台下降指定步长
python scripts/arm_control.py lift down --step 50
```

### Python API

```python
from scripts.arm_control import HomeBotArmController

arm = HomeBotArmController(robot_ip="192.168.1.13", robot_port=5557)

# 上升 20mm
result = arm.lift_up(step=20.0)

# 下降 30mm
result = arm.lift_down(step=30.0)

# 自定义控制
result = arm.move_lift(direction=1, step=25.0)   # 上升25mm
result = arm.move_lift(direction=-1, step=25.0)  # 下降25mm

arm.close()
```

---

## 4. 视觉查询 (Vision)

一键捕获机器人摄像头最新画面，**集成火山引擎 LLM 自动分析图像内容**。

### 前置要求

视觉分析功能需要配置火山引擎 API Key：

```bash
# 设置环境变量
export ARK_API_KEY="your-api-key-here"
export ARK_MODEL_ID="doubao-seed-2-0-lite-260215"  # 可选
```

### 一键完整工作流（捕获 + 分析）

```bash
# 捕获图像并自动分析
python scripts/what_does_robot_see_workflow.py

# 仅捕获图像，不进行分析
python scripts/what_does_robot_see_workflow.py --no-analysis

# 使用自定义提示词分析
python scripts/what_does_robot_see_workflow.py --prompt "图中有几个人？他们在做什么？"

# 指定不同模型
python scripts/what_does_robot_see_workflow.py --model doubao-vision-pro-250226
```

**输出示例：**
```
[INFO] 正在连接机器人 192.168.1.13:5560...
[OK] 图像捕获成功
[INFO] 保存位置: C:\...\homebot_capture_20260319_154530.jpg
[INFO] 文件大小: 45231 字节
[INFO] 正在使用火山引擎分析图片...
[INFO] 模型: doubao-vision-lite-250225
```

### 视频订阅工具

```bash
# 获取单张图像
python scripts/video_subscriber.py --ip 192.168.1.13 --port 5560

# 持续接收所有帧并保存到目录
python scripts/video_subscriber.py --ip 192.168.1.13 --port 5560 --keep-receiving --output-dir ./frames
```

### Python API

```python
from scripts.what_does_robot_see_workflow import WhatDoesRobotSeeWorkflow

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

## MCP 服务器支持 🚀

本技能内置 **Model Context Protocol (MCP)** 服务器，可直接配置给 OpenClaw/LLM 调用，让 AI 自动操控机器人！

### 功能封装

MCP 服务器封装了以下 11 个工具：

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
| `lift_platform_up` | 升降平台上升（可选步长，默认20mm） |
| `lift_platform_down` | 升降平台下降（可选步长，默认20mm） |
| `robot_what_does_robot_see` | 捕获机器人画面并 AI 分析场景 |

### MCP 配置方法

在 OpenClaw 配置文件 `config.yaml` 中添加：

```yaml
mcp:
  servers:
    homebot:
      command: "python"
      args: 
        - "{{skill_path}}/mcp_homebot_server.py"
      env:
        # === 机器人连接配置（必填）===
        HOMEBOT_IP: "192.168.1.13"
        
        # === 火山引擎视觉分析配置（可选，用于 robot_what_does_robot_see 功能）===
        ARK_API_KEY: "your-volcengine-api-key"
        ARK_MODEL_ID: "doubao-seed-2-0-lite-260215"
```

> **注意**: `{{skill_path}}` 是 OpenClaw 的变量，会自动替换为技能的实际路径。

### 依赖安装

```bash
pip install mcp
```

### 使用效果

配置完成后，LLM 即可**直接调用所有机器人控制工具**，自动完成：
- 根据自然语言指令控制机器人移动
- 调整机械臂位置
- 控制升降平台高度
- 让机器人自动观察环境并报告场景

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

---

## 依赖

- Python 3.8+
- pyzmq >= 25.0.0
- Pillow >= 9.0.0
- volcenginesdkarkruntime >= 1.0.0（视觉分析功能需要）
- mcp >= 1.0.0（MCP 服务器需要）

---

## 示例脚本

### 机械臂舞蹈

```bash
python scripts/dance.py
```

### 机械臂姿态动作（挥手、点头、摇头）

```bash
# 挥挥手
python scripts/arm_gestures.py wave
python scripts/arm_gestures.py wave --times 5  # 挥手5次

# 点点头
python scripts/arm_gestures.py nod

# 摇摇头
python scripts/arm_gestures.py shake

# 依次执行全部动作
python scripts/arm_gestures.py all
```

---

## 故障排除

### 连接超时

检查机器人 IP 是否正确，机器人服务是否已启动：
```bash
# 测试网络连通性
ping 192.168.1.13
```

### 视觉分析失败

检查 ARK_API_KEY 是否已正确设置：
```bash
# Windows
echo %ARK_API_KEY%

# Linux/Mac
echo $ARK_API_KEY
```

### 端口冲突

检查端口是否被占用：
```bash
# Windows
netstat -ano | findstr 5556

# Linux/Mac
lsof -i :5556
```

### 升降平台无响应

1. 确认机械臂服务已启动并支持升降平台功能
2. 检查机械臂服务日志是否有错误信息
3. 确认升降平台舵机连接正常
