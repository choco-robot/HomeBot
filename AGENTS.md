# HomeBot AGENTS.md

本文档面向 AI 编程助手，提供项目背景、架构概览和开发规范。

## 项目概述

HomeBot 是一个面向家庭场景的轻量级机器人项目，采用**分层模块化架构**和 **ZeroMQ** 通信总线，支持手机遥控、语音交互、模仿学习、人体跟随等多种应用。

**核心特性：**
- 纯 Python 实现，跨平台支持（Windows、Linux、macOS、树莓派）
- ZeroMQ 通信总线，低延迟轻量级（~1MB vs ROS2 ~1GB）
- 网页遥控端，支持手机/平板/PC，实时视频流显示
- 紧急停止锁定机制，触发后需手动归位解锁
- 硬件抽象层设计，易于适配不同硬件

## 技术栈

| 组件 | 技术 |
|------|------|
| 语言 | Python 3.11+ |
| 通信 | ZeroMQ (pyzmq) |
| Web 框架 | Flask + Flask-SocketIO |
| 计算机视觉 | OpenCV, Ultralytics YOLO |
| 语音识别 | sherpa-onnx |
| 语音合成 | 火山引擎 TTS |
| LLM对话 | DeepSeek API / OpenAI |
| MCP框架 | fastmcp |
| 前端 | HTML5 + JavaScript (nippleJS 虚拟摇杆) |
| 硬件驱动 | pyserial, ftservo-python-sdk |
| 其他 | numpy, filterpy |

## 项目结构

```
homebot/
├── docs/                      # 中文文档
│   ├── 软件架构与开发规划.md
│   ├── 技术方案选型.md
│   ├── 人体跟随使用指南.md
│   ├── 人体检测与跟随方案.md
│   └── 网页控制端使用指南.md
├── hardware/                  # 硬件设计文件（SolidWorks, STL）
│   └── structure/
├── software/                  # 软件代码
│   ├── src/
│   │   ├── common/            # 公共工具、消息定义、ZeroMQ 辅助
│   │   ├── configs/           # 运行时配置 (config.py)
│   │   ├── applications/      # 应用层
│   │   │   ├── remote_control/    # 网页遥控端 (Flask + WebSocket)
│   │   │   ├── gamepad_control/   # 游戏手柄控制 (Xbox手柄)
│   │   │   ├── human_follow/      # 人体跟随 (YOLO + 视觉伺服)
│   │   │   ├── speech_interaction/# 语音交互
│   │   │   └── imitation_learning/# 模仿学习
│   │   ├── services/          # 服务层
│   │   │   ├── motion_service/    # 运动控制服务
│   │   │   │   ├── chassis_service.py   # 底盘服务（含仲裁器）
│   │   │   │   └── chassis_arbiter/     # 仲裁器核心
│   │   │   ├── vision_service/    # 视觉服务（图像采集发布）
│   │   │   └── speech_service/    # 语音服务
│   │   ├── hal/               # 硬件抽象层
│   │   │   ├── camera/        # 摄像头驱动
│   │   │   ├── chassis/       # 底盘驱动（三轮全向轮）
│   │   │   ├── arm/           # 机械臂驱动
│   │   │   ├── audio/         # 音频驱动
│   │   │   └── ftservo_driver.py  # 飞特舵机底层驱动
│   │   ├── examples/          # 示例代码
│   │   └── tests/             # 测试代码
│   ├── tools/                 # 辅助脚本（模型下载等）
│   ├── models/                # 机器学习模型（YOLO 等）
│   ├── start_system.py        # 跨平台系统启动器
│   ├── start_chassis_service.py
│   ├── start_human_follow.py
│   └── start_system.bat       # Windows 一键启动
├── requirements.txt           # Python 依赖
├── pyproject.toml             # 构建系统配置
├── setup.py                   # 包安装配置
└── README.md                  # 项目说明
```

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                        Application Layer                      │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │
│  │ Remote   │ │ Gamepad  │ │  Human   │ │  Voice   │       │
│  │ Control  │ │ Control  │ │  Follow  │ │Interaction│      │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘       │
│  ┌──────────┐ ┌──────────┐                                   │
│  │Imitation │ │  ...     │                                   │
│  │Learning  │ │          │                                   │
│  └────┬─────┘ └────┬─────┘                                   │
└───────┼────────────┼──────────────────────────────────────────┘
        │            │            │            │
        └────────────┴──────┬─────┴────────────┘
                            │ ZeroMQ
┌───────────────────────────┼─────────────────────────────────┐
│                      Service Layer                            │
│  ┌──────────────┐ ┌──────┴──────┐ ┌──────────────┐          │
│  │ Motion       │ │   Vision     │ │   Speech     │          │
│  │ Service      │ │   Service    │ │   Service    │          │
│  │(Chassis+Arm) │ │(Camera Pub)  │ │              │          │
│  └──────┬───────┘ └──────┬──────┘ └──────┬───────┘          │
└─────────┼────────────────┼───────────────┼──────────────────┘
          │                │               │
          └────────────────┼───────────────┘
                           │
┌──────────────────────────┼──────────────────────────────────┐
│                     HAL (Hardware Abstraction Layer)          │
│  ┌──────────┐ ┌──────────┼──┐ ┌──────────┐ ┌──────────┐     │
│  │ Chassis  │ │   Arm    │  │ │  Camera  │ │  Audio   │     │
│  │  Driver  │ │  Driver  │  │ │  Driver  │ │  Driver  │     │
│  └──────────┘ └──────────┘  │ └──────────┘ └──────────┘     │
│  ┌──────────┐                                             │
│  │ Gamepad  │                                             │
│  │  Driver  │                                             │
│  └──────────┘                                             │
└─────────────────────────────┴─────────────────────────────────┘
```

## 核心模块说明

### 1. 硬件抽象层 (HAL)

**底盘驱动** (`hal/chassis/driver.py`):
- 基于飞特 ST3215 舵机
- 三轮全向底盘（左前、右前、后轮）
- 实现逆运动学：速度 (vx, vy, omega) → 各轮速度
- 支持轮式模式控制

**舵机驱动** (`hal/ftservo_driver.py`):
- 封装 ftservo-python-sdk (scservo_sdk)
- 支持位置模式、速度模式（轮式）
- 自动模拟模式（SDK 未安装时）

**摄像头驱动** (`hal/camera/driver.py`):
- OpenCV 封装
- 支持分辨率、帧率配置

**游戏手柄驱动** (`hal/gamepad/`):
- 基于 Windows XInput API
- 支持 Xbox 360/One/Series X|S 手柄
- 按键、摇杆、扳机键读取
- 震动控制反馈

### 2. 服务层 (Services)

**底盘服务** (`services/motion_service/chassis_service.py`):
- ZeroMQ REP 模式监听控制指令
- **仲裁器核心逻辑**：优先级-based 控制权管理
- 紧急停止锁定机制（触发后需归位解锁）
- 1秒超时自动释放控制权

控制源优先级（从高到低）：
```python
PRIORITIES = {
    "emergency": 4,  # 紧急停止（最高）
    "auto": 3,       # 自动模式（人体跟随）
    "gamepad": 2,    # 游戏手柄控制
    "voice": 2,      # 语音控制（与手柄同级）
    "web": 1,        # 网页遥控（最低）
}
```

**视觉服务** (`services/vision_service/vision.py`):
- ZeroMQ PUB 模式发布图像帧
- JPEG 编码压缩
- 支持 VisionSubscriber 订阅

**语音服务** (`services/speech_service/`):
- **WakeupASR Service** (`wakeup_asr_service.py`): 
  - ZeroMQ PUB 模式发布语音识别结果
  - 持续监听麦克风，检测唤醒词后自动ASR
  - 发布地址: `tcp://*:5571`
- **Voice Engine** (`voice_engine.py`): 语音唤醒、ASR、TTS封装
- **TTS Client** (`tts_client.py`): 火山引擎流式TTS客户端

### 3. 应用层 (Applications)

**网页遥控** (`applications/remote_control/`):
- Flask + SocketIO 实现
- 双虚拟摇杆（nippleJS）：左手底盘、右手机械臂
- MJPEG 视频流显示
- 紧急停止 + 归位按钮

**游戏手柄控制** (`applications/gamepad_control/`):
- Xbox 手柄同时控制底盘和机械臂
- 左摇杆：底盘移动/旋转，扳机键：底盘平移
- 右摇杆：机械臂基座/伸缩，十字键：升降/腕转
- Y/A/B键：手腕控制，RB/LB键：夹爪控制
- Back键：紧急停止，Start键：复位

**人体跟随** (`applications/human_follow/`):
- 检测器 (`detector.py`): YOLO 人体检测
- 跟踪器 (`tracker.py`): IoU-based 多目标跟踪
- 控制器 (`controller.py`): 视觉伺服 PID 控制
- 主应用 (`follow.py`): 整合检测、跟踪、控制、底盘通信

**语音交互** (`applications/speech_interaction/`):
- **Speech App** (`speech_app.py`): SUB模式订阅WakeupASR服务
- **Dialogue Manager** (`dialogue_manager.py`): LLM对话管理，支持工具调用
- **MCP Server** (`mcp_server.py`): 机器人控制工具集
  - `move_forward(distance, speed)`: 前进
  - `move_backward(distance, speed)`: 后退
  - `turn_left(angle, speed)`: 左转
  - `turn_right(angle, speed)`: 右转
  - `stop_robot()`: 停止
  - `get_robot_status()`: 获取状态
- 架构: WakeupASR(PUB) → SpeechApp(SUB) → TTS(本地)

## 配置管理

所有配置集中在 `software/src/configs/config.py`，使用 dataclass 定义：

```python
@dataclass
class ChassisConfig:
    serial_port: str = "/dev/tty.usbmodem5AE60527771"
    baudrate: int = 1000000
    left_front_id: int = 7
    right_front_id: int = 9
    rear_id: int = 8
    max_linear_speed: float = 0.5    # m/s
    max_angular_speed: float = 1.0   # rad/s
    service_addr: str = "tcp://127.0.0.1:5556"
```

全局配置访问方式：
```python
from configs import get_config
config = get_config()
print(config.chassis.serial_port)
```

## 构建与运行

### 安装依赖

```bash
# 在项目根目录
python -m pip install -e .
```

或手动安装：
```bash
pip install -r requirements.txt
```

### 启动方式

**方式一：一键启动（推荐）**
```bash
cd software
python start_system.py
```

**方式二：手动启动（分终端）**

终端 1 - 底盘服务：
```bash
cd software/src
python -m services.motion_service.chassis_service
# 或指定串口：python -m services.motion_service.chassis_service --port COM3
```

终端 2 - 视觉服务：
```bash
cd software/src
python -m services.vision_service
# 或带显示：python -m services.vision_service --display
```

终端 3 - Web 控制端：
```bash
cd software/src
python -m applications.remote_control
# 或指定参数：python -m applications.remote_control --host 0.0.0.0 --port 5000
```

终端 4 - 人体跟随（可选）：
```bash
cd software/src
python -m applications.human_follow
# 或带显示：python -m applications.human_follow --display
```

终端 5 - 游戏手柄控制（可选）：
```bash
cd software/src
python -m applications.gamepad_control
# 或指定手柄：python -m applications.gamepad_control --controller 0
```

终端 6 - 语音交互服务（可选）：
```bash
cd software/src
# 先启动 Wakeup+ASR 服务（PUB模式）
python -m services.speech_service wakeup

# 再启动语音交互应用（SUB模式，新终端）
python -m applications.speech_interaction
```

或者使用一键启动脚本：
```bash
cd software
python start_speech_service.py
```

检查模型文件：
```bash
cd software
python start_speech_service.py --check-models
```

下载语音模型（首次使用）：
```bash
cd software
python tools/download_speech_models.py
```

### 访问控制界面

浏览器访问：`http://<robot-ip>:5000`

界面功能：
- 实时视频流（摄像头画面）
- 虚拟摇杆（左侧控制底盘移动）
- 紧急停止按钮（红色，触发后锁定底盘）
- 归位按钮（蓝色，解锁紧急停止）

## 开发规范

### 代码风格

- 使用中文注释和文档字符串
- 类型注解推荐使用（`from typing import ...`）
- 日志使用 `common.logging.get_logger(__name__)`
- 配置通过 `configs.config.get_config()` 访问

### 消息格式

底盘控制命令：
```python
{
    "source": "web",        # 控制源: web/voice/auto/emergency/home
    "vx": 0.5,              # 线速度 X (m/s)
    "vy": 0.0,              # 线速度 Y (m/s)
    "vz": 0.3,              # 角速度 Z (rad/s)
    "priority": 1           # 优先级: 1=web, 2=voice, 3=auto, 4=emergency
}
```

### 服务启动模式

服务使用 `python -m 模块名` 方式启动：
```bash
python -m services.motion_service.chassis_service
python -m services.vision_service
python -m applications.remote_control
python -m applications.human_follow
```

每个模块应包含 `__main__.py` 作为入口点。

## 测试

测试文件位于 `software/src/tests/`：

```bash
cd software/src
python -m tests.test_zmq
python -m tests.test_human_follow
python -m tests.test_web_control
```

## 安全注意事项

1. **紧急停止机制**：点击紧急停止后，底盘进入锁定状态，拒绝所有运动命令，必须通过归位按钮解锁
2. **超时保护**：底盘服务 1 秒未收到指令自动停止
3. **速度限制**：配置中设置最大线速度和角速度，代码中强制限制
4. **串口权限**：Linux 下需要确保用户有串口访问权限（`dialout` 组）

## 常见问题

**串口连接失败：**
- Windows: 检查设备管理器中的 COM 端口号
- Linux: 检查 `/dev/ttyUSB0` 或 `/dev/ttyACM0` 权限
- macOS: 检查 `/dev/tty.usbmodem*` 设备

**端口被占用：**
运行 `start_system.py` 时会自动检测端口占用，可选择终止占用进程。

**模型下载：**
首次运行人体跟随时会自动下载 YOLO 模型，或手动运行：
```bash
python software/tools/download_models.py
```

## 扩展开发

添加新应用模块的步骤：
1. 在 `applications/` 下创建新目录
2. 添加 `__init__.py` 和 `__main__.py`
3. 使用 `ChassisArbiterClient` 与底盘服务通信
4. 使用 `VisionSubscriber` 订阅图像流

---

*最后更新：2026-03-17*
