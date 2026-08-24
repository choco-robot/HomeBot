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
| Python 环境 | `venv/` 目录下的虚拟环境 |
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
│   │   │   ├── message_bus/       # 通用消息总线（XPUB-XSUB 代理，5590/5591）
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
│   │   ├── homebot_cli/       # 统一命令行工具（homebot 命令：start/stop/status/topic/move/doctor）
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
- Windows: 基于原生 XInput API，性能最佳
- Linux: 基于原生 `/dev/input/js*` joystick 接口，无需额外依赖
- macOS / 其他平台: 使用 pygame 作为跨平台回退
- 支持 Xbox 360/One/Series X|S 手柄
- 按键、摇杆、扳机键读取
- 震动控制反馈（Windows 原生支持；Linux/macOS 视驱动能力而定）

### 2. 服务层 (Services)

**通用消息总线** (`services/message_bus/`):
- XPUB-XSUB 代理（broker），为自定义消息提供统一的发布/订阅通道
- XSUB bind `tcp://*:5590`（发布者 connect 到此），XPUB bind `tcp://*:5591`（订阅者 connect 到此）
- 消息为 ZMQ multipart `[topic, json_payload]`，信封格式 `{"type", "data", "timestamp"}`
- 新增消息通道无需申请端口；用户自定义消息约定使用 `user.*` / `ext.*` 前缀
- 客户端封装见 `common/bus.py`：
  ```python
  from common.bus import BusPublisher, BusSubscriber

  pub = BusPublisher()
  pub.publish("user.temperature", {"value": 25.6})   # 发布自定义消息

  sub = BusSubscriber()
  sub.on_message("user.", lambda msg: print(msg))    # 按前缀订阅
  sub.start()
  ```
- `common/bus.py` 另提供 `ZMQRequestClient`：带"超时自动重建 socket"的通用 REQ 客户端
- 注意：视频流等大流量数据不走总线，继续用 vision_service 专用通道（5560）

**底盘服务** (`services/motion_service/chassis_service.py`):
- ZeroMQ REP 模式监听控制指令
- **仲裁器核心逻辑**：优先级-based 控制权管理
- 紧急停止锁定机制（触发后需归位解锁）
- 1秒超时自动释放控制权

控制源优先级（从高到低）：
```python
PRIORITIES = {
    "emergency": 4,  # 紧急停止（最高）
    "teleop": 3,     # 机械臂 WLAN 遥操作
    "gamepad": 3,    # 游戏手柄控制
    "auto": 3,       # 自动模式（人体跟随）
    "voice": 2,      # 语音控制
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

**机械臂 WLAN 遥操作** (`applications/arm_teleop/`):
- 主臂读取 (`master_reader.py`): 通过 HAL `ArmDriver` 读取本地 SO101 主臂角度，默认可关闭扭矩以手动拖动
- 从臂客户端 (`slave_client.py`): 通过 ZeroMQ REQ-REP 连接远端 `arm_service`
- 遥操作核心 (`app.py`): 关节映射/限幅/死区、速度自适应、通信失败保护、开关控制、运行模式（遥操作/录制/回放）
- 轨迹录制回放 (`recorder.py`): 录制主臂角度序列为 JSON，按时间戳回放，支持速度缩放与循环
- 键盘热键 (`keyboard_input.py`): 运行时 `r` 录制、`p` 回放默认轨迹、`s` 停止回放、`e` 开关遥操作、`q` 退出
- 图形界面 (`gui.py`): 基于 tkinter 的简易 GUI，支持参数设置、遥操作开关、录制/回放、实时日志
- 入口 (`__main__.py`): CLI 参数指定从端地址、使能开关、扭矩模式、录制/回放文件、速度、循环等，或 `--gui` 启动 GUI
- 典型用法：
  ```bash
  cd software/src
  python -m applications.arm_teleop --enable --slave-addr tcp://192.168.x.x:5557
  python -m applications.arm_teleop --enable --record trajectories/demo.json
  python -m applications.arm_teleop --playback trajectories/demo.json --playback-speed 0.5 --loop 3
  python -m applications.arm_teleop --gui
  ```

**语音交互** (`applications/speech_interaction`):
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

### API 密钥管理

**重要：API 密钥不存储在代码中！**

使用环境变量或 `.env.local` 文件管理敏感配置：

1. **复制模板文件**：
   ```bash
   cd software
   cp .env.example .env.local
   ```

2. **编辑 `.env.local`**，填入你的密钥：
   ```ini
   # 火山引擎 TTS
   VOLCANO_APPID=your_appid
   VOLCANO_ACCESS_TOKEN=your_token
   
   # 火山Ark LLM
   ARK_API_KEY=your_api_key
   ARK_MODEL_ID=ep-your_model_id
   ```

3. **验证配置**：
   ```bash
   python tools/check_config.py
   ```

**支持的密钥类型**：
| 服务 | 环境变量 | 说明 |
|------|---------|------|
| 火山引擎 TTS | `VOLCANO_APPID`, `VOLCANO_ACCESS_TOKEN` | 语音合成 |
| 火山Ark LLM | `ARK_API_KEY`, `ARK_MODEL_ID` | 大语言模型（语音交互）|
| 图片理解 | `VISION_API_KEY` | 视觉分析（可选） |

**在代码中使用**：
```python
from configs import get_config, require_secrets

# 强制检查密钥（未配置时自动退出并提示）
require_secrets("tts")

# 获取配置
config = get_config()
api_key = config.llm.api_key  # 自动从环境变量加载
```

**安全注意事项**：
- `.env.local` 已添加到 `.gitignore`，不会提交到版本控制
- 永远不要硬编码密钥到代码中
- 定期轮换 API Key

## 构建与运行

### Python 环境

**项目使用 `venv/` 目录下的虚拟环境，所有 Python 命令应通过该环境执行：**

```bash
# Windows
venv\Scripts\python.exe -m pip install -e .
venv\Scripts\python.exe start_system.py

# Linux / macOS / 树莓派
venv/bin/python -m pip install -e .
venv/bin/python start_system.py
```

> 注意：不要直接使用系统默认的 `python` 命令，确保始终使用 `venv` 中的 Python 解释器。

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

**方式零：homebot CLI（推荐，统一管理入口）**

editable 安装后可直接使用 `homebot` 命令，或 `python -m homebot_cli`：

```bash
# 服务管理（后台运行，日志在 software/logs/<服务名>.log）
homebot start                 # 启动核心服务: bus motion vision web
homebot start speech          # 启动指定服务（可多个）
homebot stop motion           # 停止服务
homebot restart bus
homebot status                # 查看各服务进程/端口状态
homebot logs vision -f        # 查看/跟踪服务日志

# 功能调用与调试
homebot move --vx 0.2 -d 2                        # 底盘前进 2 秒后自动停止
homebot topic echo user.                          # 订阅消息总线（类似 ros topic echo）
homebot topic pub user.test '{"value": 1}'        # 向总线发布消息
homebot doctor                                    # 环境检查（配置/密钥/串口/模型/端口）
```

**Tab 补全**（click 内置，支持 bash/zsh/fish；Windows 下推荐 Git Bash）：

```bash
eval "$(_HOMEBOT_COMPLETE=bash_source homebot)"   # bash / Git Bash，可写入 ~/.bashrc
homebot completion --shell zsh                    # 查看其他 shell 的激活方法
```

注意：PowerShell / cmd 不支持 click 内置补全。

**方式一：一键启动（窗口式）**
```bash
cd software
..\venv\Scripts\python.exe start_system.py    # Windows
# 或
../venv/bin/python start_system.py             # Linux/macOS
```

**方式二：手动启动（分终端）**

终端 1 - 底盘服务：
```bash
cd software/src
..\..\venv\Scripts\python.exe -m services.motion_service.chassis_service
# 或指定串口：..\..\venv\Scripts\python.exe -m services.motion_service.chassis_service --port COM3
```

终端 2 - 视觉服务：
```bash
cd software/src
..\..\venv\Scripts\python.exe -m services.vision_service
# 或带显示：..\..\venv\Scripts\python.exe -m services.vision_service --display
```

终端 3 - Web 控制端：
```bash
cd software/src
..\..\venv\Scripts\python.exe -m applications.remote_control
# 或指定参数：..\..\venv\Scripts\python.exe -m applications.remote_control --host 0.0.0.0 --port 5000
```

终端 4 - 人体跟随（可选）：
```bash
cd software/src
..\..\venv\Scripts\python.exe -m applications.human_follow
# 或带显示：..\..\venv\Scripts\python.exe -m applications.human_follow --display
```

终端 5 - 游戏手柄控制（可选）：
```bash
cd software/src
..\..\venv\Scripts\python.exe -m applications.gamepad_control
# 或指定手柄：..\..\venv\Scripts\python.exe -m applications.gamepad_control --controller 0
```

终端 6 - 语音交互服务（可选）：
```bash
cd software/src
# 先启动 Wakeup+ASR 服务（PUB模式）
..\..\venv\Scripts\python.exe -m services.speech_service wakeup

# 再启动语音交互应用（SUB模式，新终端）
..\..\venv\Scripts\python.exe -m applications.speech_interaction
```

终端 7 - 机械臂 WLAN 遥操作（可选）：
```bash
cd software/src
# 从端机器人先启动 arm_service
python -m services.motion_service arm

# 主控端运行遥操作应用
python -m applications.arm_teleop --enable --slave-addr tcp://192.168.x.x:5557

# 录制动作（按 r 停止并保存，或退出时保存）
python -m applications.arm_teleop --enable --record trajectories/demo.json

# 回放动作（0.5 倍速循环 3 次）
python -m applications.arm_teleop --playback trajectories/demo.json --playback-speed 0.5 --loop 3
```

或者使用一键启动脚本：
```bash
cd software
..\venv\Scripts\python.exe start_speech_service.py
```

检查模型文件：
```bash
cd software
..\venv\Scripts\python.exe start_speech_service.py --check-models
```

下载语音模型（首次使用）：
```bash
cd software
..\venv\Scripts\python.exe tools/download_speech_models.py
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
..\..\venv\Scripts\python.exe -m tests.test_zmq
..\..\venv\Scripts\python.exe -m tests.test_human_follow
..\..\venv\Scripts\python.exe -m tests.test_web_control
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
venv\Scripts\python.exe software/tools/download_models.py    # Windows
# 或
venv/bin/python software/tools/download_models.py             # Linux/macOS
```

## GUI 打包为 exe

遥操作应用支持独立的 Tkinter GUI，并可用 PyInstaller 打包成 exe：

```bash
cd d:\develop\HomeBot
.venv\Scripts\activate
pip install -r requirements-gui.txt
python software/tools/build_arm_teleop_exe.py
```

打包完成后得到 `dist/HomeBotArmTeleop.exe`，可单独复制到其他 Windows 电脑运行。

注意：
- 打包时已排除 `opencv`、`flask`、`ultralytics`、`numpy` 等未使用的大库
- 若主控电脑安装了飞特 `scservo_sdk`，可额外加入 `--hidden-import scservo_sdk`
- exe 体积主要由 `pyzmq` 和 Python 运行时决定，通常在 15–30 MB

## CLI 打包为 exe

CLI 工具（homebot_cli）可打包为单文件可执行程序，作为控制端/调试端工具使用：

```bash
venv\Scripts\python.exe software/tools/build_homebot_cli_exe.py   # Windows
venv/bin/python software/tools/build_homebot_cli_exe.py           # Linux/macOS
```

打包完成后得到 `dist/homebot.exe`。

注意：
- 打包版仅包含远程调试命令（status / topic / move / doctor），通过网络连接机器人
- `start` / `stop` / `restart` / `logs` 服务管理命令在打包版中不可用（运行时会提示）
- 打包时已排除 `opencv`、`flask`、`ultralytics` 等大库，依赖仅 pyzmq/click

## 扩展开发

添加新应用模块的步骤：
1. 在 `applications/` 下创建新目录
2. 添加 `__init__.py` 和 `__main__.py`
3. 使用 `ChassisArbiterClient` 与底盘服务通信
4. 使用 `VisionSubscriber` 订阅图像流
5. 需要自定义消息时，使用 `common/bus.py` 的 `BusPublisher` / `BusSubscriber` 通过消息总线（5590/5591）收发，无需新增端口

---

*最后更新：2026-08-24*
