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
│   │   ├── navigation/        # 导航与 SLAM
│   │   │   ├── hal/           # 激光雷达驱动
│   │   │   ├── perception/    # 感知 (AprilTag, 深度估计)
│   │   │   ├── core/          # SLAM 核心算法
│   │   │   ├── planning/      # 路径规划
│   │   │   ├── services/      # 导航服务 (Odom, SLAM)
│   │   │   ├── applications/  # 导航应用 (目标点跟随、全局导航)
│   │   │   ├── simulation/    # Matplotlib 导航模拟器
│   │   │   └── visualization/ # Viser 3D SLAM 可视化
│   │   ├── examples/          # 示例代码
│   │   └── tests/             # 测试代码
│   ├── third_party/           # 第三方 Git 子模块
│   │   └── breezyslam/        # BreezySLAM (Git submodule)
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

## Agent 文档体系

本项目采用**分层级联 AGENTS.md** 机制：各关键模块目录下放置专门的模块级 `AGENTS.md`，当 AI Agent 操作某个目录下的文件时，能自动获取最相关的上下文。

### 文档树

```
homebot/
├── AGENTS.md                          # 根文档：项目全景、架构图、快速启动
└── software/src/
    ├── AGENTS.md                      # 源码总览：模块地图、导入规范、端口速查
    ├── services/AGENTS.md             # 服务层：接口契约、消息格式、数据流
    ├── navigation/AGENTS.md           # 导航层：坐标系、算法、仿真、可视化
    ├── applications/AGENTS.md         # 应用层：控制流、扩展开发模板
    ├── hal/AGENTS.md                  # 硬件抽象层：驱动清单、适配指南
    ├── common/AGENTS.md               # 公共工具：消息格式、ZMQ 规范
    └── configs/AGENTS.md              # 配置体系：全量配置项速查
```

### 级联生效规则（Agent 必读）

**1. 作用域规则**
- 每个 `AGENTS.md` 对其所在目录及所有子目录生效
- deeper 目录的 `AGENTS.md` 优先级高于父目录（就近原则）
- 用户直接给出的对话指令优先级最高

**2. 对于 Kimi Code CLI**
- 系统**自动**收集与当前操作相关的所有层级 `AGENTS.md`
- 按优先级合并后注入 Agent 上下文，**无需手动读取**
- 操作 `software/src/navigation/core/astar_planner.py` 时，自动获取 `navigation/AGENTS.md` 中的坐标系定义

**3. 对于其他 AI 工具 / 人类开发者**
- 修改某个目录下的代码前，**先读取该目录及上级目录的 `AGENTS.md`**
- 例如修改 `hal/chassis/driver.py` 时，依次参考：
  1. `hal/AGENTS.md`（底盘驱动参数、适配指南）
  2. `software/src/AGENTS.md`（导入规范、端口速查）
  3. `AGENTS.md`（根文档：全局架构、安全规范）

**4. 文档维护义务**
- 修改了某模块的接口、消息格式、配置项或目录结构后，**必须同步更新该模块的 `AGENTS.md`**
- 新增模块时，应在模块根目录新建 `AGENTS.md`，并在 `software/src/AGENTS.md` 的子文档索引中登记

---

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

> **详细文档参见 `software/src/services/AGENTS.md`**，包含完整的 ZeroMQ 端口表、消息格式、启动命令和数据流图。

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
- ZeroMQ PUB 模式发布图像帧 (`tcp://*:5560`)
- JPEG 编码压缩
- 支持 `VisionSubscriber` 订阅

**语音服务** (`services/speech_service/`):
- **WakeupASR Service** (`wakeup_asr_service.py`): 
  - ZeroMQ PUB 模式发布语音识别结果 (`tcp://*:5571`)
  - 持续监听麦克风，检测唤醒词后自动 ASR
- **Voice Engine** (`voice_engine.py`): 语音唤醒、ASR、TTS 封装
- **TTS Client** (`tts_client.py`): 火山引擎流式 TTS 客户端

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

**Viser SLAM 可视化** (`navigation/visualization/`):
- **ViserSLAMVisualizer** (`viser_slam_visualizer.py`): 基于 Viser 的实时 3D SLAM 可视化（RViz 替代）
- 订阅话题：odom (5559)、slam_pose (5563)、slam_map (5564)、lidar_scan (5565)、vision (5560)
- 可视化内容：坐标系树、机器人模型、激光点云、栅格地图、摄像头视锥、双轨迹
- GUI 面板：状态显示、图层控制、视角切换（跟随/自由/顶视）
- 启动：`python -m navigation.visualization`

> **导航算法详细文档参见 `software/src/navigation/AGENTS.md`**，包含坐标系定义、A*/VFH/SLAMFusion 算法说明、仿真器架构和修改指南。

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

终端 6 - SLAM 与导航服务（可选）：
```bash
cd software/src
# 启动里程计服务
python -m navigation.services.odom_service

# 启动 SLAM 融合定位服务（新终端）
python -m navigation.services.slam_service --mock-lidar --mock-tag

# 启动 Viser 3D 可视化（新终端）
python -m navigation.visualization
# 或指定端口：python -m navigation.visualization --port 8080

# 启动全局自主导航（新终端）
python -m navigation.applications.navigation --goal-x 2.0 --goal-y 1.5
# 或调整参数：python -m navigation.applications.navigation --inflation 0.25 --lookahead 0.5
```

终端 7 - 语音交互服务（可选）：
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

浏览器访问 Web 控制界面：`http://<robot-ip>:5000`

界面功能：
- 实时视频流（摄像头画面）
- 虚拟摇杆（左侧控制底盘移动）
- 紧急停止按钮（红色，触发后锁定底盘）
- 归位按钮（蓝色，解锁紧急停止）

浏览器访问 Viser SLAM 可视化：`http://<robot-ip>:8080`

界面功能：
- 3D 场景：坐标系树、机器人模型、激光点云、栅格地图
- 双轨迹对比：里程计轨迹(黄色) vs SLAM 轨迹(青色)
- 摄像头视锥：带实时图像的相机可视化
- GUI 面板：位姿状态、图层开关、视角控制

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

### ZeroMQ 数据订阅规范

**应用层订阅者必须使用后台线程持续接收，禁止单独使用 `zmq.CONFLATE`。**

原因：`zmq.CONFLATE` 在 SUB socket 上只保留队列中最新消息，但若配合 `NOBLOCK` 低频读取（如 10Hz 控制循环），在某些边界情况下会导致消息解析异常或数据 stale。服务层内部消费（如 OdomService 50Hz 主循环）可使用 `CONFLATE`，因其本身就是持续 `recv`。

**正确模式（应用层）：**
```python
from common.zmq_subscriber import ZMQJsonSubscriber

sub = ZMQJsonSubscriber("tcp://localhost:5559", required_keys=("x", "y", "yaw"))
data = sub.read()   # 非阻塞，从内存直接读取
sub.close()
```

提供的统一模板（`common.zmq_subscriber`）：
| 类 | 适用场景 |
|----|---------|
| `ZMQJsonSubscriber` | 订阅 JSON 单帧消息（如 OdomService） |
| `ZMQMultipartJsonSubscriber` | 订阅 multipart 消息，第 N 个 frame 为 JSON（如 DepthService 障碍物） |
| `ZMQMultipartImageSubscriber` | 订阅 multipart 图像帧（如 VisionService） |

**不要重复造轮子**：同一数据话题的订阅逻辑不要每个应用单独实现，优先继承上述基类或直接使用现有订阅者（如 `VisionSubscriber`）。

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

## 人类文档索引 (docs/)

`docs/` 目录下存放面向人类用户的操作指南和开发文档。Agent 在需要了解用户使用场景、故障排查、配置教程时，可参考以下索引：

| 主题 | 文档 | 受众 | 内容 |
|------|------|------|------|
| API 密钥 | `API密钥配置指南.md` | 开发者/用户 | TTS/LLM/Vision 密钥获取与配置 |
| 物料清单 | `HomeBot_BOM物料清单.md` | 组装者 | 完整硬件零件清单 |
| 新应用开发 | `HomeBot新应用开发指南.md` | 开发者 | 创建 `applications/my_app/` 的逐步指南 |
| 导航系统架构 | `homebot_nav_design.md` | 开发者 | SLAM + 深度感知分层架构设计 |
| NavigationCoordinator | `NavigationCoordinator_详细说明.md` | 开发者 | 状态机、纯追踪、参数详解 |
| Web 控制端 | `Web控制界面介绍.md`, `网页控制端使用指南.md` | 用户 | 界面功能、启动方式、手机访问 |
| 人体跟随 | `人体检测与跟随方案.md`, `人体跟随使用指南.md` | 开发者/用户 | YOLO 方案设计 + 使用教程 |
| 导航系统开发 | `导航系统开发方案.md` | 开发者 | 5 阶段增量开发计划 |
| 工具脚本 | `工具脚本使用指南.md` | 开发者/用户 | 串口/摄像头枚举、机械臂校准 |
| 技术选型 | `技术方案选型.md` | 开发者 | ZeroMQ vs MQTT、避障方案对比 |
| 游戏手柄 | `游戏手柄控制使用指南.md`, `游戏手柄控制快速参考.md` | 用户 | Xbox 手柄映射、配置、故障排查 |
| Picoclaw 控制 | `用Picoclaw小龙虾控制HomeBot.md` | 用户/开发者 | MCP 技能安装、自然语言控制 |
| 语音交互 | `语音交互使用指南.md`, `自定义唤醒词配置指南.md` | 用户/开发者 | 语音命令、唤醒词配置、模型下载 |
| 软件架构 | `软件架构与开发规划.md` | 开发者 | 分层架构、通信协议、开发时间线 |
| 配置修改 | `配置修改说明.md` | 用户/开发者 | 所有配置字段逐条解释 |
| 问题记录 | `问题记录.md` | 开发者 | 已知问题与解决方案 |
| 更新记录 | `更新记录.md` | 开发者/用户 | 版本变更日志 |

---

*最后更新：2026-06-10*
