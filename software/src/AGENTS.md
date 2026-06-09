<!-- From: d:\develop\homebot\software\src\AGENTS.md -->
# HomeBot 源码层 Agent 文档

本文档面向 AI 编程助手，提供 `software/src/` 目录下所有源码模块的总览、导入规范、公共工具速查和子文档索引。

> 根级项目文档参见 `d:\develop\homebot\AGENTS.md`
> 服务层详细文档参见 `software/src/services/AGENTS.md`
> 导航模块详细文档参见 `software/src/navigation/AGENTS.md`

---

## 模块地图

```
software/src/
├── common/              # 公共工具、消息定义、ZeroMQ 辅助、坐标变换、日志
├── configs/             # 运行时配置 (config.py) 和密钥管理 (secrets.py)
├── hal/                 # 硬件抽象层
│   ├── camera/          # OpenCV 摄像头驱动
│   ├── chassis/         # 底盘驱动（全向轮/差动轮）
│   ├── arm/             # 6-DOF 机械臂驱动 + 运动学
│   ├── audio/           # 音频驱动
│   ├── battery/         # 电池监测驱动
│   ├── gamepad/         # Xbox 手柄 XInput 驱动
│   └── ftservo_driver.py # 飞特舵机 SDK 封装
├── services/            # 服务层（详见 services/AGENTS.md）
│   ├── motion_service/  # 底盘+机械臂控制服务
│   ├── vision_service/  # 视觉采集发布服务
│   └── speech_service/  # 语音唤醒/ASR/TTS 服务
├── applications/        # 应用层
│   ├── remote_control/  # Flask + SocketIO 网页遥控
│   ├── gamepad_control/ # Xbox 手柄控制
│   ├── human_follow/    # YOLO 人体检测 + 视觉伺服跟随
│   ├── speech_interaction/ # LLM 语音对话 + MCP 工具
│   ├── vision_understanding/ # 视觉理解
│   └── imitation_learning/   # 模仿学习
├── navigation/          # 导航与 SLAM（详见 navigation/AGENTS.md）
│   ├── core/            # 占用栅格、A*、SLAM 融合
│   ├── perception/      # AprilTag、深度估计、障碍物检测
│   ├── planning/        # 局部代价地图、VFH 局部规划
│   ├── coordinator/     # NavigationCoordinator 状态机
│   ├── services/        # Odom、SLAM、Depth 服务
│   ├── applications/    # 全局自主导航应用
│   ├── simulation/      # Matplotlib 2D 仿真器
│   ├── visualization/   # Viser 3D 可视化
│   └── hal/             # LD06 激光雷达驱动
├── examples/            # 示例代码
└── tests/               # 测试代码
```

### 各目录职责速查

| 目录 | 职责 | 典型修改场景 |
|------|------|-------------|
| `common/` | 跨模块公共代码，不依赖业务逻辑 | 新增消息类型、ZMQ 工具、坐标变换 |
| `configs/` | 所有配置集中管理 | 新增硬件参数、修改 ZMQ 端口、调整阈值 |
| `hal/` | 硬件驱动封装，向上层屏蔽硬件差异 | 适配新摄像头、新底盘类型、新舵机 |
| `services/` | 后台常驻服务，通过 ZMQ 对外提供能力 | 新增传感器服务、修改通信协议 |
| `applications/` | 面向用户的可执行应用 | 新增交互应用、修改控制逻辑 |
| `navigation/` | 自主导航算法、仿真、可视化 | 修改规划算法、调整 SLAM 参数、新增可视化 |

---

## ZeroMQ 端口速查

> 完整端口表、消息格式和数据流图参见 `software/src/services/AGENTS.md`

| 端口 | 用途 | Socket |
|------|------|--------|
| 5555 | 电池状态 | PUB |
| 5556 | 底盘控制 | REP |
| 5557 | 机械臂控制 | REP |
| 5558 | 底盘状态 | PUB |
| 5559 | 里程计 | PUB |
| 5560 | 视觉图像 | PUB (multipart) |
| 5561 | 深度图像 | PUB (multipart) |
| 5562 | 深度障碍物 | PUB (multipart) |
| 5563 | SLAM 位姿 | PUB |
| 5564 | SLAM 地图 | PUB (multipart) |
| 5565 | 激光扫描 | PUB |
| 5566 | Viser 目标点 | PUB |
| 5567 | 里程计命令 | REP |
| 5568 | SLAM 命令 | REP |
| 5569 | 导航路径 | PUB |
| 5570 | 导航状态 | PUB |
| 5571 | 语音唤醒/ASR | PUB |

---

## 统一导入规范

### 配置访问
```python
from configs import get_config, require_secrets

config = get_config()
print(config.chassis.serial_port)
```

### 日志
```python
from common.logging import get_logger
logger = get_logger(__name__)
```

### ZMQ 辅助
```python
from common.zmq_helper import create_socket, send_json, recv_json
from common.zmq_subscriber import ZMQJsonSubscriber, ZMQMultipartJsonSubscriber, ZMQMultipartImageSubscriber
```

### 坐标变换
```python
from common.transform import world_to_robot2, robot_to_world2, pose2_to_matrix, matrix_to_pose2
```

### 消息序列化
```python
from common.messages import MessageType, serialize, deserialize
```

### 底盘/机械臂控制客户端
```python
from services.motion_service.chassis_arbiter.arbiter import ChassisArbiterClient, ArmArbiterClient
```

### 视觉订阅
```python
from services.vision_service.vision import VisionSubscriber
# 或通用基类
from common.zmq_subscriber import ZMQMultipartImageSubscriber
```

---

## 公共工具速查 (common/)

| 文件 | 公共 API | 说明 |
|------|---------|------|
| `logging.py` | `get_logger(name)` | 创建带格式化的 StreamHandler 日志器 |
| `messages.py` | `MessageType` (Enum), `serialize()`, `deserialize()` | 消息类型定义和 JSON 序列化 |
| `zmq_helper.py` | `create_socket()`, `send_json()`, `recv_json()` | ZMQ socket 创建和 JSON 收发封装 |
| `zmq_subscriber.py` | `ZMQJsonSubscriber`, `ZMQMultipartJsonSubscriber`, `ZMQMultipartImageSubscriber` | 后台线程订阅者基类 |
| `transform.py` | `pose2_to_matrix()`, `world_to_robot2()`, `robot_to_world2()`, `pose3_to_matrix()` 等 | 2D/3D 齐次坐标变换 |

**关键规范**: 应用层订阅者**禁止使用** `zmq.CONFLATE`，必须使用后台线程持续接收。服务层内部持续 `recv` 的循环可使用 `CONFLATE`。

---

## 配置体系速查 (configs/)

### 配置 Dataclass 清单

| Dataclass | 关键字段 | 说明 |
|-----------|---------|------|
| `CameraConfig` | `device_id`, `width`, `height`, `fps` | 摄像头参数 |
| `ArmConfig` | `serial_port`, servo IDs, `upper_arm_length`, `forearm_length`, `joint_limits`, `rest_position` | 机械臂参数 |
| `ChassisConfig` | `chassis_type` (omni/diff), `serial_port`, servo IDs, `max_linear_speed`, `max_angular_speed` | 底盘参数 |
| `ZMQConfig` | 所有 PUB/SUB/REP 地址 | ZeroMQ 端口集中配置 |
| `NavigationConfig` | `lookahead_distance_m`, `inflation_radius_m`, `safety_distance_m`, `max_replan_attempts` | 导航参数 |
| `SLAMConfig` | `map_size_pixels`, `map_size_meters`, `tag_map`, camera intrinsics | SLAM 参数 |
| `ViserConfig` | `host`, `port`, 各可视化地址 | Viser 可视化参数 |
| `SpeechConfig` | 唤醒/ASR 模型路径、采样率、关键词 | 语音参数 |
| `TTSConfig` | `appid`, `access_token`, `resource_id` | TTS 参数（从 secrets 加载） |
| `LLMConfig` | `provider`, `api_key`, `model` | LLM 参数（从 secrets 加载） |
| `VisionConfig` | `provider`, `api_key`, `model` | 视觉理解参数（从 secrets 加载） |
| `HumanFollowConfig` | `model_path`, `conf_threshold`, PID 增益 | 人体跟随参数 |
| `GamepadConfig` | 死区、臂步长、夹爪开合角 | 手柄参数 |
| `BatteryConfig` | 电压阈值、发布间隔 | 电池监测参数 |

### 密钥管理 (secrets.py)

| 函数 | 用途 |
|------|------|
| `get_secrets()` | 获取 Secrets 单例 |
| `require_secrets("tts\|llm\|vision")` | 强制检查密钥，缺失时退出并提示 |
| `check_secrets(verbose=True)` | 返回状态字典并打印配置报告 |
| `reload_secrets()` | 运行时从环境变量重新加载 |

**加载优先级**: 环境变量 > `.env.local` > `.env.development` > `.env.production` > `.env`

**重要**: `.env.local` 已加入 `.gitignore`，密钥永不入版本库。

---

## 子文档索引

| 模块 | 文档路径 | 覆盖内容 |
|------|---------|---------|
| 服务层 | `software/src/services/AGENTS.md` | 底盘/臂/视觉/语音/导航服务详细接口、消息格式、启动命令、数据流图 |
| 导航模块 | `software/src/navigation/AGENTS.md` | 坐标系定义、核心算法（A*/SLAM/VFH）、NavigationCoordinator、仿真器、Viser 可视化 |
| 应用层 | （待创建）`software/src/applications/AGENTS.md` | 各应用的控制流、扩展开发模板 |
| 硬件抽象层 | （待创建）`software/src/hal/AGENTS.md` | 驱动清单、硬件参数、新增硬件适配指南 |
| 公共模块 | （待创建）`software/src/common/AGENTS.md` | 消息格式详解、ZMQ 订阅者使用规范 |
| 配置模块 | （待创建）`software/src/configs/AGENTS.md` | 全量配置项修改指南、平台差异说明 |

---

*最后更新：2026-06-10*
