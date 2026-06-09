<!-- From: d:\develop\homebot\software\src\services\AGENTS.md -->
# HomeBot 服务层 Agent 文档

本文档面向 AI 编程助手，提供服务层的模块速查、接口契约和修改指南。

> 根级项目文档参见 `d:\develop\homebot\AGENTS.md`
> 同级模块文档参见 `software/src/navigation/AGENTS.md`

---

## 模块速查

### 服务清单

| 服务 | 入口模块 | 主类 | 启动命令 | 说明 |
|------|---------|------|---------|------|
| 底盘服务 | `services.motion_service.chassis_service` | `ChassisService` | `python -m services.motion_service.chassis_service` | 运动控制 + 仲裁 + 电池监测 |
| 机械臂服务 | `services.motion_service.arm_service` | `ArmService` | `python -m services.motion_service.arm_service` | 6-DOF 机械臂控制 |
| 运动服务启动器 | `services.motion_service` | — | `python -m services.motion_service --service both` | 共享串口总线，同时启动底盘+机械臂 |
| 视觉服务 | `services.vision_service` | `VisionService` | `python -m services.vision_service` | 摄像头采集 + JPEG 发布 |
| 语音唤醒/ASR | `services.speech_service` | `WakeupASRService` | `python -m services.speech_service wakeup` | 唤醒词检测 + ASR，PUB 识别结果 |
| 里程计服务 | `navigation.services.odom_service` | `OdomService` | `python -m navigation.services.odom_service` | 速度积分位姿，50Hz |
| SLAM 服务 | `navigation.services.slam_service` | `SLAMService` | `python -m navigation.services.slam_service` | 激光/视觉/轮式里程计融合定位建图 |
| 深度感知服务 | `navigation.services.depth_service` | `DepthService` | `python -m navigation.services` | MiDaS 深度估计 + 障碍物直方图 |

### 核心辅助类

| 文件 | 类/函数 | 职责 |
|------|---------|------|
| `services.motion_service.chassis_arbiter.arbiter` | `ChassisArbiterClient` | 底盘控制 REQ 客户端封装 |
| `services.motion_service.chassis_arbiter.arbiter` | `ArmArbiterClient` | 机械臂控制 REQ 客户端封装 |
| `services.motion_service.servo_bus_manager` | `ServoBusManager` | 共享串口总线单例（底盘+机械臂共用） |
| `services.vision_service.vision` | `VisionSubscriber` | 视觉订阅者（后台线程 + 最新帧缓存） |
| `services.speech_service.voice_engine` | `VoiceEngine` | 语音唤醒/ASR/TTS 引擎封装 |
| `services.speech_service.tts_client` | `VolcanoTTSClient` | 火山引擎流式 TTS 客户端 |

---

## ZeroMQ 端口总表

| 端口 | Socket | 服务 | 数据类型 | 方向 |
|------|--------|------|---------|------|
| 5555 | `PUB` | ChassisService (Battery) | JSON | Out |
| 5556 | `REP` | ChassisService (Control) | JSON | In |
| 5557 | `REP` | ArmService | JSON | In |
| 5558 | `PUB` | ChassisService (State) | JSON | Out |
| 5559 | `PUB` | OdomService | JSON | Out |
| 5560 | `PUB` | VisionService | multipart: frame_id + jpeg | Out |
| 5561 | `PUB` | DepthService (Depth) | multipart: frame_id + jpeg | Out |
| 5562 | `PUB` | DepthService (Obstacle) | multipart: frame_id + json | Out |
| 5563 | `PUB` | SLAMService (Pose) | JSON | Out |
| 5564 | `PUB` | SLAMService (Map) | multipart: json_meta + bytes | Out |
| 5565 | `PUB` | SLAMService (LidarScan) | JSON | Out |
| 5566 | `PUB` | Viser (Goal) | JSON | Out |
| 5567 | `REP` | OdomService (Cmd) | JSON | In |
| 5568 | `REP` | SLAMService (Cmd) | JSON | In |
| 5569 | `PUB` | NavigationApp (Path) | JSON | Out |
| 5570 | `PUB` | NavigationApp (Status) | JSON | Out |
| 5571 | `PUB` | WakeupASRService | JSON | Out |

---

## 数据流图

```
[ChassisService] --(PUB 5558)--> [OdomService] --(PUB 5559)--> [SLAMService]
       ^                                                              ^
       |                                                              |
   (REP 5556)                                                    (SUB)
       |                                                              |
[Gamepad/Web/Voice/Auto]                                    [Viser Visualizer]
                                                                    |
[VisionService] --(PUB 5560)-------------------------------------> [SLAMService]
       |                                                            |
       +---> [DepthService] --(PUB 5561/5562)--> [Nav App/Viser]    |
                                                                    |
[LiDAR HAL] --------------(internal)------------> [SLAMService]     |
                                                                    |
                                                        (PUB 5563/5564/5565)
                                                                    |
                                                            [Viser Visualizer]
```

---

## 1. 运动服务 (Motion Service)

### 1.1 底盘服务 (ChassisService)

**文件**: `services/motion_service/chassis_service.py`

**职责**: 硬件-backed 底盘控制，集成优先级仲裁器、电池监测、底盘状态发布。支持紧急停止锁定（触发后需 `home` 命令解锁）和 1 秒超时自动释放。

**ZeroMQ 接口**:

| Socket | 类型 | 默认地址 | 说明 |
|--------|------|---------|------|
| 控制 | `REP` | `tcp://*:5556` | 接收速度指令 |
| 电池 | `PUB` | `tcp://*:5555` | 发布电池电压/百分比/状态 |
| 状态 | `PUB` | `tcp://*:5558` | 发布底盘速度、控制源、紧急锁定、编码器里程计 |

**控制指令格式 (REQ → REP)**:
```json
{
    "source": "web|voice|auto|gamepad|emergency|home",
    "vx": 0.5,
    "vy": 0.0,
    "vz": 0.3,
    "priority": 1
}
```
- `priority` 可选，未提供时从 `PRIORITIES` 字典自动解析
- `source="emergency"` 触发紧急停止锁定
- `source="home"` 或 `"command":"home"` 解锁并停止

**响应格式**:
```json
{
    "success": true,
    "message": "指令已接受",
    "current_owner": "web",
    "current_priority": 1
}
```

**状态发布格式 (PUB @ 5558)**:
```json
{
    "vx": 0.5, "vy": 0.0, "vz": 0.3,
    "source": "web",
    "priority": 1,
    "timestamp": 1710662847.123,
    "emergency_locked": false,
    "odom": {"x": 0.1, "y": 0.0, "yaw": 0.05, "source": "chassis_encoder"},
    "vx_feedback": 0.48,
    "vz_feedback": 0.29
}
```

**电池发布格式 (PUB @ 5555)**:
```json
{
    "voltage": 11.4,
    "percentage": 45.0,
    "status": "normal|low|critical",
    "temperature": 25,
    "servo_id": 1
}
```

**控制优先级** (`chassis_arbiter/arbiter.py`):
```python
PRIORITIES = {
    "emergency": 4,
    "gamepad": 3,
    "auto": 2,
    "voice": 2,
    "web": 1,
}
```

**启动命令**:
```bash
# 独立启动
python -m services.motion_service.chassis_service
python -m services.motion_service.chassis_service --port COM3 --addr tcp://*:5556

# 通过共享启动器（推荐，共用串口总线）
python -m services.motion_service --service chassis
python -m services.motion_service --service both --port COM3
```

**依赖**:
- HAL: `hal.chassis.driver.OmniChassisDriver` 或 `hal.chassis.diff_driver.DiffChassisDriver`
- HAL: `hal.battery.driver.BatteryDriver`
- 共享: `servo_bus_manager.ServoBusManager`（`use_shared_bus=True` 时）
- 配置: `configs.config.ChassisConfig`, `BatteryConfig`

---

### 1.2 机械臂服务 (ArmService)

**文件**: `services/motion_service/arm_service.py`

**职责**: 6-DOF 机械臂控制，支持优先级仲裁，使用 bulk sync-write 写入舵机总线。

**ZeroMQ 接口**:

| Socket | 类型 | 默认地址 | 说明 |
|--------|------|---------|------|
| 控制 | `REP` | `tcp://*:5557` | 接收关节角指令 |

**指令格式 (3 种风格)**:
```json
// 风格 1: 数组
{"source": "web", "joints": [0, 45, 90, 0, 0, 30], "speed": 1000, "priority": 1}

// 风格 2: 命名字典
{"source": "web", "joints": {"base": 0, "shoulder": 45, "elbow": 90, "wrist_flex": 0, "wrist_roll": 0, "gripper": 30}, "speed": 1000}

// 风格 3: 查询-only
{"source": "web", "joints": {}, "query": true}
```

**响应格式**:
```json
{
    "success": true,
    "message": "指令已接受",
    "current_owner": "web",
    "current_priority": 1,
    "joint_states": {"base": 0.0, "shoulder": 45.0, ...}
}
```

**机械臂优先级** (本地定义，与底盘略有不同):
```python
PRIORITIES = {
    "emergency": 4,
    "auto": 3,
    "voice": 2,
    "web": 1,
}
```

**启动命令**:
```bash
python -m services.motion_service.arm_service
python -m services.motion_service --service arm
python -m services.motion_service --service both
```

**依赖**:
- HAL: `hal.arm.driver.ArmDriver`
- 共享: `servo_bus_manager.get_servo_bus()`（需先由 motion_service 启动器或底盘服务初始化）

---

### 1.3 串口总线管理器 (ServoBusManager)

**文件**: `services/motion_service/servo_bus_manager.py`

**职责**: 单例管理共享串口总线连接，供底盘和机械臂共同使用。

**无 ZMQ 接口**。提供:
- `get_servo_bus()` → 获取总线实例
- `is_bus_ready()` → 检查总线是否就绪

**依赖**: `hal.ftservo_driver.FTServoBus`

---

### 1.4 仲裁器客户端 (ChassisArbiterClient / ArmArbiterClient)

**文件**: `services/motion_service/chassis_arbiter/arbiter.py`

**职责**: 应用层便捷 REQ 客户端，封装与底盘/机械臂服务的通信。

| 客户端 | 默认地址 | 主要方法 |
|--------|---------|---------|
| `ChassisArbiterClient` | `tcp://127.0.0.1:5556` | `send_command(vx, vy, vz, source, priority)` |
| `ArmArbiterClient` | `tcp://127.0.0.1:5557` | `send_joint_command(joints_list, ...)` / `send_joint_dict(joints_dict, ...)` |

**应用开发规范**: 发送控制指令的频率必须 **> 1Hz**，否则底盘服务 1 秒超时后会自动停止并释放控制权。

---

## 2. 视觉服务 (Vision Service)

### 2.1 VisionService

**文件**: `services/vision_service/vision.py`

**职责**: 从摄像头采集帧，可选运行 `process_frame()`，编码为 JPEG，以 multipart ZMQ 消息发布。

**ZeroMQ 接口**:

| Socket | 类型 | 默认地址 | 说明 |
|--------|------|---------|------|
| 图像流 | `PUB` | `tcp://*:5560` | multipart: `[frame_id_str, jpeg_bytes]` |

**图像编码细节**:
- 格式: JPEG (`cv2.imencode('.jpg', frame)`)
- 传输: `send_multipart([str(frame_id).encode(), buf.tobytes()])`
- FPS: 由配置控制（默认 30），通过 sleep 维持目标间隔

**启动命令**:
```bash
python -m services.vision_service
python -m services.vision_service --display
python -m services.vision_service --addr tcp://*:5560
```

**依赖**: HAL `hal.camera.driver.CameraDriver`，配置 `CameraConfig`

---

### 2.2 VisionSubscriber

**文件**: `services/vision_service/vision.py`（同文件）

**职责**: 后台线程 SUB 消费者，始终保有一份最新帧在内存中。

**使用方式**:
```python
from services.vision_service.vision import VisionSubscriber
sub = VisionSubscriber()
sub.start()
frame_id, frame = sub.read_frame()  # 非阻塞，返回最新帧或 (None, None)
```

> 注意: VisionSubscriber 内部使用 `zmq.CONFLATE=1` + `RCVHWM=1`，这是**服务层内部消费**场景，允许使用 CONFLATE。应用层通用订阅应使用 `common.zmq_subscriber.ZMQMultipartImageSubscriber`。

---

## 3. 语音服务 (Speech Service)

### 3.1 WakeupASRService

**文件**: `services/speech_service/wakeup_asr_service.py`

**职责**: 持续监听麦克风，检测唤醒词后执行 ASR，将识别文本以 JSON 事件通过 ZMQ PUB 发布。

**ZeroMQ 接口**:

| Socket | 类型 | 默认地址 | 说明 |
|--------|------|---------|------|
| 语音事件 | `PUB` | `tcp://*:5571` | 发布 `speech_detected` 事件 |

**发布消息格式**:
```json
{
    "event": "speech_detected",
    "type": "wakeup_asr",
    "session_id": "session_1",
    "keyword": "你好小白",
    "asr_text": "向前走一米",
    "timestamp": 1710662847.123
}
```

**启动命令**:
```bash
python -m services.speech_service
python -m services.speech_service wakeup
python -m services.speech_service wakeup_asr
```

**依赖**:
- `services.speech_service.voice_engine.VoiceEngine`
- 模型: `models/wakeup/` 和 `models/asr/` 下的 sherpa-onnx ONNX 模型

---

### 3.2 VoiceEngine

**文件**: `services/speech_service/voice_engine.py`

**职责**: 封装唤醒检测、ASR 识别、TTS 合成/播放。支持 `mode="full"`（加载唤醒+ASR+TTS 模型）或 `mode="tts_only"`（轻量，不加载模型）。

**无 ZMQ 接口**。被 `WakeupASRService` 和语音交互应用直接消费。

**关键方法**:
- `wakeup() -> bool` — 阻塞直到检测到唤醒词
- `recognize() -> str` — 流式 ASR，带静音超时（默认 1.5s）
- `synthesize(text)` / `synthesize_streaming(text)` — 火山引擎 TTS
- `play_pcm_file(name)` — 播放缓存 PCM（如 `cache/wozai.pcm`）

**依赖**: `services.speech_service.tts_client`，外部库 `sherpa_onnx`, `sounddevice`, `numpy`

---

### 3.3 VolcanoTTSClient

**文件**: `services/speech_service/tts_client.py`

**职责**: 基于 WebSocket 的火山引擎（字节跳动）双向流式 TTS 客户端。

**无 ZMQ 接口**。使用 `websockets` 连接到 `wss://openspeech.bytedance.com/api/v3/tts/bidirection`。

**公共 API**:
- `tts_synthesize_stream(text)` → `AsyncGenerator[bytes, None]`
- `tts_synthesize(text)` → `bytes`
- `tts_to_file(text, audio_file)`
- `tts_connect()` / `tts_disconnect()`

**依赖**: `services.speech_service.protocols`（二进制消息帧协议），密钥 `VOLCANO_APPID`, `VOLCANO_ACCESS_TOKEN`

---

## 4. 导航服务 (Navigation Services)

> 详细算法和架构说明参见同级文档 `software/src/navigation/AGENTS.md`

### 4.1 OdomService

**文件**: `navigation/services/odom_service.py`

**职责**: 订阅底盘状态，积分速度估计机器人位姿 `(x, y, yaw)`。优先使用底盘编码器里程计；否则退化为指令速度中点法积分。50Hz 发布，支持 REP 复位命令。

**ZeroMQ 接口**:

| Socket | 类型 | 默认地址 | 说明 |
|--------|------|---------|------|
| 底盘状态订阅 | `SUB` | `tcp://localhost:5558` | 接收底盘速度/状态 |
| 里程计发布 | `PUB` | `tcp://*:5559` | 发布积分位姿 |
| 里程计命令 | `REP` | `tcp://*:5567` | 接收 reset_pose 命令 |

**复位命令格式 (REP @ 5567)**:
```json
{"cmd": "reset_pose", "x": 0.0, "y": 0.0, "yaw": 0.0}
```

**发布格式 (PUB @ 5559)**:
```json
{
    "x": 0.1234,
    "y": 0.0567,
    "yaw": 0.3142,
    "vx": 0.5,
    "vy": 0.0,
    "vz": 0.3,
    "timestamp": 1710662847.123
}
```

**启动命令**:
```bash
python -m navigation.services.odom_service
python -m navigation.services.odom_service --rate 50.0
```

---

### 4.2 SLAMService

**文件**: `navigation/services/slam_service.py`

**职责**: 融合 LD06 激光雷达、轮式里程计和 AprilTag 视觉定位，输出统一位姿和占用栅格地图。基于 BreezySLAM + `SLAMFusion`。支持地图加载/保存、纯定位模式切换、优雅退出时地图导出。

**ZeroMQ 接口**:

| Socket | 类型 | 默认地址 | 说明 |
|--------|------|---------|------|
| 视觉订阅 | `SUB` | `tcp://localhost:5560` | 接收摄像头帧用于 AprilTag |
| 里程计订阅 | `SUB` | `tcp://localhost:5559` | 接收轮式里程计 |
| 位姿发布 | `PUB` | `tcp://*:5563` | 发布融合位姿 |
| 地图发布 | `PUB` | `tcp://*:5564` | 发布占用栅格地图（multipart: meta JSON + bytes） |
| 激光扫描发布 | `PUB` | `tcp://*:5565` | 发布原始激光点云供可视化 |
| SLAM 命令 | `REP` | `tcp://*:5568` | 接收 reset_pose / set_mode 命令 |

**位姿发布格式 (PUB @ 5563)**:
```json
{
    "x": 1.234,
    "y": 0.567,
    "theta": 0.123,
    "covariance": [[...], ...],
    "state": "tracking|lost|...",
    "slam_fail_count": 0,
    "timestamp": 1710662847.123
}
```

**地图发布格式 (PUB @ 5564)**: multipart `[json_meta, map_bytes]`
- meta: `{"size_pixels": 800, "size_meters": 10.0, "timestamp": ...}`

**激光扫描格式 (PUB @ 5565)**:
```json
{
    "angles_deg": [0, 1, ...],
    "distances_m": [1.2, 1.3, ...],
    "timestamp": 1710662847.123
}
```

**命令格式 (REP @ 5568)**:
```json
{"cmd": "reset_pose", "x": 0.0, "y": 0.0, "theta": 0.0}
{"cmd": "set_mode", "mode": "localization_only|navigation|mapping"}
```

**启动命令**:
```bash
python -m navigation.services.slam_service
python -m navigation.services.slam_service --mock-lidar --mock-tag
python -m navigation.services.slam_service --load-map software/maps/home.npz --save-map software/maps/home.npz
python -m navigation.services.slam_service --init-x 0.0 --init-y 0.0 --init-theta 0.0
```

**内部架构**:
1. **主循环** (~10Hz): 激光扫描 → `SLAMFusion.update_lidar()` → 发布位姿
2. **视觉线程**: 接收最新摄像头帧
3. **里程计线程**: 接收最新里程计
4. 每 5 个循环 (~2Hz): AprilTag 检测 + 融合
5. 每 2 秒: 发布地图

**依赖**:
- 订阅: `VisionService`, `OdomService`
- HAL: `navigation.hal.lidar_driver.create_lidar_driver`
- 核心: `navigation.core.slam_fusion.SLAMFusion`
- 感知: `navigation.perception.apriltag_detector.create_apriltag_detector`

---

### 4.3 DepthService

**文件**: `navigation/services/depth_service.py`

**职责**: 订阅视觉服务帧，运行深度估计（MiDaS ONNX 或 fake 回退），伪彩色化深度为 JPEG，可选运行障碍物直方图检测，双线程发布两路流。

**ZeroMQ 接口**:

| Socket | 类型 | 默认地址 | 说明 |
|--------|------|---------|------|
| 视觉订阅 | `SUB` | `tcp://localhost:5560` | 接收摄像头帧 |
| 深度发布 | `PUB` | `tcp://*:5561` | 发布伪彩色深度 JPEG |
| 障碍物发布 | `PUB` | `tcp://*:5562` | 发布距离直方图 JSON |

**深度图像格式 (PUB @ 5561)**: multipart `[frame_id_str, jpeg_bytes]`

**障碍物信息格式 (PUB @ 5562)**: multipart `[frame_id_str, json_bytes]`
```json
{
    "histogram": [1.2, 0.8, null, ...],
    "inference_ms": 45.67,
    "estimator_type": "MiDaSDepthEstimator|FakeDepthEstimator"
}
```

**启动命令**:
```bash
python -m navigation.services
python -m navigation.services.depth_service --model models/midas.onnx --fps 10
python -m navigation.services.depth_service --no-obstacle
```

**依赖**:
- 订阅: `VisionService`
- 感知: `navigation.perception.depth_estimator.create_depth_estimator`
- 感知: `navigation.perception.obstacle_detector.DepthObstacleDetector`（可选）

---

## 修改指南

### 添加新服务

1. 在 `services/` 或 `navigation/services/` 下创建新模块目录，包含 `__init__.py` 和 `__main__.py`
2. `__main__.py` 中解析参数并调用 `main()`
3. 使用 `common.zmq_helper.create_socket()` 创建 ZMQ socket
4. 服务必须可通过 `python -m <module.path>` 启动
5. 在本文档的**服务清单**和**端口总表**中登记新服务

### 修改底盘/机械臂控制协议

- 修改 `PRIORITIES` 字典时，需同步更新本文档和根 `AGENTS.md` 中的优先级表
- 底盘和机械臂的 `PRIORITIES` 是**独立定义**的，不要混用
- 新增 `source` 类型时，需在 `chassis_service.py` 和 `arbiter.py` 中同时注册

### 修改 ZeroMQ 地址

- 所有地址必须在 `configs.config.ZMQConfig` 中定义默认值
- 服务实现中应通过 `get_config().zmq.xxx_addr` 读取，不要硬编码
- 修改端口后，需同步更新:
  1. `software/src/configs/config.py` 中的 `ZMQConfig`
  2. 本文档的**端口总表**
  3. `software/src/navigation/AGENTS.md` 中的相关引用
  4. 根 `AGENTS.md` 中的启动命令示例

### 紧急停止机制

- 紧急停止是**全局状态**，在 `ChassisService` 中维护
- 触发后，底盘进入锁定状态，拒绝所有非 `home` 命令
- 机械臂服务**独立**于底盘紧急停止，不受其影响

---

## 人类文档索引

| 主题 | 对应人类文档 (`docs/`) |
|------|----------------------|
| API 密钥配置 | `API密钥配置指南.md` |
| 网页控制端使用 | `网页控制端使用指南.md`, `Web控制界面介绍.md` |
| 游戏手柄控制 | `游戏手柄控制使用指南.md`, `游戏手柄控制快速参考.md` |
| 语音交互 | `语音交互使用指南.md`, `自定义唤醒词配置指南.md` |
| 人体跟随 | `人体检测与跟随方案.md`, `人体跟随使用指南.md` |
| 导航系统设计 | `导航系统开发方案.md`, `homebot_nav_design.md`, `NavigationCoordinator_详细说明.md` |
| 新应用开发 | `HomeBot新应用开发指南.md` |
| 配置修改 | `配置修改说明.md` |
| 工具脚本 | `工具脚本使用指南.md` |
| 技术方案选型 | `技术方案选型.md` |
| 问题记录 | `问题记录.md` |
