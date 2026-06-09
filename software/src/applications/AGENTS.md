<!-- From: d:\develop\homebot\software\src\applications\AGENTS.md -->
# HomeBot 应用层 Agent 文档

本文档面向 AI 编程助手，提供应用层各模块的速查、控制流和扩展开发指南。

> 根级项目文档参见 `d:\develop\homebot\AGENTS.md`
> 源码总览参见 `software/src/AGENTS.md`
> 服务层接口契约参见 `software/src/services/AGENTS.md`

---

## 模块速查

### 应用清单

| 应用 | 入口模块 | 主类 | 启动命令 | 控制源 | 优先级 |
|------|---------|------|---------|--------|--------|
| 网页遥控 | `applications.remote_control` | `ZMQBridge` (in `web_server.py`) | `python -m applications.remote_control` | `web` | 1 |
| 游戏手柄 | `applications.gamepad_control` | `GamepadControlApp` | `python -m applications.gamepad_control` | `gamepad` | 2 (或 3) |
| 人体跟随 | `applications.human_follow` | `HumanFollowApp` | `python -m applications.human_follow` | `auto` | 3 |
| 语音交互 | `applications.speech_interaction` | `SpeechInteractionApp` | `python -m applications.speech_interaction` | `voice` | 2 |
| 视觉理解 | `applications.vision_understanding` | `VisionAnalyzer` | `python -m applications.vision_understanding` | — | — |
| 模仿学习 | `applications.imitation_learning` | `ImitationLearner` | （未完成） | — | — |

### 应用 → 服务依赖矩阵

| 应用 | 底盘 (5556) | 机械臂 (5557) | 视觉 (5560) | 深度 (5561) | 语音 (5571) | 电池 (5555) |
|------|:-----------:|:-------------:|:-----------:|:-----------:|:-----------:|:-----------:|
| remote_control | REQ | REQ | SUB | SUB | — | — |
| gamepad_control | REQ | REQ | — | — | — | — |
| human_follow | REQ | — | SUB | — | — | — |
| speech_interaction | REQ | REQ | — | — | SUB | SUB |
| vision_understanding | — | — | SUB | — | — | — |
| imitation_learning | — | — | — | — | — | — |

---

## 1. 网页遥控 (remote_control)

**文件**: `applications/remote_control/web_server.py`, `controller.py`

**职责**: Flask + Flask-SocketIO 网页控制端。双虚拟摇杆（nippleJS）：左手底盘、右手机械臂；MJPEG 视频/深度流；紧急停止 + 归位按钮；可启停人体跟随子进程。

**通信方式**:

| 服务 | 地址 | 客户端类 | 模式 |
|------|------|---------|------|
| 底盘 | `tcp://127.0.0.1:5556` | `ZMQClient` | REQ (100ms 超时, 轮询) |
| 机械臂 | `tcp://127.0.0.1:5557` | `ArmClient` | REQ (50ms 超时, 50Hz 限流) |
| 视觉 | `tcp://127.0.0.1:5560` | `VideoStreamClient` | SUB (后台线程) |
| 深度 | `tcp://127.0.0.1:5561` | `DepthStreamClient` | SUB (后台线程) |

**控制流**:
1. `run_server()` 创建 `ZMQBridge`，启动视频/深度流客户端
2. SocketIO 事件处理前端输入：`joystick_data`, `arm_joystick`, `emergency_stop`, `home`, `gripper_toggle`, `toggle_human_follow`
3. `ZMQBridge._send_loop()` 以 **20Hz** 轮询发送底盘命令；无客户端活动时休眠 0.5s
4. 机械臂摇杆通过 `ArmClient.process_joystick()` 处理，含逆运动学自动腕部调平

**关键配置** (硬编码于 `web_server.py`):
- `max_linear = 0.5` m/s, `max_angular = 1.0` rad/s
- `_arm_joystick_min_interval = 0.02` (50Hz 服务端节流)
- Source `"web"`, Priority `1`

**特殊机制**:
- 紧急停止 → `source="emergency", priority=4`
- 归位 → `source="home", priority=0`（解锁紧急锁定）
- 人体跟随通过 `subprocess.Popen` 启停 `applications.human_follow`

---

## 2. 游戏手柄控制 (gamepad_control)

**文件**: `applications/gamepad_control/app.py`

**职责**: Xbox 手柄同时控制底盘和机械臂。支持震动反馈、死区处理、腕部自动调平。

**控制映射**:

| 输入 | 功能 |
|------|------|
| 左摇杆 | 底盘前后/旋转 |
| LT/RT 扳机键 | 底盘左右平移 |
| 右摇杆 | 机械臂基座旋转 / 伸缩（前后） |
| 十字键 | 机械臂升降 / 腕部旋转 |
| Y/A/B | 腕部屈伸手动/自动模式 |
| RB/LB | 夹爪开/合 |
| Back | 紧急停止 |
| Start | 复位 |

**通信方式**:
- `ChassisArbiterClient` (`tcp://localhost:5556`, 500ms 超时)
- `ArmArbiterClient` (`tcp://localhost:5557`, 1000ms 超时)

**控制流**:
1. `initialize()`: 连接手柄 → 连接底盘 → 连接机械臂 → 同步机械臂状态
2. `run()`: 轮询循环，间隔 `config.polling_interval` (默认 0.02s = **50Hz**)
3. 每轮: `get_state()` → 系统输入 → 底盘输入 → 发送底盘命令 → 机械臂输入 → 发送机械臂命令
4. 死区过渡检测：从运动区进入死区时发送显式停止命令

**关键配置** (`GamepadConfig`):
- `max_linear_speed = 0.5`, `max_angular_speed = 1.0`
- `trigger_deadzone = 0.1`, `left_stick_deadzone = 0.15`, `right_stick_deadzone = 0.15`
- 臂步长: `base=3.0°, elbow=2.0°, shoulder=2.0°, wrist_flex=3.0°, wrist_roll=3.0°`
- `arm_gripper_open=90.0`, `arm_gripper_close=0.0`, `arm_speed=800`

**特殊机制**:
- 腕部自动调平: `_wrist_auto_level=True` 时，`wrist_flex = 180 - shoulder - elbow`
- 手动腕部模式通过 Y/A 键进入，B 键退出
- 使用 `ArmKinematics` 维护 `(r, z)` 工作空间坐标

---

## 3. 人体跟随 (human_follow)

**文件**: `applications/human_follow/follow.py`, `detector.py`, `tracker.py`, `controller.py`

**职责**: YOLO 人体检测 → IoU 多目标跟踪 → 视觉伺服 PID 控制底盘速度。

**状态机**: `IDLE → FOLLOWING → SEARCHING → IDLE`

**通信方式**:
- `VisionSubscriber` (`tcp://localhost:5560`, SUB 后台线程)
- `ChassisArbiterClient.send_command()` (`tcp://localhost:5556`)

**控制流**:
1. `initialize()`: 启动视觉订阅 → 加载 YOLO 模型 → 初始化跟踪器和控制器 → 连接底盘
2. `run()`: 循环读取帧 `vision_sub.read_frame()`
3. `_process_frame()`: 检测 → 跟踪更新 → 选择主目标 → 计算速度 → 平滑 → 发送
4. 丢失目标: 进入 SEARCHING（旋转搜索）或停止

**关键配置** (`HumanFollowConfig`):
- `model_path = "models/yolo26n.onnx"` (代码中实际使用 `.pt`)
- `conf_threshold = 0.5`, `inference_size = 320`
- `target_distance = 1.0` m, `target_width_ratio = 0.4`
- `kp_linear = 0.8`, `kp_angular = 1.5`
- `max_linear_speed = 0.5`, `max_angular_speed = 2.0`
- `lost_patience = 30` 帧
- Source `"auto"`, Priority `3`

**特殊机制**:
- 控制器在 **320×320 归一化参考空间** 中计算，独立于实际摄像头分辨率
- `vx` 被钳制到 `>= 0`（跟随时不后退）
- 速度平滑 alpha = 0.3

---

## 4. 语音交互 (speech_interaction)

**文件**: `applications/speech_interaction/speech_app.py`, `dialogue_manager.py`, `mcp_server.py`

**职责**: 订阅 WakeupASR → LLM 对话管理（工具调用）→ 执行机器人控制 → TTS 播报回应。

**通信方式**:

| 服务 | 地址 | 方向 |
|------|------|------|
| WakeupASR | `tcp://localhost:5571` | SUB (100ms 接收超时) |
| 底盘 | `tcp://localhost:5556` | REQ via `RobotControllerClient` |
| 机械臂 | `tcp://localhost:5557` | REQ via `RobotControllerClient` |
| 电池 | `tcp://localhost:5555` | SUB (后台线程缓存) |

**控制流**:
1. `SpeechInteractionApp.run()`: 初始化 MCP 客户端 → 后台预加载人体跟随模型 → 循环接收 `speech_detected` 事件
2. `_handle_speech_event()`: 按 `session_id` 去重 → `dialogue_manager.process_query()` → `_speak()` TTS 播报
3. `DialogueManager`: 两轮 LLM — 第一轮获取 tool_calls 并静默执行，第二轮生成面向用户的总结回复
4. `mcp_server.py`: `RobotControllerClient` 封装 REQ socket，底盘命令在运动时**每 200ms 重复发送**防止 1s 超时

**MCP 工具列表**:
- 底盘: `move_forward`, `move_backward`, `turn_left`, `turn_right`, `stop_robot`
- 状态: `get_robot_status`, `get_battery_status`
- 机械臂: `reset_arm`, `raise_arm`, `lower_arm`, `extend_arm`, `retract_arm`, `rotate_arm_left`, `rotate_arm_right`, `grab_object`, `release_object`, `hold_object`, `move_arm_to_position`
- 视觉/跟随: `what_does_robot_see`, `start_human_follow`, `stop_human_follow`, `get_human_follow_status`

**关键配置**:
- `llm.temperature = 0.1`, `max_tokens = 256`
- TTS: `VoiceEngine(mode="tts_only")`（不加载唤醒/ASR 模型）
- Source `"voice"`, Priority `2`

**特殊机制**:
- 电池状态使用**全局缓存 + 后台订阅线程**，避免阻塞主循环
- 可启停人体跟随子进程

---

## 5. 视觉理解 (vision_understanding)

**文件**: `applications/vision_understanding/vision_analyzer.py`

**职责**: 订阅视觉流，截取最新帧，调用火山 Ark 视觉语言模型进行图像理解。

**通信方式**:
- `VisionSubscriber` (`tcp://localhost:5560`, SUB)

**控制流**:
1. `capture_frame()`: 确保订阅启动 → `read_frame()` → `cv2.imwrite()` 保存临时 JPEG
2. `analyze()`: Base64 编码 → 调用 `Ark.chat.completions.create()` 带 `image_url`
3. `capture_and_analyze()`: 合并上述两步

**关键配置**:
- 默认模型: `doubao-seed-2-0-mini-260215`（可被 `ARK_MODEL_ID` 覆盖）
- `base_url = "https://ark.cn-beijing.volces.com/api/v3"`
- 需要 `volcenginesdkarkruntime` 包
- 无直接机器人控制，纯分析功能

---

## 6. 模仿学习 (imitation_learning)

**文件**: `applications/imitation_learning/imitation.py`

**状态**: 占位符/未完成。订阅 `tcp://localhost:5580`（项目内无对应发布端）。`record_action` 和 `replay` 为空实现。

---

## 控制优先级汇总

| 优先级 | 控制源 | 典型应用 | 说明 |
|--------|--------|---------|------|
| 4 | `emergency` | 所有应用的紧急停止按钮 | 全局锁定，需 `home` 解锁 |
| 3 | `auto` | 人体跟随、自主导航 | 自动模式 |
| 2 | `gamepad` / `voice` | 手柄、语音交互 | 两者同级 |
| 1 | `web` | 网页遥控 | 最低优先级 |

> 机械臂服务有独立的优先级表（`emergency > auto > voice > web`），与底盘略有不同。

---

## 扩展开发指南

### 添加新应用的步骤

1. **创建目录结构**:
   ```
   applications/my_app/
   ├── __init__.py
   ├── __main__.py
   └── app.py
   ```

2. **`__main__.py` 模板**:
   ```python
   import argparse
   from .app import MyApp

   def main():
       parser = argparse.ArgumentParser()
       # 添加参数
       args = parser.parse_args()
       app = MyApp()
       app.run()

   if __name__ == "__main__":
       main()
   ```

3. **底盘通信**: 使用 `ChassisArbiterClient`
   ```python
   from services.motion_service.chassis_arbiter.arbiter import ChassisArbiterClient
   client = ChassisArbiterClient()
   client.send_command(vx, vy, vz, source="my_app", priority=2)
   ```
   > **必须保持 >1Hz 的心跳频率**，否则 1 秒超时后底盘自动停止并释放控制权。

4. **视觉订阅**: 使用 `ZMQMultipartImageSubscriber`
   ```python
   from common.zmq_subscriber import ZMQMultipartImageSubscriber
   sub = ZMQMultipartImageSubscriber("tcp://localhost:5560")
   frame_id, frame = sub.read_frame()
   ```

5. **日志**: 使用 `get_logger(__name__)`

6. **配置**: 通过 `get_config()` 读取，不要硬编码

7. **启动验证**:
   ```bash
   cd software/src
   python -m applications.my_app
   ```

---

## 修改指南

### 调整控制优先级
- 修改 `PRIORITIES` 字典时，需同步更新:
  1. `services/motion_service/chassis_arbiter/arbiter.py`
  2. `services/motion_service/chassis_service.py`（如底盘有独立副本）
  3. `services/motion_service/arm_service.py`
  4. `software/src/services/AGENTS.md` 和本文档

### 修改应用通信协议
- 所有 ZMQ 地址从 `get_config().zmq.xxx_addr` 读取
- 应用层订阅者**禁止使用 `zmq.CONFLATE`**，必须使用后台线程基类

### 网页遥控前端修改
- 前端文件位于 `applications/remote_control/templates/` 或 `static/`
- 后端 SocketIO 事件处理在 `web_server.py` 中
- 新增按钮/摇杆需同步修改前端 JS 和后端事件处理器

---

## 人类文档索引

| 主题 | 对应人类文档 (`docs/`) |
|------|----------------------|
| 网页控制端 | `网页控制端使用指南.md`, `Web控制界面介绍.md` |
| 游戏手柄 | `游戏手柄控制使用指南.md`, `游戏手柄控制快速参考.md` |
| 人体跟随 | `人体跟随使用指南.md`, `人体检测与跟随方案.md` |
| 语音交互 | `语音交互使用指南.md` |
| 新应用开发 | `HomeBot新应用开发指南.md` |
