<!-- From: d:\develop\homebot\software\src\common\AGENTS.md -->
# HomeBot 公共模块 Agent 文档

本文档面向 AI 编程助手，提供 `common/` 目录下公共工具的使用速查和关键规范。

> 根级项目文档参见 `d:\develop\homebot\AGENTS.md`
> 源码总览参见 `software/src/AGENTS.md`

---

## 模块速查

| 文件 | 公共 API | 职责 |
|------|---------|------|
| `logging.py` | `get_logger(name)` | 创建带统一格式的 `StreamHandler` 日志器 |
| `messages.py` | `MessageType`, `serialize()`, `deserialize()` | 系统消息类型枚举和 JSON 序列化 |
| `zmq_helper.py` | `create_socket()`, `send_json()`, `recv_json()` | ZMQ socket 创建和 JSON 收发封装 |
| `zmq_subscriber.py` | `ZMQJsonSubscriber`, `ZMQMultipartJsonSubscriber`, `ZMQMultipartImageSubscriber` | 后台线程订阅者基类 |
| `transform.py` | `pose2_to_matrix()`, `world_to_robot2()`, `pose3_to_matrix()` 等 | 2D/3D 齐次坐标变换 |

---

## 日志 (logging.py)

```python
from common.logging import get_logger
logger = get_logger(__name__)
logger.info("启动完成")
```

**日志级别优先级**:
1. `HOMEBOT_LOG_LEVEL` 环境变量
2. `Config().logging.level`
3. 默认 `"DEBUG"`

**格式**: `%(asctime)s [%(levelname)s] %(name)s: %(message)s`

---

## 消息类型 (messages.py)

| 成员 | 值 | 含义 |
|------|-----|------|
| `CMD_VELOCITY` | `"cmd.velocity"` | 速度控制命令 |
| `CMD_ARM_JOINT` | `"cmd.arm.joint"` | 机械臂关节命令 |
| `DETECTION_HUMAN` | `"detection.human"` | 人体检测结果 |
| `BATTERY_STATE` | `"sensor.battery"` | 电池状态 |

```python
from common.messages import MessageType, serialize, deserialize

payload = serialize(MessageType.CMD_VELOCITY, {"vx": 0.5, "vy": 0.0, "vz": 0.3})
data = deserialize(json_string)
```

---

## ZMQ 辅助 (zmq_helper.py)

| 函数 | 签名 | 用途 |
|------|------|------|
| `create_context` | `() -> zmq.Context` | 获取全局 ZMQ Context 单例 |
| `create_socket` | `(socket_type, bind, address, context=None) -> zmq.Socket` | 创建并绑定/连接 socket |
| `send_json` | `(socket, payload) -> None` | 发送 JSON |
| `recv_json` | `(socket) -> Any` | 接收并反序列化 JSON |

---

## ZMQ 订阅者基类 (zmq_subscriber.py)

> ⚠️ **关键规范（CONFLATE 规则）**
> - **应用层订阅者禁止使用 `zmq.CONFLATE`**。必须使用后台线程持续接收 + 锁保护最新数据。
> - **服务层内部消费**（如 OdomService 50Hz 主循环）**可使用 CONFLATE**，因其本身就是持续 `recv`。
> - 同一数据话题的订阅逻辑不要重复实现，优先使用本模块提供的基类。

### ZMQJsonSubscriber

订阅 JSON 单帧消息（如 OdomService 的里程计数据）。

```python
from common.zmq_subscriber import ZMQJsonSubscriber

sub = ZMQJsonSubscriber("tcp://localhost:5559", required_keys=("x", "y", "yaw"))
data = sub.read()          # 非阻塞，线程安全，返回最新数据或 None
stats = sub.get_stats()    # {"recv_count": int, "has_data": bool}
sub.close()
```

**构造参数**:
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `sub_addr` | `str` | — | 订阅地址 |
| `required_keys` | `tuple[str, ...]` | `()` | 必填字段，缺失则丢弃消息 |
| `rcv_timeout_ms` | `int` | `500` | `zmq.RCVTIMEO` 毫秒 |

### ZMQMultipartJsonSubscriber

订阅 multipart 消息，提取第 N 帧为 JSON（如 DepthService 障碍物 `[frame_id, json_payload]`）。

```python
from common.zmq_subscriber import ZMQMultipartJsonSubscriber

sub = ZMQMultipartJsonSubscriber(
    "tcp://localhost:5562",
    required_keys=("histogram",),
    json_frame_index=1    # 第 2 个 frame 是 JSON
)
data = sub.read()
```

### ZMQMultipartImageSubscriber

订阅 `[frame_id_str, jpeg_bytes]` 图像流（如 VisionService）。

```python
from common.zmq_subscriber import ZMQMultipartImageSubscriber

sub = ZMQMultipartImageSubscriber("tcp://localhost:5560")
frame_id, frame = sub.read_frame()   # frame 为 BGR numpy 数组或 None
```

---

## 坐标变换 (transform.py)

### 类型别名

| 别名 | 含义 |
|------|------|
| `Point2` | `(x, y)` 或 `np.ndarray` |
| `Pose2` | `(x, y, yaw)` 米/米/弧度 |
| `Point3` | `(x, y, z)` |
| `Pose3` | `(x, y, z, roll, pitch, yaw)` 弧度 |
| `Matrix4` | `4×4` 齐次变换矩阵 `np.ndarray` |

### 2D 变换函数

| 函数 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `pose2_to_matrix(pose)` | `(x, y, yaw)` | `4×4` 矩阵 | 二维位姿 → 齐次矩阵 |
| `matrix_to_pose2(T)` | `4×4` 矩阵 | `(x, y, yaw)` | 齐次矩阵 → 二维位姿 |
| `inverse_matrix(T)` | `4×4` 矩阵 | `4×4` 矩阵 | 优化求逆（利用 R^T） |
| `transform_point2(T, point)` | 矩阵, `(x, y)` | `(x, y)` | 变换二维点 |
| `transform_pose2(T, pose)` | 矩阵, `(x, y, yaw)` | `(x, y, yaw)` | 变换二维位姿（旋转叠加） |
| `world_to_robot2(point, robot_pose)` | 世界点, 机器人位姿 | `(x, y)` | 世界 → 机器人坐标系 |
| `robot_to_world2(point, robot_pose)` | 机器人点, 机器人位姿 | `(x, y)` | 机器人 → 世界坐标系 |
| `world_to_robot_pose2(pose, robot_pose)` | 两位姿 | 位姿 | 世界位姿 → 机器人位姿 |
| `robot_to_world_pose2(pose, robot_pose)` | 两位姿 | 位姿 | 机器人位姿 → 世界位姿 |

### VFH 坐标系（已与机器人坐标系统一）

| 函数 | 说明 |
|------|------|
| `robot_to_vfh2(point)` | 恒等映射 |
| `vfh_to_robot2(point)` | 恒等映射 |
| `world_to_vfh2(point, robot_pose)` | 内部调用 `world_to_robot2` |
| `vfh_to_world2(point, robot_pose)` | 内部调用 `robot_to_world2` |

### 3D 变换函数

| 函数 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `pose3_to_matrix(pose)` | `(x,y,z,r,p,y)` | `4×4` 矩阵 | ZYX 欧拉顺序 |
| `matrix_to_pose3(T)` | `4×4` 矩阵 | `(x,y,z,r,p,y)` | — |
| `transform_point3(T, point)` | 矩阵, `(x,y,z)` | `(x,y,z)` | — |
| `transform_pose3(T, pose)` | 矩阵, 位姿 | 位姿 | 旋转复合 |
| `translation_matrix(x,y,z)` | 平移分量 | `4×4` 矩阵 | 纯平移 |
| `rotation_matrix_z(yaw)` | 偏航角 | `4×4` 矩阵 | 绕 Z 旋转 |
| `rotation_matrix_from_euler(r,p,y,seq='ZYX')` | 欧拉角 | `4×4` 矩阵 | 默认 ZYX 顺序 |

### 坐标系约定

- **世界坐标系**: X 向右，Y 向上（俯视图），yaw 从 X 正方向逆时针
- **机器人/底盘/VFH**: x 前进为正，y 左侧为正，yaw 逆时针
- **三维旋转顺序**: ZYX（yaw → pitch → roll）

---

## 修改指南

### 新增消息类型

1. 在 `messages.py` 的 `MessageType` Enum 中新增成员
2. 在 `serialize()`/`deserialize()` 的文档字符串中说明新类型
3. 在所有使用方的 AGENTS.md 中更新消息格式说明

### 新增 ZMQ 订阅者变体

1. 优先继承 `ZMQJsonSubscriber` 或 `ZMQMultipartJsonSubscriber`
2. 覆盖 `_validate()` 方法实现自定义数据校验
3. **永远不要**在应用层直接使用 `zmq.CONFLATE`

### 新增坐标变换

1. 优先复用现有 `pose2_to_matrix` / `matrix_to_pose2` / `inverse_matrix` 组合
2. 新增函数时遵循命名规范：`{src}_to_{dst}{dim}()`
3. 在文档字符串中明确坐标系定义

---

*最后更新：2026-06-10*
