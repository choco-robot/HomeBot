<!-- From: d:\develop\homebot\software\src\configs\AGENTS.md -->
# HomeBot 配置体系 Agent 文档

本文档面向 AI 编程助手，提供配置模块的全量字段速查、修改指南和密钥管理说明。

> 根级项目文档参见 `d:\develop\homebot\AGENTS.md`
> 源码总览参见 `software/src/AGENTS.md`

---

## 配置体系概览

```
configs/
├── config.py      # 所有配置 Dataclass（15+ 个），单例访问
├── secrets.py     # API 密钥管理，环境变量/.env 加载
└── __init__.py    # 导出 get_config, require_secrets 等
```

**访问方式**:
```python
from configs import get_config, require_secrets

config = get_config()
print(config.chassis.serial_port)

require_secrets("tts")  # 未配置则退出并提示
```

---

## 路径宏

| 常量 | 计算方式 | 说明 |
|------|---------|------|
| `PROJECT_ROOT` | `configs/` 向上回溯 3 级 | 项目根目录 |
| `SOFTWARE_DIR` | `PROJECT_ROOT/software` | 软件目录 |
| `HARDWARE_DIR` | `PROJECT_ROOT/hardware` | 硬件目录 |
| `DOCS_DIR` | `PROJECT_ROOT/docs` | 文档目录 |
| `MAPS_DIR` | `PROJECT_ROOT/software/maps` | 地图目录 |
| `MODELS_DIR` | `PROJECT_ROOT/software/models` | 模型目录 |

---

## 配置 Dataclass 全量字段速查

### CameraConfig

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `device_id` | `int` | `0` | 摄像头设备索引 |
| `width` | `int` | `1920` | 原始分辨率宽度 |
| `height` | `int` | `1080` | 原始分辨率高度 |
| `fps` | `int` | `30` | 帧率 |

### ArmConfig

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `serial_port` | `str` | `"COM9"` | 串口号（与底盘共用） |
| `baudrate` | `int` | `1000000` | 波特率 |
| `base_id` | `int` | `1` | 基座关节舵机 ID |
| `shoulder_id` | `int` | `2` | 肩关节 ID |
| `elbow_id` | `int` | `3` | 肘关节 ID |
| `wrist_flex_id` | `int` | `4` | 腕屈伸 ID |
| `wrist_roll_id` | `int` | `5` | 腕旋转 ID |
| `gripper_id` | `int` | `6` | 夹爪 ID |
| `upper_arm_length` | `float` | `115.0` | 大臂长度 mm ⚠️ **人工设置，AI勿动** |
| `forearm_length` | `float` | `130.0` | 小臂长度 mm ⚠️ **人工设置，AI勿动** |
| `joint_limits` | `dict` | 见代码 | 各关节角度限制 ° ⚠️ **人工设置，AI勿动** |
| `default_speed` | `int` | `1000` | 默认速度 |
| `default_acc` | `int` | `50` | 默认加速度 |
| `rest_position` | `dict` | 见代码 | 休息/待机位置 ° ⚠️ **人工设置，AI勿动** |

### ChassisConfig

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `chassis_type` | `str` | `"diff"` | `"omni"` 三轮全向 / `"diff"` 双轮差动 |
| `serial_port` | `str` | `"/dev/tty.usbmodem..."` | 串口 |
| `baudrate` | `int` | `1000000` | 波特率 |
| `left_front_id` | `int` | `8` | 左前轮舵机 ID（omni） |
| `right_front_id` | `int` | `7` | 右前轮舵机 ID（omni） |
| `rear_id` | `int` | `9` | 后轮舵机 ID（omni） |
| `wheel_radius` | `float` | `0.08` | 轮子半径 m（omni） |
| `chassis_radius` | `float` | `0.18` | 底盘半径 m（omni） |
| `default_wheel_speed` | `int` | `3250` | 舵机最大速度（omni） |
| `wheel_track` | `float` | `0.45` | 左右轮中心距 m（diff） |
| `wheel_diameter` | `float` | `0.125` | 轮直径 m（diff） |
| `max_rpm` | `float` | `120.0` | 电机最大转速（diff） |
| `diff_chassis_id` | `int` | `0x24` | 底盘虚拟设备 ID（diff） |
| `diff_motor_left_id` | `int` | `0x21` | 左电机 ID（diff） |
| `diff_motor_right_id` | `int` | `0x22` | 右电机 ID（diff） |
| `diff_imu_id` | `int` | `0x23` | IMU 虚拟设备 ID（diff） |
| `max_linear_speed` | `float` | `0.5` | 最大线速度 m/s |
| `max_angular_speed` | `float` | `1.0` | 最大角速度 rad/s |
| `service_addr` | `str` | `"tcp://*:5556"` | 底盘服务 ZeroMQ 地址 |

### ZMQConfig

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `chassis_service_addr` | `"tcp://*:5556"` | 底盘服务 REP |
| `chassis_state_pub_addr` | `"tcp://*:5558"` | 底盘状态 PUB |
| `arm_service_addr` | `"tcp://*:5557"` | 机械臂服务 |
| `vision_pub_addr` | `"tcp://*:5560"` | 视觉图像 PUB |
| `depth_pub_addr` | `"tcp://*:5561"` | 深度图 PUB |
| `obstacle_pub_addr` | `"tcp://*:5562"` | 障碍物信息 PUB |
| `odom_pub_addr` | `"tcp://*:5559"` | 里程计 PUB |
| `slam_pose_pub_addr` | `"tcp://*:5563"` | SLAM 位姿 PUB |
| `slam_map_pub_addr` | `"tcp://*:5564"` | SLAM 地图 PUB |
| `lidar_scan_pub_addr` | `"tcp://*:5565"` | 激光扫描 PUB |
| `goal_pub_addr` | `"tcp://*:5566"` | 目标点 PUB |
| `odom_cmd_addr` | `"tcp://*:5567"` | 里程计命令 REP |
| `slam_cmd_addr` | `"tcp://*:5568"` | SLAM 命令 REP |
| `speech_service_addr` | `"tcp://*:5570"` | 语音服务备用 |
| `wakeup_pub_addr` | `"tcp://*:5571"` | 唤醒+ASR PUB |

### NavigationConfig

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_linear_speed` | `float` | `0.8` | 导航最大线速度 m/s |
| `max_angular_speed` | `float` | `0.8` | 导航最大角速度 rad/s |
| `arrival_distance_threshold_m` | `float` | `0.1` | 到达距离阈值 m |
| `arrival_angle_threshold_rad` | `float` | `0.1` | 到达角度阈值 rad |
| `use_depth_obstacle` | `bool` | `False` | 是否启用深度视觉避障 |
| `emergency_obstacle_distance_m` | `float` | `0.3` | 紧急避障距离 m |
| `safety_distance_m` | `float` | `0.5` | VFH 安全距离 m |
| `max_replan_attempts` | `int` | `5` | 全局规划最大重试次数 |
| `replan_interval_s` | `float` | `3.0` | 重规划间隔 s |
| `lookahead_distance_m` | `float` | `0.5` | 路径跟踪前瞻距离 m |
| `max_path_deviation_m` | `float` | `1.0` | 允许偏离全局路径最大距离 m |
| `inflation_radius_m` | `float` | `0.2` | 障碍物膨胀半径 m |
| `robot_radius_m` | `float` | `0.25` | 机器人半径 m |
| `enable_apriltag` | `bool` | `False` | 是否启用 AprilTag 检测 |
| `control_rate_hz` | `float` | `10.0` | 控制循环频率 Hz |
| `max_angular_accel_rad` | `float` | `2.0` | 最大角加速度 rad/s² |
| `velocity_filter_alpha` | `float` | `0.4` | 一阶低通滤波系数 |

### SLAMConfig

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lidar_port` | `str` | `"/dev/tty.usbserial-14130"` | 激光雷达串口 |
| `lidar_scan_size` | `int` | `360` | 扫描点数 |
| `lidar_max_distance_m` | `float` | `12.0` | 最大检测距离 m |
| `lidar_min_distance_m` | `float` | `0.2` | 最小检测距离 m |
| `map_size_pixels` | `int` | `800` | 栅格地图像素尺寸 |
| `map_size_meters` | `float` | `10.0` | 地图物理尺寸 m |
| `tag_family` | `str` | `"tag36h11"` | AprilTag 族 |
| `tag_size_m` | `float` | `0.165` | 标签边长 m |
| `tag_map` | `dict` | `{}` | 标签 ID → 世界位姿映射 |
| `camera_fx` / `fy` / `cx` / `cy` | `float` | `600/600/320/240` | 相机内参 |
| `confidence_threshold` | `float` | `0.8` | 硬校正触发阈值 |
| `odom_consistency_threshold` | `float` | `9.21` | 里程计一致性卡方阈值 |

### ViserConfig

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `host` | `str` | `"0.0.0.0"` | Viser 服务绑定主机 |
| `port` | `int` | `8080` | Viser 服务端口 |
| `odom_sub_addr` | `str` | `"tcp://localhost:5559"` | 里程计订阅 |
| `slam_pose_sub_addr` | `str` | `"tcp://localhost:5563"` | SLAM 位姿订阅 |
| `slam_map_sub_addr` | `str` | `"tcp://localhost:5564"` | SLAM 地图订阅 |
| `lidar_scan_sub_addr` | `str` | `"tcp://localhost:5565"` | 激光雷达订阅 |
| `vision_sub_addr` | `str` | `"tcp://localhost:5560"` | 视觉图像订阅 |
| `goal_pub_addr` | `str` | `"tcp://*:5566"` | 目标点发布 |
| `odom_cmd_addr` | `str` | `"tcp://localhost:5567"` | 里程计命令 REQ |
| `slam_cmd_addr` | `str` | `"tcp://localhost:5568"` | SLAM 命令 REQ |
| `global_path_sub_addr` | `str` | `"tcp://localhost:5569"` | 全局路径订阅 |
| `nav_status_sub_addr` | `str` | `"tcp://localhost:5570"` | 导航状态订阅 |
| `maps_dir` | `str` | `""` | 地图文件夹（空则自动探测） |
| `urdf_path` | `str` | `HARDWARE_DIR/...` | 机器人 URDF 路径 |
| `urdf_color_override` | `tuple` | `None` | URDF 颜色覆盖 RGB |
| `lidar_rotation_offset_deg` | `float` | `180.0` | 激光雷达 0° 零位偏移 |
| `max_trajectory_points` | `int` | `5000` | 最大轨迹点数 |
| `point_size` | `float` | `0.03` | 激光点大小 |
| `map_update_interval` | `float` | `2.0` | 地图更新间隔 s |
| `follow_robot` | `bool` | `True` | 默认视角跟随机器人 |

### GamepadConfig

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_linear_speed` | `float` | `0.5` | 底盘最大线速度 m/s |
| `max_angular_speed` | `float` | `1.0` | 底盘最大角速度 rad/s |
| `trigger_deadzone` | `float` | `0.1` | 扳机键死区 |
| `left_stick_deadzone` | `float` | `0.15` | 左摇杆死区 |
| `right_stick_deadzone` | `float` | `0.15` | 右摇杆死区 |
| `arm_base_step` | `float` | `3.0` | 基座关节步进 °/帧 |
| `arm_elbow_step` | `float` | `2.0` | 肘关节步进 °/帧 |
| `arm_shoulder_step` | `float` | `2.0` | 肩关节步进 °/帧 |
| `arm_wrist_flex_step` | `float` | `3.0` | 腕屈伸步进 °/次 |
| `arm_wrist_roll_step` | `float` | `3.0` | 腕旋转步进 °/帧 |
| `arm_gripper_open` | `float` | `90.0` | 夹爪打开角度 |
| `arm_gripper_close` | `float` | `0.0` | 夹爪关闭角度 |
| `arm_speed` | `int` | `800` | 机械臂运动速度 |
| `chassis_service_addr` | `str` | `"tcp://localhost:5556"` | 底盘服务地址 |
| `arm_service_addr` | `str` | `"tcp://localhost:5557"` | 机械臂服务地址 |
| `polling_interval` | `float` | `0.02` | 轮询间隔 s（50Hz） |

### HumanFollowConfig

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `model_path` | `str` | `"models/yolo26n.onnx"` | YOLO 模型路径 |
| `conf_threshold` | `float` | `0.5` | 检测置信度阈值 |
| `max_tracking_age` | `int` | `30` | 最大丢失帧数 |
| `min_iou_threshold` | `float` | `0.3` | IoU 匹配阈值 |
| `target_selection` | `str` | `"center"` | `center`/`largest`/`closest` |
| `inference_size` | `int` | `320` | 输入分辨率 |
| `use_half_precision` | `bool` | `False` | FP16 推理 |
| `target_distance` | `float` | `1.0` | 目标跟随距离 m |
| `kp_linear` | `float` | `0.8` | 线速度 P 系数 |
| `kp_angular` | `float` | `1.5` | 角速度 P 系数 |
| `max_linear_speed` | `float` | `0.5` | 最大线速度 m/s |
| `max_angular_speed` | `float` | `2.0` | 最大角速度 rad/s |
| `dead_zone_x` | `float` | `0.15` | 水平死区（比例） |
| `dead_zone_area` | `float` | `0.1` | 面积死区（相对值） |
| `lost_patience` | `int` | `30` | 丢失容忍帧数 |
| `stop_on_lost` | `bool` | `True` | 丢失目标时是否停止 |
| `search_on_lost` | `bool` | `False` | 丢失时是否旋转搜索 |
| `chassis_service_addr` | `str` | `"tcp://localhost:5556"` | 底盘服务地址 |
| `vision_sub_addr` | `str` | `"tcp://localhost:5560"` | 视觉订阅地址 |

### BatteryConfig

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `servo_ids` | `list` | `[1]` | 读取电压的舵机 ID 列表 |
| `full_voltage` | `float` | `12.6` | 满电电压 V |
| `low_voltage` | `float` | `10.5` | 低电量阈值 V |
| `critical_voltage` | `float` | `9.5` | 严重低电量阈值 V |
| `min_voltage` | `float` | `9.0` | 最低工作电压 V |
| `publish_interval` | `float` | `5.0` | 发布间隔 s |
| `pub_addr` | `str` | `"tcp://*:5555"` | 电池状态 PUB 地址 |

---

## 密钥管理 (secrets.py)

### 环境变量加载

模块导入时自动执行 `_load_all_env_files()`，按优先级加载：

| 优先级 | 文件 | 说明 |
|--------|------|------|
| 1 | `.env.local` | 最高优先级，已加入 `.gitignore` |
| 2 | `.env.development` | 开发环境 |
| 3 | `.env.production` | 生产环境 |
| 4 | `.env` | 通用模板 |

**规则**: 已存在的环境变量不会被文件覆盖（允许系统环境变量优先）。

### 环境变量映射表

| 配置项 | 主环境变量 | 回退环境变量 | 默认值 |
|--------|-----------|-------------|--------|
| `tts.appid` | `VOLCANO_APPID` | `TTS_APPID` | `""` |
| `tts.access_token` | `VOLCANO_ACCESS_TOKEN` | `TTS_ACCESS_TOKEN` | `""` |
| `llm.api_key` | `ARK_API_KEY` | `VOLCANO_API_KEY` → `DEEPSEEK_API_KEY` → `LLM_API_KEY` | `""` |
| `llm.api_url` | `ARK_API_URL` | `LLM_API_URL` | `"https://ark.cn-beijing.volces.com/api/v3"` |
| `llm.model` | `ARK_MODEL_ID` | `VOLCANO_MODEL_ID` → `DEEPSEEK_MODEL` → `LLM_MODEL` | `""` |
| `vision.provider` | `VISION_PROVIDER` | — | `"deepseek"` |
| `vision.api_key` | `VISION_API_KEY` | （provider=deepseek 时复用 LLM） | `""` |

### 公共函数

| 函数 | 用途 |
|------|------|
| `get_secrets()` | 获取 Secrets 单例（首次调用时加载） |
| `require_secrets("tts"\|"llm"\|"vision")` | 强制检查，缺失则 `sys.exit(1)` 并打印帮助 |
| `check_secrets(verbose=True)` | 返回状态字典并打印报告（密钥已脱敏） |
| `reload_secrets()` | 运行时重新加载 `.env` 文件 |

---

## 配置模块依赖关系

```
configs/secrets.py
    ├── 被 configs/config.py 导入
    │       ├── TTSConfig.__post_init__ → get_secrets().tts
    │       ├── LLMConfig.__post_init__ → get_secrets().llm
    │       └── VisionConfig.__post_init__ → get_secrets().vision + secrets.llm
    │
    └── 被 common/logging.py 导入（fallback）
            └── get_logger 尝试读取 Config().logging.level
```

**关键注意事项**:
1. `secrets.py` 模块导入时自动加载 `.env*` 到 `os.environ`
2. `TTSConfig`/`LLMConfig`/`VisionConfig` 在 `__post_init__` 中**懒加载** secrets
3. `VisionConfig` 对 `deepseek` provider 有**硬编码回退**: 未单独配置时自动复用 `secrets.llm.api_key`
4. `common/logging.py` 用 `try/except` 导入 `Config` 避免循环依赖

---

## 修改指南

### Agent 可安全修改的配置

✅ **算法参数**: `NavigationConfig` 中的阈值、距离、频率、滤波系数
✅ **速度限制**: `ChassisConfig.max_linear_speed`, `max_angular_speed`
✅ **ZMQ 地址**: 所有端口（需同步更新 `services/AGENTS.md` 和 `navigation/AGENTS.md`）
✅ **日志级别**: `LoggingConfig.level`
✅ **检测阈值**: `HumanFollowConfig.conf_threshold`, `inference_size`
✅ **死区/步长**: `GamepadConfig` 中的各类死区和步进值

### ⚠️ 人工设置，AI 勿动

🚫 **机械臂几何参数**: `ArmConfig.upper_arm_length`, `forearm_length` — 修改后需重新校准
🚫 **机械臂关节限制**: `ArmConfig.joint_limits` — 与物理结构绑定
🚫 **机械臂休息位**: `ArmConfig.rest_position` — 需确保不碰撞
🚫 **底盘舵机 ID**: `ChassisConfig.left_front_id`, `right_front_id`, `rear_id` — 与接线绑定
🚫 **相机内参**: `SLAMConfig.camera_fx/fy/cx/cy` — 需标定

### 新增配置项

1. 在 `config.py` 中新建或扩展现有 dataclass
2. 提供合理的默认值和类型注解
3. 在本文档的对应表格中登记
4. 如果涉及敏感信息，在 `secrets.py` 中新增对应字段和 `__post_init__` 加载逻辑

### 修改 ZMQ 端口

1. 修改 `ZMQConfig` 中的对应字段
2. 同步修改所有硬编码了该地址的代码（搜索 `tcp://*:PORT` 和 `tcp://localhost:PORT`）
3. 同步更新:
   - `software/src/AGENTS.md` 端口速查表
   - `software/src/services/AGENTS.md` 端口总表和数据流图
   - `software/src/navigation/AGENTS.md` 相关引用
   - 根 `AGENTS.md` 启动命令示例

---

*最后更新：2026-06-10*
