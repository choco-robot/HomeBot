<!-- From: d:\develop\homebot\software\src\navigation\AGENTS.md -->
# HomeBot 导航模块 Agent 文档

本文档面向 AI 编程助手，提供导航与 SLAM 子系统的模块速查、坐标系定义、算法说明和修改指南。

> 根级项目文档参见 `d:\develop\homebot\AGENTS.md`
> 服务层端口和消息格式参见 `software/src/services/AGENTS.md`

---

## 模块速查

### 目录结构

```
navigation/
├── core/                    # SLAM 核心算法
│   ├── occupancy_grid.py    # 占用栅格地图数据结构
│   ├── astar_planner.py     # A* 全局路径规划
│   ├── slam_fusion.py       # BreezySLAM + AprilTag 融合定位
│   └── utils.py             # 坐标转换工具函数
├── perception/              # 感知
│   ├── apriltag_detector.py # AprilTag 检测 + PnP 位姿解算
│   ├── depth_estimator.py   # MiDaS 单目深度估计
│   └── obstacle_detector.py # 深度障碍物直方图
├── planning/                # 规划
│   ├── costmap_generator.py # 局部代价地图生成
│   └── local_planner.py     # VFH 局部避障控制器
├── coordinator/             # 导航协调器
│   └── navigation_coordinator.py  # 状态机 + 全局/局部规划协调
├── services/                # 导航服务（详见 services/AGENTS.md）
│   ├── odom_service.py      # 里程计积分服务
│   ├── slam_service.py      # SLAM 融合定位服务
│   └── depth_service.py     # 深度估计服务
├── applications/            # 导航应用
│   └── navigation/app.py    # 全局自主导航应用
├── simulation/              # Matplotlib 2D 仿真器
│   ├── simulator.py         # 主仿真器（物理+传感器线程）
│   ├── robot_model.py       # 差动机器人运动学模型
│   ├── map_environment.py   # 仿真环境/地图管理
│   ├── laser_scanner.py     # 2D 激光雷达仿真（射线投射）
│   └── sim_visualizer.py    # Matplotlib 可视化
├── visualization/           # Viser 3D 可视化
│   └── viser_slam_visualizer.py  # RViz 替代，Web 3D 实时显示
└── hal/
    └── lidar_driver.py      # LD06/LD19 激光雷达串口驱动
```

### 核心类一览

| 文件 | 主类 | 职责 |
|------|------|------|
| `core/occupancy_grid.py` | `OccupancyGrid` | 2D 栅格地图：世界↔栅格转换、障碍物膨胀、序列化 |
| `core/astar_planner.py` | `AStarPlanner` | A* 全局规划 + 路径简化（滑动窗口+Bresenham） |
| `core/slam_fusion.py` | `SLAMFusion` | BreezySLAM + AprilTag Split-CIF 融合、绑架恢复 |
| `perception/apriltag_detector.py` | `AprilTagDetector` | `tag36h11` 检测 + `solvePnP` 位姿解算 |
| `perception/depth_estimator.py` | `DepthEstimator` | MiDaS-small ONNX 单目深度估计 |
| `perception/obstacle_detector.py` | `DepthObstacleDetector` | 深度图 → 1D 距离直方图（VFH 输入） |
| `planning/costmap_generator.py` | `LocalCostmapGenerator` | 深度障碍物投影到机器人中心局部栅格 |
| `planning/local_planner.py` | `VFHLocalPlanner` | 简化 VFH：直方图 → 谷地选择 → (vx, vz) |
| `coordinator/navigation_coordinator.py` | `NavigationCoordinator` | 导航状态机、目标队列、纯追踪 + 局部避障 |
| `applications/navigation/app.py` | `NavigationApp` | ZMQ 桥接：传感器流 → Coordinator → 底盘指令 |
| `visualization/viser_slam_visualizer.py` | `ViserSLAMVisualizer` | Viser Web 3D 可视化：坐标系、点云、地图、轨迹 |
| `simulation/simulator.py` | `Simulator` | 2D 物理+传感器仿真器，100Hz 物理 / 10Hz 传感器 |
| `simulation/robot_model.py` | `DifferentialRobot` | 差动运动学 + 噪声/打滑模拟 |
| `simulation/laser_scanner.py` | `LaserScanner` | 360 射线射线投射（圆/多边形/栅格） |
| `hal/lidar_driver.py` | `LD06Driver` | LD06/LD19 串口协议解析（Python 纯实现） |

---

## 坐标系定义

### 世界坐标系 (World / Map)
- 原点: 地图中心 `(0, 0)`
- X: 向右（东）
- Y: 向上（北，俯视图）
- Yaw: 从 X 轴正方向逆时针旋转，单位弧度

### 栅格坐标系 (Grid)
- 索引 `(gx, gy)`：`(0, 0)` 在左上角
- X 向右，Y 向下（与图像坐标一致）
- 通过 `OccupancyGrid.world_to_grid()` / `grid_to_world()` 转换

### 机器人坐标系 (Robot / VFH)
- 原点: 机器人底盘中心
- x: **前进方向**（正前）
- y: **左方**（正左）
- Yaw: 逆时针从 x 轴正方向
- VFH 直方图扇区与机器人坐标系对齐：扇区 0 对应正前方，向左递增

> 注意：`transform.py` 中 `robot_to_vfh2` / `vfh_to_robot2` 是恒等映射，因为 VFH 已统一为机器人坐标系。

### 摄像头坐标系
- Z: 向前（光轴方向）
- X: 向右
- Y: 向下
- 通过 `camera_to_robot_tf`（4×4 外参矩阵）转换到机器人坐标系

### Viser TF 树
```
/map (fixed)
  └── /map/odom_frame (里程计漂移帧)
        └── /map/base_link (机器人，由 SLAM 位姿更新)
              ├── /map/base_link/laser (偏移 z=0.10)
              └── /map/base_link/camera (偏移 x=0.10, z=0.65, 绕 X 旋转 90°)
  └── /map/goal (交互目标点，带 transform controls)
```

---

## 核心算法

### 占用栅格地图 (OccupancyGrid)

**关键常量**:
```python
COST_FREE = 0
COST_UNKNOWN = -1
COST_OCCUPIED = 100
COST_LETHAL = 255
```

**障碍物膨胀** (`inflate_obstacles(inflation_radius_m)`):
- 使用 `scipy.ndimage.distance_transform_edt`
- 从 `COST_LETHAL`（中心）线性递减到 `COST_OCCUPIED`（边缘）
- 膨胀半径影响路径规划的可通行区域

**BreezySLAM 语义映射**:
- BreezySLAM 内部: `255=free, 0=occupied`
- 转换为 `OccupancyGrid`: `255 → COST_FREE`, `0 → COST_LETHAL`
- 映射发生在 `NavigationApp._bytes_to_occupancy_grid()` 和 `SLAMFusion` 中

### A* 全局规划器 (AStarPlanner)

**算法**:
- 8 邻域扩展，最小堆维护 `open_set`
- 障碍物阈值: `COST_OCCUPIED`（默认），`COST_UNKNOWN` 默认可通行

**路径简化** (`_simplify_path`):
- 滑动窗口大小: 20
- Bresenham 直线可达性检测
- 最大偏离阈值: `0.15` m
- 将稠密路径压缩为少量直线段

### SLAM 融合 (SLAMFusion)

**架构**:
- **高频激光线程** (~10Hz): BreezySLAM 输出相对位姿 + 协方差 `P_slam`
- **低频视觉线程** (~2Hz): AprilTag PnP 解算绝对位姿 + 协方差 `P_tag`
- **融合**: Split CIF（协方差交集融合），权重与 `tr(P)^-1` 成正比

**关键常量**:
```python
CHI2_3DOF_999 = 16.27          # 99.9% 置信度卡方阈值
KIDNAP_FAIL_FRAMES = 5         # 连续失败帧数触发绑架恢复
Q_SCALE_XY = 0.01
Q_BASE_XY = 0.001
Q_SCALE_THETA = 0.05
Q_BASE_THETA = 0.005
max_var_xy = 2.0               # m²
max_var_theta = 1.0            # rad²
```

**一致性检验**:
- Mahalanobis 距离 vs 卡方阈值 (`odom_consistency_threshold=9.21`，99% 置信度)
- 不一致时降低融合权重，避免异常观测污染位姿

**绑架恢复**:
- 触发条件: 连续 `KIDNAP_FAIL_FRAMES` 帧失败 + 检测到 AprilTag
- 动作: 全局重定位，重置 SLAM 粒子到 Tag 位姿

**位姿重置 (`reset_pose`)**:
- 动作: 设置融合位姿并同步到底层 BreezySLAM 粒子群
- 额外效果: 清空内部里程计缓存和积分累计量，防止下一帧把重置前的运动一次性灌给 SLAM
- 联动: SLAMService 在收到 `reset_pose` 命令后，会同步向 OdomService 和 ChassisService 发送复位指令

**清空地图 (`reset_map`)**:
- 动作: 重新实例化底层 `BreezySLAMWrapper`，彻底清空栅格地图和粒子群历史
- 位姿: 默认保持当前融合位姿，可通过 `x/y/theta` 参数指定新位姿
- 协方差: 重置为较小值，状态恢复为 `NORMAL`
- 用途: Viser 前端“清空地图”按钮，方便在同一运行会话中重新开始建图

**坐标单位**:
- BreezySLAM 内部: **mm / deg**，相对地图左上角
- `SLAMFusion` 暴露: **m / rad**，世界坐标系，原点在地图中心
- `_map_center_mm = 500 * map_size_meters`（如 10m 地图 → 5000mm）

### VFH 局部规划器 (VFHLocalPlanner)

**输入**:
- `obstacles: np.ndarray` — 距离直方图（米，`inf` = 无障碍）
- `goal_x, goal_y` — 目标在机器人坐标系下（前进=+x，左=+y）

**输出**: `(vx, vz)` 速度指令

**算法步骤**:
1. 构建障碍数组: `histogram < safety_distance_m`
2. 寻找谷地（连续未阻塞扇区）
3. 选择最优谷地: 目标加权中心 (`goal_weight=0.8`)
4. 计算角速度: `vz = sector_diff * sector_diff_weight`（限幅）
5. 计算线速度: 前方阻塞或偏离目标时减速

**关键参数** (来自 `NavigationConfig`):
```python
num_sectors = 21
safety_distance_m = 0.5
goal_weight = 0.8
smooth_weight = 0.15
min_valley_width = 2
sector_diff_weight = 0.2
```

### 导航协调器 (NavigationCoordinator)

**状态机**:
```
IDLE → PLANNING → NAVIGATING → (OBSTACLE_AVOIDING / RECOVERY) → IDLE
```

**外部接口注入** (setter 方式，便于仿真和真机复用):
- `set_pose_provider(callable)` → 返回 `(x, y, theta)`
- `set_obstacle_provider(callable)` → 返回障碍物列表
- `set_velocity_sender(callable)` → 接受 `(linear, angular)` → bool
- `set_map_provider(callable)` → 返回 `OccupancyGrid`

**局部控制逻辑** (`_compute_velocity`):
- Pure Pursuit 追踪前瞻点 (`lookahead_distance=0.4` m)
- 角度误差 > π/3 → 停止并原地旋转
- 速度因子: `angle_factor × obstacle_factor × distance_factor`
- 减速区: `goal_reached_distance * 5`

**关键常量**:
```python
goal_reached_distance = 0.1       # m
goal_reached_angle = 0.1          # rad
replan_distance_threshold = 0.5   # m
max_replan_attempts = 3           # Coordinator 默认 / 5 in NavigationApp
control_frequency = 10.0          # Hz
obstacle_emergency_distance = 0.3 # m
inflation_radius = 0.25           # m
robot_radius = 0.2                # m
velocity_filter_alpha = 0.2       # 低通 IIR 滤波
max_angular_accel_rad = 2.0       # rad/s² 斜率限制
```

---

## 数据流

### 真机导航数据流

```
[ChassisService PUB 5558] ──► [OdomService SUB]
                                    │
                                    ▼ PUB 5559
[SLAMService SUB] ◄─────────────────┘
       │
       ├── SUB Vision 5560 ──► AprilTag 检测
       ├── SUB Odom 5559
       └── 内部 LiDAR HAL
       │
       ▼ PUB 5563 (Pose) / 5564 (Map) / 5565 (LidarScan)
[NavigationApp SUB]
       │
       ├── pose ──► Coordinator.set_pose_provider()
       ├── map ──► Coordinator.set_map_provider()
       ├── obstacle (DepthService PUB 5562) ──► Coordinator.set_obstacle_provider()
       │
       ▼ Coordinator._compute_velocity()
[ChassisService REQ 5556] (source="auto", priority=3)
       │
       ▼ PUB 5569 (Path) / 5570 (Status)
[ViserSLAMVisualizer SUB]
```

### 仿真数据流

```
[Simulator] ──► get_pose_provider() ──► [NavigationCoordinator]
[Simulator] ──► get_obstacle_provider() ──► [NavigationCoordinator]
[Simulator] ──► get_map_provider() ──► [NavigationCoordinator]
[NavigationCoordinator] ──► get_velocity_sender() ──► [Simulator.set_velocity()]
```

仿真器不经过 ZMQ，直接通过可调用对象注入 Coordinator。

---

## 启动命令速查

```bash
# 导航服务层
cd software/src
python -m navigation.services.odom_service
python -m navigation.services.slam_service --mock-lidar --mock-tag
python -m navigation.services

# 导航应用层
python -m navigation.applications.navigation
python -m navigation.applications.navigation --mode navigation
python -m navigation.applications.navigation --mode navigation --goal-sub tcp://localhost:5566

# Viser 3D 可视化
python -m navigation.visualization
python -m navigation.visualization --port 8080

# 仿真器
cd software
python demo_navi_sim.py
python demo_slam_sim.py
```

---

## 修改指南

### 调整导航参数

- **全局参数**: 修改 `configs.config.NavigationConfig` 中的字段
- **局部规划参数**: `VFHLocalPlanner` 的默认常量来自 `NavigationConfig`，可通过构造函数参数覆盖
- **SLAM 参数**: `configs.config.SLAMConfig` 中的 `map_size_pixels`, `map_size_meters`, `tag_map` 等

### 添加新传感器输入

1. 在 `navigation/services/` 下创建新服务（遵循 `python -m` 启动规范）
2. 定义新的 ZMQ PUB 地址，在 `ZMQConfig` 中注册
3. 在 `NavigationApp` 中新增订阅者和数据转换函数
4. 更新 `NavigationCoordinator` 的 setter 接口或直接在 App 中处理

### 修改坐标系或机器人模型

- **机器人尺寸**: 修改 `configs.config.ChassisConfig`（轮距、轮径、底盘半径）
- **摄像头外参**: 修改 `configs.config.SLAMConfig` 中的 `camera_to_robot_tf` 相关参数
- **激光雷达安装位置**: 修改 `ViserConfig.lidar_rotation_offset_deg` 和 Viser 中的 TF 偏移
- **坐标转换函数**: `common.transform.py` 提供 2D/3D 齐次变换，新增坐标系时优先复用

### 地图格式与互操作

- **保存/加载**: SLAMService 使用 `.npz` 格式（map bytes + pose）
- **导出编辑器**: `SLAMFusion.export_to_editor_format()` 导出 PNG + JSON
- **导入 ROS 地图**: `MapEnvironment` 支持 ROS YAML + PNG 格式
- **自定义地图**: `MapEnvironment.create_simple_room()` / `create_maze()` / `create_cluttered_room()`

### 仿真器与真机切换

- 仿真器通过 `Simulator.get_*_provider()` 返回的可调用对象直接注入 Coordinator
- 真机通过 `NavigationApp` 的 ZMQ 订阅桥接
- **核心原则**: `NavigationCoordinator` 不感知 ZMQ，只接收可调用对象。ZMQ 逻辑全部在 `NavigationApp` 中。

### 深度估计模型替换

- 默认: MiDaS-small ONNX (`models/midas/midas_small.onnx`)
- 替换: 修改 `depth_estimator.py` 中的 `DEFAULT_MODEL_PATH` 和预处理/后处理逻辑
- 输入: BGR 图像 `(H, W, 3)`
- 输出: 相对深度图 `(H, W)` float32，归一化到 `[0, 1]`（1=近，0=远，经反转后）

---

## 人类文档索引

| 主题 | 对应人类文档 (`docs/`) |
|------|----------------------|
| 导航系统架构设计 | `homebot_nav_design.md` |
| NavigationCoordinator 详细说明 | `NavigationCoordinator_详细说明.md` |
| 导航系统开发方案（5 阶段计划） | `导航系统开发方案.md` |
| Viser 可视化/Web 控制 | `Web控制界面介绍.md`, `网页控制端使用指南.md` |
| 配置修改 | `配置修改说明.md` |
| 问题记录 | `问题记录.md` |
