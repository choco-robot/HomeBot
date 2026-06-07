# NavigationCoordinator 详细说明

> 最后更新：2026-06-07

## 1. 概述

`NavigationCoordinator` 是 HomeBot 导航系统的**核心调度器**，负责协调全局路径规划、局部避障和运动控制三大模块。它以状态机方式运行，对外提供统一的目标管理接口，对内通过 ZeroMQ 或函数回调与底盘、视觉、SLAM 等服务通信。

### 1.1 核心职责

| 职责 | 说明 |
|------|------|
| 目标队列管理 | 支持多目标优先级队列，可同步/异步导航 |
| 全局规划触发 | 调用 A* 规划器，生成安全全局路径 |
| 局部避障执行 | 纯追踪 + 紧急停车，保障运行安全 |
| 异常恢复 | 偏离路径时自动重规划，最多重试 N 次 |

### 1.2 设计原则

- **精简可靠**：全局路径只做视线法简化，不做全局插值平滑，避免穿障碍物
- **控制层平滑**：通过 `lookahead_distance`（前瞻距离）和 P 控制器实现轨迹平滑
- **安全优先**：任何修改全局路径形状的操作都需经过碰撞检查

---

## 2. 架构与数据流

```
┌─────────────────────────────────────────────────────────────┐
│                      NavigationCoordinator                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ 目标队列  │  │ 状态机    │  │ 全局规划  │  │ 速度计算  │   │
│  │ GoalQueue│  │ State    │  │ A* + LOS │  │ PurePursuit│  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
│       │             │             │             │           │
│  ┌────┴─────────────┴─────────────┴─────────────┴─────┐    │
│  │              _control_loop (10Hz)                   │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
        ↑              ↑              ↑              ↑
   PoseProvider  ObstacleProvider VelocitySender  MapProvider
   (位姿)        (障碍物)         (速度指令)       (栅格地图)
```

### 2.1 外部接口（Setter 注入）

| 接口 | 类型 | 说明 |
|------|------|------|
| `set_pose_provider` | `() -> (x, y, theta)` | 里程计/SLAM 位姿源 |
| `set_obstacle_provider` | `() -> List[Obstacle]` | 深度视觉/激光雷达障碍物 |
| `set_velocity_sender` | `(vx, vz) -> bool` | 向底盘服务发送速度指令 |
| `set_map_provider` | `() -> OccupancyGrid` | 全局栅格地图 |

---

## 3. 状态机

```
                    ┌─────────────┐
         ┌─────────→│    IDLE     │←────────┐
         │          │   (空闲)     │         │
         │          └──────┬──────┘         │
         │                 │ 新目标入队      │
         │                 ▼                │
         │          ┌─────────────┐         │
         │          │  PLANNING   │         │
         │          │   (规划中)   │         │
         │          └──────┬──────┘         │
         │                 │ 规划成功        │
         │                 ▼                │
         │    ┌──────→┌─────────────┐       │
         │    │       │  NAVIGATING │       │
         │    │       │   (导航中)   │       │
         │    │       └──────┬──────┘       │
         │    │              │              │
         │    │ 紧急障碍物   │ 偏离路径      │
         │    │              │              │
         │    ▼              ▼              │
         │ ┌────────┐   ┌──────────┐       │
         │ │OBSTACLE│   │ RECOVERY │       │
         │ │AVOIDING│   │ (重规划)  │       │
         │ └───┬────┘   └────┬─────┘       │
         │     │ 障碍清除    │ 重规划成功    │
         └─────┘─────────────┘             │
              │                             │
              └─────────────────────────────┘
```

### 3.1 各状态说明

| 状态 | 触发条件 | 行为 |
|------|----------|------|
| `IDLE` | 初始状态 / 目标完成 | 等待新目标入队 |
| `PLANNING` | 新目标激活 | A* 全局规划 + 视线法简化 |
| `NAVIGATING` | 规划成功 | 纯追踪控制 + 障碍物检测 |
| `OBSTACLE_AVOIDING` | 检测到紧急障碍物 | 发送停止指令，等待障碍清除 |
| `RECOVERY` | 偏离路径 | 重新进入 PLANNING 状态 |
| `STOPPED` | 控制循环异常 | 安全模式，停止所有运动 |

---

## 4. 核心流程详解

### 4.1 全局规划流程

```python
# 1. A* 在栅格地图上搜索路径
path = self._global_planner.plan(start, goal)

# 2. 滑动窗口简化：在局部窗口内保留直线段，在转弯处保留细节
path = self._global_planner._simplify_path(path)

# 3. 保留折线路径（不做全局插值平滑，避免穿障碍物）
self.global_path = path
```

**滑动窗口简化（Sliding Window Simplification）**：
- 以当前点为起点，在固定大小窗口 `[i+1, i+W]`（默认 W=20）内寻找最远点
- 该点需同时满足：
  1. **视线无障碍**（Bresenham 检查）
  2. **近似直线**（窗口内所有中间点到连线的最大垂直距离 ≤ 15cm）
- 如果找到，直接跳到该点；否则缩小窗口，小步前进
- 效果：长直线段每 W 步保留一个点快速简化，转弯处小步前进保留拐角细节

**时间复杂度**：O(N × W²)，W 为常数窗口大小，实际近似 O(N)。相比纯视线法 O(N³)，长路径简化速度提升 1~2 个数量级。

### 4.2 局部控制流程（纯追踪 + 避障）

```python
# 1. 提取局部目标点（前瞻）
local_goal = self._get_local_goal(current_pose)

# 2. 计算到局部目标的方向
angle_error = normalize_angle(atan2(dy, dx) - current_theta)

# 3. P 控制器输出角速度
angular_vel = Kp * angle_error

# 4. 线速度根据角度误差和障碍物距离动态调整
if abs(angle_error) > 60°:
    linear_vel = 0.0          # 原地旋转
else:
    linear_vel = max_speed * angle_factor * obstacle_factor
```

#### 4.2.1 前瞻距离（Lookahead Distance）

`lookahead_distance` 是纯追踪控制器的核心参数，决定机器人"往多远的地方看"：

| 值 | 效果 | 适用场景 |
|---|---|---|
| 小（0.2m） | 贴线跟踪，转向激进 | 狭窄走廊、精确 docking |
| 中（0.4m） | 平衡精度和平滑度 | **默认，大多数场景** |
| 大（1.0m+） | 走大弧线，切弯严重 | 开阔场地（注意障碍物） |

> **注意**：路径的全局平滑不应通过修改路径点来实现，而应通过调整 `lookahead_distance` 在控制层实现。

#### 4.2.2 最终朝向调整

当机器人位置已到达目标点范围内（`distance <= goal_reached_distance`），但朝向未满足时：

```python
linear_vel = 0.0
angular_vel = Kp * yaw_error   # 原地旋转到目标朝向
```

这避免了纯追踪控制器在目标点附近因 `atan2(0, 0)` 不稳定而乱转。

### 4.3 偏离路径检测

计算机器人当前位置到全局路径各线段的**垂直距离**：

```python
for segment in path_segments_nearby:
    dist = point_to_segment_distance(robot_pos, segment)
    min_dist = min(min_dist, dist)

if min_dist > replan_distance_threshold:
    trigger_replanning()
```

使用点到线段的垂直距离（而非点到点的距离），避免机器人在两路径点中间时被误判为偏离。

---

## 5. 配置参数

所有参数通过 `NavigationConfig`（`software/src/configs/config.py`）集中管理。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `arrival_distance_threshold_m` | 0.1 m | 到达目标的距离阈值 |
| `arrival_angle_threshold_rad` | 0.15 rad | 到达目标的角度阈值 |
| `control_rate_hz` | 10.0 Hz | 控制循环频率 |
| `max_replan_attempts` | 5 | 最大重规划次数 |
| `emergency_obstacle_distance_m` | 0.3 m | 紧急停车距离 |
| `max_path_deviation_m` | 0.5 m | 偏离路径触发重规划的阈值 |
| `inflation_radius_m` | 0.2 m | 障碍物膨胀半径 |
| `robot_radius_m` | 0.25 m | 机器人半径 |
| `lookahead_distance_m` | 0.4 m | 路径跟踪前瞻距离 |
| `max_linear_speed` | 0.5 m/s | 导航最大线速度 |
| `max_angular_speed` | 0.8 rad/s | 导航最大角速度 |

---

## 6. 使用示例

### 6.1 基础使用（仿真环境）

```python
from navigation.coordinator import NavigationCoordinator

# 创建协调器（参数从 configs 读取）
coordinator = NavigationCoordinator({
    "goal_reached_distance": 0.1,
    "lookahead_distance": 0.4,
})

# 注入外部接口
coordinator.set_pose_provider(sim.get_odom_pose)
coordinator.set_map_provider(sim.get_map)
coordinator.set_velocity_sender(sim.set_velocity)
coordinator.set_obstacle_provider(get_obstacles)

# 启动
coordinator.start()

# 异步导航
goal_id = coordinator.navigate_to_async(x=2.0, y=1.5, yaw=0.0)

# 查询状态
feedback = coordinator.get_feedback(goal_id)
print(f"状态: {feedback.state.value}, 进度: {feedback.progress:.0%}")

# 停止
coordinator.stop()
```

### 6.2 同步导航（阻塞等待）

```python
result = coordinator.navigate_to(x=2.0, y=1.5, yaw=0.0, blocking=True)
if result.error_msg:
    print(f"导航失败: {result.error_msg}")
else:
    print("导航成功！")
```

---

## 7. 常见问题

### Q1: 路径为什么是折线而不是平滑曲线？

这是**有意设计**。全局路径采用视线法简化后的安全折线，所有拐角点都经过障碍物验证。平滑应在控制层通过调整 `lookahead_distance` 实现，而不是修改全局路径几何形状，避免插值曲线穿入障碍物。

### Q2: 机器人在拐角处急停急转怎么办？

增大 `lookahead_distance`（如从 0.4 改为 0.6~0.8），让机器人提前看向更远的路径点，自然走弧线切过拐角。不要试图用 B 样条或贝塞尔曲线平滑全局路径。

### Q3: 到达目标点后为什么原地旋转？

如果目标指定了 `yaw`（朝向），机器人在位置到达后会进入**最终朝向调整**模式：线速度置 0，只发送角速度原地旋转到目标朝向。如果不需要朝向控制，调用时传 `yaw=None`。

### Q4: 如何避免在两点中间被误判偏离路径？

偏离检测使用**点到路径线段的垂直距离**（而非点到点的距离）。只要机器人在路径线段的"带状区域"内，就不会触发重规划。

---

## 8. 代码精简记录（2026-06-07）

本次优化对 `navigation_coordinator.py` 进行了精简，行数从 **957 → 869**（减少 88 行），主要改动：

| 改动 | 说明 |
|------|------|
| 删除 `_local_planner` / `_costmap_generator` | 从未实际使用的属性 |
| 删除 `obstacle_check_frequency` | 未使用的配置项 |
| 缓存 `_nav_cfg()` 配置 | 在 `__init__` 中缓存 `max_linear_speed` / `max_angular_speed`，避免控制循环反复读取 |
| 提取 `_extract_obstacle_distance` | 统一解析多种障碍物格式，消除 `_has_emergency_obstacle` 和 `_get_min_obstacle_distance` 的重复代码 |
| 修复 `cancel_all_goals` 死锁 | 移除外层锁，避免与 `cancel_goal` 内部锁形成不可重入死锁 |
| 修复 `get_feedback` 进度计算 | 避免 `remaining_distance > total_distance` 时出现负进度 |
| 精简 setter 注释 | 移除过度详细的 docstring |
| 删除 B 样条平滑代码 | 保留 `_smooth_path` 为空实现作为预留接口 |
