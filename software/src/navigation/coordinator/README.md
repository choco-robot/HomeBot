# 导航协调器

协调全局规划、局部避障和运动控制。

## 功能特性

- **目标管理**：支持单目标和多目标导航，优先级队列
- **状态机控制**：清晰的导航状态转换
- **自动重规划**：偏离路径时自动触发重规划
- **异常处理**：处理目标不可达、超时、障碍物等情况
- **实时反馈**：提供导航进度和状态更新

## 快速开始

### 基础使用

```python
from navigation.coordinator import NavigationCoordinator
from navigation.simulation import Simulator, MapEnvironment

# 创建仿真器
sim = Simulator()
sim.set_map(MapEnvironment.create_simple_room())
sim.start()

# 创建协调器
coordinator = NavigationCoordinator()

# 连接接口
coordinator.set_pose_provider(sim.get_odom_pose)
coordinator.set_obstacle_provider(lambda: [])
coordinator.set_velocity_sender(sim.set_velocity)
coordinator.set_map_provider(sim.get_map)

coordinator.start()

# 同步导航（阻塞等待完成）
feedback = coordinator.navigate_to(x=2.0, y=3.0, yaw=0.0)

if feedback.error_msg is None:
    print("导航成功！")
else:
    print(f"导航失败: {feedback.error_msg}")

coordinator.stop()
sim.stop()
```

### 异步导航

```python
# 异步导航（立即返回）
goal_id = coordinator.navigate_to_async(x=2.0, y=3.0)

# 监控进度
while True:
    feedback = coordinator.get_feedback(goal_id)
    
    print(f"进度: {feedback.progress:.1%}")
    print(f"距离: {feedback.distance_to_goal:.2f}m")
    print(f"状态: {feedback.state.value}")
    
    if feedback.state == NavigationState.IDLE:
        break
    
    time.sleep(0.5)
```

### 多目标导航

```python
# 添加多个目标（按顺序执行）
goals = [
    (1.0, 2.0),
    (3.0, 4.0),
    (5.0, 6.0),
]

for x, y in goals:
    coordinator.navigate_to(x=x, y=y)
```

### 取消导航

```python
# 异步导航
goal_id = coordinator.navigate_to_async(x=5.0, y=5.0)

# 等待一段时间后取消
time.sleep(5.0)
coordinator.cancel_goal(goal_id)
```

## 接口说明

### 外部接口注入

协调器需要四个外部接口：

```python
# 1. 位姿提供者
def get_pose() -> Tuple[float, float, float]:
    """返回当前位姿 (x, y, theta)"""
    return (0.0, 0.0, 0.0)

coordinator.set_pose_provider(get_pose)

# 2. 障碍物提供者
def get_obstacles() -> List:
    """返回障碍物列表"""
    return []

coordinator.set_obstacle_provider(get_obstacles)

# 3. 速度发送器
def send_velocity(linear: float, angular: float) -> bool:
    """发送速度指令，返回是否成功"""
    return True

coordinator.set_velocity_sender(send_velocity)

# 4. 地图提供者
def get_map() -> OccupancyGrid:
    """返回全局地图"""
    return grid

coordinator.set_map_provider(get_map)
```

### 导航目标

```python
# 简单目标
coordinator.navigate_to(x=2.0, y=3.0)

# 带朝向的目标
coordinator.navigate_to(x=2.0, y=3.0, yaw=math.pi/2)

# 高优先级目标
coordinator.navigate_to(x=2.0, y=3.0, priority=2)

# 设置超时
coordinator.navigate_to(x=2.0, y=3.0, timeout=60.0)
```

### 导航反馈

```python
feedback = coordinator.get_feedback(goal_id)

print(f"当前状态: {feedback.state}")
print(f"当前位姿: {feedback.current_pose}")
print(f"剩余距离: {feedback.distance_to_goal}")
print(f"进度: {feedback.progress:.1%}")
print(f"已用时间: {feedback.time_elapsed:.1f}s")
print(f"错误信息: {feedback.error_msg}")
```

## 配置参数

```python
coordinator = NavigationCoordinator({
    # 重规划阈值
    'replan_distance_threshold': 0.5,  # 偏离路径超过 0.5m 时重规划
    
    # 到达判定
    'goal_reached_distance': 0.1,      # 距离目标 0.1m 内视为到达
    'goal_reached_angle': 0.1,         # 角度误差 0.1rad 内视为到达
    
    # 重规划次数
    'max_replan_attempts': 3,          # 最大重规划次数
    
    # 控制频率
    'control_frequency': 10.0,         # 控制循环频率（Hz）
    
    # 紧急避障
    'obstacle_emergency_distance': 0.3,  # 紧急停止距离（米）
})
```

## 状态机

```
IDLE（空闲）
  ↓ [收到新目标]
PLANNING（规划中）
  ↓ [全局规划成功]
NAVIGATING（导航中）
  ↓ [到达目标] 或 [遇到障碍] 或 [规划失败]
  ├─ SUCCESS → IDLE
  ├— OBSTACLE_AVOIDING → NAVIGATING
  └─ FAILED → IDLE（并报告错误）
```

## 与仿真器集成

完整示例：

```python
from navigation.simulation import Simulator, MapEnvironment
from navigation.coordinator import NavigationCoordinator

# 创建仿真器
sim = Simulator()
sim.set_map(MapEnvironment.create_maze())
sim.start()
sim.reset_robot(x=-4.0, y=-4.0, theta=0.0)

# 创建协调器
coordinator = NavigationCoordinator({
    'goal_reached_distance': 0.2,
    'control_frequency': 10.0,
})

# 连接接口
coordinator.set_pose_provider(sim.get_odom_pose)
coordinator.set_map_provider(sim.get_map)
coordinator.set_velocity_sender(sim.set_velocity)

# 障碍物提供者（将激光扫描转换为障碍物）
def get_obstacles():
    scan = sim.get_laser_scan()
    pose = sim.get_odom_pose()
    if scan is not None and pose is not None:
        # 转换逻辑
        obstacles = convert_scan_to_obstacles(scan, pose)
        return obstacles
    return []

coordinator.set_obstacle_provider(get_obstacles)
coordinator.start()

# 执行导航
feedback = coordinator.navigate_to(x=4.0, y=4.0)

# 清理
coordinator.stop()
sim.stop()
```

## 测试

运行集成测试：

```bash
cd E:/develop/homeBOT/homebot/software
python -m navigation.coordinator.test_coordinator
```

## 文件结构

```
coordinator/
├── __init__.py                   # 模块导出
├── navigation_coordinator.py     # 协调器主类（完整实现）
└── test_coordinator.py           # 集成测试
```

## 性能说明

- 控制循环频率：10 Hz（可配置）
- 全局规划时间：10-50ms（取决于地图大小）
- 内存占用：< 10MB

## 扩展功能

### 自定义速度控制器

当前使用简单的纯追踪控制器，可以替换为更复杂的控制器：

```python
# 继承 NavigationCoordinator 并重写 _compute_velocity 方法
class MyCoordinator(NavigationCoordinator):
    def _compute_velocity(self, current_pose, local_goal, obstacles):
        # 自定义控制逻辑
        linear = ...
        angular = ...
        return linear, angular
```

### 添加路径平滑

路径平滑使用 RDP 算法，可以调整参数：

```python
coordinator = NavigationCoordinator({
    'path_smoothing_epsilon': 0.05,  # 平滑阈值（米）
})
```

## 常见问题

**Q: 导航一直失败怎么办？**

A: 检查以下项：
1. 地图是否正确加载
2. 目标点是否可达（不在障碍物内）
3. 位姿提供者是否正常工作
4. 查看日志中的错误信息

**Q: 如何调整导航速度？**

A: 修改 `_compute_velocity` 方法中的速度参数，或继承并重写该方法。

**Q: 如何处理动态障碍物？**

A: 当前支持通过障碍物提供者获取实时障碍物。可以添加障碍物预测和轨迹规划功能。

## 依赖

- numpy
- navigation.core.astar_planner
- navigation.core.occupancy_grid
- common.logging
