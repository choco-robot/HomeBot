# 2D SLAM 仿真器

轻量级导航算法测试工具，无需 ROS 依赖。

## 功能特性

- **机器人模型**：差速驱动运动学模型，支持噪声模拟
- **激光雷达传感器**：360度激光扫描仿真，支持测量噪声
- **地图环境**：支持 YAML/PNG 格式地图和代码生成地图
- **碰撞检测**：圆形和多边形障碍物检测
- **可视化工具**：基于 Matplotlib 的实时可视化
- **导航协调器集成**：提供标准接口与导航协调器对接

## 快速开始

### 基础使用

```python
from navigation.simulation import Simulator, MapEnvironment

# 创建仿真器
sim = Simulator()

# 使用内置地图
sim.set_map(MapEnvironment.create_simple_room())

# 启动仿真
sim.start()

# 控制机器人
sim.set_velocity(linear=0.3, angular=0.2)

# 获取传感器数据
pose = sim.get_robot_pose()
scan = sim.get_laser_scan()

# 停止仿真
sim.stop()
```

### 内置地图

仿真器提供三种内置地图：

```python
# 1. 简单房间（8m x 6m，包含家具）
sim.set_map(MapEnvironment.create_simple_room())

# 2. 迷宫地图（10m x 10m）
sim.set_map(MapEnvironment.create_maze())

# 3. 杂乱房间（6m x 6m，随机障碍物）
sim.set_map(MapEnvironment.create_cluttered_room())
```

### 自定义地图

```python
# 创建空地图
sim.create_empty_map(width=10.0, height=10.0)

# 添加障碍物
map_env = sim.map_env

# 圆形障碍物
map_env.add_circle_obstacle(x=2.0, y=3.0, radius=0.3)

# 矩形障碍物
map_env.add_rectangle_obstacle(x=1.0, y=1.0, width=1.5, height=0.5)

# 多边形障碍物
map_env.add_polygon_obstacle(vertices=[(x1,y1), (x2,y2), (x3,y3)])
```

### 加载地图文件

支持标准 ROS 格式地图（YAML + PNG）：

```python
sim.load_map('maps/my_map.yaml')
```

## 测试导航算法

### 测试 A* 全局规划

```python
from navigation.core.astar_planner import AStarPlanner

sim = Simulator()
sim.set_map(MapEnvironment.create_maze())

planner = AStarPlanner(sim.get_map(), allow_diagonal=True)

start = (-4.0, -4.0)
goal = (4.0, 4.0)

path = planner.plan(start, goal)

if path:
    print(f"路径点数: {len(path)}")
```

### 测试 VFH 局部避障

```python
from navigation.planning.local_planner import VFHLocalPlanner
from navigation.planning.costmap_generator import LocalCostmapGenerator

sim = Simulator()
sim.set_map(MapEnvironment.create_cluttered_room())
sim.start()

vfh = VFHLocalPlanner()
costmap_gen = LocalCostmapGenerator()

goal = (2.0, 2.0)

while True:
    pose = sim.get_odom_pose()
    scan = sim.get_laser_scan()
    
    # 转换为障碍物
    obstacles = convert_scan_to_obstacles(scan, pose)
    
    # 生成代价地图
    costmap = costmap_gen.generate(obstacles, pose)
    
    # 计算速度
    linear, angular = vfh.compute_velocity(pose, goal, costmap)
    
    sim.set_velocity(linear, angular)
    
    # 检查是否到达
    if distance_to_goal < 0.2:
        break
```

### 集成导航协调器

```python
from navigation.coordinator.navigation_coordinator import NavigationCoordinator

sim = Simulator()
sim.set_map(MapEnvironment.create_simple_room())
sim.start()

coordinator = NavigationCoordinator()

# 连接仿真器接口
coordinator.set_pose_provider(sim.get_pose_provider())
coordinator.set_obstacle_provider(sim.get_obstacle_provider())
coordinator.set_velocity_sender(sim.get_velocity_sender())
coordinator.set_map_provider(sim.get_map_provider())

coordinator.start()

# 发送导航目标
coordinator.navigate_to(x=3.0, y=2.0, yaw=0.0)
```

## 可视化

```python
from navigation.simulation import SimVisualizer

# 创建可视化
viz = SimVisualizer(sim.get_map(), title="Navigation Test")

# 显示路径
viz.update_path(path)

# 显示机器人
viz.update_robot(x, y, yaw)

# 显示窗口
viz.show()

# 保存截图
viz.save_screenshot("result.png")
```

## 运行测试

```bash
# 运行所有测试
python -m navigation.simulation.test_navigation

# 运行示例
python -m navigation.simulation.example_usage
```

## 文件结构

```
simulation/
├── __init__.py              # 模块导出
├── robot_model.py           # 机器人运动学模型
├── laser_scanner.py         # 激光雷达传感器
├── map_environment.py       # 地图环境管理
├── simulator.py             # 仿真器主类
├── sim_visualizer.py        # 可视化工具（已存在）
├── test_navigation.py       # 测试脚本
└── example_usage.py         # 使用示例
```

## 配置参数

### 机器人配置

```python
from navigation.simulation.robot_model import RobotConfig

config = RobotConfig(
    radius=0.15,              # 机器人半径（米）
    max_linear_vel=0.5,       # 最大线速度（m/s）
    max_angular_vel=1.0,      # 最大角速度（rad/s）
    linear_noise_std=0.01,    # 线速度噪声（米）
    angular_noise_std=0.02,   # 角速度噪声（弧度）
)
```

### 激光雷达配置

```python
from navigation.simulation.laser_scanner import LaserConfig

config = LaserConfig(
    num_rays=360,             # 射线数量
    max_range=12.0,           # 最大距离（米）
    fov=2*pi,                 # 视场角（弧度）
    range_noise_std=0.02,     # 距离噪声（米）
)
```

### 仿真器配置

```python
from navigation.simulation.simulator import SimulatorConfig

config = SimulatorConfig(
    physics_frequency=100.0,  # 物理更新频率（Hz）
    sensor_frequency=10.0,    # 传感器频率（Hz）
    enable_visualization=True,
)
```

## 依赖项

- numpy
- matplotlib
- opencv-python (可选，用于地图图像加载)
- pyyaml (可选，用于 YAML 地图加载)

## 与 ROS 标准仿真器对比

| 特性 | 本仿真器 | STDR Simulator | Gazebo |
|------|---------|----------------|--------|
| 安装难度 | 简单 | 中等 | 复杂 |
| ROS 依赖 | 无 | 必需 | 必需 |
| 3D 支持 | 无 | 无 | 有 |
| 物理引擎 | 简化 | 简化 | 完整 |
| 适用场景 | 算法开发测试 | ROS 集成测试 | 真实仿真 |

## 性能优化建议

1. **降低传感器频率**：如果不需要高频数据，可以降低 `sensor_frequency`
2. **减少射线数量**：激光雷达 `num_rays=180` 已足够大多数场景
3. **使用简化地图**：测试算法时使用小地图（如 5m x 5m）

## 扩展功能

### 添加动态障碍物

```python
# 添加动态障碍物
obs_id = map_env.add_circle_obstacle(x=1.0, y=2.0, radius=0.2, is_static=False)

# 移除障碍物
map_env.remove_obstacle(obs_id)
```

### 录制数据

```python
# TODO: 实现数据录制功能
# sim.start_recording('test_data.bag')
# ...
# sim.stop_recording()
```

## 常见问题

**Q: 如何调整仿真速度？**

A: 修改 `physics_frequency` 和 `sensor_frequency` 参数。

**Q: 激光扫描数据格式是什么？**

A: 返回 numpy 数组，shape=(360,)，单位为米，从 -pi 到 pi 方向排列。

**Q: 如何获取无噪声的真实位姿？**

A: 使用 `get_robot_pose()`（真实位姿）和 `get_odom_pose()`（里程计位姿，带噪声）。

## 贡献

欢迎提交 Issue 和 Pull Request！
