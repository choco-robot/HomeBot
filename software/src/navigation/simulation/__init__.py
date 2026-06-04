# -*- coding: utf-8 -*-
"""2D SLAM仿真器 - 轻量级导航算法测试工具

功能：
- 差速驱动机器人模型
- 激光雷达传感器仿真
- 2D地图环境
- 实时可视化
- 与导航协调器集成

使用方式：
    from navigation.simulation import Simulator, MapEnvironment

    # 方式1：使用内置地图
    sim = Simulator()
    sim.set_map(MapEnvironment.create_simple_room())

    # 方式2：加载地图文件
    sim.load_map('maps/simple_room.yaml')

    # 启动仿真
    sim.start()

    # 控制机器人
    sim.set_velocity(linear=0.3, angular=0.2)

    # 获取传感器数据
    scan = sim.get_laser_scan()
    pose = sim.get_robot_pose()

    # 停止仿真
    sim.stop()
"""

from navigation.simulation.robot_model import DifferentialRobot, RobotConfig
from navigation.simulation.laser_scanner import LaserScanner, LaserConfig
from navigation.simulation.map_environment import MapEnvironment, Obstacle
from navigation.simulation.simulator import Simulator, SimulatorConfig
from navigation.simulation.sim_visualizer import SimVisualizer

__all__ = [
    # 主类
    "Simulator",
    "SimulatorConfig",
    # 组件
    "DifferentialRobot",
    "RobotConfig",
    "LaserScanner",
    "LaserConfig",
    "MapEnvironment",
    "Obstacle",
    # 可视化
    "SimVisualizer",
]
