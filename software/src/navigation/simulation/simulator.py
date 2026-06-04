# -*- coding: utf-8 -*-
"""2D SLAM 仿真器主类

整合机器人模型、传感器、环境和可视化：
- 实时仿真循环
- 与导航协调器集成接口
- 支持录制和回放
"""

from __future__ import annotations

import time
import threading
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import numpy as np

from common.logging import get_logger
from navigation.simulation.robot_model import DifferentialRobot, RobotConfig
from navigation.simulation.laser_scanner import LaserScanner, LaserConfig
from navigation.simulation.map_environment import MapEnvironment
from navigation.core.occupancy_grid import OccupancyGrid

logger = get_logger(__name__)


@dataclass
class SimulatorConfig:
    """仿真器配置"""

    # 仿真频率
    physics_frequency: float = 100.0  # 物理更新频率（Hz）
    sensor_frequency: float = 10.0  # 传感器更新频率（Hz）

    # 机器人配置
    robot_config: RobotConfig = None

    # 激光雷达配置
    laser_config: LaserConfig = None

    # 可视化
    enable_visualization: bool = True
    visualization_frequency: float = 30.0  # 可视化更新频率（Hz）


class Simulator:
    """2D SLAM 仿真器

    使用方式：
        # 创建仿真器
        sim = Simulator()
        sim.load_map('maps/simple_room.yaml')

        # 或使用内置地图
        sim.set_map(MapEnvironment.create_simple_room())

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

    def __init__(self, config: SimulatorConfig = None):
        """
        Args:
            config: 仿真器配置
        """
        self.config = config or SimulatorConfig()

        # 组件
        self.robot: Optional[DifferentialRobot] = None
        self.laser: Optional[LaserScanner] = None
        self.map_env: Optional[MapEnvironment] = None

        # 运行状态
        self._running = False
        self._physics_thread: Optional[threading.Thread] = None
        self._sensor_thread: Optional[threading.Thread] = None

        # 传感器数据缓存
        self._latest_scan: Optional[np.ndarray] = None
        self._scan_lock = threading.Lock()

        # 回调函数
        self._scan_callback: Optional[Callable[[np.ndarray], None]] = None

        # 统计信息
        self._start_time = 0.0
        self._frame_count = 0

        logger.info("仿真器初始化完成")

    # --------------------------------------------------------------------------
    # 地图管理
    # --------------------------------------------------------------------------

    def load_map(self, map_file: str):
        """加载地图文件

        Args:
            map_file: 地图文件路径（YAML 格式）
        """
        self.map_env = MapEnvironment(map_file=map_file)
        self._initialize_components()
        logger.info(f"加载地图: {map_file}")

    def set_map(self, map_env: MapEnvironment):
        """设置地图环境

        Args:
            map_env: 地图环境对象
        """
        self.map_env = map_env
        self._initialize_components()

    def create_empty_map(self, width: float = 10.0, height: float = 10.0):
        """创建空地图

        Args:
            width: 地图宽度（米）
            height: 地图高度（米）
        """
        self.map_env = MapEnvironment(width=width, height=height)
        self._initialize_components()

    def _initialize_components(self):
        """初始化机器人和传感器组件"""
        if self.map_env is None:
            return

        # 初始化机器人
        robot_config = self.config.robot_config or RobotConfig()
        self.robot = DifferentialRobot(config=robot_config)

        # 初始化激光雷达
        laser_config = self.config.laser_config or LaserConfig()
        self.laser = LaserScanner(
            config=laser_config,
            robot_radius=robot_config.radius,
        )

        logger.info("组件初始化完成")

    # --------------------------------------------------------------------------
    # 控制接口
    # --------------------------------------------------------------------------

    def set_velocity(self, linear: float, angular: float) -> bool:
        """设置机器人速度

        Args:
            linear: 线速度（m/s）
            angular: 角速度（rad/s）

        Returns:
            是否成功设置速度
        """
        if self.robot:
            self.robot.set_velocity(linear, angular)
            return True
        return False

    def stop_robot(self):
        """停止机器人"""
        if self.robot:
            self.robot.stop()

    def reset_robot(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """重置机器人位姿

        Args:
            x: x 坐标（米）
            y: y 坐标（米）
            theta: 朝向（弧度）
        """
        if self.robot:
            self.robot.reset(x, y, theta)
            logger.info(f"重置机器人位姿: ({x:.2f}, {y:.2f}, {theta:.2f})")

    # --------------------------------------------------------------------------
    # 传感器接口
    # --------------------------------------------------------------------------

    def get_laser_scan(self) -> Optional[np.ndarray]:
        """获取最新的激光扫描数据

        Returns:
            距离数组（米），shape = (num_rays,)
        """
        with self._scan_lock:
            return self._latest_scan.copy() if self._latest_scan is not None else None

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """获取机器人真实位姿（仿真器内部使用）

        Returns:
            (x, y, theta)
        """
        return self.robot.pose if self.robot else None

    def get_odom_pose(self) -> Optional[Tuple[float, float, float]]:
        """获取里程计位姿（带噪声，对外提供）

        Returns:
            (x, y, theta)
        """
        return self.robot.odom_pose if self.robot else None

    def get_map(self) -> Optional[OccupancyGrid]:
        """获取地图栅格"""
        return self.map_env.grid if self.map_env else None

    def get_obstacles(self) -> List:
        """获取障碍物列表"""
        return self.map_env.get_obstacles_list() if self.map_env else []

    # --------------------------------------------------------------------------
    # 回调设置
    # --------------------------------------------------------------------------

    def set_scan_callback(self, callback: Callable[[np.ndarray], None]):
        """设置激光扫描回调

        Args:
            callback: 回调函数，接收激光扫描数据
        """
        self._scan_callback = callback

    # --------------------------------------------------------------------------
    # 仿真循环
    # --------------------------------------------------------------------------

    def start(self):
        """启动仿真"""
        if self._running:
            logger.warning("仿真器已在运行")
            return

        if self.robot is None:
            logger.error("请先加载地图")
            return

        self._running = True
        self._start_time = time.time()
        self._frame_count = 0

        # 启动物理线程
        self._physics_thread = threading.Thread(
            target=self._physics_loop,
            daemon=True,
        )
        self._physics_thread.start()

        # 启动传感器线程
        self._sensor_thread = threading.Thread(
            target=self._sensor_loop,
            daemon=True,
        )
        self._sensor_thread.start()

        logger.info("仿真器已启动")

    def stop(self):
        """停止仿真"""
        self._running = False

        if self._physics_thread:
            self._physics_thread.join(timeout=1.0)

        if self._sensor_thread:
            self._sensor_thread.join(timeout=1.0)

        if self.robot:
            self.robot.stop()

        logger.info("仿真器已停止")

    def _physics_loop(self):
        """物理更新循环"""
        dt = 1.0 / self.config.physics_frequency

        while self._running:
            try:
                # 更新机器人位姿
                self.robot.update(dt)

                # 碰撞检测
                obstacles = self.map_env.get_obstacles_list()
                if self.robot.check_collision(obstacles):
                    logger.warning("检测到碰撞，停止机器人")
                    self.robot.stop()
                    # 将机器人推回上一个位置（简化处理）
                    # TODO: 实现更精细的碰撞响应

                time.sleep(dt)

            except Exception as e:
                logger.error(f"物理循环异常: {e}", exc_info=True)

    def _sensor_loop(self):
        """传感器更新循环"""
        dt = 1.0 / self.config.sensor_frequency

        while self._running:
            try:
                # 执行激光扫描
                scan = self.laser.scan(
                    robot_pose=self.robot.pose,
                    obstacles=self.map_env.get_obstacles_list(),
                    map_grid=self.map_env.grid.data,
                    map_resolution=self.map_env.grid.resolution,
                    map_origin=self.map_env.grid.origin,
                )

                # 更新缓存
                with self._scan_lock:
                    self._latest_scan = scan

                # 调用回调
                if self._scan_callback:
                    self._scan_callback(scan)

                self._frame_count += 1

                time.sleep(dt)

            except Exception as e:
                logger.error(f"传感器循环异常: {e}", exc_info=True)

    # --------------------------------------------------------------------------
    # 工具方法
    # --------------------------------------------------------------------------

    def get_simulation_time(self) -> float:
        """获取仿真运行时间（秒）"""
        return time.time() - self._start_time if self._running else 0.0

    def get_info(self) -> dict:
        """获取仿真器信息"""
        return {
            "running": self._running,
            "simulation_time": self.get_simulation_time(),
            "frame_count": self._frame_count,
            "robot_info": self.robot.get_info() if self.robot else None,
        }

    def is_running(self) -> bool:
        """检查是否正在运行"""
        return self._running

    # --------------------------------------------------------------------------
    # 导航协调器集成接口
    # --------------------------------------------------------------------------

    def get_pose_provider(self) -> Callable:
        """获取位姿提供者函数（用于导航协调器）"""
        return self.get_odom_pose

    def get_obstacle_provider(self) -> Callable:
        """获取障碍物提供者函数（用于导航协调器）"""
        return self.get_obstacles

    def get_velocity_sender(self) -> Callable:
        """获取速度发送器函数（用于导航协调器）"""
        return self.set_velocity

    def get_map_provider(self) -> Callable:
        """获取地图提供者函数（用于导航协调器）"""
        return self.get_map
