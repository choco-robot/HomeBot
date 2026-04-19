# -*- coding: utf-8 -*-
"""局部代价地图生成器

将深度障碍物投影到以机器人为中心的局部栅格地图上。
"""
from __future__ import annotations

import math
from typing import List, Optional

import numpy as np

from common.logging import get_logger
from navigation.core.occupancy_grid import COST_FREE, COST_LETHAL, OccupancyGrid
from navigation.perception.obstacle_detector import DepthObstacle

logger = get_logger(__name__)


class LocalCostmapGenerator:
    """生成以机器人为中心的局部代价地图。

    地图坐标系：
    - 机器人位于地图中心
    - X 轴向右，Y 轴向上（世界坐标系）
    - 但在栅格地图中，Y 轴向下，需要做坐标转换
    """

    def __init__(
        self,
        width_m: float = 3.0,
        height_m: float = 3.0,
        resolution: float = 0.05,
        inflation_radius_m: float = 0.15,
    ):
        """
        Args:
            width_m: 地图宽度（米）
            height_m: 地图高度（米）
            resolution: 栅格分辨率（米/栅格）
            inflation_radius_m: 障碍物膨胀半径
        """
        self.width_m = width_m
        self.height_m = height_m
        self.resolution = resolution
        self.inflation_radius_m = inflation_radius_m

        self.width = int(width_m / resolution)
        self.height = int(height_m / resolution)
        # 原点设为左上角，机器人中心对应 (width/2, height/2)
        self.origin = (-width_m / 2, -height_m / 2)

    def generate(
        self,
        obstacles: List[DepthObstacle],
        robot_pose: Optional[tuple] = None,
    ) -> OccupancyGrid:
        """根据障碍物列表生成局部代价地图。

        Args:
            obstacles: 深度障碍物列表（相机坐标系）
            robot_pose: 机器人位姿 (x, y, yaw)，当前未使用（局部地图以机器人为原点）

        Returns:
            局部 OccupancyGrid
        """
        grid = OccupancyGrid(
            width=self.width,
            height=self.height,
            resolution=self.resolution,
            origin=self.origin,
            default_cost=COST_FREE,
        )

        for obs in obstacles:
            # 障碍物坐标系：x=右，y=下（相机坐标系）
            # 转换为世界坐标系：x=右，y=上
            # 在栅格地图中，y=下，所以 world_y = -obs.z（前方为正，地图下方为正？）
            # 等等，需要明确相机坐标系定义：
            # obstacle_detector 中：
            # x_m = (px - cx) * z_m / fx  -> 水平右正
            # y_m = (py - cy) * z_m / fy  -> 垂直下正（因为图像 y 向下）
            # z_m = 深度，正前方
            #
            # 在局部地图中，我们通常使用：
            # 机器人前方为 Y+，右侧为 X+
            # 栅格地图中 Y 向下，所以前方（Y+）对应栅格向下
            # 因此：
            # world_x = obs.x
            # world_y = obs.z  （深度即前方距离）
            #
            # 但 obstacle_detector 的 y 是垂直高度，不是深度方向。
            # 对于 2D 局部地图，我们只关心地面投影 (x, z)。
            wx = obs.x
            wy = obs.z

            # 绘制圆形障碍物
            grid.set_circle_world(wx, wy, max(obs.width, obs.height) / 2, COST_LETHAL)

        if self.inflation_radius_m > 0:
            grid.inflate_obstacles(self.inflation_radius_m)

        return grid
