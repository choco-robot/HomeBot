# -*- coding: utf-8 -*-
"""地图环境

管理 2D 仿真环境：
- 地图加载（YAML/PNG/代码生成）
- 障碍物管理
- 目标点管理
- 与 OccupancyGrid 集成
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

from common.logging import get_logger
from navigation.core.occupancy_grid import (
    COST_FREE,
    COST_LETHAL,
    COST_UNKNOWN,
    OccupancyGrid,
)

logger = get_logger(__name__)


@dataclass
class Obstacle:
    """障碍物数据结构"""

    id: int
    type: str  # 'circle' 或 'polygon'
    data: np.ndarray  # 圆形：(x, y, radius)，多边形：Nx2 顶点数组
    is_static: bool = True  # 是否为静态障碍


class MapEnvironment:
    """地图环境

    功能：
    - 加载标准 ROS 格式地图（YAML + PNG）
    - 代码生成简单地图
    - 管理静态和动态障碍物
    - 提供碰撞检测接口
    """

    def __init__(
        self,
        map_file: Optional[str] = None,
        width: float = 10.0,
        height: float = 10.0,
        resolution: float = 0.05,
    ):
        """
        Args:
            map_file: 地图文件路径（YAML 格式），None 时创建空地图
            width: 空地图宽度（米）
            height: 空地图高度（米）
            resolution: 地图分辨率（米/栅格）
        """
        self.resolution = resolution
        self.obstacles: Dict[int, Obstacle] = {}
        self._next_obstacle_id = 0

        # 加载或创建地图
        if map_file:
            self._load_map(map_file)
        else:
            self._create_empty_map(width, height, resolution)

        # 目标点
        self.goals: List[Tuple[float, float]] = []

        logger.info(
            f"地图环境初始化完成: {self.grid.width}x{self.grid.height} 栅格, "
            f"分辨率 {resolution}m, 范围 {self.width:.1f}x{self.height:.1f}m"
        )

    # --------------------------------------------------------------------------
    # 地图加载
    # --------------------------------------------------------------------------

    def _load_map(self, map_file: str):
        """加载地图文件

        支持格式：
        - ROS 格式地图（YAML + PNG）
        - 地图编辑器导出的 JSON 文件
        """
        if not os.path.exists(map_file):
            raise FileNotFoundError(f"地图文件不存在: {map_file}")

        # 根据文件扩展名判断格式
        if map_file.endswith(".json"):
            self._load_map_from_json(map_file)
        else:
            self._load_map_from_yaml(map_file)

    def _load_map_from_yaml(self, map_file: str):
        """加载 ROS 格式地图（YAML + PNG）"""
        # 读取 YAML
        with open(map_file, "r") as f:
            map_info = yaml.safe_load(f)

        # 读取图像
        image_file = map_info["image"]
        if not os.path.isabs(image_file):
            image_file = os.path.join(os.path.dirname(map_file), image_file)

        import cv2

        image = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)

        if image is None:
            raise ValueError(f"无法加载地图图像: {image_file}")

        # ROS 地图格式：黑色=障碍，白色=自由
        # OccupancyGrid 格式：0=自由，255=障碍
        grid_data = np.where(image < 127, COST_LETHAL, COST_FREE).astype(np.int16)

        # 注意：图像坐标系 Y 轴向下，地图坐标系 Y 轴向上
        # 需要垂直翻转图像
        grid_data = np.flipud(grid_data)

        # 创建 OccupancyGrid
        self.grid = OccupancyGrid(
            width=image.shape[1],
            height=image.shape[0],
            resolution=map_info["resolution"],
            origin=tuple(map_info["origin"][:2]),
        )
        self.grid.data = grid_data

        logger.info(f"加载 ROS 格式地图: {map_file}")

    def _load_map_from_json(self, json_file: str):
        """加载地图编辑器导出的 JSON 文件

        支持两种格式：
        1. 包含 PNG 图像引用：直接加载栅格地图（推荐）
        2. 仅包含障碍物列表：从障碍物重建地图

        Args:
            json_file: JSON 文件路径
        """
        import json

        with open(json_file, "r", encoding="utf-8") as f:
            data = json.load(f)

        # 读取地图信息
        map_info = data.get("map_info", {})
        width = map_info.get("width", 10.0)
        height = map_info.get("height", 10.0)
        resolution = map_info.get("resolution", 0.05)
        origin = map_info.get("origin", [0.0, 0.0])

        # 检查是否引用了 PNG 图像
        image_file = data.get("image")
        if image_file:
            # 方式1：加载 PNG 栅格地图（推荐）
            if not os.path.isabs(image_file):
                # PNG 文件与 JSON 文件在同一目录
                image_file = os.path.join(os.path.dirname(json_file), image_file)

            if os.path.exists(image_file):
                logger.info(f"从 PNG 图像加载地图: {image_file}")
                self._load_from_png(image_file, resolution, origin)
                return
            else:
                logger.warning(f"PNG 文件不存在: {image_file}, 将从障碍物列表重建地图")

        # 方式2：从障碍物列表重建地图
        logger.info(f"从障碍物列表重建地图")

        # 创建空地图
        grid_width = int(width / resolution)
        grid_height = int(height / resolution)

        self.grid = OccupancyGrid(
            width=grid_width,
            height=grid_height,
            resolution=resolution,
            origin=tuple(origin[:2]),
        )

        # 添加边界墙
        self._add_boundary_walls()

        # 加载障碍物
        obstacles_data = data.get("obstacles", [])
        for obs in obstacles_data:
            obs_type = obs.get("type")

            if obs_type == "rectangle":
                x = obs.get("x", 0.0)
                y = obs.get("y", 0.0)
                w = obs.get("width", 0.0)
                h = obs.get("height", 0.0)
                self.add_rectangle_obstacle(x, y, w, h)

            elif obs_type == "circle":
                x = obs.get("x", 0.0)
                y = obs.get("y", 0.0)
                radius = obs.get("radius", 0.0)
                self.add_circle_obstacle(x, y, radius)

            elif obs_type == "polygon":
                points = obs.get("points", [])
                if len(points) >= 3:
                    self.add_polygon_obstacle(points)

            elif obs_type == "line":
                # 直线障碍物转换为矩形或多边形
                x1 = obs.get("x1", 0.0)
                y1 = obs.get("y1", 0.0)
                x2 = obs.get("x2", 0.0)
                y2 = obs.get("y2", 0.0)
                thickness = obs.get("thickness", 0.2)

                # 计算直线的四个顶点
                self._add_line_obstacle(x1, y1, x2, y2, thickness)

        logger.info(f"从 JSON 加载地图: {json_file}, {len(obstacles_data)} 个障碍物")

    def _load_from_png(self, png_file: str, resolution: float, origin: tuple):
        """从 PNG 文件加载栅格地图

        PNG 格式：
        - 灰度图：黑色（0）= 空闲，白色（255）= 障碍
        - 兼容 Breezy SLAM 格式

        Args:
            png_file: PNG 文件路径
            resolution: 地图分辨率（米/像素）
            origin: 地图原点坐标 (x, y)
        """
        import cv2

        # 读取图像（灰度模式）
        image = cv2.imread(png_file, cv2.IMREAD_GRAYSCALE)

        if image is None:
            raise ValueError(f"无法加载地图图像: {png_file}")

        # PNG 格式：黑色=空闲(0)，白色=障碍(255)
        # OccupancyGrid 格式：0=自由，255=障碍
        # 因此直接使用图像数据

        # 注意：图像坐标系 Y 轴向下，地图坐标系 Y 轴向上
        # 需要垂直翻转图像
        grid_data = np.flipud(image).astype(np.int16)

        # 创建 OccupancyGrid
        self.grid = OccupancyGrid(
            width=image.shape[1],
            height=image.shape[0],
            resolution=resolution,
            origin=origin[:2],
        )
        self.grid.data = grid_data

        logger.info(
            f"加载 PNG 地图: {png_file}, "
            f"尺寸 {image.shape[1]}x{image.shape[0]}, "
            f"分辨率 {resolution}m"
        )

    def _add_line_obstacle(
        self, x1: float, y1: float, x2: float, y2: float, thickness: float
    ):
        """添加直线障碍物（转换为矩形）

        Args:
            x1, y1: 起点坐标
            x2, y2: 终点坐标
            thickness: 厚度（米）
        """
        # 计算直线的角度
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx * dx + dy * dy)

        if length < 0.01:
            return  # 太短，忽略

        angle = math.atan2(dy, dx)

        # 计算垂直方向的偏移
        perp_x = -math.sin(angle) * thickness / 2
        perp_y = math.cos(angle) * thickness / 2

        # 四个顶点
        vertices = [
            (x1 + perp_x, y1 + perp_y),
            (x2 + perp_x, y2 + perp_y),
            (x2 - perp_x, y2 - perp_y),
            (x1 - perp_x, y1 - perp_y),
        ]

        self.add_polygon_obstacle(vertices)

    def _create_empty_map(self, width: float, height: float, resolution: float):
        """创建空地图"""
        grid_width = int(width / resolution)
        grid_height = int(height / resolution)

        self.grid = OccupancyGrid(
            width=grid_width,
            height=grid_height,
            resolution=resolution,
            origin=(-width / 2, -height / 2),
        )

        # 添加边界墙
        self._add_boundary_walls()

    def _add_boundary_walls(self):
        """添加边界墙"""
        wall_thickness = int(0.1 / self.resolution)  # 10cm 厚的墙

        # 左右墙
        self.grid.data[:, :wall_thickness] = COST_LETHAL
        self.grid.data[:, -wall_thickness:] = COST_LETHAL

        # 上下墙
        self.grid.data[:wall_thickness, :] = COST_LETHAL
        self.grid.data[-wall_thickness:, :] = COST_LETHAL

    # --------------------------------------------------------------------------
    # 障碍物管理
    # --------------------------------------------------------------------------

    def add_circle_obstacle(
        self,
        x: float,
        y: float,
        radius: float,
        is_static: bool = True,
    ) -> int:
        """添加圆形障碍物

        Args:
            x: 圆心 x 坐标（米）
            y: 圆心 y 坐标（米）
            radius: 半径（米）
            is_static: 是否为静态障碍

        Returns:
            障碍物 ID
        """
        obs_id = self._next_obstacle_id
        self._next_obstacle_id += 1

        self.obstacles[obs_id] = Obstacle(
            id=obs_id,
            type="circle",
            data=np.array([x, y, radius]),
            is_static=is_static,
        )

        # 更新栅格地图
        if is_static:
            self._add_circle_to_grid(x, y, radius)

        logger.debug(f"添加圆形障碍物 {obs_id}: ({x:.2f}, {y:.2f}), 半径 {radius:.2f}m")
        return obs_id

    def add_polygon_obstacle(
        self,
        vertices: List[Tuple[float, float]],
        is_static: bool = True,
    ) -> int:
        """添加多边形障碍物

        Args:
            vertices: 顶点列表 [(x1, y1), (x2, y2), ...]
            is_static: 是否为静态障碍

        Returns:
            障碍物 ID
        """
        obs_id = self._next_obstacle_id
        self._next_obstacle_id += 1

        vertices_array = np.array(vertices)

        self.obstacles[obs_id] = Obstacle(
            id=obs_id,
            type="polygon",
            data=vertices_array,
            is_static=is_static,
        )

        # 更新栅格地图
        if is_static:
            self._add_polygon_to_grid(vertices_array)

        logger.debug(f"添加多边形障碍物 {obs_id}: {len(vertices)} 个顶点")
        return obs_id

    def add_rectangle_obstacle(
        self,
        x: float,
        y: float,
        width: float,
        height: float,
        is_static: bool = True,
    ) -> int:
        """添加矩形障碍物（便捷方法）

        Args:
            x: 左下角 x 坐标（米）
            y: 左下角 y 坐标（米）
            width: 宽度（米）
            height: 高度（米）

        Returns:
            障碍物 ID
        """
        vertices = [
            (x, y),
            (x + width, y),
            (x + width, y + height),
            (x, y + height),
        ]
        return self.add_polygon_obstacle(vertices, is_static)

    def remove_obstacle(self, obs_id: int) -> bool:
        """移除障碍物"""
        if obs_id not in self.obstacles:
            return False

        obs = self.obstacles[obs_id]
        del self.obstacles[obs_id]

        # TODO: 从栅格地图中移除（需要重新生成）

        logger.debug(f"移除障碍物 {obs_id}")
        return True

    def clear_dynamic_obstacles(self):
        """清除所有动态障碍物"""
        to_remove = [
            obs_id for obs_id, obs in self.obstacles.items() if not obs.is_static
        ]
        for obs_id in to_remove:
            del self.obstacles[obs_id]

        logger.debug(f"清除 {len(to_remove)} 个动态障碍物")

    # --------------------------------------------------------------------------
    # 栅格地图更新
    # --------------------------------------------------------------------------

    def _add_circle_to_grid(self, x: float, y: float, radius: float):
        """将圆形添加到栅格地图"""
        gx, gy = self.grid.world_to_grid(x, y)
        gr = int(radius / self.resolution)

        for dy in range(-gr, gr + 1):
            for dx in range(-gr, gr + 1):
                if dx * dx + dy * dy <= gr * gr:
                    nx, ny = gx + dx, gy + dy
                    if self.grid.in_bounds(nx, ny):
                        self.grid.data[ny, nx] = COST_LETHAL

    def _add_polygon_to_grid(self, vertices: np.ndarray):
        """将多边形添加到栅格地图（扫描线填充算法）"""
        # 简化实现：使用 OpenCV 填充
        import cv2

        # 转换顶点到栅格坐标
        grid_points = []
        for vx, vy in vertices:
            gx, gy = self.grid.world_to_grid(vx, vy)
            grid_points.append([gx, gy])

        grid_points = np.array(grid_points, dtype=np.int32)

        # 创建掩码
        mask = np.zeros_like(self.grid.data, dtype=np.uint8)
        cv2.fillPoly(mask, [grid_points], 255)

        # 更新栅格地图
        self.grid.data[mask > 0] = COST_LETHAL

    # --------------------------------------------------------------------------
    # 碰撞检测接口
    # --------------------------------------------------------------------------

    def get_obstacles_list(self) -> List:
        """获取障碍物列表（用于激光扫描）

        Returns:
            障碍物列表，每个元素为 (x, y, radius) 或顶点数组
        """
        obstacles = []
        for obs in self.obstacles.values():
            if obs.type == "circle":
                obstacles.append(tuple(obs.data))
            else:
                obstacles.append(obs.data)
        return obstacles

    def check_point_collision(self, x: float, y: float) -> bool:
        """检查点是否与障碍物碰撞

        Args:
            x: 点 x 坐标（米）
            y: 点 y 坐标（米）

        Returns:
            是否碰撞
        """
        # 检查栅格地图
        gx, gy = self.grid.world_to_grid(x, y)
        if not self.grid.in_bounds(gx, gy):
            return True  # 地图边界外视为碰撞

        if self.grid.data[gy, gx] == COST_LETHAL:
            return True

        # 检查动态障碍物
        for obs in self.obstacles.values():
            if obs.type == "circle":
                ox, oy, radius = obs.data
                dist = math.sqrt((x - ox) ** 2 + (y - oy) ** 2)
                if dist < radius:
                    return True
            else:
                # 多边形：点在多边形内检测
                if self._point_in_polygon((x, y), obs.data):
                    return True

        return False

    @staticmethod
    def _point_in_polygon(point: Tuple[float, float], polygon: np.ndarray) -> bool:
        """判断点是否在多边形内"""
        x, y = point
        n = len(polygon)
        inside = False

        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]

            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside

            j = i

        return inside

    # --------------------------------------------------------------------------
    # 地图生成工具
    # --------------------------------------------------------------------------

    @staticmethod
    def create_simple_room() -> "MapEnvironment":
        """创建简单房间地图"""
        env = MapEnvironment(width=8.0, height=6.0, resolution=0.05)

        # 添加家具障碍物
        env.add_rectangle_obstacle(1.0, 1.0, 1.5, 0.5)  # 桌子
        env.add_rectangle_obstacle(4.0, 3.0, 0.5, 1.5)  # 柜子
        env.add_circle_obstacle(2.5, 4.0, 0.3)  # 椅子
        env.add_circle_obstacle(6.0, 1.5, 0.25)  # 椅子

        return env

    @staticmethod
    def create_maze() -> "MapEnvironment":
        """创建迷宫地图"""
        env = MapEnvironment(width=10.0, height=10.0, resolution=0.05)

        # 迷宫墙壁
        walls = [
            (-4.0, -4.0, 8.0, 0.1),  # 底部横墙
            (-4.0, 0.0, 3.0, 0.1),  # 中下左横墙
            (1.0, 0.0, 3.0, 0.1),  # 中下右横墙
            (-4.0, 3.0, 6.0, 0.1),  # 中上横墙
            (3.0, 3.0, 1.0, 0.1),  # 中上右横墙
            (-2.0, -2.0, 0.1, 2.0),  # 左竖墙
            (1.0, -2.0, 0.1, 2.0),  # 中竖墙
            (3.0, -4.0, 0.1, 4.0),  # 右竖墙
            (4.0, 0.0, 0.1, 3.0),  # 最右竖墙
        ]

        for x, y, w, h in walls:
            env.add_rectangle_obstacle(x, y, w, h)

        return env

    @staticmethod
    def create_cluttered_room() -> "MapEnvironment":
        """创建杂乱房间地图"""
        env = MapEnvironment(width=6.0, height=6.0, resolution=0.05)

        # 随机障碍物
        np.random.seed(42)
        num_obstacles = 8

        for i in range(num_obstacles):
            x = np.random.uniform(-2.5, 2.5)
            y = np.random.uniform(-2.5, 2.5)
            radius = np.random.uniform(0.2, 0.4)
            env.add_circle_obstacle(x, y, radius)

        return env

    # --------------------------------------------------------------------------
    # 属性访问
    # --------------------------------------------------------------------------

    @property
    def width(self) -> float:
        """地图宽度（米）"""
        return self.grid.width * self.grid.resolution

    @property
    def height(self) -> float:
        """地图高度（米）"""
        return self.grid.height * self.grid.resolution

    @property
    def origin(self) -> Tuple[float, float]:
        """地图原点（左下角）"""
        return self.grid.origin

    def get_grid_data(self) -> np.ndarray:
        """获取栅格地图数据"""
        return self.grid.data.copy()

    # --------------------------------------------------------------------------
    # 序列化
    # --------------------------------------------------------------------------

    def save_to_file(self, map_file: str):
        """保存地图到文件"""
        import cv2

        # 准备 YAML 信息
        map_info = {
            "image": os.path.basename(map_file).replace(".yaml", ".png"),
            "resolution": self.resolution,
            "origin": [self.origin[0], self.origin[1], 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }

        # 保存 YAML
        yaml_file = map_file if map_file.endswith(".yaml") else map_file + ".yaml"
        with open(yaml_file, "w") as f:
            yaml.dump(map_info, f)

        # 保存图像
        image_file = yaml_file.replace(".yaml", ".png")
        # OccupancyGrid -> 图像：0=自由，255=障碍
        image_data = np.where(self.grid.data == COST_LETHAL, 0, 255).astype(np.uint8)
        cv2.imwrite(image_file, image_data)

        logger.info(f"地图已保存: {yaml_file}")
