# -*- coding: utf-8 -*-
"""栅格地图数据结构"""
from __future__ import annotations

import json
import math
from typing import List, Optional, Tuple

import numpy as np

from common.logging import get_logger

logger = get_logger(__name__)

# 代价值定义（与 ROS 代价地图兼容）
COST_FREE = 0
COST_UNKNOWN = -1
COST_OCCUPIED = 100
COST_LETHAL = 255


class OccupancyGrid:
    """2D 栅格地图，支持障碍物膨胀和序列化。

    坐标系定义：
    - 世界坐标 (wx, wy)：以米为单位，地图中心为原点 (0, 0)
    - 栅格坐标 (gx, gy)：整数索引，(0, 0) 在左上角，x 向右，y 向下
    """

    def __init__(
        self,
        width: int,
        height: int,
        resolution: float = 0.05,
        origin: Tuple[float, float] = (0.0, 0.0),
        default_cost: int = COST_FREE,
    ):
        """
        Args:
            width: 地图宽度（栅格数）
            height: 地图高度（栅格数）
            resolution: 每个栅格代表的物理尺寸（米/栅格）
            origin: 世界坐标中地图左上角的坐标 (x, y)
            default_cost: 初始化时代价值
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin  # 左上角在世界坐标系中的位置

        if default_cost == COST_UNKNOWN:
            self.data = np.full((height, width), COST_UNKNOWN, dtype=np.int16)
        else:
            self.data = np.full((height, width), default_cost, dtype=np.int16)

    # ------------------------------------------------------------------
    # 坐标转换
    # ------------------------------------------------------------------
    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """世界坐标转栅格坐标"""
        gx = int((wx - self.origin[0]) / self.resolution)
        gy = int((wy - self.origin[1]) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """栅格坐标转世界坐标（返回栅格中心点）"""
        wx = self.origin[0] + (gx + 0.5) * self.resolution
        wy = self.origin[1] + (gy + 0.5) * self.resolution
        return wx, wy

    # ------------------------------------------------------------------
    # 数据访问
    # ------------------------------------------------------------------
    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def get_cost(self, gx: int, gy: int) -> int:
        if not self.in_bounds(gx, gy):
            return COST_UNKNOWN
        return int(self.data[gy, gx])

    def get_cost_world(self, wx: float, wy: float) -> int:
        gx, gy = self.world_to_grid(wx, wy)
        return self.get_cost(gx, gy)

    def set_cost(self, gx: int, gy: int, cost: int) -> None:
        if self.in_bounds(gx, gy):
            self.data[gy, gx] = cost

    def set_cost_world(self, wx: float, wy: float, cost: int) -> None:
        gx, gy = self.world_to_grid(wx, wy)
        self.set_cost(gx, gy, cost)

    def set_rectangle(
        self,
        x: int,
        y: int,
        w: int,
        h: int,
        cost: int,
    ) -> None:
        """设置矩形区域代价值"""
        x1, x2 = max(0, x), min(self.width, x + w)
        y1, y2 = max(0, y), min(self.height, y + h)
        self.data[y1:y2, x1:x2] = cost

    def set_circle_world(
        self,
        wx: float,
        wy: float,
        radius_m: float,
        cost: int,
    ) -> None:
        """在世界坐标中设置圆形区域"""
        gx, gy = self.world_to_grid(wx, wy)
        radius = int(math.ceil(radius_m / self.resolution))
        self.set_circle(gx, gy, radius, cost)

    def set_circle(self, cx: int, cy: int, radius: int, cost: int) -> None:
        """设置圆形区域代价值"""
        x1, x2 = max(0, cx - radius), min(self.width, cx + radius + 1)
        y1, y2 = max(0, cy - radius), min(self.height, cy + radius + 1)
        yy, xx = np.ogrid[y1:y2, x1:x2]
        mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius ** 2
        self.data[y1:y2, x1:x2][mask] = cost

    # ------------------------------------------------------------------
    # 障碍物膨胀
    # ------------------------------------------------------------------
    def inflate_obstacles(self, inflation_radius_m: float) -> None:
        """将当前所有被占据的栅格按半径膨胀。

        膨胀后的代价值从内到外线性递减：
        - 障碍物本身 = COST_LETHAL (255)
        - 膨胀边缘 ≈ COST_OCCUPIED (100)
        """
        if inflation_radius_m <= 0:
            return

        radius = int(math.ceil(inflation_radius_m / self.resolution))
        occupied = (self.data >= COST_OCCUPIED) & (self.data != COST_UNKNOWN)
        if not np.any(occupied):
            return

        # 用距离变换计算每个自由栅格到最近障碍物的距离（栅格数）
        from scipy.ndimage import distance_transform_edt

        dist = distance_transform_edt(~occupied)

        # 仅处理在膨胀半径内的栅格
        mask = dist <= radius
        # 线性插值代价：中心 = 255，边缘 = 100
        # 注意：dist 在整个地图上计算，cost 也要保持相同 shape
        cost_array = COST_LETHAL - (COST_LETHAL - COST_OCCUPIED) * (dist / radius)
        cost_array = cost_array.astype(np.int16)

        # 只更新比当前值大的代价（取最大）
        update_mask = mask & (cost_array > self.data)
        self.data[update_mask] = cost_array[update_mask]

        logger.debug(f"膨胀完成，半径={inflation_radius_m}m ({radius}栅格)")

    # ------------------------------------------------------------------
    # 地图生成辅助
    # ------------------------------------------------------------------
    def add_random_obstacles(
        self,
        count: int = 10,
        min_radius_m: float = 0.1,
        max_radius_m: float = 0.3,
        seed: Optional[int] = None,
    ) -> None:
        """随机生成圆形障碍物（仅用于测试）"""
        rng = np.random.default_rng(seed)
        for _ in range(count):
            wx = rng.uniform(self.origin[0], self.origin[0] + self.width * self.resolution)
            wy = rng.uniform(self.origin[1], self.origin[1] + self.height * self.resolution)
            r = rng.uniform(min_radius_m, max_radius_m)
            self.set_circle_world(wx, wy, r, COST_LETHAL)

    # ------------------------------------------------------------------
    # 序列化
    # ------------------------------------------------------------------
    def to_dict(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": list(self.origin),
            "data": self.data.tolist(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> OccupancyGrid:
        grid = cls(
            width=data["width"],
            height=data["height"],
            resolution=data["resolution"],
            origin=tuple(data["origin"]),
        )
        grid.data = np.array(data["data"], dtype=np.int16)
        return grid

    def save(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f)
        logger.info(f"地图已保存: {path}")

    @classmethod
    def load(cls, path: str) -> OccupancyGrid:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        grid = cls.from_dict(data)
        logger.info(f"地图已加载: {path}")
        return grid
