# -*- coding: utf-8 -*-
"""A* 全局路径规划器"""
from __future__ import annotations

import heapq
import math
from typing import Callable, List, Optional, Tuple

import numpy as np

from common.logging import get_logger
from .occupancy_grid import COST_LETHAL, COST_OCCUPIED, COST_UNKNOWN, OccupancyGrid

logger = get_logger(__name__)


def euclidean_distance(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def manhattan_distance(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class AStarPlanner:
    """A* 路径规划器。

    支持 8 邻域扩展，可配置障碍物阈值、启发式函数。
    """

    def __init__(
        self,
        grid: OccupancyGrid,
        allow_diagonal: bool = True,
        obstacle_threshold: int = COST_OCCUPIED,
    ):
        """
        Args:
            grid: 栅格地图
            allow_diagonal: 是否允许斜向移动
            obstacle_threshold: 大于等于该值的栅格视为不可通行
        """
        self.grid = grid
        self.allow_diagonal = allow_diagonal
        self.obstacle_threshold = obstacle_threshold

        # 8 方向邻居偏移
        self.neighbors_4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        self.neighbors_8 = self.neighbors_4 + [(1, 1), (1, -1), (-1, 1), (-1, -1)]

    def _is_valid(self, gx: int, gy: int) -> bool:
        if not self.grid.in_bounds(gx, gy):
            return False
        cost = self.grid.get_cost(gx, gy)
        if cost == COST_UNKNOWN:
            # 默认将未知区域视为可通行；如需保守策略可改为 False
            return True
        return cost < self.obstacle_threshold

    def plan(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        heuristic: Optional[Callable[[Tuple[int, int], Tuple[int, int]], float]] = None,
    ) -> Optional[List[Tuple[float, float]]]:
        """从起点到终点规划路径，返回世界坐标路径点列表（包含起点和终点）。

        Args:
            start_world: 起点世界坐标 (x, y)
            goal_world: 终点世界坐标 (x, y)
            heuristic: 启发式函数，默认使用欧氏距离

        Returns:
            路径点列表，若不可达则返回 None
        """
        sx, sy = self.grid.world_to_grid(start_world[0], start_world[1])
        gx, gy = self.grid.world_to_grid(goal_world[0], goal_world[1])

        if not self._is_valid(sx, sy):
            logger.warning(f"起点无效或被占据: {start_world} -> grid({sx},{sy})")
            return None
        if not self._is_valid(gx, gy):
            logger.warning(f"终点无效或被占据: {goal_world} -> grid({gx},{gy})")
            return None

        h_func = heuristic or euclidean_distance
        start = (sx, sy)
        goal = (gx, gy)

        # A* 数据结构
        open_set: List[Tuple[float, Tuple[int, int]]] = []
        heapq.heappush(open_set, (0.0, start))
        came_from: dict = {}
        g_score = {start: 0.0}
        f_score = {start: h_func(start, goal)}

        closed_set = set()
        neighbors = self.neighbors_8 if self.allow_diagonal else self.neighbors_4

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path_grid = self._reconstruct_path(came_from, current)
                path_world = [self.grid.grid_to_world(x, y) for x, y in path_grid]
                logger.info(f"A* 规划成功，路径点数量: {len(path_world)}")
                return path_world

            if current in closed_set:
                continue
            closed_set.add(current)

            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                neighbor = (nx, ny)

                if not self._is_valid(nx, ny):
                    continue
                if neighbor in closed_set:
                    continue

                # 斜向移动代价为 sqrt(2)，直线为 1
                step_cost = math.hypot(dx, dy)
                tentative_g = g_score[current] + step_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + h_func(neighbor, goal)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))

        logger.warning("A* 规划失败：无可达路径")
        return None

    def _reconstruct_path(
        self,
        came_from: dict,
        current: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def plan_with_simplification(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
    ) -> Optional[List[Tuple[float, float]]]:
        """规划并简化路径（去除冗余中间点）。"""
        path = self.plan(start_world, goal_world)
        if path is None or len(path) <= 2:
            return path
        return self._simplify_path(path)

    def _simplify_path(
        self,
        path: List[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """滑动窗口路径简化（O(N * W²) ≈ O(N)）。

        策略：以当前点为起点，在窗口 [i+1, i+W] 内寻找最远的、
        视线无障碍且近似直线的点作为下一段起点。

        效果：
        - 长直线段：每 W 步保留一个点，快速简化
        - 转弯段：窗口内找不到直线点，小步前进，保留拐角细节
        """
        return self._sliding_window_simplify(path, window_size=20)

    def _sliding_window_simplify(
        self,
        path: List[Tuple[float, float]],
        window_size: int = 20,
    ) -> List[Tuple[float, float]]:
        """滑动窗口简化（O(N * W²) ≈ O(N)）。

        策略：以当前点为起点，在窗口 [i+1, i+W] 内寻找最远的、
        视线无障碍且近似直线的点作为下一段起点。

        效果：
        - 长直线段：每 W 步保留一个点，快速粗化
        - 转弯段：窗口内找不到直线点，小步前进，保留细节
        """
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        i = 0
        max_deviation_threshold = 0.15

        while i < len(path) - 1:
            # 窗口右边界
            j = min(i + window_size, len(path) - 1)

            # 从窗口最远处往回找
            while j > i + 1:
                if self._has_line_of_sight(path[i], path[j]):
                    if self._is_straight_segment(path, i, j, max_deviation_threshold):
                        break
                j -= 1

            simplified.append(path[j])
            i = j

        return simplified

    def _is_straight_segment(
        self,
        path: List[Tuple[float, float]],
        start: int,
        end: int,
        threshold: float,
    ) -> bool:
        """检查 path[start:end] 是否是近似直线（所有中间点偏离连线不超过阈值）。"""
        if end - start <= 2:
            return True

        x0, y0 = path[start]
        x1, y1 = path[end]
        dx = x1 - x0
        dy = y1 - y0
        line_len = math.hypot(dx, dy)

        if line_len < 1e-6:
            return True

        for k in range(start + 1, end):
            x, y = path[k]
            dev = abs(dy * (x - x0) - dx * (y - y0)) / line_len
            if dev > threshold:
                return False

        return True

    def _has_line_of_sight(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
    ) -> bool:
        """判断两点之间是否存在无障碍直线路径（Bresenham 采样）。"""
        x1, y1 = self.grid.world_to_grid(p1[0], p1[1])
        x2, y2 = self.grid.world_to_grid(p2[0], p2[1])

        # Bresenham 直线算法
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        x, y = x1, y1
        while True:
            if not self._is_valid(x, y):
                return False
            if x == x2 and y == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return True
