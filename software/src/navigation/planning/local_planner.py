# -*- coding: utf-8 -*-
"""简化版 VFH（Vector Field Histogram）局部避障控制器

输入：
- 距离直方图 np.ndarray，每个元素表示该扇区方向的最近障碍物距离（米）
- 目标点方向（相对机器人坐标系）

输出：
- 推荐的速度指令 (vx, vz)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple, Union

import numpy as np

from common.logging import get_logger
from navigation.perception.obstacle_detector import DepthObstacle

logger = get_logger(__name__)


@dataclass
class LocalPlannerConfig:
    """VFH 局部规划器配置"""
    max_linear_speed: float = 0.3       # 最大线速度 (m/s)
    max_angular_speed: float = 1.0      # 最大角速度 (rad/s)
    num_sectors: int = 21               # 扇区数量（默认与深度条带数一致）
    fov_deg: float = 66.0               # 相机水平视场角（度）fov_deg/num_sectors = 每个扇区的角度范围
    safety_distance_m: float = 0.5      # 安全距离（小于此距离视为阻挡）
    goal_weight: float = 0.8            # 目标方向权重（用于选择最佳谷）数值越接近1，越倾向于选择接近目标方向的谷，越接近0，越倾向于选择宽阔的谷
    smooth_weight: float = 0.15          # 当前方向平滑权重
    min_valley_width: int = 2           # 最小可通行谷宽度（扇区数）
    sector_diff_weight: float = 0.2     # sector_diff 系数（用于角速度计算）


class VFHLocalPlanner:
    """简化 VFH 局部避障规划器。"""

    def __init__(self, config: Optional[LocalPlannerConfig] = None):
        self.config = config or LocalPlannerConfig()
        self._prev_vz = 0.0  # 上一次的角速度，用于平滑

    def plan(
        self,
        obstacles: np.ndarray,
        goal_x: float,
        goal_y: float,
        current_vx: float = 0.0,
        current_vz: float = 0.0,
    ) -> Tuple[float, float]:
        """根据障碍物和目标点计算局部速度指令。

        Args:
            obstacles: 距离直方图 np.ndarray（米，inf=无障碍）
            goal_x: 目标点 x（机器人前方为正）
            goal_y: 目标点 y（机器人左侧为正）
            current_vx: 当前线速度
            current_vz: 当前角速度

        Returns:
            (vx, vz) 速度指令
        """
        cfg = self.config

        # 1. 计算目标方向
        goal_angle = math.atan2(goal_y, goal_x)
        goal_distance = math.hypot(goal_x, goal_y)

        # # 如果目标不在正面90°范围内，优先原地旋转调整朝向
        # if abs(goal_angle) > math.pi/4:
        #     return 0.0, cfg.max_angular_speed if goal_angle > 0 else -cfg.max_angular_speed

        # 2. 构建阻挡数组
        histogram = obstacles
        n = len(histogram)

        # 无障碍物数据时，默认所有方向畅通，直接朝向目标前进
        if n == 0:
            vx = cfg.max_linear_speed
            if goal_distance < 0.3:
                vx *= goal_distance / 0.3
            vz = np.clip(goal_angle * 0.5, -cfg.max_angular_speed, cfg.max_angular_speed)
            return float(vx), float(vz)

        blocked = histogram < cfg.safety_distance_m

        # 3. 寻找可通行谷
        valleys = self._find_valleys(blocked, n)
        logger.info(f'valleys={valleys}')
        if not valleys:
            logger.warning("VFH: 所有方向被阻挡，原地旋转")
            vz = cfg.max_angular_speed if self._prev_vz >= 0 else -cfg.max_angular_speed
            self._prev_vz = vz
            return 0.0, vz

        # 4. 选择最佳谷
        best_sector = self._select_best_sector(valleys, goal_angle, current_vz, n)

        logger.info(f'best_sector={best_sector}, blocked={blocked[best_sector]}')
        # 5. 计算角速度
        # 最佳扇区与正前方扇区的索引差值，用于决定角速度的方向和大小
        sector_diff = best_sector - n // 2
        vz = np.clip(sector_diff * cfg.sector_diff_weight, -cfg.max_angular_speed, cfg.max_angular_speed)
        vz = (1 - cfg.smooth_weight) * vz + cfg.smooth_weight * current_vz
        vz = np.clip(vz, -cfg.max_angular_speed, cfg.max_angular_speed)
        self._prev_vz = vz
        logger.info(f'sector_diff={sector_diff}, vz={vz:.2f}')

        # 6. 计算线速度
        forward_sector = n // 2  # 正前方对应中间扇区
        forward_blocked = blocked[forward_sector]

        if forward_blocked:
            vx = cfg.max_linear_speed * 0.3
        else:
            speed_factor = max(0.3, 1.0 - abs(sector_diff) / n) # 根据偏离程度调整速度，偏离越大越慢
            vx = cfg.max_linear_speed * speed_factor

        if goal_distance < 0.3:
            vx *= goal_distance / 0.3

        self._prev_vz = vz
        return float(vx), float(vz)

    def _find_valleys(self, blocked: np.ndarray, n: int) -> List[Tuple[int, int]]:
        """寻找所有可通行谷（连续未阻挡扇区）。"""
        cfg = self.config
        valleys = []
        in_valley = False
        start = 0

        for i in range(1,n):
            if not blocked[i] and not in_valley:
                in_valley = True
                start = i
            elif blocked[i] and in_valley:
                in_valley = False
                end = i - 1
                width = end - start + 1
                if width >= cfg.min_valley_width:
                    valleys.append((start, end))

        if in_valley:
            end = n - 1
            width = end - start + 1
            if width >= cfg.min_valley_width:
                valleys.append((start % n, end % n))

        return valleys

    def _select_best_sector(
        self,
        valleys: List[Tuple[int, int]],
        goal_angle: float,
        current_vz: float,
        n: int,
    ) -> int:
        """从所有谷中选择最接近目标方向的扇区中心。"""
        goal_sector = self._angle_to_sector(goal_angle, n)
        logger.info(f'goal_angle={math.degrees(goal_angle):.1f}°, goal_sector={goal_sector}')

        # 1. 如果目标扇区本身就在某个可通行谷中，直接走目标方向
        for start, end in valleys:
            if start <= end:
                if start <= goal_sector <= end:
                    return goal_sector

        # 2. 目标方向被阻挡，在所有谷中选择最接近目标的中心
        best_sector = None
        best_cost = float('inf')

        for start, end in valleys:
            center = (start + end) // 2
            width = end - start + 1

            diff = abs(center - goal_sector)
            diff = min(diff, n - diff)
            cost = diff * self.config.goal_weight

            if width >= self.config.min_valley_width and cost < best_cost:
                best_cost = cost
                if goal_sector<center:
                    best_sector = int(start * self.config.goal_weight + center * (1 - self.config.goal_weight))
                else:
                    best_sector = int(end * self.config.goal_weight + center * (1 - self.config.goal_weight))

        return best_sector if best_sector is not None else goal_sector

    def _angle_to_sector(self, angle: float, n: int) -> int:
        """将弧度角度转换为扇区索引。"""
        return np.clip(int(round(angle / math.radians(self.config.fov_deg) * n)) + n // 2, 0, n - 1)

    