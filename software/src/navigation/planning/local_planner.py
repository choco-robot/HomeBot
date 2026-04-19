# -*- coding: utf-8 -*-
"""简化版 VFH（Vector Field Histogram）局部避障控制器

输入：
- 深度障碍物列表
- 目标点方向（相对机器人坐标系）
- 当前速度（可选）

输出：
- 推荐的速度指令 (vx, vz)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from common.logging import get_logger
from navigation.perception.obstacle_detector import DepthObstacle

logger = get_logger(__name__)


@dataclass
class LocalPlannerConfig:
    """VFH 局部规划器配置"""
    max_linear_speed: float = 0.3       # 最大线速度 (m/s)
    max_angular_speed: float = 1.0      # 最大角速度 (rad/s)
    num_sectors: int = 36               # 角度分扇区数量（每扇区 10°）
    safety_distance_m: float = 0.5      # 安全距离（小于此距离的障碍物视为阻挡）
    obstacle_threshold: float = 0.3     # 扇区阻挡阈值（0~1 归一化后）
    goal_weight: float = 1.0            # 目标方向权重
    smooth_weight: float = 0.3          # 当前方向平滑权重
    min_valley_width: int = 2           # 最小可通行谷宽度（扇区数）


class VFHLocalPlanner:
    """简化 VFH 局部避障规划器。"""

    def __init__(self, config: Optional[LocalPlannerConfig] = None):
        self.config = config or LocalPlannerConfig()
        self._prev_vz = 0.0  # 上一次的角速度，用于平滑

    def plan(
        self,
        obstacles: List[DepthObstacle],
        goal_x: float,
        goal_y: float,
        current_vx: float = 0.0,
        current_vz: float = 0.0,
    ) -> Tuple[float, float]:
        """根据障碍物和目标点计算局部速度指令。

        Args:
            obstacles: 深度障碍物列表（相机坐标系）
            goal_x: 目标点 x（机器人右侧为正）
            goal_y: 目标点 y（机器人前方为正）
            current_vx: 当前线速度
            current_vz: 当前角速度

        Returns:
            (vx, vz) 速度指令
        """
        cfg = self.config

        # 1. 计算目标方向（机器人坐标系：0=正前方，pi/2=正右方）
        goal_angle = math.atan2(goal_x, goal_y)
        goal_distance = math.hypot(goal_x, goal_y)

        # 2. 构建极坐标直方图
        histogram = self._build_histogram(obstacles)

        # 3. 标记被阻挡的扇区
        blocked = histogram > cfg.obstacle_threshold

        # 4. 寻找可通行谷（连续的未被阻挡扇区）
        valleys = self._find_valleys(blocked)

        if not valleys:
            # 完全阻挡，原地旋转寻找出路
            logger.warning("VFH: 所有方向被阻挡，原地旋转")
            vz = cfg.max_angular_speed if self._prev_vz >= 0 else -cfg.max_angular_speed
            self._prev_vz = vz
            return 0.0, vz

        # 5. 选择最佳谷
        best_sector = self._select_best_sector(valleys, goal_angle, current_vz)
        best_angle = self._sector_to_angle(best_sector)

        # 6. 计算速度指令
        # 角速度：朝向最佳方向
        angle_diff = best_angle - 0.0  # 当前机器人朝向为 0
        # 规范化到 [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        vz = np.clip(
            angle_diff * 2.0,  # P 控制
            -cfg.max_angular_speed,
            cfg.max_angular_speed,
        )

        # 平滑角速度
        vz = (1 - cfg.smooth_weight) * vz + cfg.smooth_weight * current_vz
        vz = np.clip(vz, -cfg.max_angular_speed, cfg.max_angular_speed)
        self._prev_vz = vz

        # 线速度：根据前方障碍物密度和到目标距离决定
        # 如果最佳方向与正前方偏差大，减速
        forward_sector = self._angle_to_sector(0.0)
        forward_blocked = blocked[forward_sector]

        if forward_blocked:
            # 正前方有障碍，进一步减速
            vx = cfg.max_linear_speed * 0.3
        else:
            # 根据角度偏差减速
            speed_factor = max(0.3, 1.0 - abs(angle_diff) / math.radians(45))
            vx = cfg.max_linear_speed * speed_factor

        # 接近目标时减速
        if goal_distance < 0.3:
            vx *= goal_distance / 0.3

        self._prev_vz = vz
        return float(vx), float(vz)

    def _build_histogram(self, obstacles: List[DepthObstacle]) -> np.ndarray:
        """构建障碍物极坐标直方图。"""
        cfg = self.config
        histogram = np.zeros(cfg.num_sectors, dtype=np.float32)

        for obs in obstacles:
            # 极坐标角度（从机器人坐标系看）
            # atan2(x, z): 0=正前方，pi/2=正右方，-pi/2=正左方
            angle = math.atan2(obs.x, obs.z)
            sector = self._angle_to_sector(angle)

            # 距离越近，权重越大
            dist = math.hypot(obs.x, obs.z)
            if dist < 0.05:
                weight = 1.0
            else:
                weight = min(1.0, cfg.safety_distance_m / dist)

            # 障碍物宽度占用的扇区范围
            width_angle = math.atan2(obs.width / 2, dist)
            width_sectors = max(1, int(width_angle / (2 * math.pi / cfg.num_sectors)) + 1)

            for i in range(-width_sectors, width_sectors + 1):
                s = (sector + i) % cfg.num_sectors
                histogram[s] = max(histogram[s], weight)

        return histogram

    def _find_valleys(self, blocked: np.ndarray) -> List[Tuple[int, int]]:
        """寻找所有可通行谷（连续未阻挡扇区），返回 (start, end) 列表。"""
        cfg = self.config
        valleys = []
        n = cfg.num_sectors

        # 处理环形边界，将数组复制一份拼接
        extended = np.concatenate([blocked, blocked])
        in_valley = False
        start = 0

        for i in range(2 * n):
            if not extended[i] and not in_valley:
                in_valley = True
                start = i
            elif extended[i] and in_valley:
                in_valley = False
                end = i - 1
                width = end - start + 1
                if width >= cfg.min_valley_width:
                    # 限制在原始范围内
                    s = start % n
                    e = end % n
                    valleys.append((s, e))

        if in_valley:
            end = 2 * n - 1
            width = end - start + 1
            if width >= cfg.min_valley_width:
                s = start % n
                e = end % n
                valleys.append((s, e))

        return valleys

    def _select_best_sector(
        self,
        valleys: List[Tuple[int, int]],
        goal_angle: float,
        current_vz: float,
    ) -> int:
        """从所有谷中选择最接近目标方向的扇区中心。"""
        cfg = self.config
        goal_sector = self._angle_to_sector(goal_angle)

        # 1. 如果目标扇区本身就在某个可通行谷中，直接走目标方向
        for start, end in valleys:
            if start <= end:
                if start <= goal_sector <= end:
                    return goal_sector
            else:
                # 跨越 0° 的谷
                if goal_sector >= start or goal_sector <= end:
                    return goal_sector

        # 2. 目标方向被阻挡，在所有谷中选择最接近目标的中心
        best_sector = None
        best_cost = float('inf')

        for start, end in valleys:
            if start <= end:
                center = (start + end) // 2
            else:
                width = (cfg.num_sectors - start) + end + 1
                center = (start + width // 2) % cfg.num_sectors

            diff = abs(center - goal_sector)
            diff = min(diff, cfg.num_sectors - diff)
            cost = diff * cfg.goal_weight

            if cost < best_cost:
                best_cost = cost
                best_sector = center

        return best_sector if best_sector is not None else goal_sector

    def _angle_to_sector(self, angle: float) -> int:
        """将弧度角度转换为扇区索引。"""
        cfg = self.config
        # 规范化到 [0, 2pi)
        angle = angle % (2 * math.pi)
        if angle < 0:
            angle += 2 * math.pi
        sector = int(round(angle / (2 * math.pi) * cfg.num_sectors)) % cfg.num_sectors
        return sector

    def _sector_to_angle(self, sector: int) -> float:
        """将扇区索引转换为弧度角度。"""
        cfg = self.config
        return sector * (2 * math.pi / cfg.num_sectors)
