# -*- coding: utf-8 -*-
"""基于深度图的简化障碍物检测器

策略：
1. 将深度图分为若干个垂直条带（columns）
2. 对每个条带取中值深度，代表该方向的障碍物距离
3. 检测深度突变区域，标记为潜在障碍物
4. 输出障碍物列表（相对相机坐标）

坐标系：
- x: 水平方向（左负右正），单位：米（估算）
- y: 垂直方向（下负上正），单位：米（估算）
- z: 深度方向（正前方），单位：米（相对深度，非绝对）
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from common.logging import get_logger

logger = get_logger(__name__)


@dataclass
class DepthObstacle:
    """深度图检测到的障碍物"""
    x: float          # 水平中心位置（米，估算）
    y: float          # 垂直中心位置（米，估算）
    z: float          # 深度值（0~1 相对值，或估算的米）
    width: float      # 水平宽度（米，估算）
    height: float     # 垂直高度（米，估算）
    confidence: float # 置信度 0~1


class DepthObstacleDetector:
    """基于深度图的障碍物检测器。

    适用场景：
    - 检测悬空障碍物（桌面、椅子座位）
    - 检测地面障碍物（箱子、墙壁）
    """

    def __init__(
        self,
        fov_horizontal: float = math.radians(60.0),
        fov_vertical: float = math.radians(45.0),
        min_obstacle_size_m: float = 0.1,
        max_depth: float = 1.0,
        num_columns: int = 20,
        ground_threshold_ratio: float = 0.7,
    ):
        """
        Args:
            fov_horizontal: 水平视场角（弧度）
            fov_vertical: 垂直视场角（弧度）
            min_obstacle_size_m: 最小障碍物尺寸（米，用于过滤噪声）
            max_depth: 最大有效深度（相对深度的上限，用于过滤远景）
            num_columns: 水平分块数量
            ground_threshold_ratio: 地面深度阈值比例（底部区域超过该比例视为地面）
        """
        self.fov_h = fov_horizontal
        self.fov_v = fov_vertical
        self.min_size = min_obstacle_size_m
        self.max_depth = max_depth
        self.num_columns = num_columns
        self.ground_threshold = ground_threshold_ratio

    def detect(
        self,
        depth: np.ndarray,
        bgr_image: Optional[np.ndarray] = None,
    ) -> List[DepthObstacle]:
        """从深度图中检测障碍物。

        Args:
            depth: 相对深度图 (H, W)，值域 0~1，越大越远
            bgr_image: 可选的原始彩色图，用于辅助调试（当前未使用）

        Returns:
            障碍物列表
        """
        h, w = depth.shape
        col_width = w // self.num_columns

        # 1. 简单的地面分割：取下 20% 行作为地面参考
        ground_rows = int(h * 0.2)
        ground_depth = np.median(depth[-ground_rows:, :]) if ground_rows > 0 else 0.5

        obstacles = []
        for i in range(self.num_columns):
            x1 = i * col_width
            x2 = (i + 1) * col_width if i < self.num_columns - 1 else w
            col_depth = depth[:, x1:x2]

            # 取中值深度曲线（沿垂直方向）
            col_median = np.median(col_depth, axis=1)

            # 忽略过远的区域
            valid_mask = col_median < self.max_depth
            if not np.any(valid_mask):
                continue

            # 检测深度不连续（突然变近）
            # 使用一阶差分检测突变
            diff = np.diff(col_median)
            # 寻找显著的正差分（从远到近）
            edge_indices = np.where(diff < -0.15)[0]
            if len(edge_indices) == 0:
                continue

            # 取第一个显著的近处边缘
            edge_y = int(edge_indices[0])
            obstacle_depth = float(col_median[edge_y])

            # 估算实际位置（使用小孔模型近似）
            cx = (x1 + x2) / 2
            cy = edge_y
            x_m, y_m, z_m = self._pixel_to_camera(cx, cy, obstacle_depth, w, h)

            # 估算宽度和高度
            # 宽度 ≈ 柱子物理宽度
            col_angle = self.fov_h / self.num_columns
            width_m = 2 * z_m * math.tan(col_angle / 2)
            # 高度 ≈ 从边缘到图像顶部的物理高度
            height_m = max(0.05, -y_m)  # 保守估计

            if width_m < self.min_size or height_m < self.min_size:
                continue

            # 过滤明显是地面的情况：如果边缘在图像底部且深度与地面接近
            if edge_y > h * self.ground_threshold and abs(obstacle_depth - ground_depth) < 0.1:
                continue

            obstacles.append(DepthObstacle(
                x=x_m,
                y=y_m,
                z=z_m,
                width=width_m,
                height=height_m,
                confidence=min(1.0, abs(diff[edge_y]) * 3.0),
            ))

        # 2. 进一步过滤：按 x 位置去重（合并相邻柱子）
        merged = self._merge_obstacles(obstacles)
        return merged

    def _pixel_to_camera(
        self,
        px: float,
        py: float,
        depth_val: float,
        img_w: int,
        img_h: int,
    ) -> Tuple[float, float, float]:
        """将像素坐标和相对深度转换为相机坐标系下的近似位置。

        这里 depth_val 是相对深度 0~1，我们将其映射到 0.1m ~ 3.0m 的绝对深度范围
        用于演示和局部避障。实际使用时需要标定相机内参。
        """
        # 将相对深度映射到估算的绝对深度（越远值越大）
        # depth_val=0 -> 0.1m, depth_val=1 -> 3.0m
        z_m = 0.1 + depth_val * 2.9

        cx = img_w / 2
        cy = img_h / 2
        fx = (img_w / 2) / math.tan(self.fov_h / 2)
        fy = (img_h / 2) / math.tan(self.fov_v / 2)

        x_m = (px - cx) * z_m / fx
        y_m = (py - cy) * z_m / fy
        return x_m, y_m, z_m

    def _merge_obstacles(self, obstacles: List[DepthObstacle]) -> List[DepthObstacle]:
        """合并相邻的障碍物。"""
        if not obstacles:
            return []
        # 按 x 排序
        obstacles = sorted(obstacles, key=lambda o: o.x)
        merged = [obstacles[0]]
        for obs in obstacles[1:]:
            last = merged[-1]
            # 如果水平距离小于平均宽度的一半，合并
            if abs(obs.x - last.x) < (last.width + obs.width) / 2:
                # 加权平均
                total_conf = last.confidence + obs.confidence
                last.x = (last.x * last.confidence + obs.x * obs.confidence) / total_conf
                last.y = (last.y * last.confidence + obs.y * obs.confidence) / total_conf
                last.z = (last.z * last.confidence + obs.z * obs.confidence) / total_conf
                last.width += obs.width * 0.5
                last.height = max(last.height, obs.height)
                last.confidence = min(1.0, total_conf)
            else:
                merged.append(obs)
        return merged

    def draw_overlay(
        self,
        bgr_image: np.ndarray,
        obstacles: List[DepthObstacle],
        depth: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """在图像上绘制障碍物检测框和深度信息。"""
        overlay = bgr_image.copy()
        h, w = overlay.shape[:2]

        for obs in obstacles:
            # 将相机坐标反投影到像素（简化）
            px = int(w / 2 + obs.x * w / (2 * math.tan(self.fov_h / 2) * obs.z))
            py = int(h / 2 + obs.y * h / (2 * math.tan(self.fov_v / 2) * obs.z))
            box_w = int(obs.width * w / (2 * math.tan(self.fov_h / 2) * obs.z))
            box_h = int(obs.height * h / (2 * math.tan(self.fov_v / 2) * obs.z))

            x1 = max(0, px - box_w // 2)
            y1 = max(0, py - box_h // 2)
            x2 = min(w, px + box_w // 2)
            y2 = min(h, py + box_h // 2)

            color = (0, 255, 0) if obs.y < -0.1 else (0, 165, 255)  # 悬空用橙色，地面用绿色
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
            label = f"z={obs.z:.2f}m h={obs.height:.2f}m"
            cv2.putText(overlay, label, (x1, max(20, y1 - 5)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # 绘制底部深度图小窗
        if depth is not None:
            depth_color = cv2.applyColorMap((depth * 255).clip(0, 255).astype(np.uint8), cv2.COLORMAP_JET)
            mini_h = h // 4
            mini_w = w // 4
            depth_mini = cv2.resize(depth_color, (mini_w, mini_h))
            overlay[h - mini_h:h, 0:mini_w] = depth_mini
            cv2.putText(overlay, "Depth", (5, h - mini_h + 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        return overlay
