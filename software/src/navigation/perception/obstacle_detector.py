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
        fov_horizontal: float = math.radians(66.0),
        fov_vertical: float = math.radians(37.0),
        min_obstacle_size_m: float = 0.1,
        max_depth: float = 1.0,
        num_columns: int = 21,
        ground_threshold_ratio: float = 0.7,
        edge_threshold: float = -0.15,
    ):
        """
        Args:
            fov_horizontal: 水平视场角（弧度）
            fov_vertical: 垂直视场角（弧度）
            min_obstacle_size_m: 最小障碍物尺寸（米，用于过滤噪声）
            max_depth: 最大有效深度（相对深度的上限，用于过滤远景）
            num_columns: 水平分块数量
            ground_threshold_ratio: 地面深度阈值比例（底部区域超过该比例视为地面）
            edge_threshold: 深度突变检测阈值（一阶差分，负值表示从远到近）
        """
        self.fov_h = fov_horizontal
        self.fov_v = fov_vertical
        self.min_size = min_obstacle_size_m
        self.max_depth = max_depth
        self.num_columns = num_columns
        self.ground_threshold = ground_threshold_ratio
        self.edge_threshold = edge_threshold

    def detect(
        self,
        depth: np.ndarray,
        bgr_image: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """从深度图中检测障碍物。

        内部已改为基于 detect_histogram 生成伪障碍物列表。
        """
        histogram = self.detect_histogram(depth)
        return histogram

    def detect_histogram(self, disparity: np.ndarray) -> np.ndarray:
        """将二维视差图压缩为一维距离直方图（带地面锚点标定）。

        输入为原始视差（MiDaS 输出）：值越大表示越近。
        利用图像底部 10% 区域的 15% 分位数作为深度锚点，对应绝对深度 0.8m，
        通过视差 ∝ 1/深度 的关系恢复各条带的绝对距离。

        Args:
            disparity: 原始视差图 (H, W)，越大越近

        Returns:
            distances: np.ndarray, shape (num_columns,)
                       每个条带方向的最近障碍物距离（米），
                       无有效障碍物时返回 np.inf
        """
        h, w = disparity.shape
        col_width = w // self.num_columns
        distances = np.full(self.num_columns, 0, dtype=np.float32)

        # 深度锚点：底部 10% 区域的 75% 分位数视差，对应绝对深度 1.0m
        anchor_rows = max(1, int(h * 0.1))
        anchor_disp = float(np.percentile(disparity[-anchor_rows:, :], 75))
        if anchor_disp < 1e-6:
            anchor_disp = 1e-6

        # 比例因子：depth = scale / disp
        scale = 1.0 / anchor_disp

        for i in range(self.num_columns):
            x1 = i * col_width
            x2 = (i + 1) * col_width if i < self.num_columns - 1 else w
            col_disp = disparity[:, x1:x2]
            col_median = np.median(col_disp, axis=1)

            obstacle_disp = float(col_median[int(h*0.4):int(h*0.8)].min())
            distances[i] = scale * obstacle_disp

        return distances

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
