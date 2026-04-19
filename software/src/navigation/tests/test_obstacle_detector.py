# -*- coding: utf-8 -*-
"""障碍物检测器单元测试"""
import math
import unittest

import numpy as np

from navigation.perception.obstacle_detector import DepthObstacleDetector


class TestDepthObstacleDetector(unittest.TestCase):
    def setUp(self):
        self.detector = DepthObstacleDetector(
            fov_horizontal=math.radians(60),
            fov_vertical=math.radians(45),
            num_columns=20,
        )
        self.h, self.w = 240, 320

    def _make_depth_with_obstacle(self, center_x_ratio: float = 0.5, depth_val: float = 0.2) -> np.ndarray:
        """生成一个悬空障碍物在指定水平位置的深度图"""
        depth = np.ones((self.h, self.w), dtype=np.float32) * 0.9  # 远景
        x_start = int(self.w * (center_x_ratio - 0.15))
        x_end = int(self.w * (center_x_ratio + 0.15))
        y_start = int(self.h * 0.3)
        y_end = int(self.h * 0.7)
        depth[y_start:y_end, x_start:x_end] = depth_val  # 近处障碍物
        return depth

    def test_no_obstacles_in_empty_scene(self):
        depth = np.ones((self.h, self.w), dtype=np.float32) * 0.9
        obstacles = self.detector.detect(depth)
        self.assertEqual(len(obstacles), 0)

    def test_detect_obstacle(self):
        depth = self._make_depth_with_obstacle(center_x_ratio=0.5, depth_val=0.2)
        obstacles = self.detector.detect(depth)
        self.assertGreater(len(obstacles), 0)
        # 至少有一个障碍物在图像中心附近
        xs = [o.x for o in obstacles]
        self.assertTrue(any(abs(x) < 0.5 for x in xs))

    def test_obstacle_has_positive_dimensions(self):
        depth = self._make_depth_with_obstacle(depth_val=0.2)
        obstacles = self.detector.detect(depth)
        self.assertGreater(len(obstacles), 0)
        obs = obstacles[0]
        self.assertGreater(obs.width, 0)
        self.assertGreater(obs.height, 0)
        self.assertGreaterEqual(obs.confidence, 0)
        self.assertLessEqual(obs.confidence, 1)

    def test_ground_filtering(self):
        """底部与地面深度接近的区域应被过滤"""
        depth = np.ones((self.h, self.w), dtype=np.float32) * 0.8
        # 模拟地面：底部 30% 行深度一致
        ground_rows = int(self.h * 0.3)
        depth[-ground_rows:, :] = 0.5
        obstacles = self.detector.detect(depth)
        # 纯地面不应产生障碍物
        self.assertEqual(len(obstacles), 0)

    def test_draw_overlay(self):
        depth = self._make_depth_with_obstacle(depth_val=0.2)
        obstacles = self.detector.detect(depth)
        bgr = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        overlay = self.detector.draw_overlay(bgr, obstacles, depth)
        self.assertEqual(overlay.shape, (self.h, self.w, 3))


if __name__ == "__main__":
    unittest.main()
