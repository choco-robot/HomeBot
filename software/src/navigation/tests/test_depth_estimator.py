# -*- coding: utf-8 -*-
"""深度估计器单元测试"""
import unittest

import cv2
import numpy as np

from navigation.perception.depth_estimator import (
    DepthEstimator,
    FakeDepthEstimator,
    create_depth_estimator,
)


class TestFakeDepthEstimator(unittest.TestCase):
    def setUp(self):
        self.estimator = FakeDepthEstimator(mode="gradient")
        self.test_frame = np.zeros((240, 320, 3), dtype=np.uint8)

    def test_estimate_shape(self):
        depth = self.estimator.estimate(self.test_frame)
        self.assertEqual(depth.shape, (240, 320))
        self.assertEqual(depth.dtype, np.float32)

    def test_estimate_value_range(self):
        depth = self.estimator.estimate(self.test_frame)
        self.assertTrue(np.all(depth >= 0.0))
        self.assertTrue(np.all(depth <= 1.0))

    def test_gradient_mode(self):
        depth = self.estimator.estimate(self.test_frame)
        # 中心应该比角落浅（值更小）
        cy, cx = 120, 160
        self.assertLess(depth[cy, cx], depth[0, 0])

    def test_brightness_mode(self):
        est = FakeDepthEstimator(mode="brightness")
        bright = np.full((100, 100, 3), 255, dtype=np.uint8)
        dark = np.full((100, 100, 3), 0, dtype=np.uint8)
        depth_bright = est.estimate(bright)
        depth_dark = est.estimate(dark)
        # 越亮越近（深度值越小）
        self.assertLess(np.mean(depth_bright), np.mean(depth_dark))

    def test_colorize(self):
        depth = self.estimator.estimate(self.test_frame)
        colored = self.estimator.colorize(depth)
        self.assertEqual(colored.shape, (240, 320, 3))
        self.assertEqual(colored.dtype, np.uint8)


class TestDepthEstimator(unittest.TestCase):
    def test_unavailable_when_model_missing(self):
        est = DepthEstimator(model_path="/nonexistent/path/model.onnx")
        self.assertFalse(est.is_available)

    def test_estimate_raises_when_unavailable(self):
        est = DepthEstimator(model_path="/nonexistent/path/model.onnx")
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        with self.assertRaises(RuntimeError):
            est.estimate(frame)

    def test_estimate_safe_returns_none_when_unavailable(self):
        est = DepthEstimator(model_path="/nonexistent/path/model.onnx")
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        result = est.estimate_safe(frame)
        self.assertIsNone(result)


class TestFactory(unittest.TestCase):
    def test_create_real_when_available(self):
        # 即使模型不存在，allow_fake=True 也会回退
        est = create_depth_estimator(model_path="/nonexistent/path/model.onnx", allow_fake=True)
        self.assertIsInstance(est, FakeDepthEstimator)

    def test_create_fake_fallback(self):
        est = create_depth_estimator(model_path="/nonexistent/path/model.onnx", allow_fake=True)
        self.assertTrue(est.is_available)
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        depth = est.estimate(frame)
        self.assertEqual(depth.shape, (100, 100))

    def test_create_raises_when_no_fallback(self):
        with self.assertRaises(RuntimeError):
            create_depth_estimator(model_path="/nonexistent/path/model.onnx", allow_fake=False)


if __name__ == "__main__":
    unittest.main()
