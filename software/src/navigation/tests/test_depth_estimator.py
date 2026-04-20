# -*- coding: utf-8 -*-
"""深度估计器单元测试"""
import unittest

import numpy as np

from navigation.perception.depth_estimator import DepthEstimator, create_depth_estimator


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
    def test_create_returns_depth_estimator(self):
        est = create_depth_estimator(model_path="/nonexistent/path/model.onnx")
        self.assertIsInstance(est, DepthEstimator)


if __name__ == "__main__":
    unittest.main()
