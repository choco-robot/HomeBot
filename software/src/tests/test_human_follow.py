#!/usr/bin/env python3
"""
人体跟随功能测试

测试内容:
1. 检测器初始化
2. 跟踪器功能
3. 控制器计算
4. 整体集成

运行方式:
    cd software/src
    python -m tests.test_human_follow

注意: 需要先有YOLO26模型文件
"""
import sys
import os
import time
import unittest
from unittest.mock import Mock, MagicMock
import numpy as np

# 添加src到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestHumanDetector(unittest.TestCase):
    """测试人体检测器"""
    
    @classmethod
    def setUpClass(cls):
        """测试类开始前执行"""
        print("\n" + "=" * 60)
        print("测试 HumanDetector")
        print("=" * 60)
    
    def test_initialization(self):
        """测试初始化"""
        from applications.human_follow.detector import HumanDetector
        
        detector = HumanDetector(
            model_path="models/yolo26n.pt",
            conf_threshold=0.5
        )
        
        self.assertEqual(detector.model_path.name, "yolo26n.pt")
        self.assertEqual(detector.conf_threshold, 0.5)
        self.assertFalse(detector._initialized)
        print("✓ 检测器初始化测试通过")
    
    def test_detection_data_structure(self):
        """测试检测结果数据结构"""
        from applications.human_follow.detector import Detection
        
        det = Detection(
            bbox=(100, 200, 300, 400),
            confidence=0.85,
            class_id=0,
            class_name="person"
        )
        
        self.assertEqual(det.bbox, (100, 200, 300, 400))
        self.assertEqual(det.confidence, 0.85)
        self.assertEqual(det.center, (200, 300))
        self.assertEqual(det.area, 40000)
        print("✓ 检测结果数据结构测试通过")


class TestTargetTracker(unittest.TestCase):
    """测试目标跟踪器"""
    
    @classmethod
    def setUpClass(cls):
        print("\n" + "=" * 60)
        print("测试 TargetTracker")
        print("=" * 60)
    
    def test_initialization(self):
        """测试初始化"""
        from applications.human_follow.tracker import TargetTracker
        
        tracker = TargetTracker(
            max_age=30,
            min_iou=0.3,
            selection_strategy="center"
        )
        
        self.assertEqual(tracker.max_age, 30)
        self.assertEqual(tracker.min_iou, 0.3)
        self.assertEqual(tracker.selection_strategy, "center")
        print("✓ 跟踪器初始化测试通过")
    
    def test_iou_calculation(self):
        """测试IoU计算"""
        from applications.human_follow.tracker import compute_iou
        
        # 完全重叠
        box1 = (0, 0, 100, 100)
        box2 = (0, 0, 100, 100)
        self.assertEqual(compute_iou(box1, box2), 1.0)
        
        # 一半重叠
        box1 = (0, 0, 100, 100)
        box2 = (50, 0, 150, 100)
        self.assertAlmostEqual(compute_iou(box1, box2), 1/3, places=2)
        
        # 不重叠
        box1 = (0, 0, 100, 100)
        box2 = (200, 200, 300, 300)
        self.assertEqual(compute_iou(box1, box2), 0.0)
        
        print("✓ IoU计算测试通过")
    
    def test_target_selection(self):
        """测试目标选择"""
        from applications.human_follow.tracker import TargetTracker, Target, TargetStatus
        from applications.human_follow.detector import Detection
        
        tracker = TargetTracker(selection_strategy="largest")
        
        # 创建两个检测
        det1 = Detection(bbox=(0, 0, 100, 100), confidence=0.8, class_id=0, class_name="person")
        det2 = Detection(bbox=(0, 0, 200, 200), confidence=0.9, class_id=0, class_name="person")
        
        # 更新跟踪
        tracker.update([det1, det2])
        
        # 检查选择了最大的目标
        primary = tracker.get_primary_target()
        self.assertIsNotNone(primary)
        self.assertEqual(primary.area, 40000)  # 200*200
        
        print("✓ 目标选择测试通过")


class TestFollowController(unittest.TestCase):
    """测试跟随控制器"""
    
    @classmethod
    def setUpClass(cls):
        print("\n" + "=" * 60)
        print("测试 FollowController")
        print("=" * 60)
    
    def test_initialization(self):
        """测试初始化"""
        from applications.human_follow.controller import FollowController
        
        controller = FollowController(
            target_distance=1.0,
            kp_linear=0.001,
            kp_angular=0.003,
            max_linear_speed=0.3,
            max_angular_speed=0.8
        )
        
        self.assertEqual(controller.target_distance, 1.0)
        self.assertEqual(controller.max_linear_speed, 0.3)
        self.assertEqual(controller.max_angular_speed, 0.8)
        print("✓ 控制器初始化测试通过")
    
    def test_velocity_calculation(self):
        """测试速度计算"""
        from applications.human_follow.controller import FollowController, VelocityCommand
        from applications.human_follow.tracker import Target, TargetStatus
        
        controller = FollowController(
            target_distance=1.0,
            kp_linear=0.001,
            kp_angular=0.003,
            max_linear_speed=0.3,
            max_angular_speed=0.8,
            frame_width=640,
            frame_height=480
        )
        
        # 创建目标在画面中央 (100x140 面积的框)
        # bbox中心 = ((270+370)/2, (170+310)/2) = (320, 240) = 画面中央
        target = Target(
            id=1,
            bbox=(270, 170, 370, 310),  # 中心约(320, 240), 面积=100*140=14000
            confidence=0.9,
            status=TargetStatus.TRACKING
        )
        
        # 计算速度
        cmd = controller.compute_velocity(target)
        
        self.assertIsNotNone(cmd)
        self.assertIsInstance(cmd, VelocityCommand)
        # 速度值应该在限制范围内
        self.assertLessEqual(abs(cmd.vx), controller.max_linear_speed)
        self.assertLessEqual(abs(cmd.vz), controller.max_angular_speed)
        
        print("✓ 速度计算测试通过")
    
    def test_velocity_limits(self):
        """测试速度限制"""
        from applications.human_follow.controller import FollowController, VelocityCommand
        
        controller = FollowController(
            max_linear_speed=0.3,
            max_angular_speed=0.8
        )
        
        # 测试限制函数 (min_val, value, max_val)
        clamped = controller._clamp(0.5, -0.3, 0.3)
        self.assertEqual(clamped, 0.3)  # value > max, return max
        
        clamped = controller._clamp(-0.5, -0.3, 0.3)
        self.assertEqual(clamped, -0.3)  # value < min, return min
        
        clamped = controller._clamp(0.1, -0.3, 0.3)
        self.assertEqual(clamped, 0.1)  # min < value < max, return value
        
        print("✓ 速度限制测试通过")


class TestIntegration(unittest.TestCase):
    """集成测试"""
    
    @classmethod
    def setUpClass(cls):
        print("\n" + "=" * 60)
        print("集成测试")
        print("=" * 60)
    
    def test_config_loading(self):
        """测试配置加载"""
        from configs import get_config, HumanFollowConfig
        
        config = get_config()
        self.assertIsNotNone(config.human_follow)
        self.assertIsInstance(config.human_follow, HumanFollowConfig)
        self.assertEqual(config.human_follow.model_path, "models/yolo26n.pt")
        
        print("✓ 配置加载测试通过")
    
    def test_follow_status(self):
        """测试状态管理"""
        from applications.human_follow.follow import FollowMode, FollowStatus
        from applications.human_follow.controller import VelocityCommand
        
        status = FollowStatus(
            mode=FollowMode.FOLLOWING,
            target_id=1,
            target_confidence=0.85,
            velocity=VelocityCommand(0.1, 0.0, 0.05),
            fps=25.0
        )
        
        self.assertEqual(status.mode, FollowMode.FOLLOWING)
        self.assertEqual(status.target_id, 1)
        self.assertEqual(status.fps, 25.0)
        
        print("✓ 状态管理测试通过")


def run_tests():
    """运行所有测试"""
    print("\n" + "=" * 60)
    print("人体跟随功能测试套件")
    print("=" * 60)
    
    # 创建测试套件
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # 添加测试类
    suite.addTests(loader.loadTestsFromTestCase(TestHumanDetector))
    suite.addTests(loader.loadTestsFromTestCase(TestTargetTracker))
    suite.addTests(loader.loadTestsFromTestCase(TestFollowController))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegration))
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 输出结果
    print("\n" + "=" * 60)
    print("测试摘要")
    print("=" * 60)
    print(f"测试总数: {result.testsRun}")
    print(f"通过: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"失败: {len(result.failures)}")
    print(f"错误: {len(result.errors)}")
    
    if result.wasSuccessful():
        print("\n✓ 所有测试通过!")
        return 0
    else:
        print("\n✗ 测试未通过")
        return 1


if __name__ == "__main__":
    exit(run_tests())
