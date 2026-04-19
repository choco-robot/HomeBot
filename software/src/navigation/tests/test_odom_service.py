# -*- coding: utf-8 -*-
"""OdomService 单元测试"""
import math
import unittest

from navigation.services.odom_service import OdomService


class TestOdomServiceIntegration(unittest.TestCase):
    def test_integrate_forward(self):
        odom = OdomService()
        odom._integrate(vx=1.0, vy=0.0, vz=0.0, dt=1.0)
        self.assertAlmostEqual(odom.x, 1.0, places=4)
        self.assertAlmostEqual(odom.y, 0.0, places=4)
        self.assertAlmostEqual(odom.yaw, 0.0, places=4)

    def test_integrate_rotate(self):
        odom = OdomService()
        odom._integrate(vx=0.0, vy=0.0, vz=math.radians(90), dt=1.0)
        self.assertAlmostEqual(odom.x, 0.0, places=4)
        self.assertAlmostEqual(odom.y, 0.0, places=4)
        self.assertAlmostEqual(odom.yaw, math.radians(90), places=4)

    def test_integrate_arc(self):
        odom = OdomService()
        # 以 1.0 rad/s 旋转，同时前进 1.0 m/s，1秒后转过1弧度
        # 中值积分：mid_yaw = 0.5，dx = cos(0.5)，dy = sin(0.5)
        odom._integrate(vx=1.0, vy=0.0, vz=1.0, dt=1.0)
        self.assertAlmostEqual(odom.x, math.cos(0.5), places=2)
        self.assertAlmostEqual(odom.y, math.sin(0.5), places=2)
        self.assertAlmostEqual(odom.yaw, 1.0, places=4)

    def test_yaw_normalization(self):
        odom = OdomService()
        odom._integrate(vx=0.0, vy=0.0, vz=math.pi * 3, dt=1.0)
        self.assertTrue(-math.pi <= odom.yaw <= math.pi)

    def test_reset_pose(self):
        odom = OdomService()
        odom._integrate(vx=1.0, vy=0.0, vz=0.0, dt=1.0)
        odom.reset_pose(x=2.0, y=3.0, yaw=math.pi / 2)
        self.assertEqual(odom.x, 2.0)
        self.assertEqual(odom.y, 3.0)
        self.assertEqual(odom.yaw, math.pi / 2)


if __name__ == "__main__":
    unittest.main()
