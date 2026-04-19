# -*- coding: utf-8 -*-
"""局部规划器单元测试"""
import math
import unittest

from navigation.perception.obstacle_detector import DepthObstacle
from navigation.planning.local_planner import LocalPlannerConfig, VFHLocalPlanner


class TestVFHLocalPlanner(unittest.TestCase):
    def setUp(self):
        self.planner = VFHLocalPlanner(LocalPlannerConfig(
            max_linear_speed=0.5,
            max_angular_speed=1.0,
            num_sectors=36,
            safety_distance_m=0.5,
            obstacle_threshold=0.3,
        ))

    def test_free_space_to_goal(self):
        """无障碍时直接朝目标前进"""
        obstacles = []
        vx, vz = self.planner.plan(obstacles, goal_x=0.0, goal_y=2.0)
        self.assertGreater(vx, 0.1)   # 应前进
        self.assertAlmostEqual(vz, 0.0, places=2)  # 不应旋转

    def test_avoid_obstacle_in_front(self):
        """正前方有障碍时应绕行"""
        obstacles = [
            DepthObstacle(x=0.0, y=0.0, z=0.3, width=0.3, height=0.3, confidence=1.0),
        ]
        vx, vz = self.planner.plan(obstacles, goal_x=0.0, goal_y=2.0)
        # 正前方被阻挡，应该有旋转分量
        self.assertNotAlmostEqual(vz, 0.0, places=2)

    def test_slow_down_near_goal(self):
        """接近目标时减速"""
        obstacles = []
        vx, vz = self.planner.plan(obstacles, goal_x=0.0, goal_y=0.1)
        self.assertLess(vx, 0.2)  # 接近目标应减速

    def test_blocked_all_directions(self):
        """所有方向被阻挡时应原地旋转"""
        obstacles = []
        for angle in range(0, 360, 15):
            rad = math.radians(angle)
            obstacles.append(DepthObstacle(
                x=0.3 * math.cos(rad), y=0.0, z=0.3 * math.sin(rad),
                width=0.2, height=0.2, confidence=1.0
            ))
        vx, vz = self.planner.plan(obstacles, goal_x=0.0, goal_y=2.0)
        self.assertAlmostEqual(vx, 0.0, places=2)
        self.assertGreater(abs(vz), 0.1)


class TestCostmapGenerator(unittest.TestCase):
    def test_generate_empty(self):
        from navigation.planning.costmap_generator import LocalCostmapGenerator
        gen = LocalCostmapGenerator(width_m=2.0, height_m=2.0, resolution=0.1)
        grid = gen.generate([])
        self.assertEqual(grid.width, 20)
        self.assertEqual(grid.height, 20)
        # 无障碍物，地图应全为 FREE
        self.assertTrue((grid.data == 0).all())

    def test_generate_with_obstacle(self):
        from navigation.planning.costmap_generator import LocalCostmapGenerator
        gen = LocalCostmapGenerator(width_m=2.0, height_m=2.0, resolution=0.1)
        obstacles = [DepthObstacle(x=0.5, y=0.0, z=0.5, width=0.2, height=0.2, confidence=1.0)]
        grid = gen.generate(obstacles)
        # 障碍物附近应有 LETHAL 值
        cx, cy = grid.world_to_grid(0.5, 0.5)
        self.assertGreater(grid.get_cost(cx, cy), 0)


if __name__ == "__main__":
    unittest.main()
