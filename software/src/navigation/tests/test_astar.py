# -*- coding: utf-8 -*-
"""A* 规划器和栅格地图单元测试"""
import time
import unittest

from navigation.core.occupancy_grid import COST_FREE, COST_LETHAL, OccupancyGrid
from navigation.core.astar_planner import AStarPlanner, euclidean_distance


class TestOccupancyGrid(unittest.TestCase):
    def test_basic_creation(self):
        grid = OccupancyGrid(10, 10, resolution=0.1, origin=(0.0, 0.0))
        self.assertEqual(grid.width, 10)
        self.assertEqual(grid.height, 10)
        self.assertEqual(grid.get_cost(0, 0), COST_FREE)
        self.assertEqual(grid.get_cost(-1, 0), -1)  # 越界返回 UNKNOWN

    def test_coordinate_conversion(self):
        grid = OccupancyGrid(100, 100, resolution=0.05, origin=(-2.5, -2.5))
        # 世界 (0,0) 对应栅格 (50,50)
        self.assertEqual(grid.world_to_grid(0.0, 0.0), (50, 50))
        wx, wy = grid.grid_to_world(50, 50)
        self.assertAlmostEqual(wx, 0.025, places=5)
        self.assertAlmostEqual(wy, 0.025, places=5)

    def test_set_cost_and_obstacles(self):
        grid = OccupancyGrid(20, 20, resolution=0.1)
        grid.set_cost(5, 5, COST_LETHAL)
        self.assertEqual(grid.get_cost(5, 5), COST_LETHAL)
        grid.set_rectangle(10, 10, 3, 3, COST_LETHAL)
        self.assertEqual(grid.get_cost(11, 11), COST_LETHAL)
        self.assertEqual(grid.get_cost(9, 9), COST_FREE)

    def test_circle_world(self):
        grid = OccupancyGrid(100, 100, resolution=0.05, origin=(0.0, 0.0))
        grid.set_circle_world(2.5, 2.5, 0.2, COST_LETHAL)
        # 圆心附近应为障碍
        cx, cy = grid.world_to_grid(2.5, 2.5)
        self.assertEqual(grid.get_cost(cx, cy), COST_LETHAL)
        # 稍远处应为自由
        self.assertEqual(grid.get_cost(cx + 10, cy), COST_FREE)

    def test_inflation(self):
        grid = OccupancyGrid(40, 40, resolution=0.05)
        grid.set_cost(20, 20, COST_LETHAL)
        grid.inflate_obstacles(0.15)
        # 膨胀后周围应有代价值
        self.assertTrue(grid.get_cost(21, 20) > COST_FREE)
        self.assertTrue(grid.get_cost(20, 21) > COST_FREE)
        # 远处仍应为自由
        self.assertEqual(grid.get_cost(0, 0), COST_FREE)

    def test_serialization(self):
        grid = OccupancyGrid(10, 10, resolution=0.1)
        grid.set_cost(3, 4, COST_LETHAL)
        data = grid.to_dict()
        grid2 = OccupancyGrid.from_dict(data)
        self.assertEqual(grid2.get_cost(3, 4), COST_LETHAL)
        self.assertEqual(grid2.resolution, 0.1)


class TestAStarPlanner(unittest.TestCase):
    def _make_grid_with_wall(self, gap_y: int = 50) -> OccupancyGrid:
        """创建带一堵墙的地图，墙中间有缺口"""
        grid = OccupancyGrid(100, 100, resolution=0.05, origin=(0.0, 0.0))
        grid.set_rectangle(40, 0, 5, 100, COST_LETHAL)
        # 在 gap_y 处打开缺口
        grid.set_rectangle(40, gap_y, 5, 10, COST_FREE)
        return grid

    def test_straight_line(self):
        grid = OccupancyGrid(50, 50, resolution=0.1)
        planner = AStarPlanner(grid)
        start = (0.5, 0.5)
        goal = (2.5, 0.5)
        path = planner.plan(start, goal)
        self.assertIsNotNone(path)
        # world->grid->world 会映射到栅格中心，允许误差
        self.assertAlmostEqual(path[0][0], start[0], delta=grid.resolution)
        self.assertAlmostEqual(path[0][1], start[1], delta=grid.resolution)
        self.assertAlmostEqual(path[-1][0], goal[0], delta=grid.resolution)
        self.assertAlmostEqual(path[-1][1], goal[1], delta=grid.resolution)

    def test_blocked_goal(self):
        grid = OccupancyGrid(20, 20, resolution=0.1)
        grid.set_cost(15, 15, COST_LETHAL)
        planner = AStarPlanner(grid)
        path = planner.plan((0.5, 0.5), (1.55, 1.55))
        self.assertIsNone(path)

    def test_u_shape_obstacle(self):
        grid = OccupancyGrid(100, 100, resolution=0.05, origin=(0.0, 0.0))
        # U 型障碍：左、右、下三面墙
        grid.set_rectangle(20, 20, 5, 60, COST_LETHAL)   # 左
        grid.set_rectangle(75, 20, 5, 60, COST_LETHAL)   # 右
        grid.set_rectangle(20, 75, 60, 5, COST_LETHAL)   # 下
        planner = AStarPlanner(grid)
        # 注意：100x100@0.05m 最大坐标为 4.975m（栅格99中心）
        path = planner.plan((2.5, 2.5), (4.5, 4.5))
        self.assertIsNotNone(path)

    def test_narrow_passage(self):
        grid = self._make_grid_with_wall(gap_y=45)
        planner = AStarPlanner(grid)
        start = (1.0, 2.5)
        goal = (4.0, 2.5)
        path = planner.plan(start, goal)
        self.assertIsNotNone(path)
        # 路径应经过缺口，起点终点允许栅格中心误差
        self.assertAlmostEqual(path[0][0], start[0], delta=grid.resolution)
        self.assertAlmostEqual(path[0][1], start[1], delta=grid.resolution)
        self.assertAlmostEqual(path[-1][0], goal[0], delta=grid.resolution)
        self.assertAlmostEqual(path[-1][1], goal[1], delta=grid.resolution)

    def test_simplified_path(self):
        grid = OccupancyGrid(50, 50, resolution=0.1)
        planner = AStarPlanner(grid)
        path = planner.plan_with_simplification((0.5, 0.5), (4.5, 0.5))
        # 直线路径简化后应只有起点和终点
        self.assertEqual(len(path), 2)

    def test_performance_100x100(self):
        grid = OccupancyGrid(100, 100, resolution=0.05, origin=(0.0, 0.0))
        grid.add_random_obstacles(count=20, seed=42)
        planner = AStarPlanner(grid)
        times = []
        for _ in range(20):
            t0 = time.perf_counter()
            path = planner.plan((0.5, 0.5), (4.5, 4.5))
            t1 = time.perf_counter()
            self.assertIsNotNone(path)
            times.append((t1 - t0) * 1000)
        avg_ms = sum(times) / len(times)
        self.assertLess(avg_ms, 50, f"A* 平均耗时 {avg_ms:.2f}ms，超过 50ms 阈值")

    def test_diagonal_vs_straight(self):
        grid = OccupancyGrid(20, 20, resolution=0.1)
        planner_diag = AStarPlanner(grid, allow_diagonal=True)
        planner_straight = AStarPlanner(grid, allow_diagonal=False)

        path_diag = planner_diag.plan((0.5, 0.5), (1.5, 1.5))
        path_straight = planner_straight.plan((0.5, 0.5), (1.5, 1.5))

        self.assertIsNotNone(path_diag)
        self.assertIsNotNone(path_straight)
        # 允许斜向时路径点应更少
        self.assertLess(len(path_diag), len(path_straight))

    def test_heuristic_custom(self):
        grid = OccupancyGrid(20, 20, resolution=0.1)
        planner = AStarPlanner(grid)
        path = planner.plan((0.5, 0.5), (1.5, 1.5), heuristic=euclidean_distance)
        self.assertIsNotNone(path)


if __name__ == "__main__":
    unittest.main()
