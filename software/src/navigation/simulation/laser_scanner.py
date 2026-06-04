# -*- coding: utf-8 -*-
"""激光雷达传感器仿真

模拟 2D 激光雷达扫描：
- 射线投射算法检测障碍物
- 角度分辨率、最大距离、噪声模拟
- 与 BreezySLAM 兼容的输出格式
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from common.logging import get_logger

logger = get_logger(__name__)


@dataclass
class LaserConfig:
    """激光雷达配置"""

    num_rays: int = 360  # 射线数量
    max_range: float = 12.0  # 最大测量距离（米）
    min_range: float = 0.1  # 最小测量距离（米）
    fov: float = 2 * math.pi  # 视场角（弧度，360度）
    angular_resolution: float = None  # 角度分辨率（弧度），None 时自动计算

    # 噪声参数
    range_noise_std: float = 0.02  # 距离测量噪声标准差（米）
    angle_noise_std: float = 0.001  # 角度测量噪声标准差（弧度）

    # 测量失败模拟
    dropout_probability: float = 0.01  # 测量失败概率

    def __post_init__(self):
        if self.angular_resolution is None:
            self.angular_resolution = self.fov / self.num_rays


class LaserScanner:
    """激光雷达传感器

    功能：
    - 射线投射计算障碍物距离
    - 测量噪声模拟
    - 与地图环境交互
    """

    def __init__(
        self,
        config: LaserConfig = None,
        robot_radius: float = 0.15,
    ):
        """
        Args:
            config: 激光雷达配置
            robot_radius: 机器人半径（用于避开机器人自身）
        """
        self.config = config or LaserConfig()
        self.robot_radius = robot_radius

        # 预计算射线角度（从 -pi/2 开始，顺时针方向）
        self._angles = np.linspace(
            -self.config.fov / 2,
            self.config.fov / 2,
            self.config.num_rays,
            endpoint=False,
        )

    # --------------------------------------------------------------------------
    # 扫描接口
    # --------------------------------------------------------------------------

    def scan(
        self,
        robot_pose: Tuple[float, float, float],
        obstacles: List,
        map_grid: Optional[np.ndarray] = None,
        map_resolution: float = 0.05,
        map_origin: Tuple[float, float] = (0.0, 0.0),
    ) -> np.ndarray:
        """执行激光扫描

        Args:
            robot_pose: 机器人位姿 (x, y, theta)
            obstacles: 障碍物列表
            map_grid: 地图栅格（可选，用于墙壁等静态障碍）
            map_resolution: 地图分辨率（米/栅格）
            map_origin: 地图原点 (x, y)

        Returns:
            距离数组（米），shape = (num_rays,)
        """
        rx, ry, rtheta = robot_pose

        # 初始化距离数组（最大距离）
        ranges = np.full(self.config.num_rays, self.config.max_range)

        # 对每条射线进行投射
        for i, local_angle in enumerate(self._angles):
            # 世界坐标系下的射线方向
            world_angle = rtheta + local_angle

            # 添加角度噪声
            if self.config.angle_noise_std > 0:
                world_angle += np.random.normal(0, self.config.angle_noise_std)

            # 计算障碍物距离
            distance = self._cast_ray(
                start=(rx, ry),
                angle=world_angle,
                obstacles=obstacles,
                map_grid=map_grid,
                map_resolution=map_resolution,
                map_origin=map_origin,
            )

            # 添加距离噪声
            if distance < self.config.max_range and self.config.range_noise_std > 0:
                distance += np.random.normal(0, self.config.range_noise_std)
                distance = np.clip(
                    distance, self.config.min_range, self.config.max_range
                )

            # 模拟测量失败
            if np.random.random() < self.config.dropout_probability:
                distance = self.config.max_range  # 返回最大距离

            ranges[i] = distance

        return ranges

    def _cast_ray(
        self,
        start: Tuple[float, float],
        angle: float,
        obstacles: List,
        map_grid: Optional[np.ndarray],
        map_resolution: float,
        map_origin: Tuple[float, float],
    ) -> float:
        """单条射线投射

        Args:
            start: 射线起点 (x, y)
            angle: 射线方向（弧度）
            obstacles: 障碍物列表
            map_grid: 地图栅格
            map_resolution: 地图分辨率
            map_origin: 地图原点

        Returns:
            到最近障碍物的距离
        """
        min_distance = self.config.max_range

        # 1. 检查障碍物（圆形或多边形）
        for obs in obstacles:
            if len(obs) == 3:
                # 圆形障碍物 (x, y, radius)
                dist = self._ray_circle_intersection(
                    start, angle, (obs[0], obs[1]), obs[2]
                )
                if dist is not None:
                    min_distance = min(min_distance, dist)
            else:
                # 多边形障碍物
                dist = self._ray_polygon_intersection(start, angle, np.array(obs))
                if dist is not None:
                    min_distance = min(min_distance, dist)

        # 2. 检查地图栅格（墙壁等）
        if map_grid is not None:
            dist = self._ray_map_intersection(
                start, angle, map_grid, map_resolution, map_origin
            )
            if dist is not None:
                min_distance = min(min_distance, dist)

        # 确保不会打到机器人自身
        min_distance = max(min_distance, self.robot_radius)

        return min_distance

    # --------------------------------------------------------------------------
    # 射线-几何体相交计算
    # --------------------------------------------------------------------------

    def _ray_circle_intersection(
        self,
        ray_start: Tuple[float, float],
        ray_angle: float,
        circle_center: Tuple[float, float],
        circle_radius: float,
    ) -> Optional[float]:
        """射线与圆形相交

        Args:
            ray_start: 射线起点 (x, y)
            ray_angle: 射线方向（弧度）
            circle_center: 圆心 (x, y)
            circle_radius: 圆半径

        Returns:
            相交距离，None 表示不相交
        """
        # 射线方向向量
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)

        # 圆心到射线起点的向量
        fx = ray_start[0] - circle_center[0]
        fy = ray_start[1] - circle_center[1]

        # 二次方程求解：a*t^2 + b*t + c = 0
        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - circle_radius * circle_radius

        discriminant = b * b - 4 * a * c

        if discriminant < 0:
            return None

        # 取最近的交点
        t1 = (-b - math.sqrt(discriminant)) / (2 * a)
        t2 = (-b + math.sqrt(discriminant)) / (2 * a)

        # 只考虑正方向的交点
        t = t1 if t1 > 0 else t2
        if t <= 0:
            return None

        return t

    def _ray_polygon_intersection(
        self,
        ray_start: Tuple[float, float],
        ray_angle: float,
        polygon: np.ndarray,
    ) -> Optional[float]:
        """射线与多边形相交

        Args:
            ray_start: 射线起点 (x, y)
            ray_angle: 射线方向（弧度）
            polygon: 多边形顶点数组 (Nx2)

        Returns:
            相交距离，None 表示不相交
        """
        min_t = None

        # 射线方向向量
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)

        # 检查每条边
        n = len(polygon)
        for i in range(n):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % n]

            t = self._ray_segment_intersection(ray_start, (dx, dy), p1, p2)
            if t is not None:
                if min_t is None or t < min_t:
                    min_t = t

        return min_t

    @staticmethod
    def _ray_segment_intersection(
        ray_start: Tuple[float, float],
        ray_dir: Tuple[float, float],
        seg_start: Tuple[float, float],
        seg_end: Tuple[float, float],
    ) -> Optional[float]:
        """射线与线段相交

        Returns:
            相交参数 t（距离），None 表示不相交
        """
        # 射线：P = ray_start + t * ray_dir
        # 线段：Q = seg_start + s * (seg_end - seg_start), s ∈ [0, 1]

        rx, ry = ray_start
        rdx, rdy = ray_dir
        sx, sy = seg_start
        ex, ey = seg_end

        # 线段方向向量
        sdx = ex - sx
        sdy = ey - sy

        # 求解方程组
        denom = rdx * sdy - rdy * sdx

        if abs(denom) < 1e-10:
            return None  # 平行

        t = ((sx - rx) * sdy - (sy - ry) * sdx) / denom
        s = ((sx - rx) * rdy - (sy - ry) * rdx) / denom

        if t > 0 and 0 <= s <= 1:
            return t

        return None

    def _ray_map_intersection(
        self,
        ray_start: Tuple[float, float],
        ray_angle: float,
        map_grid: np.ndarray,
        map_resolution: float,
        map_origin: Tuple[float, float],
    ) -> Optional[float]:
        """射线与地图栅格相交（Bresenham算法）

        Args:
            ray_start: 射线起点 (x, y)
            ray_angle: 射线方向（弧度）
            map_grid: 地图栅格，0=自由，255=障碍
            map_resolution: 地图分辨率
            map_origin: 地图原点

        Returns:
            相交距离，None 表示不相交
        """
        # 起点栅格坐标
        gx0 = int((ray_start[0] - map_origin[0]) / map_resolution)
        gy0 = int((ray_start[1] - map_origin[1]) / map_resolution)

        # 终点（最大距离处）
        end_x = ray_start[0] + self.config.max_range * math.cos(ray_angle)
        end_y = ray_start[1] + self.config.max_range * math.sin(ray_angle)
        gx1 = int((end_x - map_origin[0]) / map_resolution)
        gy1 = int((end_y - map_origin[1]) / map_resolution)

        # Bresenham 射线投射
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy

        gx, gy = gx0, gy0

        # 最大步数限制
        max_steps = int(self.config.max_range / map_resolution) + 1
        steps = 0

        while steps < max_steps:
            # 检查是否在地图范围内
            if 0 <= gx < map_grid.shape[1] and 0 <= gy < map_grid.shape[0]:
                # 检查是否为障碍物
                if map_grid[gy, gx] > 127:  # 障碍物阈值
                    # 计算距离
                    world_x = map_origin[0] + (gx + 0.5) * map_resolution
                    world_y = map_origin[1] + (gy + 0.5) * map_resolution
                    dist = math.sqrt(
                        (world_x - ray_start[0]) ** 2 + (world_y - ray_start[1]) ** 2
                    )
                    return dist

            # 移动到下一个栅格
            if gx == gx1 and gy == gy1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                gx += sx
            if e2 < dx:
                err += dx
                gy += sy

            steps += 1

        return None

    # --------------------------------------------------------------------------
    # 工具方法
    # --------------------------------------------------------------------------

    def get_angles(self) -> np.ndarray:
        """获取所有射线角度（机器人坐标系）"""
        return self._angles.copy()

    def ranges_to_points(
        self,
        ranges: np.ndarray,
        robot_pose: Tuple[float, float, float],
    ) -> np.ndarray:
        """将距离数组转换为点云（世界坐标系）

        Args:
            ranges: 距离数组（米）
            robot_pose: 机器人位姿 (x, y, theta)

        Returns:
            点云数组 (Nx2)
        """
        rx, ry, rtheta = robot_pose

        points = []
        for i, distance in enumerate(ranges):
            if distance < self.config.max_range:
                # 机器人坐标系下的点
                local_angle = self._angles[i]
                local_x = distance * math.cos(local_angle)
                local_y = distance * math.sin(local_angle)

                # 转换到世界坐标系
                world_x = rx + local_x * math.cos(rtheta) - local_y * math.sin(rtheta)
                world_y = ry + local_x * math.sin(rtheta) + local_y * math.cos(rtheta)

                points.append([world_x, world_y])

        return np.array(points) if points else np.array([]).reshape(0, 2)
