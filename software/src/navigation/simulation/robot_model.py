# -*- coding: utf-8 -*-
"""差速驱动机器人模型

模拟两轮差速驱动底盘的运动学：
- 输入：线速度 v (m/s)，角速度 w (rad/s)
- 输出：位姿 (x, y, theta)
- 更新公式：
    x' = x + v * cos(theta) * dt
    y' = y + v * sin(theta) * dt
    theta' = theta + w * dt
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from common.logging import get_logger

logger = get_logger(__name__)


@dataclass
class RobotConfig:
    """机器人配置"""

    radius: float = 0.15  # 机器人半径（米）
    max_linear_vel: float = 0.5  # 最大线速度（m/s）
    max_angular_vel: float = 1.0  # 最大角速度（rad/s）
    wheel_base: float = 0.2  # 轮距（米）

    # 运动噪声参数（模拟真实传感器误差）
    linear_noise_std: float = 0.01  # 线速度噪声标准差
    angular_noise_std: float = 0.02  # 角速度噪声标准差
    slip_probability: float = 0.01  # 打滑概率


class DifferentialRobot:
    """差速驱动机器人

    特性：
    - 运动学模型（无滑动假设）
    - 速度限制
    - 运动噪声模拟
    - 碰撞检测接口
    """

    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        config: RobotConfig = None,
    ):
        """
        Args:
            x: 初始 x 坐标（米）
            y: 初始 y 坐标（米）
            theta: 初始朝向（弧度）
            config: 机器人配置
        """
        self.config = config or RobotConfig()

        # 真实位姿（仿真器内部使用）
        self._x = x
        self._y = y
        self._theta = theta

        # 里程计位姿（带噪声，对外提供）
        self._odom_x = x
        self._odom_y = y
        self._odom_theta = theta

        # 当前速度
        self._linear_vel = 0.0
        self._angular_vel = 0.0

        # 统计信息
        self._total_distance = 0.0
        self._total_rotation = 0.0

    # --------------------------------------------------------------------------
    # 位姿访问
    # --------------------------------------------------------------------------

    @property
    def pose(self) -> Tuple[float, float, float]:
        """获取真实位姿（仿真器内部使用）"""
        return (self._x, self._y, self._theta)

    @property
    def odom_pose(self) -> Tuple[float, float, float]:
        """获取里程计位姿（带噪声，对外提供）"""
        return (self._odom_x, self._odom_y, self._odom_theta)

    @property
    def velocity(self) -> Tuple[float, float]:
        """获取当前速度（线速度, 角速度）"""
        return (self._linear_vel, self._angular_vel)

    # --------------------------------------------------------------------------
    # 控制接口
    # --------------------------------------------------------------------------

    def set_velocity(self, linear: float, angular: float):
        """设置目标速度

        Args:
            linear: 线速度（m/s）
            angular: 角速度（rad/s）
        """
        # 速度限制
        self._linear_vel = np.clip(
            linear, -self.config.max_linear_vel, self.config.max_linear_vel
        )
        self._angular_vel = np.clip(
            angular, -self.config.max_angular_vel, self.config.max_angular_vel
        )

    def stop(self):
        """停止运动"""
        self._linear_vel = 0.0
        self._angular_vel = 0.0

    # --------------------------------------------------------------------------
    # 运动更新
    # --------------------------------------------------------------------------

    def update(self, dt: float):
        """更新机器人位姿

        Args:
            dt: 时间步长（秒）
        """
        if abs(self._linear_vel) < 1e-6 and abs(self._angular_vel) < 1e-6:
            return

        # 添加运动噪声
        linear_noisy = self._linear_vel
        angular_noisy = self._angular_vel

        if self.config.linear_noise_std > 0:
            linear_noisy += np.random.normal(0, self.config.linear_noise_std)

        if self.config.angular_noise_std > 0:
            angular_noisy += np.random.normal(0, self.config.angular_noise_std)

        # 模拟打滑
        if np.random.random() < self.config.slip_probability:
            linear_noisy *= 0.5  # 打滑时速度减半

        # 更新真实位姿（无噪声）
        self._update_pose(self._linear_vel, self._angular_vel, dt, is_real=True)

        # 更新里程计位姿（带噪声）
        self._update_pose(linear_noisy, angular_noisy, dt, is_real=False)

        # 更新统计
        self._total_distance += abs(self._linear_vel) * dt
        self._total_rotation += abs(self._angular_vel) * dt

    def _update_pose(self, linear: float, angular: float, dt: float, is_real: bool):
        """更新位姿（内部方法）"""
        if is_real:
            x, y, theta = self._x, self._y, self._theta
        else:
            x, y, theta = self._odom_x, self._odom_y, self._odom_theta

        # 差速驱动运动学
        if abs(angular) < 1e-6:
            # 直线运动
            new_x = x + linear * math.cos(theta) * dt
            new_y = y + linear * math.sin(theta) * dt
            new_theta = theta
        else:
            # 弧线运动
            R = linear / angular  # 转弯半径
            new_x = x + R * (math.sin(theta + angular * dt) - math.sin(theta))
            new_y = y + R * (math.cos(theta) - math.cos(theta + angular * dt))
            new_theta = theta + angular * dt

        # 规范化角度到 [-pi, pi]
        new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))

        if is_real:
            self._x, self._y, self._theta = new_x, new_y, new_theta
        else:
            self._odom_x, self._odom_y, self._odom_theta = new_x, new_y, new_theta

    # --------------------------------------------------------------------------
    # 碰撞检测
    # --------------------------------------------------------------------------

    def get_footprint(self) -> np.ndarray:
        """获取机器人轮廓点（用于碰撞检测）

        Returns:
            Nx2 数组，表示机器人轮廓点（世界坐标系）
        """
        # 简化为圆形
        num_points = 16
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

        points = np.column_stack(
            [
                self._x + self.config.radius * np.cos(angles),
                self._y + self.config.radius * np.sin(angles),
            ]
        )

        return points

    def check_collision(self, obstacles: list, check_points: int = 8) -> bool:
        """检查是否与障碍物碰撞

        Args:
            obstacles: 障碍物列表，每个障碍物为 (x, y, radius) 或多边形顶点数组
            check_points: 检查点数量

        Returns:
            是否发生碰撞
        """
        for obs in obstacles:
            if len(obs) == 3:
                # 圆形障碍物
                ox, oy, radius = obs
                dist = math.sqrt((self._x - ox) ** 2 + (self._y - oy) ** 2)
                if dist < self.config.radius + radius:
                    return True
            else:
                # 多边形障碍物
                obs_points = np.array(obs)
                # 简化检查：只检查机器人轮廓点是否在障碍物内
                robot_points = self.get_footprint()
                for point in robot_points[::check_points]:
                    if self._point_in_polygon(point, obs_points):
                        return True

        return False

    @staticmethod
    def _point_in_polygon(point: np.ndarray, polygon: np.ndarray) -> bool:
        """判断点是否在多边形内（射线法）"""
        x, y = point
        n = len(polygon)
        inside = False

        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]

            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside

            j = i

        return inside

    # --------------------------------------------------------------------------
    # 工具方法
    # --------------------------------------------------------------------------

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """重置机器人位姿"""
        self._x = x
        self._y = y
        self._theta = theta
        self._odom_x = x
        self._odom_y = y
        self._odom_theta = theta
        self._linear_vel = 0.0
        self._angular_vel = 0.0
        self._total_distance = 0.0
        self._total_rotation = 0.0

    def get_info(self) -> dict:
        """获取机器人信息"""
        return {
            "pose": self.pose,
            "odom_pose": self.odom_pose,
            "velocity": self.velocity,
            "total_distance": self._total_distance,
            "total_rotation": self._total_rotation,
        }
