# -*- coding: utf-8 -*-
"""导航协调器 - 协调全局规划、局部避障和运动控制

核心职责：
1. 管理导航目标队列
2. 协调全局规划和局部避障
3. 监控导航执行状态
4. 处理异常和重规划

使用方式：
    coordinator = NavigationCoordinator()
    coordinator.set_pose_provider(odom_service.get_pose)
    coordinator.set_obstacle_provider(obstacle_detector.get_obstacles)
    coordinator.set_velocity_sender(motion_service.send_velocity)
    coordinator.set_map_provider(slam_service.get_map)

    coordinator.start()

    # 同步导航
    result = coordinator.navigate_to(x=2.0, y=3.0, yaw=0.0)

    # 异步导航
    goal_id = coordinator.navigate_to_async(x=2.0, y=3.0, yaw=0.0)
    feedback = coordinator.get_feedback(goal_id)
"""

from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple
from queue import PriorityQueue

import numpy as np

from common.logging import get_logger
from navigation.core.occupancy_grid import OccupancyGrid, COST_LETHAL
from configs import get_config

logger = get_logger(__name__)


def _nav_cfg():
    """获取导航配置快捷方式"""
    return get_config().navigation


# ------------------------------------------------------------------------------
# 数据结构定义
# ------------------------------------------------------------------------------


class GoalStatus(Enum):
    """目标状态"""

    PENDING = "pending"  # 等待执行
    ACTIVE = "active"  # 正在执行
    COMPLETED = "completed"  # 已完成
    FAILED = "failed"  # 失败
    CANCELLED = "cancelled"  # 已取消


class NavigationState(Enum):
    """导航状态"""

    IDLE = "idle"  # 空闲
    PLANNING = "planning"  # 规划中
    NAVIGATING = "navigating"  # 导航中
    OBSTACLE_AVOIDING = "avoiding"  # 避障中
    RECOVERY = "recovery"  # 恢复中（重规划）
    STOPPED = "stopped"  # 已停止（安全模式）


@dataclass
class NavigationGoal:
    """导航目标"""

    goal_id: str
    target_pose: Tuple[float, float, Optional[float]]  # (x, y, yaw); yaw=None 表示不约束朝向
    priority: int = 0  # 优先级（0=普通，1=高，2=紧急）
    timestamp: float = field(default_factory=time.time)
    timeout: float = 300.0  # 超时时间（秒）
    status: GoalStatus = GoalStatus.PENDING
    error_msg: Optional[str] = None

    def __lt__(self, other):
        """优先队列比较（优先级高的排前面，时间早的排前面）"""
        if self.priority != other.priority:
            return self.priority > other.priority
        return self.timestamp < other.timestamp


@dataclass
class NavigationFeedback:
    """导航反馈"""

    goal_id: str
    state: NavigationState
    current_pose: Tuple[float, float, float]
    distance_to_goal: float
    time_elapsed: float
    progress: float  # 0.0 ~ 1.0
    error_msg: Optional[str] = None


# ------------------------------------------------------------------------------
# 导航协调器
# ------------------------------------------------------------------------------


class NavigationCoordinator:
    """导航协调器

    核心功能：
    - 目标队列管理
    - 全局规划触发
    - 局部避障执行
    - 异常恢复和重规划
    """

    def __init__(self, config: Optional[dict] = None):
        """初始化导航协调器"""
        config = config or {}

        # 配置参数
        self.replan_distance_threshold = config.get("replan_distance_threshold", 0.5)
        self.goal_reached_distance = config.get("goal_reached_distance", 0.1)
        self.goal_reached_angle = config.get("goal_reached_angle", 0.1)
        self.max_replan_attempts = config.get("max_replan_attempts", 3)
        self.control_frequency = config.get("control_frequency", 10.0)
        self.obstacle_emergency_distance = config.get(
            "obstacle_emergency_distance", 0.3
        )
        self.inflation_radius = config.get("inflation_radius", 0.25)
        self.robot_radius = config.get("robot_radius", 0.2)
        self.lookahead_distance = config.get("lookahead_distance", 0.4)
        self._max_angular_accel = config.get("max_angular_accel_rad", 2.0)
        self._velocity_filter_alpha = config.get("velocity_filter_alpha", 0.2)

        # 缓存导航配置（避免控制循环中反复读取）
        nav = _nav_cfg()
        self._max_linear_speed = nav.max_linear_speed
        self._max_angular_speed = nav.max_angular_speed
        logger.info(f"max_linear_speed: {self._max_linear_speed:.2f} m/s, max_angular_speed: {self._max_angular_speed:.2f} rad/s, max_angular_accel: {self._max_angular_accel:.2f} rad/s², velocity_filter_alpha: {self._velocity_filter_alpha:.2f}")

        # 速度低通滤波器状态（一阶 IIR）
        self._filtered_linear_vel = 0.0
        self._filtered_angular_vel = 0.0

        # 角速度变化率限制（用于平滑转向突变）
        self._last_angular_vel = 0.0

        # 状态
        self.state = NavigationState.IDLE
        self.current_goal: Optional[NavigationGoal] = None
        self.global_path: Optional[List[Tuple[float, float]]] = None
        self.current_path_index = 0
        self._current_local_goal: Optional[Tuple[float, float]] = None

        # 目标队列
        self.goal_queue: PriorityQueue[NavigationGoal] = PriorityQueue()
        self.goals: Dict[str, NavigationGoal] = {}
        self._goals_lock = threading.Lock()

        # 外部接口（需通过 setter 注入）
        self._pose_provider: Optional[Callable] = None
        self._obstacle_provider: Optional[Callable] = None
        self._velocity_sender: Optional[Callable] = None
        self._map_provider: Optional[Callable] = None


        # 运行控制
        self._running = False
        self._worker_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # 统计信息
        self._start_time = 0.0
        self._replan_count = 0

        logger.info("导航协调器初始化完成")

    # --------------------------------------------------------------------------
    # 外部接口注入
    # --------------------------------------------------------------------------

    def set_pose_provider(self, provider: Callable):
        """设置位姿提供者"""
        self._pose_provider = provider

    def set_obstacle_provider(self, provider: Callable):
        """设置障碍物提供者"""
        self._obstacle_provider = provider

    def set_velocity_sender(self, sender: Callable):
        """设置速度发送器"""
        self._velocity_sender = sender

    def set_map_provider(self, provider: Callable):
        """设置地图提供者"""
        self._map_provider = provider

    # --------------------------------------------------------------------------
    # 目标管理
    # --------------------------------------------------------------------------

    def navigate_to(
        self,
        x: float,
        y: float,
        yaw: Optional[float] = None,
        priority: int = 0,
        timeout: float = 300.0,
        blocking: bool = True,
    ) -> NavigationFeedback:
        """导航到指定目标（同步或异步）

        Args:
            x: 目标 x 坐标（米）
            y: 目标 y 坐标（米）
            yaw: 目标朝向（弧度），None 表示不要求朝向
            priority: 优先级（0=普通，1=高，2=紧急）
            timeout: 超时时间（秒）
            blocking: 是否阻塞等待完成

        Returns:
            导航反馈（同步模式）或目标ID字符串（异步模式）
        """
        goal_id = str(uuid.uuid4())[:8]
        target_pose = (x, y, yaw)

        goal = NavigationGoal(
            goal_id=goal_id,
            target_pose=target_pose,
            priority=priority,
            timeout=timeout,
        )

        with self._goals_lock:
            self.goals[goal_id] = goal
        self.goal_queue.put(goal)

        yaw_str = f"{yaw:.2f}" if yaw is not None else "N/A"
        logger.info(f"添加导航目标: {goal_id} -> ({x:.2f}, {y:.2f}, yaw={yaw_str})")

        if not blocking:
            return goal_id

        # 阻塞等待完成
        while True:
            time.sleep(0.1)
            feedback = self.get_feedback(goal_id)

            if feedback.state == NavigationState.IDLE:
                return feedback

            if feedback.state == NavigationState.STOPPED:
                return feedback

    def navigate_to_async(
        self,
        x: float,
        y: float,
        yaw: Optional[float] = None,
        priority: int = 0,
        timeout: float = 300.0,
    ) -> str:
        """异步导航到指定目标

        Returns:
            目标ID，用于查询状态
        """
        return self.navigate_to(x, y, yaw, priority, timeout, blocking=False)

    def cancel_goal(self, goal_id: str) -> bool:
        """取消指定目标

        Args:
            goal_id: 目标ID

        Returns:
            是否成功取消
        """
        with self._goals_lock:
            if goal_id not in self.goals:
                return False

            goal = self.goals[goal_id]
            goal.status = GoalStatus.CANCELLED

            if self.current_goal and self.current_goal.goal_id == goal_id:
                self._stop_navigation()
                self.state = NavigationState.IDLE

            logger.info(f"取消导航目标: {goal_id}")
            return True

    def cancel_all_goals(self):
        """取消所有目标"""
        for goal_id in list(self.goals.keys()):
            self.cancel_goal(goal_id)

    def get_feedback(self, goal_id: str) -> Optional[NavigationFeedback]:
        """获取导航反馈

        Args:
            goal_id: 目标ID

        Returns:
            导航反馈，目标不存在时返回 None
        """
        with self._goals_lock:
            if goal_id not in self.goals:
                return None

            goal = self.goals[goal_id]

        current_pose = self._get_current_pose()

        if current_pose is None:
            return NavigationFeedback(
                goal_id=goal_id,
                state=NavigationState.STOPPED,
                current_pose=(0, 0, 0),
                distance_to_goal=0,
                time_elapsed=0,
                progress=0,
                error_msg="无法获取当前位姿",
            )

        dx = goal.target_pose[0] - current_pose[0]
        dy = goal.target_pose[1] - current_pose[1]
        distance = math.sqrt(dx * dx + dy * dy)

        # 计算进度（按路径长度估算）
        progress = 0.0
        if self.global_path:
            total_distance = self._calculate_path_length(self.global_path)
            if total_distance > 0:
                progress = max(0, min(1, 1 - distance / total_distance))

        time_elapsed = (
            time.time() - goal.timestamp if goal.status == GoalStatus.ACTIVE else 0
        )

        return NavigationFeedback(
            goal_id=goal_id,
            state=self.state,
            current_pose=current_pose,
            distance_to_goal=distance,
            time_elapsed=time_elapsed,
            progress=progress,
            error_msg=goal.error_msg,
        )

    # --------------------------------------------------------------------------
    # 核心控制循环
    # --------------------------------------------------------------------------

    def start(self):
        """启动导航协调器"""
        if self._running:
            logger.warning("导航协调器已在运行")
            return

        self._running = True
        self._stop_event.clear()
        self._worker_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._worker_thread.start()

        logger.info("导航协调器已启动")

    def stop(self):
        """停止导航协调器"""
        self._running = False
        self._stop_event.set()

        if self._worker_thread:
            self._worker_thread.join(timeout=2.0)

        self._stop_navigation()
        logger.info("导航协调器已停止")

    def _control_loop(self):
        """主控制循环"""
        dt = 1.0 / self.control_frequency

        while self._running and not self._stop_event.is_set():
            try:
                # 状态机处理
                if self.state == NavigationState.IDLE:
                    self._process_idle_state()

                elif self.state == NavigationState.PLANNING:
                    self._process_planning_state()

                elif self.state == NavigationState.NAVIGATING:
                    self._process_navigating_state()

                elif self.state == NavigationState.OBSTACLE_AVOIDING:
                    self._process_obstacle_avoiding_state()

                elif self.state == NavigationState.RECOVERY:
                    self._process_recovery_state()

                # 等待下一次循环
                self._stop_event.wait(dt)

            except Exception as e:
                logger.error(f"控制循环异常: {e}", exc_info=True)
                self.state = NavigationState.STOPPED

    def _process_idle_state(self):
        """处理空闲状态：从队列中取出下一个目标"""
        if self.goal_queue.empty():
            return

        try:
            goal = self.goal_queue.get_nowait()
        except:
            return

        with self._goals_lock:
            if goal.goal_id not in self.goals:
                return

            if goal.status == GoalStatus.CANCELLED:
                return

            self.current_goal = goal
            goal.status = GoalStatus.ACTIVE

        self.state = NavigationState.PLANNING
        self._start_time = time.time()
        self._replan_count = 0

        logger.info(f"开始执行目标: {goal.goal_id}")

    def _process_planning_state(self):
        """处理规划状态：执行全局规划"""
        if not self.current_goal:
            self.state = NavigationState.IDLE
            return

        # 获取当前位姿
        current_pose = self._get_current_pose()
        if current_pose is None:
            self._fail_current_goal("无法获取当前位姿")
            return

        # 获取全局地图
        global_map = self._get_global_map()
        if global_map is None:
            self._fail_current_goal("无法获取全局地图")
            return
        # 复制地图，避免原地膨胀污染原始地图
        global_map = global_map.copy()
        # 膨胀障碍物（机器人半径 + 安全边距）
        global_map.inflate_obstacles(self.inflation_radius + self.robot_radius)

        # 每次规划都重新创建规划器，确保使用最新地图，避免"幽灵障碍物"
        try:
            from navigation.core.astar_planner import AStarPlanner

            planner = AStarPlanner(global_map, allow_diagonal=True)
        except ImportError as e:
            self._fail_current_goal(f"无法加载规划器: {e}")
            return

        # 执行全局规划
        start = (current_pose[0], current_pose[1])
        goal = (self.current_goal.target_pose[0], self.current_goal.target_pose[1])

        logger.debug(f"全局规划: {start} -> {goal}")

        try:
            path = planner.plan(start, goal)
        except Exception as e:
            logger.error(f"规划异常: {e}", exc_info=True)
            path = None

        if not path:
            self._fail_current_goal("全局规划失败，无法找到路径")
            return

        # 视线法简化：检查两点之间是否可直线通行，拉直折线
        if len(path) > 2:
            path = planner._simplify_path(path)
            logger.info(f"视线法简化后路径点数量: {len(path)}")

        # 路径处理：视线法简化后不再做额外平滑
        # 保留原始折线路径，避免插值导致穿障碍物或过度切弯

        self.global_path = path
        self.current_path_index = 0
        self.state = NavigationState.NAVIGATING

        logger.info(f"全局规划成功，路径长度: {len(path)} 个点")

    def _process_navigating_state(self):
        """处理导航状态：执行局部避障和运动控制"""
        current_pose = self._get_current_pose()
        if current_pose is None:
            self._stop_navigation()
            self.state = NavigationState.IDLE
            return

        # 检查是否到达目标
        if self._is_goal_reached(current_pose):
            self._complete_current_goal()
            return

        # 检查是否超时
        if time.time() - self._start_time > self.current_goal.timeout:
            self._fail_current_goal("导航超时")
            return

        # 获取障碍物
        obstacles = self._get_obstacles()

        # 检查紧急障碍物
        if self._has_emergency_obstacle(obstacles):
            logger.warning("检测到紧急障碍物，进入避障状态")
            self.state = NavigationState.OBSTACLE_AVOIDING
            return

        # 最终朝向调整：位置已到达但角度未满足时，原地旋转到目标朝向
        target_yaw = self.current_goal.target_pose[2]
        if target_yaw is not None:
            dx = self.current_goal.target_pose[0] - current_pose[0]
            dy = self.current_goal.target_pose[1] - current_pose[1]
            distance = math.sqrt(dx * dx + dy * dy)
            if distance <= self.goal_reached_distance:
                yaw_error = self._normalize_angle(target_yaw - current_pose[2])

                # 已在角度阈值内，停止旋转，避免过冲/抖动
                if abs(yaw_error) <= self.goal_reached_angle:
                    self._send_velocity(0.0, 0.0)
                    self._last_angular_vel = 0.0
                    return

                # 低增益 P 控制 + 角速度变化率限制，防止速度跳变导致震荡
                angular_vel = 1.0 * yaw_error
                angular_vel = float(np.clip(angular_vel, -self._max_angular_speed, self._max_angular_speed))
                dt = 1.0 / self.control_frequency
                max_delta = self._max_angular_accel * dt
                angular_vel = float(np.clip(angular_vel, self._last_angular_vel - max_delta, self._last_angular_vel + max_delta))
                self._last_angular_vel = angular_vel
                self._send_velocity(0.0, angular_vel)
                return

        # 计算当前到最终目标的直线距离（用于终点减速）
        dx_goal = self.current_goal.target_pose[0] - current_pose[0]
        dy_goal = self.current_goal.target_pose[1] - current_pose[1]
        distance_to_goal = math.sqrt(dx_goal * dx_goal + dy_goal * dy_goal)

        # 获取局部目标点并计算速度指令
        local_goal = self._get_local_goal(current_pose)
        self._current_local_goal = local_goal
        linear_vel, angular_vel = self._compute_velocity(
            current_pose=current_pose,
            local_goal=local_goal,
            obstacles=obstacles,
            distance_to_goal=distance_to_goal,
        )

        # 一阶低通滤波器：平滑输出速度
        a = self._velocity_filter_alpha
        self._filtered_linear_vel = a * linear_vel + (1.0 - a) * self._filtered_linear_vel
        self._filtered_angular_vel = a * angular_vel + (1.0 - a) * self._filtered_angular_vel

        # 发送速度指令
        if not self._send_velocity(self._filtered_linear_vel, self._filtered_angular_vel):
            logger.warning("速度指令发送失败")

        # 检查是否需要重规划
        if self._needs_replanning(current_pose):
            logger.info("偏离路径，触发重规划")
            self.state = NavigationState.RECOVERY

    def _process_obstacle_avoiding_state(self):
        """处理避障状态：紧急避障"""
        # 获取障碍物
        obstacles = self._get_obstacles()

        # 检查是否有紧急障碍
        has_emergency = self._has_emergency_obstacle(obstacles)

        if has_emergency:
            # 紧急停止
            self._send_velocity(0.0, 0.0)
            logger.warning("检测到紧急障碍物，停止运动")
        else:
            # 恢复导航
            logger.info("紧急障碍已清除，恢复导航")
            self.state = NavigationState.NAVIGATING

    def _process_recovery_state(self):
        """处理恢复状态：重新规划"""
        self._replan_count += 1

        if self._replan_count > self.max_replan_attempts:
            self._fail_current_goal(
                f"重规划失败，已达最大尝试次数 {self.max_replan_attempts}"
            )
            return

        logger.info(f"执行第 {self._replan_count} 次重规划")

        # 重新规划
        self.state = NavigationState.PLANNING

    # --------------------------------------------------------------------------
    # 辅助方法
    # --------------------------------------------------------------------------

    def _get_current_pose(self) -> Optional[Tuple[float, float, float]]:
        """获取当前位姿"""
        if self._pose_provider:
            try:
                pose = self._pose_provider()
                if pose and len(pose) == 3:
                    return tuple(pose)
            except Exception as e:
                logger.error(f"获取位姿失败: {e}")
        return None

    def _get_obstacles(self) -> List:
        """获取障碍物列表"""
        if self._obstacle_provider:
            try:
                return self._obstacle_provider() or []
            except Exception as e:
                logger.error(f"获取障碍物失败: {e}")
        return []

    def _get_global_map(self) -> Optional[OccupancyGrid]:
        """获取全局地图"""
        if self._map_provider:
            try:
                return self._map_provider()
            except Exception as e:
                logger.error(f"获取地图失败: {e}")
        return None

    def _send_velocity(self, linear: float, angular: float) -> bool:
        """发送速度指令"""
        if self._velocity_sender:
            try:
                return self._velocity_sender(linear, angular)
            except Exception as e:
                logger.error(f"发送速度失败: {e}")
        return False

    def _is_goal_reached(self, current_pose: Tuple[float, float, float]) -> bool:
        """检查是否到达目标"""
        if not self.current_goal:
            return False

        dx = self.current_goal.target_pose[0] - current_pose[0]
        dy = self.current_goal.target_pose[1] - current_pose[1]
        distance = math.sqrt(dx * dx + dy * dy)

        # 检查距离
        if distance > self.goal_reached_distance:
            return False

        # 检查角度（如果有要求）
        if self.current_goal.target_pose[2] is not None:
            yaw_error = abs(
                self._normalize_angle(
                    self.current_goal.target_pose[2] - current_pose[2]
                )
            )
            if yaw_error > self.goal_reached_angle:
                return False

        return True

    @staticmethod
    def _extract_obstacle_distance(obs: Any) -> Optional[float]:
        """从各种障碍物格式中提取距离值。"""
        if hasattr(obs, "z"):
            return float(obs.z)
        if isinstance(obs, (tuple, list)) and len(obs) >= 3:
            return float(obs[2])
        if isinstance(obs, dict) and "distance" in obs:
            return float(obs["distance"])
        return None

    def _has_emergency_obstacle(self, obstacles: List) -> bool:
        """检查是否有紧急障碍物"""
        for obs in obstacles:
            dist = self._extract_obstacle_distance(obs)
            if dist is not None and dist < self.obstacle_emergency_distance:
                return True
        return False

    @staticmethod
    def _point_to_segment_distance(
        px: float, py: float, x1: float, y1: float, x2: float, y2: float
    ) -> float:
        """计算点 (px, py) 到线段 (x1,y1)-(x2,y2) 的最短距离。"""
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-12:
            return math.hypot(px - x1, py - y1)
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    def _needs_replanning(self, current_pose: Tuple[float, float, float]) -> bool:
        """检查是否需要重规划

        计算当前位姿到全局路径各线段的垂直距离，取最小值与阈值比较。
        """
        if not self.global_path or self.current_path_index >= len(self.global_path):
            return False

        min_distance = float("inf")

        # 只检查当前位置附近的路径线段
        start_idx = max(0, self.current_path_index - 5)
        end_idx = min(len(self.global_path), self.current_path_index + 20)

        for i in range(start_idx, end_idx - 1):
            x1, y1 = self.global_path[i]
            x2, y2 = self.global_path[i + 1]
            distance = self._point_to_segment_distance(
                current_pose[0], current_pose[1], x1, y1, x2, y2
            )
            min_distance = min(min_distance, distance)

        return min_distance > self.replan_distance_threshold

    def _get_local_goal(
        self, current_pose: Tuple[float, float, float]
    ) -> Tuple[float, float]:
        """从全局路径中提取局部目标点

        策略：向前看 1-2 米，选择最近的路径点
        """
        if not self.global_path:
            return current_pose[:2]

        # 从当前路径索引开始查找
        for i in range(self.current_path_index, len(self.global_path)):
            point = self.global_path[i]
            dx = point[0] - current_pose[0]
            dy = point[1] - current_pose[1]
            distance = math.sqrt(dx * dx + dy * dy)

            if distance >= self.lookahead_distance:
                self.current_path_index = i
                return point

        # 已到达路径末尾，返回最终目标
        return self.global_path[-1]

    def _compute_velocity(
        self,
        current_pose: Tuple[float, float, float],
        local_goal: Tuple[float, float],
        obstacles: List,
        distance_to_goal: float = float("inf"),
    ) -> Tuple[float, float]:
        """计算速度指令

        纯追踪控制器 + 终点距离减速：
        - 计算到局部目标的方向
        - 调整角速度朝向目标
        - 根据障碍物距离和终点距离调整线速度

        Returns:
            (linear_vel, angular_vel)
        """
        # 计算到局部目标的方向
        dx = local_goal[0] - current_pose[0]
        dy = local_goal[1] - current_pose[1]
        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)

        # 计算角度误差
        angle_error = self._normalize_angle(target_angle - current_pose[2])

        # 角速度 P 控制器
        angular_vel = 1.5 * angle_error
        angular_vel = float(np.clip(angular_vel, -self._max_angular_speed, self._max_angular_speed))

        # 角速度变化率限制（slew rate limit）
        dt = 1.0 / self.control_frequency
        max_delta = self._max_angular_accel * dt
        angular_vel = float(np.clip(angular_vel, self._last_angular_vel - max_delta, self._last_angular_vel + max_delta))
        self._last_angular_vel = angular_vel

        # 线速度：角度误差大时原地旋转，否则根据误差、障碍物和终点距离降速
        if abs(angle_error) > math.pi / 3:
            linear_vel = 0.0
        else:
            angle_factor = 1.0 - abs(angle_error) / (math.pi / 3)

            # 障碍物因子
            obstacle_factor = 1.0
            min_obstacle_dist = self._get_min_obstacle_distance(obstacles)
            if min_obstacle_dist < 1.0:
                obstacle_factor = min_obstacle_dist / 1.0

            # 终点距离减速因子：进入减速区后按距离比例降速
            distance_factor = 1.0
            decel_zone = self.goal_reached_distance * 5
            if distance_to_goal < decel_zone:
                distance_factor = max(0.15, distance_to_goal / decel_zone)

            linear_vel = self._max_linear_speed * angle_factor * obstacle_factor * distance_factor
            linear_vel = max(0.0, linear_vel)

        return linear_vel, angular_vel

    def _get_min_obstacle_distance(self, obstacles: List) -> float:
        """获取最近障碍物距离"""
        min_dist = float("inf")
        for obs in obstacles:
            dist = self._extract_obstacle_distance(obs)
            if dist is not None:
                min_dist = min(min_dist, dist)
        return min_dist if min_dist != float("inf") else 10.0

    def _smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """平滑路径（预留接口，当前直接返回原始路径）。"""
        return path

    def _calculate_path_length(self, path: List[Tuple[float, float]]) -> float:
        """计算路径总长度"""
        if not path or len(path) < 2:
            return 0.0

        length = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            length += math.sqrt(dx * dx + dy * dy)

        return length

    def _normalize_angle(self, angle: float) -> float:
        """将角度规范化到 [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _stop_navigation(self):
        """停止导航"""
        self._send_velocity(0.0, 0.0)
        self.global_path = None
        self.current_path_index = 0
        self._current_local_goal = None
        self._last_angular_vel = 0.0
        self._filtered_linear_vel = 0.0
        self._filtered_angular_vel = 0.0

    def _complete_current_goal(self):
        """完成当前目标"""
        if self.current_goal:
            with self._goals_lock:
                self.current_goal.status = GoalStatus.COMPLETED
            logger.info(f"目标完成: {self.current_goal.goal_id}")

        self._stop_navigation()
        self.state = NavigationState.IDLE
        self.current_goal = None

    def _fail_current_goal(self, error_msg: str):
        """标记当前目标失败"""
        if self.current_goal:
            with self._goals_lock:
                self.current_goal.status = GoalStatus.FAILED
                self.current_goal.error_msg = error_msg
            logger.error(f"目标失败: {self.current_goal.goal_id} - {error_msg}")

        self._stop_navigation()
        self.state = NavigationState.IDLE
        self.current_goal = None

    # --------------------------------------------------------------------------
    # 工具方法
    # --------------------------------------------------------------------------

    def get_current_state(self) -> NavigationState:
        """获取当前导航状态"""
        return self.state

    def get_local_goal(self) -> Optional[Tuple[float, float]]:
        """获取当前局部目标点（供可视化使用）"""
        return self._current_local_goal

    def is_running(self) -> bool:
        """检查协调器是否在运行"""
        return self._running

    def get_active_goals(self) -> List[str]:
        """获取所有活跃目标的ID列表"""
        with self._goals_lock:
            return [
                goal_id
                for goal_id, goal in self.goals.items()
                if goal.status in (GoalStatus.PENDING, GoalStatus.ACTIVE)
            ]

    def clear_completed_goals(self):
        """清除已完成和失败的目标"""
        with self._goals_lock:
            to_remove = [
                goal_id
                for goal_id, goal in self.goals.items()
                if goal.status
                in (GoalStatus.COMPLETED, GoalStatus.FAILED, GoalStatus.CANCELLED)
            ]
            for goal_id in to_remove:
                del self.goals[goal_id]

        logger.debug(f"清除了 {len(to_remove)} 个已完成/失败的目标")
