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
from typing import Callable, Dict, List, Optional, Tuple
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
    target_pose: Tuple[float, float, float]  # (x, y, yaw)
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
        """初始化导航协调器

        Args:
            config: 配置字典，包含以下参数：
                - replan_distance_threshold: 偏离路径阈值（米），默认 0.5
                - goal_reached_distance: 到达目标距离阈值（米），默认 0.1
                - goal_reached_angle: 到达目标角度阈值（弧度），默认 0.1
                - max_replan_attempts: 最大重规划次数，默认 3
                - obstacle_check_frequency: 障碍物检测频率（Hz），默认 10.0
                - control_frequency: 控制循环频率（Hz），默认 10.0
                - obstacle_emergency_distance: 紧急停止距离（米），默认 0.3
                - inflation_radius: 障碍物膨胀半径（米），默认 0.25
                - robot_radius: 机器人半径（米），默认 0.2
        """
        config = config or {}

        # 配置参数
        self.replan_distance_threshold = config.get("replan_distance_threshold", 0.5)
        self.goal_reached_distance = config.get("goal_reached_distance", 0.1)
        self.goal_reached_angle = config.get("goal_reached_angle", 0.1)
        self.max_replan_attempts = config.get("max_replan_attempts", 3)
        self.obstacle_check_frequency = config.get("obstacle_check_frequency", 10.0)
        self.control_frequency = config.get("control_frequency", 10.0)
        self.obstacle_emergency_distance = config.get(
            "obstacle_emergency_distance", 0.3
        )
        self.inflation_radius = config.get("inflation_radius", 0.25)
        self.robot_radius = config.get("robot_radius", 0.2)

        # 状态
        self.state = NavigationState.IDLE
        self.current_goal: Optional[NavigationGoal] = None
        self.global_path: Optional[List[Tuple[float, float]]] = None
        self.current_path_index = 0

        # 目标队列
        self.goal_queue: PriorityQueue[NavigationGoal] = PriorityQueue()
        self.goals: Dict[str, NavigationGoal] = {}  # goal_id -> Goal
        self._goals_lock = threading.Lock()

        # 外部接口（需要通过 setter 注入）
        self._pose_provider: Optional[Callable[[], Tuple[float, float, float]]] = None
        self._obstacle_provider: Optional[Callable[[], List]] = None
        self._velocity_sender: Optional[Callable[[float, float], bool]] = None
        self._map_provider: Optional[Callable[[], OccupancyGrid]] = None

        # 规划器（延迟导入，避免循环依赖）
        self._global_planner = None
        self._local_planner = None
        self._costmap_generator = None

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

    def set_pose_provider(self, provider: Callable[[], Tuple[float, float, float]]):
        """设置位姿提供者

        Args:
            provider: 返回当前机器人位姿 的函数
        """
        self._pose_provider = provider
        logger.debug("位姿提供者已设置")

    def set_obstacle_provider(self, provider: Callable[[], List]):
        """设置障碍物提供者

        Args:
            provider: 返回障碍物列表的函数
        """
        self._obstacle_provider = provider
        logger.debug("障碍物提供者已设置")

    def set_velocity_sender(self, sender: Callable[[float, float], bool]):
        """设置速度发送器

        Args:
            sender: 发送线速度和角速度的函数 返回是否成功
        """
        self._velocity_sender = sender
        logger.debug("速度发送器已设置")

    def set_map_provider(self, provider: Callable[[], OccupancyGrid]):
        """设置地图提供者

        Args:
            provider: 返回全局地图 的函数
        """
        self._map_provider = provider
        logger.debug("地图提供者已设置")

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
        target_pose = (x, y, yaw if yaw is not None else 0.0)

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
        with self._goals_lock:
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

        # 计算进度
        progress = 0.0
        if self.global_path:
            total_distance = self._calculate_path_length(self.global_path)
            remaining_distance = distance
            if total_distance > 0:
                progress = max(0, min(1, 1 - remaining_distance / total_distance))

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

        # 初始化规划器（延迟加载）
        if self._global_planner is None:
            try:
                from navigation.core.astar_planner import AStarPlanner

                self._global_planner = AStarPlanner(global_map, allow_diagonal=True)
            except ImportError as e:
                self._fail_current_goal(f"无法加载规划器: {e}")
                return

        # 执行全局规划
        start = (current_pose[0], current_pose[1])
        goal = (self.current_goal.target_pose[0], self.current_goal.target_pose[1])

        logger.debug(f"全局规划: {start} -> {goal}")

        try:
            path = self._global_planner.plan(start, goal)
        except Exception as e:
            logger.error(f"规划异常: {e}", exc_info=True)
            path = None

        if not path:
            self._fail_current_goal("全局规划失败，无法找到路径")
            return

        # 平滑路径
        path = self._smooth_path(path)

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

        # 更新局部代价地图（如果规划器需要）
        if self._costmap_generator is None:
            try:
                from navigation.planning.costmap_generator import LocalCostmapGenerator

                self._costmap_generator = LocalCostmapGenerator()
            except ImportError:
                pass

        # 获取局部目标点
        local_goal = self._get_local_goal(current_pose)
        

        # 计算速度指令
        linear_vel, angular_vel = self._compute_velocity(
            current_pose=current_pose,
            local_goal=local_goal,
            obstacles=obstacles,
        )

        # 发送速度指令
        if not self._send_velocity(linear_vel, angular_vel):
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

    def _has_emergency_obstacle(self, obstacles: List) -> bool:
        """检查是否有紧急障碍物

        Args:
            obstacles: 障碍物列表，每个障碍物应有 z 属性（距离）

        Returns:
            是否有紧急障碍物
        """
        for obs in obstacles:
            # 支持不同的障碍物格式
            if hasattr(obs, "z"):
                # DepthObstacle 类型
                if obs.z < self.obstacle_emergency_distance:
                    return True
            elif isinstance(obs, (tuple, list)) and len(obs) >= 3:
                # (x, y, distance) 格式
                if obs[2] < self.obstacle_emergency_distance:
                    return True
            elif isinstance(obs, dict) and "distance" in obs:
                # 字典格式
                if obs["distance"] < self.obstacle_emergency_distance:
                    return True

        return False

    def _needs_replanning(self, current_pose: Tuple[float, float, float]) -> bool:
        """检查是否需要重规划"""
        if not self.global_path or self.current_path_index >= len(self.global_path):
            return False

        # 计算到全局路径的距离
        min_distance = float("inf")

        # 只检查当前位置附近的路径点
        start_idx = max(0, self.current_path_index - 5)
        end_idx = min(len(self.global_path), self.current_path_index + 20)

        for i in range(start_idx, end_idx):
            point = self.global_path[i]
            dx = point[0] - current_pose[0]
            dy = point[1] - current_pose[1]
            distance = math.sqrt(dx * dx + dy * dy)
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

        lookahead_distance = 0.1  # 向前看 0.5 米

        # 从当前路径索引开始查找
        for i in range(self.current_path_index, len(self.global_path)):
            point = self.global_path[i]
            dx = point[0] - current_pose[0]
            dy = point[1] - current_pose[1]
            distance = math.sqrt(dx * dx + dy * dy)

            if distance >= lookahead_distance:
                self.current_path_index = i
                return point

        # 已到达路径末尾，返回最终目标
        return self.global_path[-1]

    def _compute_velocity(
        self,
        current_pose: Tuple[float, float, float],
        local_goal: Tuple[float, float],
        obstacles: List,
    ) -> Tuple[float, float]:
        """计算速度指令

        使用简单的纯追踪控制器：
        - 计算到局部目标的方向
        - 调整角速度朝向目标
        - 根据障碍物距离调整线速度

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

        # 角速度：简单的 P 控制器
        nav = _nav_cfg()
        angular_vel = 1.5 * angle_error  # Kp = 1.5 (降低增益，使转向更平滑)
        angular_vel = np.clip(angular_vel, -nav.max_angular_speed, nav.max_angular_speed)  # 限制角速度

        # 线速度：根据角度误差和障碍物距离调整
        if abs(angle_error) > math.pi / 3:  # 60度（放宽阈值）
            # 角度误差太大，原地旋转
            linear_vel = 0.0
        else:
            # 根据角度误差和障碍物距离调整速度
            max_linear = nav.max_linear_speed  # 从全局配置读取最大线速度

            # 角度因子：角度误差越小，速度越快
            angle_factor = 1.0 - abs(angle_error) / (math.pi / 3)

            # 障碍物因子：障碍物越近，速度越慢
            obstacle_factor = 1.0
            min_obstacle_dist = self._get_min_obstacle_distance(obstacles)
            if min_obstacle_dist < 1.5:  # 提高到1.5m
                obstacle_factor = min_obstacle_dist / 1.5

            linear_vel = max_linear * angle_factor * obstacle_factor
            linear_vel = max(0.0, linear_vel)  # 不后退

        return linear_vel, angular_vel

    def _get_min_obstacle_distance(self, obstacles: List) -> float:
        """获取最近障碍物距离"""
        min_dist = float("inf")

        for obs in obstacles:
            if hasattr(obs, "z"):
                dist = obs.z
            elif isinstance(obs, (tuple, list)) and len(obs) >= 3:
                dist = obs[2]
            elif isinstance(obs, dict) and "distance" in obs:
                dist = obs["distance"]
            else:
                continue

            min_dist = min(min_dist, dist)

        return min_dist if min_dist != float("inf") else 10.0

    def _smooth_path(
        self, path: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        """平滑路径（RDP算法简化版本）

        Args:
            path: 原始路径点列表

        Returns:
            平滑后的路径点列表
        """
        if len(path) < 3:
            return path

        # RDP 算法参数
        epsilon = 0.05  # 简化阈值（米）

        # 递归实现 RDP
        def rdp_simplify(
            points: List[Tuple[float, float]], eps: float
        ) -> List[Tuple[float, float]]:
            if len(points) < 3:
                return points

            # 找到距离首尾连线最远的点
            start = np.array(points[0])
            end = np.array(points[-1])

            max_dist = 0
            max_idx = 0

            for i in range(1, len(points) - 1):
                point = np.array(points[i])

                # 计算点到线段的距离
                line_vec = end - start
                point_vec = point - start
                line_len = np.linalg.norm(line_vec)

                if line_len < 1e-6:
                    dist = np.linalg.norm(point_vec)
                else:
                    line_unit = line_vec / line_len
                    proj_length = np.dot(point_vec, line_unit)
                    proj_length = np.clip(proj_length, 0, line_len)
                    proj_point = start + proj_length * line_unit
                    dist = np.linalg.norm(point - proj_point)

                if dist > max_dist:
                    max_dist = dist
                    max_idx = i

            # 如果最大距离大于阈值，递归简化
            if max_dist > eps:
                left = rdp_simplify(points[: max_idx + 1], eps)
                right = rdp_simplify(points[max_idx:], eps)
                return left[:-1] + right
            else:
                return [points[0], points[-1]]

        try:
            simplified = rdp_simplify(path, epsilon)
            logger.debug(f"路径平滑: {len(path)} -> {len(simplified)} 个点")
            return simplified
        except Exception as e:
            logger.warning(f"路径平滑失败: {e}，返回原始路径")
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
