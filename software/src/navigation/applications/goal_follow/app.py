# -*- coding: utf-8 -*-
"""GoalFollowApp - 目标点跟随应用（基于 NavigationCoordinator）

整合 NavigationCoordinator 实现全局路径规划 + 局部避障，
在真机上通过 ZeroMQ 订阅传感器数据，向底盘服务发送命令。

数据流：
- SUB /odom/pose       -> 获取当前位姿
- SUB /depth/obstacles -> 获取障碍物直方图（转换为 DepthObstacle）
- SUB /slam/map        -> 获取 SLAM 栅格地图
- NavigationCoordinator -> 全局规划 + 局部控制
- REQ ChassisService   -> 发送底盘命令

真机启动依赖（按顺序）：
1. OdomService     (tcp://localhost:5559)
2. VisionService   (tcp://localhost:5560)
3. DepthService    (tcp://localhost:5562)
4. SLAMService     (tcp://localhost:5563/5564)
5. ChassisService  (tcp://127.0.0.1:5556)
6. GoalFollowApp
"""
from __future__ import annotations

import json
import math
import threading
import time
from typing import List, Optional, Tuple

import numpy as np
import zmq

from common.logging import get_logger
from common.zmq_subscriber import ZMQJsonSubscriber, ZMQMultipartJsonSubscriber
from common.zmq_helper import create_socket
from navigation.coordinator.navigation_coordinator import (
    NavigationCoordinator,
    NavigationState,
)
from navigation.core.occupancy_grid import (
    OccupancyGrid,
    COST_FREE,
    COST_UNKNOWN,
    COST_LETHAL,
)
from navigation.perception.obstacle_detector import DepthObstacle

logger = get_logger(__name__)

DEFAULT_ODOM_ADDR = "tcp://localhost:5559"
DEFAULT_OBSTACLE_ADDR = "tcp://localhost:5562"
DEFAULT_LIDAR_SCAN_ADDR = "tcp://localhost:5565"
DEFAULT_MAP_ADDR = "tcp://localhost:5564"
DEFAULT_CHASSIS_ADDR = "tcp://127.0.0.1:5556"
DEFAULT_GOAL_SUB_ADDR = "tcp://localhost:5566"


# ------------------------------------------------------------------------------
# ZeroMQ 订阅者
# ------------------------------------------------------------------------------


class OdomSubscriber(ZMQJsonSubscriber):
    """里程计订阅者"""

    def __init__(self, sub_addr: str = DEFAULT_ODOM_ADDR):
        super().__init__(sub_addr, required_keys=("x", "y", "yaw"))


class GoalSubscriber(ZMQJsonSubscriber):
    """导航目标点订阅者

    订阅 Viser SLAM 可视化器发布的导航目标点。
    消息格式: {"x": float, "y": float, "theta": float, "timestamp": float}
    """

    def __init__(self, sub_addr: str = DEFAULT_GOAL_SUB_ADDR):
        super().__init__(sub_addr, required_keys=("x", "y", "timestamp"))


class HistogramSubscriber(ZMQMultipartJsonSubscriber):
    """障碍物距离直方图订阅者

    订阅 DepthService 发布的 multipart 消息，自动解析 JSON 中的 histogram 数组。
    """

    def __init__(self, sub_addr: str = DEFAULT_OBSTACLE_ADDR):
        super().__init__(sub_addr, required_keys=("histogram",))
        self._latest_histogram: Optional[np.ndarray] = None
        self._hist_lock = threading.Lock()

    def _receive_loop(self) -> None:
        """覆盖基类接收循环，增加 histogram 数组解析。"""
        while self._running:
            try:
                parts = self._sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) > self._json_frame_index:
                    data = json.loads(parts[self._json_frame_index].decode("utf-8"))
                    if self._validate(data):
                        hist_list = data.get("histogram", [])
                        hist = np.array(
                            [np.inf if v is None else float(v) for v in hist_list],
                            dtype=np.float32,
                        )
                        with self._lock:
                            self._latest_data = data
                            self._recv_count += 1
                        with self._hist_lock:
                            self._latest_histogram = hist
                    else:
                        logger.warning(
                            f"[{self.__class__.__name__}] 收到不符合要求的数据: {data}"
                        )
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"[{self.__class__.__name__}] 接收异常: {e}")
            time.sleep(0.001)

    def read(self) -> Optional[np.ndarray]:
        """读取最新直方图数组（线程安全，非阻塞）。"""
        with self._hist_lock:
            return (
                self._latest_histogram.copy()
                if self._latest_histogram is not None
                else None
            )

    def close(self) -> None:
        super().close()


class LidarScanSubscriber(ZMQJsonSubscriber):
    """激光雷达扫描数据订阅者。

    订阅 SLAMService 发布的激光雷达扫描数据：
    {"angles_deg": [...], "distances_m": [...], "timestamp": ...}
    """

    def __init__(self, sub_addr: str = DEFAULT_LIDAR_SCAN_ADDR):
        super().__init__(sub_addr, required_keys=("angles_deg", "distances_m"))
        self._latest_angles: Optional[np.ndarray] = None
        self._latest_distances: Optional[np.ndarray] = None
        self._scan_lock = threading.Lock()

    def _receive_loop(self) -> None:
        """覆盖基类接收循环，解析激光雷达扫描数组。"""
        while self._running:
            try:
                data = self._sub.recv_json(flags=zmq.NOBLOCK)
                if self._validate(data):
                    angles = np.array(data["angles_deg"], dtype=np.float32)
                    distances = np.array(data["distances_m"], dtype=np.float32)
                    with self._lock:
                        self._latest_data = data
                        self._recv_count += 1
                    with self._scan_lock:
                        self._latest_angles = angles
                        self._latest_distances = distances
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"[{self.__class__.__name__}] 接收异常: {e}")
            time.sleep(0.001)

    def read(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """读取最新扫描数据（线程安全，非阻塞）。"""
        with self._scan_lock:
            a = (
                self._latest_angles.copy()
                if self._latest_angles is not None
                else None
            )
            d = (
                self._latest_distances.copy()
                if self._latest_distances is not None
                else None
            )
            return a, d

    def close(self) -> None:
        super().close()


class MapSubscriber:
    """SLAM 栅格地图订阅者。

    订阅 SLAMService 发布的 multipart 地图消息：
    - frame 0: JSON 元信息 {"size_pixels": 800, "size_meters": 10.0, ...}
    - frame 1: 地图字节数组 (bytearray)
    """

    def __init__(self, sub_addr: str = DEFAULT_MAP_ADDR):
        self._sub = create_socket(zmq.SUB, bind=False, address=sub_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.setsockopt(zmq.RCVTIMEO, 500)

        self._latest_grid: Optional[OccupancyGrid] = None
        self._latest_meta: Optional[dict] = None
        self._lock = threading.Lock()
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._start_receiver()

    def _start_receiver(self) -> None:
        self._running = True
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

    def _receive_loop(self) -> None:
        while self._running:
            try:
                parts = self._sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) >= 2:
                    meta = json.loads(parts[0].decode("utf-8"))
                    map_bytes = parts[1]
                    grid = _bytes_to_occupancy_grid(
                        map_bytes, meta["size_pixels"], meta["size_meters"]
                    )
                    with self._lock:
                        self._latest_meta = meta
                        self._latest_grid = grid
                        logger.debug(
                            f"[MapSubscriber] 收到地图 "
                            f"{meta['size_pixels']}x{meta['size_pixels']}"
                        )
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"[MapSubscriber] 接收异常: {e}")
            time.sleep(0.001)

    def read(self) -> Optional[OccupancyGrid]:
        """读取最新地图（线程安全，非阻塞）。"""
        with self._lock:
            return self._latest_grid

    def close(self) -> None:
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        self._sub.close()


# ------------------------------------------------------------------------------
# 工具函数
# ------------------------------------------------------------------------------


def lidar_scan_to_obstacles(
    angles_deg: Optional[np.ndarray],
    distances_m: Optional[np.ndarray],
    max_range: float = 5.0,
    min_range: float = 0.3,
) -> List[DepthObstacle]:
    """将激光雷达扫描数据转换为 DepthObstacle 列表。

    Args:
        angles_deg: 角度数组（度），0=正前方，正=逆时针（左侧为正）
        distances_m: 距离数组（米）
        max_range: 最大有效距离
        min_range: 最小有效距离（过滤机器人自身）

    Returns:
        DepthObstacle 列表（相机坐标系，z=前，x=右）
    """
    if angles_deg is None or distances_m is None:
        return []

    obstacles = []
    step = max(1, len(angles_deg) // 72)  # 降采样到约72个点

    for i in range(0, len(angles_deg), step):
        dist = float(distances_m[i])
        if dist >= max_range or dist < min_range:
            continue

        angle_rad = math.radians(float(angles_deg[i]))
        # 相机/雷达坐标系：z=前，x=右
        z = dist * math.cos(angle_rad)
        x = dist * math.sin(angle_rad)

        obstacles.append(
            DepthObstacle(
                x=x,
                y=0.0,
                z=z,
                width=0.1,
                height=0.1,
                confidence=0.9,
            )
        )

    return obstacles


def histogram_to_obstacles(histogram: Optional[np.ndarray]) -> List[DepthObstacle]:
    """将距离直方图转换为 DepthObstacle 列表。

    Args:
        histogram: 距离直方图，shape=(num_columns,)，单位：米，inf=无障碍

    Returns:
        DepthObstacle 列表（相机坐标系，z=前，x=右）
    """
    if histogram is None or len(histogram) == 0:
        return []

    obstacles = []
    num_columns = len(histogram)
    fov_horizontal = math.radians(66.0)  # 与 DepthObstacleDetector 默认一致

    for i, dist in enumerate(histogram):
        if dist is None or np.isinf(dist) or dist <= 0:
            continue

        # 条带中心角度（相机坐标系，向右为正）
        angle = ((i + 0.5) / num_columns) * fov_horizontal - fov_horizontal / 2

        # 相机坐标系：z=前，x=右
        z = float(dist)
        x = z * math.tan(angle)

        obstacles.append(
            DepthObstacle(
                x=x,
                y=0.0,
                z=z,
                width=0.1,
                height=0.1,
                confidence=0.8,
            )
        )

    return obstacles


def _bytes_to_occupancy_grid(
    map_bytes: bytes, size_pixels: int, size_meters: float
) -> OccupancyGrid:
    """将 BreezySLAM 地图字节数组转换为 OccupancyGrid。

    BreezySLAM 地图字节含义：
    - 0   ~ 50  : 空闲
    - 50  ~ 200 : 未知
    - 200 ~ 255 : 占用
    """
    resolution = size_meters / size_pixels
    origin = (-size_meters / 2, -size_meters / 2)

    grid = OccupancyGrid(
        width=size_pixels,
        height=size_pixels,
        resolution=resolution,
        origin=origin,
        default_cost=COST_UNKNOWN,
    )

    arr = np.frombuffer(map_bytes, dtype=np.uint8).reshape((size_pixels, size_pixels))

    # 阈值映射为 OccupancyGrid 代价值
    grid.data = np.where(
        arr < 50,
        COST_FREE,
        np.where(arr > 200, COST_LETHAL, COST_UNKNOWN),
    ).astype(np.int16)

    return grid


# ------------------------------------------------------------------------------
# 应用主类
# ------------------------------------------------------------------------------


class GoalFollowApp:
    """目标点跟随应用（基于 NavigationCoordinator）。

    将 NavigationCoordinator 整合到真机运行环境中：
    - 位姿：订阅 OdomService
    - 障碍物：订阅 DepthService 直方图并转换为 DepthObstacle
    - 地图：订阅 SLAMService 栅格地图
    - 速度：通过 ChassisArbiterClient 发送给底盘服务
    """

    def __init__(
        self,
        goal_x: float = 1.0,
        goal_y: float = 0.0,
        odom_addr: str = DEFAULT_ODOM_ADDR,
        obstacle_addr: str = DEFAULT_OBSTACLE_ADDR,
        lidar_scan_addr: str = DEFAULT_LIDAR_SCAN_ADDR,
        map_addr: str = DEFAULT_MAP_ADDR,
        chassis_addr: str = DEFAULT_CHASSIS_ADDR,
        goal_sub_addr: str = DEFAULT_GOAL_SUB_ADDR,
        arrival_threshold_m: float = 0.2,
        control_rate: float = 10.0,
        use_depth: bool = True,
    ):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.arrival_threshold = arrival_threshold_m
        self._use_depth = use_depth

        # 数据订阅
        self._odom_sub = OdomSubscriber(odom_addr)
        if use_depth:
            self._obstacle_sub = HistogramSubscriber(obstacle_addr)
            self._lidar_sub = None
            logger.info("障碍物检测模式: 深度感知 (DepthService)")
        else:
            self._obstacle_sub = None
            self._lidar_sub = LidarScanSubscriber(lidar_scan_addr)
            logger.info("障碍物检测模式: 激光雷达 (SLAM LidarScan)")
        self._map_sub = MapSubscriber(map_addr)
        self._goal_sub = GoalSubscriber(goal_sub_addr)

        # 底盘客户端
        from services.motion_service.chassis_arbiter import ChassisArbiterClient

        self._chassis = ChassisArbiterClient(chassis_addr, timeout_ms=500)

        # 导航协调器
        self._coordinator = NavigationCoordinator(
            {
                "goal_reached_distance": arrival_threshold_m,
                "control_frequency": control_rate,
                "max_replan_attempts": 5,
                "obstacle_emergency_distance": 0.3,
                "replan_distance_threshold": 0.5,
                "inflation_radius": 0.15,
                "robot_radius": 0.25,
            }
        )

        # 注入外部接口（闭包引用 self）
        self._coordinator.set_pose_provider(self._get_pose)
        self._coordinator.set_obstacle_provider(self._get_obstacles)
        self._coordinator.set_velocity_sender(self._send_velocity)
        self._coordinator.set_map_provider(self._get_map)

        self._running = False
        self._current_goal_id: Optional[str] = None
        self._last_goal_timestamp: float = 0.0

    # ------------------------------------------------------------------
    # Coordinator 接口提供者
    # ------------------------------------------------------------------

    def _get_pose(self) -> Optional[Tuple[float, float, float]]:
        """位姿提供者（供 Coordinator 调用）。"""
        odom = self._odom_sub.read()
        if odom:
            return (
                odom.get("x", 0.0),
                odom.get("y", 0.0),
                odom.get("yaw", 0.0),
            )
        return None

    def _get_obstacles(self) -> List[DepthObstacle]:
        """障碍物提供者（供 Coordinator 调用）。"""
        if self._use_depth:
            histogram = self._obstacle_sub.read()
            return histogram_to_obstacles(histogram)
        else:
            angles, distances = self._lidar_sub.read()
            return lidar_scan_to_obstacles(angles, distances)

    def _send_velocity(self, linear: float, angular: float) -> bool:
        """速度发送器（供 Coordinator 调用）。

        Coordinator 接口: (linear, angular) -> bool
        ChassisArbiterClient 接口: (vx, vy, vz, source, priority)
        """
        try:
            response = self._chassis.send_command(
                linear, 0.0, angular, source="auto", priority=3
            )
            return response is not None
        except Exception as e:
            logger.warning(f"底盘命令发送异常: {e}")
            return False

    def _get_map(self) -> Optional[OccupancyGrid]:
        """地图提供者（供 Coordinator 调用）。"""
        return self._map_sub.read()

    def _check_viser_goal(self) -> Optional[dict]:
        """检查 Viser 可视化器是否发布了新的目标点。

        Returns:
            新目标点字典，如果没有新目标则返回 None
        """
        if self._goal_sub is None:
            return None
        goal = self._goal_sub.read()
        if goal is None:
            return None
        ts = goal.get("timestamp", 0)
        if ts <= self._last_goal_timestamp:
            return None
        self._last_goal_timestamp = ts
        return goal

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(self) -> None:
        """启动目标跟随循环。"""
        self._running = True
        self._coordinator.start()

        logger.info(
            f"GoalFollowApp 启动，初始目标=({self.goal_x}, {self.goal_y}), "
            f"使用 NavigationCoordinator 进行全局规划+局部避障"
        )

        # 等待传感器数据就绪（避免地图/位姿未收到就规划导致失败）
        logger.info("等待传感器数据就绪...")
        wait_count = 0
        while self._running:
            pose = self._get_pose()
            grid = self._get_map()
            if pose is not None and grid is not None:
                logger.info(
                    f"传感器就绪，位姿=({pose[0]:.2f}, {pose[1]:.2f}), "
                    f"地图={grid.width}x{grid.height}"
                )
                break
            wait_count += 1
            if wait_count % 10 == 0:
                missing = []
                if pose is None:
                    missing.append("odom")
                if grid is None:
                    missing.append("map")
                logger.info(f"等待 {', '.join(missing)} 数据...")
            time.sleep(0.5)

        if not self._running:
            return

        # 发送初始目标（异步）
        self._current_goal_id = self._coordinator.navigate_to_async(
            self.goal_x, self.goal_y
        )

        try:
            while self._running:
                time.sleep(0.5)

                if not self._current_goal_id:
                    continue

                feedback = self._coordinator.get_feedback(self._current_goal_id)
                if not feedback:
                    continue

                state = feedback.state
                dist = feedback.distance_to_goal
                progress = feedback.progress

                logger.info(
                    f"状态: {state.value} | 距离: {dist:.2f}m | 进度: {progress:.0%}"
                )

                # 导航完成或失败
                if state == NavigationState.IDLE:
                    if feedback.error_msg:
                        logger.error(f"导航失败: {feedback.error_msg}")
                    else:
                        logger.info("导航成功！")

                    # 优先检查 Viser 发来的目标点
                    viser_goal = self._check_viser_goal()
                    if viser_goal:
                        x = viser_goal.get("x", 0.0)
                        y = viser_goal.get("y", 0.0)
                        logger.info(f"收到 Viser 目标点: ({x}, {y})")
                        self.set_goal(x, y)
                        continue

                    # 否则命令行输入下一个目标
                    next_goal = input(
                        "请输入下一个目标点 (x y)，或 'exit' 退出: "
                    )
                    if next_goal.strip().lower() == "exit":
                        logger.info("用户请求退出")
                        break

                    try:
                        x_str, y_str = next_goal.strip().split()
                        x, y = float(x_str), float(y_str)
                        self.set_goal(x, y)
                    except Exception as e:
                        logger.warning(f"无效输入 '{next_goal}': {e}")

        except KeyboardInterrupt:
            logger.info("GoalFollowApp 被用户中断")
        except Exception as e:
            logger.error(f"GoalFollowApp 异常: {e}")
            import traceback

            logger.error(traceback.format_exc())
        finally:
            self.stop()

    def stop(self) -> None:
        """停止应用并释放资源。"""
        self._running = False
        self._coordinator.stop()
        self._send_velocity(0.0, 0.0)
        self._odom_sub.close()
        if self._obstacle_sub:
            self._obstacle_sub.close()
        if self._lidar_sub:
            self._lidar_sub.close()
        if hasattr(self, "_goal_sub") and self._goal_sub:
            self._goal_sub.close()
        self._map_sub.close()
        self._chassis.close()
        logger.info("GoalFollowApp 已停止")

    def set_goal(self, x: float, y: float) -> None:
        """动态更新目标点。"""
        self.goal_x = x
        self.goal_y = y
        self._current_goal_id = self._coordinator.navigate_to_async(x, y)
        logger.info(f"目标点已更新: ({x}, {y}), ID={self._current_goal_id}")


# ------------------------------------------------------------------------------
# 入口
# ------------------------------------------------------------------------------


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="HomeBot 目标点跟随应用（NavigationCoordinator）"
    )
    parser.add_argument("--goal-x", type=float, default=1.0, help="目标点 X（米）")
    parser.add_argument("--goal-y", type=float, default=0.0, help="目标点 Y（米）")
    parser.add_argument("--odom", default=DEFAULT_ODOM_ADDR, help="里程计 SUB 地址")
    parser.add_argument(
        "--obstacle", default=DEFAULT_OBSTACLE_ADDR, help="障碍物 SUB 地址"
    )
    parser.add_argument("--map", default=DEFAULT_MAP_ADDR, help="SLAM 地图 SUB 地址")
    parser.add_argument("--lidar", default=DEFAULT_LIDAR_SCAN_ADDR, help="激光雷达扫描 SUB 地址")
    parser.add_argument("--chassis", default=DEFAULT_CHASSIS_ADDR, help="底盘服务地址")
    parser.add_argument(
        "--goal-sub",
        default=DEFAULT_GOAL_SUB_ADDR,
        help="Viser 目标点 SUB 地址 (默认 tcp://localhost:5566)",
    )
    parser.add_argument("--rate", type=float, default=10.0, help="控制频率 Hz")
    parser.add_argument("--threshold", type=float, default=0.1, help="到达阈值（米）")
    parser.add_argument(
        "--use-depth",
        action="store_true",
        default=False,
        help="启用深度感知障碍物检测（默认关闭，使用激光雷达）",
    )
    args = parser.parse_args()

    app = GoalFollowApp(
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        odom_addr=args.odom,
        obstacle_addr=args.obstacle,
        lidar_scan_addr=args.lidar,
        map_addr=args.map,
        chassis_addr=args.chassis,
        goal_sub_addr=args.goal_sub,
        control_rate=args.rate,
        arrival_threshold_m=args.threshold,
        use_depth=args.use_depth,
    )
    app.start()


if __name__ == "__main__":
    main()
