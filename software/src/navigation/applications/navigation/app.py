# -*- coding: utf-8 -*-
"""NavigationApp - 全局自主导航应用（基于 NavigationCoordinator）

整合 NavigationCoordinator 实现全局路径规划 + 局部避障，
在真机上通过 ZeroMQ 订阅传感器数据，向底盘服务发送命令。

数据流：
- SUB /slam/pose       -> SLAM融合位姿（全局定位）
- SUB /slam/map        -> SLAM栅格地图（全局规划）
- SUB /depth/obstacles -> 深度障碍物直方图（局部避障）
- NavigationCoordinator -> 全局规划 + 局部控制
- REQ ChassisService   -> 发送底盘命令

Usage:
    cd software/src
    python -m navigation.applications.navigation --goal-x 2.0 --goal-y 1.5
"""
from __future__ import annotations

import argparse
import json
import math
import time
from typing import List, Optional, Tuple

import numpy as np
import zmq

from common.logging import get_logger
from common.zmq_helper import create_socket
from common.zmq_subscriber import ZMQJsonSubscriber, ZMQMultipartJsonSubscriber
from configs import get_config
from navigation.core.occupancy_grid import (
    COST_FREE,
    COST_LETHAL,
    COST_UNKNOWN,
    OccupancyGrid,
)
from navigation.perception.obstacle_detector import DepthObstacle

logger = get_logger(__name__)

def _nav_cfg():
    """获取导航配置快捷方式"""
    return get_config().navigation

DEFAULT_SLAM_POSE_ADDR = "tcp://localhost:5563"
DEFAULT_SLAM_MAP_ADDR = "tcp://localhost:5564"
DEFAULT_OBSTACLE_ADDR = "tcp://localhost:5562"
DEFAULT_CHASSIS_ADDR = "tcp://127.0.0.1:5556"
DEFAULT_PATH_PUB_ADDR = "tcp://*:5569"
DEFAULT_GOAL_SUB_ADDR = "tcp://localhost:5566"


# ------------------------------------------------------------------------------
# SLAM 地图订阅者（继承 ZMQMultipartJsonSubscriber，复用线程与 socket 管理）
# ------------------------------------------------------------------------------
class SLAMMapSubscriber(ZMQMultipartJsonSubscriber):
    """订阅SLAM栅格地图 (multipart: json_meta + map_bytes)。"""

    def __init__(self, sub_addr: str = DEFAULT_SLAM_MAP_ADDR):
        super().__init__(sub_addr, required_keys=("size_pixels",), json_frame_index=0)
        self._latest_map_bytes: Optional[bytes] = None

    def _receive_loop(self) -> None:
        while self._running:
            try:
                parts = self._sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) >= 2:
                    data = json.loads(parts[0].decode("utf-8"))
                    if self._validate(data):
                        with self._lock:
                            self._latest_data = data
                            self._latest_map_bytes = parts[1]
                            self._recv_count += 1
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"[SLAMMapSubscriber] 接收异常: {e}")
            time.sleep(0.01)

    def read(self) -> Tuple[Optional[dict], Optional[bytes]]:
        with self._lock:
            return (
                self._latest_data.copy() if self._latest_data else None,
                self._latest_map_bytes,
            )


# ------------------------------------------------------------------------------
# 工具函数
# ------------------------------------------------------------------------------
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
) -> Optional[OccupancyGrid]:
    """将SLAM地图字节转换为OccupancyGrid。

    BreezySLAM getmap 值与标准占据栅格相反：
      255 = 确定空闲, 127 = 未知, 0 = 确定占据
      值越大越空闲，值越小越占据
    """
    expected_len = size_pixels * size_pixels
    if len(map_bytes) < expected_len:
        logger.warning(f"地图字节长度不足: {len(map_bytes)} < {expected_len}")
        return None

    resolution = size_meters / size_pixels
    origin = (-size_meters / 2.0, -size_meters / 2.0)

    grid = OccupancyGrid(
        size_pixels, size_pixels, resolution=resolution, origin=origin
    )

    arr = np.frombuffer(map_bytes, dtype=np.uint8).reshape(
        (size_pixels, size_pixels)
    )

    # BreezySLAM getmap 值映射
    grid.data[arr >= 200] = COST_FREE
    grid.data[(arr >= 50) & (arr < 200)] = COST_UNKNOWN
    grid.data[arr < 50] = COST_LETHAL

    return grid


# ------------------------------------------------------------------------------
# 全局导航应用
# ------------------------------------------------------------------------------
class NavigationApp:
    """全局自主导航应用（基于 NavigationCoordinator）。

    将 NavigationCoordinator 整合到真机运行环境中：
    - 位姿：订阅 SLAM 融合位姿
    - 障碍物：订阅 DepthService 直方图并转换为 DepthObstacle
    - 地图：订阅 SLAMService 栅格地图
    - 速度：通过 ChassisArbiterClient 发送给底盘服务
    """

    def __init__(
        self,
        goal_x: float = 1.0,
        goal_y: float = 0.0,
        slam_pose_addr: str = DEFAULT_SLAM_POSE_ADDR,
        slam_map_addr: str = DEFAULT_SLAM_MAP_ADDR,
        obstacle_addr: str = DEFAULT_OBSTACLE_ADDR,
        chassis_addr: str = DEFAULT_CHASSIS_ADDR,
        path_pub_addr: str = DEFAULT_PATH_PUB_ADDR,
        goal_sub_addr: str = DEFAULT_GOAL_SUB_ADDR,
        planner_config: Optional[LocalPlannerConfig] = None,
        arrival_threshold_m: Optional[float] = None,
        lookahead_distance_m: Optional[float] = None,
        replan_interval_s: Optional[float] = None,
        control_rate: Optional[float] = None,
        inflation_radius_m: Optional[float] = None,
        max_path_deviation_m: Optional[float] = None,
        use_depth: Optional[bool] = None,
    ):
        nav = _nav_cfg()
        self._goal: Tuple[float, float] = (goal_x, goal_y)
        self._arrival_threshold = arrival_threshold_m if arrival_threshold_m is not None else nav.arrival_distance_threshold_m
        self._lookahead = lookahead_distance_m if lookahead_distance_m is not None else nav.lookahead_distance_m
        self._replan_interval = replan_interval_s if replan_interval_s is not None else nav.replan_interval_s
        self._control_interval = 1.0 / (control_rate if control_rate is not None else nav.control_rate_hz) if (control_rate if control_rate is not None else nav.control_rate_hz) > 0 else 0.1
        self._inflation_radius = inflation_radius_m if inflation_radius_m is not None else nav.inflation_radius_m
        self._max_deviation = max_path_deviation_m if max_path_deviation_m is not None else nav.max_path_deviation_m

        # 数据订阅
        self._slam_pose_sub = ZMQJsonSubscriber(
            slam_pose_addr, required_keys=("x", "y", "theta")
        )
        self._slam_map_sub = SLAMMapSubscriber(slam_map_addr)
        # DepthService 发布 multipart: [topic, json]，json_frame_index=1
        self._use_depth = use_depth if use_depth is not None else nav.use_depth_obstacle
        if self._use_depth:
            self._obstacle_sub = ZMQMultipartJsonSubscriber(
                obstacle_addr, required_keys=("histogram",), json_frame_index=1
            )
        else:
            self._obstacle_sub = None
            logger.info("深度障碍物订阅已禁用，局部避障将仅依赖全局地图规划")

        self._goal_sub = ZMQJsonSubscriber(goal_sub_addr, required_keys=("x", "y"))
        logger.info(f"目标点 SUB: {goal_sub_addr}")

        # 底盘客户端
        from services.motion_service.chassis_arbiter import ChassisArbiterClient

        self._chassis = ChassisArbiterClient(chassis_addr, timeout_ms=500)

        # 全局路径发布者（供可视化端订阅）
        self._path_pub = create_socket(zmq.PUB, bind=True, address=path_pub_addr)
        logger.info(f"全局路径 PUB: {path_pub_addr}")

        # 导航协调器
        self._coordinator = NavigationCoordinator(
            {
                "goal_reached_distance": arrival_threshold_m,
                "control_frequency": control_rate,
                "max_replan_attempts": 5,
                "obstacle_emergency_distance": 0.3,
                "replan_distance_threshold": max_path_deviation_m,
                "inflation_radius": inflation_radius_m,
                "robot_radius": 0.25,
                "lookahead_distance": nav.lookahead_distance_m,
                "max_angular_accel_rad": nav.max_angular_accel_rad,
                "velocity_filter_alpha": nav.velocity_filter_alpha,
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
        self._idle_reported: bool = False

    # ------------------------------------------------------------------
    # Coordinator 接口提供者
    # ------------------------------------------------------------------
    def _get_pose(self) -> Optional[Tuple[float, float, float]]:
        """位姿提供者（供 Coordinator 调用）。"""
        msg = self._slam_pose_sub.read()
        if msg:
            return (
                float(msg.get("x", 0.0)),
                float(msg.get("y", 0.0)),
                float(msg.get("theta", 0.0)),
            )
        return None

    def _get_obstacles(self) -> List[DepthObstacle]:
        """障碍物提供者（供 Coordinator 调用）。"""
        if not self._use_depth:
            return []

        msg = self._obstacle_sub.read()
        hist = self._extract_histogram(msg)
        return histogram_to_obstacles(hist)

    def _get_map(self) -> Optional[OccupancyGrid]:
        """地图提供者（供 Coordinator 调用）。"""
        meta, map_bytes = self._slam_map_sub.read()
        if meta is None or map_bytes is None:
            return None
        try:
            return _bytes_to_occupancy_grid(
                map_bytes,
                meta.get("size_pixels", 800),
                meta.get("size_meters", 10.0),
            )
        except Exception as e:
            logger.warning(f"地图转换失败: {e}")
            return None

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

    def _check_viser_goal(self) -> Optional[dict]:
        """检查 Viser 可视化器是否发布了新的目标点。

        Returns:
            新目标点字典，如果没有新目标则返回 None
        """
        goal = self._goal_sub.read()
        if goal is None:
            return None
        ts = goal.get("timestamp", 0)
        if ts <= self._last_goal_timestamp:
            return None
        self._last_goal_timestamp = ts
        return goal

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------
    def set_goal(self, x: float, y: float, yaw: Optional[float] = None) -> None:
        """动态更新目标点。"""
        self.goal_x = x
        self.goal_y = y
        self._current_goal_id = self._coordinator.navigate_to_async(x, y, yaw=yaw)
        self._idle_reported = False
        yaw_str = f"{yaw:.2f}" if yaw is not None else "N/A"
        logger.info(f"目标点已更新: ({x}, {y}, yaw={yaw_str}), ID={self._current_goal_id}")

    def stop_navigation(self) -> None:
        """立即停止导航：取消所有目标并停止机器人。"""
        self._coordinator.cancel_all_goals()
        self._send_velocity(0.0, 0.0)
        self._current_goal_id = None
        self._idle_reported = False
        logger.info("导航已停止：所有目标已取消，机器人已停止")

    def set_waypoints(self, waypoints: List[Tuple[float, float]], final_goal: Tuple[float, float, Optional[float]]) -> None:
        """设置途径点队列，依次导航经过各途径点后到达最终目标。

        Args:
            waypoints: 途径点列表 [(x, y), ...]
            final_goal: 最终目标 (x, y, yaw)，yaw 可为 None
        """
        if not waypoints:
            # 无途径点时直接设置最终目标
            self.set_goal(final_goal[0], final_goal[1], final_goal[2])
            return

        # 先取消所有待处理目标
        self._coordinator.cancel_all_goals()

        # 依次添加途径点到队列
        for i, (wx, wy) in enumerate(waypoints):
            self._coordinator.navigate_to_async(wx, wy, yaw=None, priority=0)
            logger.info(f"添加途径点 {i+1}/{len(waypoints)}: ({wx:.2f}, {wy:.2f})")

        # 最后添加最终目标
        self._current_goal_id = self._coordinator.navigate_to_async(
            final_goal[0], final_goal[1], yaw=final_goal[2], priority=0
        )
        self.goal_x = final_goal[0]
        self.goal_y = final_goal[1]
        self._idle_reported = False
        yaw_str = f"{final_goal[2]:.2f}" if final_goal[2] is not None else "N/A"
        logger.info(
            f"导航任务已设置: {len(waypoints)} 个途径点 → 最终目标 "
            f"({final_goal[0]:.2f}, {final_goal[1]:.2f}, yaw={yaw_str}), "
            f"最终目标ID={self._current_goal_id}"
        )

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def start(self) -> None:
        """启动导航主循环。"""
        self._running = True
        self._coordinator.start()

        logger.info(
            f"NavigationApp 启动，初始目标=({self.goal_x}, {self.goal_y}), "
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
                    missing.append("slam_pose")
                if grid is None:
                    missing.append("slam_map")
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

                # 发布 Coordinator 的全局路径（供可视化端订阅）
                self._publish_coordinator_path()

                # 检查 Viser 可视化器是否发布了新的目标点/导航任务
                viser_goal = self._check_viser_goal()
                if viser_goal:
                    # 判断是否为停止命令
                    if viser_goal.get("cmd") == "stop":
                        logger.info("收到 Viser 停止导航命令")
                        self.stop_navigation()
                        continue

                    # 判断是否为途径点任务格式
                    waypoints_raw = viser_goal.get("waypoints")
                    if waypoints_raw and isinstance(waypoints_raw, list):
                        waypoints = []
                        for wp in waypoints_raw:
                            if isinstance(wp, dict):
                                waypoints.append((float(wp.get("x", 0.0)), float(wp.get("y", 0.0))))
                            elif isinstance(wp, (list, tuple)) and len(wp) >= 2:
                                waypoints.append((float(wp[0]), float(wp[1])))
                        final = viser_goal.get("final_goal", {})
                        fx = float(final.get("x", viser_goal.get("x", 0.0)))
                        fy = float(final.get("y", viser_goal.get("y", 0.0)))
                        ftheta = final.get("theta")
                        if ftheta is not None:
                            ftheta = float(ftheta)
                        logger.info(f"收到 Viser 导航任务: {len(waypoints)} 个途径点 → 最终目标 ({fx:.2f}, {fy:.2f})")
                        self.set_waypoints(waypoints, (fx, fy, ftheta))
                    else:
                        # 旧格式：单目标点
                        x = float(viser_goal.get("x", 0.0))
                        y = float(viser_goal.get("y", 0.0))
                        theta = viser_goal.get("theta")
                        if theta is not None:
                            theta = float(theta)
                        logger.info(f"收到 Viser 目标点: ({x:.2f}, {y:.2f}, theta={theta})")
                        self.set_goal(x, y, yaw=theta)
                    continue

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
                    if not self._idle_reported:
                        if feedback.error_msg:
                            logger.error(f"导航失败: {feedback.error_msg}")
                        else:
                            logger.info("导航成功！等待新的 Viser 目标点...")
                        self._idle_reported = True
                    # 非阻塞等待 Viser 发布新目标，继续轮询
                    continue

        except KeyboardInterrupt:
            logger.info("NavigationApp 被用户中断")
        except Exception as e:
            logger.error(f"NavigationApp 异常: {e}")
            import traceback

            logger.error(traceback.format_exc())
        finally:
            self.stop()

    def stop(self) -> None:
        """停止应用并释放资源。"""
        self._running = False
        self._coordinator.stop()
        self._send_velocity(0.0, 0.0)
        self._slam_pose_sub.close()
        self._slam_map_sub.close()
        if self._obstacle_sub is not None:
            self._obstacle_sub.close()
        self._goal_sub.close()
        self._path_pub.close()
        self._chassis.close()
        logger.info("NavigationApp 已停止")

    # ------------------------------------------------------------------
    # 障碍物数据转换
    # ------------------------------------------------------------------
    @staticmethod
    def _extract_histogram(obstacle_msg: Optional[dict]) -> Optional[np.ndarray]:
        """从 ZMQMultipartJsonSubscriber 读取的 JSON 中提取 histogram 数组。"""
        if obstacle_msg is None:
            return None
        hist_list = obstacle_msg.get("histogram", [])
        if not hist_list:
            return None
        return np.array(
            [np.inf if v is None else float(v) for v in hist_list],
            dtype=np.float32,
        )

    # ------------------------------------------------------------------
    # 路径发布
    # ------------------------------------------------------------------
    def _publish_coordinator_path(self) -> None:
        """发布 Coordinator 的全局路径到 ZMQ，供可视化端订阅。"""
        path = self._coordinator.global_path
        if not path:
            return
        try:
            msg = {
                "path": [[float(x), float(y)] for x, y in path],
                "timestamp": time.time(),
            }
            self._path_pub.send_json(msg, flags=zmq.NOBLOCK)
        except Exception as e:
            logger.debug(f"发布全局路径失败: {e}")


# ------------------------------------------------------------------------------
# 入口
# ------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="HomeBot 全局自主导航应用")
    parser.add_argument(
        "--goal-x", type=float, default=1.0, help="目标点 X（米，地图坐标系）"
    )
    parser.add_argument(
        "--goal-y", type=float, default=0.0, help="目标点 Y（米，地图坐标系）"
    )
    parser.add_argument(
        "--slam-pose", default=DEFAULT_SLAM_POSE_ADDR, help="SLAM位姿 SUB 地址"
    )
    parser.add_argument(
        "--slam-map", default=DEFAULT_SLAM_MAP_ADDR, help="SLAM地图 SUB 地址"
    )
    parser.add_argument(
        "--obstacle", default=DEFAULT_OBSTACLE_ADDR, help="障碍物 SUB 地址"
    )
    parser.add_argument(
        "--chassis", default=DEFAULT_CHASSIS_ADDR, help="底盘服务地址"
    )
    parser.add_argument("--rate", type=float, default=10.0, help="控制频率 Hz")
    parser.add_argument(
        "--threshold", type=float, default=0.15, help="到达阈值（米）"
    )
    parser.add_argument(
        "--inflation", type=float, default=0.2, help="障碍物膨胀半径（米）"
    )
    parser.add_argument(
        "--deviation", type=float, default=0.5, help="最大路径偏离阈值（米）"
    )
    parser.add_argument(
        "--use-depth",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="是否使用深度图做局部避障（默认开启，--no-use-depth 可关闭）",
    )
    parser.add_argument(
        "--path-pub", default=DEFAULT_PATH_PUB_ADDR, help="全局路径 PUB 地址"
    )
    parser.add_argument(
        "--goal-sub", default=DEFAULT_GOAL_SUB_ADDR, help="目标点 SUB 地址（Viser 发布端）"
    )
    args = parser.parse_args()

    app = NavigationApp(
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        slam_pose_addr=args.slam_pose,
        slam_map_addr=args.slam_map,
        obstacle_addr=args.obstacle,
        chassis_addr=args.chassis,
        path_pub_addr=args.path_pub,
        goal_sub_addr=args.goal_sub,
        control_rate=args.rate,
        arrival_threshold_m=args.threshold,
        inflation_radius_m=args.inflation,
        max_path_deviation_m=args.deviation,
        use_depth=args.use_depth,
    )
    app.start()


if __name__ == "__main__":
    main()
