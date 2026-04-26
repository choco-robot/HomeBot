# -*- coding: utf-8 -*-
"""NavigationApp - 全局自主导航应用

基于SLAM地图进行A*全局路径规划，结合VFH局部避障控制器，
实现机器人在已知地图中从当前位姿自主移动到目标点。

数据流：
- SUB /slam/pose      -> SLAM融合位姿（全局定位）
- SUB /slam/map       -> SLAM栅格地图（全局规划）
- SUB /depth/obstacles-> 深度障碍物直方图（局部避障）
- A* Global Planner   -> 全局路径
- VFH Local Planner   -> 局部速度指令
- REQ ChassisService  -> 发送底盘命令

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
from common.transform import world_to_robot2
from common.zmq_subscriber import ZMQJsonSubscriber, ZMQMultipartJsonSubscriber
from navigation.core.occupancy_grid import (
    COST_FREE,
    COST_LETHAL,
    COST_OCCUPIED,
    COST_UNKNOWN,
    OccupancyGrid,
)
from navigation.core.astar_planner import AStarPlanner
from navigation.planning.local_planner import LocalPlannerConfig, VFHLocalPlanner

logger = get_logger(__name__)

DEFAULT_SLAM_POSE_ADDR = "tcp://localhost:5563"
DEFAULT_SLAM_MAP_ADDR = "tcp://localhost:5564"
DEFAULT_OBSTACLE_ADDR = "tcp://localhost:5562"
DEFAULT_CHASSIS_ADDR = "tcp://127.0.0.1:5556"


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
# 全局导航应用
# ------------------------------------------------------------------------------
class NavigationApp:
    """全局自主导航应用。

    整合SLAM全局定位、A*全局规划、VFH局部避障，实现目标点自主导航。
    """

    def __init__(
        self,
        goal_x: float = 1.0,
        goal_y: float = 0.0,
        slam_pose_addr: str = DEFAULT_SLAM_POSE_ADDR,
        slam_map_addr: str = DEFAULT_SLAM_MAP_ADDR,
        obstacle_addr: str = DEFAULT_OBSTACLE_ADDR,
        chassis_addr: str = DEFAULT_CHASSIS_ADDR,
        planner_config: Optional[LocalPlannerConfig] = None,
        arrival_threshold_m: float = 0.15,
        lookahead_distance_m: float = 0.4,
        replan_interval_s: float = 3.0,
        control_rate: float = 10.0,
        inflation_radius_m: float = 0.2,
        max_path_deviation_m: float = 0.5,
        use_depth: bool = True,
    ):
        self._goal: Tuple[float, float] = (goal_x, goal_y)
        self._arrival_threshold = arrival_threshold_m
        self._lookahead = lookahead_distance_m
        self._replan_interval = replan_interval_s
        self._control_interval = 1.0 / control_rate if control_rate > 0 else 0.1
        self._inflation_radius = inflation_radius_m
        self._max_deviation = max_path_deviation_m

        # 数据订阅（直接复用 common.zmq_subscriber 提供的类）
        self._slam_pose_sub = ZMQJsonSubscriber(
            slam_pose_addr, required_keys=("x", "y", "theta")
        )
        self._slam_map_sub = SLAMMapSubscriber(slam_map_addr)
        # DepthService 发布 multipart: [topic, json]，json_frame_index=1
        self._use_depth = use_depth
        if self._use_depth:
            self._obstacle_sub = ZMQMultipartJsonSubscriber(
                obstacle_addr, required_keys=("histogram",), json_frame_index=1
            )
        else:
            self._obstacle_sub = None
            logger.info("深度障碍物订阅已禁用，局部避障将仅依赖全局地图规划")

        # 底盘客户端
        from services.motion_service.chassis_arbiter import ChassisArbiterClient

        self._chassis = ChassisArbiterClient(chassis_addr, timeout_ms=500)

        # 局部规划器
        self._local_planner = VFHLocalPlanner(
            planner_config or LocalPlannerConfig()
        )

        # 状态
        self._running = False
        self._state = "IDLE"  # IDLE, PLANNING, FOLLOWING, REACHED, FAILED
        self._global_path: Optional[List[Tuple[float, float]]] = None
        self._last_replan_time = 0.0
        self._last_pose: Optional[Tuple[float, float, float]] = None
        self._grid: Optional[OccupancyGrid] = None
        self._reached_reported = False

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------
    def set_goal(self, x: float, y: float) -> None:
        """动态更新目标点。"""
        self._goal = (x, y)
        self._reached_reported = False
        logger.info(f"目标点已更新: ({x}, {y})")
        self._state = "PLANNING"
        self._plan()

    def start(self) -> None:
        """启动导航主循环。"""
        self._running = True
        logger.info(
            f"NavigationApp 启动，目标=({self._goal[0]}, {self._goal[1]}), "
            f"控制频率={1/self._control_interval:.0f} Hz"
        )

        # 尝试初始规划
        if self._state == "IDLE":
            self._state = "PLANNING"
        if not self._plan():
            logger.warning("初始规划失败，将在主循环中持续尝试...")

        try:
            while self._running:
                t0 = time.perf_counter()

                # 1. 读取最新位姿
                pose_msg = self._slam_pose_sub.read()
                if pose_msg is None:
                    logger.debug("尚未收到SLAM位姿，等待中...")
                    time.sleep(0.1)
                    continue

                pose = (
                    float(pose_msg.get("x", 0.0)),
                    float(pose_msg.get("y", 0.0)),
                    float(pose_msg.get("theta", 0.0)),
                )
                self._last_pose = pose

                # 2. 读取障碍物直方图（从 JSON dict 中提取并转换）
                if self._use_depth:
                    obstacle_msg = self._obstacle_sub.read()
                    histogram = self._extract_histogram(obstacle_msg)
                else:
                    histogram = None

                # 3. 检查是否到达最终目标
                dist_to_goal = math.hypot(
                    pose[0] - self._goal[0], pose[1] - self._goal[1]
                )
                if dist_to_goal < self._arrival_threshold:
                    if not self._reached_reported:
                        logger.info(f"已到达目标附近！距离={dist_to_goal:.3f}m")
                        self._reached_reported = True
                        self._state = "REACHED"
                    self._send_command(0.0, 0.0)
                    self._prompt_next_goal()
                    continue
                else:
                    self._reached_reported = False

                # 4. 重规划检查
                if self._state in ("FAILED", "PLANNING") or self._should_replan(pose):
                    if not self._plan():
                        logger.debug("规划失败，原地旋转等待...")
                        self._send_command(0.0, 0.5)
                        self._sleep_remaining(t0)
                        continue

                # 5. 选择局部目标（前瞻点）
                if not self._global_path:
                    self._sleep_remaining(t0)
                    continue

                local_goal = self._select_local_goal(pose, self._global_path)

                # 6. 转换到机器人坐标系
                goal_x, goal_y = world_to_robot2(local_goal, pose)

                # 7. VFH 局部规划
                vx, vz = self._local_planner.plan(
                    obstacles=histogram if histogram is not None else np.array([]),
                    goal_x=goal_x,
                    goal_y=goal_y,
                    current_vx=pose_msg.get("vx", 0.0),
                    current_vz=pose_msg.get("vz", 0.0),
                )

                # 8. 发送底盘命令
                self._send_command(vx, vz)

                blocked_count = (
                    int(np.sum(histogram < self._local_planner.config.safety_distance_m))
                    if histogram is not None
                    else 0
                )
                logger.debug(
                    f"pos=({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f}) "
                    f"local_goal=({local_goal[0]:.2f},{local_goal[1]:.2f}) "
                    f"robot_goal=({goal_x:.2f},{goal_y:.2f}) "
                    f"cmd=({vx:.2f},{vz:.2f}) blocked={blocked_count}"
                )

                # 9. 帧率控制
                self._sleep_remaining(t0)

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
        self._send_command(0.0, 0.0)
        self._slam_pose_sub.close()
        self._slam_map_sub.close()
        if self._obstacle_sub is not None:
            self._obstacle_sub.close()
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
    # 规划
    # ------------------------------------------------------------------
    def _plan(self) -> bool:
        """执行全局路径规划。

        Returns:
            规划是否成功
        """
        t_plan_start = time.perf_counter()

        pose_msg = self._slam_pose_sub.read()
        if pose_msg is None:
            logger.warning("尚未收到SLAM位姿，无法规划")
            self._state = "PLANNING"
            return False

        meta, map_bytes = self._slam_map_sub.read()
        if meta is None or map_bytes is None:
            logger.warning("尚未收到SLAM地图，无法规划")
            self._state = "PLANNING"
            return False

        # 构建栅格地图
        grid = self._build_grid(meta, map_bytes)
        if grid is None:
            self._state = "FAILED"
            return False

        self._grid = grid

        # 膨胀障碍物
        if self._inflation_radius > 0:
            grid.inflate_obstacles(self._inflation_radius)

        start = (float(pose_msg["x"]), float(pose_msg["y"]))
        goal = self._goal

        # 清除起点周围的障碍，避免机器人自身或近距离噪声导致规划失败
        self._clear_robot_footprint(grid, start)

        planner = AStarPlanner(
            grid, allow_diagonal=True, obstacle_threshold=COST_OCCUPIED
        )
        path = planner.plan_with_simplification(start, goal)

        t_plan_end = time.perf_counter()

        if path is None:
            logger.warning(
                f"A* 全局规划失败（耗时 {(t_plan_end - t_plan_start)*1000:.1f}ms）"
            )
            self._global_path = None
            self._state = "FAILED"
            return False

        self._global_path = path
        self._state = "FOLLOWING"
        self._last_replan_time = time.time()
        logger.info(
            f"A* 规划成功，路径点={len(path)}，"
            f"耗时 {(t_plan_end - t_plan_start)*1000:.1f}ms"
        )
        return True

    def _build_grid(
        self, meta: dict, map_bytes: bytes
    ) -> Optional[OccupancyGrid]:
        """将SLAM地图字节转换为OccupancyGrid。

        BreezySLAM getmap 的填充顺序与 OccupancyGrid 的 gy 方向一致
        （arr[0] 在顶部，对应世界 y 最小/下方），无需翻转。
        """
        size_pixels = meta.get("size_pixels", 800)
        size_meters = meta.get("size_meters", 10.0)

        expected_len = size_pixels * size_pixels
        if len(map_bytes) < expected_len:
            logger.warning(
                f"地图字节长度不足: {len(map_bytes)} < {expected_len}"
            )
            return None

        resolution = size_meters / size_pixels
        origin = (-size_meters / 2.0, -size_meters / 2.0)

        grid = OccupancyGrid(
            size_pixels, size_pixels, resolution=resolution, origin=origin
        )

        arr = np.frombuffer(map_bytes, dtype=np.uint8).reshape(
            (size_pixels, size_pixels)
        )

        # BreezySLAM getmap 值与标准占据栅格相反：
        #   255 = 确定空闲, 127 = 未知, 0 = 确定占据
        #   值越大越空闲，值越小越占据
        grid.data[arr >= 200] = COST_FREE
        grid.data[(arr >= 50) & (arr < 200)] = COST_UNKNOWN
        grid.data[arr < 50] = COST_LETHAL

        return grid

    def _clear_robot_footprint(
        self,
        grid: OccupancyGrid,
        pose: Tuple[float, float],
        radius_m: float = 0.35,
    ) -> None:
        """将机器人位置周围的小区域强制设为可通行。

        避免 SLAM 将机器人自身/底盘误标记为障碍，或膨胀后覆盖起点
        导致规划失败。
        """
        px, py = pose
        gx, gy = grid.world_to_grid(px, py)
        cost_before = grid.get_cost(gx, gy)
        grid.set_circle_world(px, py, radius_m, COST_FREE)
        cost_after = grid.get_cost(gx, gy)
        if cost_before >= COST_OCCUPIED:
            logger.info(
                f"清除机器人足迹: ({px:.2f},{py:.2f}) -> grid({gx},{gy}), "
                f"cost {cost_before} -> {cost_after}, 半径={radius_m}m"
            )

    # ------------------------------------------------------------------
    # 路径跟踪
    # ------------------------------------------------------------------
    def _select_local_goal(
        self,
        pose: Tuple[float, float, float],
        path: List[Tuple[float, float]],
    ) -> Tuple[float, float]:
        """在全局路径上选择前瞻目标点。

        找到路径上距离机器人最近的点，然后向前搜索 lookahead_distance
        的路径点作为局部目标，使跟踪更平滑。
        """
        x, y, _ = pose

        # 找到最近点索引
        min_dist = float("inf")
        closest_idx = 0
        for i, (px, py) in enumerate(path):
            d = math.hypot(px - x, py - y)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        # 从最近点向前搜索 lookahead 距离
        local_goal = path[-1]
        for i in range(closest_idx, len(path)):
            px, py = path[i]
            d = math.hypot(px - x, py - y)
            if d >= self._lookahead:
                local_goal = (px, py)
                break

        return local_goal

    def _should_replan(self, pose: Tuple[float, float, float]) -> bool:
        """判断是否需要重新规划。"""
        # 定时重规划
        if time.time() - self._last_replan_time > self._replan_interval:
            return True

        path = self._global_path
        if not path:
            return True

        # 偏离路径太远
        x, y, _ = pose
        min_dist = min(math.hypot(px - x, py - y) for px, py in path)
        if min_dist > self._max_deviation:
            logger.info(f"偏离全局路径 {min_dist:.2f}m，触发重规划")
            return True

        return False

    # ------------------------------------------------------------------
    # 交互与底盘
    # ------------------------------------------------------------------
    def _prompt_next_goal(self) -> None:
        """到达后提示用户输入下一个目标点。"""
        try:
            next_goal = input("请输入下一个目标点 (x y)，或 'exit' 退出: ")
        except EOFError:
            time.sleep(0.5)
            return

        if next_goal.strip().lower() == "exit":
            logger.info("用户请求退出，停止应用")
            self._running = False
            return

        try:
            x_str, y_str = next_goal.strip().split()
            x, y = float(x_str), float(y_str)
            self.set_goal(x, y)
        except Exception as e:
            logger.warning(f"无效输入 '{next_goal}'，请重新输入。错误: {e}")

    def _send_command(self, vx: float, vz: float) -> None:
        """通过仲裁器客户端发送速度命令。"""
        try:
            response = self._chassis.send_command(
                vx, 0.0, vz, source="auto", priority=3
            )
            if response is None:
                logger.warning("底盘命令发送失败（无响应）")
        except Exception as e:
            logger.warning(f"底盘命令发送异常: {e}")

    def _sleep_remaining(self, t0: float) -> None:
        """帧率控制。"""
        elapsed = time.perf_counter() - t0
        rem = self._control_interval - elapsed
        if rem > 0:
            time.sleep(rem)


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
        "--lookahead", type=float, default=0.4, help="前瞻距离（米）"
    )
    parser.add_argument(
        "--replan", type=float, default=3.0, help="重规划间隔（秒）"
    )
    parser.add_argument(
        "--inflation", type=float, default=0.2, help="障碍物膨胀半径（米）"
    )
    parser.add_argument(
        "--use-depth",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="是否使用深度图做局部避障（默认开启，--no-use-depth 可关闭）",
    )
    args = parser.parse_args()

    app = NavigationApp(
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        slam_pose_addr=args.slam_pose,
        slam_map_addr=args.slam_map,
        obstacle_addr=args.obstacle,
        chassis_addr=args.chassis,
        control_rate=args.rate,
        arrival_threshold_m=args.threshold,
        lookahead_distance_m=args.lookahead,
        replan_interval_s=args.replan,
        inflation_radius_m=args.inflation,
        use_depth=args.use_depth,
    )
    app.start()


if __name__ == "__main__":
    main()
