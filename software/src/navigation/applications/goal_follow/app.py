# -*- coding: utf-8 -*-
"""GoalFollowApp - 目标点跟随应用

给定一个相对目标点 (x, y)，机器人自动朝目标移动，
结合深度障碍物检测实现局部避障。

数据流：
- SUB /odom/pose       -> 获取当前位姿
- SUB /depth/obstacles -> 获取障碍物信息
- VFH Local Planner    -> 计算局部速度指令
- REQ ChassisService   -> 发送底盘命令
"""
from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional, Tuple

import numpy as np
import zmq

from common.logging import get_logger
from common.transform import world_to_robot2
from common.zmq_subscriber import ZMQJsonSubscriber, ZMQMultipartJsonSubscriber
from common.zmq_helper import create_socket
from navigation.perception.obstacle_detector import DepthObstacle
from navigation.planning.local_planner import LocalPlannerConfig, VFHLocalPlanner

logger = get_logger(__name__)

DEFAULT_ODOM_ADDR = "tcp://localhost:5559"
DEFAULT_OBSTACLE_ADDR = "tcp://localhost:5562"
DEFAULT_CHASSIS_ADDR = "tcp://127.0.0.1:5556"


class OdomSubscriber(ZMQJsonSubscriber):
    """里程计订阅者"""

    def __init__(self, sub_addr: str = DEFAULT_ODOM_ADDR):
        super().__init__(sub_addr, required_keys=("x", "y", "yaw"))


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
                        hist = np.array([
                            np.inf if v is None else float(v)
                            for v in hist_list
                        ], dtype=np.float32)
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
            return self._latest_histogram.copy() if self._latest_histogram is not None else None

    def close(self) -> None:
        super().close()


class GoalFollowApp:
    """目标点跟随应用。"""

    def __init__(
        self,
        goal_x: float = 1.0,
        goal_y: float = 0.0,
        odom_addr: str = DEFAULT_ODOM_ADDR,
        obstacle_addr: str = DEFAULT_OBSTACLE_ADDR,
        chassis_addr: str = DEFAULT_CHASSIS_ADDR,
        planner_config: Optional[LocalPlannerConfig] = None,
        arrival_threshold_m: float = 0.05,
        control_rate: float = 10.0,
    ):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.arrival_threshold = arrival_threshold_m
        self.control_interval = 1.0 / control_rate if control_rate > 0 else 0.1

        # 数据订阅
        self._odom_sub = OdomSubscriber(odom_addr)
        self._obstacle_sub = HistogramSubscriber(obstacle_addr)

        # 底盘客户端
        from services.motion_service.chassis_arbiter import ChassisArbiterClient
        self._chassis = ChassisArbiterClient(chassis_addr, timeout_ms=500)

        # 局部规划器
        self._planner = VFHLocalPlanner(planner_config or LocalPlannerConfig())

        # 状态
        self._running = False
        self._reached = False

    def start(self) -> None:
        """启动目标跟随循环。"""
        self._running = True
        logger.info(
            f"GoalFollowApp 启动，目标=({self.goal_x}, {self.goal_y}), "
            f"控制频率={1/self.control_interval:.0f} Hz"
        )

        try:
            while self._running:
                t0 = time.perf_counter()

                # 1. 读取最新位姿和直方图
                odom = self._odom_sub.read()
                histogram = self._obstacle_sub.read()

                if odom is None:
                    logger.warning("尚未收到里程计数据，等待中...")
                    time.sleep(0.1)
                    continue

                # 2. 读取当前位姿（世界坐标系）
                rx = odom.get("x", 0.0)
                ry = odom.get("y", 0.0)
                yaw = odom.get("yaw", 0.0)

                # 将世界坐标系下的目标点转换到机器人坐标系（底盘坐标系）
                # VFHLocalPlanner 期望 goal_x（前方为正）、goal_y（左侧为正）
                goal_x, goal_y = world_to_robot2((self.goal_x, self.goal_y), (rx, ry, yaw))
                distance = math.hypot(goal_x, goal_y)

                # 3. 检查是否到达
                if distance < self.arrival_threshold:
                    if not self._reached:
                        logger.info(f"已到达目标附近！距离={distance:.3f}m")
                        self._reached = True
                    self._send_command(0.0, 0.0)
                    time.sleep(self.control_interval)
                    # 输入下一个目标点
                    next_goal = input("请输入下一个目标点 (x y)，或 'exit' 退出: ")
                    if next_goal.strip().lower() == "exit":
                        logger.info("用户请求退出，停止应用")
                        break
                    try:                        
                        x_str, y_str = next_goal.strip().split()
                        x, y = float(x_str), float(y_str)
                        self.set_goal(x, y)
                    except Exception as e:
                        logger.warning(f"无效输入 '{next_goal}'，请重新输入。错误: {e}")
                    continue
                else:
                    self._reached = False

                # 4. 局部规划（直接传入距离直方图）
                vx, vz = self._planner.plan(
                    obstacles=histogram if histogram is not None else np.array([]),
                    goal_x=goal_x,
                    goal_y=goal_y,
                    current_vx=odom.get("vx", 0.0),
                    current_vz=odom.get("vz", 0.0),
                )

                # 5. 发送底盘命令
                self._send_command(vx, vz)
                blocked_count = int(np.sum(histogram < 0.5)) if histogram is not None else 0
                logger.debug(
                    f"pos=({rx:.2f},{ry:.2f},{yaw:.2f}) "
                    f"goal_robot=({goal_x:.2f},{goal_y:.2f}) "
                    f"cmd=({vx:.2f},{vz:.2f}) blocked={blocked_count}"
                )

                # 7. 帧率控制
                elapsed = time.perf_counter() - t0
                rem = self.control_interval - elapsed
                if rem > 0:
                    time.sleep(rem)

        except KeyboardInterrupt:
            logger.info("GoalFollowApp 被用户中断")
        except Exception as e:
            logger.error(f"GoalFollowApp 异常: {e}")
            import traceback
            logger.error(traceback.format_exc())
        finally:
            self.stop()

    def _send_command(self, vx: float, vz: float) -> None:
        """通过仲裁器客户端发送速度命令。"""
        try:
            response = self._chassis.send_command(vx, 0.0, vz, source="auto", priority=3)
            if response is None:
                logger.warning("底盘命令发送失败（无响应）")
        except Exception as e:
            logger.warning(f"底盘命令发送异常: {e}")

    def stop(self) -> None:
        """停止应用并释放资源。"""
        self._running = False
        self._send_command(0.0, 0.0)
        self._odom_sub.close()
        self._obstacle_sub.close()
        self._chassis.close()
        logger.info("GoalFollowApp 已停止")

    def set_goal(self, x: float, y: float) -> None:
        """动态更新目标点。"""
        self.goal_x = x
        self.goal_y = y
        self._reached = False
        logger.info(f"目标点已更新: ({x}, {y})")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="HomeBot 目标点跟随应用")
    parser.add_argument("--goal-x", type=float, default=1.0, help="目标点 X（米，机器人前方为正）")
    parser.add_argument("--goal-y", type=float, default=0.0, help="目标点 Y（米，机器人左侧为正）")
    parser.add_argument("--odom", default=DEFAULT_ODOM_ADDR, help="里程计 SUB 地址")
    parser.add_argument("--obstacle", default=DEFAULT_OBSTACLE_ADDR, help="障碍物 SUB 地址")
    parser.add_argument("--chassis", default=DEFAULT_CHASSIS_ADDR, help="底盘服务地址")
    parser.add_argument("--rate", type=float, default=10.0, help="控制频率 Hz")
    parser.add_argument("--threshold", type=float, default=0.05, help="到达阈值（米）")
    args = parser.parse_args()

    app = GoalFollowApp(
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        odom_addr=args.odom,
        obstacle_addr=args.obstacle,
        chassis_addr=args.chassis,
        control_rate=args.rate,
        arrival_threshold_m=args.threshold,
    )
    app.start()


if __name__ == "__main__":
    main()
