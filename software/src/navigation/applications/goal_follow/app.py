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
import time
from typing import Optional, Tuple

import numpy as np
import zmq

from common.logging import get_logger
from common.zmq_helper import create_socket
from navigation.perception.obstacle_detector import DepthObstacle
from navigation.planning.local_planner import LocalPlannerConfig, VFHLocalPlanner

logger = get_logger(__name__)

DEFAULT_ODOM_ADDR = "tcp://localhost:5559"
DEFAULT_OBSTACLE_ADDR = "tcp://localhost:5562"
DEFAULT_CHASSIS_ADDR = "tcp://127.0.0.1:5556"


class OdomSubscriber:
    """里程计订阅者 - 后台线程保持最新位姿"""

    def __init__(self, sub_addr: str = DEFAULT_ODOM_ADDR):
        self._sub = create_socket(zmq.SUB, bind=False, address=sub_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.setsockopt(zmq.RCVTIMEO, 1000)
        self._sub.setsockopt(zmq.CONFLATE, 1)
        self._latest_odom: Optional[dict] = None

    def read(self) -> Optional[dict]:
        try:
            odom = self._sub.recv_json(flags=zmq.NOBLOCK)
            self._latest_odom = odom
            return odom
        except zmq.Again:
            return self._latest_odom
        except Exception as e:
            logger.warning(f"读取里程计失败: {e}")
            return self._latest_odom

    def close(self):
        self._sub.close()


class ObstacleSubscriber:
    """障碍物订阅者 - 后台线程保持最新障碍物"""

    def __init__(self, sub_addr: str = DEFAULT_OBSTACLE_ADDR):
        self._sub = create_socket(zmq.SUB, bind=False, address=sub_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.setsockopt(zmq.RCVTIMEO, 1000)
        self._sub.setsockopt(zmq.CONFLATE, 1)
        self._latest_obstacles: Optional[list] = None

    def read(self) -> Optional[list]:
        try:
            parts = self._sub.recv_multipart(flags=zmq.NOBLOCK)
            if len(parts) >= 2:
                data = json.loads(parts[1].decode("utf-8"))
                obs_list = data.get("obstacles", [])
                self._latest_obstacles = obs_list
                return obs_list
        except zmq.Again:
            return self._latest_obstacles
        except Exception as e:
            logger.warning(f"读取障碍物失败: {e}")
            return self._latest_obstacles

    def close(self):
        self._sub.close()


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
        arrival_threshold_m: float = 0.15,
        control_rate: float = 10.0,
    ):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.arrival_threshold = arrival_threshold_m
        self.control_interval = 1.0 / control_rate if control_rate > 0 else 0.1

        # 数据订阅
        self._odom_sub = OdomSubscriber(odom_addr)
        self._obstacle_sub = ObstacleSubscriber(obstacle_addr)

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

                # 1. 读取最新位姿和障碍物
                odom = self._odom_sub.read()
                obstacles_raw = self._obstacle_sub.read()

                if odom is None:
                    logger.warning("尚未收到里程计数据，等待中...")
                    time.sleep(0.1)
                    continue

                # 2. 计算相对目标向量（里程计坐标系下目标始终不变，因为我们希望走到相对位置）
                # 但 odom 积分的是世界坐标，如果从 (0,0,0) 开始，目标就是 (goal_x, goal_y)
                rx = odom.get("x", 0.0)
                ry = odom.get("y", 0.0)
                yaw = odom.get("yaw", 0.0)

                # 目标在世界坐标系中就是 (goal_x, goal_y)
                # 但机器人坐标系下的相对目标方向用于 VFH：
                dx_world = self.goal_x - rx
                dy_world = self.goal_y - ry

                # 转换到机器人坐标系
                dx_robot = dx_world * math.cos(-yaw) - dy_world * math.sin(-yaw)
                dy_robot = dx_world * math.sin(-yaw) + dy_world * math.cos(-yaw)

                distance = math.hypot(dx_robot, dy_robot)

                # 3. 检查是否到达
                if distance < self.arrival_threshold:
                    if not self._reached:
                        logger.info(f"已到达目标附近！距离={distance:.3f}m")
                        self._reached = True
                    self._send_command(0.0, 0.0)
                    time.sleep(self.control_interval)
                    continue
                else:
                    self._reached = False

                # 4. 解析障碍物
                obstacles = self._parse_obstacles(obstacles_raw or [])

                # 5. 局部规划
                vx, vz = self._planner.plan(
                    obstacles=obstacles,
                    goal_x=dx_robot,
                    goal_y=dy_robot,
                    current_vx=odom.get("vx", 0.0),
                    current_vz=odom.get("vz", 0.0),
                )

                # 6. 发送底盘命令
                self._send_command(vx, vz)
                logger.debug(
                    f"pos=({rx:.2f},{ry:.2f},{yaw:.2f}) "
                    f"goal_rel=({dx_robot:.2f},{dy_robot:.2f}) "
                    f"cmd=({vx:.2f},{vz:.2f}) obs={len(obstacles)}"
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

    def _parse_obstacles(self, raw_list: list) -> list:
        """将原始字典列表解析为 DepthObstacle 对象。"""
        obstacles = []
        for item in raw_list:
            try:
                obstacles.append(DepthObstacle(
                    x=float(item.get("x", 0)),
                    y=float(item.get("y", 0)),
                    z=float(item.get("z", 0)),
                    width=float(item.get("width", 0.1)),
                    height=float(item.get("height", 0.1)),
                    confidence=float(item.get("confidence", 1)),
                ))
            except Exception:
                continue
        return obstacles

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
    parser.add_argument("--goal-x", type=float, default=1.0, help="目标点 X（米，机器人右侧为正）")
    parser.add_argument("--goal-y", type=float, default=0.0, help="目标点 Y（米，机器人前方为正）")
    parser.add_argument("--odom", default=DEFAULT_ODOM_ADDR, help="里程计 SUB 地址")
    parser.add_argument("--obstacle", default=DEFAULT_OBSTACLE_ADDR, help="障碍物 SUB 地址")
    parser.add_argument("--chassis", default=DEFAULT_CHASSIS_ADDR, help="底盘服务地址")
    parser.add_argument("--rate", type=float, default=10.0, help="控制频率 Hz")
    parser.add_argument("--threshold", type=float, default=0.15, help="到达阈值（米）")
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
