# -*- coding: utf-8 -*-
"""OdomService - 轮式里程计服务

订阅 ChassisService 发布的底盘状态（速度指令 + 实际轮速），
通过积分推算机器人位姿 (x, y, yaw)，并通过 ZeroMQ PUB 发布。

优先使用实际轮速计算的速度（actual_vx/vy/vz）进行积分；
若实际轮速读取失败，则回退到命令速度（vx/vy/vz）。

注意：舵机速度到物理速度的转换参数已根据数据手册确认（3250 = 47.45 RPM）。
"""
from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import numpy as np
import zmq

from common.logging import get_logger
from common.zmq_helper import create_socket

logger = get_logger(__name__)

DEFAULT_CHASSIS_STATE_ADDR = "tcp://localhost:5558"
DEFAULT_ODOM_PUB_ADDR = "tcp://*:5559"
DEFAULT_ODOM_CMD_ADDR = "tcp://*:5567"


class OdomService:
    """轮式里程计服务。

    - SUB ChassisService 状态发布
    - 对速度做数值积分，推算位姿
    - PUB 里程计数据 (x, y, yaw, vx, vy, vz)
    """

    def __init__(
        self,
        chassis_state_addr: str = DEFAULT_CHASSIS_STATE_ADDR,
        odom_pub_addr: str = DEFAULT_ODOM_PUB_ADDR,
        odom_cmd_addr: str = DEFAULT_ODOM_CMD_ADDR,
        publish_rate: float = 50.0,
    ):
        self.chassis_state_addr = chassis_state_addr
        self.odom_pub_addr = odom_pub_addr
        self.odom_cmd_addr = odom_cmd_addr
        self.publish_interval = 1.0 / publish_rate if publish_rate > 0 else 0.02

        # SUB socket 订阅底盘状态
        self._sub_socket = create_socket(zmq.SUB, bind=False, address=self.chassis_state_addr)
        self._sub_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub_socket.setsockopt(zmq.RCVTIMEO, 1000)
        self._sub_socket.setsockopt(zmq.CONFLATE, 1)  # 只保留最新状态
        logger.info(f"OdomService SUB connected to {self.chassis_state_addr}")

        # PUB socket 发布里程计
        self._pub_socket = create_socket(zmq.PUB, bind=True, address=self.odom_pub_addr)
        logger.info(f"OdomService PUB odom={self.odom_pub_addr}")

        # REP socket 接收命令（重置位姿等）
        self._cmd_socket = create_socket(zmq.REP, bind=True, address=self.odom_cmd_addr)
        logger.info(f"OdomService REP cmd={self.odom_cmd_addr}")

        # 位姿状态（世界坐标系）
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 上一时刻的速度和时间戳
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_vz = 0.0
        self._last_time: Optional[float] = None

        # 运行标志
        self._running = False

        # 性能统计
        self._pub_count = 0
        self._last_log_time = time.time()

    def start(self) -> None:
        """启动里程计服务主循环。"""
        self._running = True
        logger.info(f"OdomService 已启动，发布频率={1/self.publish_interval:.0f} Hz")

        # Poller 同时监听 SUB 和 REP
        poller = zmq.Poller()
        poller.register(self._sub_socket, zmq.POLLIN)
        poller.register(self._cmd_socket, zmq.POLLIN)

        try:
            while self._running:
                t0 = time.perf_counter()

                # 0. 处理命令请求（非阻塞）
                socks = dict(poller.poll(0))
                if self._cmd_socket in socks:
                    self._handle_command()

                # 1. 接收最新底盘状态
                state = self._recv_state()
                now = time.time()

                if state is not None:
                    # 优先使用实际轮速计算的速度，否则回退到命令速度
                    if "actual_vx" in state and "actual_vy" in state and "actual_vz" in state:
                        vx = float(state["actual_vx"])
                        vy = float(state["actual_vy"])
                        vz = float(state["actual_vz"])
                        using_actual = True
                    else:
                        vx = float(state.get("vx", 0.0))
                        vy = float(state.get("vy", 0.0))
                        vz = float(state.get("vz", 0.0))
                        using_actual = False

                    # 2. 积分推算位姿
                    if self._last_time is not None:
                        dt = now - self._last_time
                        if dt > 0 and dt < 1.0:  # 忽略异常大的时间间隔
                            self._integrate(vx, vy, vz, dt)

                    self._last_vx = vx
                    self._last_vy = vy
                    self._last_vz = vz
                    self._last_time = now
                else:
                    # 没有收到新状态，但时间仍在流逝，用上一时刻速度继续积分
                    if self._last_time is not None:
                        dt = now - self._last_time
                        if dt > 0 and dt < 1.0:
                            self._integrate(self._last_vx, self._last_vy, self._last_vz, dt)
                        self._last_time = now

                # 3. 发布里程计
                self._publish_odom()
                self._pub_count += 1

                # 4. 性能统计
                self._update_stats()

                # 5. 帧率控制
                elapsed = time.perf_counter() - t0
                rem = self.publish_interval - elapsed
                if rem > 0:
                    time.sleep(rem)

        except KeyboardInterrupt:
            logger.info("OdomService 被用户中断")
        except Exception as e:
            logger.error(f"OdomService 异常: {e}")
            import traceback
            logger.error(traceback.format_exc())
        finally:
            self.stop()

    def _recv_state(self) -> Optional[dict]:
        """从 ChassisService 接收最新状态。"""
        try:
            state = self._sub_socket.recv_json(flags=zmq.NOBLOCK)
            return state
        except zmq.Again:
            return None
        except Exception as e:
            logger.warning(f"接收底盘状态失败: {e}")
            return None

    def _integrate(self, vx: float, vy: float, vz: float, dt: float) -> None:
        """对速度进行数值积分，更新位姿。

        坐标系：
        - vx, vy 是机器人坐标系下的速度（前进/左移）
        - 积分时转换为世界坐标系
        """
        # 中值积分（更稳定）
        mid_yaw = self.yaw + vz * dt * 0.5

        # 机器人坐标系 -> 世界坐标系
        dx_world = vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)
        dy_world = vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)

        self.x += dx_world * dt
        self.y += dy_world * dt
        self.yaw += vz * dt

        # 规范化 yaw 到 [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    def _publish_odom(self) -> None:
        """发布里程计数据。"""
        odom = {
            "x": float(round(self.x, 4)),
            "y": float(round(self.y, 4)),
            "yaw": float(round(self.yaw, 4)),
            "vx": float(round(self._last_vx, 4)),
            "vy": float(round(self._last_vy, 4)),
            "vz": float(round(self._last_vz, 4)),
            "timestamp": time.time(),
        }
        try:
            self._pub_socket.send_json(odom, flags=zmq.NOBLOCK)
        except zmq.Again:
            pass
        except Exception as e:
            logger.warning(f"发布里程计失败: {e}")

    def _update_stats(self) -> None:
        now = time.time()
        if now - self._last_log_time >= 5.0:
            fps = self._pub_count / (now - self._last_log_time)
            logger.info(
                f"OdomService 性能: {fps:.1f} Hz, "
                f"位姿=({self.x:.3f}, {self.y:.3f}, {self.yaw:.3f})"
            )
            self._pub_count = 0
            self._last_log_time = now

    def reset_pose(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """重置里程计位姿。"""
        self.x = x
        self.y = y
        self.yaw = yaw
        self._last_time = None
        logger.info(f"里程计已重置: ({x}, {y}, {yaw})")

    def _handle_command(self) -> None:
        """处理 REP 命令请求。"""
        try:
            req = self._cmd_socket.recv_json(flags=zmq.NOBLOCK)
            cmd = req.get("cmd", "")
            if cmd == "reset_pose":
                x = req.get("x", 0.0)
                y = req.get("y", 0.0)
                yaw = req.get("yaw", 0.0)
                self.reset_pose(x, y, yaw)
                self._cmd_socket.send_json({"success": True, "message": f"已重置为 ({x}, {y}, {yaw})"})
            else:
                self._cmd_socket.send_json({"success": False, "message": f"未知命令: {cmd}"})
        except zmq.Again:
            pass
        except Exception as e:
            logger.warning(f"处理命令失败: {e}")
            try:
                self._cmd_socket.send_json({"success": False, "message": str(e)})
            except Exception:
                pass

    def stop(self) -> None:
        """停止服务并释放资源。"""
        self._running = False
        if self._sub_socket:
            self._sub_socket.close()
        if self._pub_socket:
            self._pub_socket.close()
        if self._cmd_socket:
            self._cmd_socket.close()
        logger.info("OdomService 已停止")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="HomeBot 轮式里程计服务")
    parser.add_argument("--chassis-state", default=DEFAULT_CHASSIS_STATE_ADDR, help="底盘状态 SUB 地址")
    parser.add_argument("--odom-pub", default=DEFAULT_ODOM_PUB_ADDR, help="里程计 PUB 地址")
    parser.add_argument("--odom-cmd", default=DEFAULT_ODOM_CMD_ADDR, help="里程计命令 REP 地址")
    parser.add_argument("--rate", type=float, default=50.0, help="发布频率 Hz")
    args = parser.parse_args()

    service = OdomService(
        chassis_state_addr=args.chassis_state,
        odom_pub_addr=args.odom_pub,
        odom_cmd_addr=args.odom_cmd,
        publish_rate=args.rate,
    )
    service.start()


if __name__ == "__main__":
    main()
