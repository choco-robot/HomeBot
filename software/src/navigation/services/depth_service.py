# -*- coding: utf-8 -*-
"""DepthService - 深度估计与障碍物检测服务

订阅 VisionService 的图像流，运行深度估计和障碍物检测，
通过 ZeroMQ PUB 发布伪彩色深度图和障碍物信息。
"""
from __future__ import annotations

import json
import time
from threading import Lock, Thread
from typing import Optional

import cv2
import numpy as np
import zmq

from common.logging import get_logger
from common.zmq_helper import create_socket

logger = get_logger(__name__)

DEFAULT_VISION_ADDR = "tcp://localhost:5560"
DEFAULT_DEPTH_PUB_ADDR = "tcp://*:5561"
DEFAULT_OBSTACLE_PUB_ADDR = "tcp://*:5562"


class DepthService:
    """深度感知服务。

    - SUB VisionService 图像流
    - 运行深度估计（MiDaS ONNX 或 Fake 回退）
    - 可选：运行障碍物检测
    - PUB 伪彩色深度图 JPEG
    - PUB 障碍物信息 JSON
    """

    def __init__(
        self,
        vision_addr: str = DEFAULT_VISION_ADDR,
        depth_pub_addr: str = DEFAULT_DEPTH_PUB_ADDR,
        obstacle_pub_addr: str = DEFAULT_OBSTACLE_PUB_ADDR,
        model_path: Optional[str] = None,
        inference_size: tuple = (256, 256),
        enable_obstacle_detection: bool = True,
        publish_fps: int = 10,
    ):
        self.vision_addr = vision_addr
        self.depth_pub_addr = depth_pub_addr
        self.obstacle_pub_addr = obstacle_pub_addr
        self.publish_fps = publish_fps
        self._running = False

        # 创建 SUB socket 连接 VisionService
        self._sub_socket = create_socket(zmq.SUB, bind=False, address=self.vision_addr)
        self._sub_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub_socket.setsockopt(zmq.RCVTIMEO, 1000)
        self._sub_socket.setsockopt(zmq.CONFLATE, 1)  # 只保留最新帧
        logger.info(f"DepthService SUB connected to {self.vision_addr}")

        # 创建 PUB sockets
        self._depth_pub = create_socket(zmq.PUB, bind=True, address=self.depth_pub_addr)
        self._obstacle_pub = create_socket(zmq.PUB, bind=True, address=self.obstacle_pub_addr)
        logger.info(f"DepthService PUB depth={self.depth_pub_addr}, obstacle={self.obstacle_pub_addr}")

        # 初始化深度估计器
        from navigation.perception.depth_estimator import create_depth_estimator
        self._estimator = create_depth_estimator(
            model_path=model_path,
            inference_size=inference_size,
            allow_fake=True,
        )
        logger.info(f"深度估计器类型: {type(self._estimator).__name__}")

        # 初始化障碍物检测器
        self._enable_obstacle = enable_obstacle_detection
        if self._enable_obstacle:
            from navigation.perception.obstacle_detector import DepthObstacleDetector
            self._detector = DepthObstacleDetector()
        else:
            self._detector = None

        # 性能统计
        self._frame_count = 0
        self._last_log_time = time.time()
        self._avg_inference_ms = 0.0

        # 双线程：后台接收 + 前台处理
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_id: int = -1
        self._frame_lock = Lock()
        self._recv_thread: Optional[Thread] = None

    def start(self) -> None:
        """启动深度估计服务主循环。"""
        self._running = True
        logger.info(
            f"DepthService 已启动（双线程模式：接收线程+处理线程），"
            f"目标发布帧率≤{self.publish_fps} FPS"
        )

        # 启动后台接收线程
        self._recv_thread = Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

        tgt_interval = 1.0 / self.publish_fps if self.publish_fps > 0 else 0
        last_processed_id = -1

        try:
            while self._running:
                t0 = time.perf_counter()

                # 1. 从内存中获取最新帧（非阻塞）
                with self._frame_lock:
                    frame = self._latest_frame.copy() if self._latest_frame is not None else None
                    frame_id = self._latest_frame_id

                if frame is None:
                    time.sleep(0.01)
                    continue

                # 如果这帧已经处理过，跳过（避免重复计算同一帧）
                if frame_id == last_processed_id:
                    time.sleep(0.005)
                    continue
                last_processed_id = frame_id

                # 2. 深度估计（始终处理内存中的最新帧）
                t_inf0 = time.perf_counter()
                depth = self._estimator.estimate_safe(frame)
                t_inf = (time.perf_counter() - t_inf0) * 1000

                if depth is None:
                    logger.warning("深度估计返回空，跳过此帧")
                    continue

                # 3. 障碍物检测
                obstacles = []
                if self._enable_obstacle and self._detector is not None:
                    obstacles = self._detector.detect(depth, frame)

                # 4. 发布深度图（伪彩色 JPEG）
                depth_color = self._estimator.colorize(depth)
                ret, jpeg = cv2.imencode(".jpg", depth_color)
                if ret:
                    self._depth_pub.send_multipart([
                        str(frame_id).encode(),
                        jpeg.tobytes(),
                    ])

                # 5. 发布障碍物信息
                if self._enable_obstacle:
                    obs_dicts = [self._obstacle_to_dict(o) for o in obstacles]
                    self._obstacle_pub.send_multipart([
                        str(frame_id).encode(),
                        json.dumps({
                            "obstacles": obs_dicts,
                            "inference_ms": float(round(t_inf, 2)),
                            "estimator_type": type(self._estimator).__name__,
                        }, ensure_ascii=False).encode("utf-8"),
                    ])

                # 6. 性能统计
                self._update_stats(t_inf)

                # 7. 帧率控制（最大发布帧率限制）
                elapsed = time.perf_counter() - t0
                rem = tgt_interval - elapsed
                if rem > 0:
                    time.sleep(rem)

        except KeyboardInterrupt:
            logger.info("DepthService 被用户中断")
        except Exception as e:
            logger.error(f"DepthService 异常: {e}")
            import traceback
            logger.error(traceback.format_exc())
        finally:
            self.stop()

    def _receive_loop(self) -> None:
        """后台线程：持续接收 VisionService 图像，始终只保留最新帧到内存。"""
        logger.info("DepthService 图像接收线程已启动")
        while self._running:
            try:
                parts = self._sub_socket.recv_multipart()
                if len(parts) >= 2:
                    frame_id = int(parts[0].decode())
                    buf = np.frombuffer(parts[1], dtype=np.uint8)
                    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self._frame_lock:
                            self._latest_frame = frame
                            self._latest_frame_id = frame_id
            except zmq.Again:
                # 超时，继续循环
                continue
            except Exception as e:
                logger.warning(f"接收图像失败: {e}")
                time.sleep(0.01)

    def _obstacle_to_dict(self, obs) -> dict:
        return {
            "x": float(round(obs.x, 3)),
            "y": float(round(obs.y, 3)),
            "z": float(round(obs.z, 3)),
            "width": float(round(obs.width, 3)),
            "height": float(round(obs.height, 3)),
            "confidence": float(round(obs.confidence, 3)),
        }

    def _update_stats(self, inference_ms: float) -> None:
        """更新并打印性能统计。"""
        alpha = 0.1
        self._avg_inference_ms = (1 - alpha) * self._avg_inference_ms + alpha * inference_ms
        self._frame_count += 1

        now = time.time()
        if now - self._last_log_time >= 5.0:
            fps = self._frame_count / (now - self._last_log_time)
            logger.info(
                f"DepthService 性能: {fps:.1f} FPS, "
                f"平均推理耗时={self._avg_inference_ms:.1f}ms, "
                f"估计器={type(self._estimator).__name__}"
            )
            self._frame_count = 0
            self._last_log_time = now

    def stop(self) -> None:
        """停止服务并释放资源。"""
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        if self._sub_socket:
            self._sub_socket.close()
        if self._depth_pub:
            self._depth_pub.close()
        if self._obstacle_pub:
            self._obstacle_pub.close()
        logger.info("DepthService 已停止")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="HomeBot 深度感知服务")
    parser.add_argument("--vision", default=DEFAULT_VISION_ADDR, help="VisionService 地址")
    parser.add_argument("--depth-pub", default=DEFAULT_DEPTH_PUB_ADDR, help="深度图 PUB 地址")
    parser.add_argument("--obstacle-pub", default=DEFAULT_OBSTACLE_PUB_ADDR, help="障碍物 PUB 地址")
    parser.add_argument("--model", default=None, help="ONNX 模型路径")
    parser.add_argument("--inference-size", type=int, default=256, help="推理分辨率")
    parser.add_argument("--fps", type=int, default=10, help="发布帧率")
    parser.add_argument("--no-obstacle", action="store_true", help="禁用障碍物检测")
    args = parser.parse_args()

    service = DepthService(
        vision_addr=args.vision,
        depth_pub_addr=args.depth_pub,
        obstacle_pub_addr=args.obstacle_pub,
        model_path=args.model,
        inference_size=(args.inference_size, args.inference_size),
        enable_obstacle_detection=not args.no_obstacle,
        publish_fps=args.fps,
    )
    service.start()


if __name__ == "__main__":
    main()
