# -*- coding: utf-8 -*-
"""ZeroMQ 统一订阅者模板

规范（重要）：
  - 应用层订阅者**禁止**使用 zmq.CONFLATE，必须使用后台线程持续接收 + 锁保护最新数据
  - 服务层内部消费（如 OdomService 50Hz 主循环）可使用 CONFLATE，因其本身就是持续 recv
  - 同一数据话题的订阅逻辑不要重复实现，优先使用本模块提供的基类

提供的类：
  - ZMQJsonSubscriber: 订阅 JSON 单帧消息（如 OdomService）
  - ZMQMultipartJsonSubscriber: 订阅 multipart 消息，第 2 个 frame 为 JSON（如 DepthService 障碍物）
  - ZMQMultipartImageSubscriber: 订阅 multipart 图像帧（如 VisionService）

Usage:
    from common.zmq_subscriber import ZMQJsonSubscriber

    sub = ZMQJsonSubscriber("tcp://localhost:5559", required_keys=("x", "y", "yaw"))
    data = sub.read()          # 非阻塞，线程安全
    stats = sub.get_stats()    # {"recv_count": int, "has_data": bool}
    sub.close()
"""
from __future__ import annotations

import json
import threading
import time
from typing import Any, Callable, Optional, Tuple

import numpy as np
import zmq

from common.logging import get_logger
from common.zmq_helper import create_socket

logger = get_logger(__name__)


class ZMQJsonSubscriber:
    """JSON 数据订阅者基类（后台线程持续接收）。

    特性：
      - 实例化时自动启动后台接收线程
      - read() 从内存中直接读取最新数据（非阻塞、线程安全）
      - 支持必填字段校验，过滤非法数据
      - 不使用 zmq.CONFLATE，避免消息解析异常
    """

    def __init__(
        self,
        sub_addr: str,
        required_keys: tuple[str, ...] = (),
        rcv_timeout_ms: int = 500,
    ):
        self._sub = create_socket(zmq.SUB, bind=False, address=sub_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.setsockopt(zmq.RCVTIMEO, rcv_timeout_ms)
        # 注意：不设置 CONFLATE。应用层订阅者通过后台线程来保持最新数据。
        self._required_keys = required_keys

        self._latest_data: Optional[dict] = None
        self._lock = threading.Lock()
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._recv_count = 0
        self._start_receiver()

    def _start_receiver(self) -> None:
        """启动后台接收线程。"""
        self._running = True
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

    def _receive_loop(self) -> None:
        """后台线程：持续接收 JSON 数据。"""
        while self._running:
            try:
                data = self._sub.recv_json(flags=zmq.NOBLOCK)
                if self._validate(data):
                    with self._lock:
                        self._latest_data = data
                        self._recv_count += 1
                else:
                    logger.warning(f"[{self.__class__.__name__}] 收到不符合要求的数据: {data}")
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"[{self.__class__.__name__}] 接收异常: {e}")
            time.sleep(0.001)

    def _validate(self, data: Any) -> bool:
        """校验数据格式。子类可覆盖。"""
        if not isinstance(data, dict):
            return False
        return all(k in data for k in self._required_keys)

    def read(self) -> Optional[dict]:
        """读取最新数据（线程安全，非阻塞）。"""
        with self._lock:
            return self._latest_data.copy() if self._latest_data is not None else None

    def get_stats(self) -> dict:
        """获取接收统计。"""
        with self._lock:
            return {
                "recv_count": self._recv_count,
                "has_data": self._latest_data is not None,
            }

    def close(self) -> None:
        """停止接收线程并关闭 socket。"""
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        self._sub.close()


class ZMQMultipartJsonSubscriber:
    """Multipart 消息订阅者基类，第 2 个 frame 为 JSON（后台线程持续接收）。

    适用于 DepthService 障碍物直方图等多帧协议。
    """

    def __init__(
        self,
        sub_addr: str,
        required_keys: tuple[str, ...] = (),
        json_frame_index: int = 1,
        rcv_timeout_ms: int = 500,
    ):
        self._sub = create_socket(zmq.SUB, bind=False, address=sub_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.setsockopt(zmq.RCVTIMEO, rcv_timeout_ms)
        self._required_keys = required_keys
        self._json_frame_index = json_frame_index

        self._latest_data: Optional[dict] = None
        self._lock = threading.Lock()
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._recv_count = 0
        self._start_receiver()

    def _start_receiver(self) -> None:
        self._running = True
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

    def _receive_loop(self) -> None:
        while self._running:
            try:
                parts = self._sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) > self._json_frame_index:
                    data = json.loads(parts[self._json_frame_index].decode("utf-8"))
                    if self._validate(data):
                        with self._lock:
                            self._latest_data = data
                            self._recv_count += 1
                    else:
                        logger.warning(
                            f"[{self.__class__.__name__}] 收到不符合要求的数据: {data}"
                        )
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"[{self.__class__.__name__}] 接收异常: {e}")
            time.sleep(0.001)

    def _validate(self, data: Any) -> bool:
        if not isinstance(data, dict):
            return False
        return all(k in data for k in self._required_keys)

    def read(self) -> Optional[dict]:
        with self._lock:
            return self._latest_data.copy() if self._latest_data is not None else None

    def get_stats(self) -> dict:
        with self._lock:
            return {
                "recv_count": self._recv_count,
                "has_data": self._latest_data is not None,
            }

    def close(self) -> None:
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        self._sub.close()


class ZMQMultipartImageSubscriber:
    """Multipart 图像帧订阅者基类（后台线程持续接收）。

    适用于 VisionService 等发布 [frame_id, jpeg_bytes] 的协议。
    """

    def __init__(
        self,
        sub_addr: str,
        rcv_timeout_ms: int = 500,
    ):
        self._sub = create_socket(zmq.SUB, bind=False, address=sub_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.setsockopt(zmq.RCVTIMEO, rcv_timeout_ms)

        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_id: int = -1
        self._lock = threading.Lock()
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._recv_count = 0
        self._start_receiver()

    def _start_receiver(self) -> None:
        self._running = True
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

    def _receive_loop(self) -> None:
        import cv2

        while self._running:
            try:
                parts = self._sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) >= 2:
                    frame_id_str = parts[0].decode()
                    buf = np.frombuffer(parts[1], dtype=np.uint8)
                    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if img is not None:
                        with self._lock:
                            self._latest_frame = img
                            try:
                                self._latest_frame_id = int(frame_id_str)
                            except ValueError:
                                self._latest_frame_id += 1
                            self._recv_count += 1
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"[{self.__class__.__name__}] 接收异常: {e}")
            time.sleep(0.001)

    def read_frame(self) -> Tuple[Optional[int], Optional[np.ndarray]]:
        """读取最新帧（线程安全，非阻塞）。"""
        with self._lock:
            if self._latest_frame is None:
                return None, None
            return self._latest_frame_id, self._latest_frame.copy()

    def get_stats(self) -> dict:
        with self._lock:
            return {
                "recv_count": self._recv_count,
                "has_data": self._latest_frame is not None,
            }

    def close(self) -> None:
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        self._sub.close()
