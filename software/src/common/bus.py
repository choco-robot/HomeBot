# -*- coding: utf-8 -*-
"""通用消息总线客户端

基于 XPUB-XSUB 代理（services.message_bus）的发布/订阅接口，
用户无需申请新端口即可扩展自定义消息。

约定：
- 消息在线路上为 ZMQ multipart: [topic(bytes), json_payload(bytes)]
- topic 即消息类型字符串，采用点分层级（sensor.battery、user.xxx、ext.xxx）
- 用户自定义消息建议使用 "user.*" 或 "ext.*" 前缀
- payload 沿用 common.messages 的 {"type", "data", "timestamp"} JSON 信封

用法示例：

    # 发布自定义消息
    from common.bus import BusPublisher
    pub = BusPublisher()
    pub.publish("user.temperature", {"value": 25.6})

    # 订阅自定义消息
    from common.bus import BusSubscriber
    sub = BusSubscriber()
    def on_temp(payload):
        print(payload["data"])
    sub.on_message("user.temperature", on_temp)
    sub.start()
"""
import json
import threading
from typing import Any, Callable, Dict, List, Optional, Union

import zmq

from common.logging import get_logger
from common.messages import MessageType, serialize, resolve_type
from common.zmq_helper import create_context

logger = get_logger(__name__)


def _default_addr(kind: str) -> str:
    """从全局配置获取总线地址，bind 地址自动转换为本地 connect 地址"""
    from configs import get_config

    config = get_config()
    addr = config.zmq.bus_xsub_addr if kind == "xsub" else config.zmq.bus_xpub_addr
    return addr.replace("*", "localhost")


class BusPublisher:
    """消息总线发布者

    PUB socket connect 到 broker 的 XSUB 地址（默认 tcp://localhost:5590）。
    """

    def __init__(self, addr: Optional[str] = None, context: Optional[zmq.Context] = None):
        """
        Args:
            addr: broker XSUB 地址，默认从配置读取（tcp://localhost:5590）
            context: 可选的 ZeroMQ 上下文，不传则使用全局实例
        """
        self._addr = addr or _default_addr("xsub")
        self._ctx = context or create_context()
        self._socket = self._ctx.socket(zmq.PUB)
        self._socket.connect(self._addr)
        self._lock = threading.Lock()
        logger.info(f"BusPublisher connected to {self._addr}")

    def publish(
        self,
        msg_type: Union[MessageType, str],
        data: Dict[str, Any],
        timestamp: float = None,
    ) -> None:
        """发布一条消息

        Args:
            msg_type: 内置 MessageType 枚举或自定义类型字符串（如 "user.temperature"）
            data: 消息数据（必须可 JSON 序列化）
            timestamp: 可选时间戳
        """
        topic = resolve_type(msg_type)
        payload = serialize(topic, data, timestamp)
        with self._lock:
            self._socket.send_multipart(
                [topic.encode("utf-8"), json.dumps(payload).encode("utf-8")]
            )

    def close(self) -> None:
        """关闭发布 socket"""
        if self._socket and not self._socket.closed:
            self._socket.close()
            logger.info("BusPublisher closed")


class BusSubscriber:
    """消息总线订阅者

    SUB socket connect 到 broker 的 XPUB 地址（默认 tcp://localhost:5591），
    后台线程接收消息并按 topic 前缀分发到注册的回调。
    """

    def __init__(
        self,
        addr: Optional[str] = None,
        context: Optional[zmq.Context] = None,
        recv_timeout_ms: int = 1000,
    ):
        """
        Args:
            addr: broker XPUB 地址，默认从配置读取（tcp://localhost:5591）
            context: 可选的 ZeroMQ 上下文，不传则使用全局实例
            recv_timeout_ms: 接收超时（毫秒），用于后台线程检查停止标志
        """
        self._addr = addr or _default_addr("xpub")
        self._ctx = context or create_context()
        self._socket = self._ctx.socket(zmq.SUB)
        self._socket.setsockopt(zmq.RCVTIMEO, recv_timeout_ms)
        self._socket.connect(self._addr)

        self._handlers: List[tuple] = []  # [(prefix, callback), ...]
        self._handlers_lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        logger.info(f"BusSubscriber connected to {self._addr}")

    def subscribe(self, prefix: Union[MessageType, str] = "") -> None:
        """订阅 topic 前缀（ZMQ 层面过滤，必须在 start() 之前调用才立即生效）

        Args:
            prefix: topic 前缀，空字符串表示订阅全部
        """
        self._socket.setsockopt_string(zmq.SUBSCRIBE, resolve_type(prefix))

    def on_message(
        self,
        msg_type_prefix: Union[MessageType, str],
        callback: Callable[[Dict[str, Any]], None],
    ) -> None:
        """注册消息回调，按 topic 前缀分发

        同时自动完成对应前缀的 ZMQ 订阅，无需单独调用 subscribe()。

        Args:
            msg_type_prefix: 消息类型前缀（如 "user." 匹配所有 user.* 消息）
            callback: 回调函数，参数为完整消息信封 {"type", "data", "timestamp"}
        """
        prefix = resolve_type(msg_type_prefix)
        with self._handlers_lock:
            self._handlers.append((prefix, callback))
        self.subscribe(prefix)

    def start(self) -> None:
        """启动后台接收线程"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        logger.info("BusSubscriber started background receiver")

    def stop(self) -> None:
        """停止后台接收线程并关闭 socket"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._socket and not self._socket.closed:
            self._socket.close()
        logger.info("BusSubscriber stopped")

    def _receive_loop(self) -> None:
        """后台线程持续接收并分发消息"""
        while self._running:
            try:
                parts = self._socket.recv_multipart()
            except zmq.Again:
                # 超时，检查停止标志后继续
                continue
            except zmq.ContextTerminated:
                break
            except Exception as e:
                if self._running:
                    logger.warning(f"BusSubscriber receive error: {e}")
                continue

            if len(parts) != 2:
                continue
            topic = parts[0].decode("utf-8", errors="replace")
            try:
                payload = json.loads(parts[1].decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                logger.warning(f"BusSubscriber invalid payload on topic {topic}: {e}")
                continue

            with self._handlers_lock:
                handlers = list(self._handlers)
            for prefix, callback in handlers:
                if topic.startswith(prefix):
                    try:
                        callback(payload)
                    except Exception as e:
                        logger.warning(f"BusSubscriber handler error on {topic}: {e}")


class ZMQRequestClient:
    """通用 REQ 客户端，内置"超时 → close → 重建 socket"故障恢复

    适用于向 REP 服务（如底盘服务 5556、机械臂服务 5557）发送请求，
    收敛各处重复的 socket 重建逻辑。
    """

    def __init__(
        self,
        addr: str,
        timeout_ms: int = 1000,
        context: Optional[zmq.Context] = None,
    ):
        """
        Args:
            addr: REP 服务地址（如 "tcp://localhost:5556"）
            timeout_ms: 请求超时（毫秒）
            context: 可选的 ZeroMQ 上下文，不传则使用全局实例
        """
        self._addr = addr
        self._timeout_ms = timeout_ms
        self._ctx = context or create_context()
        self._socket: Optional[zmq.Socket] = None
        self._lock = threading.Lock()
        self._create_socket()

    def _create_socket(self) -> None:
        """创建（或重建）REQ socket"""
        if self._socket and not self._socket.closed:
            self._socket.close()
        self._socket = self._ctx.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, self._timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(self._addr)

    def request(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """发送请求并等待响应，超时或失败时自动重建 socket

        Args:
            payload: 请求内容（JSON 可序列化）

        Returns:
            响应字典；超时或失败返回 None
        """
        with self._lock:
            try:
                self._socket.send_json(payload)
                return self._socket.recv_json()
            except zmq.Again:
                logger.warning(f"ZMQRequestClient timeout, recreating socket to {self._addr}")
                self._create_socket()
                return None
            except Exception as e:
                logger.warning(f"ZMQRequestClient error: {e}, recreating socket")
                self._create_socket()
                return None

    def close(self) -> None:
        """关闭 socket"""
        if self._socket and not self._socket.closed:
            self._socket.close()
