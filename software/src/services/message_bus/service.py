# -*- coding: utf-8 -*-
"""消息总线 broker：XPUB-XSUB 代理

发布者(PUB) connect 到 XSUB 端口，订阅者(SUB) connect 到 XPUB 端口，
broker 在两者之间转发消息，新增消息通道无需申请新端口。

    发布者(PUB) ──connect──> XSUB(5590) ──[zmq.proxy]──> XPUB(5591) <──connect── 订阅者(SUB)
"""
import zmq

from common.logging import get_logger

logger = get_logger(__name__)


def run_broker(xsub_addr: str, xpub_addr: str) -> None:
    """启动 XPUB-XSUB 代理（阻塞运行，Ctrl+C 退出）

    Args:
        xsub_addr: XSUB 绑定地址，发布者 connect 到此（如 "tcp://*:5590"）
        xpub_addr: XPUB 绑定地址，订阅者 connect 到此（如 "tcp://*:5591"）
    """
    context = zmq.Context()
    xsub = context.socket(zmq.XSUB)
    xpub = context.socket(zmq.XPUB)

    xsub.bind(xsub_addr)
    xpub.bind(xpub_addr)
    logger.info(f"Message bus broker started: PUB-side(XSUB) {xsub_addr}, SUB-side(XPUB) {xpub_addr}")

    # 注意：不使用 zmq.proxy()——它阻塞在 C 层循环中，Python 信号处理器
    # 无法执行，导致 Windows 下 Ctrl+C 无法退出。这里用带超时的 Poller
    # 手动转发，效果等价（XPUB 的订阅消息转发给 XSUB，XSUB 的发布消息
    # 转发给 XPUB），且每 100ms 回到 Python 层一次，可正常响应 Ctrl+C。
    poller = zmq.Poller()
    poller.register(xsub, zmq.POLLIN)
    poller.register(xpub, zmq.POLLIN)

    try:
        while True:
            events = dict(poller.poll(timeout=100))
            if xsub in events:
                xpub.send_multipart(xsub.recv_multipart())
            if xpub in events:
                xsub.send_multipart(xpub.recv_multipart())
    except KeyboardInterrupt:
        logger.info("Message bus broker interrupted")
    except zmq.ContextTerminated:
        pass
    finally:
        xsub.close()
        xpub.close()
        context.term()
        logger.info("Message bus broker stopped")
