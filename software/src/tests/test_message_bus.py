# -*- coding: utf-8 -*-
"""通用消息总线测试

在线程内启动 XPUB-XSUB broker，验证 BusPublisher / BusSubscriber：
1. 发布-订阅往返（自定义 user.* 类型消息送达）
2. topic 前缀过滤（订阅 user. 收不到 sensor.battery）
3. messages.serialize() 接受字符串类型（向后兼容 MessageType 枚举）

运行方式:
    cd software/src
    python -m tests.test_message_bus
"""
import socket
import threading
import time

import zmq

from common.bus import BusPublisher, BusSubscriber
from common.messages import MessageType, serialize


def _free_port() -> int:
    """获取一个随机空闲端口"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_serialize_open_types():
    """serialize() 应同时接受 MessageType 枚举和自定义字符串类型"""
    payload_enum = serialize(MessageType.BATTERY_STATE, {"voltage": 12.0})
    assert payload_enum["type"] == "sensor.battery", payload_enum

    payload_str = serialize("user.temperature", {"value": 25.6})
    assert payload_str["type"] == "user.temperature", payload_str
    print("[OK] serialize() 支持枚举与自定义字符串类型")


def test_pub_sub_roundtrip_and_filter():
    """发布-订阅往返 + topic 前缀过滤"""
    xsub_port = _free_port()
    xpub_port = _free_port()
    xsub_addr = f"tcp://127.0.0.1:{xsub_port}"
    xpub_addr = f"tcp://127.0.0.1:{xpub_port}"

    # 线程内启动 broker
    broker_ctx = zmq.Context()
    broker_xsub = broker_ctx.socket(zmq.XSUB)
    broker_xpub = broker_ctx.socket(zmq.XPUB)
    broker_xsub.bind(xsub_addr)
    broker_xpub.bind(xpub_addr)
    def _run_proxy():
        try:
            zmq.proxy(broker_xsub, broker_xpub)
        except Exception:
            pass  # 测试结束时 socket/context 被主线程回收，proxy 退出属正常

    broker_thread = threading.Thread(target=_run_proxy, daemon=True)
    broker_thread.start()

    received = []
    received_event = threading.Event()

    def on_user_msg(payload):
        received.append(payload)
        received_event.set()

    # 共享 context 供 pub/sub 客户端使用
    ctx = zmq.Context()
    pub = BusPublisher(addr=xsub_addr, context=ctx)
    sub = BusSubscriber(addr=xpub_addr, context=ctx, recv_timeout_ms=200)
    sub.on_message("user.", on_user_msg)
    sub.start()

    try:
        # 等待订阅关系经 XPUB 传播到 XSUB 侧
        time.sleep(0.5)

        # 自定义消息应送达
        pub.publish("user.temperature", {"value": 25.6}, timestamp=time.time())
        # 非订阅前缀的消息不应送达
        pub.publish(MessageType.BATTERY_STATE, {"voltage": 12.0})

        assert received_event.wait(timeout=3.0), "超时未收到 user.temperature 消息"
        time.sleep(0.3)  # 给 battery 消息留足送达时间

        user_msgs = [m for m in received if m["type"] == "user.temperature"]
        battery_msgs = [m for m in received if m["type"] == "sensor.battery"]

        assert len(user_msgs) == 1, f"应收到 1 条 user 消息，实际 {len(user_msgs)}"
        assert user_msgs[0]["data"]["value"] == 25.6, user_msgs[0]
        assert "timestamp" in user_msgs[0], "消息应包含 timestamp"
        assert len(battery_msgs) == 0, "订阅 user. 不应收到 sensor.battery"

        print("[OK] 发布-订阅往返正常，topic 前缀过滤生效")
    finally:
        sub.stop()
        pub.close()
        broker_xsub.close()
        broker_xpub.close()
        broker_ctx.term()
        ctx.term()


def main():
    print("=" * 60)
    print("通用消息总线测试")
    print("=" * 60)

    test_serialize_open_types()
    test_pub_sub_roundtrip_and_filter()

    print("=" * 60)
    print("全部测试通过")
    print("=" * 60)


if __name__ == "__main__":
    main()
