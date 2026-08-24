# -*- coding: utf-8 -*-
"""消息总线调试命令：topic echo / topic pub"""
import json
import threading

import click

from common.bus import BusPublisher, BusSubscriber


@click.group()
def topic():
    """消息总线调试（发布/订阅自定义消息）"""


@topic.command("echo")
@click.argument("topic_prefix")
@click.option("--addr", default=None, help="broker XPUB 地址，默认从配置读取")
@click.option("--count", "-c", default=0, show_default=True,
              help="收到 N 条消息后退出，0 表示持续监听")
def topic_echo(topic_prefix: str, addr: str, count: int):
    """订阅消息总线并打印消息（类似 ros topic echo）

    示例:
        homebot topic echo user.
        homebot topic echo sensor.battery -c 1
    """
    received = 0
    done = threading.Event()

    def on_message(payload):
        nonlocal received
        received += 1
        click.echo(json.dumps(payload, ensure_ascii=False))
        if count > 0 and received >= count:
            done.set()

    sub = BusSubscriber(addr=addr)
    sub.on_message(topic_prefix, on_message)
    sub.start()
    click.echo(f"订阅中: {topic_prefix} (Ctrl+C 退出)", err=True)

    try:
        while not done.wait(timeout=0.2):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        sub.stop()


@topic.command("pub")
@click.argument("msg_type")
@click.argument("data")
@click.option("--addr", default=None, help="broker XSUB 地址，默认从配置读取")
def topic_pub(msg_type: str, data: str, addr: str):
    """向消息总线发布一条消息

    DATA 为 JSON 对象字符串。

    示例:
        homebot topic pub user.temperature "{\\"value\\": 25.6}"
    """
    try:
        payload = json.loads(data)
    except json.JSONDecodeError as e:
        raise click.BadParameter(f"DATA 不是合法 JSON: {e}")
    if not isinstance(payload, dict):
        raise click.BadParameter("DATA 必须是 JSON 对象（{...}）")

    pub = BusPublisher(addr=addr)
    # PUB 连接建立需要时间，等待订阅传播后再发，避免丢消息
    import time
    time.sleep(0.5)
    pub.publish(msg_type, payload, timestamp=time.time())
    time.sleep(0.2)  # 确保消息在进程退出前发出
    pub.close()
    click.echo(f"已发布: {msg_type} -> {json.dumps(payload, ensure_ascii=False)}")
