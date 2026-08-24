# -*- coding: utf-8 -*-
"""底盘调试命令：move

通过通用 REQ 客户端向底盘服务（默认 tcp://localhost:5556）发送速度命令。
不导入 services.motion_service（其 __init__ 会连带导入串口/HAL 重依赖），
命令格式与 chassis_arbiter.arbiter.ChassisArbiterClient 保持一致。
"""
import time

import click

from common.bus import ZMQRequestClient


@click.command()
@click.option("--vx", default=0.0, show_default=True, help="线速度 X (m/s)，前进为正")
@click.option("--vy", default=0.0, show_default=True, help="线速度 Y (m/s)，左移为正")
@click.option("--vz", default=0.0, show_default=True, help="角速度 Z (rad/s)，左旋为正")
@click.option("--duration", "-d", default=1.0, show_default=True, help="运动时长（秒），结束后自动停止")
@click.option("--addr", default=None, help="底盘服务地址，默认从配置读取")
def move(vx: float, vy: float, vz: float, duration: float, addr: str):
    """发送底盘速度命令（调试用途），持续 duration 秒后自动停止

    示例:
        homebot move --vx 0.2 -d 2     # 0.2 m/s 前进 2 秒
        homebot move --vz 0.5 -d 1     # 原地左转 1 秒
    """
    if addr is None:
        from configs import get_config
        addr = get_config().chassis.service_addr.replace("*", "localhost")

    client = ZMQRequestClient(addr=addr, timeout_ms=1000)

    def send(vx_, vy_, vz_):
        return client.request({
            "source": "web",
            "vx": vx_,
            "vy": vy_,
            "vz": vz_,
            "priority": 1,
            "timestamp": time.time(),
        })

    # 仲裁器 1 秒未收到指令会超时停止，持续期间每 0.2 秒重发一次
    click.echo(f"底盘运动: vx={vx} vy={vy} vz={vz}，持续 {duration}s")
    deadline = time.time() + duration
    resp = None
    try:
        while time.time() < deadline:
            resp = send(vx, vy, vz)
            if resp is None:
                click.echo("错误: 底盘服务无响应（服务未启动或地址不对）", err=True)
                raise SystemExit(1)
            if not resp.get("success", False):
                click.echo(f"命令被拒绝: {resp.get('message', '')} "
                           f"(当前控制源: {resp.get('current_owner', '?')})", err=True)
                raise SystemExit(1)
            time.sleep(0.2)
    finally:
        # 发送停止
        send(0.0, 0.0, 0.0)
        client.close()
    click.echo("已停止")
