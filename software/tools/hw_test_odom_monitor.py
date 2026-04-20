# -*- coding: utf-8 -*-
"""硬件测试：轮式里程计实时监听器（固定位置刷新版）

Usage:
    cd software/src
    python ../tools/hw_test_odom_monitor.py

特性：
  - 终端固定位置刷新，防止刷屏
  - 显示当前位姿、速度、来源、测试计时
  - 保留最近 8 条事件日志
"""
from __future__ import annotations

import math
import os
import sys
import time
from collections import deque
from pathlib import Path

# 将 src 加入路径
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import zmq

from common.logging import get_logger
from common.zmq_helper import create_socket

logger = get_logger(__name__)

CLEAR_CMD = "cls" if os.name == "nt" else "clear"


def clear_screen():
    """清屏（兼容 Windows/Linux）"""
    os.system(CLEAR_CMD)


def draw_panel(state: dict, logs: deque):
    """绘制固定格式的监控面板"""
    odom = state.get("odom", {})
    chassis = state.get("chassis", {})
    test_start = state.get("test_start", time.time())
    elapsed = time.time() - test_start

    x = odom.get("x", 0.0)
    y = odom.get("y", 0.0)
    yaw = odom.get("yaw", 0.0)
    yaw_deg = math.degrees(yaw)
    odom_vx = odom.get("vx", 0.0)
    odom_vz = odom.get("vz", 0.0)

    cvx = chassis.get("vx", 0.0)
    cvz = chassis.get("vz", 0.0)
    csrc = chassis.get("source", "none")
    locked = chassis.get("emergency_locked", False)

    lines = []
    lines.append("╔" + "═" * 68 + "╗")
    lines.append("║" + " HomeBot 轮式里程计硬件测试监听器".center(68) + "║")
    lines.append("╠" + "═" * 68 + "╣")
    lines.append("║ 测试计时 : {:>10.1f} 秒".format(elapsed).ljust(69) + "║")
    lines.append("╠" + "═" * 68 + "╣")
    lines.append("║  里程计位姿".ljust(69) + "║")
    lines.append("║    x(m)  : {:>+10.4f}        y(m)  : {:>+10.4f}".format(x, y).ljust(69) + "║")
    lines.append("║    yaw   : {:>+10.2f}°       rad   : {:>+10.4f}".format(yaw_deg, yaw).ljust(69) + "║")
    lines.append("╠" + "═" * 68 + "╣")
    lines.append("║  速度对比".ljust(69) + "║")
    lines.append("║    里程计 vx : {:>+6.3f} m/s    vz : {:>+6.3f} rad/s".format(odom_vx, odom_vz).ljust(69) + "║")
    lines.append("║    底盘   vx : {:>+6.3f} m/s    vz : {:>+6.3f} rad/s".format(cvx, cvz).ljust(69) + "║")
    lines.append("║    控制来源  : {:<12}  急停锁定 : {}".format(csrc, "🔒 是" if locked else "  否").ljust(69) + "║")
    lines.append("╠" + "═" * 68 + "╣")
    lines.append("║  事件日志 (最近 {})".format(len(logs)).ljust(69) + "║")
    lines.append("║" + "-" * 68 + "║")

    # 固定显示 8 行日志
    log_entries = list(logs)[-8:]
    for i in range(8):
        if i < len(log_entries):
            text = log_entries[i][:68]
        else:
            text = ""
        lines.append("║ " + text.ljust(67) + "║")

    lines.append("╠" + "═" * 68 + "╣")
    lines.append("║ 操作指南：用手机/手柄控制底盘，观察里程计积分是否与实际位移一致".ljust(69) + "║")
    lines.append("║ [Ctrl+C] 结束测试".ljust(69) + "║")
    lines.append("╚" + "═" * 68 + "╝")

    clear_screen()
    print("\n".join(lines))


def main():
    import argparse
    parser = argparse.ArgumentParser(description="HomeBot 里程计硬件测试监听器")
    parser.add_argument("--odom", default="tcp://localhost:5559", help="里程计 PUB 地址")
    parser.add_argument("--chassis-state", default="tcp://localhost:5558", help="底盘状态 PUB 地址")
    parser.add_argument("--refresh", type=float, default=0.2, help="刷新间隔(秒)")
    args = parser.parse_args()

    ctx = zmq.Context()

    # 订阅里程计
    odom_sub = create_socket(zmq.SUB, bind=False, address=args.odom)
    odom_sub.setsockopt(zmq.SUBSCRIBE, b"")
    odom_sub.setsockopt(zmq.RCVTIMEO, 200)

    # 订阅底盘状态
    state_sub = create_socket(zmq.SUB, bind=False, address=args.chassis_state)
    state_sub.setsockopt(zmq.SUBSCRIBE, b"")
    state_sub.setsockopt(zmq.RCVTIMEO, 200)

    state = {
        "odom": {},
        "chassis": {},
        "test_start": time.time(),
    }
    logs = deque(maxlen=8)
    last_draw = 0

    logs.append("[系统] 监听器已启动，等待数据...")

    try:
        while True:
            # 非阻塞接收里程计
            try:
                odom = odom_sub.recv_json(flags=zmq.NOBLOCK)
                state["odom"] = odom
                t = time.strftime("%H:%M:%S", time.localtime())
                x = odom.get("x", 0)
                y = odom.get("y", 0)
                yaw_deg = math.degrees(odom.get("yaw", 0))
                logs.append(f"[{t}] ODOM  x={x:+.3f} y={y:+.3f} yaw={yaw_deg:+.1f}°")
            except zmq.Again:
                pass

            # 非阻塞接收底盘状态
            try:
                chassis = state_sub.recv_json(flags=zmq.NOBLOCK)
                state["chassis"] = chassis
                t = time.strftime("%H:%M:%S", time.localtime())
                vx = chassis.get("vx", 0)
                vz = chassis.get("vz", 0)
                src = chassis.get("source", "none")
                if abs(vx) > 0.001 or abs(vz) > 0.001:
                    logs.append(f"[{t}] CHAS  vx={vx:+.2f} vz={vz:+.2f} src={src}")
            except zmq.Again:
                pass

            # 定时刷新面板
            now = time.time()
            if now - last_draw >= args.refresh:
                draw_panel(state, logs)
                last_draw = now
            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        clear_screen()
        print("\n测试已结束。")
        odom = state.get("odom", {})
        x = odom.get("x", 0)
        y = odom.get("y", 0)
        yaw_deg = math.degrees(odom.get("yaw", 0))
        elapsed = time.time() - state["test_start"]
        print(f"最终位姿: x={x:.4f}m, y={y:.4f}m, yaw={yaw_deg:.2f}°")
        print(f"测试时长: {elapsed:.1f}秒")
    finally:
        odom_sub.close()
        state_sub.close()
        ctx.term()


if __name__ == "__main__":
    main()
