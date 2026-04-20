# -*- coding: utf-8 -*-
"""硬件测试：局部避障实时监听器（固定位置刷新版）

Usage:
    cd software/src
    python ../tools/hw_test_obstacle_avoid.py

特性：
  - 终端固定位置刷新，防止刷屏
  - 显示障碍物列表、位姿、底盘速度、事件日志
  - 保留最近 8 条事件日志
"""
from __future__ import annotations

import json
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
    obstacles = state.get("obstacles", [])
    test_start = state.get("test_start", time.time())
    elapsed = time.time() - test_start

    x = odom.get("x", 0.0)
    y = odom.get("y", 0.0)
    yaw = odom.get("yaw", 0.0)
    yaw_deg = math.degrees(yaw)

    cvx = chassis.get("vx", 0.0)
    cvz = chassis.get("vz", 0.0)
    csrc = chassis.get("source", "none")
    locked = chassis.get("emergency_locked", False)

    lines = []
    lines.append("╔" + "═" * 68 + "╗")
    lines.append("║" + " HomeBot 局部避障硬件测试监听器".center(68) + "║")
    lines.append("╠" + "═" * 68 + "╣")
    lines.append("║ 测试计时 : {:>10.1f} 秒".format(elapsed).ljust(69) + "║")
    lines.append("╠" + "═" * 68 + "╣")
    lines.append("║  机器人位姿".ljust(69) + "║")
    lines.append("║    x(m)  : {:>+10.4f}        y(m)  : {:>+10.4f}".format(x, y).ljust(69) + "║")
    lines.append("║    yaw   : {:>+10.2f}°       rad   : {:>+10.4f}".format(yaw_deg, yaw).ljust(69) + "║")
    lines.append("╠" + "═" * 68 + "╣")
    lines.append("║  障碍物检测 (最近 {})".format(len(obstacles)).ljust(69) + "║")
    lines.append("║" + "-" * 68 + "║")

    # 固定显示 6 行障碍物
    if obstacles:
        for i, obs in enumerate(obstacles[:6]):
            ox, oz = obs.get("x", 0), obs.get("z", 0)
            w, h = obs.get("width", 0), obs.get("height", 0)
            dist = math.hypot(ox, oz)
            text = "  #{} 距离={:.2f}m 水平={:+.2f}m 深度={:.2f}m 尺寸={:.2f}×{:.2f}m".format(
                i + 1, dist, ox, oz, w, h
            )
            lines.append("║" + text[:68].ljust(68) + "║")
        for _ in range(6 - len(obstacles[:6])):
            lines.append("║" + " " * 68 + "║")
    else:
        lines.append("║  (暂无障碍物)".ljust(69) + "║")
        for _ in range(5):
            lines.append("║" + " " * 68 + "║")

    lines.append("╠" + "═" * 68 + "╣")
    lines.append("║  底盘执行".ljust(69) + "║")
    lines.append("║    vx : {:>+6.3f} m/s    vz : {:>+6.3f} rad/s".format(cvx, cvz).ljust(69) + "║")
    lines.append("║    来源 : {:<12}        急停锁定 : {}".format(
        csrc, "🔒 是" if locked else "  否"
    ).ljust(69) + "║")
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
    lines.append("║ 操作：启动 GoalFollowApp 后，观察机器人对障碍物的反应".ljust(69) + "║")
    lines.append("║ [Ctrl+C] 结束测试".ljust(69) + "║")
    lines.append("╚" + "═" * 68 + "╝")

    clear_screen()
    print("\n".join(lines))


def main():
    import argparse
    parser = argparse.ArgumentParser(description="HomeBot 局部避障硬件测试监听器")
    parser.add_argument("--odom", default="tcp://localhost:5559", help="里程计 PUB 地址")
    parser.add_argument("--obstacle", default="tcp://localhost:5562", help="障碍物 PUB 地址")
    parser.add_argument("--chassis-state", default="tcp://localhost:5558", help="底盘状态 PUB 地址")
    parser.add_argument("--refresh", type=float, default=0.2, help="刷新间隔(秒)")
    args = parser.parse_args()

    ctx = zmq.Context()

    # 订阅里程计
    odom_sub = create_socket(zmq.SUB, bind=False, address=args.odom)
    odom_sub.setsockopt(zmq.SUBSCRIBE, b"")
    odom_sub.setsockopt(zmq.RCVTIMEO, 200)

    # 订阅障碍物
    obs_sub = create_socket(zmq.SUB, bind=False, address=args.obstacle)
    obs_sub.setsockopt(zmq.SUBSCRIBE, b"")
    obs_sub.setsockopt(zmq.RCVTIMEO, 200)

    # 订阅底盘状态
    state_sub = create_socket(zmq.SUB, bind=False, address=args.chassis_state)
    state_sub.setsockopt(zmq.SUBSCRIBE, b"")
    state_sub.setsockopt(zmq.RCVTIMEO, 200)

    state = {
        "odom": {},
        "chassis": {},
        "obstacles": [],
        "test_start": time.time(),
    }
    logs = deque(maxlen=8)
    last_draw = 0

    logs.append("[系统] 监听器已启动，等待数据...")

    try:
        while True:
            # 非阻塞接收障碍物
            try:
                parts = obs_sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) >= 2:
                    data = json.loads(parts[1].decode("utf-8"))
                    obstacles = data.get("obstacles", [])
                    state["obstacles"] = obstacles
                    inf_ms = data.get("inference_ms", 0)
                    t = time.strftime("%H:%M:%S", time.localtime())
                    logs.append(f"[{t}] OBS  {len(obstacles)}个障碍物 推理{inf_ms:.0f}ms")
            except zmq.Again:
                pass

            # 非阻塞接收里程计
            try:
                odom = odom_sub.recv_json(flags=zmq.NOBLOCK)
                state["odom"] = odom
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
                    logs.append(f"[{t}] CHAS vx={vx:+.2f} vz={vz:+.2f} src={src}")
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
        obs_sub.close()
        state_sub.close()
        ctx.term()


if __name__ == "__main__":
    main()
