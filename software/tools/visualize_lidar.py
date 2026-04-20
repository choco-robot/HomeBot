# -*- coding: utf-8 -*-
"""LD06 激光雷达实时可视化工具

用法:
    python tools/visualize_lidar.py --port COM7
    python tools/visualize_lidar.py --port /dev/ttyUSB0 --range 5
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

# 将项目 src 加入路径（兼容从 software/ 或项目根目录运行）
src_path = os.path.join(os.path.dirname(__file__), "..", "src")
sys.path.insert(0, os.path.abspath(src_path))

from navigation.hal.lidar_driver import LD06Driver


def main():
    parser = argparse.ArgumentParser(description="LD06 激光雷达实时可视化")
    parser.add_argument("--port", default="COM7", help="串口号 (默认 COM7)")
    parser.add_argument("--baud", type=int, default=230400, help="波特率")
    parser.add_argument("--range", type=float, default=5.0, help="显示范围 (米)")
    parser.add_argument("--size", type=int, default=360, help="扫描分辨率")
    args = parser.parse_args()

    print(f"正在连接雷达 {args.port} @ {args.baud} ...")
    lidar = LD06Driver(port=args.port, baudrate=args.baud, scan_size=args.size)
    lidar.start()
    print("雷达已启动，等待数据...")

    # 等待第一帧数据
    for _ in range(10):
        if lidar.get_scan() is not None:
            break
        time.sleep(0.2)
    else:
        print("错误: 未收到雷达数据，请检查串口号和连接。")
        lidar.stop()
        return

    # Matplotlib 实时模式
    import matplotlib
    import matplotlib.pyplot as plt

    # 设置中文字体
    zh_fonts = ["Microsoft YaHei", "SimHei", "WenQuanYi Micro Hei", "Arial Unicode MS"]
    available_fonts = [f.name for f in matplotlib.font_manager.fontManager.ttflist]
    for font in zh_fonts:
        if font in available_fonts:
            plt.rcParams["font.sans-serif"] = [font] + plt.rcParams["font.sans-serif"]
            break
    plt.rcParams["axes.unicode_minus"] = False

    plt.ion()
    plt.rcParams["figure.figsize"] = [8, 8]
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(-args.range, args.range)
    ax.set_ylim(-args.range, args.range)
    ax.set_title(f"LD06 LiDAR 实时点云 [{args.port}]")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle="--", alpha=0.5)

    # 中心画一个圆代表机器人
    robot_circle = plt.Circle((0, 0), 0.15, color="red", fill=True, alpha=0.8)
    ax.add_patch(robot_circle)

    # 散点图对象
    scatter = ax.scatter([], [], s=5, c="blue", alpha=0.6)
    info_text = ax.text(
        0.02, 0.98, "", transform=ax.transAxes,
        verticalalignment="top", fontsize=10,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5)
    )

    print("可视化已启动，关闭窗口或按 Ctrl+C 退出。")
    running = True

    def on_close(_):
        nonlocal running
        running = False

    fig.canvas.mpl_connect("close_event", on_close)

    try:
        while running:
            scan = lidar.get_scan()
            if scan is None:
                plt.pause(0.05)
                continue

            angles_deg, distances_mm = scan
            angles = np.radians(angles_deg)
            distances = np.array(distances_mm) / 1000.0  # mm → m

            # 过滤无效点（距离为 0 或超出量程）
            valid = (distances > 0.05) & (distances < 12.0)
            angles = angles[valid]
            distances = distances[valid]

            # 极坐标 → 笛卡尔坐标（机器人朝前为 X 正方向）
            x = distances * np.sin(angles)
            y = distances * np.cos(angles)

            # 按置信度着色（如果有）
            colors = distances

            scatter.set_offsets(np.c_[x, y])
            scatter.set_array(colors)
            scatter.set_clim(0, args.range)

            info_text.set_text(
                f"点数: {len(x)}\n"
                f"距离范围: {distances.min():.2f}~{distances.max():.2f} m\n"
                f"更新时间: {time.strftime('%H:%M:%S')}"
            )

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.05)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        lidar.stop()
        plt.close()
        print("可视化已退出")


if __name__ == "__main__":
    main()
