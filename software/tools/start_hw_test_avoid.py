# -*- coding: utf-8 -*-
"""一键启动：局部避障硬件测试环境

启动的服务：
  1. 底盘服务
  2. VisionService（摄像头）
  3. DepthService（深度估计 + 障碍物检测）
  4. OdomService（里程计）
  5. 避障监听器（本窗口）

然后手动启动 GoalFollowApp：
    cd software/src
    python -m navigation.applications.goal_follow --goal-x 1.0 --goal-y 0.0

Usage:
    cd software
    python tools/start_hw_test_avoid.py [--port COM3]
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

SRC_DIR = Path(__file__).parent.parent / "src"
MODEL_PATH = Path(__file__).parent.parent / "models" / "midas" / "midas_small.onnx"


def run_in_new_terminal(title: str, cmd: list):
    full_cmd = " ".join(cmd)
    subprocess.Popen(
        ["start", "cmd", "/k", f"title {title} && cd {SRC_DIR} && {full_cmd}"],
        shell=True,
    )
    time.sleep(1.5)


def main():
    parser = argparse.ArgumentParser(description="启动局部避障硬件测试环境")
    parser.add_argument("--port", default="COM3", help="底盘串口")
    parser.add_argument("--no-terminal", action="store_true", help="不在新窗口启动")
    parser.add_argument("--goal-x", type=float, default=1.0, help="目标点 X（米）")
    parser.add_argument("--goal-y", type=float, default=0.0, help="目标点 Y（米）")
    args = parser.parse_args()

    print("=" * 60)
    print("HomeBot 局部避障硬件测试启动器")
    print("=" * 60)

    if not args.no_terminal:
        print("正在启动服务...")

        run_in_new_terminal(
            "CHASSIS",
            [sys.executable, "-m", "services.motion_service.chassis_service", "--port", args.port],
        )
        print("✅ 底盘服务已启动")

        run_in_new_terminal(
            "VISION",
            [sys.executable, "-m", "services.vision_service"],
        )
        print("✅ VisionService 已启动")

        run_in_new_terminal(
            "DEPTH",
            [
                sys.executable, "-m", "navigation.services.depth_service",
                "--model", str(MODEL_PATH),
                "--fps", "5",
            ],
        )
        print("✅ DepthService 已启动")

        run_in_new_terminal(
            "ODOM",
            [sys.executable, "-m", "navigation.services.odom_service"],
        )
        print("✅ OdomService 已启动")

        time.sleep(2)
        print("\n核心服务已启动。")
        print(f"\n下一步：在新终端手动启动 GoalFollowApp：")
        print(f"  cd software/src")
        print(f"  python -m navigation.applications.goal_follow --goal-x {args.goal_x} --goal-y {args.goal_y}")
        print("=" * 60)

    # 在当前终端启动监听器
    monitor_script = Path(__file__).parent / "hw_test_obstacle_avoid.py"
    subprocess.run([sys.executable, str(monitor_script)])


if __name__ == "__main__":
    main()
