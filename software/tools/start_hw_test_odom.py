# -*- coding: utf-8 -*-
"""一键启动：轮式里程计硬件测试环境

启动的服务：
  1. 底盘服务（含状态发布）
  2. OdomService
  3. 里程计监听器（本窗口）

Usage:
    cd software
    python tools/start_hw_test_odom.py [--port COM3]
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

SRC_DIR = Path(__file__).parent.parent / "src"


def run_in_new_terminal(title: str, cmd: list):
    """在新终端窗口中运行命令（Windows）"""
    # 使用 start 命令在新窗口运行
    full_cmd = " ".join(cmd)
    subprocess.Popen(
        ["start", "cmd", "/k", f"title {title} && cd {SRC_DIR} && {full_cmd}"],
        shell=True,
    )
    time.sleep(1.5)


def main():
    parser = argparse.ArgumentParser(description="启动里程计硬件测试环境")
    parser.add_argument("--port", default="COM3", help="底盘串口")
    parser.add_argument("--no-terminal", action="store_true", help="不在新窗口启动（手动启动时使用）")
    args = parser.parse_args()

    print("=" * 60)
    print("HomeBot 轮式里程计硬件测试启动器")
    print("=" * 60)

    if not args.no_terminal:
        print("正在启动服务...")
        # 终端 1: 底盘服务
        run_in_new_terminal(
            "CHASSIS",
            [sys.executable, "-m", "services.motion_service.chassis_service", "--port", args.port],
        )
        print("✅ 底盘服务已启动")

        # 终端 2: OdomService
        run_in_new_terminal(
            "ODOM",
            [sys.executable, "-m", "navigation.services.odom_service"],
        )
        print("✅ OdomService 已启动")

        time.sleep(2)
        print("\n所有服务已启动。接下来启动监听器...")
        print("=" * 60)

    # 在当前终端启动监听器
    monitor_script = Path(__file__).parent / "hw_test_odom_monitor.py"
    subprocess.run([sys.executable, str(monitor_script)])


if __name__ == "__main__":
    main()
