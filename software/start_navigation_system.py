#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HomeBot 导航系统一键启动器

启动导航核心服务（无需视觉/深度感知）：
  1. 底盘服务   (services.motion_service --service chassis)
  2. 里程计服务 (navigation.services.odom_service)
  3. SLAM 服务  (navigation.services.slam_service)

跨平台支持 Windows/Linux/macOS

用法：
    cd software
    python start_navigation_system.py
    python start_navigation_system.py --port COM3
    python start_navigation_system.py --mock-lidar --mock-tag
"""

import os
import sys
import socket
import subprocess
import signal
import time
import platform
from pathlib import Path


# 服务配置
SERVICES = [
    {
        "name": "Chassis Service",
        "module": "services.motion_service",
        "args": ["--service", "chassis"],
        "ports": [5556, 5558],
        "desc": "底盘控制服务 (ZeroMQ: 5556)",
    },
    {
        "name": "Odom Service",
        "module": "navigation.services.odom_service",
        "ports": [5559, 5567],
        "desc": "里程计服务 (ZeroMQ: 5559)",
    },
    {
        "name": "SLAM Service",
        "module": "navigation.services.slam_service",
        "ports": [5563, 5564, 5565, 5568],
        "desc": "SLAM 融合定位 (激光雷达 + 里程计)",
    },
]


def print_header():
    """打印启动标题"""
    print("=" * 60)
    print("   HomeBot 导航系统一键启动器")
    print("=" * 60)
    print()
    print("将启动以下服务（无需视觉/深度感知）：")
    for i, svc in enumerate(SERVICES, 1):
        ports = f" 端口{svc['ports']}" if svc["ports"] else ""
        print(f"  {i}. {svc['name']} — {svc['desc']}{ports}")
    print()


def check_port(port):
    """检查端口是否被占用"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(("127.0.0.1", port))
            return False
        except socket.error:
            return True


def get_process_on_port(port):
    """获取占用端口的进程 PID"""
    try:
        if platform.system() == "Windows":
            result = subprocess.run(
                ["netstat", "-ano"],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="ignore",
            )
            for line in result.stdout.split("\n"):
                if f":{port}" in line and "LISTENING" in line:
                    parts = line.strip().split()
                    if len(parts) >= 5:
                        return int(parts[-1])
        else:
            result = subprocess.run(
                ["lsof", "-i", f"TCP:{port}", "-sTCP:LISTEN", "-t"],
                capture_output=True,
                text=True,
            )
            if result.returncode == 0 and result.stdout.strip():
                return int(result.stdout.strip().split("\n")[0])
    except Exception:
        pass
    return None


def kill_process(pid):
    """终止进程"""
    try:
        if platform.system() == "Windows":
            subprocess.run(
                ["taskkill", "/F", "/PID", str(pid)],
                capture_output=True,
                check=False,
            )
        else:
            os.kill(pid, signal.SIGKILL)
        return True
    except Exception:
        return False


def check_ports():
    """检查端口状态"""
    print("[检查] 检查端口占用情况...")
    print()
    occupied = []
    for svc in SERVICES:
        for port in svc["ports"]:
            if check_port(port):
                pid = get_process_on_port(port)
                info = f" (PID={pid})" if pid else ""
                print(f"  [占用] 端口 {port} 已被占用{info} [{svc['name']}]")
                occupied.append((port, pid, svc["name"]))
            else:
                print(f"  [可用] 端口 {port} [{svc['name']}]")
    print()
    return occupied


def prompt_user(occupied):
    """提示用户处理被占用的端口"""
    print("=" * 60)
    print("[警告] 部分端口已被占用，可能导致服务启动失败！")
    print()
    print("选项:")
    print("  1. 终止占用进程并继续")
    print("  2. 继续启动（可能出错）")
    print("  3. 退出")
    print("=" * 60)
    print()

    try:
        choice = input("请选择 [1-3]: ").strip()
    except (KeyboardInterrupt, EOFError):
        print("\n[退出] 用户取消")
        return False

    if choice == "3":
        return False
    elif choice == "2":
        print("[继续] 带警告启动...\n")
        return True
    elif choice == "1":
        killed = set()
        for port, pid, name in occupied:
            if pid and pid not in killed:
                print(f"  [终止] PID {pid} (端口 {port}, {name})...")
                kill_process(pid)
                killed.add(pid)
        print("[完成] 清理完毕，等待 2 秒...\n")
        time.sleep(2)
        return True
    else:
        print("[退出] 无效选项")
        return False


def start_service(svc, src_dir, extra_args=None):
    """在新窗口中启动单个服务"""
    print(f"[启动] {svc['name']}...")

    cmd_parts = [f'"{sys.executable}"', "-m", svc["module"]]
    if "args" in svc:
        cmd_parts.extend(svc["args"])
    if extra_args:
        cmd_parts.extend(extra_args)
    cmd_str = " ".join(cmd_parts)

    system = platform.system()

    if system == "Windows":
        subprocess.Popen(
            f'start "{svc["name"]}" cmd /k "cd /d \"{src_dir}\" && {cmd_str}"',
            shell=True,
        )
    elif system == "Darwin":
        script = f"""
        tell application "Terminal"
            do script "cd {src_dir} && {cmd_str}"
            set custom title of front window to "{svc['name']}"
        end tell
        """
        subprocess.Popen(
            ["osascript", "-e", script],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    else:
        cmd_full = f"cd {src_dir} && {cmd_str}"
        terminals = [
            (
                "gnome-terminal",
                [
                    "--title",
                    svc["name"],
                    "--",
                    "bash",
                    "-c",
                    f"{cmd_full}; exec bash",
                ],
            ),
            (
                "konsole",
                [
                    "--new-tab",
                    "-p",
                    f"tabtitle={svc['name']}",
                    "-e",
                    "bash",
                    "-c",
                    f"{cmd_full}; exec bash",
                ],
            ),
            (
                "xterm",
                ["-T", svc["name"], "-e", "bash", "-c", f"{cmd_full}; exec bash"],
            ),
        ]
        started = False
        for term, args in terminals:
            if subprocess.run(["which", term], capture_output=True).returncode == 0:
                subprocess.Popen(
                    [term] + args,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                started = True
                break
        if not started:
            print(f"    [提示] 未找到终端模拟器，后台运行 {svc['name']}...")
            subprocess.Popen(
                cmd_parts[1:],
                cwd=src_dir,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="HomeBot 导航系统一键启动器")
    parser.add_argument("--port", default=None, help="底盘串口号（如 COM3 或 /dev/ttyUSB0）")
    parser.add_argument("--mock-lidar", action="store_true", help="使用模拟激光雷达")
    parser.add_argument("--mock-tag", action="store_true", help="使用模拟 AprilTag")
    parser.add_argument(
        "--lidar-port", default=None, help="激光雷达串口号（如 COM4）"
    )
    args = parser.parse_args()

    print_header()

    # 检查端口
    occupied = check_ports()
    if occupied:
        if not prompt_user(occupied):
            input("\n按 Enter 退出...")
            sys.exit(1)
    else:
        print("[通过] 所有端口均可用\n")

    # 切换到 src 目录
    script_dir = Path(__file__).parent
    src_dir = script_dir / "src"
    if not src_dir.exists():
        print(f"[错误] 目录不存在: {src_dir}")
        input("按 Enter 退出...")
        sys.exit(1)

    # 构建额外参数
    # 底盘服务参数
    chassis_extra = []
    if args.port:
        chassis_extra.extend(["--port", args.port])

    # SLAM 服务参数
    slam_extra = []
    if args.mock_lidar:
        slam_extra.append("--mock-lidar")
    if args.mock_tag:
        slam_extra.append("--mock-tag")
    if args.lidar_port:
        slam_extra.extend(["--lidar-port", args.lidar_port])

    # 启动三个服务
    for i, svc in enumerate(SERVICES):
        extra = []
        if svc["name"] == "Chassis Service" and chassis_extra:
            extra = chassis_extra
        elif svc["name"] == "SLAM Service" and slam_extra:
            extra = slam_extra

        start_service(svc, str(src_dir), extra)
        time.sleep(1.5)  # 间隔启动，避免资源冲突

    print()
    print("=" * 60)
    print("[完成] 所有导航核心服务已启动！")
    print()
    print("服务地址:")
    print("  - 底盘命令:   tcp://127.0.0.1:5556 (REP)")
    print("  - 底盘状态:   tcp://127.0.0.1:5558 (PUB)")
    print("  - 里程计:     tcp://127.0.0.1:5559 (PUB)")
    print("  - SLAM 位姿:  tcp://localhost:5563 (PUB)")
    print("  - SLAM 地图:  tcp://localhost:5564 (PUB)")
    print("  - 激光雷达:   tcp://localhost:5565 (PUB)")
    print()
    print("接下来可启动 GoalFollowApp:")
    print("  python -m navigation.applications.goal_follow --goal-x 2.0 --goal-y 0.0")
    print("=" * 60)
    print()
    print("按 Ctrl+C 停止本脚本（各服务窗口需手动关闭）")

    # 主线程保持运行，直到用户按 Ctrl+C
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[退出] 用户中断")
        sys.exit(0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[退出] 已取消")
        sys.exit(0)
