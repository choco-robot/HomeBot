#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HomeBot System Launcher
跨平台启动器，支持 Windows/Linux/macOS
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
        "name": "Motion Service",
        "module": "services.motion_service",
        "port": 5556,  # 底盘服务端口
        "port2": 5557,  # 机械臂服务端口
        "desc": "Chassis + Arm Service",
        "args": ["--service", "both"]
    },
    {
        "name": "Vision Service",
        "module": "services.vision_service",
        "port": 5560,
        "desc": "Vision Service"
    },
    {
        "name": "Web Control",
        "module": "applications.remote_control",
        "port": 5000,
        "desc": "Web Server"
    }
]


def print_header():
    """打印启动标题"""
    print("=" * 50)
    print("   HomeBot System Launcher")
    print("=" * 50)
    print()


def check_port(port):
    """检查端口是否被占用"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(('127.0.0.1', port))
            return False  # 端口可用
        except socket.error:
            return True  # 端口被占用


def get_process_on_port(port):
    """获取占用端口的进程 PID（跨平台）"""
    try:
        if platform.system() == "Windows":
            # Windows: 使用 netstat 和 findstr
            result = subprocess.run(
                ["netstat", "-ano"],
                capture_output=True,
                text=True,
                encoding='utf-8',
                errors='ignore'
            )
            for line in result.stdout.split('\n'):
                if f":{port}" in line and "LISTENING" in line:
                    parts = line.strip().split()
                    if len(parts) >= 5:
                        return int(parts[-1])
        else:
            # Linux/macOS: 使用 lsof
            result = subprocess.run(
                ["lsof", "-i", f"TCP:{port}", "-sTCP:LISTEN", "-t"],
                capture_output=True,
                text=True
            )
            if result.returncode == 0 and result.stdout.strip():
                return int(result.stdout.strip().split('\n')[0])
    except Exception:
        pass
    return None


def kill_process(pid):
    """终止进程"""
    try:
        if platform.system() == "Windows":
            subprocess.run(["taskkill", "/F", "/PID", str(pid)], 
                         capture_output=True, check=False)
        else:
            os.kill(pid, signal.SIGKILL)
        return True
    except Exception:
        return False


def check_ports():
    """检查所有端口状态"""
    print("[Check] Checking required ports...")
    print()
    
    occupied = []
    for svc in SERVICES:
        # 检查主端口
        if check_port(svc["port"]):
            print(f"[WARN] Port {svc['port']} is occupied ({svc['desc']})")
            occupied.append(svc)
        else:
            print(f"[OK] Port {svc['port']} is available")
        
        # 检查第二端口（如果有）
        if "port2" in svc:
            if check_port(svc["port2"]):
                print(f"[WARN] Port {svc['port2']} is occupied ({svc['desc']} - Arm)")
                if svc not in occupied:
                    occupied.append(svc)
            else:
                print(f"[OK] Port {svc['port2']} is available")
    
    print()
    return occupied


def prompt_user(occupied):
    """提示用户处理被占用的端口"""
    print("=" * 50)
    print("[WARNING] Some ports are already in use!")
    print()
    print("This may cause services to fail starting.")
    print()
    print("Options:")
    print("   1. Kill occupying processes and continue")
    print("   2. Continue anyway (may cause errors)")
    print("   3. Exit")
    print("=" * 50)
    print()
    
    try:
        choice = input("Select option [1-3]: ").strip()
    except (KeyboardInterrupt, EOFError):
        print("\n[Exit] User cancelled.")
        return False
    
    if choice == "3":
        print("[Exit] User cancelled.")
        return False
    elif choice == "2":
        print("[Continue] Starting with warnings...")
        print()
        return True
    elif choice == "1":
        print("[Action] Killing processes on occupied ports...")
        for svc in occupied:
            pid = get_process_on_port(svc["port"])
            if pid:
                print(f"   Killing PID {pid} on port {svc['port']}...")
                kill_process(pid)
        print("[OK] Cleanup complete")
        time.sleep(2)
        print()
        return True
    else:
        print("[Exit] Invalid option.")
        return False


def start_service(svc, src_dir):
    """启动单个服务"""
    print(f"[Start] Starting {svc['name']}...")
    
    cmd = [sys.executable, "-m", svc["module"]]
    
    # 添加额外参数（如果有）
    if "args" in svc:
        cmd.extend(svc["args"])
    
    # 在新窗口中启动（跨平台）
    if platform.system() == "Windows":
        # Windows: 使用 start 命令
        cmd_str = f"cd /d {src_dir} && python -m {svc['module']}"
        subprocess.Popen(
            f'start "{svc["name"]}" cmd /k {cmd_str}',
            shell=True
        )
    elif platform.system() == "Darwin":
        # macOS: 使用 osascript
        script = f'''
        tell application "Terminal"
            do script "cd {src_dir} && {sys.executable} -m {svc['module']}"
            set custom title of front window to "{svc['name']}"
        end tell
        '''
        subprocess.Popen(["osascript", "-e", script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        # Linux: 尝试使用各种终端
        cmd_str = f"cd {src_dir} && {sys.executable} -m {svc['module']}"
        
        terminals = [
            ("gnome-terminal", ["--title", svc["name"], "--", "bash", "-c", f"{cmd_str}; exec bash"]),
            ("konsole", ["--new-tab", "-p", f"tabtitle={svc['name']}", "-e", "bash", "-c", f"{cmd_str}; exec bash"]),
            ("xterm", ["-T", svc["name"], "-e", "bash", "-c", f"{cmd_str}; exec bash"]),
        ]
        
        started = False
        for term, args in terminals:
            if subprocess.run(["which", term], capture_output=True).returncode == 0:
                subprocess.Popen([term] + args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                started = True
                break
        
        if not started:
            # 没有可用的终端模拟器，后台运行
            print(f"   [Note] No terminal emulator found, running {svc['name']} in background...")
            subprocess.Popen(cmd, cwd=src_dir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    """主函数"""
    print_header()
    
    # 检查端口
    occupied = check_ports()
    
    # 如果有端口被占用，询问用户
    if occupied:
        if not prompt_user(occupied):
            input("\nPress Enter to exit...")
            sys.exit(1)
    else:
        print("[OK] All ports are available")
        print()
    
    # 切换到 src 目录
    script_dir = Path(__file__).parent
    src_dir = script_dir / "src"
    
    if not src_dir.exists():
        print(f"[Error] Directory not found: {src_dir}")
        input("\nPress Enter to exit...")
        sys.exit(1)
    
    # 启动所有服务
    for svc in SERVICES:
        start_service(svc, str(src_dir))
        if svc != SERVICES[-1]:  # 不是最后一个服务，等待一下
            time.sleep(2)
    
    # 打印完成信息
    print()
    print("=" * 50)
    print("[OK] All services started!")
    print()
    print("Services:")
    for svc in SERVICES:
        if svc["name"] == "Web Control":
            print(f"   - {svc['name']} (Flask: http://0.0.0.0:{svc['port']})")
        elif svc["name"] == "Motion Service":
            print(f"   - {svc['name']} (ZeroMQ: tcp://127.0.0.1:{svc['port']} & :{svc['port2']})")
        else:
            proto = "Camera" if "Vision" in svc["name"] else "ZeroMQ"
            print(f"   - {svc['name']} ({proto}: tcp://127.0.0.1:{svc['port']})")
    print()
    print("URL: http://localhost:5000")
    print("Video: http://localhost:5000/video_feed")
    print("=" * 50)
    
    input("\nPress Enter to exit...")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[Exit] Interrupted by user.")
        sys.exit(0)
