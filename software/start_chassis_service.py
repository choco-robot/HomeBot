#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HomeBot 底盘服务启动器
跨平台启动器，支持 Windows/Linux/macOS
"""

import os
import sys
import subprocess
import platform
import glob
from pathlib import Path


def print_header():
    """打印启动标题"""
    print("=" * 50)
    print("   HomeBot 底盘服务启动器")
    print("=" * 50)
    print()
    print("串口从 configs/config.py 读取，默认 COM3")
    print("可通过 --port 参数临时覆盖")
    print()


def check_python():
    """检查 Python 是否可用"""
    try:
        result = subprocess.run(
            [sys.executable, "--version"],
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            return True
    except Exception:
        pass
    return False


def list_serial_ports():
    """列出可用串口（跨平台）"""
    print("[信息] 可用串口:")
    
    system = platform.system()
    ports = []
    
    if system == "Windows":
        # Windows: 使用 mode 命令
        try:
            result = subprocess.run(
                ["mode"],
                capture_output=True,
                text=True,
                encoding='utf-8',
                errors='ignore',
                shell=True
            )
            for line in result.stdout.split('\n'):
                if 'COM' in line:
                    print(f"   {line.strip()}")
                    ports.append(line.strip())
        except Exception:
            pass
    else:
        # Linux/macOS: 扫描 /dev 目录
        patterns = []
        if system == "Darwin":
            # macOS
            patterns = ['/dev/tty.usbserial*', '/dev/tty.usbmodem*', '/dev/tty.SLAB*', '/dev/tty.wch*']
        else:
            # Linux
            patterns = ['/dev/ttyUSB*', '/dev/ttyACM*']
        
        for pattern in patterns:
            matched = glob.glob(pattern)
            for port in matched:
                print(f"   {port}")
                ports.append(port)
    
    if not ports:
        print("   未检测到串口")
    
    print()


def main():
    """主函数"""
    print_header()
    
    # 检查 Python
    if not check_python():
        print("[错误] 未找到Python")
        input("\nPress Enter to exit...")
        sys.exit(1)
    
    # 列出串口
    list_serial_ports()
    
    # 解析命令行参数
    extra_args = sys.argv[1:]
    if extra_args:
        print(f"[命令行参数] {' '.join(extra_args)}")
    
    print("[启动] 正在启动底盘服务...")
    print("[提示] 按Ctrl+C停止")
    print()
    
    # 切换到 src 目录
    script_dir = Path(__file__).parent
    src_dir = script_dir / "src"
    
    if not src_dir.exists():
        print(f"[错误] 目录不存在: {src_dir}")
        input("\nPress Enter to exit...")
        sys.exit(1)
    
    # 构建命令
    cmd = [sys.executable, "-m", "services.motion_service.chassis_service"] + extra_args
    
    # 启动服务
    try:
        result = subprocess.run(cmd, cwd=str(src_dir))
        sys.exit(result.returncode)
    except KeyboardInterrupt:
        print("\n[退出] 用户中断")
        sys.exit(0)
    except Exception as e:
        print(f"[错误] 启动失败: {e}")
        input("\nPress Enter to exit...")
        sys.exit(1)


if __name__ == "__main__":
    main()
