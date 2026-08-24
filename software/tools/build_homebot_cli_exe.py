#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HomeBot CLI 打包脚本

使用 PyInstaller 将 homebot_cli 打包为单文件可执行程序（homebot.exe / homebot）。
打包版本定位为控制端/调试端工具：status、topic、move 等通过网络连接机器人；
start/stop/restart/logs 服务管理命令在打包版中不可用（运行时会提示）。

用法:
    venv\\Scripts\\python.exe software/tools/build_homebot_cli_exe.py   # Windows
    venv/bin/python software/tools/build_homebot_cli_exe.py            # Linux/macOS
"""
import os
import subprocess
import sys


def main() -> None:
    tools_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.join(os.path.dirname(tools_dir), "src")
    entry = os.path.join(src_dir, "homebot_cli", "__main__.py")

    if not os.path.exists(entry):
        print(f"入口文件不存在: {entry}")
        sys.exit(1)

    cmd = [
        sys.executable, "-m", "PyInstaller",
        "--noconfirm",
        "--onefile",
        "--console",
        "--name", "homebot",
        "--paths", src_dir,
        "--hidden-import", "zmq",
        "--hidden-import", "zmq.backend.cython",
        # 排除重型依赖，CLI 本体只需要 pyzmq/click/标准库
        "--exclude-module", "opencv_python",
        "--exclude-module", "cv2",
        "--exclude-module", "flask",
        "--exclude-module", "flask_socketio",
        "--exclude-module", "ultralytics",
        "--exclude-module", "numpy",
        "--exclude-module", "filterpy",
        "--exclude-module", "sherpa_onnx",
        "--exclude-module", "sounddevice",
        "--exclude-module", "openai",
        "--exclude-module", "fastmcp",
        "--exclude-module", "websockets",
        "--exclude-module", "volcengine",
        "--exclude-module", "serial",
        entry,
    ]

    print("执行命令:")
    print(" ".join(cmd))
    print()

    try:
        subprocess.check_call(cmd)
        suffix = ".exe" if os.name == "nt" else ""
        print(f"\n打包完成，输出: dist/homebot{suffix}")
    except subprocess.CalledProcessError as e:
        print(f"\n打包失败: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
