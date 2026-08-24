# -*- coding: utf-8 -*-
"""跨平台进程管理：后台启动 / 停止 / 存活检测 / 端口检测

所有平台差异封装在本模块：
- 启动: Windows 用 DETACHED_PROCESS，POSIX 用 start_new_session
- 停止: Windows 用 taskkill /T（进程树），POSIX 用 killpg（先 SIGTERM 后 SIGKILL）
- 存活检测: Windows 用 OpenProcess + GetExitCodeProcess，POSIX 用 os.kill(pid, 0)
"""
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

from homebot_cli.services import ServiceInfo

# 路径解析：本文件位于 software/src/homebot_cli/process.py
SRC_DIR = Path(__file__).resolve().parent.parent          # software/src
SOFTWARE_DIR = SRC_DIR.parent                              # software
LOG_DIR = SOFTWARE_DIR / "logs"
RUN_DIR = SOFTWARE_DIR / "cache" / "run"


def is_frozen() -> bool:
    """是否为 PyInstaller 打包的可执行文件"""
    return getattr(sys, "frozen", False)


def pid_file(name: str) -> Path:
    return RUN_DIR / f"{name}.pid"


def log_file(name: str) -> Path:
    return LOG_DIR / f"{name}.log"


def read_pid(name: str) -> Optional[int]:
    """读取服务 PID 文件，不存在或内容非法返回 None"""
    path = pid_file(name)
    try:
        return int(path.read_text().strip())
    except (OSError, ValueError):
        return None


def is_alive(pid: int) -> bool:
    """检测进程是否存活"""
    if pid <= 0:
        return False
    if os.name == "nt":
        import ctypes

        PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
        STILL_ACTIVE = 259
        handle = ctypes.windll.kernel32.OpenProcess(
            PROCESS_QUERY_LIMITED_INFORMATION, False, pid
        )
        if not handle:
            return False
        try:
            exit_code = ctypes.c_ulong()
            if not ctypes.windll.kernel32.GetExitCodeProcess(handle, ctypes.byref(exit_code)):
                return False
            return exit_code.value == STILL_ACTIVE
        finally:
            ctypes.windll.kernel32.CloseHandle(handle)
    else:
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False


def port_listening(port: int, host: str = "127.0.0.1", timeout: float = 0.3) -> bool:
    """检测 TCP 端口是否有进程在监听"""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def service_running(name: str) -> bool:
    """服务是否正在运行（PID 存活）"""
    pid = read_pid(name)
    return pid is not None and is_alive(pid)


def start_service(info: ServiceInfo) -> int:
    """后台启动服务，返回 PID

    stdout/stderr 重定向到 software/logs/<name>.log，不产生新窗口。
    若服务已在运行（PID 存活）则直接返回现有 PID。
    """
    existing = read_pid(info.name)
    if existing and is_alive(existing):
        return existing

    RUN_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    logf = open(log_file(info.name), "a", encoding="utf-8", errors="replace")
    cmd = [sys.executable, "-m", info.module, *info.args]

    if os.name == "nt":
        # CREATE_NO_WINDOW 分配一个隐藏控制台：Python 3.13 的 venv 重定向器
        # 会再拉起 base 解释器（孙进程），孙进程默认继承父控制台——
        # 若用 DETACHED_PROCESS 父进程无控制台，孙进程会新建一个可见的空窗口
        creationflags = (
            subprocess.CREATE_NEW_PROCESS_GROUP
            | getattr(subprocess, "CREATE_NO_WINDOW", 0x08000000)
        )
        proc = subprocess.Popen(
            cmd, cwd=str(SRC_DIR), stdout=logf, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, creationflags=creationflags, close_fds=True,
        )
    else:
        proc = subprocess.Popen(
            cmd, cwd=str(SRC_DIR), stdout=logf, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True, close_fds=True,
        )

    pid_file(info.name).write_text(str(proc.pid))
    return proc.pid


def stop_service(name: str, wait: float = 3.0) -> bool:
    """停止服务进程（含子进程树），返回是否成功停止

    服务不是本 CLI 启动（无 PID 文件或进程已退出）时返回 False。
    """
    pid = read_pid(name)
    if pid is None or not is_alive(pid):
        try:
            pid_file(name).unlink(missing_ok=True)
        except OSError:
            pass
        return False

    if os.name == "nt":
        # /T 终止进程树，/F 强制
        subprocess.run(
            ["taskkill", "/PID", str(pid), "/T", "/F"],
            capture_output=True, check=False,
        )
    else:
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except OSError:
            pass
        # 等待优雅退出，超时后 SIGKILL
        deadline = time.time() + wait
        while time.time() < deadline and is_alive(pid):
            time.sleep(0.1)
        if is_alive(pid):
            try:
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            except OSError:
                pass

    try:
        pid_file(name).unlink(missing_ok=True)
    except OSError:
        pass
    return True
