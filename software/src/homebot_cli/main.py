# -*- coding: utf-8 -*-
"""HomeBot CLI 主入口

命令分组:
    服务管理: start / stop / restart / status / logs
    功能调用: move
    调试:     topic echo / topic pub
    环境:     doctor / completion
"""
import os
import time

import click

from homebot_cli import __version__
from homebot_cli.services import SERVICES, CORE_SERVICES
from homebot_cli.process import (
    is_frozen, start_service, stop_service, service_running,
    read_pid, is_alive, port_listening, log_file, LOG_DIR,
)
from homebot_cli.topic import topic
from homebot_cli.move import move

CONTEXT_SETTINGS = dict(help_option_names=["-h", "--help"])


def _frozen_guard() -> bool:
    """打包版不支持服务管理，返回 True 表示应中止"""
    if is_frozen():
        click.echo("错误: 打包版本（exe）不包含服务管理功能，"
                   "请使用 status / topic / move 等远程调试命令。", err=True)
        return True
    return False


def _resolve_services(names: tuple, default_all: bool = False) -> list:
    """解析服务名参数：无参数时返回核心服务（或全部），非法名称报错"""
    if not names:
        return list(SERVICES) if default_all else list(CORE_SERVICES)
    for name in names:
        if name not in SERVICES:
            valid = ", ".join(SERVICES)
            raise click.BadParameter(f"未知服务 '{name}'，可选: {valid}")
    return list(names)


@click.group(context_settings=CONTEXT_SETTINGS, invoke_without_command=True)
@click.version_option(__version__, prog_name="homebot")
def cli():
    """HomeBot 命令行工具：服务管理 / 功能调用 / 状态查看 / 调试"""
    ctx = click.get_current_context()
    if ctx.invoked_subcommand is None:
        click.echo(ctx.get_help())


# ============ 服务管理 ============

@cli.command()
@click.argument("services", nargs=-1)
@click.option("--all", "start_all", is_flag=True, help="启动全部服务（含语音）")
def start(services: tuple, start_all: bool):
    """后台启动服务（无参数时启动核心服务: bus motion vision web）"""
    if _frozen_guard():
        raise SystemExit(1)
    names = _resolve_services(services, default_all=start_all)
    for name in names:
        info = SERVICES[name]
        if service_running(name):
            click.echo(f"[跳过] {name} 已在运行 (PID {read_pid(name)})")
            continue
        pid = start_service(info)
        click.echo(f"[启动] {name:<12} PID {pid:<8} {info.desc}  日志: {log_file(name)}")


@cli.command()
@click.argument("services", nargs=-1)
def stop(services: tuple):
    """停止服务（无参数时停止全部已注册服务）"""
    if _frozen_guard():
        raise SystemExit(1)
    names = _resolve_services(services, default_all=True)
    for name in names:
        if stop_service(name):
            click.echo(f"[停止] {name}")
        else:
            click.echo(f"[跳过] {name} 未运行（或无 PID 记录）")


@cli.command()
@click.argument("services", nargs=-1)
def restart(services: tuple):
    """重启服务（无参数时重启核心服务）"""
    if _frozen_guard():
        raise SystemExit(1)
    names = _resolve_services(services)
    for name in names:
        stop_service(name)
        info = SERVICES[name]
        pid = start_service(info)
        click.echo(f"[重启] {name:<12} PID {pid}")


@cli.command()
def status():
    """查看各服务运行状态（进程 + 端口）"""
    click.echo(f"{'服务':<12} {'状态':<8} {'PID':<8} {'端口':<20} 说明")
    click.echo("-" * 70)
    for name, info in SERVICES.items():
        pid = read_pid(name)
        alive = pid is not None and is_alive(pid)
        if alive:
            state = click.style("运行中", fg="green")
        elif port_listening_any(info.ports):
            state = click.style("端口占用", fg="yellow")  # 端口在监听但非本 CLI 启动
        else:
            state = click.style("已停止", fg="red")
        pid_str = str(pid) if alive else "-"
        ports = " ".join(
            f"{p}{'*' if port_listening(p) else ''}" for p in info.ports
        ) or "-"
        click.echo(f"{name:<12} {state:<16} {pid_str:<8} {ports:<20} {info.desc}")
    click.echo("-" * 70)
    click.echo("端口后的 * 表示该端口正在监听")


def port_listening_any(ports) -> bool:
    return any(port_listening(p) for p in ports)


@cli.command()
@click.argument("service")
@click.option("--follow", "-f", is_flag=True, help="持续跟踪日志输出")
@click.option("--lines", "-n", default=50, show_default=True, help="显示最后 N 行")
def logs(service: str, follow: bool, lines: int):
    """查看服务日志（homebot start 启动的服务）"""
    if service not in SERVICES:
        valid = ", ".join(SERVICES)
        raise click.BadParameter(f"未知服务 '{service}'，可选: {valid}")
    path = log_file(service)
    if not path.exists():
        click.echo(f"日志文件不存在: {path}（服务可能不是通过 homebot start 启动的）", err=True)
        raise SystemExit(1)

    content = path.read_text(encoding="utf-8", errors="replace").splitlines()
    for line in content[-lines:]:
        click.echo(line)

    if follow:
        click.echo(f"--- 跟踪 {path} (Ctrl+C 退出) ---", err=True)
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as f:
                f.seek(0, os.SEEK_END)
                while True:
                    line = f.readline()
                    if line:
                        click.echo(line, nl=False)
                    else:
                        time.sleep(0.3)
        except KeyboardInterrupt:
            pass


# ============ 功能调用 ============

cli.add_command(move)
cli.add_command(topic)


# ============ 环境 ============

@cli.command()
def doctor():
    """环境检查：Python / 配置 / 密钥 / 串口 / 模型 / 端口"""
    ok = click.style("OK", fg="green")
    warn = click.style("WARN", fg="yellow")
    fail = click.style("FAIL", fg="red")

    def report(status: str, item: str, detail: str = ""):
        click.echo(f"[{status}] {item}" + (f" - {detail}" if detail else ""))

    import sys
    report(ok, "Python", sys.version.split()[0])

    # 配置加载
    try:
        from configs import get_config
        config = get_config()
        report(ok, "配置文件", "configs.config 加载成功")
    except Exception as e:
        report(fail, "配置文件", str(e))
        return

    # 密钥文件
    from homebot_cli.process import SOFTWARE_DIR
    env_local = SOFTWARE_DIR / ".env.local"
    if env_local.exists():
        report(ok, "密钥文件", str(env_local))
    else:
        report(warn, "密钥文件", ".env.local 不存在，TTS/LLM 功能不可用")

    # 串口
    try:
        from serial.tools import list_ports
        available = {p.device for p in list_ports.comports()}
        chassis_port = config.chassis.serial_port
        if chassis_port in available:
            report(ok, "底盘串口", chassis_port)
        else:
            report(warn, "底盘串口",
                   f"{chassis_port} 不在可用列表: {sorted(available) or '（无串口设备）'}")
    except Exception as e:
        report(warn, "串口检测", f"无法检测: {e}")

    # 模型文件
    model = SOFTWARE_DIR / config.human_follow.model_path
    if model.exists():
        report(ok, "YOLO 模型", config.human_follow.model_path)
    else:
        report(warn, "YOLO 模型",
               f"{config.human_follow.model_path} 不存在，运行 tools/download_models.py 下载")

    # 核心端口占用情况
    for name, info in SERVICES.items():
        for port in info.ports:
            if port_listening(port):
                report(ok, f"端口 {port}", f"{name} 正在监听")
            else:
                report(warn, f"端口 {port}", f"{name} 未监听")

    click.echo()
    click.echo(f"日志目录: {LOG_DIR}")


@cli.command()
@click.option("--shell", "shell_type",
              type=click.Choice(["bash", "zsh", "fish"]), default="bash",
              show_default=True, help="目标 shell 类型")
def completion(shell_type: str):
    """输出 tab 补全激活方法

    click 内置支持 bash/zsh/fish；Windows 下推荐使用 Git Bash。
    PowerShell / cmd 暂不支持。
    """
    prog = "homebot"
    env_var = "_HOMEBOT_COMPLETE"
    click.echo(f"在当前 shell 激活补全（{shell_type}）:")
    click.echo()
    if shell_type == "fish":
        click.echo(f"    {env_var}=fish_source {prog} | source")
    else:
        click.echo(f'    eval "$({env_var}={shell_type}_source {prog})"')
    click.echo()
    click.echo("如需永久生效，将上面一行追加到 shell 配置文件：")
    profiles = {
        "bash": "~/.bashrc  (Windows 下 Git Bash 同样使用 ~/.bashrc)",
        "zsh": "~/.zshrc",
        "fish": "~/.config/fish/config.fish",
    }
    click.echo(f"    {profiles[shell_type]}")
    click.echo()
    click.echo("注意: PowerShell / cmd 不支持 click 的内置补全，Windows 下请使用 Git Bash。")


if __name__ == "__main__":
    cli()
