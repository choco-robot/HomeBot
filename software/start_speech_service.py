#!/usr/bin/env python3
"""
HomeBot 语音服务一键启动脚本

自动启动 WakeupASR 服务和语音交互应用

使用方法:
    cd software
    python start_speech_service.py
    
选项:
    --wakeup-only    仅启动 WakeupASR 服务
    --app-only       仅启动语音交互应用
    --check-models   检查模型文件是否存在
"""

import os
import sys
import time
import signal
import subprocess
import platform
from pathlib import Path


# 服务配置
SERVICES = {
    "wakeup_asr": {
        "name": "WakeupASR Service",
        "module": "services.speech_service",
        "args": ["wakeup"],
        "desc": "语音唤醒 + ASR识别 (PUB模式)"
    },
    "speech_app": {
        "name": "Speech Interaction",
        "module": "applications.speech_interaction",
        "args": [],
        "desc": "语音对话 + TTS (SUB模式)"
    }
}


def print_header():
    """打印启动标题"""
    print("=" * 60)
    print("   HomeBot 语音服务启动器")
    print("=" * 60)
    print()


def check_models():
    """检查模型文件是否存在"""
    script_dir = Path(__file__).parent
    models_dir = script_dir / "models"
    
    asr_dir = models_dir / "asr"
    wakeup_dir = models_dir / "wakeup"
    
    print("[检查] 检查语音模型文件...")
    print()
    
    all_exist = True
    
    # 检查 ASR 模型
    asr_files = ["encoder.int8.onnx", "decoder.onnx", "joiner.int8.onnx", "tokens.txt"]
    asr_exists = all((asr_dir / f).exists() for f in asr_files)
    
    if asr_exists:
        print(f"  [OK] ASR 模型 ({asr_dir})")
    else:
        print(f"  [缺失] ASR 模型")
        print(f"        路径: {asr_dir}")
        print(f"        运行: python tools/download_speech_models.py")
        all_exist = False
    
    # 检查唤醒模型
    wakeup_files = [
        "encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx",
        "decoder-epoch-13-avg-2-chunk-16-left-64.onnx",
        "joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx",
        "tokens.txt",
        "keywords.txt"
    ]
    wakeup_exists = all((wakeup_dir / f).exists() for f in wakeup_files)
    
    if wakeup_exists:
        print(f"  [OK] 唤醒模型 ({wakeup_dir})")
    else:
        print(f"  [缺失] 唤醒模型")
        print(f"        路径: {wakeup_dir}")
        print(f"        运行: python tools/download_speech_models.py")
        all_exist = False
    
    print()
    return all_exist


def start_service(service_key: str, src_dir: Path, wait: bool = False):
    """启动单个服务
    
    Args:
        service_key: 服务配置键
        src_dir: src 目录路径
        wait: 是否等待用户按键
    """
    svc = SERVICES[service_key]
    print(f"[启动] {svc['name']}...")
    print(f"       {svc['desc']}")
    
    cmd = [sys.executable, "-m", svc["module"]] + svc["args"]
    
    # 在新窗口中启动
    if platform.system() == "Windows":
        cmd_parts = [f'"{sys.executable}"', "-m", svc["module"]] + svc["args"]
        cmd_str = " ".join(cmd_parts)
        
        subprocess.Popen(
            f'start "{svc["name"]}" cmd /k "cd /d "{src_dir}" && {cmd_str}"',
            shell=True
        )
    elif platform.system() == "Darwin":
        script = f'''
        tell application "Terminal"
            do script "cd {src_dir} && {sys.executable} -m {svc["module"]} {' '.join(svc["args"])}"
            set custom title of front window to "{svc["name"]}"
        end tell
        '''
        subprocess.Popen(["osascript", "-e", script], 
                        stdout=subprocess.DEVNULL, 
                        stderr=subprocess.DEVNULL)
    else:
        # Linux
        cmd_str = f"cd {src_dir} && {sys.executable} -m {svc['module']} {' '.join(svc['args'])}"
        
        terminals = [
            ("gnome-terminal", ["--title", svc["name"], "--", "bash", "-c", f"{cmd_str}; exec bash"]),
            ("konsole", ["--new-tab", "-p", f"tabtitle={svc['name']}", "-e", "bash", "-c", f"{cmd_str}; exec bash"]),
            ("xterm", ["-T", svc["name"], "-e", "bash", "-c", f"{cmd_str}; exec bash"]),
        ]
        
        started = False
        for term, args in terminals:
            if subprocess.run(["which", term], capture_output=True).returncode == 0:
                subprocess.Popen([term] + args, 
                                stdout=subprocess.DEVNULL, 
                                stderr=subprocess.DEVNULL)
                started = True
                break
        
        if not started:
            print(f"       [注意] 没有找到终端模拟器，后台运行...")
            subprocess.Popen(cmd, cwd=src_dir, 
                           stdout=subprocess.DEVNULL, 
                           stderr=subprocess.DEVNULL)
    
    if wait:
        input("\n按 Enter 键继续...")
    else:
        time.sleep(2)


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="启动 HomeBot 语音服务")
    parser.add_argument(
        "--wakeup-only",
        action="store_true",
        help="仅启动 WakeupASR 服务"
    )
    parser.add_argument(
        "--app-only",
        action="store_true",
        help="仅启动语音交互应用"
    )
    parser.add_argument(
        "--check-models",
        action="store_true",
        help="仅检查模型文件"
    )
    parser.add_argument(
        "--skip-model-check",
        action="store_true",
        help="跳过模型检查"
    )
    
    args = parser.parse_args()
    
    print_header()
    
    # 检查模型
    if args.check_models:
        if check_models():
            print("[OK] 所有模型文件已就绪")
            return 0
        else:
            print("[提示] 请运行以下命令下载模型:")
            print("      python tools/download_speech_models.py")
            return 1
    
    # 切换到 src 目录
    script_dir = Path(__file__).parent
    src_dir = script_dir / "src"
    
    if not src_dir.exists():
        print(f"[错误] 目录不存在: {src_dir}")
        return 1
    
    # 检查模型
    if not args.skip_model_check:
        if not check_models():
            print()
            response = input("模型文件缺失，是否继续启动? [y/N]: ").strip().lower()
            if response != 'y':
                print()
                print("[提示] 请先下载模型:")
                print("      python tools/download_speech_models.py")
                return 1
            print()
    
    # 启动服务
    if args.wakeup_only:
        print("[模式] 仅启动 WakeupASR 服务\n")
        start_service("wakeup_asr", src_dir)
        
    elif args.app_only:
        print("[模式] 仅启动语音交互应用\n")
        start_service("speech_app", src_dir)
        
    else:
        print("[模式] 启动完整语音服务\n")
        
        # 先启动 WakeupASR 服务
        start_service("wakeup_asr", src_dir)
        
        # 等待一下让服务准备好
        print()
        print("等待 WakeupASR 服务初始化...")
        time.sleep(3)
        
        # 启动语音交互应用
        start_service("speech_app", src_dir, wait=False)
    
    # 打印完成信息
    print()
    print("=" * 60)
    print("[OK] 语音服务已启动!")
    print()
    print("使用说明:")
    print("  1. 说出唤醒词 '你好小白' 激活机器人")
    print("  2. 听到 '我在' 提示音后，说出指令如:")
    print("     - '向前走一米'")
    print("     - '向左转90度'")
    print("     - '停止'")
    print()
    print("注意:")
    print("  - 确保底盘服务已启动 (python start_system.py)")
    print("  - 每个窗口可以单独关闭")
    print("  - 按 Ctrl+C 可以停止服务")
    print("=" * 60)
    
    input("\n按 Enter 键退出此窗口...")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n[退出] 用户中断")
        sys.exit(0)
