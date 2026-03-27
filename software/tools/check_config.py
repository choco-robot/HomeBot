#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""配置检查工具

检查 HomeBot 的各项配置是否正确，特别是 API 密钥

用法:
    python tools/check_config.py
    python tools/check_config.py --secrets-only
    python tools/check_config.py --verbose
"""
import sys
import argparse
from pathlib import Path

# 添加src到路径
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from configs import check_secrets, get_config


def main():
    parser = argparse.ArgumentParser(description="HomeBot 配置检查工具")
    parser.add_argument(
        "--secrets-only",
        action="store_true",
        help="仅检查 API 密钥配置"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="显示详细配置信息"
    )
    args = parser.parse_args()
    
    print("\n[检查] HomeBot 配置检查\n")
    
    # 检查密钥
    secrets_status = check_secrets(verbose=True)
    
    if args.secrets_only:
        # 检查是否所有必要密钥都已配置
        all_ok = all(
            secrets_status.get(svc, {}).get("configured", False)
            for svc in ["tts", "llm", "vision"]
        )
        sys.exit(0 if all_ok else 1)
    
    # 显示详细配置
    if args.verbose:
        print("\n[详情] 系统配置详情\n")
        print("-" * 50)
        
        config = get_config()
        
        print(f"底盘串口: {config.chassis.serial_port}")
        print(f"机械臂串口: {config.arm.serial_port}")
        print(f"ZeroMQ 底盘地址: {config.zmq.chassis_service_addr}")
        print(f"ZeroMQ 视觉地址: {config.zmq.vision_pub_addr}")
        print(f"摄像头设备: {config.camera.device_id} ({config.camera.width}x{config.camera.height})")
        print(f"日志级别: {config.logging.level}")
        
        print("-" * 50)
    
    # 总结
    print("\n[完成] 配置检查完成\n")
    
    # 返回退出码
    tts_ok = secrets_status.get("tts", {}).get("configured", False)
    llm_ok = secrets_status.get("llm", {}).get("configured", False)
    
    if not tts_ok or not llm_ok:
        print("[!] 提示: 部分 API 密钥未配置，相关功能将无法使用")
        print("   请复制 .env.example 为 .env.local 并填入你的密钥\n")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
