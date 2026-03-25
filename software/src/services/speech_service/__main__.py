"""语音服务入口

支持启动不同服务:
    python -m services.speech_service wakeup    # 启动唤醒+ASR服务
    python -m services.speech_service           # 默认启动唤醒+ASR服务

注意: TTS 由应用层直接调用，无需独立服务
"""
import sys
import argparse

from common.logging import get_logger
from services.speech_service.wakeup_asr_service import main as wakeup_main

logger = get_logger(__name__)


def main():
    parser = argparse.ArgumentParser(description="语音服务")
    parser.add_argument(
        "service",
        nargs="?",
        default="wakeup",
        choices=["wakeup", "wakeup_asr"],
        help="要启动的服务类型 (默认: wakeup)"
    )
    
    args = parser.parse_args()
    
    if args.service in ["wakeup", "wakeup_asr"]:
        logger.info("启动唤醒+ASR服务...")
        wakeup_main()
    else:
        logger.error(f"未知服务类型: {args.service}")
        sys.exit(1)


if __name__ == "__main__":
    main()
