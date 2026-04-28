"""
Vision Service 启动入口

Usage:
    python -m services.vision_service
    python -m services.vision_service --display
    python -m services.vision_service --webcamera --url rtsp://192.168.1.100/live
"""
import argparse
from services.vision_service import VisionService, WebCameraVisionService
from configs.config import get_config


def main():
    parser = argparse.ArgumentParser(description='HomeBot Vision Service')
    parser.add_argument('--display', action='store_true', help='Show video window')
    parser.add_argument('--addr', default=None, help='Publish address (default: tcp://*:5560)')
    parser.add_argument('--webcamera', action='store_true', help='Use network camera (WebCamera) instead of local camera')
    parser.add_argument('--url', default=None, help='WebCamera URL (overrides config)')
    args = parser.parse_args()
    
    if args.webcamera or args.url:
        config = get_config()
        # 如果命令行指定了 URL，覆盖配置
        if args.url:
            config.webcamera.url = args.url
            config.webcamera.enabled = True
        service = WebCameraVisionService(pub_addr=args.addr, config=config) if args.addr else WebCameraVisionService(config=config)
    else:
        service = VisionService(pub_addr=args.addr) if args.addr else VisionService()
    
    try:
        service.start(display=args.display)
    finally:
        # 确保进程退出时释放资源（对单客户端设备如 ESP32 尤其重要）
        service.stop()


if __name__ == '__main__':
    main()
