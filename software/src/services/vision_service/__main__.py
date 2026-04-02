"""
Vision Service 启动入口

Usage:
    python -m services.vision_service
    python -m services.vision_service --display
    python -m services.vision_service --addr tcp://*:5560 --device 0
    python -m services.vision_service --addr tcp://*:5562 --device 1
"""
import argparse
from services.vision_service import VisionService


def main():
    parser = argparse.ArgumentParser(description='HomeBot Vision Service')
    parser.add_argument('--display', action='store_true', help='Show video window')
    parser.add_argument('--addr', default=None, help='Publish address (default: tcp://*:5560)')
    parser.add_argument('--device', type=int, default=None, help='Camera device ID (default: from config)')
    args = parser.parse_args()
    
    # 构造关键字参数，仅当值不为 None 时才传递
    kwargs = {'device_id': args.device}
    if args.addr is not None:
        kwargs['pub_addr'] = args.addr
    
    service = VisionService(**kwargs)
    service.start(display=args.display)


if __name__ == '__main__':
    main()
