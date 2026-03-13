#!/usr/bin/env python3
"""
HomeBot 视觉查询技能
执行流程：
1. 订阅机器人视频服务，获取一张最新图片
2. 保存图片到本地
3. 返回图片文件路径（供发送和图像描述）

使用方式：
python what_did_robot_see.py --ip 192.168.0.12 --port 5560 --timeout 10
"""

import argparse
from video_subscriber import VideoSubscriber
import os
from datetime import datetime


def capture_latest_frame(robot_ip: str = "192.168.0.12", port: int = 5560, timeout: float = 10.0, output_dir: str = ".") -> str:
    """
    捕获机器人最新摄像头画面
    
    Args:
        robot_ip: 机器人IP地址
        port: ZeroMQ PUB端口
        timeout: 等待超时（秒）
        output_dir: 输出目录
    
    Returns:
        保存的图片文件路径，失败返回None
    """
    subscriber = VideoSubscriber(robot_ip, port)
    
    try:
        frame = subscriber.wait_for_frame(timeout)
        if frame is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = os.path.join(output_dir, f"homebot_capture_{timestamp}.jpg")
            saved_path = subscriber.save_latest_frame(output_path)
            return saved_path
        else:
            return None
    finally:
        subscriber.close()


def main():
    parser = argparse.ArgumentParser(description='HomeBot 视觉查询 - 捕获最新画面')
    parser.add_argument('--ip', type=str, default='192.168.0.12', help='机器人IP地址')
    parser.add_argument('--port', type=int, default=5560, help='ZeroMQ PUB端口')
    parser.add_argument('--timeout', type=float, default=10.0, help='等待超时（秒）')
    parser.add_argument('--output-dir', type=str, default='.', help='输出目录')
    
    args = parser.parse_args()
    
    print(f"[INFO] 正在连接机器人 {args.ip}:{args.port}...")
    saved_path = capture_latest_frame(args.ip, args.port, args.timeout, args.output_dir)
    
    if saved_path:
        print(f"[OK] 画面捕获成功，保存到: {saved_path}")
        print(f"[INFO] 文件大小: {os.path.getsize(saved_path)} 字节")
    else:
        print("[ERROR] 画面捕获失败")
        exit(1)


if __name__ == "__main__":
    main()
