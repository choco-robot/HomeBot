#!/usr/bin/env python3
"""
HomeBot 视频订阅器
订阅机器人端的ZeroMQ视频话题，保存最新图像

测试命令:
python video_subscriber.py --ip 192.168.0.12 --port 5560 --timeout 10
"""

import zmq
import argparse
import time
import os
from datetime import datetime

class VideoSubscriber:
    def __init__(self, ip: str, port: int):
        """
        初始化视频订阅器
        
        Args:
            ip: 机器人IP地址
            port: ZeroMQ PUB端口
        """
        self.ip = ip
        self.port = port
        self.connect_addr = f"tcp://{ip}:{port}"
        
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.connect_addr)
        # 订阅所有消息
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        self.latest_image_data = None
        self.latest_timestamp = None
        print(f"[OK] 已连接到 {self.connect_addr}")
    
    def receive_frame(self, timeout_ms: int = 5000) -> bytes:
        """
        接收一帧图像
        
        Protocol: 消息格式是 [帧编号, JPEG图像数据]
        Returns:
            图像二进制数据，如果超时返回None
        """
        try:
            poller = zmq.Poller()
            poller.register(self.socket, zmq.POLLIN)
            events = poller.poll(timeout_ms)
            
            if events:
                # 接收多部分消息
                messages = self.socket.recv_multipart()
                if len(messages) >= 2:
                    # 最后一个部分是图像数据
                    image_data = messages[-1]
                    self.latest_image_data = image_data
                    self.latest_timestamp = datetime.now()
                    return image_data
                elif len(messages) == 1:
                    image_data = messages[0]
                    self.latest_image_data = image_data
                    self.latest_timestamp = datetime.now()
                    return image_data
            else:
                return None
        except Exception as e:
            print(f"[ERROR] 接收帧错误: {e}")
            return None
    
    def get_latest_frame(self) -> bytes:
        """获取最新接收的帧"""
        return self.latest_image_data
    
    def save_latest_frame(self, output_path: str = None) -> str:
        """
        保存最新帧到文件
        
        Args:
            output_path: 输出路径，None则自动生成文件名
        
        Returns:
            保存的文件路径，失败返回None
        """
        if self.latest_image_data is None:
            print("[ERROR] 没有接收到图像数据")
            return None
        
        if output_path is None:
            timestamp = self.latest_timestamp.strftime("%Y%m%d_%H%M%S")
            output_path = f"homebot_frame_{timestamp}.jpg"
        
        try:
            with open(output_path, 'wb') as f:
                f.write(self.latest_image_data)
            print(f"[OK] 图像已保存到: {output_path}")
            return output_path
        except Exception as e:
            print(f"[ERROR] 保存图像失败: {e}")
            return None
    
    def wait_for_frame(self, timeout_seconds: float = 10.0) -> bytes:
        """
        等待并接收第一帧
        
        Args:
            timeout_seconds: 超时时间（秒）
        
        Returns:
            图像数据，超时返回None
        """
        start_time = time.time()
        while time.time() - start_time < timeout_seconds:
            frame = self.receive_frame()
            if frame is not None:
                return frame
            time.sleep(0.01)
        print(f"[TIMEOUT] 超时，{timeout_seconds} 秒内未接收到图像")
        return None
    
    def close(self):
        """关闭连接"""
        self.socket.close()
        self.context.term()
        print("[DISCONNECT] 连接已关闭")


def main():
    parser = argparse.ArgumentParser(description='HomeBot 视频订阅器')
    parser.add_argument('--ip', type=str, default='192.168.0.12', help='机器人IP地址')
    parser.add_argument('--port', type=int, default=5560, help='ZeroMQ PUB端口')
    parser.add_argument('--timeout', type=float, default=10.0, help='等待超时（秒）')
    parser.add_argument('--output', type=str, default=None, help='输出文件路径')
    parser.add_argument('--keep-receiving', action='store_true', help='持续接收并保存所有帧')
    parser.add_argument('--output-dir', type=str, default='.', help='持续接收时保存目录')
    
    args = parser.parse_args()
    
    subscriber = VideoSubscriber(args.ip, args.port)
    
    try:
        if args.keep_receiving:
            # 持续接收模式
            os.makedirs(args.output_dir, exist_ok=True)
            print(f"[START] 持续接收模式，图像将保存到: {args.output_dir}")
            print("按 Ctrl+C 停止")
            
            while True:
                frame = subscriber.receive_frame(timeout_ms=1000)
                if frame is not None:
                    subscriber.save_latest_frame(os.path.join(args.output_dir, f"frame_{int(time.time()*1000)}.jpg"))
                time.sleep(0.01)
        else:
            # 只接收一帧
            print(f"[WAIT] 等待图像... (超时: {args.timeout}秒)")
            frame = subscriber.wait_for_frame(args.timeout)
            if frame is not None:
                output_path = subscriber.save_latest_frame(args.output)
                print(f"[INFO] 图像大小: {len(frame)} 字节")
    except KeyboardInterrupt:
        print("\n[INTERRUPT] 用户中断")
    finally:
        subscriber.close()


if __name__ == "__main__":
    main()
