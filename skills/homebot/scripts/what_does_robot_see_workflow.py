#!/usr/bin/env python3
"""
What Does The Robot See - 完整工作流
执行：
1. 订阅机器人视频服务获取最新图像
2. 保存图像到本地
3. 返回图像文件路径（第二步发送，第三步AI描述由Picoclaw完成）
"""

import sys
import os
import glob

# 添加homebot-controller scripts目录到路径
sys.path.append(os.path.join(os.path.dirname(__file__), '../../homebot-controller/scripts'))

from video_subscriber import VideoSubscriber
from datetime import datetime
import robot_config as config


class WhatDoesRobotSeeWorkflow:
    """完整的'机器人看到了什么'工作流"""
    
    def __init__(self, robot_ip=None, port=None, timeout=None, output_dir=None):
        self.robot_ip = robot_ip or config.ROBOT_IP
        self.port = port or config.VIDEO_PORT
        self.timeout = timeout or config.CAPTURE_TIMEOUT
        self.output_dir = output_dir or config.OUTPUT_DIR
        
        # 转换输出目录为绝对路径
        if not os.path.isabs(self.output_dir):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            self.output_dir = os.path.join(script_dir, self.output_dir)
    
    def capture(self) -> str:
        """
        第一步：捕获最新画面
        
        Returns:
            保存的图片文件路径，失败返回None
        """
        subscriber = VideoSubscriber(self.robot_ip, self.port)
        
        try:
            print(f"[INFO] 正在连接机器人 {self.robot_ip}:{self.port}...")
            frame = subscriber.wait_for_frame(self.timeout)
            
            if frame is not None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                output_path = os.path.join(self.output_dir, f"homebot_capture_{timestamp}.jpg")
                saved_path = subscriber.save_latest_frame(output_path)
                
                if saved_path:
                    file_size = os.path.getsize(saved_path)
                    print(f"[OK] 图像捕获成功")
                    print(f"[INFO] 保存位置: {saved_path}")
                    print(f"[INFO] 文件大小: {file_size} 字节")
                    return saved_path
                else:
                    print("[ERROR] 图像保存失败")
                    return None
            else:
                print("[ERROR] 获取图像超时，未收到帧")
                return None
        finally:
            subscriber.close()
    
    def get_image_info(self, image_path: str) -> dict:
        """获取图像信息"""
        if not os.path.exists(image_path):
            return None
        
        return {
            "path": image_path,
            "size_bytes": os.path.getsize(image_path),
            "timestamp": os.path.getctime(image_path),
            "filename": os.path.basename(image_path)
        }
    
    def get_latest_capture(self) -> str:
        """获取最新捕获的图像文件路径"""
        image_files = []
        for ext in ['.jpg', '.jpeg', '.png']:
            pattern = os.path.join(self.output_dir, f"*{ext}")
            image_files.extend(glob.glob(pattern))
        
        if not image_files:
            return None
        
        # 按修改时间排序，返回最新的
        image_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)
        return image_files[0]


def main():
    """命令行入口"""
    workflow = WhatDoesRobotSeeWorkflow()
    image_path = workflow.capture()
    
    if image_path:
        # 输出路径供Picoclaw调用获取
        print(f"\n--- RESULT ---")
        print(image_path)
        sys.exit(0)
    else:
        print(f"\n--- FAILED ---")
        sys.exit(1)


if __name__ == "__main__":
    main()
