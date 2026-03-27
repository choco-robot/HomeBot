#!/usr/bin/env python3
"""
What Does The Robot See - 完整工作流（集成火山引擎视觉分析）
执行：
1. 订阅机器人视频服务获取最新图像
2. 保存图像到本地
3. 调用火山引擎 LLM 分析图片内容
4. 返回图像路径和分析结果

使用方式：
    python what_does_robot_see_workflow.py
    python what_does_robot_see_workflow.py --no-analysis  # 仅捕获不分析
    python what_does_robot_see_workflow.py --prompt "描述图中的物体"  # 自定义提示词
"""

import sys
import os
import glob
import argparse

# 修复Windows控制台中文编码问题
if sys.platform.startswith('win'):
    # 设置控制台代码页为UTF-8
    import ctypes
    ctypes.windll.kernel32.SetConsoleOutputCP(65001)
    ctypes.windll.kernel32.SetConsoleCP(65001)
    os.system('chcp 65001 > NUL')
    # 设置Python默认编码
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')

# 添加当前目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from video_subscriber import VideoSubscriber
from datetime import datetime
import robot_config as config

# 尝试导入火山引擎视觉客户端
try:
    from volcengine_vision_client import analyze_images
    VOLCENGINE_AVAILABLE = True
except ImportError:
    VOLCENGINE_AVAILABLE = False
    print("[WARN] volcengine_vision_client 未找到，视觉分析功能不可用")


class WhatDoesRobotSeeWorkflow:
    """完整的'机器人看到了什么'工作流（集成火山引擎视觉分析）"""
    
    def __init__(self, robot_ip=None, port=None, timeout=None, output_dir=None, 
                 enable_analysis=True, prompt=None, model=None):
        self.robot_ip = robot_ip or config.ROBOT_IP
        self.port = port or config.VIDEO_PORT
        self.timeout = timeout or config.CAPTURE_TIMEOUT
        self.output_dir = output_dir or config.OUTPUT_DIR
        self.enable_analysis = enable_analysis
        self.prompt = prompt or "以一台家用机器人的视角观察图片，找出可能需要跟随的人或需要抓取/操作的物品"
        self.model = model or os.getenv("ARK_MODEL_ID", "doubao-seed-2-0-lite-260215")
        
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
    
    def analyze(self, image_path: str) -> str:
        """
        使用火山引擎 LLM 分析图片内容
        
        Args:
            image_path: 图片文件路径
            
        Returns:
            分析结果文本，失败返回 None
        """
        if not VOLCENGINE_AVAILABLE:
            print("[WARN] 火山引擎视觉分析模块不可用，跳过分析")
            return None
        
        if not os.path.exists(image_path):
            print(f"[ERROR] 图片文件不存在: {image_path}")
            return None
        
        try:
            print(f"[INFO] 正在使用火山引擎分析图片...")
            print(f"[INFO] 模型: {self.model}")
            print(f"[INFO] 提示词: {self.prompt}")
            
            result = analyze_images(
                image_paths=[image_path],
                prompt=self.prompt,
                model=self.model,
                max_tokens=2048
            )
            
            print(f"[OK] 分析完成")
            return result
        except Exception as e:
            print(f"[ERROR] 视觉分析失败: {e}")
            return None
    
    def capture_and_analyze(self) -> dict:
        """
        完整工作流：捕获 + 分析
        
        Returns:
            包含 image_path 和 analysis 的字典
        """
        result = {
            "image_path": None,
            "analysis": None,
            "success": False
        }
        
        # 第一步：捕获图像
        image_path = self.capture()
        if not image_path:
            print("[ERROR] 图像捕获失败")
            return result
        
        result["image_path"] = image_path
        result["success"] = True
        
        # 第二步：视觉分析（如果启用）
        if self.enable_analysis and VOLCENGINE_AVAILABLE:
            analysis = self.analyze(image_path)
            result["analysis"] = analysis
        elif self.enable_analysis and not VOLCENGINE_AVAILABLE:
            print("[WARN] 视觉分析已启用但模块不可用，请安装 volcenginesdkarkruntime")
        
        return result


def main():
    """命令行入口"""
    parser = argparse.ArgumentParser(
        description="HomeBot 视觉查询 - 捕获并分析机器人看到的画面",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python what_does_robot_see_workflow.py              # 捕获并分析
  python what_does_robot_see_workflow.py --no-analysis # 仅捕获图像
  python what_does_robot_see_workflow.py --prompt "图中有几个人？"
        """
    )
    
    parser.add_argument("--no-analysis", action="store_true", 
                        help="禁用视觉分析，仅捕获图像")
    parser.add_argument("--prompt", type=str,
                        default="请描述这张图片的内容，包括主要物体、场景和动作",
                        help="自定义分析提示词")
    parser.add_argument("--model", type=str,
                        help="指定火山引擎模型ID")
    parser.add_argument("--ip", type=str, default=config.ROBOT_IP,
                        help=f"机器人IP地址 (默认: {config.ROBOT_IP})")
    parser.add_argument("--port", type=int, default=config.VIDEO_PORT,
                        help=f"视频端口 (默认: {config.VIDEO_PORT})")
    parser.add_argument("--output-dir", type=str, default=config.OUTPUT_DIR,
                        help="图像保存目录")
    
    args = parser.parse_args()
    
    # 创建工作流实例
    workflow = WhatDoesRobotSeeWorkflow(
        robot_ip=args.ip,
        port=args.port,
        output_dir=args.output_dir,
        enable_analysis=not args.no_analysis,
        prompt=args.prompt,
        model=args.model
    )
    
    # 执行完整工作流
    result = workflow.capture_and_analyze()
    
    if result["success"]:
        print(f"\n{'='*50}")
        print(f"图像路径: {result['image_path']}")
        
        if result["analysis"]:
            print(f"\n{'='*50}")
            print("视觉分析结果:")
            print(f"{'='*50}")
            print(result["analysis"])
        
        print(f"{'='*50}")
        
        # 输出路径供其他工具调用
        print(f"\n--- RESULT ---")
        print(result["image_path"])
        sys.exit(0)
    else:
        print(f"\n--- FAILED ---")
        sys.exit(1)


if __name__ == "__main__":
    main()