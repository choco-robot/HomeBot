"""视觉分析器 - 获取视频帧并调用 LLM 分析画面内容

核心功能：
1. 订阅 VisionService 发布的图像帧
2. 保存最新帧为临时图片
3. 调用火山引擎 Ark LLM 进行图片内容理解
4. 返回画面描述
"""

import os
import sys
import base64
import tempfile
import time
from typing import Optional, Dict, Any
from pathlib import Path

# 添加项目根目录到路径
_current_dir = Path(__file__).parent
_project_root = _current_dir.parents[2]  # software/src -> software -> homebot
sys.path.insert(0, str(_project_root))

import cv2
import numpy as np
from services.vision_service.vision import VisionSubscriber
from common.logging import get_logger
from configs.secrets import get_secrets, require_secrets

logger = get_logger(__name__)

# 默认火山引擎配置
DEFAULT_ARK_MODEL = "doubao-seed-2-0-mini-260215"
DEFAULT_ARK_BASE_URL = "https://ark.cn-beijing.volces.com/api/v3"


class VisionAnalyzer:
    """视觉分析器 - 捕获视频帧并进行 AI 分析"""
    
    def __init__(
        self,
        video_addr: str = "tcp://localhost:5560",
        api_key: Optional[str] = None,
        model: Optional[str] = None,
        base_url: Optional[str] = None,
        timeout_ms: int = 5000
    ):
        """初始化视觉分析器
        
        Args:
            video_addr: VisionService PUB 地址
            api_key: 火山引擎 API Key，默认从环境变量或 .env.local 获取
            model: 模型 ID，默认使用 doubao-vision-lite-250225
            base_url: Ark API 基础 URL
            timeout_ms: 视频帧获取超时（毫秒）
        """
        self.video_addr = video_addr
        self.timeout_ms = timeout_ms
        
        # 加载密钥配置（优先使用传入的参数，其次环境变量，最后secrets模块）
        secrets = get_secrets()
        
        # API Key: 参数 > 环境变量 > secrets模块
        self.api_key = api_key or os.getenv("ARK_API_KEY", "")
        if not self.api_key and secrets.vision.api_key:
            self.api_key = secrets.vision.api_key
        # 如果vision没有单独配置，尝试使用火山TTS的appid+token（部分Ark服务支持）
        if not self.api_key and secrets.tts.access_token:
            self.api_key = secrets.tts.access_token
        
        # 模型ID
        self.model = model or os.getenv("ARK_MODEL_ID", DEFAULT_ARK_MODEL)
        
        # API基础URL
        self.base_url = base_url or os.getenv("ARK_BASE_URL", DEFAULT_ARK_BASE_URL)
        # 如果配置了自定义的vision URL，使用它
        if secrets.vision.api_url:
            self.base_url = secrets.vision.api_url
        
        # 视频订阅器
        self._subscriber: Optional[VisionSubscriber] = None
        
        # 临时目录
        self._temp_dir = tempfile.mkdtemp(prefix="homebot_vision_")
        
        logger.info(f"VisionAnalyzer initialized, video_addr={video_addr}")
    
    def _ensure_subscriber(self) -> VisionSubscriber:
        """确保视频订阅器已启动
        
        Returns:
            VisionSubscriber 实例
        """
        if self._subscriber is None:
            self._subscriber = VisionSubscriber(self.video_addr)
            self._subscriber.start()
            logger.info(f"VisionSubscriber started, connecting to {self.video_addr}")
            # 等待订阅器接收第一帧
            time.sleep(0.5)
        return self._subscriber
    
    def capture_frame(self, output_path: Optional[str] = None) -> Optional[str]:
        """捕获最新视频帧并保存为图片
        
        Args:
            output_path: 图片保存路径，默认使用临时目录
            
        Returns:
            保存的图片路径，失败返回 None
        """
        subscriber = self._ensure_subscriber()
        
        # 尝试多次获取帧
        max_retries = 3
        for attempt in range(max_retries):
            frame_id, frame = subscriber.read_frame()
            if frame is not None:
                break
            logger.warning(f"Frame not ready, retry {attempt + 1}/{max_retries}")
            time.sleep(0.1)
        else:
            logger.error("Failed to get frame from video service")
            return None
        
        # 确定保存路径
        if output_path is None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_path = os.path.join(self._temp_dir, f"capture_{timestamp}.jpg")
        
        # 保存图片
        try:
            cv2.imwrite(output_path, frame)
            logger.info(f"Frame saved to {output_path}, frame_id={frame_id}")
            return output_path
        except Exception as e:
            logger.error(f"Failed to save frame: {e}")
            return None
    
    def encode_image(self, image_path: str) -> str:
        """将图片文件编码为 base64 字符串
        
        Args:
            image_path: 图片文件路径
            
        Returns:
            base64 编码字符串
        """
        with open(image_path, "rb") as f:
            return base64.b64encode(f.read()).decode("utf-8")
    
    def analyze(
        self,
        image_path: str,
        prompt: str = "请描述这张图片的内容",
        max_tokens: int = 4096
    ) -> Dict[str, Any]:
        """调用火山引擎 LLM 分析图片
        
        Args:
            image_path: 图片文件路径
            prompt: 对图片的提问或指令
            max_tokens: 最大输出 token 数
            
        Returns:
            包含状态和结果的字典
        """
        if not self.api_key:
            return {
                "status": "error",
                "message": "API Key 未配置，请设置 ARK_API_KEY 环境变量，或在 .env.local 中配置 VISION_API_KEY 或 VOLCANO_ACCESS_TOKEN"
            }
        
        if not os.path.exists(image_path):
            return {
                "status": "error",
                "message": f"图片文件不存在: {image_path}"
            }
        
        try:
            # 尝试导入火山引擎 SDK
            try:
                from volcenginesdkarkruntime import Ark
            except ImportError:
                return {
                    "status": "error",
                    "message": "未安装 volcenginesdkarkruntime，请运行: pip install volcenginesdkarkruntime"
                }
            
            # 初始化客户端
            client = Ark(base_url=self.base_url, api_key=self.api_key)
            
            # 编码图片
            base64_image = self.encode_image(image_path)
            
            # 构建消息内容
            content = [
                {"type": "text", "text": prompt},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{base64_image}"
                    }
                }
            ]
            
            # 发送请求
            logger.info(f"Sending analysis request to Ark LLM, model={self.model}")
            response = client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": content}],
                max_tokens=max_tokens,
                stream=False
            )
            
            result_text = response.choices[0].message.content
            logger.info("Analysis completed successfully")
            
            return {
                "status": "success",
                "description": result_text,
                "image_path": image_path
            }
            
        except Exception as e:
            logger.error(f"Analysis failed: {e}")
            return {
                "status": "error",
                "message": f"图像分析失败: {e}"
            }
    
    def capture_and_analyze(
        self,
        prompt: str = "请描述这张图片的内容",
        max_tokens: int = 4096
    ) -> Dict[str, Any]:
        """一键捕获并分析画面
        
        Args:
            prompt: 对图片的提问或指令
            max_tokens: 最大输出 token 数
            
        Returns:
            包含状态和结果的字典
        """
        # 捕获帧
        image_path = self.capture_frame()
        if image_path is None:
            return {
                "status": "error",
                "message": "无法获取视频帧，请检查视觉服务是否已启动"
            }
        
        # 分析图片
        return self.analyze(image_path, prompt, max_tokens)
    
    def close(self):
        """关闭资源"""
        if self._subscriber:
            self._subscriber.stop()
            self._subscriber = None
            logger.info("VisionSubscriber stopped")
        
        # 清理临时文件
        try:
            import shutil
            if os.path.exists(self._temp_dir):
                shutil.rmtree(self._temp_dir)
                logger.info(f"Temp directory cleaned: {self._temp_dir}")
        except Exception as e:
            logger.warning(f"Failed to clean temp directory: {e}")
    
    def __enter__(self):
        """上下文管理器入口"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.close()
        return False


def main():
    """命令行入口"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="HomeBot 视觉理解工具 - 捕获画面并调用 LLM 分析",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基本使用（描述画面内容）
  python -m applications.vision_understanding

  # 自定义提问
  python -m applications.vision_understanding -p "图中有几个人？"

  # 指定视频服务地址
  python -m applications.vision_understanding --video-addr tcp://192.168.1.100:5560

  # 使用特定模型
  python -m applications.vision_understanding --model doubao-vision-pro-250226
        """
    )
    
    parser.add_argument(
        "-p", "--prompt",
        default="请描述这张图片的内容",
        help="对图片的提问或指令 (默认: 请描述这张图片的内容)"
    )
    
    parser.add_argument(
        "--video-addr",
        default="tcp://localhost:5560",
        help="VisionService PUB 地址 (默认: tcp://localhost:5560)"
    )
    
    parser.add_argument(
        "--model",
        default=None,
        help="模型 ID (默认从 ARK_MODEL_ID 环境变量获取)"
    )
    
    parser.add_argument(
        "--api-key",
        default=None,
        help="API Key (默认从 ARK_API_KEY 环境变量获取)"
    )
    
    parser.add_argument(
        "-o", "--output",
        default=None,
        help="图片保存路径 (默认使用临时目录)"
    )
    
    parser.add_argument(
        "--save-image",
        action="store_true",
        help="保存捕获的图片到当前目录"
    )
    
    args = parser.parse_args()
    
    # 创建分析器
    analyzer = VisionAnalyzer(
        video_addr=args.video_addr,
        api_key=args.api_key,
        model=args.model
    )
    
    try:
        # 捕获帧
        print(f"[INFO] 正在连接视频服务 {args.video_addr}...")
        image_path = analyzer.capture_frame(args.output)
        if image_path is None:
            print("[ERROR] 无法获取视频帧，请检查视觉服务是否已启动")
            sys.exit(1)
        
        print(f"[OK] 图像捕获成功: {image_path}")
        
        # 如果需要保存到当前目录
        if args.save_image and args.output is None:
            saved_path = f"capture_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
            import shutil
            shutil.copy(image_path, saved_path)
            print(f"[OK] 图片已保存到: {saved_path}")
        
        # 分析图片
        print(f"[INFO] 正在分析图片，提示词: {args.prompt}")
        print(f"[INFO] 使用模型: {analyzer.model}")
        print()
        
        result = analyzer.analyze(image_path, args.prompt)
        
        if result["status"] == "success":
            print("=" * 50)
            print("分析结果:")
            print("=" * 50)
            print(result["description"])
            print("=" * 50)
        else:
            print(f"[ERROR] {result['message']}")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断")
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)
    finally:
        analyzer.close()


if __name__ == "__main__":
    main()
