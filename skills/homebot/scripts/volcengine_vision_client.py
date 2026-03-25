#!/usr/bin/env python3
"""
火山引擎 Ark LLM 图片内容理解客户端
支持本地图片文件或多张图片进行内容分析和理解
"""

import os
import sys
import base64
import argparse
from typing import List, Dict, Any, Optional
from volcenginesdkarkruntime import Ark

#在此填写 API KEY
ARK_API_KEY = "a82b691f-d965-43c8-a7cd-f91dc52ba6b6"

def encode_image(image_path: str) -> str:
    """将本地图片文件编码为base64字符串"""
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")


def create_image_content(image_path: str) -> Dict[str, Any]:
    """创建图片内容对象，支持本地文件路径"""
    if image_path.startswith(("http://", "https://")):
        # 如果是URL，直接使用
        return {
            "type": "image_url",
            "image_url": {"url": image_path}
        }
    else:
        # 本地文件，编码为base64
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"图片文件不存在: {image_path}")
        
        # 检测文件类型
        ext = os.path.splitext(image_path)[1].lower()
        mime_types = {
            ".jpg": "image/jpeg",
            ".jpeg": "image/jpeg",
            ".png": "image/png",
            ".gif": "image/gif",
            ".webp": "image/webp",
            ".bmp": "image/bmp",
        }
        mime_type = mime_types.get(ext, "image/jpeg")
        
        base64_image = encode_image(image_path)
        return {
            "type": "image_url",
            "image_url": {
                "url": f"data:{mime_type};base64,{base64_image}"
            }
        }


def analyze_images(
    image_paths: List[str],
    prompt: str = "请描述这张图片的内容",
    model: Optional[str] = None,
    api_key: Optional[str] = None,
    base_url: Optional[str] = None,
    stream: bool = False,
    max_tokens: int = 4096,
    reasoning_effort: str = "low"
) -> str:
    """
    分析一张或多张图片的内容
    
    Args:
        image_paths: 图片文件路径列表
        prompt: 对图片的提问或指令
        model: 模型ID，默认从环境变量 ARK_MODEL_ID 获取
        api_key: API密钥，默认从环境变量 ARK_API_KEY 获取
        base_url: Ark API基础URL
        stream: 是否使用流式输出
        max_tokens: 最大输出token数
        reasoning_effort: 推理努力程度 (low/medium/high)
    
    Returns:
        模型返回的文本内容
    """
    # 获取配置
    api_key = api_key or os.getenv("ARK_API_KEY")
    if not api_key:
        api_key = ARK_API_KEY
    
    model = model or os.getenv("ARK_MODEL_ID", "doubao-seed-2-0-lite-260215")
    base_url = base_url or os.getenv("ARK_BASE_URL", "https://ark.cn-beijing.volces.com/api/v3")
    
    # 初始化客户端
    client = Ark(base_url=base_url, api_key=api_key)
    
    # 构建消息内容
    content = [{"type": "text", "text": prompt}]
    
    for image_path in image_paths:
        content.append(create_image_content(image_path))
    
    # 发送请求
    response = client.chat.completions.create(
        model=model,
        messages=[{"role": "user", "content": content}],
        max_tokens=max_tokens,
        stream=stream
    )
    
    if stream:
        # 流式输出处理
        result = []
        for chunk in response:
            if chunk.choices and chunk.choices[0].delta.content:
                result.append(chunk.choices[0].delta.content)
        return "".join(result)
    else:
        return response.choices[0].message.content


def main():
    parser = argparse.ArgumentParser(
        description="火山引擎 Ark LLM 图片内容理解工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s image.jpg
  %(prog)s image1.jpg image2.png -p "这两张图有什么区别？"
  %(prog)s image.jpg -m doubao-vision-pro-250226
  %(prog)s https://example.com/image.jpg
        """
    )
    
    parser.add_argument(
        "images",
        nargs="+",
        help="图片文件路径或URL（可指定多个）"
    )
    
    parser.add_argument(
        "-p", "--prompt",
        default="请描述这张图片的内容",
        help="对图片的提问或指令 (默认: 请描述这张图片的内容)"
    )
    
    parser.add_argument(
        "-m", "--model",
        help="模型ID (默认从 ARK_MODEL_ID 环境变量获取，或使用 doubao-vision-lite-250225)"
    )
    
    parser.add_argument(
        "--api-key",
        help="API密钥 (默认从 ARK_API_KEY 环境变量获取)"
    )
    
    parser.add_argument(
        "--base-url",
        help="Ark API基础URL"
    )
    
    parser.add_argument(
        "-s", "--stream",
        action="store_true",
        help="使用流式输出"
    )
    
    parser.add_argument(
        "--max-tokens",
        type=int,
        default=4096,
        help="最大输出token数 (默认: 4096)"
    )
    
    args = parser.parse_args()
    
    try:
        result = analyze_images(
            image_paths=args.images,
            prompt=args.prompt,
            model=args.model,
            api_key=args.api_key,
            base_url=args.base_url,
            stream=args.stream,
            max_tokens=args.max_tokens
        )
        print(result)
    except FileNotFoundError as e:
        print(f"错误: {e}", file=sys.stderr)
        sys.exit(1)
    except ValueError as e:
        print(f"错误: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"请求失败: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()