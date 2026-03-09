#!/usr/bin/env python3
"""
模型下载脚本
自动下载YOLO26等预训练模型到models目录
"""
import os
import sys
import urllib.request
from pathlib import Path

# 添加src到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from common.logging import get_logger

logger = get_logger(__name__)

# 模型配置
MODELS = {
    "yolo26n.pt": {
        "url": "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt",
        "description": "YOLO26 Nano - fastest edge model (~2.4MB)",
        "size_mb": 2.4
    },
    "yolo26s.pt": {
        "url": "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26s.pt",
        "description": "YOLO26 Small - balanced speed and accuracy (~9.5MB)",
        "size_mb": 9.5
    },
    "yolo11n.pt": {
        "url": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt",
        "description": "YOLO11 Nano - alternative lightweight model (~2.6MB)",
        "size_mb": 2.6
    }
}


def get_models_dir() -> Path:
    """获取模型目录"""
    # 从配置文件路径推断
    script_dir = Path(__file__).parent.parent  # software/
    models_dir = script_dir / "models"
    models_dir.mkdir(exist_ok=True)
    return models_dir


def download_file(url: str, dest: Path, desc: str = ""):
    """下载文件并显示进度"""
    def progress_hook(count, block_size, total_size):
        percent = min(int(count * block_size * 100 / total_size), 100)
        sys.stdout.write(f"\r{desc}: {percent}%")
        sys.stdout.flush()
    
    urllib.request.urlretrieve(url, dest, reporthook=progress_hook)
    sys.stdout.write("\n")


def download_model(model_name: str, force: bool = False) -> bool:
    """
    下载指定模型
    
    Args:
        model_name: 模型文件名
        force: 是否强制重新下载
        
    Returns:
        bool: 是否成功
    """
    if model_name not in MODELS:
        logger.error(f"未知模型: {model_name}")
        logger.info(f"可用模型: {', '.join(MODELS.keys())}")
        return False
    
    model_info = MODELS[model_name]
    models_dir = get_models_dir()
    dest_path = models_dir / model_name
    
    # 检查是否已存在
    if dest_path.exists() and not force:
        logger.info(f"模型已存在: {dest_path}")
        return True
    
    # 下载模型
    logger.info(f"下载 {model_name}...")
    logger.info(f"  描述: {model_info['description']}")
    logger.info(f"  大小: ~{model_info['size_mb']}MB")
    logger.info(f"  URL: {model_info['url']}")
    
    try:
        download_file(model_info['url'], dest_path, f"下载 {model_name}")
        logger.info(f"✓ 下载完成: {dest_path}")
        return True
    except Exception as e:
        logger.error(f"✗ 下载失败: {e}")
        # 清理不完整的文件
        if dest_path.exists():
            dest_path.unlink()
        return False


def download_all_models(force: bool = False):
    """下载所有模型"""
    logger.info("=" * 60)
    logger.info("下载所有模型")
    logger.info("=" * 60)
    
    success_count = 0
    for model_name in MODELS:
        if download_model(model_name, force):
            success_count += 1
        logger.info("")
    
    logger.info("=" * 60)
    logger.info(f"下载完成: {success_count}/{len(MODELS)} 个模型")
    logger.info("=" * 60)
    return success_count == len(MODELS)


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='下载YOLO模型')
    parser.add_argument('model', nargs='?', default='yolo26n',
                       help='模型名称 (默认: yolo26n)')
    parser.add_argument('--all', action='store_true',
                       help='下载所有模型')
    parser.add_argument('--force', '-f', action='store_true',
                       help='强制重新下载')
    parser.add_argument('--list', '-l', action='store_true',
                       help='列出可用模型')
    
    args = parser.parse_args()
    
    if args.list:
        print("可用模型:")
        for name, info in MODELS.items():
            print(f"  {name:15s} - {info['description']}")
        return
    
    if args.all:
        download_all_models(args.force)
    else:
        # 自动添加.pt后缀
        model_name = args.model if args.model.endswith('.pt') else f"{args.model}.pt"
        download_model(model_name, args.force)


if __name__ == "__main__":
    main()
