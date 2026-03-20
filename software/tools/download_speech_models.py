#!/usr/bin/env python3
"""
sherpa-onnx 语音模型下载脚本

自动下载语音识别(ASR)和唤醒词检测所需的模型文件

使用方法:
    cd software
    python tools/download_speech_models.py

模型来源:
    - ASR: sherpa-onnx 中文流式 zipformer 模型
    - 唤醒: sherpa-onnx 中文唤醒词模型
"""

import os
import sys
import zipfile
import urllib.request
from pathlib import Path
from typing import Optional

# 添加 src 到路径
script_dir = Path(__file__).parent.parent
sys.path.insert(0, str(script_dir / "src"))

from common.logging import get_logger

logger = get_logger(__name__)

# 模型配置
MODELS = {
    "asr": {
        "name": "中文流式ASR模型",
        "url": "https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-streaming-zipformer-zh-int8-2025-06-30.tar.bz2",
        "archive_type": "tar.bz2",
        "target_dir": "models/asr",
        "files": [
            "encoder.int8.onnx",
            "decoder.onnx", 
            "joiner.int8.onnx",
            "tokens.txt"
        ]
    },
    "wakeup": {
        "name": "中文唤醒词模型",
        "url": "https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2",
        "archive_type": "tar.bz2",
        "target_dir": "models/wakeup",
        "files": [
            "encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx",
            "decoder-epoch-13-avg-2-chunk-16-left-64.onnx",
            "joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx",
            "tokens.txt"
        ]
    }
}


def get_project_root() -> Path:
    """获取项目根目录"""
    # 从当前文件向上查找
    current = Path(__file__).resolve()
    # software/tools/download_speech_models.py -> software/
    return current.parent.parent


def check_model_exists(model_config: dict) -> bool:
    """检查模型文件是否已存在"""
    project_root = get_project_root()
    target_dir = project_root / model_config["target_dir"]
    
    if not target_dir.exists():
        return False
    
    # 检查关键文件是否存在
    for file_name in model_config["files"]:
        if not (target_dir / file_name).exists():
            return False
    
    return True


def download_file(url: str, target_path: Path, show_progress: bool = True) -> bool:
    """下载文件并显示进度
    
    Args:
        url: 下载地址
        target_path: 保存路径
        show_progress: 是否显示进度条
        
    Returns:
        bool: 下载是否成功
    """
    try:
        logger.info(f"下载: {url}")
        logger.info(f"保存到: {target_path}")
        
        def progress_hook(count, block_size, total_size):
            if show_progress and total_size > 0:
                percent = int(count * block_size * 100 / total_size)
                percent = min(percent, 100)
                sys.stdout.write(f"\r进度: {percent}% ({count * block_size / 1024 / 1024:.1f} MB / {total_size / 1024 / 1024:.1f} MB)")
                sys.stdout.flush()
        
        target_path.parent.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(url, target_path, reporthook=progress_hook)
        
        if show_progress:
            sys.stdout.write("\n")
        
        logger.info("下载完成")
        return True
        
    except Exception as e:
        logger.error(f"下载失败: {e}")
        return False


def extract_archive(archive_path: Path, extract_to: Path, archive_type: str) -> bool:
    """解压归档文件
    
    Args:
        archive_path: 归档文件路径
        extract_to: 解压目标目录
        archive_type: 归档类型 (tar.bz2, zip等)
        
    Returns:
        bool: 解压是否成功
    """
    try:
        logger.info(f"解压: {archive_path}")
        extract_to.mkdir(parents=True, exist_ok=True)
        
        if archive_type == "tar.bz2":
            import tarfile
            with tarfile.open(archive_path, "r:bz2") as tar:
                tar.extractall(extract_to)
        elif archive_type == "zip":
            with zipfile.ZipFile(archive_path, 'r') as zip_ref:
                zip_ref.extractall(extract_to)
        else:
            logger.error(f"不支持的归档类型: {archive_type}")
            return False
        
        logger.info("解压完成")
        return True
        
    except Exception as e:
        logger.error(f"解压失败: {e}")
        return False


def move_model_files(extract_dir: Path, target_dir: Path, model_config: dict) -> bool:
    """移动模型文件到目标目录
    
    Args:
        extract_dir: 解压目录
        target_dir: 目标目录
        model_config: 模型配置
        
    Returns:
        bool: 是否成功
    """
    try:
        # 查找解压后的子目录
        subdirs = [d for d in extract_dir.iterdir() if d.is_dir()]
        if subdirs:
            source_dir = subdirs[0]  # 通常解压后会创建一个子目录
        else:
            source_dir = extract_dir
        
        target_dir.mkdir(parents=True, exist_ok=True)
        
        # 移动需要的文件
        for file_name in model_config["files"]:
            src = source_dir / file_name
            dst = target_dir / file_name
            
            if src.exists():
                import shutil
                shutil.move(str(src), str(dst))
                logger.info(f"移动: {file_name}")
            else:
                # 尝试在子目录中查找
                for subdir in source_dir.rglob(file_name):
                    shutil.move(str(subdir), str(dst))
                    logger.info(f"移动: {file_name}")
                    break
        
        return True
        
    except Exception as e:
        logger.error(f"移动文件失败: {e}")
        return False


def create_keywords_file(target_dir: Path):
    """创建唤醒词配置文件"""
    keywords_file = target_dir / "keywords.txt"
    if not keywords_file.exists():
        content = """n ǐ h ǎo x iǎo b ái @你好小白
"""
        keywords_file.write_text(content, encoding="utf-8")
        logger.info(f"创建唤醒词配置文件: {keywords_file}")


def download_model(model_type: str, force: bool = False) -> bool:
    """下载指定类型的模型
    
    Args:
        model_type: 模型类型 (asr 或 wakeup)
        force: 是否强制重新下载
        
    Returns:
        bool: 下载是否成功
    """
    if model_type not in MODELS:
        logger.error(f"未知模型类型: {model_type}")
        return False
    
    config = MODELS[model_type]
    project_root = get_project_root()
    target_dir = project_root / config["target_dir"]
    
    # 检查是否已存在
    if not force and check_model_exists(config):
        logger.info(f"{config['name']} 已存在，跳过下载")
        logger.info(f"位置: {target_dir}")
        # 确保唤醒词文件存在
        if model_type == "wakeup":
            create_keywords_file(target_dir)
        return True
    
    # 下载目录
    download_dir = project_root / "cache" / "downloads"
    download_dir.mkdir(parents=True, exist_ok=True)
    
    # 下载文件
    archive_name = config["url"].split("/")[-1]
    archive_path = download_dir / archive_name
    
    if not download_file(config["url"], archive_path):
        return False
    
    # 解压
    extract_dir = download_dir / "extracted" / model_type
    if not extract_archive(archive_path, extract_dir, config["archive_type"]):
        return False
    
    # 移动文件
    if not move_model_files(extract_dir, target_dir, config):
        return False
    
    # 创建唤醒词文件
    if model_type == "wakeup":
        create_keywords_file(target_dir)
    
    # 清理
    try:
        import shutil
        shutil.rmtree(extract_dir)
        archive_path.unlink()
        logger.info("清理临时文件")
    except Exception as e:
        logger.warning(f"清理临时文件失败: {e}")
    
    logger.info(f"{config['name']} 安装完成!")
    logger.info(f"位置: {target_dir}")
    return True


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description="下载 sherpa-onnx 语音模型")
    parser.add_argument(
        "--model",
        choices=["asr", "wakeup", "all"],
        default="all",
        help="要下载的模型类型 (默认: all)"
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="强制重新下载，即使文件已存在"
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("HomeBot 语音模型下载工具")
    print("=" * 60)
    print()
    
    project_root = get_project_root()
    print(f"项目目录: {project_root}")
    print()
    
    success = True
    
    if args.model in ["asr", "all"]:
        print("-" * 60)
        print("下载 ASR 语音识别模型...")
        print("-" * 60)
        if not download_model("asr", args.force):
            success = False
        print()
    
    if args.model in ["wakeup", "all"]:
        print("-" * 60)
        print("下载唤醒词检测模型...")
        print("-" * 60)
        if not download_model("wakeup", args.force):
            success = False
        print()
    
    print("=" * 60)
    if success:
        print("所有模型下载完成!")
        print()
        print("模型位置:")
        print(f"  - ASR:    {project_root / 'models/asr'}")
        print(f"  - 唤醒:   {project_root / 'models/wakeup'}")
    else:
        print("部分模型下载失败，请检查网络连接后重试")
        return 1
    print("=" * 60)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
