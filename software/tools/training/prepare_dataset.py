"""
麻将数据集准备工具

用于解压和准备 Roboflow 下载的 YOLO 格式数据集

使用方法:
    cd software
    python tools/training/prepare_dataset.py --zip path/to/mahjong.zip
"""

import argparse
import sys
import zipfile
import shutil
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

from common.logging import get_logger

logger = get_logger(__name__)


def prepare_roboflow_dataset(
    zip_path: str,
    output_dir: str = "datasets/mahjong",
    validate: bool = True
):
    """
    准备 Roboflow 下载的 YOLO 格式数据集
    
    Args:
        zip_path: ZIP 文件路径
        output_dir: 输出目录
        validate: 是否验证数据集完整性
    """
    zip_file = Path(zip_path)
    if not zip_file.exists():
        logger.error(f"ZIP 文件不存在: {zip_path}")
        return False
    
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    logger.info("=" * 50)
    logger.info("准备麻将数据集")
    logger.info("=" * 50)
    logger.info(f"ZIP 文件: {zip_path}")
    logger.info(f"输出目录: {output_dir}")
    
    try:
        # 解压 ZIP 文件
        logger.info("解压数据集...")
        with zipfile.ZipFile(zip_file, 'r') as zf:
            # 列出根目录内容
            root_items = set()
            for name in zf.namelist():
                parts = name.split('/')
                if len(parts) > 0 and parts[0]:
                    root_items.add(parts[0])
            
            logger.info(f"ZIP 根目录: {root_items}")
            
            # 解压到临时目录
            temp_dir = output_path / "_temp"
            if temp_dir.exists():
                shutil.rmtree(temp_dir)
            
            zf.extractall(temp_dir)
        
        # 查找实际的数据目录（Roboflow 通常有一个嵌套目录）
        data_root = temp_dir
        subdirs = [d for d in temp_dir.iterdir() if d.is_dir()]
        
        if len(subdirs) == 1 and subdirs[0].name not in ["train", "valid", "test"]:
            # 有一个嵌套目录，进入它
            data_root = subdirs[0]
            logger.info(f"检测到嵌套目录: {data_root.name}")
        
        # 移动文件到目标目录
        for item in data_root.iterdir():
            target = output_path / item.name
            if target.exists():
                if target.is_dir():
                    shutil.rmtree(target)
                else:
                    target.unlink()
            
            shutil.move(str(item), str(target))
            logger.info(f"  ✓ {item.name}")
        
        # 清理临时目录
        shutil.rmtree(temp_dir)
        
        logger.info("解压完成")
        
        # 验证数据集
        if validate:
            return validate_dataset(output_path)
        
        return True
        
    except Exception as e:
        logger.error(f"准备数据集失败: {e}")
        import traceback
        logger.debug(traceback.format_exc())
        return False


def validate_dataset(dataset_dir: Path):
    """
    验证数据集完整性
    
    Args:
        dataset_dir: 数据集目录
    """
    logger.info("=" * 50)
    logger.info("验证数据集")
    logger.info("=" * 50)
    
    # 检查 data.yaml
    yaml_file = dataset_dir / "data.yaml"
    if not yaml_file.exists():
        logger.error(f"缺少 data.yaml 文件")
        return False
    logger.info(f"✓ 找到 data.yaml")
    
    # 检查目录结构
    splits = ["train", "valid"]
    optional_splits = ["test"]
    
    for split in splits:
        images_dir = dataset_dir / split / "images"
        labels_dir = dataset_dir / split / "labels"
        
        if not images_dir.exists():
            logger.error(f"缺少目录: {split}/images")
            return False
        
        if not labels_dir.exists():
            logger.error(f"缺少目录: {split}/labels")
            return False
        
        image_count = len(list(images_dir.glob("*.*")))
        label_count = len(list(labels_dir.glob("*.txt")))
        
        logger.info(f"✓ {split}: {image_count} 张图像, {label_count} 个标签")
    
    for split in optional_splits:
        images_dir = dataset_dir / split / "images"
        if images_dir.exists():
            image_count = len(list(images_dir.glob("*.*")))
            logger.info(f"✓ {split} (可选): {image_count} 张图像")
    
    # 读取 data.yaml 检查类别
    try:
        import yaml
        with open(yaml_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        num_classes = config.get('nc', 0)
        class_names = config.get('names', [])
        
        logger.info(f"✓ 类别数量: {num_classes}")
        logger.info(f"✓ 类别名称: {class_names[:5]}..." if len(class_names) > 5 else f"✓ 类别名称: {class_names}")
        
        # 检查是否是麻将牌类别
        expected_tiles = 34  # 34 种麻将牌
        if num_classes < expected_tiles:
            logger.warning(f"类别数量较少 ({num_classes} < {expected_tiles})，可能不包含所有麻将牌")
        
    except Exception as e:
        logger.warning(f"无法解析 data.yaml: {e}")
    
    # 修正 data.yaml 中的路径
    fix_yaml_paths(yaml_file, dataset_dir)
    
    logger.info("=" * 50)
    logger.info("数据集验证完成")
    logger.info("=" * 50)
    logger.info(f"数据集路径: {dataset_dir.absolute()}")
    logger.info(f"配置文件: {yaml_file.absolute()}")
    
    return True


def fix_yaml_paths(yaml_file: Path, dataset_dir: Path):
    """
    修正 data.yaml 中的路径为绝对路径，避免训练时找不到数据
    """
    try:
        import yaml
        
        with open(yaml_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        # 更新路径为绝对路径
        modified = False
        
        for key in ['train', 'val', 'test']:
            if key in config:
                original_path = config[key]
                # 如果是相对路径，转换为绝对路径
                if not Path(original_path).is_absolute():
                    absolute_path = str(dataset_dir / original_path)
                    config[key] = absolute_path
                    modified = True
        
        if modified:
            with open(yaml_file, 'w', encoding='utf-8') as f:
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
            logger.info("✓ 已修正 data.yaml 中的路径为绝对路径")
        
    except Exception as e:
        logger.warning(f"无法修正 data.yaml 路径: {e}")


def check_class_mapping(dataset_dir: Path):
    """
    检查数据集类别与项目类别映射是否一致
    """
    yaml_file = dataset_dir / "data.yaml"
    if not yaml_file.exists():
        return
    
    try:
        import yaml
        with open(yaml_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        class_names = config.get('names', [])
        
        # 项目定义的类别
        from applications.mahjong_bot.detector import TILE_CLASSES
        
        logger.info("=" * 50)
        logger.info("类别映射检查")
        logger.info("=" * 50)
        logger.info(f"数据集类别 ({len(class_names)} 个):")
        for i, name in enumerate(class_names):
            logger.info(f"  {i}: {name}")
        
        logger.info(f"\n项目类别 ({len(TILE_CLASSES)} 个):")
        for i, name in TILE_CLASSES.items():
            logger.info(f"  {i}: {name}")
        
    except Exception as e:
        logger.warning(f"无法检查类别映射: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="麻将数据集准备工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 准备数据集
  python tools/training/prepare_dataset.py --zip ~/Downloads/mahjong.zip
  
  # 指定输出目录
  python tools/training/prepare_dataset.py --zip ~/Downloads/mahjong.zip --output datasets/mahjong-v2
  
  # 仅验证已有数据集
  python tools/training/prepare_dataset.py --validate --dir datasets/mahjong
        """
    )
    
    parser.add_argument("--zip", "-z", type=str, default=None,
                        help="Roboflow 下载的 ZIP 文件路径")
    parser.add_argument("--output", "-o", type=str, default="datasets/mahjong",
                        help="输出目录 (默认: datasets/mahjong)")
    parser.add_argument("--validate", "-v", action="store_true",
                        help="仅验证模式")
    parser.add_argument("--dir", "-d", type=str, default="datasets/mahjong",
                        help="数据集目录 (用于验证模式)")
    
    args = parser.parse_args()
    
    # 验证模式
    if args.validate:
        dataset_dir = Path(args.dir)
        if not dataset_dir.exists():
            logger.error(f"数据集目录不存在: {args.dir}")
            sys.exit(1)
        
        success = validate_dataset(dataset_dir)
        check_class_mapping(dataset_dir)
        sys.exit(0 if success else 1)
    
    # 准备模式
    if not args.zip:
        logger.error("请提供 ZIP 文件路径 (--zip)")
        parser.print_help()
        sys.exit(1)
    
    success = prepare_roboflow_dataset(args.zip, args.output)
    
    if success:
        # 准备完成后检查类别映射
        check_class_mapping(Path(args.output))
        logger.info("\n准备完成！现在可以开始训练:")
        logger.info(f"  python tools/training/train_mahjong.py --data {args.output}/data.yaml")
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
