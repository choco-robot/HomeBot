"""
麻将牌检测模型训练脚本

基于 Ultralytics YOLOv8 训练 34 类麻将牌检测模型

使用方法:
    cd software
    python tools/training/train_mahjong.py --data path/to/data.yaml --epochs 100

Roboflow 数据集准备:
    1. 从 Roboflow 下载 YOLOv8 格式数据集
    2. 解压到 datasets/mahjong/ 目录
    3. 确认 data.yaml 中的路径正确
"""

import argparse
import sys
from pathlib import Path

# 添加 src 到路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

from common.logging import get_logger

logger = get_logger(__name__)


def check_ultralytics():
    """检查 ultralytics 是否安装"""
    try:
        from ultralytics import YOLO
        return True
    except ImportError:
        logger.error("未安装 ultralytics 库，请先运行: pip install ultralytics")
        return False


def train_model(
    data_yaml: str,
    model_type: str = "yolov8n.pt",
    epochs: int = 100,
    imgsz: int = 640,
    batch: int = 16,
    device: str = "cpu",
    project: str = "models/mahjong",
    name: str = "train",
    patience: int = 20,
    workers: int = 4
):
    """
    训练麻将检测模型
    
    Args:
        data_yaml: 数据集配置文件路径
        model_type: 基础模型类型 (yolov8n.pt/yolov8s.pt/yolov8m.pt)
        epochs: 训练轮数
        imgsz: 输入图像尺寸
        batch: 批次大小
        device: 训练设备 (cpu/cuda/mps)
        project: 输出项目目录
        name: 训练任务名称
        patience: 早停耐心值
        workers: 数据加载线程数
    """
    if not check_ultralytics():
        return False
    
    from ultralytics import YOLO
    
    data_path = Path(data_yaml)
    if not data_path.exists():
        logger.error(f"数据集配置文件不存在: {data_yaml}")
        logger.info("请确认路径正确，或使用绝对路径")
        return False
    
    # 创建输出目录
    project_path = Path(project)
    project_path.mkdir(parents=True, exist_ok=True)
    
    logger.info("=" * 50)
    logger.info("开始训练麻将牌检测模型")
    logger.info("=" * 50)
    logger.info(f"数据集配置: {data_yaml}")
    logger.info(f"基础模型: {model_type}")
    logger.info(f"训练轮数: {epochs}")
    logger.info(f"图像尺寸: {imgsz}x{imgsz}")
    logger.info(f"批次大小: {batch}")
    logger.info(f"训练设备: {device}")
    logger.info(f"输出目录: {project}/{name}")
    logger.info("=" * 50)
    
    try:
        # 加载预训练模型
        logger.info(f"加载基础模型: {model_type}")
        model = YOLO(model_type)
        
        # 开始训练
        logger.info("开始训练...")
        results = model.train(
            data=str(data_path),
            epochs=epochs,
            imgsz=imgsz,
            batch=batch,
            device=device,
            project=project,
            name=name,
            patience=patience,
            workers=workers,
            exist_ok=True,
            pretrained=True,
            optimizer="AdamW",
            lr0=0.001,
            lrf=0.01,
            momentum=0.937,
            weight_decay=0.0005,
            warmup_epochs=3.0,
            box=7.5,
            cls=0.5,
            dfl=1.5,
            plots=True,
            save=True,
            save_period=-1,
            cache=False,
            verbose=True
        )
        
        logger.info("=" * 50)
        logger.info("训练完成!")
        logger.info("=" * 50)
        
        # 输出最佳模型路径
        best_model_path = Path(project) / name / "weights" / "best.pt"
        if best_model_path.exists():
            logger.info(f"最佳模型: {best_model_path}")
            logger.info(f"模型大小: {best_model_path.stat().st_size / 1024 / 1024:.2f} MB")
            
            # 复制到标准位置
            target_path = Path("models/mahjong_yolo.pt")
            target_path.parent.mkdir(parents=True, exist_ok=True)
            
            import shutil
            shutil.copy2(best_model_path, target_path)
            logger.info(f"✓ 模型已复制到: {target_path}")
        
        return True
        
    except Exception as e:
        logger.error(f"训练失败: {e}")
        import traceback
        logger.debug(traceback.format_exc())
        return False


def validate_model(
    model_path: str,
    data_yaml: str,
    imgsz: int = 640,
    device: str = "cpu"
):
    """
    验证模型性能
    
    Args:
        model_path: 模型文件路径
        data_yaml: 数据集配置文件路径
        imgsz: 输入图像尺寸
        device: 验证设备
    """
    if not check_ultralytics():
        return False
    
    from ultralytics import YOLO
    
    logger.info("=" * 50)
    logger.info("开始验证模型")
    logger.info("=" * 50)
    
    try:
        model = YOLO(model_path)
        metrics = model.val(
            data=data_yaml,
            imgsz=imgsz,
            device=device,
            verbose=True
        )
        
        logger.info("=" * 50)
        logger.info("验证结果:")
        logger.info(f"  mAP50: {metrics.box.map50:.4f}")
        logger.info(f"  mAP50-95: {metrics.box.map:.4f}")
        logger.info(f"  精确率: {metrics.box.mp:.4f}")
        logger.info(f"  召回率: {metrics.box.mr:.4f}")
        logger.info("=" * 50)
        
        return True
        
    except Exception as e:
        logger.error(f"验证失败: {e}")
        return False


def export_model(
    model_path: str,
    format: str = "onnx",
    imgsz: int = 640
):
    """
    导出模型为其他格式
    
    Args:
        model_path: 模型文件路径
        format: 导出格式 (onnx/torchscript/engine)
        imgsz: 输入图像尺寸
    """
    if not check_ultralytics():
        return False
    
    from ultralytics import YOLO
    
    logger.info(f"导出模型到 {format} 格式...")
    
    try:
        model = YOLO(model_path)
        model.export(format=format, imgsz=imgsz)
        logger.info(f"✓ 导出完成")
        return True
    except Exception as e:
        logger.error(f"导出失败: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="麻将牌检测模型训练脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基础训练
  python tools/training/train_mahjong.py --data datasets/mahjong/data.yaml
  
  # 使用 GPU 训练更多轮数
  python tools/training/train_mahjong.py --data datasets/mahjong/data.yaml --epochs 200 --device cuda
  
  # 使用更大的模型
  python tools/training/train_mahjong.py --data datasets/mahjong/data.yaml --model yolov8s.pt
  
  # 仅验证已有模型
  python tools/training/train_mahjong.py --validate --model models/mahjong_yolo.pt --data datasets/mahjong/data.yaml
        """
    )
    
    parser.add_argument("--data", "-d", type=str, default="datasets/mahjong/data.yaml",
                        help="数据集配置文件路径 (默认: datasets/mahjong/data.yaml)")
    parser.add_argument("--model", "-m", type=str, default="yolov8n.pt",
                        help="基础模型类型 (默认: yolov8n.pt, 可选: yolov8s.pt/yolov8m.pt)")
    parser.add_argument("--epochs", "-e", type=int, default=100,
                        help="训练轮数 (默认: 100)")
    parser.add_argument("--imgsz", "-s", type=int, default=640,
                        help="输入图像尺寸 (默认: 640)")
    parser.add_argument("--batch", "-b", type=int, default=16,
                        help="批次大小 (默认: 16)")
    parser.add_argument("--device", type=str, default="cpu",
                        help="训练设备 (默认: cpu, 可选: cuda/mps)")
    parser.add_argument("--project", "-p", type=str, default="models/mahjong",
                        help="输出项目目录 (默认: models/mahjong)")
    parser.add_argument("--name", "-n", type=str, default="train",
                        help="训练任务名称 (默认: train)")
    parser.add_argument("--patience", type=int, default=20,
                        help="早停耐心值 (默认: 20)")
    parser.add_argument("--workers", "-w", type=int, default=4,
                        help="数据加载线程数 (默认: 4)")
    parser.add_argument("--validate", action="store_true",
                        help="仅验证模式，不训练")
    parser.add_argument("--export", type=str, default=None,
                        help="导出格式 (onnx/torchscript/engine)")
    
    args = parser.parse_args()
    
    # 验证模式
    if args.validate:
        success = validate_model(
            model_path=args.model,
            data_yaml=args.data,
            imgsz=args.imgsz,
            device=args.device
        )
        sys.exit(0 if success else 1)
    
    # 导出模式
    if args.export:
        success = export_model(
            model_path=args.model,
            format=args.export,
            imgsz=args.imgsz
        )
        sys.exit(0 if success else 1)
    
    # 训练模式
    success = train_model(
        data_yaml=args.data,
        model_type=args.model,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
        patience=args.patience,
        workers=args.workers
    )
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
