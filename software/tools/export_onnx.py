#!/usr/bin/env python3
"""
YOLO模型转换工具
将PyTorch模型(.pt)转换为ONNX格式，加速推理

使用方法:
    # 基本转换
    python tools/export_onnx.py models/yolo26n.pt
    
    # 指定输入尺寸
    python tools/export_onnx.py models/yolo26n.pt --imgsz 320
    
    # 半精度模型（更小更快，但可能损失精度）
    python tools/export_onnx.py models/yolo26n.pt --half
    
    # 指定输出路径
    python tools/export_onnx.py models/yolo26n.pt -o models/yolo26n_custom.onnx
"""
import sys
import os
import argparse
from pathlib import Path

# 添加src到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from common.logging import get_logger

logger = get_logger(__name__)


def export_onnx(
    model_path: str,
    output_path: str = None,
    imgsz: int = 320,
    half: bool = False,
    simplify: bool = True,
    opset: int = 12,
    device: str = "cpu"
):
    """
    将YOLO模型导出为ONNX格式
    
    Args:
        model_path: 输入模型路径 (.pt文件)
        output_path: 输出路径 (默认与输入同名，.onnx后缀)
        imgsz: 输入图像尺寸
        half: 是否使用半精度(FP16)
        simplify: 是否简化模型
        opset: ONNX opset版本
        device: 设备 (cpu/cuda)
    
    Returns:
        bool: 是否成功
    """
    model_path = Path(model_path)
    
    if not model_path.exists():
        logger.error(f"模型文件不存在: {model_path}")
        return False
    
    # 默认输出路径
    if output_path is None:
        output_path = model_path.with_suffix('.onnx')
    else:
        output_path = Path(output_path)
    
    logger.info("=" * 60)
    logger.info("YOLO模型导出为ONNX")
    logger.info("=" * 60)
    logger.info(f"输入模型: {model_path}")
    logger.info(f"输出路径: {output_path}")
    logger.info(f"输入尺寸: {imgsz}x{imgsz}")
    logger.info(f"半精度: {half}")
    logger.info(f"简化模型: {simplify}")
    logger.info(f"ONNX opset: {opset}")
    logger.info(f"设备: {device}")
    logger.info("=" * 60)
    
    try:
        from ultralytics import YOLO
        
        # 加载模型
        logger.info("加载YOLO模型...")
        model = YOLO(str(model_path))
        logger.info(f"✓ 模型已加载: {model_path.name}")
        
        # 导出ONNX
        logger.info("开始导出ONNX...")
        
        export_args = {
            'format': 'onnx',
            'imgsz': imgsz,
            'half': half,
            'simplify': simplify,
            'opset': opset,
            'device': device,
        }
        
        # 执行导出
        model.export(**export_args)
        
        # ultralytics默认导出到同目录
        default_output = model_path.with_suffix('.onnx')
        
        # 如果指定了不同的输出路径，移动文件
        if output_path != default_output:
            if default_output.exists():
                default_output.rename(output_path)
                logger.info(f"✓ 模型已移动到: {output_path}")
        
        if output_path.exists() or default_output.exists():
            final_path = output_path if output_path.exists() else default_output
            file_size = final_path.stat().st_size / (1024 * 1024)  # MB
            logger.info(f"✓ 导出成功!")
            logger.info(f"  文件: {final_path}")
            logger.info(f"  大小: {file_size:.2f} MB")
            logger.info("=" * 60)
            return True
        else:
            logger.error("✗ 导出失败: 输出文件未生成")
            return False
            
    except ImportError:
        logger.error("未安装ultralytics，请先安装: pip install ultralytics")
        return False
    except Exception as e:
        logger.error(f"导出失败: {e}")
        import traceback
        logger.debug(traceback.format_exc())
        return False


def benchmark_onnx(onnx_path: str, imgsz: int = 320, iterations: int = 100):
    """
    对ONNX模型进行性能测试
    
    Args:
        onnx_path: ONNX模型路径
        imgsz: 输入尺寸
        iterations: 测试迭代次数
    """
    logger.info("=" * 60)
    logger.info("ONNX模型性能测试")
    logger.info("=" * 60)
    
    try:
        import onnxruntime as ort
        import numpy as np
        import time
        
        # 创建推理会话
        providers = ['CPUExecutionProvider']
        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = 4
        
        session = ort.InferenceSession(str(onnx_path), sess_options, providers=providers)
        input_name = session.get_inputs()[0].name
        
        # 创建随机输入
        input_shape = (1, 3, imgsz, imgsz)
        dummy_input = np.random.randn(*input_shape).astype(np.float32)
        
        # 预热
        logger.info("预热...")
        for _ in range(10):
            session.run(None, {input_name: dummy_input})
        
        # 正式测试
        logger.info(f"测试 {iterations} 次迭代...")
        times = []
        
        for i in range(iterations):
            start = time.perf_counter()
            session.run(None, {input_name: dummy_input})
            elapsed = (time.perf_counter() - start) * 1000  # ms
            times.append(elapsed)
        
        # 统计结果
        times = np.array(times)
        avg_time = np.mean(times)
        min_time = np.min(times)
        max_time = np.max(times)
        std_time = np.std(times)
        fps = 1000 / avg_time
        
        logger.info("测试结果:")
        logger.info(f"  平均时间: {avg_time:.2f} ms")
        logger.info(f"  最小时间: {min_time:.2f} ms")
        logger.info(f"  最大时间: {max_time:.2f} ms")
        logger.info(f"  标准差: {std_time:.2f} ms")
        logger.info(f"  等效FPS: {fps:.1f}")
        logger.info("=" * 60)
        
        return True
        
    except ImportError:
        logger.warning("未安装onnxruntime，跳过性能测试")
        logger.info("安装命令: pip install onnxruntime")
        return False
    except Exception as e:
        logger.error(f"测试失败: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='将YOLO模型导出为ONNX格式',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基本转换
  python export_onnx.py models/yolo26n.pt
  
  # 指定尺寸和半精度
  python export_onnx.py models/yolo26n.pt --imgsz 320 --half
  
  # 自定义输出路径
  python export_onnx.py models/yolo26n.pt -o output/model.onnx
  
  # 转换后性能测试
  python export_onnx.py models/yolo26n.pt --benchmark
        """
    )
    
    parser.add_argument('model', type=str, help='输入模型路径 (.pt文件)')
    parser.add_argument('-o', '--output', type=str, default=None,
                       help='输出路径 (默认: 与输入同名，.onnx后缀)')
    parser.add_argument('--imgsz', type=int, default=320,
                       help='输入图像尺寸 (默认: 320)')
    parser.add_argument('--half', action='store_true',
                       help='使用半精度(FP16)，模型更小但可能损失精度')
    parser.add_argument('--no-simplify', action='store_true',
                       help='不简化模型（默认会简化）')
    parser.add_argument('--opset', type=int, default=12,
                       help='ONNX opset版本 (默认: 12)')
    parser.add_argument('--device', type=str, default='cpu', choices=['cpu', 'cuda'],
                       help='导出设备 (默认: cpu)')
    parser.add_argument('--benchmark', action='store_true',
                       help='导出后进行性能测试')
    
    args = parser.parse_args()
    
    # 执行导出
    success = export_onnx(
        model_path=args.model,
        output_path=args.output,
        imgsz=args.imgsz,
        half=args.half,
        simplify=not args.no_simplify,
        opset=args.opset,
        device=args.device
    )
    
    if success and args.benchmark:
        output_path = args.output or Path(args.model).with_suffix('.onnx')
        benchmark_onnx(output_path, args.imgsz)
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
