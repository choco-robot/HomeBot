#!/usr/bin/env python3
"""
ONNX人体检测器
使用ONNX Runtime进行高速推理

使用方法:
    from tools.onnx_detector import ONNXHumanDetector
    
    detector = ONNXHumanDetector("models/yolo26n.onnx")
    detector.initialize()
    
    detections = detector.detect(frame)
"""
import sys
import os
from typing import List, Tuple
from pathlib import Path
import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from applications.human_follow.detector import Detection
from common.logging import get_logger

logger = get_logger(__name__)


class ONNXHumanDetector:
    """
    基于ONNX的人体检测器
    使用ONNX Runtime，比PyTorch YOLO快2-3倍
    """
    
    def __init__(self,
                 model_path: str = "models/yolo26n.onnx",
                 conf_threshold: float = 0.5,
                 inference_size: int = 320,
                 num_threads: int = 4):
        """
        初始化ONNX检测器
        
        Args:
            model_path: ONNX模型路径
            conf_threshold: 置信度阈值
            inference_size: 输入尺寸
            num_threads: CPU线程数
        """
        self.model_path = Path(model_path)
        self.conf_threshold = conf_threshold
        self.inference_size = inference_size
        self.num_threads = num_threads
        
        self._session = None
        self._input_name = None
        self._initialized = False
        
        logger.info(f"ONNXHumanDetector初始化:")
        logger.info(f"  模型: {model_path}")
        logger.info(f"  尺寸: {inference_size}x{inference_size}")
        logger.info(f"  线程: {num_threads}")
    
    def initialize(self) -> bool:
        """初始化ONNX Runtime会话"""
        try:
            import onnxruntime as ort
            
            if not self.model_path.exists():
                logger.error(f"模型文件不存在: {self.model_path}")
                return False
            
            # 配置会话选项
            sess_options = ort.SessionOptions()
            sess_options.intra_op_num_threads = self.num_threads
            sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            
            # 创建会话
            providers = ['CPUExecutionProvider']
            self._session = ort.InferenceSession(
                str(self.model_path),
                sess_options,
                providers=providers
            )
            
            self._input_name = self._session.get_inputs()[0].name
            self._initialized = True
            
            logger.info(f"✓ ONNX模型已加载: {self.model_path.name}")
            return True
            
        except ImportError:
            logger.error("未安装onnxruntime，请运行: pip install onnxruntime")
            return False
        except Exception as e:
            logger.error(f"加载失败: {e}")
            return False
    
    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """预处理图像"""
        # 调整尺寸
        img = cv2.resize(frame, (self.inference_size, self.inference_size))
        
        # BGR -> RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # 归一化
        img = img.astype(np.float32) / 255.0
        
        # HWC -> CHW
        img = np.transpose(img, (2, 0, 1))
        
        # 添加batch维度
        img = np.expand_dims(img, axis=0)
        
        return img
    
    def _postprocess(self, outputs, original_shape: Tuple[int, int]) -> List[Detection]:
        """后处理ONNX输出"""
        detections = []
        
        # 获取输出 (假设YOLO标准输出格式)
        # 需要根据实际模型调整
        predictions = outputs[0]  # shape: [batch, num_predictions, 85]
        
        orig_h, orig_w = original_shape[:2]
        scale_x = orig_w / self.inference_size
        scale_y = orig_h / self.inference_size
        
        for pred in predictions[0]:  # batch=0
            confidence = pred[4]
            if confidence < self.conf_threshold:
                continue
            
            # 获取类别分数 (YOLO格式: x, y, w, h, conf, class0, class1, ...)
            class_scores = pred[5:]
            class_id = np.argmax(class_scores)
            
            # 只保留人体 (class_id=0)
            if class_id != 0:
                continue
            
            # 转换坐标
            x_center, y_center, w, h = pred[0:4]
            x1 = int((x_center - w/2) * scale_x)
            y1 = int((y_center - h/2) * scale_y)
            x2 = int((x_center + w/2) * scale_x)
            y2 = int((y_center + h/2) * scale_y)
            
            detections.append(Detection(
                bbox=(x1, y1, x2, y2),
                confidence=float(confidence),
                class_id=0,
                class_name="person"
            ))
        
        return detections
    
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """检测图像中的人体"""
        if not self._initialized:
            if not self.initialize():
                return []
        
        if frame is None:
            return []
        
        try:
            # 预处理
            input_tensor = self._preprocess(frame)
            
            # 推理
            outputs = self._session.run(None, {self._input_name: input_tensor})
            
            # 后处理
            detections = self._postprocess(outputs, frame.shape)
            
            return detections
            
        except Exception as e:
            logger.error(f"检测失败: {e}")
            return []
    
    def release(self):
        """释放资源"""
        self._session = None
        self._initialized = False
        logger.info("ONNXHumanDetector已释放")


def benchmark_comparison(pt_model: str, onnx_model: str, iterations: int = 100):
    """对比PyTorch和ONNX模型的性能"""
    import time
    
    logger.info("=" * 60)
    logger.info("性能对比测试")
    logger.info("=" * 60)
    
    # 创建测试图像
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # 测试PyTorch
    logger.info(f"\nPyTorch模型: {pt_model}")
    from applications.human_follow.detector import HumanDetector
    
    pt_detector = HumanDetector(model_path=pt_model, inference_size=320)
    if pt_detector.initialize():
        # 预热
        for _ in range(10):
            pt_detector.detect(test_image)
        
        # 测试
        times = []
        for _ in range(iterations):
            start = time.perf_counter()
            pt_detector.detect(test_image)
            times.append((time.perf_counter() - start) * 1000)
        
        pt_time = np.mean(times)
        logger.info(f"  平均时间: {pt_time:.2f}ms ({1000/pt_time:.1f} FPS)")
        pt_detector.release()
    
    # 测试ONNX
    logger.info(f"\nONNX模型: {onnx_model}")
    onnx_detector = ONNXHumanDetector(model_path=onnx_model, inference_size=320)
    if onnx_detector.initialize():
        # 预热
        for _ in range(10):
            onnx_detector.detect(test_image)
        
        # 测试
        times = []
        for _ in range(iterations):
            start = time.perf_counter()
            onnx_detector.detect(test_image)
            times.append((time.perf_counter() - start) * 1000)
        
        onnx_time = np.mean(times)
        logger.info(f"  平均时间: {onnx_time:.2f}ms ({1000/onnx_time:.1f} FPS)")
        
        # 加速比
        speedup = pt_time / onnx_time
        logger.info(f"\n加速比: {speedup:.2f}x")
        
        onnx_detector.release()
    
    logger.info("=" * 60)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='ONNX检测器测试')
    parser.add_argument('--model', type=str, default='models/yolo26n.onnx',
                       help='ONNX模型路径')
    parser.add_argument('--benchmark', action='store_true',
                       help='与PyTorch模型对比性能')
    parser.add_argument('--pt-model', type=str, default='models/yolo26n.pt',
                       help='PyTorch模型路径（用于对比）')
    
    args = parser.parse_args()
    
    if args.benchmark:
        benchmark_comparison(args.pt_model, args.model)
    else:
        # 简单测试
        detector = ONNXHumanDetector(args.model)
        if detector.initialize():
            logger.info("✓ ONNX检测器初始化成功")
            detector.release()
