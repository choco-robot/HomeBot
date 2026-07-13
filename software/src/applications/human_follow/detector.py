"""
人体检测器 - 基于YOLO26
使用Ultralytics YOLO库进行人体检测
"""
from typing import List, Optional, Tuple
from dataclasses import dataclass
from pathlib import Path
import numpy as np

from common.logging import get_logger

logger = get_logger(__name__)


@dataclass
class Detection:
    """检测结果数据结构"""
    bbox: Tuple[int, int, int, int]  # [x1, y1, x2, y2]
    confidence: float                # 置信度 0-1
    class_id: int                    # 类别ID (person=0)
    class_name: str                  # 类别名称
    
    @property
    def center(self) -> Tuple[int, int]:
        """计算检测框中心点"""
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) // 2, (y1 + y2) // 2)
    
    @property
    def area(self) -> int:
        """计算检测框面积"""
        x1, y1, x2, y2 = self.bbox
        return max(0, x2 - x1) * max(0, y2 - y1)
    
    @property
    def width(self) -> int:
        """检测框宽度"""
        return self.bbox[2] - self.bbox[0]
    
    @property
    def height(self) -> int:
        """检测框高度"""
        return self.bbox[3] - self.bbox[1]


class HumanDetector:
    """
    人体/人脸检测器 - 使用YOLO26 / YOLOv8-Face
    
    支持YOLO26、YOLO11、YOLOv8等系列模型
    通过 detect_mode 参数切换人体检测或人脸检测模式
    自动处理模型下载和加载
    """
    
    # 检测模式配置
    _MODE_CONFIG = {
        "person": {
            "default_model": "models/yolo26n.pt",
            "default_inference_size": 320,
            "class_id": 0,
            "class_name": "person",
            "box_color": (0, 255, 0),      # 绿色
            "center_color": (0, 0, 255),   # 红色
        },
        "face": {
            "default_model": "models/yolov8n-face-lindevs.pt",
            "default_inference_size": 640,
            "class_id": 0,
            "class_name": "face",
            "box_color": (255, 0, 255),    # 紫色
            "center_color": (255, 255, 0), # 青色
        },
    }
    
    def __init__(self, 
                 model_path: str = None,
                 conf_threshold: float = 0.5,
                 inference_size: int = None,
                 use_half: bool = False,
                 device: str = "cpu",
                 detect_mode: str = "person"):
        """
        初始化检测器
        
        Args:
            model_path: YOLO模型文件路径 (None 则根据 detect_mode 使用默认模型)
            conf_threshold: 检测置信度阈值
            inference_size: 推理输入尺寸 (None 则根据 detect_mode 使用默认值)
            use_half: 是否使用FP16半精度
            device: 计算设备 (auto/cpu/cuda/mps)
            detect_mode: 检测模式 - "person" 人体检测 / "face" 人脸检测
        """
        if detect_mode not in self._MODE_CONFIG:
            raise ValueError(f"不支持的检测模式: {detect_mode}, 可选: {list(self._MODE_CONFIG.keys())}")
        
        self.detect_mode = detect_mode
        self._mode_cfg = self._MODE_CONFIG[detect_mode]
        
        # 未指定模型路径时使用默认路径
        if model_path is None:
            model_path = self._mode_cfg["default_model"]
        
        # 未指定推理尺寸时使用模式默认值
        if inference_size is None:
            inference_size = self._mode_cfg["default_inference_size"]
        
        self.model_path = Path(model_path)
        self.conf_threshold = conf_threshold
        self.inference_size = inference_size
        self.use_half = use_half
        self.device = device
        
        self._model = None
        self._initialized = False
        
        logger.info(f"HumanDetector初始化 [模式: {detect_mode}]:")
        logger.info(f"  模型路径: {model_path}")
        logger.info(f"  置信度阈值: {conf_threshold}")
        logger.info(f"  推理尺寸: {inference_size}x{inference_size}")
        logger.info(f"  半精度: {use_half}")
        logger.info(f"  设备: {device}")
    
    def _resolve_model_path(self) -> Path:
        """解析模型路径：优先检查 .onnx，回退到 .pt/.torchscript 等.
        
        支持从多个位置查找模型：
        1. 传入路径的直接 .onnx 版本
        2. 项目根目录的 models/ 文件夹（基于 detector.py 位置计算）
        3. 当前运行目录的 models/ 文件夹
        4. 传入路径的原始文件
        
        Returns:
            实际要加载的模型文件路径
        """
        # 计算项目 models/ 目录的绝对路径
        # detector.py 在 software/src/applications/human_follow/ 下
        detector_dir = Path(__file__).resolve().parent
        project_models_dir = detector_dir.parent.parent.parent / "models"
        
        onnx_name = self.model_path.with_suffix(".onnx").name
        pt_name = self.model_path.name
        
        candidates = []
        
        # 1. 与传入路径同目录的 .onnx
        candidates.append(self.model_path.with_suffix(".onnx"))
        
        # 2. 项目根目录 models/ 下的 .onnx
        candidates.append(project_models_dir / onnx_name)
        
        # 3. 当前运行目录 models/ 下的 .onnx
        candidates.append(Path("models") / onnx_name)
        
        # 4. 项目根目录 models/ 下的 .pt
        candidates.append(project_models_dir / pt_name)
        
        # 5. 当前运行目录 models/ 下的 .pt
        candidates.append(Path("models") / pt_name)
        
        # 6. 传入路径的原始文件
        candidates.append(self.model_path)
        
        for path in candidates:
            if path.exists():
                logger.info(f"发现模型: {path}")
                return path
        
        # 都找不到，返回原始路径（让后续逻辑报错）
        logger.debug(f"模型搜索路径: {[str(c) for c in candidates]}")
        return self.model_path
    
    def _load_model(self) -> bool:
        """加载YOLO模型"""
        try:
            from ultralytics import YOLO
        except ImportError:
            logger.error("未安装ultralytics库，请运行: pip install ultralytics")
            return False
        
        # 解析最终要加载的模型路径（优先 ONNX）
        model_to_load = self._resolve_model_path()
        
        # 检查模型文件是否存在
        if not model_to_load.exists():
            logger.warning(f"模型文件不存在: {model_to_load}")
            logger.info("尝试自动下载模型...")
            
            # 尝试使用ultralytics自动下载（下载 .pt 格式）
            try:
                model_name = self.model_path.name
                self._model = YOLO(model_name)
                logger.info(f"✓ 自动下载并加载模型: {model_name}")
                self._initialized = True
                return True
            except Exception as e:
                logger.error(f"自动下载失败: {e}")
                logger.info("请手动下载模型或运行: python tools/download_models.py")
                return False
        
        # 加载本地模型
        try:
            self._model = YOLO(str(model_to_load))
            logger.info(f"✓ 加载本地模型: {model_to_load}")
            self._initialized = True
            return True
        except Exception as e:
            logger.error(f"加载模型失败: {e}")
            return False
    
    def initialize(self) -> bool:
        """初始化检测器"""
        if self._initialized:
            return True
        return self._load_model()
    
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        检测图像中的人体或人脸
        
        Args:
            frame: OpenCV图像 (BGR格式)
            
        Returns:
            List[Detection]: 检测结果列表
        """
        if not self._initialized:
            if not self.initialize():
                return []
        
        if frame is None or frame.size == 0:
            logger.warning("输入图像为空")
            return []
        
        try:
            # 运行YOLO推理
            results = self._model(
                frame,
                conf=self.conf_threshold,
                classes=[self._mode_cfg["class_id"]],  # 只检测目标类别
                imgsz=self.inference_size,
                quantize='fp16' if self.use_half else None,
                device=self.device,
                verbose=False  # 禁用ultralytics的默认输出
            )
            
            # 解析结果
            detections = []
            class_name = self._mode_cfg["class_name"]
            for result in results:
                if result.boxes is None:
                    continue
                    
                for box in result.boxes:
                    # 获取坐标
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    conf = float(box.conf[0].cpu().numpy())
                    cls_id = int(box.cls[0].cpu().numpy())
                    
                    detection = Detection(
                        bbox=(x1, y1, x2, y2),
                        confidence=conf,
                        class_id=cls_id,
                        class_name=class_name
                    )
                    detections.append(detection)
            
            return detections
            
        except Exception as e:
            logger.error(f"检测失败: {e}")
            return []
    
    def detect_and_draw(self, frame: np.ndarray) -> Tuple[np.ndarray, List[Detection]]:
        """
        检测并在图像上绘制结果
        
        Args:
            frame: 输入图像
            
        Returns:
            (绘制后的图像, 检测结果列表)
        """
        import cv2
        
        detections = self.detect(frame)
        output = frame.copy()
        
        box_color = self._mode_cfg["box_color"]
        center_color = self._mode_cfg["center_color"]
        mode_label = self.detect_mode.upper()
        
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            cx, cy = det.center
            
            # 绘制检测框
            cv2.rectangle(output, (x1, y1), (x2, y2), box_color, 2)
            
            # 绘制中心点
            cv2.circle(output, (cx, cy), 4, center_color, -1)
            
            # 绘制标签
            label = f"{det.class_name}: {det.confidence:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            label_y = max(y1 - 10, label_size[1] + 10)
            
            cv2.rectangle(output, 
                         (x1, label_y - label_size[1] - 5),
                         (x1 + label_size[0], label_y + 5),
                         box_color, -1)
            cv2.putText(output, label, (x1, label_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        # 绘制检测数量和模式标签
        info_text = f"[{mode_label}] Detections: {len(detections)}"
        cv2.putText(output, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
        
        return output, detections
    
    def get_model_info(self) -> dict:
        """获取模型信息"""
        if not self._initialized:
            return {"status": "not_loaded"}
        
        try:
            info = {
                "status": "loaded",
                "model_name": getattr(self._model, 'model_name', 'unknown'),
                "task": getattr(self._model, 'task', 'unknown'),
            }
            
            # 尝试获取模型参数数量
            if hasattr(self._model, 'model') and hasattr(self._model.model, 'parameters'):
                params = sum(p.numel() for p in self._model.model.parameters())
                info["parameters"] = f"{params / 1e6:.2f}M"
            
            return info
        except Exception as e:
            return {"status": "error", "error": str(e)}
    
    def release(self):
        """释放资源"""
        self._model = None
        self._initialized = False
        logger.info("HumanDetector已释放")


# 简单的测试代码
if __name__ == "__main__":
    import cv2
    import argparse
    
    parser = argparse.ArgumentParser(description='Human/Face Detector Test')
    parser.add_argument('--mode', default='person', choices=['person', 'face'],
                       help='检测模式: person=人体, face=人脸')
    parser.add_argument('--model', default=None, help='模型路径 (None 使用模式默认)')
    parser.add_argument('--conf', type=float, default=0.5, help='置信度阈值')
    parser.add_argument('--source', type=int, default=0, help='摄像头设备索引')
    args = parser.parse_args()
    
    # 初始化检测器
    detector = HumanDetector(
        model_path=args.model,
        conf_threshold=args.conf,
        device='cpu',
        use_half=True,
        detect_mode=args.mode
    )
    
    if not detector.initialize():
        logger.error("检测器初始化失败")
        exit(1)
    
    # 打印模型信息
    info = detector.get_model_info()
    logger.info(f"模型信息: {info}")
    
    # 测试摄像头
    cap = cv2.VideoCapture(args.source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    logger.info(f"按 'q' 退出测试 [模式: {args.mode}]")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 检测并绘制
        output, detections = detector.detect_and_draw(frame)
        
        # 显示结果
        window_title = f"{args.mode.upper()} Detection"
        cv2.imshow(window_title, output)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    detector.release()
