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
    人体检测器 - 使用YOLO26
    
    支持YOLO26、YOLO11、YOLOv8等系列模型
    自动处理模型下载和加载
    """
    
    # COCO数据集人体类别ID
    PERSON_CLASS_ID = 0
    PERSON_CLASS_NAME = "person"
    
    def __init__(self, 
                 model_path: str = "models/yolo26n.pt",
                 conf_threshold: float = 0.5,
                 inference_size: int = 320,
                 use_half: bool = False,
                 device: str = "cpu"):
        """
        初始化人体检测器
        
        Args:
            model_path: YOLO模型文件路径
            conf_threshold: 检测置信度阈值
            inference_size: 推理输入尺寸
            use_half: 是否使用FP16半精度
            device: 计算设备 (auto/cpu/cuda/mps)
        """
        self.model_path = Path(model_path)
        self.conf_threshold = conf_threshold
        self.inference_size = inference_size
        self.use_half = use_half
        self.device = device
        
        self._model = None
        self._initialized = False
        
        logger.info(f"HumanDetector初始化:")
        logger.info(f"  模型路径: {model_path}")
        logger.info(f"  置信度阈值: {conf_threshold}")
        logger.info(f"  推理尺寸: {inference_size}x{inference_size}")
        logger.info(f"  半精度: {use_half}")
        logger.info(f"  设备: {device}")
    
    def _load_model(self) -> bool:
        """加载YOLO模型"""
        try:
            from ultralytics import YOLO
        except ImportError:
            logger.error("未安装ultralytics库，请运行: pip install ultralytics")
            return False
        
        # 检查模型文件是否存在
        if not self.model_path.exists():
            logger.warning(f"模型文件不存在: {self.model_path}")
            logger.info("尝试自动下载模型...")
            
            # 尝试使用ultralytics自动下载
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
            self._model = YOLO(str(self.model_path))
            logger.info(f"✓ 加载本地模型: {self.model_path}")
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
        检测图像中的人体
        
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
                classes=[self.PERSON_CLASS_ID],  # 只检测人体
                imgsz=self.inference_size,
                half=self.use_half,
                device=self.device,
                verbose=False  # 禁用ultralytics的默认输出
            )
            
            # 解析结果
            detections = []
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
                        class_name=self.PERSON_CLASS_NAME
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
        
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            cx, cy = det.center
            
            # 绘制检测框
            cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 绘制中心点
            cv2.circle(output, (cx, cy), 4, (0, 0, 255), -1)
            
            # 绘制标签
            label = f"{det.class_name}: {det.confidence:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            label_y = max(y1 - 10, label_size[1] + 10)
            
            cv2.rectangle(output, 
                         (x1, label_y - label_size[1] - 5),
                         (x1 + label_size[0], label_y + 5),
                         (0, 255, 0), -1)
            cv2.putText(output, label, (x1, label_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        # 绘制检测数量
        info_text = f"Detections: {len(detections)}"
        cv2.putText(output, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
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
    
    # 初始化检测器
    detector = HumanDetector(
        model_path="models/yolo26n.pt",
        conf_threshold=0.5,
        inference_size=320,
        device='cpu',
        use_half=True
    )
    
    if not detector.initialize():
        logger.error("检测器初始化失败")
        exit(1)
    
    # 打印模型信息
    info = detector.get_model_info()
    logger.info(f"模型信息: {info}")
    
    # 测试摄像头
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    logger.info("按 'q' 退出测试")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 检测并绘制
        output, detections = detector.detect_and_draw(frame)
        
        # 显示结果
        cv2.imshow("Human Detection (YOLO26)", output)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    detector.release()
