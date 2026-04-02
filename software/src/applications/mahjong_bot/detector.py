"""
麻将牌检测器 - 基于 YOLO

复用 HumanDetector 架构，检测 34 种国标麻将牌面：
- 万子: 1-9万
- 条子: 1-9条  
- 筒子: 1-9筒
- 字牌: 东、南、西、北、中、发、白
"""

from typing import List, Tuple, Optional
from dataclasses import dataclass
from pathlib import Path
import numpy as np

from common.logging import get_logger

logger = get_logger(__name__)


@dataclass
class MahjongTile:
    """麻将牌检测结果"""
    bbox: Tuple[int, int, int, int]  # [x1, y1, x2, y2]
    confidence: float                # 置信度 0-1
    class_id: int                    # 类别ID
    class_name: str                  # 类别名称，如 "5-wan", "dong", "fa"
    
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
        return self.bbox[2] - self.bbox[0]
    
    @property
    def height(self) -> int:
        return self.bbox[3] - self.bbox[1]


# 麻将牌类别映射（34种 + 牌背可选）
# 支持两种命名格式：项目格式 和 Roboflow 数据集格式
TILE_CLASSES = {
    # 项目格式（简洁命名）
    0: "1-wan", 1: "2-wan", 2: "3-wan", 3: "4-wan", 4: "5-wan",
    5: "6-wan", 6: "7-wan", 7: "8-wan", 8: "9-wan",
    9: "1-tiao", 10: "2-tiao", 11: "3-tiao", 12: "4-tiao", 13: "5-tiao",
    14: "6-tiao", 15: "7-tiao", 16: "8-tiao", 17: "9-tiao",
    18: "1-tong", 19: "2-tong", 20: "3-tong", 21: "4-tong", 22: "5-tong",
    23: "6-tong", 24: "7-tong", 25: "8-tong", 26: "9-tong",
    27: "dong", 28: "nan", 29: "xi", 30: "bei",
    31: "zhong", 32: "fa", 33: "bai",
    34: "back",  # 牌背（可选）
}

# Roboflow 数据集类别映射
TILE_CLASSES_ROBOFLOW = {
    0: "1-tiao", 1: "2-tiao", 2: "3-tiao", 3: "4-tiao", 4: "5-tiao",
    5: "6-tiao", 6: "7-tiao", 7: "8-tiao", 8: "9-tiao",
    9: "1-wan", 10: "2-wan", 11: "3-wan", 12: "4-wan", 13: "5-wan",
    14: "6-wan", 15: "7-wan", 16: "8-wan", 17: "9-wan",
    18: "1-tong", 19: "2-tong", 20: "3-tong", 21: "4-tong", 22: "5-tong",
    23: "6-tong", 24: "7-tong", 25: "8-tong", 26: "9-tong",
    27: "dong", 28: "fa", 29: "bei", 30: "zhong", 31: "nan", 32: "xi", 33: "bai",
}

# 中文显示映射（用于UI展示）
TILE_CLASSES_CN = {
    "1-wan": "一万", "2-wan": "二万", "3-wan": "三万", "4-wan": "四万", "5-wan": "五万",
    "6-wan": "六万", "7-wan": "七万", "8-wan": "八万", "9-wan": "九万",
    "1-tiao": "一条", "2-tiao": "二条", "3-tiao": "三条", "4-tiao": "四条", "5-tiao": "五条",
    "6-tiao": "六条", "7-tiao": "七条", "8-tiao": "八条", "9-tiao": "九条",
    "1-tong": "一筒", "2-tong": "二筒", "3-tong": "三筒", "4-tong": "四筒", "5-tong": "五筒",
    "6-tong": "六筒", "7-tong": "七筒", "8-tong": "八筒", "9-tong": "九筒",
    "dong": "东风", "nan": "南风", "xi": "西风", "bei": "北风",
    "zhong": "红中", "fa": "发财", "bai": "白板",
    "back": "牌背",
}


class MahjongDetector:
    """
    麻将牌检测器
    
    基于 Ultralytics YOLO 模型，复用现有 human_follow 的检测架构。
    """
    
    def __init__(self,
                 model_path: str = "models/mahjong_yolo.pt",
                 conf_threshold: float = 0.6,
                 inference_size: int = 640,
                 device: str = "cpu",
                 use_roboflow_classes: bool = True):
        """
        初始化麻将检测器
        
        Args:
            model_path: YOLO 模型路径
            conf_threshold: 检测置信度阈值
            inference_size: 推理输入尺寸
            device: 计算设备 (cpu/cuda)
            use_roboflow_classes: 是否使用 Roboflow 数据集类别映射
        """
        self.model_path = Path(model_path)
        self.conf_threshold = conf_threshold
        self.inference_size = inference_size
        self.device = device
        self.use_roboflow_classes = use_roboflow_classes
        self.tile_classes = TILE_CLASSES_ROBOFLOW if use_roboflow_classes else TILE_CLASSES
        
        self._model = None
        self._initialized = False
        
        logger.info(f"MahjongDetector 初始化:")
        logger.info(f"  模型路径: {model_path}")
        logger.info(f"  置信度阈值: {conf_threshold}")
        logger.info(f"  推理尺寸: {inference_size}x{inference_size}")
        logger.info(f"  设备: {device}")
        logger.info(f"  类别映射: {'Roboflow' if use_roboflow_classes else 'Standard'}")
    
    def _load_model(self) -> bool:
        """加载 YOLO 模型"""
        try:
            from ultralytics import YOLO
        except ImportError:
            logger.error("未安装 ultralytics 库，请运行: pip install ultralytics")
            return False
        
        if not self.model_path.exists():
            logger.warning(f"模型文件不存在: {self.model_path}")
            logger.info("请手动下载模型或训练后放置到该路径")
            return False
        
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
    
    def detect(self, frame: np.ndarray) -> List[MahjongTile]:
        """
        检测图像中的麻将牌
        
        Args:
            frame: OpenCV 图像 (BGR 格式)
            
        Returns:
            List[MahjongTile]: 检测结果列表，按 x 坐标从左到右排序
        """
        if not self._initialized:
            if not self.initialize():
                return []
        
        if frame is None or frame.size == 0:
            logger.warning("输入图像为空")
            return []
        
        try:
            results = self._model(
                frame,
                conf=self.conf_threshold,
                imgsz=self.inference_size,
                device=self.device,
                verbose=False
            )
            
            detections = []
            for result in results:
                if result.boxes is None:
                    continue
                
                for box in result.boxes:
                    # 转换为 Python 原生 int，避免 numpy int64 JSON 序列化问题
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    conf = float(box.conf[0].cpu().numpy())
                    cls_id = int(box.cls[0].cpu().numpy())
                    cls_name = self.tile_classes.get(cls_id, f"unknown-{cls_id}")
                    
                    detections.append(MahjongTile(
                        bbox=(x1, y1, x2, y2),
                        confidence=conf,
                        class_id=cls_id,
                        class_name=cls_name
                    ))
            
            # 按 x 坐标从左到右排序（便于手牌排列）
            detections.sort(key=lambda t: t.center[0])
            return detections
            
        except Exception as e:
            logger.error(f"检测失败: {e}")
            return []
    
    def detect_and_draw(self, frame: np.ndarray) -> Tuple[np.ndarray, List[MahjongTile]]:
        """
        检测并在图像上绘制结果
        
        Returns:
            (绘制后的图像, 检测结果列表)
        """
        import cv2
        
        detections = self.detect(frame)
        output = frame.copy()
        
        for tile in detections:
            x1, y1, x2, y2 = tile.bbox
            cx, cy = tile.center
            
            cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(output, (cx, cy), 4, (0, 0, 255), -1)
            
            label = f"{tile.class_name}: {tile.confidence:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            label_y = max(y1 - 10, label_size[1] + 10)
            
            cv2.rectangle(output,
                         (x1, label_y - label_size[1] - 5),
                         (x1 + label_size[0], label_y + 5),
                         (0, 255, 0), -1)
            cv2.putText(output, label, (x1, label_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        info_text = f"Tiles: {len(detections)}"
        cv2.putText(output, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return output, detections
    
    def release(self):
        """释放资源"""
        self._model = None
        self._initialized = False
        logger.info("MahjongDetector 已释放")


if __name__ == "__main__":
    import cv2
    
    detector = MahjongDetector(
        model_path="models/mahjong_yolo.pt",
        conf_threshold=0.6,
        inference_size=640,
        device="cpu",
        use_roboflow_classes=True
    )
    
    if not detector.initialize():
        logger.error("检测器初始化失败，请确认模型文件存在")
        exit(1)
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    logger.info("按 'q' 退出测试")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        output, detections = detector.detect_and_draw(frame)
        cv2.imshow("Mahjong Detection", output)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    detector.release()
