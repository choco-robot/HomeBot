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
import sys
import cv2

# Windows 上默认使用 DirectShow 后端，避免 MSMF 不稳定问题
_DEFAULT_CAP_API = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2


@dataclass
class MahjongTile:
    """麻将牌检测结果"""
    bbox: Tuple[int, int, int, int]  # [x1, y1, x2, y2]
    confidence: float                # 置信度 0-1
    class_id: int                    # 类别ID
    class_name: str                  # 类别名称，如 "5-wan", "dong", "fa"
    track_id: Optional[int] = None   # 跟踪ID（仅在使用track模式时有效）
    
    @property
    def center(self) -> Tuple[int, int]:
        """计算检测框中心点"""
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) // 2, (y1 + y2) // 2)
    
    @property
    def bottom_center(self) -> Tuple[int, int]:
        """计算检测框底部中心点（用于推倒牌）"""
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) // 2, y2)
    
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


def _get_default_model_path() -> str:
    """
    自动推断默认模型路径
    
    尝试从当前文件位置或运行目录找到 models/mahjong_yolo.pt
    """
    # 尝试路径（按优先级）
    possible_paths = [
        # 1. 相对于当前文件的路径 (src/applications/mahjong_bot/detector.py -> ../../../models/)
        Path(__file__).parent.parent.parent.parent / "models" / "mahjong_yolo.pt",
        # 2. 相对于运行目录的 models/
        Path("models/mahjong_yolo.pt"),
        # 3. 相对于运行目录的上级 (从 src/ 运行)
        Path("../../models/mahjong_yolo.pt"),
        # 4. 绝对路径检查
        Path("/models/mahjong_yolo.pt"),
    ]
    
    for path in possible_paths:
        if path.exists():
            return str(path)
    
    # 默认返回相对路径，让调用者处理错误提示
    return "models/mahjong_yolo.pt"


class MahjongDetector:
    """
    麻将牌检测器
    
    基于 Ultralytics YOLO 模型，复用现有 human_follow 的检测架构。
    """
    
    def __init__(self,
                 model_path: str = None,
                 conf_threshold: float = 0.6,
                 inference_size: int = 640,
                 device: str = "cuda",
                 use_roboflow_classes: bool = True,
                 roi_enabled: bool = False,
                 roi_x: int = 0,
                 roi_y: int = 0,
                 roi_width: int = 0,
                 roi_height: int = 0,
                 use_tracking: bool = True,
                 tracker: str = "bytetrack.yaml",
                 track_persist: bool = True):
        """
        初始化麻将检测器
        
        Args:
            model_path: YOLO 模型路径
            conf_threshold: 检测置信度阈值
            inference_size: 推理输入尺寸
            device: 计算设备 (cpu/cuda)
            use_roboflow_classes: 是否使用 Roboflow 数据集类别映射
            roi_enabled: 是否启用 ROI 过滤
            roi_x: ROI 左上角 X 坐标
            roi_y: ROI 左上角 Y 坐标
            roi_width: ROI 宽度
            roi_height: ROI 高度
            use_tracking: 是否启用跟踪模式（替代纯检测）
            tracker: 跟踪器配置（bytetrack.yaml 或 botsort.yaml）
            track_persist: 是否在帧间保持跟踪ID
        """
        # 如果未指定模型路径，自动推断
        if model_path is None:
            model_path = _get_default_model_path()
        
        self.model_path = Path(model_path)
        self.conf_threshold = conf_threshold
        self.inference_size = inference_size
        self.device = device
        self.use_roboflow_classes = use_roboflow_classes
        self.roi_enabled = roi_enabled
        self.roi_x = roi_x
        self.roi_y = roi_y
        self.roi_width = roi_width
        self.roi_height = roi_height
        self.tile_classes = TILE_CLASSES_ROBOFLOW if use_roboflow_classes else TILE_CLASSES
        
        # 跟踪相关配置
        self.use_tracking = use_tracking
        self.tracker = tracker
        self.track_persist = track_persist
        
        self._model = None
        self._initialized = False
        
        logger.info(f"MahjongDetector 初始化:")
        logger.info(f"  模型路径: {model_path}")
        logger.info(f"  置信度阈值: {conf_threshold}")
        logger.info(f"  推理尺寸: {inference_size}x{inference_size}")
        logger.info(f"  设备: {device}")
        logger.info(f"  类别映射: {'Roboflow' if use_roboflow_classes else 'Standard'}")
        logger.info(f"  ROI 启用: {roi_enabled}")
        if roi_enabled:
            logger.info(f"  ROI 区域: x={roi_x}, y={roi_y}, w={roi_width}, h={roi_height}")
        logger.info(f"  跟踪模式: {use_tracking}")
        if use_tracking:
            logger.info(f"  跟踪器: {tracker}, 持久化: {track_persist}")

    def _is_within_roi(self, tile: MahjongTile) -> bool:
        """判断检测结果是否位于 ROI 区域内（使用中心点判断）。"""
        if not self.roi_enabled or self.roi_width <= 0 or self.roi_height <= 0:
            return True

        cx, cy = tile.center
        return (self.roi_x <= cx <= self.roi_x + self.roi_width and
                self.roi_y <= cy <= self.roi_y + self.roi_height)

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
        检测图像中的麻将牌（使用跟踪模式以获得更稳定的检测）
        
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
            if self.use_tracking:
                # 使用跟踪模式：在连续帧间保持目标ID，减少丢失
                results = self._model.track(
                    frame,
                    conf=self.conf_threshold,
                    imgsz=self.inference_size,
                    device=self.device,
                    tracker=self.tracker,
                    persist=self.track_persist,
                    verbose=False
                )
            else:
                # 回退到纯检测模式
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
                    
                    # 获取跟踪ID（如果可用）
                    track_id = None
                    if hasattr(box, 'id') and box.id is not None:
                        track_id = int(box.id[0].cpu().numpy())
                    
                    detections.append(MahjongTile(
                        bbox=(x1, y1, x2, y2),
                        confidence=conf,
                        class_id=cls_id,
                        class_name=cls_name,
                        track_id=track_id
                    ))
            
            # 仅保留 ROI 内的检测结果
            if self.roi_enabled and self.roi_width > 0 and self.roi_height > 0:
                detections = [t for t in detections if self._is_within_roi(t)]

            # 按 x 坐标从左到右排序（便于手牌排列）
            detections.sort(key=lambda t: t.center[0])
            
            if detections:
                names = [t.class_name for t in detections]
                logger.info(f"检测到 {len(detections)} 张牌: {names}")
            else:
                logger.info("未检测到牌")
            
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
            
            # 显示标签包含跟踪ID（如果有）
            if tile.track_id is not None:
                label = f"{tile.class_name} #{tile.track_id}: {tile.confidence:.2f}"
            else:
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
        conf_threshold=0.2,
        inference_size=640,
        device="cuda",
        use_roboflow_classes=True
    )
    
    if not detector.initialize():
        logger.error("检测器初始化失败，请确认模型文件存在")
        exit(1)
    
    cap = cv2.VideoCapture(1, _DEFAULT_CAP_API)
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
