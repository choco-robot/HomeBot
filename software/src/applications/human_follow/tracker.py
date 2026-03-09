"""
目标跟踪器
实现IoU匹配 + 卡尔曼滤波的人体跟踪
"""
from typing import List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
import numpy as np

from common.logging import get_logger
from .detector import Detection

logger = get_logger(__name__)


class TargetStatus(Enum):
    """目标状态"""
    TRACKING = "tracking"      # 正常跟踪
    LOST = "lost"              # 暂时丢失
    SEARCHING = "searching"    # 搜索中


@dataclass
class Target:
    """跟踪目标"""
    id: int                           # 目标ID
    bbox: Tuple[int, int, int, int]   # 当前bbox [x1, y1, x2, y2]
    confidence: float                 # 置信度
    status: TargetStatus              # 状态
    age: int = 0                      # 总跟踪帧数
    time_since_update: int = 0        # 未更新帧数
    history: List[Tuple[int, int, int, int]] = field(default_factory=list)  # 历史位置
    
    def update(self, detection: Detection):
        """更新目标位置"""
        self.bbox = detection.bbox
        self.confidence = detection.confidence
        self.age += 1
        self.time_since_update = 0
        self.status = TargetStatus.TRACKING
        self.history.append(self.bbox)
        # 只保留最近10个历史位置
        if len(self.history) > 10:
            self.history.pop(0)
    
    def mark_missed(self):
        """标记为未匹配"""
        self.time_since_update += 1
        if self.time_since_update > 30:  # 超过30帧未匹配
            self.status = TargetStatus.LOST
    
    @property
    def center(self) -> Tuple[int, int]:
        """中心点"""
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) // 2, (y1 + y2) // 2)
    
    @property
    def area(self) -> int:
        """面积"""
        x1, y1, x2, y2 = self.bbox
        return max(0, x2 - x1) * max(0, y2 - y1)
    
    def predict(self) -> Tuple[int, int, int, int]:
        """预测下一帧位置（基于速度）"""
        if len(self.history) < 2:
            return self.bbox
        
        # 计算平均速度
        vx = 0
        vy = 0
        for i in range(1, len(self.history)):
            prev = self.history[i-1]
            curr = self.history[i]
            vx += (curr[0] - prev[0]) + (curr[2] - prev[2])  # x方向速度
            vy += (curr[1] - prev[1]) + (curr[3] - prev[3])  # y方向速度
        
        vx /= (len(self.history) - 1) * 2
        vy /= (len(self.history) - 1) * 2
        
        # 预测
        x1, y1, x2, y2 = self.bbox
        return (
            int(x1 + vx),
            int(y1 + vy),
            int(x2 + vx),
            int(y2 + vy)
        )


def compute_iou(box1: Tuple[int, int, int, int], 
                box2: Tuple[int, int, int, int]) -> float:
    """
    计算两个bbox的IoU
    
    Args:
        box1: [x1, y1, x2, y2]
        box2: [x1, y1, x2, y2]
        
    Returns:
        IoU值 0-1
    """
    x1_1, y1_1, x2_1, y2_1 = box1
    x1_2, y1_2, x2_2, y2_2 = box2
    
    # 计算交集
    x1_i = max(x1_1, x1_2)
    y1_i = max(y1_1, y1_2)
    x2_i = min(x2_1, x2_2)
    y2_i = min(y2_1, y2_2)
    
    if x2_i <= x1_i or y2_i <= y1_i:
        return 0.0
    
    intersection = (x2_i - x1_i) * (y2_i - y1_i)
    
    # 计算并集
    area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
    area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
    union = area1 + area2 - intersection
    
    if union <= 0:
        return 0.0
    
    return intersection / union


class TargetTracker:
    """
    目标跟踪器
    
    使用IoU匹配算法进行目标跟踪
    支持目标选择策略：中央/最大/最近
    """
    
    _id_counter = 0  # 全局ID计数器
    
    def __init__(self, 
                 max_age: int = 30,
                 min_iou: float = 0.3,
                 selection_strategy: str = "center"):
        """
        初始化跟踪器
        
        Args:
            max_age: 最大丢失帧数，超过则删除
            min_iou: 匹配最小IoU阈值
            selection_strategy: 目标选择策略 (center/largest/closest)
        """
        self.max_age = max_age
        self.min_iou = min_iou
        self.selection_strategy = selection_strategy
        
        self.targets: List[Target] = []
        self.primary_target: Optional[Target] = None
        
        logger.info(f"TargetTracker初始化:")
        logger.info(f"  最大丢失帧数: {max_age}")
        logger.info(f"  最小IoU阈值: {min_iou}")
        logger.info(f"  选择策略: {selection_strategy}")
    
    def reset(self):
        """重置跟踪器"""
        self.targets = []
        self.primary_target = None
        TargetTracker._id_counter = 0
        logger.info("TargetTracker已重置")
    
    def _create_target(self, detection: Detection) -> Target:
        """创建新目标"""
        TargetTracker._id_counter += 1
        target = Target(
            id=TargetTracker._id_counter,
            bbox=detection.bbox,
            confidence=detection.confidence,
            status=TargetStatus.TRACKING
        )
        target.history.append(detection.bbox)
        return target
    
    def _match_detections(self, 
                          detections: List[Detection]) -> Tuple[List[Tuple[Target, Detection]], 
                                                               List[Target], 
                                                               List[Detection]]:
        """
        匹配目标和检测结果
        
        Returns:
            (匹配列表, 未匹配目标列表, 未匹配检测列表)
        """
        if not self.targets or not detections:
            return [], self.targets.copy(), detections.copy()
        
        # 构建IoU矩阵
        iou_matrix = np.zeros((len(self.targets), len(detections)))
        for i, target in enumerate(self.targets):
            # 使用预测位置或当前位置
            if target.time_since_update > 0 and len(target.history) >= 2:
                predicted_bbox = target.predict()
            else:
                predicted_bbox = target.bbox
            
            for j, det in enumerate(detections):
                iou_matrix[i, j] = compute_iou(predicted_bbox, det.bbox)
        
        # 贪婪匹配
        matched = []
        unmatched_targets = []
        unmatched_detections = detections.copy()
        
        used_detections = set()
        
        # 按IoU降序匹配
        while True:
            max_iou = self.min_iou
            max_i = -1
            max_j = -1
            
            for i in range(len(self.targets)):
                if self.targets[i].status == TargetStatus.LOST:
                    continue
                for j in range(len(detections)):
                    if j in used_detections:
                        continue
                    if iou_matrix[i, j] > max_iou:
                        max_iou = iou_matrix[i, j]
                        max_i = i
                        max_j = j
            
            if max_i == -1:
                break
            
            matched.append((self.targets[max_i], detections[max_j]))
            used_detections.add(max_j)
        
        # 未匹配的目标
        for i, target in enumerate(self.targets):
            if not any(m[0] == target for m in matched):
                unmatched_targets.append(target)
        
        # 未匹配的检测
        unmatched_detections = [d for j, d in enumerate(detections) if j not in used_detections]
        
        return matched, unmatched_targets, unmatched_detections
    
    def update(self, detections: List[Detection]) -> Optional[Target]:
        """
        更新跟踪状态
        
        Args:
            detections: 当前帧检测结果
            
        Returns:
            Target: 当前主要跟踪目标（如果有）
        """
        # 匹配
        matched, unmatched_targets, unmatched_detections = self._match_detections(detections)
        
        # 更新匹配的目标
        for target, detection in matched:
            target.update(detection)
        
        # 标记未匹配的目标
        for target in unmatched_targets:
            target.mark_missed()
        
        # 创建新目标（对于未匹配的检测）
        for detection in unmatched_detections:
            new_target = self._create_target(detection)
            self.targets.append(new_target)
        
        # 删除长期丢失的目标
        self.targets = [t for t in self.targets 
                       if t.time_since_update < self.max_age]
        
        # 选择主要目标
        self._select_primary_target()
        
        return self.primary_target
    
    def _select_primary_target(self):
        """选择主要跟踪目标"""
        if not self.targets:
            self.primary_target = None
            return
        
        # 只选择当前帧刚更新的目标（time_since_update == 0）
        # 且边界框有效的目标，避免选择已丢失或无效的目标
        def _is_valid_target(t):
            if t.time_since_update != 0:
                return False
            x1, y1, x2, y2 = t.bbox
            # 确保边界框有效（宽度高度都大于10像素）
            if x2 - x1 < 10 or y2 - y1 < 10:
                return False
            # 避免紧贴边缘的异常框
            if x1 < 0 or y1 < 0:
                return False
            return True
        
        valid_targets = [t for t in self.targets if _is_valid_target(t)]
        
        if not valid_targets:
            self.primary_target = None
            return
        
        # 根据策略选择
        if self.selection_strategy == "center":
            # 选择画面中央的目标
            # 使用画面中心作为参考点，选择离中心最近的目标
            # 默认中心点 (320, 240)，可通过外部传入的 frame_center 调整
            center_x = getattr(self, 'frame_center_x', 320)
            center_y = getattr(self, 'frame_center_y', 240)
            
            def distance_to_center(target):
                cx, cy = target.center
                return (cx - center_x) ** 2 + (cy - center_y) ** 2
            
            self.primary_target = min(valid_targets, key=distance_to_center)
            
        elif self.selection_strategy == "largest":
            # 选择最大的目标
            self.primary_target = max(valid_targets, key=lambda t: t.area)
            
        elif self.selection_strategy == "closest":
            # 选择最近的目标（假设框越大越近）
            self.primary_target = max(valid_targets, key=lambda t: t.area)
            
        else:
            # 默认选择置信度最高的
            self.primary_target = max(valid_targets, key=lambda t: t.confidence)
    
    def get_all_targets(self) -> List[Target]:
        """获取所有跟踪目标"""
        return self.targets.copy()
    
    def get_primary_target(self) -> Optional[Target]:
        """获取主要跟踪目标"""
        return self.primary_target
    
    def is_tracking(self) -> bool:
        """是否正在跟踪目标"""
        return (self.primary_target is not None and 
                self.primary_target.status == TargetStatus.TRACKING)
    
    def get_stats(self) -> dict:
        """获取跟踪统计信息"""
        return {
            "total_targets": len(self.targets),
            "tracking": sum(1 for t in self.targets if t.status == TargetStatus.TRACKING),
            "lost": sum(1 for t in self.targets if t.status == TargetStatus.LOST),
            "primary_target_id": self.primary_target.id if self.primary_target else None,
            "next_id": TargetTracker._id_counter
        }


# 测试代码
if __name__ == "__main__":
    from .detector import HumanDetector
    import cv2
    
    # 初始化
    detector = HumanDetector(model_path="models/yolo26n.pt")
    tracker = TargetTracker(max_age=30, selection_strategy="center")
    
    if not detector.initialize():
        logger.error("检测器初始化失败")
        exit(1)
    
    # 测试摄像头
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    logger.info("按 'q' 退出测试，按 'r' 重置跟踪")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 检测
        detections = detector.detect(frame)
        
        # 跟踪
        target = tracker.update(detections)
        
        # 绘制
        output = frame.copy()
        
        # 绘制所有检测框
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 1)
        
        # 绘制所有跟踪目标
        for t in tracker.get_all_targets():
            x1, y1, x2, y2 = t.bbox
            cx, cy = t.center
            
            if t == tracker.get_primary_target():
                # 主要目标 - 红色粗框
                color = (0, 0, 255)
                thickness = 3
                label = f"Target {t.id} (Primary)"
            elif t.status == TargetStatus.TRACKING:
                # 跟踪中 - 蓝色
                color = (255, 0, 0)
                thickness = 2
                label = f"Target {t.id}"
            else:
                # 丢失 - 灰色
                color = (128, 128, 128)
                thickness = 1
                label = f"Target {t.id} (Lost)"
            
            cv2.rectangle(output, (x1, y1), (x2, y2), color, thickness)
            cv2.circle(output, (cx, cy), 4, color, -1)
            cv2.putText(output, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # 显示统计信息
        stats = tracker.get_stats()
        info_text = f"Targets: {stats['total_targets']} | Tracking: {stats['tracking']} | Primary: {stats['primary_target_id']}"
        cv2.putText(output, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow("Target Tracking", output)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            tracker.reset()
            logger.info("跟踪器已重置")
    
    cap.release()
    cv2.destroyAllWindows()
