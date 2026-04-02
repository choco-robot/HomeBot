"""
麻将牌坐标转换器

负责坐标系转换：
1. 图像坐标 (像素) -> 牌桌物理坐标 (mm)
2. 牌桌坐标 -> 机械臂坐标

使用四点透视标定 (Homography) 进行图像到物理平面的映射
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from common.logging import get_logger
from configs.config import get_config

logger = get_logger(__name__)


@dataclass
class CalibrationPoint:
    """标定点"""
    image_x: float  # 图像坐标 (像素)
    image_y: float
    table_x: float  # 牌桌物理坐标 (mm)
    table_y: float


class CoordinateTransformer:
    """
    坐标转换器
    
    使用Homography矩阵将图像坐标映射到物理坐标
    """
    
    def __init__(self):
        """初始化坐标转换器"""
        config = get_config()
        
        # 加载标定矩阵（从配置读取，默认单位矩阵）
        self.homography_matrix = np.array(config.mahjong.homography_matrix).reshape(3, 3)
        self.inv_homography = np.linalg.inv(self.homography_matrix)
        
        # 机械臂底座相对于牌桌的偏移
        self.arm_offset_x = config.mahjong.arm_offset_x
        self.arm_offset_y = config.mahjong.arm_offset_y
        
        # 牌桌尺寸
        self.table_width = config.mahjong.table_width_mm
        self.table_height = config.mahjong.table_height_mm
        
        # 标定状态
        self.is_calibrated = not np.array_equal(self.homography_matrix, np.eye(3))
        
        logger.info("CoordinateTransformer 初始化")
        logger.info(f"  机械臂偏移: ({self.arm_offset_x}, {self.arm_offset_y}) mm")
        logger.info(f"  牌桌尺寸: {self.table_width}x{self.table_height} mm")
        logger.info(f"  标定状态: {'已标定' if self.is_calibrated else '未标定'}")
    
    def set_homography_matrix(self, matrix: np.ndarray):
        """
        设置Homography矩阵
        
        Args:
            matrix: 3x3 Homography矩阵
        """
        if matrix.shape != (3, 3):
            raise ValueError("Homography矩阵必须是3x3")
        
        self.homography_matrix = matrix
        self.inv_homography = np.linalg.inv(matrix)
        self.is_calibrated = True
        
        logger.info("Homography矩阵已更新")
    
    def calibrate_from_points(self, points: List[CalibrationPoint]):
        """
        从标定点计算Homography矩阵
        
        需要至少4个点（推荐4个角点）
        
        Args:
            points: 标定点列表 (至少4个)
        """
        if len(points) < 4:
            raise ValueError("至少需要4个标定点")
        
        # 构建方程组 Ax = b
        A = []
        b = []
        
        for p in points:
            # 图像坐标 (u, v)
            u, v = p.image_x, p.image_y
            # 物理坐标 (x, y)
            x, y = p.table_x, p.table_y
            
            # 每个点提供2个方程
            A.append([u, v, 1, 0, 0, 0, -u*x, -v*x])
            A.append([0, 0, 0, u, v, 1, -u*y, -v*y])
            b.append(x)
            b.append(y)
        
        A = np.array(A)
        b = np.array(b)
        
        # 求解最小二乘解
        h = np.linalg.lstsq(A, b, rcond=None)[0]
        
        # 构建3x3 Homography矩阵
        H = np.array([
            [h[0], h[1], h[2]],
            [h[3], h[4], h[5]],
            [h[6], h[7], 1.0]
        ])
        
        self.set_homography_matrix(H)
        
        # 验证标定精度
        errors = []
        for p in points:
            tx, ty = self.image_to_table(p.image_x, p.image_y)
            error = np.sqrt((tx - p.table_x)**2 + (ty - p.table_y)**2)
            errors.append(error)
        
        avg_error = np.mean(errors)
        max_error = np.max(errors)
        logger.info(f"标定完成: 平均误差={avg_error:.2f}mm, 最大误差={max_error:.2f}mm")
        
        return avg_error
    
    def image_to_table(self, image_x: float, image_y: float) -> Tuple[float, float]:
        """
        图像坐标 -> 牌桌物理坐标
        
        Args:
            image_x: 图像X坐标 (像素)
            image_y: 图像Y坐标 (像素)
        
        Returns:
            (table_x, table_y) 单位 mm
        """
        # 齐次坐标
        p = np.array([image_x, image_y, 1.0])
        
        # 应用Homography变换
        result = self.homography_matrix @ p
        
        # 归一化
        w = result[2]
        if abs(w) > 1e-10:
            table_x = result[0] / w
            table_y = result[1] / w
        else:
            table_x, table_y = 0.0, 0.0
        
        return table_x, table_y
    
    def table_to_image(self, table_x: float, table_y: float) -> Tuple[float, float]:
        """
        牌桌物理坐标 -> 图像坐标
        
        Args:
            table_x: 牌桌X坐标 (mm)
            table_y: 牌桌Y坐标 (mm)
        
        Returns:
            (image_x, image_y) 单位像素
        """
        # 齐次坐标
        p = np.array([table_x, table_y, 1.0])
        
        # 应用逆Homography变换
        result = self.inv_homography @ p
        
        # 归一化
        w = result[2]
        if abs(w) > 1e-10:
            image_x = result[0] / w
            image_y = result[1] / w
        else:
            image_x, image_y = 0.0, 0.0
        
        return image_x, image_y
    
    def table_to_arm(self, table_x: float, table_y: float, 
                    table_z: float = 30.0) -> Tuple[float, float, float]:
        """
        牌桌坐标 -> 机械臂坐标
        
        考虑机械臂底座相对于牌桌的偏移
        
        Args:
            table_x: 牌桌X坐标 (mm)
            table_y: 牌桌Y坐标 (mm)
            table_z: 牌桌Z高度 (mm)
        
        Returns:
            (arm_x, arm_y, arm_z) 机械臂坐标系下的位置
        """
        arm_x = table_x - self.arm_offset_x
        arm_y = table_y - self.arm_offset_y
        arm_z = table_z
        
        return arm_x, arm_y, arm_z
    
    def arm_to_table(self, arm_x: float, arm_y: float, 
                    arm_z: float) -> Tuple[float, float, float]:
        """
        机械臂坐标 -> 牌桌坐标
        
        Args:
            arm_x: 机械臂X坐标 (mm)
            arm_y: 机械臂Y坐标 (mm)
            arm_z: 机械臂Z坐标 (mm)
        
        Returns:
            (table_x, table_y, table_z)
        """
        table_x = arm_x + self.arm_offset_x
        table_y = arm_y + self.arm_offset_y
        table_z = arm_z
        
        return table_x, table_y, table_z
    
    def image_to_arm(self, image_x: float, image_y: float, 
                    table_z: float = 30.0) -> Optional[Tuple[float, float, float]]:
        """
        图像坐标 -> 机械臂坐标（完整转换链路）
        
        Args:
            image_x: 图像X坐标 (像素)
            image_y: 图像Y坐标 (像素)
            table_z: 目标高度 (mm)
        
        Returns:
            (arm_x, arm_y, arm_z) 或 None（如果未标定）
        """
        if not self.is_calibrated:
            logger.warning("坐标转换器未标定，无法转换")
            return None
        
        # 图像 -> 牌桌
        table_x, table_y = self.image_to_table(image_x, image_y)
        
        # 牌桌 -> 机械臂
        arm_x, arm_y, arm_z = self.table_to_arm(table_x, table_y, table_z)
        
        return arm_x, arm_y, arm_z
    
    def get_calibration_guide(self) -> List[CalibrationPoint]:
        """
        获取推荐标定点（牌桌四个角）
        
        Returns:
            4个标定点的列表
        """
        # 牌桌四个角点（假设机械臂在牌桌一侧）
        # 坐标系：原点在机械臂基座，x向前，y向左
        
        return [
            CalibrationPoint(
                image_x=0, image_y=0,  # 图像左上角
                table_x=0, table_y=0    # 对应牌桌远左角
            ),
            CalibrationPoint(
                image_x=1920, image_y=0,  # 图像右上角
                table_x=0, table_y=self.table_height  # 对应牌桌远右角
            ),
            CalibrationPoint(
                image_x=1920, image_y=1080,  # 图像右下角
                table_x=self.table_width, table_y=self.table_height  # 对应牌桌近右角
            ),
            CalibrationPoint(
                image_x=0, image_y=1080,  # 图像左下角
                table_x=self.table_width, table_y=0  # 对应牌桌近左角
            ),
        ]


class CalibrationTool:
    """
    标定工具 - 交互式采集标定点
    """
    
    def __init__(self, transformer: CoordinateTransformer):
        self.transformer = transformer
        self.points: List[CalibrationPoint] = []
    
    def add_point(self, image_x: float, image_y: float, 
                  table_x: float, table_y: float):
        """添加标定点"""
        point = CalibrationPoint(image_x, image_y, table_x, table_y)
        self.points.append(point)
        logger.info(f"添加标定点 {len(self.points)}: "
                   f"图像({image_x:.0f}, {image_y:.0f}) -> "
                   f"牌桌({table_x:.0f}, {table_y:.0f})")
    
    def calibrate(self) -> float:
        """执行标定"""
        if len(self.points) < 4:
            raise ValueError(f"需要至少4个标定点，当前只有{len(self.points)}个")
        
        return self.transformer.calibrate_from_points(self.points)
    
    def save_calibration(self, filepath: str):
        """保存标定结果到文件"""
        import json
        
        data = {
            'homography_matrix': self.transformer.homography_matrix.flatten().tolist(),
            'points': [
                {
                    'image_x': p.image_x,
                    'image_y': p.image_y,
                    'table_x': p.table_x,
                    'table_y': p.table_y
                }
                for p in self.points
            ]
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        logger.info(f"标定结果已保存到: {filepath}")
    
    def load_calibration(self, filepath: str):
        """从文件加载标定结果"""
        import json
        
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        matrix = np.array(data['homography_matrix']).reshape(3, 3)
        self.transformer.set_homography_matrix(matrix)
        
        self.points = [
            CalibrationPoint(p['image_x'], p['image_y'], p['table_x'], p['table_y'])
            for p in data['points']
        ]
        
        logger.info(f"标定结果已从 {filepath} 加载")


if __name__ == "__main__":
    # 测试坐标转换器
    print("=" * 60)
    print("坐标转换器测试")
    print("=" * 60)
    
    transformer = CoordinateTransformer()
    
    # 测试1: 模拟标定
    print("\n测试1: 四点标定")
    
    # 模拟标定点（实际使用时需要真实测量）
    points = [
        CalibrationPoint(100, 100, 0, 0),
        CalibrationPoint(900, 100, 0, 400),
        CalibrationPoint(900, 500, 600, 400),
        CalibrationPoint(100, 500, 600, 0),
    ]
    
    error = transformer.calibrate_from_points(points)
    print(f"\n标定误差: {error:.2f}mm")
    
    # 测试2: 坐标转换
    print("\n测试2: 坐标转换")
    test_image_points = [
        (200, 200),
        (500, 300),
        (800, 400),
    ]
    
    for img_x, img_y in test_image_points:
        table_x, table_y = transformer.image_to_table(img_x, img_y)
        arm_x, arm_y, arm_z = transformer.table_to_arm(table_x, table_y, 30.0)
        
        print(f"\n图像({img_x}, {img_y})")
        print(f"  -> 牌桌({table_x:.1f}, {table_y:.1f})")
        print(f"  -> 机械臂({arm_x:.1f}, {arm_y:.1f}, {arm_z:.1f})")
        
        # 反向验证
        img_x_back, img_y_back = transformer.table_to_image(table_x, table_y)
        print(f"  <- 验证: 图像({img_x_back:.1f}, {img_y_back:.1f})")
