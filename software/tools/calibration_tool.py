"""
麻将机械臂标定工具

交互式四点标定工具，用于建立图像坐标到机械臂坐标的映射。

使用方法:
    cd software
    python tools/calibration_tool.py

标定流程:
    1. 放置一张牌在牌桌左上角
    2. 在图像中点击该位置，输入机械臂实际坐标
    3. 重复4个点（左上、右上、右下、左下）
    4. 保存标定结果
"""

import argparse
import sys
import json
import time
from pathlib import Path
from typing import List, Tuple, Optional

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import cv2
import numpy as np

from applications.mahjong_bot.coordinate_transformer import (
    CoordinateTransformer, CalibrationPoint, CalibrationTool
)
from applications.mahjong_bot.arm_client import ArmServiceClient
from common.logging import get_logger
from configs.config import get_config, set_config

logger = get_logger(__name__)


class InteractiveCalibration:
    """交互式标定工具"""
    
    def __init__(self, camera_device: int = 0, arm_addr: str = "tcp://localhost:5557"):
        """
        初始化标定工具
        
        Args:
            camera_device: 摄像头设备ID
            arm_addr: 机械臂服务地址
        """
        self.camera_device = camera_device
        self.transformer = CoordinateTransformer()
        self.calibration = CalibrationTool(self.transformer)
        self.arm_client = ArmServiceClient(arm_addr)
        
        # 摄像头
        self.cap = None
        self.current_frame = None
        
        # 标定点采集状态
        self.pending_image_point: Optional[Tuple[float, float]] = None
        self.step = 0
        self.steps_desc = [
            "左上", "右上", "右下", "左下"
        ]
        
    def init_camera(self) -> bool:
        """初始化摄像头"""
        logger.info(f"初始化摄像头设备 {self.camera_device}...")
        self.cap = cv2.VideoCapture(self.camera_device)
        
        if not self.cap.isOpened():
            logger.error("无法打开摄像头")
            return False
        
        # 设置分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        # 读取一帧测试
        ret, frame = self.cap.read()
        if not ret:
            logger.error("无法读取画面")
            return False
        
        logger.info(f"摄像头就绪: {frame.shape[1]}x{frame.shape[0]}")
        return True
    
    def connect_arm(self) -> bool:
        """连接机械臂"""
        logger.info("连接机械臂服务...")
        if self.arm_client.connect():
            logger.info("机械臂已连接")
            return True
        else:
            logger.warning("无法连接机械臂服务，将使用手动输入坐标模式")
            return False
    
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标点击回调"""
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.step < 4 and self.pending_image_point is None:
                self.pending_image_point = (x, y)
                logger.info(f"点击位置: ({x}, {y})")
    
    def get_arm_position_manual(self) -> Optional[Tuple[float, float, float]]:
        """手动输入机械臂坐标"""
        print(f"\n请输入第 {self.step + 1} 个点 ({self.steps_desc[self.step]}) 的机械臂坐标:")
        try:
            x = float(input("  X (mm): "))
            y = float(input("  Y (mm): "))
            z = float(input("  Z (mm, 默认30): ") or "30")
            return (x, y, z)
        except ValueError:
            print("输入无效")
            return None
    
    def get_arm_position_auto(self) -> Optional[Tuple[float, float, float]]:
        """自动获取机械臂当前位置（通过示教）"""
        print(f"\n第 {self.step + 1} 个点 ({self.steps_desc[self.step]}):")
        print("请将机械臂末端移动到该位置，然后按回车")
        input("按回车确认...")
        
        state = self.arm_client.get_state()
        if state and state.joint_angles:
            # 这里需要从关节角度计算末端位置
            # 简化版本：要求用户手动输入
            print("请手动输入当前坐标:")
            return self.get_arm_position_manual()
        else:
            print("无法获取机械臂状态")
            return None
    
    def run(self):
        """运行标定流程"""
        print("=" * 60)
        print("麻将机械臂标定工具")
        print("=" * 60)
        print("\n标定说明:")
        print("1. 在牌桌上放置一张牌作为标记")
        print("2. 在图像窗口中点击该牌的位置")
        print("3. 输入该位置对应的机械臂坐标 (x, y, z)")
        print("4. 重复4个点：左上、右上、右下、左下")
        print("5. 保存标定结果")
        
        # 初始化
        if not self.init_camera():
            return False
        
        use_arm = self.connect_arm()
        
        # 创建窗口
        cv2.namedWindow("Calibration")
        cv2.setMouseCallback("Calibration", self.mouse_callback)
        
        print("\n请在图像窗口中点击4个标定点...")
        print("按 'q' 退出，按 'r' 重置，按 's' 保存")
        
        while self.step < 4:
            # 读取帧
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            self.current_frame = frame.copy()
            
            # 显示已采集的点
            for i, point in enumerate(self.calibration.points):
                ix, iy = int(point.image_x), int(point.image_y)
                cv2.circle(frame, (ix, iy), 8, (0, 255, 0), -1)
                cv2.putText(frame, str(i+1), (ix-5, iy+5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
            # 显示当前待采集点
            if self.pending_image_point:
                ix, iy = int(self.pending_image_point[0]), int(self.pending_image_point[1])
                cv2.circle(frame, (ix, iy), 8, (0, 165, 255), -1)
                cv2.putText(frame, f"P{self.step+1}", (ix+10, iy-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            
            # 显示提示信息
            status_text = f"Step {self.step + 1}/4: {self.steps_desc[self.step]}"
            cv2.putText(frame, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            collected = f"已采集: {len(self.calibration.points)}/4"
            cv2.putText(frame, collected, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            
            cv2.imshow("Calibration", frame)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n用户取消标定")
                break
            elif key == ord('r'):
                print("\n重置标定")
                self.calibration.points = []
                self.step = 0
                self.pending_image_point = None
            elif key == ord('s') and len(self.calibration.points) >= 4:
                break
            
            # 处理待输入的点
            if self.pending_image_point and self.step < 4:
                # 获取机械臂坐标
                if use_arm:
                    arm_pos = self.get_arm_position_auto()
                else:
                    arm_pos = self.get_arm_position_manual()
                
                if arm_pos:
                    # 添加到标定点
                    img_x, img_y = self.pending_image_point
                    # 将机械臂坐标转换为牌桌坐标
                    table_pos = self.transformer.arm_to_table(*arm_pos)
                    
                    point = CalibrationPoint(
                        image_x=img_x,
                        image_y=img_y,
                        table_x=table_pos[0],
                        table_y=table_pos[1]
                    )
                    self.calibration.add_point(point)
                    self.step += 1
                
                self.pending_image_point = None
        
        # 执行标定
        success = False
        if len(self.calibration.points) >= 4:
            print("\n执行标定...")
            try:
                error = self.calibration.calibrate()
                print(f"标定完成！平均误差: {error:.2f}mm")
                success = True
                
                # 保存标定结果
                self._save_calibration()
                
            except Exception as e:
                print(f"标定失败: {e}")
        
        # 清理
        self.cap.release()
        cv2.destroyAllWindows()
        
        if self.arm_client.is_connected():
            self.arm_client.disconnect()
        
        return success
    
    def _save_calibration(self):
        """保存标定结果"""
        # 保存到配置文件
        config = get_config()
        
        # 更新 Homography 矩阵
        config.mahjong.homography_matrix = self.transformer.homography_matrix.flatten().tolist()
        
        # 保存到文件
        output_file = Path("calibration_result.json")
        self.calibration.save_calibration(output_file)
        
        print(f"\n标定结果已保存到:")
        print(f"  - {output_file}")
        print(f"  - 配置已更新 (mahjong.homography_matrix)")
    
    def test_calibration(self):
        """测试标定结果"""
        print("\n" + "=" * 60)
        print("测试标定结果")
        print("=" * 60)
        
        # 重新初始化摄像头
        if not self.init_camera():
            return
        
        print("在图像中移动鼠标，查看对应的机械臂坐标")
        print("按 'q' 退出")
        
        cv2.namedWindow("Test Calibration")
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # 获取鼠标位置（简化：使用窗口中心）
            # 实际应该使用鼠标回调
            h, w = frame.shape[:2]
            test_x, test_y = w // 2, h // 2
            
            # 转换坐标
            arm_pos = self.transformer.image_to_arm(test_x, test_y, 30.0)
            
            if arm_pos:
                arm_x, arm_y, arm_z = arm_pos
                text = f"Image: ({test_x}, {test_y}) -> Arm: ({arm_x:.1f}, {arm_y:.1f}, {arm_z:.1f})"
                cv2.putText(frame, text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow("Test Calibration", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="麻将机械臂标定工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 运行标定
  python tools/calibration_tool.py
  
  # 指定摄像头和机械臂地址
  python tools/calibration_tool.py --camera 1 --arm tcp://192.168.1.100:5557
  
  # 加载已有标定并测试
  python tools/calibration_tool.py --load calibration_result.json --test
        """
    )
    
    parser.add_argument("--camera", "-c", type=int, default=0,
                        help="摄像头设备ID (默认: 0)")
    parser.add_argument("--arm", "-a", type=str, default="tcp://localhost:5557",
                        help="机械臂服务地址 (默认: tcp://localhost:5557)")
    parser.add_argument("--load", "-l", type=str, default=None,
                        help="加载已有标定文件")
    parser.add_argument("--test", "-t", action="store_true",
                        help="测试模式（不执行标定，只测试）")
    
    args = parser.parse_args()
    
    tool = InteractiveCalibration(args.camera, args.arm)
    
    # 加载已有标定
    if args.load:
        print(f"加载标定文件: {args.load}")
        tool.calibration.load_calibration(args.load)
    
    # 测试模式
    if args.test:
        tool.test_calibration()
    else:
        # 运行标定
        success = tool.run()
        if success:
            print("\n标定成功！")
            # 询问是否测试
            test = input("是否测试标定结果? (y/n): ")
            if test.lower() == 'y':
                tool.test_calibration()
        else:
            print("\n标定失败或取消")


if __name__ == "__main__":
    main()
