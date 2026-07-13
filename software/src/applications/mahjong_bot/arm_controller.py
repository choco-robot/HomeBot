"""
麻将机械臂控制器 - 纯运动控制层

职责：
- CoordinateTransformer: 坐标转换
- MotionPlanner: 运动规划
- ArmClient: 机械臂通信

不包含手牌状态管理，只提供基于坐标的运动控制接口
"""

from typing import Optional, Dict, Callable, Tuple
import time

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from common.logging import get_logger
from configs.config import get_config

from .coordinate_transformer import CoordinateTransformer
from .motion_planner import MotionPlanner, Pose3D, MotionStep
from .arm_client import ArmServiceClient

logger = get_logger(__name__)


class MahjongArmController:
    """
    麻将机械臂控制器
    
    纯运动控制层，只负责机械臂运动。
    手牌状态管理由上层（GameStateManager）负责。
    """
    
    def __init__(self, arm_service_addr: str = "tcp://localhost:5557", arm_id: int = 1):
        """
        初始化控制器
        
        Args:
            arm_service_addr: 机械臂服务地址
            arm_id: 机械臂ID，1=第一套(arm)，2=第二套(arm2)
        """
        self.arm_id = arm_id
        arm_name = "arm2" if arm_id == 2 else "arm"
        
        # 初始化运动控制相关组件
        self.coordinate_transformer = CoordinateTransformer(arm_id)
        self.motion_planner = MotionPlanner()
        self.arm_client = ArmServiceClient(arm_service_addr)
        
        # 状态
        self.is_calibrated = self.coordinate_transformer.is_calibrated
        self.is_arm_connected = False
        
        # 默认牌高度（从配置读取）
        config = get_config()
        self.default_tile_height = getattr(config.mahjong, 'tile_height', 30.0)
        
        logger.info(f"MahjongArmController 初始化完成 [{arm_name}]")
        logger.info(f"  服务地址: {arm_service_addr}")
        logger.info(f"  标定状态: {'已标定' if self.is_calibrated else '未标定'}")
    
    def connect_arm(self) -> bool:
        """连接机械臂服务"""
        if self.arm_client.connect():
            self.is_arm_connected = True
            logger.info("机械臂已连接")
            return True
        else:
            logger.error("机械臂连接失败")
            return False
    
    def disconnect_arm(self):
        """断开机械臂连接"""
        self.arm_client.disconnect()
        self.is_arm_connected = False
        logger.info("机械臂已断开")
    
    def image_to_arm_coords(self, image_x: float, image_y: float, 
                           table_z: float = None) -> Optional[Tuple[float, float, float]]:
        """
        图像坐标转换为机械臂坐标
        
        Args:
            image_x: 图像X坐标（像素）
            image_y: 图像Y坐标（像素）
            table_z: 牌桌高度（mm），默认使用配置值
        
        Returns:
            (arm_x, arm_y, arm_z) 机械臂坐标，转换失败返回None
        """
        if table_z is None:
            table_z = self.default_tile_height
        
        return self.coordinate_transformer.image_to_arm(image_x, image_y, table_z)
    
    def pick_and_place(self,
                      image_x: float,
                      image_y: float,
                      on_step: Optional[Callable[[str, int, int], None]] = None,
                      table_z: float = None) -> bool:
        """
        抓取并出牌（基于图像坐标）
        
        Args:
            image_x: 目标牌图像X坐标（像素）
            image_y: 目标牌图像Y坐标（像素）
            on_step: 步骤回调函数 (step_name, current_step, total_steps)
            table_z: 牌桌高度（mm），默认使用配置值
        
        Returns:
            是否成功
        """
        # 坐标转换
        arm_pos = self.image_to_arm_coords(image_x, image_y, table_z)
        if arm_pos is None:
            logger.error("坐标转换失败，请检查标定")
            return False
        
        arm_x, arm_y, arm_z = arm_pos
        logger.info(f"出牌目标: 图像({image_x:.0f}, {image_y:.0f}) -> 机械臂({arm_x:.1f}, {arm_y:.1f}, {arm_z:.1f})")
        
        # 检查可达性
        if not self.motion_planner.is_position_reachable(arm_x, arm_y, arm_z):
            logger.error(f"位置 ({arm_x}, {arm_y}, {arm_z}) 不可达")
            return False
        
        # 规划运动序列
        sequence = self.motion_planner.plan_pick_and_place(
            tile_x=arm_x,
            tile_y=arm_y,
            tile_height=arm_z
        )
        
        if not sequence:
            logger.error("运动规划失败")
            return False
        
        # 执行运动序列
        logger.info(f"执行出牌动作序列: {len(sequence)} 步")
        return self._execute_sequence(sequence, on_step)
    
    def push_tile(self,
                 image_x: float,
                 image_y: float,
                 on_step: Optional[Callable[[str, int, int], None]] = None,
                 push_distance: float = 50.0,
                 table_z: float = None) -> bool:
        """
        推倒牌（基于图像坐标）
        
        Args:
            image_x: 目标牌图像X坐标（像素）
            image_y: 目标牌图像Y坐标（像素）
            on_step: 步骤回调函数 (step_name, current_step, total_steps)
            push_distance: 前推距离 (mm)，默认50mm
            table_z: 牌桌高度（mm），默认使用配置值
        
        Returns:
            是否成功
        """
        # 坐标转换
        arm_pos = self.image_to_arm_coords(image_x, image_y, table_z)
        if arm_pos is None:
            logger.error("坐标转换失败，请检查标定")
            return False
        
        arm_x, arm_y, arm_z = arm_pos
        logger.info(f"推倒目标: 图像({image_x:.0f}, {image_y:.0f}) -> 机械臂({arm_x:.1f}, {arm_y:.1f}, {arm_z:.1f})")
        
        # 检查可达性
        if not self.motion_planner.is_position_reachable(arm_x, arm_y, arm_z):
            logger.error(f"位置 ({arm_x}, {arm_y}, {arm_z}) 不可达")
            return False
        
        # 规划推倒运动序列
        sequence = self.motion_planner.plan_push(
            tile_x=arm_x,
            tile_y=arm_y,
            tile_height=arm_z,
            push_distance=push_distance
        )
        
        if not sequence:
            logger.error("运动规划失败")
            return False
        
        # 执行运动序列
        logger.info(f"执行推倒动作序列: {len(sequence)} 步")
        return self._execute_sequence(sequence, on_step)
    
    def _execute_sequence(self,
                         sequence: list,
                         on_step: Optional[Callable[[str, int, int], None]] = None) -> bool:
        """
        执行运动序列（内部方法）
        
        Args:
            sequence: 运动步骤列表
            on_step: 步骤回调函数
        
        Returns:
            是否成功
        """
        def step_callback(step, idx):
            if on_step:
                on_step(step.name, idx + 1, len(sequence))
        
        success = self.motion_planner.execute_sequence(
            self.arm_client,
            on_step_start=step_callback
        )
        
        if success:
            logger.info("动作序列执行完成")
        else:
            logger.error("动作序列执行失败")
        
        return success
    
    def move_to_rest(self) -> bool:
        """移动到休息位"""
        config = get_config()
        rest_pos = config.arm.rest_position
        
        logger.info("移动到休息位")
        return self.arm_client.move_joints(rest_pos, speed=500)
    
    def calibrate_from_points(self, calibration_points: list) -> float:
        """
        使用标定点进行标定
        
        Args:
            calibration_points: 标定点列表
        
        Returns:
            标定误差
        """
        error = self.coordinate_transformer.calibrate_from_points(calibration_points)
        self.is_calibrated = True
        logger.info(f"标定完成，误差: {error:.2f}mm")
        return error
    
    def get_workspace_info(self) -> Dict:
        """获取工作空间信息"""
        return self.motion_planner.get_workspace_info()
    
    def is_position_reachable(self, image_x: float, image_y: float, table_z: float = None) -> bool:
        """
        检查图像坐标位置是否可达
        
        Args:
            image_x: 图像X坐标
            image_y: 图像Y坐标
            table_z: 牌桌高度
        
        Returns:
            是否可达
        """
        arm_pos = self.image_to_arm_coords(image_x, image_y, table_z)
        if arm_pos is None:
            return False
        return self.motion_planner.is_position_reachable(*arm_pos)


# 简单的命令行测试接口
if __name__ == "__main__":
    print("=" * 60)
    print("麻将机械臂控制器测试 - 点到点正方形路径")
    print("=" * 60)
    
    controller = MahjongArmController()
    
    # 显示工作空间
    print("\n1. 工作空间信息")
    workspace = controller.get_workspace_info()
    print(f"  半径范围: {workspace['r_min']:.0f} ~ {workspace['r_max']:.0f} mm")
    print(f"  高度范围: {workspace['z_min']:.0f} ~ {workspace['z_max']:.0f} mm")
    
    # 连接机械臂
    print("\n2. 连接机械臂服务")
    if not controller.connect_arm():
        print("连接失败，退出测试")
        exit(1)
    
    try:
        # 正方形路径参数
        side_mm = 100.0  # 正方形边长 10cm
        center_x = 200.0
        center_y = 0.0
        z = -100.0
        
        # 计算正方形四个顶点（从中心点出发）
        half = side_mm / 2.0
        square_points = [
            (center_x - half, center_y - half, z),  # 左下
            (center_x + half, center_y - half, z),  # 右下
            (center_x + half, center_y + half, z),  # 右上
            (center_x - half, center_y + half, z),  # 左上
            (center_x - half, center_y - half, z),  # 回到起点
        ]
        
        print(f"\n3. 规划正方形路径")
        print(f"  边长: {side_mm:.0f} mm")
        print(f"  中心: ({center_x:.0f}, {center_y:.0f})")
        print(f"  高度: z={z:.0f} mm")
        print(f"  路径点: {len(square_points)} 个")
        
        sequence = []
        for i, (x, y, z_pos) in enumerate(square_points):
            pose = Pose3D(x=x, y=y, z=z_pos, orientation=90.0)
            joints = controller.motion_planner._pose_to_joints(pose)
            if joints is None:
                print(f"  路径点 {i+1} ({x:.0f}, {y:.0f}, {z_pos:.0f}) 不可达，跳过")
                continue
            
            sequence.append(MotionStep(
                name=f"move_to_point_{i+1}",
                target_pose=pose,
                joint_angles=joints,
                gripper_open=True,
                duration=0.0,
                description=f"移动到路径点 {i+1}: ({x:.0f}, {y:.0f}, {z_pos:.0f})"
            ))
            print(f"  路径点 {i+1}: ({x:.0f}, {y:.0f}, {z_pos:.0f}) -> 已规划")
        
        if not sequence:
            print("没有可达的路径点，退出测试")
            exit(1)
        
        # 执行运动序列
        print(f"\n4. 执行正方形路径 ({len(sequence)} 步)")
        controller.motion_planner.motion_sequence = sequence
        success = controller.motion_planner.execute_sequence(controller.arm_client)
        
        if success:
            print("\n  正方形路径执行完成")
        else:
            print("\n  正方形路径执行失败")
        
        # 返回休息位
        print("\n5. 返回休息位")
        rest_success = controller.move_to_rest()
        print(f"  休息位指令发送: {'成功' if rest_success else '失败'}")
        if rest_success:
            # 等待机械臂运动到休息位
            print("  等待 3 秒让机械臂到达休息位...")
            time.sleep(3.0)
            
            # 读取当前状态确认
            state = controller.arm_client.get_state()
            if state and state.joint_angles:
                print(f"  当前关节角度: {state.joint_angles}")
            
    finally:
        controller.disconnect_arm()
    
    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)
