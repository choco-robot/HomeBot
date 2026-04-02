"""
麻将机械臂运动规划器

负责将出牌动作分解为机械臂可执行的运动序列：
1. 移动到目标牌上方（悬停）
2. 下降并抓取
3. 提起并移动到出牌槽
4. 释放/出牌
5. 返回休息位
"""

import time
import math
from typing import List, Dict, Tuple, Optional, Callable
from dataclasses import dataclass
from enum import Enum, auto

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from common.logging import get_logger
from hal.arm.Kinematics import Arm3DKinematics
from configs.config import get_config

logger = get_logger(__name__)


class MotionState(Enum):
    """运动状态"""
    IDLE = auto()
    MOVING_TO_HOVER = auto()
    DESCENDING = auto()
    GRASPING = auto()
    LIFTING = auto()
    MOVING_TO_SLOT = auto()
    RELEASING = auto()
    RETURNING = auto()
    ERROR = auto()


@dataclass
class Pose3D:
    """3D位姿"""
    x: float  # mm, 向前
    y: float  # mm, 向左
    z: float  # mm, 向上
    orientation: float = 0.0  # 末端俯仰角，0=水平


@dataclass
class MotionStep:
    """运动步骤"""
    name: str
    target_pose: Optional[Pose3D]
    joint_angles: Optional[Dict[str, float]]
    gripper_open: bool
    duration: float  # 预计耗时(秒)
    description: str


class MotionPlanner:
    """
    机械臂运动规划器
    
    基于3DOF逆运动学，规划出牌动作序列
    """
    
    # 预设高度
    HOVER_HEIGHT = 80.0      # 悬停高度 (mm)
    GRASP_HEIGHT = 30.0      # 抓取高度 (mm)
    LIFT_HEIGHT = 100.0      # 提起高度 (mm)
    SLOT_HEIGHT = 80.0       # 出牌槽高度 (mm)
    
    # 出牌槽位置 (机械臂坐标系)
    DISCARD_SLOT_X = 200.0   # 出牌槽X位置
    DISCARD_SLOT_Y = 0.0     # 出牌槽Y位置（正前方）
    
    def __init__(self):
        """初始化运动规划器"""
        config = get_config()
        arm_cfg = config.arm
        
        # 初始化运动学
        self.kinematics = Arm3DKinematics(
            L1=arm_cfg.upper_arm_length,
            L2=arm_cfg.forearm_length
        )
        
        # 关节限制（从配置读取）
        self.joint_limits = arm_cfg.joint_limits
        
        # 休息位姿
        rest = arm_cfg.rest_position
        self.rest_pose = self._joints_to_pose(
            rest['base'], rest['shoulder'], rest['elbow']
        )
        
        # 当前状态
        self.state = MotionState.IDLE
        self.current_step = 0
        self.motion_sequence: List[MotionStep] = []
        
        logger.info("MotionPlanner 初始化完成")
        logger.info(f"  大臂长度 L1: {self.kinematics.L1}mm")
        logger.info(f"  小臂长度 L2: {self.kinematics.L2}mm")
        logger.info(f"  工作空间 r: {self.kinematics.get_workspace()['r_min']:.0f}~{self.kinematics.get_workspace()['r_max']:.0f}mm")
    
    def _joints_to_pose(self, base: float, shoulder: float, elbow: float) -> Pose3D:
        """关节角度转换为位姿"""
        x, y, z = self.kinematics.forward_kinematics(base, shoulder, elbow)
        return Pose3D(x=x, y=y, z=z)
    
    def _pose_to_joints(self, pose: Pose3D, 
                       elbow_up: bool = True) -> Optional[Dict[str, float]]:
        """位姿转换为关节角度"""
        result = self.kinematics.solve_for_position(
            x=pose.x,
            y=pose.y,
            z=pose.z,
            target_orientation=pose.orientation,
            target_yaw=0.0,  # 保持夹爪朝前
            elbow_up=elbow_up
        )
        return result
    
    def _validate_joints(self, joints: Dict[str, float]) -> bool:
        """验证关节角度是否在限制范围内"""
        for joint_name, angle in joints.items():
            if joint_name in self.joint_limits:
                min_val, max_val = self.joint_limits[joint_name]
                if not (min_val <= angle <= max_val):
                    logger.warning(f"关节 {joint_name} 角度 {angle:.1f}° 超出范围 [{min_val}, {max_val}]")
                    return False
        return True
    
    def plan_pick_and_place(self, 
                           tile_x: float, 
                           tile_y: float,
                           tile_height: float = 30.0) -> List[MotionStep]:
        """
        规划抓取并出牌的动作序列
        
        Args:
            tile_x: 目标牌X坐标 (mm)
            tile_y: 目标牌Y坐标 (mm)
            tile_height: 牌的高度 (mm)
        
        Returns:
            运动步骤列表
        """
        sequence = []
        
        # 步骤1: 移动到牌上方（悬停）
        hover_pose = Pose3D(
            x=tile_x,
            y=tile_y,
            z=self.HOVER_HEIGHT,
            orientation=0.0
        )
        hover_joints = self._pose_to_joints(hover_pose)
        if hover_joints:
            sequence.append(MotionStep(
                name="move_to_hover",
                target_pose=hover_pose,
                joint_angles=hover_joints,
                gripper_open=True,
                duration=2.0,
                description=f"移动到牌上方 ({tile_x:.0f}, {tile_y:.0f})"
            ))
        
        # 步骤2: 下降并抓取
        grasp_pose = Pose3D(
            x=tile_x,
            y=tile_y,
            z=tile_height + 10.0,  # 稍微高于牌面
            orientation=0.0
        )
        grasp_joints = self._pose_to_joints(grasp_pose)
        if grasp_joints:
            sequence.append(MotionStep(
                name="descend_and_grasp",
                target_pose=grasp_pose,
                joint_angles=grasp_joints,
                gripper_open=True,
                duration=1.5,
                description="下降准备抓取"
            ))
            
            # 步骤3: 夹紧
            sequence.append(MotionStep(
                name="close_gripper",
                target_pose=None,
                joint_angles={**grasp_joints, 'gripper': 0.0},  # 夹紧
                gripper_open=False,
                duration=0.5,
                description="夹紧牌"
            ))
        
        # 步骤4: 提起
        lift_pose = Pose3D(
            x=tile_x,
            y=tile_y,
            z=self.LIFT_HEIGHT,
            orientation=0.0
        )
        lift_joints = self._pose_to_joints(lift_pose)
        if lift_joints:
            sequence.append(MotionStep(
                name="lift",
                target_pose=lift_pose,
                joint_angles={**lift_joints, 'gripper': 0.0},  # 保持夹紧
                gripper_open=False,
                duration=1.5,
                description="提起牌"
            ))
        
        # 步骤5: 移动到出牌槽上方
        slot_hover_pose = Pose3D(
            x=self.DISCARD_SLOT_X,
            y=self.DISCARD_SLOT_Y,
            z=self.SLOT_HEIGHT,
            orientation=0.0
        )
        slot_joints = self._pose_to_joints(slot_hover_pose)
        if slot_joints:
            sequence.append(MotionStep(
                name="move_to_slot",
                target_pose=slot_hover_pose,
                joint_angles={**slot_joints, 'gripper': 0.0},
                gripper_open=False,
                duration=2.5,
                description="移动到出牌槽"
            ))
            
            # 步骤6: 下降出牌
            slot_release_pose = Pose3D(
                x=self.DISCARD_SLOT_X,
                y=self.DISCARD_SLOT_Y,
                z=tile_height + 20.0,
                orientation=0.0
            )
            release_joints = self._pose_to_joints(slot_release_pose)
            if release_joints:
                sequence.append(MotionStep(
                    name="lower_to_release",
                    target_pose=slot_release_pose,
                    joint_angles={**release_joints, 'gripper': 0.0},
                    gripper_open=False,
                    duration=1.5,
                    description="下降准备出牌"
                ))
                
                # 步骤7: 释放
                sequence.append(MotionStep(
                    name="open_gripper",
                    target_pose=None,
                    joint_angles={**release_joints, 'gripper': 45.0},  # 松开
                    gripper_open=True,
                    duration=0.5,
                    description="松开牌"
                ))
        
        # 步骤8: 提起并返回休息位
        rest_pose = self.rest_pose
        rest_joints = self._pose_to_joints(rest_pose)
        if rest_joints:
            sequence.append(MotionStep(
                name="return_to_rest",
                target_pose=rest_pose,
                joint_angles=rest_joints,
                gripper_open=True,
                duration=2.0,
                description="返回休息位"
            ))
        
        self.motion_sequence = sequence
        logger.info(f"规划完成: {len(sequence)} 个步骤")
        for i, step in enumerate(sequence):
            logger.info(f"  {i+1}. {step.name}: {step.description}")
        
        return sequence
    
    def execute_sequence(self, 
                        arm_client,
                        on_step_start: Optional[Callable[[MotionStep, int], None]] = None,
                        on_step_complete: Optional[Callable[[MotionStep, int], None]] = None) -> bool:
        """
        执行规划好的动作序列
        
        Args:
            arm_client: 机械臂客户端
            on_step_start: 步骤开始回调 (step, index)
            on_step_complete: 步骤完成回调 (step, index)
        
        Returns:
            是否全部执行成功
        """
        if not self.motion_sequence:
            logger.error("没有规划好的动作序列")
            return False
        
        for i, step in enumerate(self.motion_sequence):
            logger.info(f"执行步骤 {i+1}/{len(self.motion_sequence)}: {step.name}")
            
            if on_step_start:
                on_step_start(step, i)
            
            try:
                if step.joint_angles:
                    # 发送关节角度指令
                    success = arm_client.move_joints(step.joint_angles, speed=800)
                    if not success:
                        logger.error(f"步骤 {step.name} 执行失败")
                        return False
                    
                    # 等待运动完成
                    time.sleep(step.duration)
                
                if on_step_complete:
                    on_step_complete(step, i)
                    
            except Exception as e:
                logger.error(f"执行步骤 {step.name} 时出错: {e}")
                return False
        
        logger.info("动作序列执行完成")
        return True
    
    def is_position_reachable(self, x: float, y: float, z: float) -> bool:
        """检查位置是否可达"""
        return self.kinematics.is_reachable(x, y, z)
    
    def get_workspace_info(self) -> Dict:
        """获取工作空间信息"""
        return self.kinematics.get_workspace()


# 简单的机械臂客户端接口
class ArmClient:
    """
    机械臂客户端（简化版，实际应使用ZMQ连接到ArmService）
    """
    
    def __init__(self, service_addr: str = "tcp://localhost:5557"):
        self.service_addr = service_addr
        self._connected = False
    
    def move_joints(self, joint_angles: Dict[str, float], speed: int = 800) -> bool:
        """
        移动关节到指定角度
        
        Args:
            joint_angles: 关节角度字典
            speed: 运动速度
        
        Returns:
            是否成功
        """
        # 这里应该通过ZMQ发送指令到ArmService
        # 简化版本，仅打印
        angles_str = ", ".join([f"{k}={v:.1f}" for k, v in joint_angles.items()])
        logger.info(f"[ArmClient] 移动关节: {angles_str} (speed={speed})")
        return True
    
    def move_to_position(self, x: float, y: float, z: float, 
                        orientation: float = 0.0) -> bool:
        """
        移动到指定位置（使用逆运动学）
        
        Args:
            x, y, z: 目标位置 (mm)
            orientation: 末端方向 (度)
        
        Returns:
            是否成功
        """
        planner = MotionPlanner()
        joints = planner.kinematics.solve_for_position(x, y, z, orientation)
        
        if joints is None:
            logger.error(f"位置 ({x}, {y}, {z}) 不可达")
            return False
        
        return self.move_joints(joints, speed=800)


if __name__ == "__main__":
    # 测试运动规划器
    planner = MotionPlanner()
    
    print("=" * 60)
    print("运动规划器测试")
    print("=" * 60)
    
    # 测试1: 规划出牌动作
    print("\n测试1: 规划出牌动作")
    sequence = planner.plan_pick_and_place(
        tile_x=150.0,  # 牌的位置
        tile_y=50.0,
        tile_height=30.0
    )
    
    print(f"\n共 {len(sequence)} 个步骤:")
    for i, step in enumerate(sequence):
        print(f"\n步骤 {i+1}: {step.name}")
        print(f"  描述: {step.description}")
        print(f"  预计耗时: {step.duration}s")
        if step.joint_angles:
            angles_str = ", ".join([f"{k}={v:.1f}°" for k, v in step.joint_angles.items()])
            print(f"  关节角度: {angles_str}")
    
    # 测试2: 可达性检查
    print("\n" + "=" * 60)
    print("测试2: 可达性检查")
    print("=" * 60)
    
    test_positions = [
        (200, 0, 100, "正前方"),
        (150, 100, 80, "左前方"),
        (100, 0, 50, "近距离"),
        (300, 0, 100, "远距离"),
    ]
    
    for x, y, z, desc in test_positions:
        reachable = planner.is_position_reachable(x, y, z)
        status = "✓ 可达" if reachable else "✗ 不可达"
        print(f"{desc} ({x}, {y}, {z}): {status}")
