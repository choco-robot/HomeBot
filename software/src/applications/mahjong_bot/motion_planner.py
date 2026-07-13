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
    HOVER_HEIGHT = 145.0      # 悬停高度 (mm)
    GRASP_HEIGHT = 100.0      # 抓取高度 (mm)
    LIFT_HEIGHT = 170.0      # 提起高度 (mm)
    SLOT_HEIGHT = 0.0       # 出牌槽高度 (mm)
    
    # 出牌槽位置 (机械臂坐标系)
    DISCARD_SLOT_X = 200.0   # 出牌槽X位置
    DISCARD_SLOT_Y = 170.0     # 出牌槽Y位置（正前方）

    #默认速度
    DEFAULT_SPEED = 1200
    DEFAULT_ACC = 20
    
    def __init__(self):
        """初始化运动规划器"""
        config = get_config()
        arm_cfg = config.arm
        
        # 初始化运动学（从配置读取URDF路径）
        project_root = Path(__file__).resolve().parents[4]
        urdf_path = str(project_root / arm_cfg.urdf_path)
        self.kinematics = Arm3DKinematics(
            L1=arm_cfg.upper_arm_length,
            L2=arm_cfg.forearm_length,
            urdf_path=urdf_path,
        )
        
        # 关节限制（从配置读取）
        self.joint_limits = arm_cfg.joint_limits
        
        # 休息位姿
        self.rest_pose = arm_cfg.rest_position
        
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
            elbow_up=elbow_up  # 使用 elbow_up 构型
        )
        # 设置 wrist_roll = base，使夹爪随基座同步旋转，保持相对朝向
        if result and 'base' in result:
            result['wrist_roll'] = result['base']
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
    
    def plan_pick_and_place_cube(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        linear_step_mm: Optional[float] = None,
        interp_duration: float = 0.0,
    ) -> List[MotionStep]:
        """
        规划抓取并放置立方体的完整动作序列

        流程：悬停 -> 下降 -> 夹紧 -> 提起 -> 移动到放置区 -> 下降 -> 松开 -> 返回休息位

        Args:
            target_x: 目标X坐标 (mm)
            target_y: 目标Y坐标 (mm)
            target_z: 目标Z坐标 (mm)
            linear_step_mm: 直线插补步长 (mm)，大于0时在每个笛卡尔位姿移动之间插入中间点
            interp_duration: 插补中间点的统一等待时长 (秒)。
                            0.0 表示等待舵机稳定；大于0表示固定等待时间

        Returns:
            运动步骤列表
        """
        sequence = []

        # 步骤1: 移动到目标上方悬停
        hover_pose = Pose3D(x=target_x, y=target_y, z=target_z+30, orientation=90.0)
        hover_joints = self._pose_to_joints(hover_pose)
        if hover_joints:
            sequence.append(MotionStep(
                name="move_to_hover",
                target_pose=hover_pose,
                joint_angles={**hover_joints, 'gripper': 90.0},
                gripper_open=True,
                duration=0.0,
                description=f"移动到目标上方 ({target_x:.0f}, {target_y:.0f})"
            ))

        # 步骤2: 下降准备抓取
        grab_pose = Pose3D(x=target_x, y=target_y, z=target_z, orientation=90.0)
        grab_joints = self._pose_to_joints(grab_pose)
        if grab_joints:
            sequence.append(MotionStep(
                name="descend_and_grasp",
                target_pose=grab_pose,
                joint_angles=grab_joints,
                gripper_open=True,
                duration=0.0,
                description="下降准备抓取"
            ))

            # 步骤3: 夹紧
            sequence.append(MotionStep(
                name="close_gripper",
                target_pose=None,
                joint_angles={**grab_joints, 'gripper': 20.0},  # 夹紧
                gripper_open=False,
                duration=2.5,
                description="夹紧立方体"
            ))

        # 步骤4: 提起
        lift_pose = Pose3D(x=target_x, y=target_y, z=self.SLOT_HEIGHT, orientation=90.0)
        lift_joints = self._pose_to_joints(lift_pose)
        if lift_joints:
            sequence.append(MotionStep(
                name="lift",
                target_pose=lift_pose,
                joint_angles={**lift_joints, 'gripper': 20.0},  # 保持夹紧
                gripper_open=False,
                duration=0.0,
                description="提起立方体"
            ))

        # 步骤5: 移动到放置区上方
        place_hover_pose = Pose3D(
            x=self.DISCARD_SLOT_X,
            y=self.DISCARD_SLOT_Y,
            z=self.SLOT_HEIGHT,
            orientation=90.0
        )
        place_joints = self._pose_to_joints(place_hover_pose)
        if place_joints:
            sequence.append(MotionStep(
                name="move_to_place",
                target_pose=place_hover_pose,
                joint_angles={**place_joints, 'gripper': 20.0},
                gripper_open=False,
                duration=0.0,
                description="移动到放置区上方"
            ))

            # 步骤6: 下降准备放置
            place_down_pose = Pose3D(
                x=self.DISCARD_SLOT_X,
                y=self.DISCARD_SLOT_Y,
                z=self.SLOT_HEIGHT - 40.0,
                orientation=90.0
            )
            place_down_joints = self._pose_to_joints(place_down_pose)
            if place_down_joints:
                sequence.append(MotionStep(
                    name="descend_to_place",
                    target_pose=place_down_pose,
                    joint_angles={**place_down_joints, 'gripper': 20.0},
                    gripper_open=False,
                    duration=0.0,
                    description="下降准备放置"
                ))

                # 步骤7: 松开夹爪
                sequence.append(MotionStep(
                    name="open_gripper",
                    target_pose=None,
                    joint_angles={**place_down_joints, 'gripper': 45.0},  # 松开
                    gripper_open=True,
                    duration=0.5,
                    description="松开立方体"
                ))

        # 步骤8: 返回休息位
        sequence.append(MotionStep(
            name="return_to_rest",
            target_pose=None,
            joint_angles=self.rest_pose,
            gripper_open=True,
            duration=2.0,
            description="返回休息位"
        ))

        if linear_step_mm is not None and linear_step_mm > 0:
            sequence = self.interpolate_sequence_linear(
                sequence, step_mm=linear_step_mm, interp_duration=interp_duration
            )

        self.motion_sequence = sequence
        logger.info(f"规划完成: {len(sequence)} 个步骤")
        for i, step in enumerate(sequence):
            logger.info(f"  {i+1}. {step.name}: {step.description}")

        return sequence

    def interpolate_sequence_linear(
        self,
        sequence: List[MotionStep],
        step_mm: float = 10.0,
        interp_duration: float = 0.0,
    ) -> List[MotionStep]:
        """
        对运动序列中的笛卡尔位姿移动进行直线插补

        只对相邻且都包含 target_pose 的步骤之间进行插补，夹爪动作、回休息位等
        无 target_pose 的步骤会被原样保留。

        Args:
            sequence: 原始运动步骤列表
            step_mm: 最大直线插补步长 (mm)，两点间距超过此值时插入中间点
            interp_duration: 插补中间点的统一等待时长 (秒)。
                             0.0 表示等待舵机稳定；大于0表示固定等待时间

        Returns:
            插补后的运动步骤列表
        """
        if not sequence or step_mm <= 0:
            return sequence

        interpolated: List[MotionStep] = []
        prev_pose_step: Optional[MotionStep] = None

        for step in sequence:
            # 无 target_pose 的步骤直接保留（如 close_gripper / open_gripper / return_to_rest）
            if step.target_pose is None:
                interpolated.append(step)
                continue

            # 首个有位姿的步骤直接保留
            if prev_pose_step is None:
                interpolated.append(step)
                prev_pose_step = step
                continue

            start = prev_pose_step.target_pose
            end = step.target_pose

            # 计算直线距离
            dx = end.x - start.x
            dy = end.y - start.y
            dz = end.z - start.z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            # 距离足够小则无需插补
            if distance <= step_mm:
                interpolated.append(step)
                prev_pose_step = step
                continue

            # 计算需要插入的中间点数量
            num_segments = int(math.ceil(distance / step_mm))

            for i in range(1, num_segments):
                t = i / num_segments
                interp_pose = Pose3D(
                    x=start.x + dx * t,
                    y=start.y + dy * t,
                    z=start.z + dz * t,
                    orientation=start.orientation + (end.orientation - start.orientation) * t,
                )

                joints = self._pose_to_joints(interp_pose)
                if joints is None:
                    logger.warning(
                        f"直线插补点不可达，已跳过: "
                        f"({interp_pose.x:.1f}, {interp_pose.y:.1f}, {interp_pose.z:.1f})"
                    )
                    continue

                interpolated.append(MotionStep(
                    name=f"{step.name}_interp_{i}",
                    target_pose=interp_pose,
                    joint_angles=joints,
                    gripper_open=step.gripper_open,
                    duration=interp_duration,
                    description=f"{step.description} (插补 {i}/{num_segments - 1})",
                ))

            interpolated.append(step)
            prev_pose_step = step

        logger.info(
            f"直线插补完成: 原始 {len(sequence)} 步 -> 插补后 {len(interpolated)} 步"
        )
        return interpolated

    def plan_push(self, 
                           tile_x: float, 
                           tile_y: float,
                           tile_height: float = 40.0,
                           push_distance: float = 50) -> List[MotionStep]:
        sequence = []
        
        # 步骤1: 移动到牌上方（悬停）
        hover_pose = Pose3D(
            x=tile_x-20,
            y=tile_y,
            z=self.HOVER_HEIGHT,
            orientation=90.0  # 手腕竖直向下
        )
        hover_joints = self._pose_to_joints(hover_pose)
        if hover_joints:
            sequence.append(MotionStep(
                name="move_to_hover",
                target_pose=hover_pose,
                joint_angles={**hover_joints,'gripper':0},
                gripper_open=True,
                duration=0.0,
                description=f"移动到牌上方 ({tile_x:.0f}, {tile_y:.0f})"
            ))

        # step2
        hover_pose = Pose3D(
            x=tile_x-20,
            y=tile_y,
            z=60,
            orientation=90.0  # 手腕竖直向下
        )
        hover_joints = self._pose_to_joints(hover_pose)
        if hover_joints:
            sequence.append(MotionStep(
                name="move_to_hover",
                target_pose=hover_pose,
                joint_angles=hover_joints,
                gripper_open=True,
                duration=0.0,
                description=f"移动到牌上方 ({tile_x:.0f}, {tile_y:.0f})"
            ))

        # step3
        hover_pose = Pose3D(
            x=tile_x-20+push_distance,
            y=tile_y,
            z=60,
            orientation=90.0  # 手腕竖直向下
        )
        hover_joints = self._pose_to_joints(hover_pose)
        if hover_joints:
            sequence.append(MotionStep(
                name="move_to_hover",
                target_pose=hover_pose,
                joint_angles=hover_joints,
                gripper_open=True,
                duration=0.0,
                description=f"移动到牌上方 ({tile_x:.0f}, {tile_y:.0f})"
            ))

        #step4
        hover_pose = Pose3D(
            x=tile_x+20,
            y=tile_y,
            z=100,
            orientation=90.0  # 手腕竖直向下
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
        
        self.motion_sequence = sequence
        logger.info(f"规划完成: {len(sequence)} 个步骤")
        for i, step in enumerate(sequence):
            logger.info(f"  {i+1}. {step.name}: {step.description}")

        return sequence
        
    
    def plan_pick_and_place(self, 
                           tile_x: float, 
                           tile_y: float,
                           tile_height: float = 40.0) -> List[MotionStep]:
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
            orientation=90.0  # 手腕竖直向下
        )
        hover_joints = self._pose_to_joints(hover_pose)
        if hover_joints:
            sequence.append(MotionStep(
                name="move_to_hover",
                target_pose=hover_pose,
                joint_angles=hover_joints,
                gripper_open=True,
                duration=0.0,
                description=f"移动到牌上方 ({tile_x:.0f}, {tile_y:.0f})"
            ))
        
        # 步骤2: 下降并抓取
        grasp_pose = Pose3D(
            x=tile_x,
            y=tile_y,
            z=tile_height,
            orientation=90.0  # 手腕竖直向下
        )
        grasp_joints = self._pose_to_joints(grasp_pose)
        if grasp_joints:
            sequence.append(MotionStep(
                name="descend_and_grasp",
                target_pose=grasp_pose,
                joint_angles=grasp_joints,
                gripper_open=True,
                duration=0.0,
                description="下降准备抓取"
            ))
            
            # 步骤3: 夹紧
            sequence.append(MotionStep(
                name="close_gripper",
                target_pose=None,
                joint_angles={**grasp_joints, 'gripper': 0.0},  # 夹紧
                gripper_open=False,
                duration=0.0,
                description="夹紧牌"
            ))
        
        # 步骤4: 提起
        lift_pose = Pose3D(
            x=tile_x,
            y=tile_y,
            z=self.LIFT_HEIGHT,
            orientation=90.0  # 手腕竖直向下
        )
        lift_joints = self._pose_to_joints(lift_pose)
        if lift_joints:
            sequence.append(MotionStep(
                name="lift",
                target_pose=lift_pose,
                joint_angles={**lift_joints, 'gripper': 0.0},  # 保持夹紧
                gripper_open=False,
                duration=0.0,
                description="提起牌"
            ))
        
        # 步骤5: 移动到出牌槽上方
        slot_hover_pose = Pose3D(
            x=self.DISCARD_SLOT_X,
            y=self.DISCARD_SLOT_Y,
            z=self.SLOT_HEIGHT,
            orientation=100.0  # 手腕竖直向下
        )
        slot_joints = self._pose_to_joints(slot_hover_pose)
        slot_joints['wrist_flex']=90
        if slot_joints:
            sequence.append(MotionStep(
                name="move_to_slot",
                target_pose=slot_hover_pose,
                joint_angles={**slot_joints, 'gripper': 0.0},
                gripper_open=False,
                duration=0.0,
                description="移动到出牌槽"
            ))
            
            # 步骤6: 下降出牌
            
            slot_release_pose = Pose3D(
                x=self.DISCARD_SLOT_X,
                y=self.DISCARD_SLOT_Y,
                z=self.SLOT_HEIGHT-40,
                orientation=100.0  # 手腕竖直向下
            )
            release_joints = self._pose_to_joints(slot_release_pose)
            release_joints['wrist_flex']=90
            if release_joints:
                # sequence.append(MotionStep(
                #     name="lower_to_release",
                #     target_pose=slot_release_pose,
                #     joint_angles={**release_joints, 'gripper': 0.0},
                #     gripper_open=False,
                #     duration=0.0,
                #     description="下降准备出牌"
                # ))

                # slot_push_pose = Pose3D(
                #     x=self.DISCARD_SLOT_X+50.0,  # 前推50mm
                #     y=self.DISCARD_SLOT_Y,
                #     z=tile_height + 35.0,
                #     orientation=90.0  # 手腕竖直向下
                # )

                # sequence.append(MotionStep(
                #     name="open_gripper",
                #     target_pose=slot_push_pose,
                #     joint_angles=self._pose_to_joints(slot_push_pose),
                #     gripper_open=False,
                #     duration=0.0,
                #     description="松开牌"
                # ))

                
                # 步骤7: 释放
                sequence.append(MotionStep(
                    name="open_gripper",
                    target_pose=None,
                    joint_angles={**release_joints, 'gripper': 45.0},  # 松开
                    gripper_open=True,
                    duration=0.5,
                    description="松开牌"
                ))

                # 步骤8: 前推放倒牌
                slot_release_pose = Pose3D(
                    x=self.DISCARD_SLOT_X,
                    y=self.DISCARD_SLOT_Y,
                    z=tile_height + 60.0, #抬高一点
                    orientation=90
                )
                sequence.append(MotionStep(
                    name="lift_after_release",
                    target_pose=slot_release_pose,
                    joint_angles=self._pose_to_joints(slot_release_pose),   
                    gripper_open=True,
                    duration=0.5,
                    description="松开后稍微提起"
                ))
                release_joints = self._pose_to_joints(slot_release_pose)
                release_joints['wrist_flex'] -= 30  # 前推动作，调整腕关节屈伸
                sequence.append(MotionStep(
                    name="push_tile",
                    target_pose=None,
                    joint_angles=release_joints ,  # 前推动作
                    gripper_open=True,
                    duration=0.0,
                    description="前推放倒牌"
                ))
        
        # 步骤9: 提起并返回休息位
        sequence.append(MotionStep(
            name="return_to_rest",
            target_pose=None,
            joint_angles=self.rest_pose,
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
                    success = arm_client.move_joints(step.joint_angles, speed=self.DEFAULT_SPEED, acc=self.DEFAULT_ACC)
                    if not success:
                        logger.error(f"步骤 {step.name} 执行失败")
                        return False
                    
                    # 等待运动完成
                    if step.duration > 0:
                        # 固定时长等待
                        time.sleep(step.duration)
                    else:
                        # duration=0 时，等待舵机停止运动
                        if not self._wait_for_motion_stop(arm_client, timeout=10.0):
                            logger.warning(f"步骤 {step.name} 等待运动完成超时")
                
                if on_step_complete:
                    on_step_complete(step, i)
                    
            except Exception as e:
                logger.error(f"执行步骤 {step.name} 时出错: {e}")
                return False
        
        logger.info("动作序列执行完成")
        return True
    
    def _wait_for_motion_stop(self, arm_client, timeout: float = 10.0, 
                              check_interval: float = 0.1,
                              stable_threshold: float = 2.0,
                              stable_count: int = 2) -> bool:
        """
        等待舵机停止运动
        
        通过轮询关节角度变化来判断运动是否完成：
        1. 先等待最短延时（让运动开始）
        2. 连续读取关节角度
        3. 当连续多次读取的角度变化都小于阈值时，认为运动完成
        
        Args:
            arm_client: 机械臂客户端
            timeout: 最大等待时间（秒）
            check_interval: 检查间隔（秒）
            stable_threshold: 角度变化阈值（度），小于此值认为已停止
            stable_count: 连续稳定次数，达到此次数认为运动完成
        
        Returns:
            True=运动完成，False=超时
        """
        # 先等待最短延时，让运动开始
        time.sleep(0.3)
        
        start_time = time.time()
        last_angles = None
        stable_times = 0
        
        while time.time() - start_time < timeout:
            # 获取当前关节角度
            state = arm_client.get_state()
            if not state or not state.joint_angles:
                time.sleep(check_interval)
                continue
            
            current_angles = state.joint_angles
            
            if last_angles is not None:
                # 计算角度变化
                max_diff = 0.0
                for joint, angle in current_angles.items():
                    if joint in last_angles:
                        diff = abs(angle - last_angles[joint])
                        max_diff = max(max_diff, diff)
                
                # 如果角度变化小于阈值，增加稳定计数
                if max_diff < stable_threshold:
                    stable_times += 1
                    if stable_times >= stable_count:
                        logger.debug(f"运动完成，关节角度已稳定")
                        return True
                else:
                    # 角度变化较大，重置稳定计数
                    stable_times = 0
            
            last_angles = current_angles.copy()
            time.sleep(check_interval)
        
        # 超时
        return False
    
    def is_position_reachable(self, x: float, y: float, z: float) -> bool:
        """检查位置是否可达"""
        return self.kinematics.is_reachable(x, y, z)
    
    def get_workspace_info(self) -> Dict:
        """获取工作空间信息"""
        return self.kinematics.get_workspace()

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
