"""
游戏手柄控制主应用

同时控制底盘和机械臂：
- 底盘：左摇杆 + 扳机键
- 机械臂：右摇杆(基座旋转/前伸后缩) + 十字键(升降/腕转) + ABXY + 肩键

坐标系说明：
- R: 水平距离，负值为前伸方向，正值为后缩方向
- Z: 垂直高度，向上为正
"""
import sys
import os
import time
import threading
import signal
import atexit
from typing import Optional, Dict
from dataclasses import dataclass

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from common.logging import get_logger
from configs import GamepadConfig, get_config
from services.motion_service.chassis_arbiter import ChassisArbiterClient, PRIORITIES
from services.motion_service.chassis_arbiter.arbiter import ArmArbiterClient

# 导入运动学模块
try:
    from hal.arm.Kinematics import ArmKinematics
    KINEMATICS_AVAILABLE = True
except ImportError:
    KINEMATICS_AVAILABLE = False
    ArmKinematics = None

# 导入 Xbox 手柄驱动
try:
    from hal.gamepad import XboxController, Button, get_connected_controllers, wait_for_connection
    GAMEPAD_AVAILABLE = True
except ImportError:
    GAMEPAD_AVAILABLE = False
    # 定义占位符，避免导入错误
    class XboxController:
        def __init__(self, index=0):
            raise RuntimeError("游戏手柄驱动未找到")
    class Button:
        pass

logger = get_logger(__name__)


@dataclass
class ChassisVelocity:
    """底盘速度指令"""
    vx: float = 0.0  # 前进/后退
    vy: float = 0.0  # 左右平移
    vz: float = 0.0  # 旋转


class GamepadControlApp:
    """
    游戏手柄控制应用
    
    同时控制底盘和机械臂，无需模式切换。
    """
    
    def __init__(self, config: Optional[GamepadConfig] = None, controller_index: int = 0):
        """
        初始化游戏手柄控制应用
        
        Args:
            config: 游戏手柄配置
            controller_index: 手柄索引 (0-3)
        """
        self.config = config or GamepadConfig()
        self.controller_index = controller_index
        
        # 手柄实例
        self.controller: Optional[XboxController] = None
        
        # 客户端
        self.chassis_client: Optional[ChassisArbiterClient] = None
        self.arm_client: Optional[ArmArbiterClient] = None
        
        # 机械臂当前状态缓存 (用于增量控制)
        self.arm_state: Dict[str, float] = {
            "base": 0.0,
            "shoulder": 0.0,
            "elbow": 90.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 45.0,
        }
        
        # 升降平台状态 (Plus版本)
        self.lift_height: float = 0.0  # 当前高度 (mm)
        self.lift_step: float = 10.0   # 每次调整的步进 (mm)
        
        # 运行状态
        self.running = False
        self.emergency_stopped = False
        self._stop_event = threading.Event()
        
        # 运动学相关
        self._kinematics = None
        self._arm_pos = {"r": 150.0, "z": 150.0}  # 末端位置 (mm)
        
        # 手腕控制模式
        self._wrist_auto_level = True  # True: 自动保持水平, False: 手动模式
        
        # 底盘控制状态
        self._last_chassis_moving = False  # 上次是否处于运动状态（用于死区检测）
        
        # 统计信息
        self.loop_count = 0
        self.last_print_time = time.time()
        
        # 按键状态缓存（用于检测边沿）
        self._prev_button_states: Dict[str, bool] = {}
        
        logger.info("GamepadControlApp 初始化完成")
    
    def initialize(self) -> bool:
        """
        初始化所有组件
        
        Returns:
            bool: 是否初始化成功
        """
        logger.info("=" * 60)
        logger.info("初始化游戏手柄控制应用")
        logger.info("=" * 60)
        
        # 1. 检查手柄驱动
        if not GAMEPAD_AVAILABLE:
            logger.error("游戏手柄驱动未找到，请确保 hal.gamepad 模块可用")
            return False
        
        # 2. 连接手柄
        logger.info(f"连接手柄 (索引: {self.controller_index})...")
        connected = get_connected_controllers()
        if connected:
            logger.info(f"已连接的手柄: {connected}")
        else:
            logger.info("等待手柄连接...")
            if not wait_for_connection(self.controller_index, timeout=10):
                logger.error("超时，未检测到手柄")
                return False
            logger.info("手柄已连接！")
        
        try:
            self.controller = XboxController(self.controller_index)
            # 设置死区
            self.controller.left_deadzone = int(self.config.left_stick_deadzone * 32767)
            self.controller.right_deadzone = int(self.config.right_stick_deadzone * 32767)
            logger.info("✓ 手柄已初始化")
        except Exception as e:
            logger.error(f"手柄初始化失败: {e}")
            return False
        
        # 3. 连接底盘服务
        logger.info("连接底盘服务...")
        try:
            self.chassis_client = ChassisArbiterClient(
                service_addr=self.config.chassis_service_addr,
                timeout_ms=500
            )
            # 测试连接 - 发送一个停止命令
            test_response = self.chassis_client.send_command(
                vx=0.0, vy=0.0, vz=0.0,
                source="gamepad",
                priority=PRIORITIES.get("gamepad", 3)
            )
            if test_response:
                logger.info(f"✓ 底盘客户端已连接: {self.config.chassis_service_addr}")
                logger.debug(f"  测试响应: success={test_response.success}, owner={test_response.current_owner}")
            else:
                logger.warning(f"底盘服务连接成功但测试无响应，服务可能未启动: {self.config.chassis_service_addr}")
        except Exception as e:
            logger.error(f"底盘服务连接失败: {e}")
            return False
        
        # 4. 连接机械臂服务
        logger.info("连接机械臂服务...")
        try:
            self.arm_client = ArmArbiterClient(
                service_addr=self.config.arm_service_addr,
                timeout_ms=1000
            )
            logger.info(f"✓ 机械臂客户端已连接: {self.config.arm_service_addr}")
        except Exception as e:
            logger.error(f"机械臂服务连接失败: {e}")
            return False
        
        # 5. 获取机械臂初始状态
        self._init_kinematics()
        self._sync_arm_state()
        
        # 6. 同步升降平台状态
        self._sync_lift_state()
        
        logger.info("=" * 60)
        logger.info("初始化完成，等待启动...")
        logger.info("=" * 60)
        return True
    
    def _init_kinematics(self):
        """初始化运动学"""
        if KINEMATICS_AVAILABLE:
            config = get_config()
            self._kinematics = ArmKinematics(
                L1=config.arm.upper_arm_length,
                L2=config.arm.forearm_length
            )
            L1 = config.arm.upper_arm_length
            L2 = config.arm.forearm_length
            max_reach = L1 + L2
            min_reach = abs(L1 - L2)
            logger.info(f"运动学初始化: L1={L1}mm, L2={L2}mm")
            logger.info(f"  工作空间: R方向 [-{max_reach:.0f} ~ {max_reach:.0f}]mm, "
                       f"Z方向 [{min_reach:.0f} ~ {max_reach:.0f}]mm")
            logger.info(f"  注意: 负R值为前伸方向，正R值为后缩方向")
        else:
            logger.warning("运动学模块不可用")
    
    def _sync_arm_state(self):
        """从硬件同步机械臂状态，并通过正运动学计算当前末端位置"""
        # 尝试获取当前关节状态
        try:
            if self.arm_client:
                # 发送查询命令 - 使用特殊标记请求当前状态
                response = self.arm_client.send_joint_dict(
                    joints_dict={"query": True},  # 查询标记
                    source="gamepad",
                    priority=PRIORITIES.get("gamepad", 3),
                    speed=0
                )
                if response and response.success:
                    # 检查返回的关节状态
                    joint_states = getattr(response, 'joint_states', None)
                    if joint_states and isinstance(joint_states, dict):
                        shoulder = joint_states.get("shoulder")
                        elbow = joint_states.get("elbow")
                        
                        if shoulder is not None and elbow is not None:
                            # 更新本地状态
                            self.arm_state.update({
                                "base": joint_states.get("base", self.arm_state["base"]),
                                "shoulder": shoulder,
                                "elbow": elbow,
                                "wrist_flex": joint_states.get("wrist_flex", self.arm_state["wrist_flex"]),
                                "wrist_roll": joint_states.get("wrist_roll", self.arm_state["wrist_roll"]),
                                "gripper": joint_states.get("gripper", self.arm_state["gripper"]),
                            })
                            
                            # 通过正运动学计算末端位置
                            if self._kinematics:
                                r, z = self._kinematics.forward_kinematics(shoulder, elbow)
                                self._arm_pos["r"] = r
                                self._arm_pos["z"] = z
                                logger.info(f"机械臂初始位置(从硬件): r={r:.1f}mm, z={z:.1f}mm, "
                                          f"shoulder={shoulder:.1f}°, elbow={elbow:.1f}°")
                                return
        except Exception as e:
            logger.debug(f"获取机械臂初始状态失败: {e}")
        
        # 使用配置中的 rest_position 作为初始位置
        try:
            from configs.config import get_config
            config = get_config()
            rest_pos = config.arm.rest_position
            
            self.arm_state.update({
                "base": rest_pos.get("base", 0.0),
                "shoulder": rest_pos.get("shoulder", 0.0),
                "elbow": rest_pos.get("elbow", 90.0),
                "wrist_flex": rest_pos.get("wrist_flex", 0.0),
                "wrist_roll": rest_pos.get("wrist_roll", 0.0),
                "gripper": rest_pos.get("gripper", 45.0),
            })
            
            # 通过正运动学计算末端位置
            if self._kinematics:
                r, z = self._kinematics.forward_kinematics(
                    self.arm_state["shoulder"], 
                    self.arm_state["elbow"]
                )
                self._arm_pos["r"] = r
                self._arm_pos["z"] = z
                logger.info(f"机械臂初始位置(从配置): r={r:.1f}mm, z={z:.1f}mm")
        except Exception as e:
            logger.warning(f"使用默认初始位置: {e}")
            # 使用硬编码的默认位置
            if self._kinematics:
                r, z = self._kinematics.forward_kinematics(0.0, 90.0)
                self._arm_pos["r"] = r
                self._arm_pos["z"] = z
                logger.info(f"机械臂初始位置(默认): r={r:.1f}mm, z={z:.1f}mm")
    
    def _sync_lift_state(self):
        """从硬件同步升降平台状态"""
        try:
            if self.arm_client:
                response = self.arm_client.send_joint_dict(
                    joints_dict={"query": True},
                    source="gamepad",
                    priority=PRIORITIES.get("gamepad", 3),
                    speed=0
                )
                if response and response.success:
                    lift_height = getattr(response, 'lift_height', None)
                    if lift_height is not None:
                        self.lift_height = lift_height
                        logger.info(f"升降平台初始高度(从硬件): {self.lift_height:.1f}mm")
                        return
        except Exception as e:
            logger.debug(f"获取升降平台初始状态失败: {e}")
        
        logger.info(f"升降平台初始高度(默认): {self.lift_height:.1f}mm")
    
    def _send_lift_command(self, target_height: float):
        """
        发送升降平台控制指令
        
        Args:
            target_height: 目标高度 (mm)
        """
        if self.arm_client is None or self.emergency_stopped:
            return
        
        try:
            response = self.arm_client.send_lift_command(
                height=target_height,
                source="gamepad",
                priority=PRIORITIES.get("gamepad", 3)
            )
            
            if response and response.success:
                self.lift_height = target_height
                logger.debug(f"升降平台: {target_height:.1f}mm")
            elif response:
                logger.debug(f"升降平台指令被拒绝: {response.message}")
                
        except Exception as e:
            logger.warning(f"升降平台通信失败: {e}")
    
    def _handle_chassis_input(self, state) -> ChassisVelocity:
        """
        处理底盘输入
        
        Returns:
            ChassisVelocity: 底盘速度指令
        """
        # 左摇杆控制 (X: 旋转, Y: 前后)
        lx, ly = state.get_left_stick()
        
        # 扳机键控制左右平移 (RT: 右, LT: 左)
        lt = state.left_trigger if state.left_trigger > self.config.trigger_deadzone else 0.0
        rt = state.right_trigger if state.right_trigger > self.config.trigger_deadzone else 0.0
        
        # 计算速度
        # ly: 摇杆上推为负(Y轴向下)，取反使上推为前进
        # 修正：前进为正 vx，后退为负 vx
        vx = ly * self.config.max_linear_speed   # 前进/后退 (修正方向)
        vy = (rt - lt) * self.config.max_linear_speed  # 左右平移
        vz = lx * self.config.max_angular_speed   # 旋转
        
        # 调试日志：当有任何输入时记录
        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(vz) > 0.01:
            logger.debug(f"底盘输入: lx={lx:.3f}, ly={ly:.3f}, lt={lt:.3f}, rt={rt:.3f} -> "
                        f"vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}")
        
        return ChassisVelocity(vx=vx, vy=vy, vz=vz)
    
    def _send_chassis_command(self, velocity: ChassisVelocity):
        """发送底盘控制指令
        
        当从运动状态进入死区时，会发送停止命令以确保底盘及时停止。
        """
        if self.chassis_client is None or self.emergency_stopped:
            if self.emergency_stopped:
                logger.debug("底盘控制被忽略：紧急停止状态")
            return
        
        # 检查当前是否处于运动状态
        is_moving = abs(velocity.vx) > 0.01 or abs(velocity.vy) > 0.01 or abs(velocity.vz) > 0.01
        
        if is_moving:
            # 正常运动时发送命令
            self._last_chassis_moving = True
            try:
                logger.debug(f"发送底盘命令: vx={velocity.vx:.3f}, vy={velocity.vy:.3f}, vz={velocity.vz:.3f}")
                response = self.chassis_client.send_command(
                    vx=velocity.vx,
                    vy=velocity.vy,
                    vz=velocity.vz,
                    source="gamepad",
                    priority=PRIORITIES.get("gamepad", 3)
                )
                if response:
                    if not response.success:
                        logger.debug(f"底盘指令被拒绝: {response.message}")
                    else:
                        logger.debug(f"底盘指令已接受: owner={response.current_owner}")
                else:
                    logger.debug("底盘指令无响应")
            except Exception as e:
                logger.warning(f"底盘通信失败: {e}")
        elif self._last_chassis_moving:
            # 从运动状态进入死区，发送停止命令
            self._last_chassis_moving = False
            try:
                logger.debug("摇杆进入死区，发送停止命令")
                response = self.chassis_client.send_command(
                    vx=0.0, vy=0.0, vz=0.0,
                    source="gamepad",
                    priority=PRIORITIES.get("gamepad", 3)
                )
                if response and response.success:
                    logger.debug("底盘已停止")
            except Exception as e:
                logger.warning(f"发送停止命令失败: {e}")
    
    def _handle_arm_input(self, state) -> Dict[str, float]:
        """
        处理机械臂输入 - 使用运动学计算
        
        控制映射:
        - 右摇杆 ↑: 后缩 (R向正方向移动)
        - 右摇杆 ↓: 前伸 (R向负方向移动)
        - 右摇杆 ←→: 基座旋转
        - 十字键 ↑↓: 抬高/放低 (Z方向移动)
        - 十字键 ←→: 手腕旋转
        - Y: 手腕下翻 (切换手动模式)
        - A: 手腕上翻 (切换手动模式)
        - B: 手腕水平 (切换自动水平模式)
        - RB/LB: 夹爪打开/关闭
        
        手腕控制模式:
        - 自动水平模式 (AUTO): 抬高/降低/前伸/后缩时自动保持手腕水平
        - 手动模式 (MANU): 按Y/A键后进入，抬高/降低/前伸/后缩时保持当前角度不变
        
        坐标系:
        - R: 水平距离，范围 [-(L1+L2), (L1+L2)]，负值为前伸
        - Z: 垂直高度，范围 [abs(L1-L2), L1+L2]
        
        Returns:
            Dict[str, float]: 关节更新字典
        """
        joint_updates = {}
        current = self.arm_state.copy()
        
        # ========== 右摇杆 ==========
        rx, ry = state.get_right_stick()
        
        # 右摇杆左右 -> 基座旋转 (base)
        if abs(rx) > self.config.right_stick_deadzone:
            joint_updates["base"] = current["base"] + rx * self.config.arm_base_step
        
        # ========== 运动学控制 ==========
        # 使用逆运动学计算 shoulder 和 elbow 角度
        # 注意: 在实际安装中，负R值为前伸方向，正R值为后缩方向
        target_r = self._arm_pos["r"]
        target_z = self._arm_pos["z"]
        need_ik = False
        ik_reason = ""  # 用于调试
        
        max_reach = self._kinematics.L1 + self._kinematics.L2
        min_reach = abs(self._kinematics.L1 - self._kinematics.L2)
        
        # 右摇杆上下 -> 前伸/后缩 (R方向)
        # 摇杆上推 (ry < 0) -> 后缩 (R增大，向正方向)
        # 摇杆下推 (ry > 0) -> 前伸 (R减小，向负方向)
        if abs(ry) > self.config.right_stick_deadzone:
            # 摇杆上推 (ry < 0): delta_r > 0, R 增大 = 后缩
            # 摇杆下推 (ry > 0): delta_r < 0, R 减小 = 前伸
            delta_r = -ry * self.config.arm_elbow_step * 3  # 放大步进
            new_r = self._arm_pos["r"] + delta_r
            
            # R 的范围: -(L1+L2) ~ (L1+L2)
            target_r = max(-max_reach + 5, min(new_r, max_reach - 5))
            
            need_ik = True
            direction = "前伸" if delta_r < 0 else "后缩"
            ik_reason = f"R{direction}: delta_r={delta_r:.1f}"
        
        # 十字键上下 -> 抬高/放低 (Z方向)
        if state.is_pressed(Button.DPAD_UP):
            target_z = min(self._arm_pos["z"] + self.config.arm_shoulder_step * 3, 
                           max_reach - 20)
            need_ik = True
            ik_reason = "Z增加"
        elif state.is_pressed(Button.DPAD_DOWN):
            target_z = max(min_reach + 10, self._arm_pos["z"] - self.config.arm_shoulder_step * 3)
            need_ik = True
            ik_reason = "Z减少"
        
        # 使用逆运动学计算关节角度
        if need_ik and self._kinematics:
            # 首先检查目标是否在工作空间内 (以原点为中心的圆环)
            dist = (target_r**2 + target_z**2) ** 0.5
            
            if dist > max_reach or dist < min_reach:
                logger.debug(f"目标位置超出工作空间: r={target_r:.1f}, z={target_z:.1f}, "
                           f"dist={dist:.1f}, 范围=[{min_reach:.1f}, {max_reach:.1f}]")
                # 尝试调整到可达位置
                if dist > max_reach:
                    scale = (max_reach - 1) / dist
                    target_r *= scale
                    target_z *= scale
                elif dist < min_reach:
                    scale = (min_reach + 5) / dist if dist > 0.1 else 1.0
                    target_r *= scale
                    target_z *= scale
            
            angles = self._kinematics.inverse_kinematics(target_r, target_z, elbow_up=True)
            if angles:
                shoulder_angle, elbow_angle = angles
                joint_updates["shoulder"] = shoulder_angle
                joint_updates["elbow"] = elbow_angle
                
                # 根据手腕控制模式决定是否自动计算 wrist_flex
                if self._wrist_auto_level:
                    # 自动模式：保持末端水平
                    wrist_flex = 180.0 - shoulder_angle - elbow_angle
                    joint_updates["wrist_flex"] = wrist_flex
                    logger.debug(f"IK成功(自动水平): {ik_reason} -> r={target_r:.1f}, z={target_z:.1f}, "
                               f"shoulder={shoulder_angle:.1f}°, elbow={elbow_angle:.1f}°, "
                               f"wrist_flex={wrist_flex:.1f}°")
                else:
                    # 手动模式：保持当前 wrist_flex 不变（不添加到 joint_updates）
                    logger.debug(f"IK成功(手动模式): {ik_reason} -> r={target_r:.1f}, z={target_z:.1f}, "
                               f"shoulder={shoulder_angle:.1f}°, elbow={elbow_angle:.1f}°, "
                               f"wrist_flex保持={current['wrist_flex']:.1f}°")
                
                # 更新记录的位置
                self._arm_pos["r"] = target_r
                self._arm_pos["z"] = target_z
            else:
                logger.warning(f"目标位置不可达: r={target_r:.1f}mm, z={target_z:.1f}mm, {ik_reason}")
        
        # ←→ -> 手腕旋转 (wrist_roll)
        if state.is_pressed(Button.DPAD_LEFT):
            joint_updates["wrist_roll"] = current["wrist_roll"] + self.config.arm_wrist_roll_step
        elif state.is_pressed(Button.DPAD_RIGHT):
            joint_updates["wrist_roll"] = current["wrist_roll"] - self.config.arm_wrist_roll_step
        
        # ========== ABXY 按键 ==========
        # Y键 -> 手腕下翻 (wrist_flex -)，切换到手动模式
        if state.is_pressed(Button.Y):
            joint_updates["wrist_flex"] = current["wrist_flex"] - self.config.arm_wrist_flex_step
            if self._wrist_auto_level:
                self._wrist_auto_level = False
                logger.info(f"手腕手动模式: 下翻至 {joint_updates['wrist_flex']:.1f}°")
            else:
                logger.debug(f"手腕下翻: {current['wrist_flex']:.1f}° -> {joint_updates['wrist_flex']:.1f}°")
        
        # A键 -> 手腕上翻 (wrist_flex +)，切换到手动模式
        if state.is_pressed(Button.A):
            joint_updates["wrist_flex"] = current["wrist_flex"] + self.config.arm_wrist_flex_step
            if self._wrist_auto_level:
                self._wrist_auto_level = False
                logger.info(f"手腕手动模式: 上翻至 {joint_updates['wrist_flex']:.1f}°")
            else:
                logger.debug(f"手腕上翻: {current['wrist_flex']:.1f}° -> {joint_updates['wrist_flex']:.1f}°")
        
        # B键 -> 手腕一键水平，切换回自动水平模式
        if state.is_pressed(Button.B):
            shoulder = joint_updates.get("shoulder", current["shoulder"])
            elbow = joint_updates.get("elbow", current["elbow"])
            # 保持末端水平的补偿角度
            wrist_horizontal = 180.0 - shoulder - elbow
            joint_updates["wrist_flex"] = wrist_horizontal
            self._wrist_auto_level = True  # 切换回自动模式
            logger.info(f"手腕自动水平模式: shoulder={shoulder:.1f}°, elbow={elbow:.1f}° -> wrist_flex={wrist_horizontal:.1f}°")
        
        # ========== 肩键 (LB/RB) ==========
        # RB -> 夹爪打开
        if state.is_pressed(Button.RIGHT_SHOULDER):
            joint_updates["gripper"] = self.config.arm_gripper_open
        
        # LB -> 夹爪关闭
        if state.is_pressed(Button.LEFT_SHOULDER):
            joint_updates["gripper"] = self.config.arm_gripper_close
        
        return joint_updates
    
    def _send_arm_command(self, joint_updates: Dict[str, float]):
        """发送机械臂控制指令"""
        if self.arm_client is None or self.emergency_stopped:
            return
        
        if not joint_updates:
            return
        
        try:
            # 更新本地状态
            self.arm_state.update(joint_updates)
            
            # 发送指令
            response = self.arm_client.send_joint_dict(
                joints_dict=self.arm_state,
                source="gamepad",
                priority=PRIORITIES.get("gamepad", 3),
                speed=self.config.arm_speed
            )
            
            if response and not response.success:
                logger.debug(f"机械臂指令被拒绝: {response.message}")
                
        except Exception as e:
            logger.warning(f"机械臂通信失败: {e}")
    
    def _handle_system_input(self, state):
        """处理系统控制输入"""
        # Back键 -> 紧急停止
        if state.is_pressed(Button.BACK):
            self._emergency_stop()
            return True
        
        # Start键 -> 复位
        if state.is_pressed(Button.START):
            self._reset()
            return True
        
        # L3 (左摇杆按下) -> 升降平台上升 (向0移动)
        if self._is_button_just_pressed(state, Button.LEFT_THUMB):
            from configs import get_config
            max_height = get_config().lift_platform.max_height  # 通常为 0
            new_height = min(max_height, self.lift_height + self.lift_step)
            logger.info(f"升降平台上升: {self.lift_height:.1f}mm -> {new_height:.1f}mm")
            self._send_lift_command(new_height)
            return True
        
        # R3 (右摇杆按下) -> 升降平台下降 (向负方向移动)
        if self._is_button_just_pressed(state, Button.RIGHT_THUMB):
            from configs import get_config
            min_height = get_config().lift_platform.min_height  # 通常为 -200
            new_height = max(min_height, self.lift_height - self.lift_step)
            logger.info(f"升降平台下降: {self.lift_height:.1f}mm -> {new_height:.1f}mm")
            self._send_lift_command(new_height)
            return True
        
        return False
    
    def _is_button_just_pressed(self, state, button) -> bool:
        """
        检测按钮是否刚刚被按下（边沿检测）
        
        Args:
            state: 当前手柄状态
            button: 按钮枚举值
            
        Returns:
            是否刚按下（之前未按下，现在按下）
        """
        button_name = button.name if hasattr(button, 'name') else str(button)
        is_pressed = state.is_pressed(button)
        was_pressed = self._prev_button_states.get(button_name, False)
        self._prev_button_states[button_name] = is_pressed
        return is_pressed and not was_pressed
    
    def _emergency_stop(self):
        """紧急停止"""
        if self.emergency_stopped:
            return
        
        self.emergency_stopped = True
        logger.error("!!! 紧急停止已触发 !!!")
        
        # 停止底盘
        if self.chassis_client:
            try:
                self.chassis_client.send_command(
                    vx=0.0, vy=0.0, vz=0.0,
                    source="emergency",
                    priority=PRIORITIES.get("emergency", 4)
                )
            except Exception as e:
                logger.error(f"急停底盘失败: {e}")
        
        # 震动反馈
        if self.controller:
            try:
                self.controller.set_vibration(1.0, 1.0)
                time.sleep(0.3)
                self.controller.stop_vibration()
            except:
                pass
    
    def _reset(self):
        """复位系统 - 所有关节归位，升降平台归零，然后从硬件同步实际状态"""
        logger.info("系统复位...")
        self.emergency_stopped = False
        
        # 停止底盘
        if self.chassis_client:
            try:
                self.chassis_client.send_command(
                    vx=0.0, vy=0.0, vz=0.0,
                    source="gamepad",
                    priority=PRIORITIES.get("gamepad", 3)
                )
            except Exception as e:
                logger.error(f"复位底盘失败: {e}")
        
        # 机械臂归位
        if self.arm_client:
            try:
                response = self.arm_client.send_joint_dict(
                    joints_dict={},  # 空字典表示归位
                    source="gamepad",
                    priority=PRIORITIES.get("gamepad", 3),
                    speed=self.config.arm_speed
                )
                if response and response.success:
                    logger.info("机械臂归位指令已发送")
            except Exception as e:
                logger.error(f"复位机械臂失败: {e}")
            
            # 升降平台归零
            try:
                logger.info("升降平台归位...")
                response = self.arm_client.send_lift_command(
                    height=0.0,
                    source="gamepad",
                    priority=PRIORITIES.get("gamepad", 3)
                )
                if response and response.success:
                    self.lift_height = 0.0
                    logger.info("升降平台归位指令已发送")
            except Exception as e:
                logger.error(f"复位升降平台失败: {e}")
        
        # 等待机械臂完成归位（给舵机一些时间运动）
        time.sleep(0.5)
        
        # 从硬件同步实际关节状态
        self._sync_arm_state()
        self._sync_lift_state()
        
        # 重置手腕控制模式为自动水平
        self._wrist_auto_level = True
        
        logger.info("系统复位完成")
    
    def _print_status(self, chassis_vel: ChassisVelocity, arm_updates: Dict[str, float]):
        """打印状态信息 (每秒一次)"""
        current_time = time.time()
        if current_time - self.last_print_time >= 1.0:
            # 清行并打印状态
            print("\033[2K\r", end="")  # 清除当前行
            wrist_mode = "AUTO" if self._wrist_auto_level else "MANU"
            status = (
                f"[底盘] vx={chassis_vel.vx:+.2f} vy={chassis_vel.vy:+.2f} vz={chassis_vel.vz:+.2f} | "
                f"[机械臂] base={self.arm_state['base']:+.0f}° shoulder={self.arm_state['shoulder']:+.0f}° "
                f"elbow={self.arm_state['elbow']:+.0f}° wrist_flex={self.arm_state['wrist_flex']:+.0f}°({wrist_mode}) "
                f"gripper={self.arm_state['gripper']:.0f}° | "
                f"[升降] {self.lift_height:.0f}mm"
            )
            print(status, end="", flush=True)
            self.last_print_time = current_time
    
    def run(self):
        """主循环"""
        if not self.initialize():
            logger.error("初始化失败，无法启动")
            return
        
        self.running = True
        self._stop_event.clear()
        
        # 注册信号处理程序
        self._setup_signal_handlers()
        
        # 注册退出处理
        atexit.register(self.stop)
        
        logger.info("=" * 60)
        logger.info("游戏手柄控制已启动")
        logger.info("=" * 60)
        logger.info("控制映射:")
        logger.info("  底盘: 左摇杆(移动/旋转) + LT/RT(平移)")
        logger.info("  机械臂:")
        logger.info("    右摇杆 ←→: 基座旋转")
        logger.info("    右摇杆 ↑: 后缩 (R+)")
        logger.info("    右摇杆 ↓: 前伸 (R-)")
        logger.info("    十字键 ↑↓: 抬高/放低 (Z方向)")
        logger.info("    Y: 手腕下翻(MANU), A: 手腕上翻(MANU), B: 手腕水平(AUTO)")
        logger.info("    RB/LB: 夹爪打开/关闭")
        logger.info("  升降平台 (Plus):")
        logger.info("    L3(左摇杆按下): 下降(向负方向), R3(右摇杆按下): 上升(向0)")
        logger.info("    坐标系: 最高点=0mm, 向下为负, 最低点=-200mm")
        logger.info("  系统: Back(急停) / Start(复位)")
        logger.info("=" * 60)
        logger.info("按 Ctrl+C 停止")
        logger.info("=" * 60)
        
        # 启动手柄轮询
        self.controller.start_polling(interval=self.config.polling_interval)
        
        try:
            while self.running and not self._stop_event.is_set():
                # 读取手柄状态
                try:
                    state = self.controller.get_state()
                except Exception as e:
                    logger.warning(f"获取手柄状态失败: {e}")
                    time.sleep(0.1)
                    continue
                
                if not state.connected:
                    logger.warning("手柄已断开，等待重新连接...")
                    time.sleep(0.5)
                    continue
                
                # 处理系统输入 (急停/复位)
                if self._handle_system_input(state):
                    time.sleep(self.config.polling_interval)
                    continue
                
                # 处理底盘输入
                chassis_vel = self._handle_chassis_input(state)
                self._send_chassis_command(chassis_vel)
                
                # 处理机械臂输入
                arm_updates = self._handle_arm_input(state)
                if arm_updates:
                    self._send_arm_command(arm_updates)
                
                # 打印状态 (调试用)
                self._print_status(chassis_vel, arm_updates)
                
                # 控制循环频率 - 使用短 sleep 以快速响应停止事件
                self._stop_event.wait(self.config.polling_interval)
                
        except KeyboardInterrupt:
            logger.info("\n收到中断信号，正在停止...")
        except Exception as e:
            logger.error(f"运行异常: {e}")
        finally:
            self.stop()
            # 取消退出处理注册
            try:
                atexit.unregister(self.stop)
            except:
                pass
    
    def _setup_signal_handlers(self):
        """设置信号处理程序"""
        def signal_handler(signum, frame):
            logger.info(f"收到信号 {signum}，正在停止...")
            self._stop_event.set()
            self.running = False
        
        # 注册 SIGINT (Ctrl+C) 和 SIGTERM 处理程序
        signal.signal(signal.SIGINT, signal_handler)
        if hasattr(signal, 'SIGTERM'):
            signal.signal(signal.SIGTERM, signal_handler)
        # Windows 上的 Ctrl+Break
        if hasattr(signal, 'SIGBREAK'):
            signal.signal(signal.SIGBREAK, signal_handler)
    
    def stop(self):
        """停止应用"""
        if not self.running and self._stop_event.is_set():
            # 已经停止，避免重复执行
            return
        
        logger.info("停止游戏手柄控制应用...")
        self.running = False
        self._stop_event.set()
        self._last_chassis_moving = False  # 重置底盘运动状态
        
        # 停止底盘 - 发送停止命令
        if self.chassis_client:
            try:
                self.chassis_client.send_command(
                    vx=0.0, vy=0.0, vz=0.0,
                    source="gamepad",
                    priority=PRIORITIES.get("gamepad", 3)
                )
            except Exception as e:
                logger.debug(f"停止底盘时出错: {e}")
        
        # 停止手柄轮询
        if self.controller:
            try:
                self.controller.stop_vibration()
                self.controller.stop_polling()
                logger.info("手柄轮询已停止")
            except Exception as e:
                logger.debug(f"停止手柄轮询时出错: {e}")
        
        # 关闭客户端
        if self.chassis_client:
            try:
                self.chassis_client.close()
                logger.info("底盘客户端已关闭")
            except Exception as e:
                logger.debug(f"关闭底盘客户端时出错: {e}")
            self.chassis_client = None
        
        if self.arm_client:
            try:
                self.arm_client.close()
                logger.info("机械臂客户端已关闭")
            except Exception as e:
                logger.debug(f"关闭机械臂客户端时出错: {e}")
            self.arm_client = None
        
        logger.info("应用已停止")


# 入口函数
def main():
    """主入口"""
    import argparse
    
    parser = argparse.ArgumentParser(description="游戏手柄控制应用")
    parser.add_argument("--controller", "-c", type=int, default=0, help="手柄索引")
    parser.add_argument("--verbose", "-v", action="store_true", help="详细日志")
    
    args = parser.parse_args()
    
    if args.verbose:
        import logging
        logging.getLogger().setLevel(logging.DEBUG)
    
    app = GamepadControlApp(controller_index=args.controller)
    app.run()


if __name__ == "__main__":
    main()
