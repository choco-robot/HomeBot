"""
游戏手柄控制主应用

同时控制底盘和机械臂：
- 底盘：左摇杆 + 扳机键
- 机械臂：右摇杆 + 十字键 + ABXY + 肩键
"""
import sys
import os
import time
import threading
from typing import Optional, Dict
from dataclasses import dataclass

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from common.logging import get_logger
from configs import GamepadConfig
from services.motion_service.chassis_arbiter import ChassisArbiterClient, PRIORITIES
from services.motion_service.chassis_arbiter.arbiter import ArmArbiterClient

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
        
        # 运行状态
        self.running = False
        self.emergency_stopped = False
        self._stop_event = threading.Event()
        
        # 统计信息
        self.loop_count = 0
        self.last_print_time = time.time()
        
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
            logger.info(f"✓ 底盘客户端已连接: {self.config.chassis_service_addr}")
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
        self._sync_arm_state()
        
        logger.info("=" * 60)
        logger.info("初始化完成，等待启动...")
        logger.info("=" * 60)
        return True
    
    def _sync_arm_state(self):
        """从硬件同步机械臂状态"""
        # 初始使用默认值，后续可以通过服务获取
        logger.info(f"机械臂初始状态: {self.arm_state}")
    
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
        # ly: 摇杆上推为负(Y轴向下)，需要取反使上推为前进
        vx = -ly * self.config.max_linear_speed   # 前进/后退
        vy = (rt - lt) * self.config.max_linear_speed  # 左右平移
        vz = lx * self.config.max_angular_speed   # 旋转
        
        return ChassisVelocity(vx=vx, vy=vy, vz=vz)
    
    def _send_chassis_command(self, velocity: ChassisVelocity):
        """发送底盘控制指令"""
        if self.chassis_client is None or self.emergency_stopped:
            return
        
        try:
            response = self.chassis_client.send_command(
                vx=velocity.vx,
                vy=velocity.vy,
                vz=velocity.vz,
                source="gamepad",
                priority=PRIORITIES.get("voice", 2)  # 使用voice优先级
            )
            if response and not response.success:
                logger.debug(f"底盘指令被拒绝: {response.message}")
        except Exception as e:
            logger.warning(f"底盘通信失败: {e}")
    
    def _handle_arm_input(self, state) -> Dict[str, float]:
        """
        处理机械臂输入
        
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
        
        # 右摇杆上下 -> 肘关节 (elbow) 前伸/后缩
        # 摇杆上推为负，取反使上推为前伸（elbow角度增大）
        if abs(ry) > self.config.right_stick_deadzone:
            joint_updates["elbow"] = current["elbow"] - ry * self.config.arm_elbow_step
        
        # ========== 十字键 ==========
        # ↑↓ -> 肩关节 (shoulder) 上升/下降
        if state.is_pressed(Button.DPAD_UP):
            joint_updates["shoulder"] = current["shoulder"] + self.config.arm_shoulder_step
        elif state.is_pressed(Button.DPAD_DOWN):
            joint_updates["shoulder"] = current["shoulder"] - self.config.arm_shoulder_step
        
        # ←→ -> 手腕旋转 (wrist_roll)
        if state.is_pressed(Button.DPAD_LEFT):
            joint_updates["wrist_roll"] = current["wrist_roll"] + self.config.arm_wrist_roll_step
        elif state.is_pressed(Button.DPAD_RIGHT):
            joint_updates["wrist_roll"] = current["wrist_roll"] - self.config.arm_wrist_roll_step
        
        # ========== ABXY 按键 ==========
        # Y键 -> 手腕上翻 (wrist_flex +)
        if state.is_pressed(Button.Y):
            joint_updates["wrist_flex"] = current["wrist_flex"] + self.config.arm_wrist_flex_step
        
        # A键 -> 手腕下翻 (wrist_flex -)
        if state.is_pressed(Button.A):
            joint_updates["wrist_flex"] = current["wrist_flex"] - self.config.arm_wrist_flex_step
        
        # B键 -> 手腕一键水平 (自动计算补偿角度)
        if state.is_pressed(Button.B):
            shoulder = current.get("shoulder", 0.0)
            elbow = current.get("elbow", 90.0)
            # 保持末端水平的补偿角度
            wrist_horizontal = 180.0 - shoulder - elbow
            joint_updates["wrist_flex"] = wrist_horizontal
            logger.info(f"手腕水平: shoulder={shoulder:.1f}°, elbow={elbow:.1f}° -> wrist_flex={wrist_horizontal:.1f}°")
        
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
                priority=PRIORITIES.get("voice", 2),
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
        
        return False
    
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
        """复位系统"""
        logger.info("系统复位...")
        self.emergency_stopped = False
        
        # 停止底盘
        if self.chassis_client:
            try:
                self.chassis_client.send_command(
                    vx=0.0, vy=0.0, vz=0.0,
                    source="gamepad",
                    priority=PRIORITIES.get("voice", 2)
                )
            except Exception as e:
                logger.error(f"复位底盘失败: {e}")
        
        # 机械臂归位
        if self.arm_client:
            try:
                response = self.arm_client.send_joint_dict(
                    joints_dict={},  # 空字典表示归位
                    source="gamepad",
                    priority=PRIORITIES.get("voice", 2),
                    speed=self.config.arm_speed
                )
                if response and response.success:
                    logger.info("机械臂归位指令已发送")
            except Exception as e:
                logger.error(f"复位机械臂失败: {e}")
        
        # 重置本地状态为默认值
        self.arm_state = {
            "base": 0.0,
            "shoulder": 0.0,
            "elbow": 90.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 45.0,
        }
        
        logger.info("系统复位完成")
    
    def _print_status(self, chassis_vel: ChassisVelocity, arm_updates: Dict[str, float]):
        """打印状态信息 (每秒一次)"""
        current_time = time.time()
        if current_time - self.last_print_time >= 1.0:
            # 清行并打印状态
            print("\033[2K\r", end="")  # 清除当前行
            status = (
                f"[底盘] vx={chassis_vel.vx:+.2f} vy={chassis_vel.vy:+.2f} vz={chassis_vel.vz:+.2f} | "
                f"[机械臂] base={self.arm_state['base']:+.0f}° shoulder={self.arm_state['shoulder']:+.0f}° "
                f"elbow={self.arm_state['elbow']:+.0f}° wrist_flex={self.arm_state['wrist_flex']:+.0f}° "
                f"gripper={self.arm_state['gripper']:.0f}°"
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
        
        logger.info("=" * 60)
        logger.info("游戏手柄控制已启动")
        logger.info("=" * 60)
        logger.info("控制映射:")
        logger.info("  底盘: 左摇杆(移动/旋转) + LT/RT(平移)")
        logger.info("  机械臂: 右摇杆(基座/伸缩) + 十字键(升降/腕转) + Y/A/B(腕翻) + RB/LB(夹爪)")
        logger.info("  系统: Back(急停) / Start(复位)")
        logger.info("=" * 60)
        logger.info("按 Ctrl+C 停止")
        logger.info("=" * 60)
        
        # 启动手柄轮询
        self.controller.start_polling(interval=self.config.polling_interval)
        
        try:
            while self.running and not self._stop_event.is_set():
                # 读取手柄状态
                state = self.controller.get_state()
                
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
                
                # 控制循环频率
                time.sleep(self.config.polling_interval)
                
        except KeyboardInterrupt:
            logger.info("\n收到中断信号")
        except Exception as e:
            logger.error(f"运行异常: {e}")
        finally:
            self.stop()
    
    def stop(self):
        """停止应用"""
        logger.info("停止游戏手柄控制应用...")
        self.running = False
        self._stop_event.set()
        
        # 停止底盘
        if self.chassis_client:
            try:
                self.chassis_client.send_command(
                    vx=0.0, vy=0.0, vz=0.0,
                    source="gamepad",
                    priority=PRIORITIES.get("voice", 2)
                )
            except:
                pass
        
        # 停止手柄轮询
        if self.controller:
            try:
                self.controller.stop_vibration()
                self.controller.stop_polling()
            except:
                pass
        
        # 关闭客户端
        if self.chassis_client:
            try:
                self.chassis_client.close()
            except:
                pass
        
        if self.arm_client:
            try:
                self.arm_client.close()
            except:
                pass
        
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
