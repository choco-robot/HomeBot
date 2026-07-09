"""
主臂角度读取模块

直接通过 HAL ArmDriver 读取本地 SO101 机械臂关节角度，支持关闭扭矩以方便手动拖动。
"""
import threading
import time
from typing import Dict

from hal.arm.driver import ArmDriver, ArmConfig as HalArmConfig
from common.logging import get_logger

logger = get_logger(__name__)


class MasterArmReader:
    """
    主臂读取器

    封装本地机械臂 HAL 驱动，提供线程安全的最新角度读取。
    """

    def __init__(self, arm_config: HalArmConfig, torque_off: bool = True):
        """
        初始化主臂读取器

        Args:
            arm_config: HAL 层机械臂配置（关节ID、限位、串口等）
            torque_off: 是否关闭主臂扭矩以便手动拖动，默认 True
        """
        self.arm = ArmDriver(arm_config)
        self.torque_off = torque_off
        self._lock = threading.Lock()
        self._latest_angles: Dict[str, float] = {}
        self._initialized = False

    def initialize(self) -> bool:
        """
        初始化主臂硬件

        Returns:
            是否初始化成功
        """
        logger.info("正在初始化主臂...")
        if not self.arm.initialize(auto_home=False, enable_torque=False):
            logger.error("主臂初始化失败，请检查串口和舵机连接")
            return False

        if self.torque_off:
            logger.info("关闭主臂扭矩，允许手动拖动")
            disabled: list = []
            failed: list = []
            for name, servo_id in self.arm.config.joint_ids.items():
                if self.arm.bus.torque_disable(servo_id):
                    disabled.append(name)
                else:
                    failed.append(name)
                time.sleep(0.02)
            if failed:
                logger.warning(f"以下关节扭矩关闭失败: {failed}")
            logger.info(f"已关闭扭矩的关节: {disabled}")
        else:
            logger.info("保持主臂扭矩使能")
            for servo_id in self.arm.config.joint_ids.values():
                self.arm.bus.torque_enable(servo_id)
                time.sleep(0.02)

        angles = self.arm.get_all_joint_angles()
        with self._lock:
            self._latest_angles = angles

        self._initialized = True
        logger.info(f"主臂初始化完成，当前角度: {angles}")
        return True

    def read(self) -> Dict[str, float]:
        """
        从主臂舵机读取最新角度

        Returns:
            关节角度字典
        """
        if not self._initialized:
            return {}

        try:
            angles = self.arm.get_all_joint_angles()
            with self._lock:
                self._latest_angles = angles
            return angles
        except Exception as e:
            logger.warning(f"读取主臂角度失败: {e}")
            return self.get_latest()

    def get_latest(self) -> Dict[str, float]:
        """
        获取最近一次成功读取的角度

        Returns:
            关节角度字典副本
        """
        with self._lock:
            return dict(self._latest_angles)

    def close(self) -> None:
        """
        关闭主臂读取

        注意：不调用 ArmDriver.close()，避免触发主臂自动归位动作。
        """
        if not self._initialized:
            return

        logger.info("正在关闭主臂读取...")
        try:
            # 恢复扭矩，避免主臂在断开前因重力下垂
            if self.torque_off:
                for name, servo_id in self.arm.config.joint_ids.items():
                    if not self.arm.bus.torque_enable(servo_id):
                        logger.warning(f"恢复关节 {name}(ID={servo_id}) 扭矩失败")
                    time.sleep(0.02)
        except Exception as e:
            logger.warning(f"恢复主臂扭矩失败: {e}")

        try:
            self.arm.bus.disconnect()
        except Exception as e:
            logger.warning(f"断开主臂串口失败: {e}")

        self._initialized = False
        logger.info("主臂读取已关闭")
