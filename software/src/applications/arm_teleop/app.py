"""
机械臂 WLAN 主从遥操作核心应用

实现：
- 以固定频率读取本地主臂关节角度
- 通过 ZeroMQ 将角度同步到远端从臂服务
- 支持关节映射、限位、死区、速度自适应、通信超时保护
- 支持动作录制（Record）与回放（Playback）
- 支持运行时键盘热键
"""
import os
import threading
import time
from datetime import datetime
from enum import Enum, auto
from typing import Dict, Optional, Tuple

from hal.arm.driver import ArmConfig as HalArmConfig
from configs import get_config, ArmConfig as GlobalArmConfig, ArmTeleopConfig
from common.logging import get_logger

from .master_reader import MasterArmReader
from .slave_client import SlaveArmClient
from .recorder import TrajectoryRecorder, TrajectoryPlayer
from .keyboard_input import KeyboardListener

logger = get_logger(__name__)


class TeleopMode(Enum):
    """遥操作运行模式"""
    TELEOP = auto()
    RECORDING = auto()
    PLAYBACK = auto()


def build_master_arm_config(arm_cfg: GlobalArmConfig) -> HalArmConfig:
    """
    从全局机械臂配置构建 HAL 层主臂配置

    Args:
        arm_cfg: configs.config.ArmConfig 实例

    Returns:
        HAL 层 ArmConfig 实例
    """
    return HalArmConfig(
        joint_ids={
            "base": arm_cfg.base_id,
            "shoulder": arm_cfg.shoulder_id,
            "elbow": arm_cfg.elbow_id,
            "wrist_flex": arm_cfg.wrist_flex_id,
            "wrist_roll": arm_cfg.wrist_roll_id,
            "gripper": arm_cfg.gripper_id,
        },
        joint_limits=getattr(arm_cfg, "joint_limits", {
            "base": (-180, 180),
            "shoulder": (0, 180),
            "elbow": (0, 180),
            "wrist_flex": (-90, 90),
            "wrist_roll": (-180, 180),
            "gripper": (0, 90),
        }),
        default_speed=getattr(arm_cfg, "default_speed", 1000),
        default_acc=getattr(arm_cfg, "default_acc", 50),
        home_position=getattr(arm_cfg, "rest_position", {
            "base": 0,
            "shoulder": 0,
            "elbow": 90,
            "wrist_flex": 0,
            "wrist_roll": 0,
            "gripper": 45,
        }),
        port=arm_cfg.serial_port,
        baudrate=arm_cfg.baudrate,
    )


class ArmTeleopApp:
    """
    机械臂遥操作应用

    负责主臂读取、从臂发送、录制回放、安全开关和异常处理的生命周期管理。
    """

    def __init__(self, teleop_cfg: ArmTeleopConfig, master_arm_cfg: HalArmConfig):
        """
        初始化遥操作应用

        Args:
            teleop_cfg: 遥操作专属配置
            master_arm_cfg: 主臂 HAL 配置
        """
        self.cfg = teleop_cfg
        self.master_cfg = master_arm_cfg
        self.reader = MasterArmReader(master_arm_cfg, torque_off=teleop_cfg.torque_off)
        self.client = SlaveArmClient(teleop_cfg.slave_arm_addr, teleop_cfg.timeout_ms)
        self.recorder = TrajectoryRecorder()
        self.player = TrajectoryPlayer(self)
        self.keyboard = KeyboardListener(self._on_key)

        self.enabled = teleop_cfg.enabled_by_default
        self.mode = TeleopMode.TELEOP
        self._running = False
        self._read_thread: Optional[threading.Thread] = None

        self._last_sent_angles: Optional[Dict[str, float]] = None
        self._last_send_time = 0.0
        self._consecutive_failures = 0
        self._max_failures = 5

        self.read_interval = 1.0 / teleop_cfg.read_rate
        self.send_interval = 1.0 / teleop_cfg.send_rate

        self._record_file: Optional[str] = None
        self._playback_file: Optional[str] = None
        self._playback_speed = 1.0
        self._playback_loop = 1
        self._pending_playback: Optional[Tuple[str, float, int]] = None

    def initialize(self) -> bool:
        """
        初始化主臂硬件并检测从臂服务

        Returns:
            主臂初始化是否成功
        """
        logger.info("=" * 60)
        logger.info("HomeBot 机械臂 WLAN 遥操作")
        logger.info("=" * 60)

        if not self.reader.initialize():
            logger.error("主臂初始化失败，应用退出")
            return False

        logger.info(f"从臂服务地址: {self.cfg.slave_arm_addr}")
        logger.info("正在检测从臂服务连通性...")
        states = self.client.send_query()
        if states is None:
            logger.warning("无法连接从臂服务，请检查网络与从端 arm_service；应用将继续运行并尝试重连")
        else:
            logger.info(f"从臂服务连接成功，当前关节状态: {states}")

        self.keyboard.start()
        self._print_hotkeys()

        logger.info(f"遥操作状态: {'已开启' if self.enabled else '已关闭'}")
        logger.info("=" * 60)
        return True

    def _print_hotkeys(self) -> None:
        """打印热键说明"""
        logger.info("热键说明: [e] 开关遥操作  [r] 开始/停止录制  [p] 播放默认轨迹  [s] 停止回放  [q] 退出")

    def _on_key(self, key: str) -> None:
        """键盘热键回调"""
        key = key.lower()
        if key == "e":
            self.set_enabled(not self.enabled)
        elif key == "r":
            self._toggle_recording()
        elif key == "p":
            self._start_default_playback()
        elif key == "s":
            self._stop_playback()
        elif key == "q":
            logger.info("收到退出热键")
            self.stop()

    def run(self) -> None:
        """启动遥操作主循环"""
        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        # 如果 CLI 指定了回放，等读线程启动后再开始
        if self._pending_playback is not None:
            path, speed, loop = self._pending_playback
            self._pending_playback = None
            self.start_playback(path, speed, loop)

        try:
            self._main_loop()
        except KeyboardInterrupt:
            logger.info("收到用户中断信号")
        finally:
            self.stop()

    def _read_loop(self) -> None:
        """后台读取主臂角度"""
        while self._running:
            try:
                self.reader.read()
            except Exception as e:
                logger.error(f"主臂读线程异常: {e}")
            time.sleep(self.read_interval)

    def _main_loop(self) -> None:
        """主线程循环：根据模式执行遥操作或等待回放"""
        while self._running:
            if self.mode == TeleopMode.PLAYBACK:
                # 回放由独立线程执行，主线程空转等待
                time.sleep(0.05)
                continue

            if self.enabled:
                angles = self.reader.get_latest()
                if angles:
                    self._send_if_changed(angles)
            time.sleep(self.send_interval)

    def _send_if_changed(self, master_angles: Dict[str, float]) -> None:
        """
        应用映射、限位、死区后，决定是否向从臂发送

        Args:
            master_angles: 主臂当前角度
        """
        # 1. 应用关节映射（默认恒等映射）
        slave_angles: Dict[str, float] = {}
        for master_joint, (slave_joint, sign) in self.cfg.joint_mapping.items():
            if master_joint in master_angles:
                slave_angles[slave_joint] = sign * master_angles[master_joint]

        if not slave_angles:
            logger.warning("映射后无有效关节角度")
            return

        # 2. 关节限幅
        clamped: Dict[str, float] = {}
        for joint, angle in slave_angles.items():
            if joint in self.master_cfg.joint_limits:
                lo, hi = self.master_cfg.joint_limits[joint]
                clamped[joint] = max(lo, min(hi, angle))
            else:
                clamped[joint] = angle

        # 3. 死区过滤
        if self._last_sent_angles is not None:
            changed = any(
                abs(clamped.get(j, 0.0) - self._last_sent_angles.get(j, 0.0)) >= self.cfg.deadband_deg
                for j in clamped
            )
            if not changed:
                return

        # 4. 速度计算
        speed = self._compute_speed(clamped)

        # 5. 发送
        if self.client.send_joint_angles(clamped, speed):
            self._last_sent_angles = clamped
            self._last_send_time = time.time()
            self._consecutive_failures = 0
            logger.debug(f"已下发从臂目标: {clamped}, speed={speed}")

            # 6. 录制（记录主臂原始角度）
            if self.mode == TeleopMode.RECORDING:
                self.recorder.record(master_angles)
        else:
            self._consecutive_failures += 1
            if self._consecutive_failures >= self._max_failures:
                logger.error(f"连续 {self._max_failures} 次发送失败，自动关闭遥操作")
                self.set_enabled(False)

    def _compute_speed(self, angles: Dict[str, float]) -> int:
        """根据角度变化量自适应计算下发速度"""
        if self.cfg.speed_scale <= 0 or self._last_sent_angles is None:
            return self.cfg.default_speed

        max_delta = max(
            (abs(angles.get(j, 0.0) - self._last_sent_angles.get(j, 0.0)) for j in angles),
            default=0.0,
        )
        speed = int(max_delta / self.send_interval * self.cfg.speed_scale)
        return max(self.cfg.min_speed, min(self.cfg.max_speed, speed))

    def set_enabled(self, enabled: bool) -> None:
        """开启/关闭遥操作"""
        if enabled == self.enabled:
            return

        if enabled and self.mode == TeleopMode.PLAYBACK:
            logger.warning("回放中无法开启遥操作，请先停止回放")
            return

        if not enabled:
            # 关闭时让从臂保持当前位置
            hold_angles = self._last_sent_angles
            if not hold_angles:
                hold_angles = self._map_and_clamp(self.reader.get_latest())
            if hold_angles:
                if self.client.send_joint_angles(hold_angles, self.cfg.hold_speed):
                    logger.info("已发送保持位置命令")
            self._consecutive_failures = 0

        self.enabled = enabled
        logger.info(f"遥操作已{'开启' if enabled else '关闭'}")

    def _map_and_clamp(self, master_angles: Optional[Dict[str, float]]) -> Optional[Dict[str, float]]:
        """将主臂角度映射并限幅为从臂角度"""
        if not master_angles:
            return None

        slave_angles: Dict[str, float] = {}
        for master_joint, (slave_joint, sign) in self.cfg.joint_mapping.items():
            if master_joint in master_angles:
                slave_angles[slave_joint] = sign * master_angles[master_joint]

        clamped: Dict[str, float] = {}
        for joint, angle in slave_angles.items():
            if joint in self.master_cfg.joint_limits:
                lo, hi = self.master_cfg.joint_limits[joint]
                clamped[joint] = max(lo, min(hi, angle))
            else:
                clamped[joint] = angle
        return clamped

    # ==================== 录制相关 ====================

    def _toggle_recording(self) -> None:
        """切换录制状态"""
        if self.mode == TeleopMode.PLAYBACK:
            logger.warning("回放中无法录制")
            return
        if self.mode == TeleopMode.RECORDING:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self) -> None:
        """开始录制"""
        if not self.enabled:
            logger.info("录制需要先开启遥操作，自动开启")
            self.set_enabled(True)

        path = self._record_file or self._generate_record_path()
        self.recorder.start()
        self.mode = TeleopMode.RECORDING
        self._record_file = path
        logger.info(f"开始录制，保存到: {path}")

    def _stop_recording(self) -> None:
        """停止录制并保存"""
        self.recorder.stop()
        self.mode = TeleopMode.TELEOP
        if self._record_file:
            self.recorder.save(self._record_file)
            self._record_file = None

    def _generate_record_path(self) -> str:
        """生成默认录制文件路径"""
        directory = self.cfg.trajectory_dir or "trajectories"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(directory, f"teleop_{timestamp}.json")

    # ==================== 回放相关 ====================

    def _start_default_playback(self) -> None:
        """热键 'p'：播放配置中指定的默认轨迹"""
        if not self.cfg.default_playback_file:
            logger.warning("未配置 default_playback_file，无法通过热键播放")
            return
        self.start_playback(self.cfg.default_playback_file)

    def start_playback(self, path: str, speed: float = 1.0, loop: int = 1) -> None:
        """
        启动轨迹回放

        Args:
            path: 轨迹文件路径
            speed: 回放速度倍数
            loop: 循环次数，0 表示无限
        """
        frames = TrajectoryRecorder.load(path)
        if not frames:
            logger.error(f"无法加载轨迹文件: {path}")
            return

        if self.mode == TeleopMode.RECORDING:
            self._stop_recording()

        self.set_enabled(False)
        self.mode = TeleopMode.PLAYBACK
        self._playback_file = path
        self._playback_speed = speed
        self._playback_loop = loop
        self.player.reset()

        logger.info(f"开始回放: {path}, speed={speed}, loop={'∞' if loop <= 0 else loop}, 帧数={len(frames)}")
        threading.Thread(
            target=self.player.play,
            args=(frames, speed, loop),
            daemon=True,
        ).start()

    def _stop_playback(self) -> None:
        """停止回放"""
        self.player.stop()
        if self.mode == TeleopMode.PLAYBACK:
            self.mode = TeleopMode.TELEOP
            logger.info("回放已停止，回到遥操作模式")

    def _set_mode_teleop(self) -> None:
        """供 Player 回调：回到遥操作模式"""
        if self.mode == TeleopMode.PLAYBACK:
            self.mode = TeleopMode.TELEOP
            logger.info("回放结束，回到遥操作模式")

    # ==================== 生命周期 ====================

    def stop(self) -> None:
        """停止应用并释放资源"""
        if not self._running:
            return

        logger.info("正在停止遥操作应用...")
        self._running = False

        if self.mode == TeleopMode.RECORDING:
            self._stop_recording()
        self._stop_playback()

        if self._read_thread:
            self._read_thread.join(timeout=1.0)

        self.keyboard.stop()
        self.client.close()
        self.reader.close()
        logger.info("遥操作应用已停止")


def create_default_app() -> ArmTeleopApp:
    """使用全局配置创建默认遥操作应用实例"""
    config = get_config()
    master_arm_cfg = build_master_arm_config(config.arm)
    return ArmTeleopApp(config.arm_teleop, master_arm_cfg)
