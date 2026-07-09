"""
轨迹录制与回放模块

提供机械臂遥操作动作的 JSON 序列化记录与按时间戳回放能力。
"""
import json
import threading
import time
import os
from datetime import datetime
from typing import Dict, List, Optional, TYPE_CHECKING

from common.logging import get_logger

if TYPE_CHECKING:
    from .app import ArmTeleopApp

logger = get_logger(__name__)


class TrajectoryRecorder:
    """
    轨迹录制器

    将主臂角度序列保存为带时间戳的 JSON 文件。
    """

    def __init__(self):
        self.frames: List[Dict] = []
        self.start_time: Optional[float] = None
        self.recording = False

    def start(self) -> None:
        """开始录制"""
        self.frames = []
        self.start_time = time.monotonic()
        self.recording = True
        logger.info("轨迹录制已开始")

    def record(self, angles: Dict[str, float]) -> None:
        """
        记录一帧主臂角度

        Args:
            angles: 主臂原始关节角度
        """
        if not self.recording or self.start_time is None:
            return
        t = time.monotonic() - self.start_time
        self.frames.append({
            "t": round(t, 4),
            "angles": {k: float(v) for k, v in angles.items()},
        })

    def stop(self) -> None:
        """停止录制"""
        self.recording = False
        logger.info(f"轨迹录制已停止，共 {len(self.frames)} 帧")

    def save(self, path: str) -> bool:
        """
        保存轨迹到 JSON 文件

        Args:
            path: 文件路径

        Returns:
            是否保存成功
        """
        if not self.frames:
            logger.warning("没有可保存的轨迹帧")
            return False

        try:
            os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
            data = {
                "version": 1,
                "created_at": datetime.now().isoformat(),
                "joint_names": list(self.frames[0]["angles"].keys()),
                "frames": self.frames,
            }
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            logger.info(f"轨迹已保存: {path} ({len(self.frames)} 帧)")
            return True
        except Exception as e:
            logger.error(f"保存轨迹失败: {e}")
            return False

    @staticmethod
    def load(path: str) -> List[Dict]:
        """
        从 JSON 文件加载轨迹

        Args:
            path: 文件路径

        Returns:
            帧列表，加载失败返回空列表
        """
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            frames = data.get("frames", [])
            logger.info(f"轨迹加载成功: {path} ({len(frames)} 帧)")
            return frames
        except FileNotFoundError:
            logger.error(f"轨迹文件不存在: {path}")
            return []
        except Exception as e:
            logger.error(f"加载轨迹失败: {e}")
            return []


class TrajectoryPlayer:
    """
    轨迹播放器

    按时间戳将轨迹发送到从臂，支持速度缩放与循环。
    """

    def __init__(self, app: "ArmTeleopApp"):
        self.app = app
        self._stop_event = threading.Event()

    def play(self, frames: List[Dict], speed: float = 1.0, loop: int = 1) -> None:
        """
        播放轨迹

        Args:
            frames: 轨迹帧列表
            speed: 播放速度倍数（>0）
            loop: 循环次数，0 或负数表示无限循环
        """
        if not frames:
            logger.warning("空轨迹，跳过回放")
            return

        if speed <= 0:
            logger.warning(f"回放速度 {speed} 无效，使用 1.0")
            speed = 1.0

        max_loop = loop if loop > 0 else 999999
        for i in range(max_loop):
            if self._stop_event.is_set() or not self.app._running:
                break
            if loop > 0:
                logger.info(f"开始第 {i + 1}/{loop} 次回放")
            else:
                logger.info(f"开始第 {i + 1} 次回放（无限循环）")
            self._play_once(frames, speed)

        if not self._stop_event.is_set():
            logger.info("回放结束")
            self.app._set_mode_teleop()

    def _play_once(self, frames: List[Dict], speed: float) -> None:
        """单次回放"""
        start_time = time.monotonic()
        last_angles: Optional[Dict[str, float]] = None
        last_t = 0.0

        for frame in frames:
            if self._stop_event.is_set() or not self.app._running:
                break

            angles = self.app._map_and_clamp(frame.get("angles", {}))
            if not angles:
                continue

            target_t = frame.get("t", 0.0) / speed
            desired_time = start_time + target_t

            # 等待到目标时间点
            now = time.monotonic()
            wait = desired_time - now
            if wait > 0:
                time.sleep(wait)

            # 计算与上一帧的实际/缩放时间差，用于伺服速度
            dt = target_t - last_t
            servo_speed = self._compute_playback_speed(angles, last_angles, dt)

            if self.app.client.send_joint_angles(angles, servo_speed):
                last_angles = angles
                last_t = target_t
                logger.debug(f"回放到: {angles}, speed={servo_speed}")

    def _compute_playback_speed(
        self,
        angles: Dict[str, float],
        last_angles: Optional[Dict[str, float]],
        dt: float,
    ) -> int:
        """根据帧间角度差和时间差计算伺服速度"""
        cfg = self.app.cfg
        if last_angles is None or dt <= 0 or cfg.speed_scale <= 0:
            return cfg.default_speed

        max_delta = max(
            abs(angles.get(j, 0.0) - last_angles.get(j, 0.0)) for j in angles
        )
        speed = int(max_delta / dt * cfg.speed_scale)
        return max(cfg.min_speed, min(cfg.max_speed, speed))

    def stop(self) -> None:
        """停止回放"""
        self._stop_event.set()

    def reset(self) -> None:
        """重置停止事件，允许再次播放"""
        self._stop_event.clear()


