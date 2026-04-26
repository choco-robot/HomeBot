# -*- coding: utf-8 -*-
"""SLAM + AprilTag 融合定位核心

实现 BreezySLAM 与 AprilTag 视觉定位的协方差交叉融合，
支持硬校正、软融合、绑架恢复三种模式。

核心算法：
- 高频 SLAM 线程 (10Hz): BreezySLAM 输出相对位姿和协方差 P_slam
- 低频视觉线程 (2Hz): AprilTag PnP 解算绝对位姿和协方差 P_tag
- Split CIF 融合: x_fused = w1*x_slam + w2*x_abs, w ∝ tr(P)^-1
- 绑架恢复: 连续匹配失败 + 检测到标签 → 全局重定位

依赖 BreezySLAM，未安装时初始化将直接失败。
"""
from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from common.logging import get_logger

logger = get_logger(__name__)

# ------------------------------------------------------------------------------
# 常量
# ------------------------------------------------------------------------------

# 卡方分布 3自由度 99.9% 阈值，用于里程计一致性检验
CHI2_3DOF_999 = 16.27

# 连续匹配失败阈值（帧数），触发绑架恢复
KIDNAP_FAIL_FRAMES = 5

# SLAM 协方差传播的过程噪声系数
Q_SCALE_XY = 0.01       # 位移 1% 相对噪声
Q_BASE_XY = 0.001       # 1mm 基础噪声
Q_SCALE_THETA = 0.05    # 角度 5% 相对噪声
Q_BASE_THETA = 0.005    # ~0.3° 基础噪声

# ------------------------------------------------------------------------------
# BreezySLAM 封装
# ------------------------------------------------------------------------------

class BreezySLAMWrapper:
    """封装 BreezySLAM，统一接口并暴露位姿设置能力。"""

    def __init__(self, scan_size: int, map_size_pixels: int, map_size_meters: float):
        try:
            from breezyslam.algorithms import RMHC_SLAM
            from breezyslam.sensors import Laser
        except ImportError as e:
            raise RuntimeError(f"BreezySLAM 未安装，无法初始化 SLAM: {e}") from e

        self.laser = Laser(
            scan_size=scan_size,
            scan_rate_hz=10,
            detection_angle_degrees=360,
            distance_no_detection_mm=12000,
            detection_margin=0,
            offset_mm=0,
        )
        self._slam = RMHC_SLAM(
            self.laser,
            map_size_pixels,
            map_size_meters,
            random_seed=42,
            map_quality=100,
            hole_width_mm=120,
        )
        self._map_size_meters = map_size_meters

    def update(self, scans_mm: List[float], pose_change: Tuple[float, float, float], scan_angles_degrees: Optional[List[float]] = None):
        """更新 SLAM。

        Args:
            scans_mm: 扫描距离列表（毫米）
            pose_change: (dxy_mm, dtheta_degrees, dt_seconds)
            scan_angles_degrees: 可选的角度列表
        """
        self._slam.update(scans_mm, pose_change, scan_angles_degrees)

    def getpos(self) -> Tuple[float, float, float]:
        """返回当前位姿 (x_mm, y_mm, theta_degrees)。"""
        return self._slam.getpos()

    def setpos(self, x_mm: float, y_mm: float, theta_degrees: float) -> None:
        """直接设置 SLAM 位姿（用于硬校正 / 绑架恢复）。"""
        import pybreezyslam
        self._slam.position = pybreezyslam.Position(x_mm, y_mm, theta_degrees)

    def getmap(self, mapbytes: bytearray) -> None:
        self._slam.getmap(mapbytes)

    def setmap(self, mapbytes: bytearray) -> None:
        if hasattr(self._slam, 'setmap'):
            self._slam.setmap(mapbytes)
        else:
            logger.warning("底层 SLAM 不支持 setmap，地图加载被忽略")


# ------------------------------------------------------------------------------
# SLAM + AprilTag 融合核心
# ------------------------------------------------------------------------------

class SLAMFusion:
    """SLAM + AprilTag 融合定位核心。

    状态:
        NORMAL:              正常运行
        SEARCHING_TAG:       里程计不一致，主动寻标
        KIDNAPPED_RECOVERING: 绑架恢复中（全局重定位）
    """

    def __init__(
        self,
        map_size_pixels: int = 800,
        map_size_meters: float = 20.0,
        scan_size: int = 360,
        confidence_threshold: float = 0.8,
        odom_consistency_threshold: float = 9.21,  # chi2(3, 0.99)
    ):
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.scan_size = scan_size
        self.confidence_threshold = confidence_threshold
        self.odom_consistency_threshold = odom_consistency_threshold

        # BreezySLAM 封装
        self.slam = BreezySLAMWrapper(scan_size, map_size_pixels, map_size_meters)

        # 融合位姿（世界坐标，米）
        self.x = map_size_meters / 2.0
        self.y = map_size_meters / 2.0
        self.theta = 0.0

        # 协方差矩阵 (3x3)
        self.P = np.diag([0.01, 0.01, 0.001])

        # 状态机
        self.state = "NORMAL"
        self._slam_fail_count = 0

        # 里程计缓存（用于一致性检验和 pose_change 计算）
        self._last_odom_xyt: Optional[Tuple[float, float, float]] = None
        self._last_odom_time: Optional[float] = None

        # 用于 SLAM update 的里程计积分（毫米/度）
        self._odom_accum_dxy_mm = 0.0
        self._odom_accum_dtheta_deg = 0.0
        self._odom_accum_dt = 0.0

        # 地图中心偏移（BreezySLAM 位姿以地图左上角为原点）
        self._map_center_mm = 500 * map_size_meters

        logger.info(
            f"SLAMFusion 初始化: map={map_size_pixels}x{map_size_pixels} "
            f"({map_size_meters}m)"
        )

    # ------------------------------------------------------------------
    # 高频 Lidar 更新 (10Hz)
    # ------------------------------------------------------------------
    def update_lidar(
        self,
        angles_deg: List[float],
        distances_mm: List[float],
        odom: Optional[Tuple[float, float, float, float]] = None,
    ) -> None:
        """传入一圈激光扫描数据和最新里程计，更新 SLAM。

        Args:
            angles_deg: 扫描角度列表（度）
            distances_mm: 扫描距离列表（毫米）
            odom: 可选的当前里程计 (x_m, y_m, theta_rad, timestamp_s)
        """
        # 1. 计算 pose_change for BreezySLAM
        pose_change = self._compute_pose_change(odom)

        # 2. BreezySLAM 更新
        self.slam.update(distances_mm, pose_change, angles_deg)

        # 3. 获取 SLAM 相对位姿
        x_mm, y_mm, theta_deg = self.slam.getpos()
        x_slam = (x_mm - self._map_center_mm) / 1000.0
        y_slam = (y_mm - self._map_center_mm) / 1000.0
        theta_slam = math.radians(theta_deg)

        # 4. 里程计一致性检验
        slam_delta = np.array([x_slam - self.x, y_slam - self.y, _normalize_angle(theta_slam - self.theta)])
        odom_delta = self._get_odom_delta_since_last()
        consistent = self._check_consistency(slam_delta, odom_delta)

        if not consistent:
            self._slam_fail_count += 1
            if self.state != "KIDNAPPED_RECOVERING" and self._slam_fail_count >= 3:
                self.state = "SEARCHING_TAG"
                logger.warning(
                    f"SLAM 里程计不一致，进入 SEARCHING_TAG 状态 "
                    f"(连续失败={self._slam_fail_count})"
                )
        else:
            if self._slam_fail_count > 0:
                self._slam_fail_count = max(0, self._slam_fail_count - 1)
            if self.state == "SEARCHING_TAG" and self._slam_fail_count == 0:
                self.state = "NORMAL"

        # 5. 软融合模式：未检测到标签时，纯依赖 SLAM 位姿
        if self.state in ("NORMAL", "SEARCHING_TAG"):
            # 传播协方差
            self._propagate_covariance(odom_delta)
            # 更新位姿为 SLAM 输出
            self.x, self.y, self.theta = x_slam, y_slam, theta_slam

    # ------------------------------------------------------------------
    # 低频视觉更新 (2Hz)
    # ------------------------------------------------------------------
    def update_apriltag(self, detections: List[Dict]) -> None:
        """传入 AprilTag 检测结果，执行融合校正。

        Args:
            detections: AprilTagDetector.detect() 返回的列表
        """
        if not detections:
            return

        # 选取置信度最高的标签
        best = max(detections, key=lambda d: d["confidence"])
        conf = best["confidence"]

        # 硬校正模式：高置信度标签 → Split CIF 融合并重置 SLAM
        if conf > self.confidence_threshold:
            self._hard_correction(best)
            return

        # 绑架恢复检测：连续 SLAM 失败但检测到标签
        if self._slam_fail_count >= KIDNAP_FAIL_FRAMES:
            self._kidnap_recovery(best)

    def _hard_correction(self, detection: Dict) -> None:
        """硬校正：Split CIF 融合 + 重置 SLAM 粒子群。"""
        x_abs = detection["x"]
        y_abs = detection["y"]
        theta_abs = detection["theta"]
        P_tag = detection["covariance"]

        x_slam, y_slam, theta_slam = self.x, self.y, self.theta
        P_slam = self.P

        # Split CIF 权重
        tr_slam = float(np.trace(P_slam))
        tr_tag = float(np.trace(P_tag))
        # 避免除零
        tr_slam = max(tr_slam, 1e-6)
        tr_tag = max(tr_tag, 1e-6)

        w_slam = tr_tag / (tr_slam + tr_tag)
        w_tag = tr_slam / (tr_slam + tr_tag)

        # 融合位姿（角度需要特殊处理）
        x_f = w_slam * x_slam + w_tag * x_abs
        y_f = w_slam * y_slam + w_tag * y_abs
        theta_f = _angle_weighted_average(theta_slam, theta_abs, w_slam, w_tag)

        # 融合协方差（信息融合公式）
        try:
            P_inv_slam = np.linalg.inv(P_slam)
            P_inv_tag = np.linalg.inv(P_tag)
            P_f = np.linalg.inv(w_slam * P_inv_slam + w_tag * P_inv_tag)
        except np.linalg.LinAlgError:
            # 矩阵奇异时退化为加权平均
            P_f = w_slam ** 2 * P_slam + w_tag ** 2 * P_tag

        # 更新内部状态
        self.x, self.y, self.theta = x_f, y_f, theta_f
        self.P = P_f
        self._slam_fail_count = 0
        self.state = "NORMAL"

        # 重置 SLAM 粒子群到校正后位姿
        x_mm = x_f * 1000.0 + self._map_center_mm
        y_mm = y_f * 1000.0 + self._map_center_mm
        theta_deg = math.degrees(theta_f)
        self.slam.setpos(x_mm, y_mm, theta_deg)

        logger.info(
            f"硬校正完成: pos=({x_f:.3f}, {y_f:.3f}, {math.degrees(theta_f):.2f}°), "
            f"w_slam={w_slam:.3f}, w_tag={w_tag:.3f}"
        )

    def _kidnap_recovery(self, detection: Dict) -> None:
        """绑架恢复：以标签位姿全局重定位。"""
        x_abs = detection["x"]
        y_abs = detection["y"]
        theta_abs = detection["theta"]
        P_tag = detection["covariance"]

        self.state = "KIDNAPPED_RECOVERING"
        logger.warning(
            f"触发绑架恢复! 全局重定位到 ({x_abs:.3f}, {y_abs:.3f}, "
            f"{math.degrees(theta_abs):.2f}°)"
        )

        # 直接以标签位姿作为当前位姿
        self.x, self.y, self.theta = x_abs, y_abs, theta_abs
        self.P = P_tag.copy()
        self._slam_fail_count = 0

        # 重置 SLAM 到该位姿
        x_mm = x_abs * 1000.0 + self._map_center_mm
        y_mm = y_abs * 1000.0 + self._map_center_mm
        theta_deg = math.degrees(theta_abs)
        self.slam.setpos(x_mm, y_mm, theta_deg)

        # 状态恢复
        self.state = "NORMAL"
        logger.info("绑架恢复完成，状态恢复为 NORMAL")

    # ------------------------------------------------------------------
    # 状态查询
    # ------------------------------------------------------------------
    def get_pose(self) -> Tuple[float, float, float, np.ndarray]:
        """返回融合后的位姿和协方差。

        Returns:
            (x_m, y_m, theta_rad, P_3x3)
        """
        return self.x, self.y, self.theta, self.P.copy()

    def get_map_bytes(self) -> bytearray:
        """获取当前栅格地图字节数组。"""
        mapbytes = bytearray(self.map_size_pixels * self.map_size_pixels)
        self.slam.getmap(mapbytes)
        return mapbytes

    def get_status(self) -> dict:
        """返回当前状态字典。"""
        return {
            "state": self.state,
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "theta": round(self.theta, 4),
            "covariance": self.P.tolist(),
            "slam_fail_count": self._slam_fail_count,
        }

    # ------------------------------------------------------------------
    # 内部工具
    # ------------------------------------------------------------------
    def _compute_pose_change(
        self, odom: Optional[Tuple[float, float, float, float]]
    ) -> Tuple[float, float, float]:
        """从里程计计算 BreezySLAM 所需的 pose_change (dxy_mm, dtheta_deg, dt_s)。"""
        if odom is None or self._last_odom_xyt is None:
            # 无里程计时，使用内部累积值
            dxy_mm = self._odom_accum_dxy_mm
            dtheta_deg = self._odom_accum_dtheta_deg
            dt = self._odom_accum_dt
            self._odom_accum_dxy_mm = 0.0
            self._odom_accum_dtheta_deg = 0.0
            self._odom_accum_dt = 0.0
            return dxy_mm, dtheta_deg, max(dt, 0.01)

        x, y, theta, ts = odom
        lx, ly, ltheta = self._last_odom_xyt

        dt = ts - self._last_odom_time if self._last_odom_time else 0.1
        dt = max(dt, 1e-6)

        # 沿机器人朝向的位移分量
        dx = x - lx
        dy = y - ly
        mid_theta = ltheta + (theta - ltheta) * 0.5
        dxy = dx * math.cos(mid_theta) + dy * math.sin(mid_theta)

        dtheta = _normalize_angle(theta - ltheta)

        self._last_odom_xyt = (x, y, theta)
        self._last_odom_time = ts

        return dxy * 1000.0, math.degrees(dtheta), dt

    def _get_odom_delta_since_last(self) -> np.ndarray:
        """返回从上帧以来的里程计变化量 [dx, dy, dtheta]。"""
        if self._last_odom_xyt is None:
            return np.zeros(3)
        # 这里简化处理：如果没有新里程计，返回零
        # 实际应由调用方维护上帧里程计
        return np.zeros(3)

    def _check_consistency(self, slam_delta: np.ndarray, odom_delta: np.ndarray) -> bool:
        """检验 SLAM 位姿变化与里程计积分是否一致。"""
        diff = slam_delta - odom_delta
        # 角度差规范化
        diff[2] = _normalize_angle(diff[2])

        # Mahalanobis 距离
        try:
            P_inv = np.linalg.inv(self.P + np.eye(3) * 1e-4)
            d2 = float(diff.T @ P_inv @ diff)
        except np.linalg.LinAlgError:
            d2 = float(diff.T @ diff) * 1000.0

        return d2 < self.odom_consistency_threshold

    def _propagate_covariance(self, odom_delta: np.ndarray) -> None:
        """基于里程计运动模型传播协方差。"""
        dx, dy, dtheta = odom_delta
        dxy = math.hypot(dx, dy)
        theta = self.theta

        # 状态转移 Jacobian
        F = np.array([
            [1, 0, -dxy * math.sin(theta + dtheta / 2)],
            [0, 1,  dxy * math.cos(theta + dtheta / 2)],
            [0, 0,  1],
        ])

        # 过程噪声
        sigma_xy = Q_SCALE_XY * abs(dxy) + Q_BASE_XY
        sigma_theta = Q_SCALE_THETA * abs(dtheta) + Q_BASE_THETA
        Q = np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_theta ** 2])

        self.P = F @ self.P @ F.T + Q

        # 限制协方差上界，防止无界增长
        max_var_xy = 2.0  # m^2
        max_var_theta = 1.0  # rad^2
        self.P[0, 0] = min(self.P[0, 0], max_var_xy)
        self.P[1, 1] = min(self.P[1, 1], max_var_xy)
        self.P[2, 2] = min(self.P[2, 2], max_var_theta)

    def reset_odom(self, odom: Tuple[float, float, float, float]) -> None:
        """重置里程计基准。"""
        x, y, theta, ts = odom
        self._last_odom_xyt = (x, y, theta)
        self._last_odom_time = ts

    # ------------------------------------------------------------------
    # 地图持久化
    # ------------------------------------------------------------------
    def save_map(self, path: str) -> None:
        """将当前栅格地图和位姿保存为 .npz 文件。

        Args:
            path: 保存路径，如 "maps/home_map.npz"
        """
        mapbytes = self.get_map_bytes()
        np_bytes = np.frombuffer(mapbytes, dtype=np.uint8)
        np.savez(
            path,
            map_bytes=np_bytes,
            map_size_pixels=self.map_size_pixels,
            map_size_meters=self.map_size_meters,
            pose_x=self.x,
            pose_y=self.y,
            pose_theta=self.theta,
            timestamp=time.time(),
        )
        logger.info(f"地图已保存: {path} (pose=({self.x:.3f}, {self.y:.3f}, {math.degrees(self.theta):.2f}°))")

    def load_map(self, path: str) -> None:
        """从 .npz 文件加载栅格地图。

        加载后会自动将地图写入底层 SLAM，但不会改变当前位姿。
        如需同时恢复位姿，请在 load_map 后调用 set_initial_pose()。

        Args:
            path: 地图文件路径
        """
        if not os.path.exists(path):
            raise FileNotFoundError(f"地图文件不存在: {path}")

        data = np.load(path)
        loaded_pixels = int(data["map_size_pixels"])
        loaded_meters = float(data["map_size_meters"])

        if loaded_pixels != self.map_size_pixels or abs(loaded_meters - self.map_size_meters) > 1e-6:
            raise ValueError(
                f"地图尺寸不匹配: 文件={loaded_pixels}px/{loaded_meters}m, "
                f"当前={self.map_size_pixels}px/{self.map_size_meters}m"
            )

        np_bytes = data["map_bytes"].astype(np.uint8)
        mapbytes = bytearray(np_bytes.tobytes())
        self.slam.setmap(mapbytes)

        logger.info(f"地图已加载: {path} ({loaded_pixels}px/{loaded_meters}m)")

    def get_saved_pose(self, path: str) -> Optional[Tuple[float, float, float]]:
        """读取地图文件中保存的位姿（若存在）。

        Returns:
            (x, y, theta) 或 None
        """
        if not os.path.exists(path):
            return None
        try:
            data = np.load(path)
            if "pose_x" in data and "pose_y" in data and "pose_theta" in data:
                return float(data["pose_x"]), float(data["pose_y"]), float(data["pose_theta"])
        except Exception as e:
            logger.warning(f"读取地图保存位姿失败: {e}")
        return None

    # ------------------------------------------------------------------
    # 初始位姿设置
    # ------------------------------------------------------------------
    def set_initial_pose(self, x: float, y: float, theta: float) -> None:
        """设置机器人初始位姿，同步更新融合状态和底层 SLAM 粒子群。

        Args:
            x: 世界坐标 X (m)
            y: 世界坐标 Y (m)
            theta: 朝向 (rad)
        """
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.P = np.diag([0.01, 0.01, 0.001])
        self._slam_fail_count = 0
        self.state = "NORMAL"

        # 重置 SLAM 底层位姿（BreezySLAM 以地图左上角为原点，单位 mm/deg）
        x_mm = self.x * 1000.0 + self._map_center_mm
        y_mm = self.y * 1000.0 + self._map_center_mm
        theta_deg = math.degrees(self.theta)
        self.slam.setpos(x_mm, y_mm, theta_deg)

        logger.info(
            f"初始位姿已设置: ({self.x:.3f}, {self.y:.3f}, {math.degrees(self.theta):.2f}°)"
        )

    def reset_pose(self, x: float, y: float, theta: float) -> None:
        """重置 SLAM 融合位姿（硬校正）。"""
        self.x = x
        self.y = y
        self.theta = theta
        self.P = np.diag([0.01, 0.01, 0.001])
        self._slam_fail_count = 0
        self.state = "NORMAL"
        # 同步重置 BreezySLAM 内部位姿
        x_mm = x * 1000.0 + self._map_center_mm
        y_mm = y * 1000.0 + self._map_center_mm
        theta_deg = math.degrees(theta)
        self.slam.setpos(x_mm, y_mm, theta_deg)
        logger.info(f"SLAM 位姿已重置: ({x:.3f}, {y:.3f}, {theta:.3f})")


# ------------------------------------------------------------------------------
# 工具函数
# ------------------------------------------------------------------------------

def _normalize_angle(angle: float) -> float:
    """规范化角度到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def _angle_weighted_average(a1: float, a2: float, w1: float, w2: float) -> float:
    """两个角度的加权平均（处理环绕）。"""
    # 使用复数平均避免角度环绕问题
    z1 = complex(math.cos(a1), math.sin(a1))
    z2 = complex(math.cos(a2), math.sin(a2))
    z = w1 * z1 + w2 * z2
    return math.atan2(z.imag, z.real)
