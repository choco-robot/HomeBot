# -*- coding: utf-8 -*-
r"""AprilTag 视觉定位检测器

基于 pupil-apriltags + OpenCV solvePnP 实现 tag36h11 族标签检测，
参考 E:\develop\AprilTagTracker 已验证的检测 pipeline。

核心流程（与 AprilTagTracker 一致）：
    1. pupil_apriltags.Detector 检测标签角点（不调内置 PnP）
    2. OpenCV solvePnP(ITERATIVE/EPNP/IPPE_SQUARE) 解算相机到标签位姿
    3. 已知标签世界坐标，推导相机绝对位姿
    4. 输出协方差与置信度

支持真实检测和 Mock 模拟模式。
"""
from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from common.logging import get_logger

logger = get_logger(__name__)

# ------------------------------------------------------------------------------
# 工具函数
# ------------------------------------------------------------------------------

def _normalize_angle(angle: float) -> float:
    """将角度规范化到 [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))


def _rotation_matrix_to_euler_zyx(R: np.ndarray) -> Tuple[float, float, float]:
    """从 3x3 旋转矩阵提取 ZYX Euler 角 (roll, pitch, yaw)。"""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


# ------------------------------------------------------------------------------
# 真实 AprilTag 检测器
# ------------------------------------------------------------------------------

class AprilTagDetector:
    """AprilTag 检测与视觉定位（AprilTagTracker 兼容版本）。"""

    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: Optional[np.ndarray] = None,
        tag_size_m: float = 0.165,
        tag_map: Optional[Dict[int, Tuple[float, float, float]]] = None,
        camera_to_robot_tf: Optional[np.ndarray] = None,
        confidence_threshold: float = 0.8,
    ):
        """
        Args:
            camera_matrix: 3x3 相机内参矩阵
            dist_coeffs: 畸变系数，None 时假设无畸变
            tag_size_m: 标签物理边长（米）
            tag_map: 标签世界位姿映射 {tag_id: (x_m, y_m, theta_rad)}
            camera_to_robot_tf: 4x4 相机→机器人外参，None 时重合
            confidence_threshold: 硬校正触发阈值
        """
        self.camera_matrix = camera_matrix.astype(np.float64)
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros((4, 1), dtype=np.float64)
        self.tag_size_m = tag_size_m
        self.tag_map = tag_map or {}
        self.confidence_threshold = confidence_threshold

        # 相机→机器人外参
        self.T_robot_cam = camera_to_robot_tf if camera_to_robot_tf is not None else np.eye(4)
        self.T_cam_robot = np.linalg.inv(self.T_robot_cam)

        # 标签 3D 坐标（以标签中心为原点，Z 轴垂直标签向外）
        s = tag_size_m / 2.0
        self.obj_points = np.array([
            [-s, -s, 0],
            [s, -s, 0],
            [s, s, 0],
            [-s, s, 0],
        ], dtype=np.float64)

        # 初始化 pupil-apriltags（参数与 AprilTagTracker 一致）
        try:
            from pupil_apriltags import Detector
            self._detector = Detector(
                families="tag36h11",
                nthreads=4,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0,
            )
            logger.info("AprilTagDetector 初始化完成 (tag36h11, 参数对齐 AprilTagTracker)")
        except Exception as e:
            logger.error(f"pupil_apriltags 初始化失败: {e}")
            raise

        # 历史位姿缓存（用于 PnP 初始值）
        self._pose_history: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}

        # 统计
        self._detect_count = 0
        self._avg_time_ms = 0.0
        self._last_log_time = time.time()

    def detect(self, frame: np.ndarray) -> List[Dict]:
        """检测图像中的 AprilTag，返回所有有效定位结果。

        Returns:
            列表，每个元素为字典:
            {
                "tag_id": int,
                "x": float,          # 机器人世界坐标 X (m)
                "y": float,          # 机器人世界坐标 Y (m)
                "theta": float,      # 机器人世界航向角 (rad)
                "confidence": float, # 置信度 [0, 1]
                "pose_err": float,   # 重投影误差（pixel）
                "covariance": ndarray(3x3),
                "rvec": ndarray,     # 旋转向量（相机→标签）
                "tvec": ndarray,     # 平移向量（相机→标签）
                "corners": ndarray,  # 图像角点 (4,2)
            }
        """
        t0 = time.perf_counter()

        if frame is None or frame.size == 0:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Step 1: 检测标签角点（与 AprilTagTracker 一致，不调用内置 PnP）
        raw_detections = self._detector.detect(gray)

        # 调试：输出原始检测数量
        if raw_detections:
            logger.debug(f"pupil_apriltags 原始检测: {len(raw_detections)} 个标签 "
                        f"({[d.tag_id for d in raw_detections]})")

        # 如果 tag_map 为空，检测所有标签（测试/探索模式）
        # 如果 tag_map 非空，只过滤已知 ID（SLAM 融合模式）
        has_tag_map = bool(self.tag_map)

        results = []
        for det in raw_detections:
            tag_id = det.tag_id
            if has_tag_map and tag_id not in self.tag_map:
                continue

            corners = det.corners.astype(np.float64)
            confidence = min(1.0, max(0.0, det.decision_margin / 100.0))

            # Step 2: OpenCV solvePnP（多算法尝试，与 AprilTagTracker 一致）
            rvec, tvec, success, reproj_err = self._solve_pnp(corners, tag_id)
            if not success:
                logger.debug(f"Tag {tag_id}: solvePnP 失败")
                continue

            # Step 3: 计算世界位姿（仅当 tag_map 已知时）
            if has_tag_map:
                x_r, y_r, theta_r = self._compute_world_pose(rvec, tvec, tag_id)
            else:
                # tag_map 为空时，返回相机坐标系下的相对位姿
                # 机器人在标签坐标系中的位置（简化）
                x_r = y_r = theta_r = 0.0

            # Step 4: 协方差估计
            distance = float(np.linalg.norm(tvec))
            covariance = self._estimate_covariance(distance, reproj_err)

            # 缓存位姿（供下一帧 PnP 初始值使用）
            self._pose_history[tag_id] = (rvec.copy(), tvec.copy())

            results.append({
                "tag_id": tag_id,
                "x": x_r,
                "y": y_r,
                "theta": theta_r,
                "confidence": confidence,
                "pose_err": reproj_err,
                "covariance": covariance,
                "rvec": rvec,
                "tvec": tvec,
                "corners": corners,
            })

        if results:
            logger.debug(f"最终返回检测结果: {len(results)} 个标签")
        self._update_stats((time.perf_counter() - t0) * 1000)
        return results

    # ------------------------------------------------------------------
    # PnP 解算（与 AprilTagTracker.utils.calculate_pose 一致）
    # ------------------------------------------------------------------
    def _solve_pnp(
        self, img_points: np.ndarray, tag_id: int
    ) -> Tuple[np.ndarray, np.ndarray, bool, float]:
        """使用多种算法求解 PnP，返回 (rvec, tvec, success, reprojection_error)。"""
        rvec_init, tvec_init = self._pose_history.get(tag_id, (None, None))

        algorithms = [
            (cv2.SOLVEPNP_ITERATIVE, "ITERATIVE"),
            (cv2.SOLVEPNP_EPNP, "EPNP"),
            (cv2.SOLVEPNP_IPPE_SQUARE, "IPPE_SQUARE"),
        ]

        best_rvec, best_tvec = None, None
        best_err = float("inf")

        for flags, name in algorithms:
            try:
                if rvec_init is not None and tvec_init is not None and flags == cv2.SOLVEPNP_ITERATIVE:
                    success, rvec, tvec = cv2.solvePnP(
                        self.obj_points, img_points, self.camera_matrix, self.dist_coeffs,
                        rvec_init, tvec_init, True, flags=flags
                    )
                else:
                    success, rvec, tvec = cv2.solvePnP(
                        self.obj_points, img_points, self.camera_matrix, self.dist_coeffs,
                        flags=flags
                    )

                if success and self._is_pose_valid(rvec, tvec):
                    # 计算重投影误差
                    proj, _ = cv2.projectPoints(
                        self.obj_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
                    )
                    err = float(np.mean(np.abs(proj.reshape(-1, 2) - img_points)))

                    if err < best_err:
                        best_err = err
                        best_rvec, best_tvec = rvec, tvec

                    # 如果有初始值且成功，优先使用（与 AprilTagTracker 一致）
                    if rvec_init is not None:
                        break
            except Exception:
                continue

        if best_rvec is None:
            # 全部失败，返回上一帧位姿（如果有）
            if rvec_init is not None:
                return rvec_init, tvec_init, True, float("inf")
            return np.zeros((3, 1)), np.zeros((3, 1)), False, float("inf")

        return best_rvec, best_tvec, True, best_err

    def _is_pose_valid(self, rvec: np.ndarray, tvec: np.ndarray) -> bool:
        """检查位姿是否在合理范围（AprilTagTracker 兼容）。"""
        t_norm = np.linalg.norm(tvec)
        if t_norm > 10.0 or t_norm < 0.01:
            return False
        if np.linalg.norm(rvec) > np.pi:
            return False
        return True

    # ------------------------------------------------------------------
    # 世界坐标系位姿计算
    # ------------------------------------------------------------------
    def _compute_world_pose(
        self, rvec: np.ndarray, tvec: np.ndarray, tag_id: int
    ) -> Tuple[float, float, float]:
        """从相机→标签位姿，推导机器人在世界坐标系中的 2D 位姿。"""
        # 标签世界位姿
        x_tag, y_tag, theta_tag = self.tag_map[tag_id]

        # R_ct: 标签→相机
        R_ct, _ = cv2.Rodrigues(rvec)
        t_ct = tvec.flatten()

        # 相机→标签 = 逆
        R_tc = R_ct.T
        t_tc = -R_tc @ t_ct

        # 标签→世界
        c = math.cos(theta_tag)
        s = math.sin(theta_tag)
        R_wt = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        t_wt = np.array([x_tag, y_tag, 0.0])

        # 相机在世界坐标系中
        R_wc = R_wt @ R_tc
        t_wc = t_wt + R_wt @ t_tc

        # 应用相机-机器人外参
        T_wc = np.eye(4)
        T_wc[:3, :3] = R_wc
        T_wc[:3, 3] = t_wc
        T_wr = T_wc @ self.T_cam_robot

        _, _, yaw = _rotation_matrix_to_euler_zyx(T_wr[:3, :3])
        return float(T_wr[0, 3]), float(T_wr[1, 3]), _normalize_angle(yaw)

    # ------------------------------------------------------------------
    # 协方差估计
    # ------------------------------------------------------------------
    def _estimate_covariance(self, distance: float, pose_err: float) -> np.ndarray:
        """估计视觉定位协方差。"""
        base_xy = 0.005 + 0.02 * distance
        base_theta = 0.02 + 0.05 * distance
        err_factor = 1.0 + max(0, pose_err - 1.0)
        sigma_xy = base_xy * err_factor
        sigma_theta = base_theta * err_factor
        return np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_theta ** 2])

    def _update_stats(self, elapsed_ms: float) -> None:
        alpha = 0.2
        self._avg_time_ms = (1 - alpha) * self._avg_time_ms + alpha * elapsed_ms
        self._detect_count += 1
        now = time.time()
        if now - self._last_log_time >= 5.0:
            logger.info(
                f"AprilTag 检测性能: 平均耗时={self._avg_time_ms:.1f}ms, "
                f"累计帧数={self._detect_count}"
            )
            self._last_log_time = now


# ------------------------------------------------------------------------------
# Mock 模拟检测器
# ------------------------------------------------------------------------------

class MockAprilTagDetector:
    """模拟 AprilTag 检测器，用于无相机/无标签环境测试。"""

    def __init__(
        self,
        tag_map: Optional[Dict[int, Tuple[float, float, float]]] = None,
        fov_deg: float = 75.0,
        max_range_m: float = 3.0,
        confidence_threshold: float = 0.8,
    ):
        self.tag_map = tag_map or {}
        self.fov_rad = math.radians(fov_deg)
        self.max_range_m = max_range_m
        self.confidence_threshold = confidence_threshold
        self._robot_pose: Optional[Tuple[float, float, float]] = None

    def set_robot_pose(self, x: float, y: float, theta: float) -> None:
        self._robot_pose = (x, y, theta)

    def detect(self, frame: Optional[np.ndarray] = None) -> List[Dict]:
        if self._robot_pose is None:
            return []

        rx, ry, rtheta = self._robot_pose
        results = []

        for tag_id, (tx, ty, ttheta) in self.tag_map.items():
            dx = tx - rx
            dy = ty - ry
            dist = math.hypot(dx, dy)
            if dist > self.max_range_m or dist < 0.1:
                continue

            bearing = math.atan2(dy, dx)
            rel_angle = _normalize_angle(bearing - rtheta)
            if abs(rel_angle) > self.fov_rad / 2:
                continue

            confidence = 1.0 - (abs(rel_angle) / (self.fov_rad / 2)) * 0.3
            confidence -= (dist / self.max_range_m) * 0.2
            confidence = min(1.0, max(0.0, confidence))

            noise_xy = 0.02 * dist
            noise_theta = 0.03 * dist
            x_r = rx + np.random.normal(0, noise_xy)
            y_r = ry + np.random.normal(0, noise_xy)
            theta_r = _normalize_angle(rtheta + np.random.normal(0, noise_theta))

            sigma_xy = 0.01 + 0.015 * dist
            P = np.diag([sigma_xy ** 2, sigma_xy ** 2, (0.02 + 0.03 * dist) ** 2])

            results.append({
                "tag_id": tag_id,
                "x": float(x_r),
                "y": float(y_r),
                "theta": float(theta_r),
                "confidence": float(confidence),
                "pose_err": float(0.5 + dist * 0.3),
                "covariance": P,
                "rvec": np.zeros((3, 1)),
                "tvec": np.zeros((3, 1)),
                "corners": None,
            })

        return results


# ------------------------------------------------------------------------------
# 工厂函数
# ------------------------------------------------------------------------------

def create_apriltag_detector(
    camera_matrix: Optional[np.ndarray] = None,
    tag_map: Optional[Dict[int, Tuple[float, float, float]]] = None,
    tag_size_m: float = 0.165,
    mock: bool = False,
) -> object:
    """创建 AprilTag 检测器。"""
    if mock:
        return MockAprilTagDetector(tag_map=tag_map)

    if camera_matrix is None:
        camera_matrix = np.array([
            [600.0, 0.0, 320.0],
            [0.0, 600.0, 240.0],
            [0.0, 0.0, 1.0],
        ])
        logger.warning("AprilTag 使用默认相机内参，请根据实际标定结果替换！")

    return AprilTagDetector(
        camera_matrix=camera_matrix,
        tag_map=tag_map,
        tag_size_m=tag_size_m,
    )
