# -*- coding: utf-8 -*-
"""坐标变换工具 - 基于齐次变换矩阵和 scipy.spatial.transform

提供二维/三维位姿的齐次变换矩阵构建、求逆、点变换、坐标系转换等通用功能。

坐标系约定（已统一为底盘坐标系）：
  - 世界坐标系：X 向右，Y 向上（俯视图中），yaw 从 X 轴正方向逆时针旋转
  - 机器人坐标系（底盘驱动 / VFH）：x 前进为正，y 左侧为正，yaw 逆时针

主要场景：
  - 世界坐标系 ↔ 机器人坐标系（底盘驱动约定）
  - 世界坐标系 ↔ VFH 坐标系（已与底盘坐标系统一）
  - 二维位姿 (x, y, yaw) 的矩阵表示与运算
  - 三维位姿 (x, y, z, roll, pitch, yaw) 的矩阵表示与运算

依赖：
  - numpy
  - scipy.spatial.transform

Usage:
    from common.transform import (
        pose2_to_matrix, matrix_to_pose2,
        transform_point2, transform_pose2,
        world_to_robot2, robot_to_world2,
        world_to_vfh2, vfh_to_world2,
    )

    # 机器人位姿（世界坐标系）
    robot_pose = (1.0, 0.5, 0.78)  # x, y, yaw(rad)

    # 世界坐标系中的目标点
    target_world = (3.0, 2.0)

    # 转换到机器人坐标系（底盘约定：x=前, y=左）
    target_robot = world_to_robot2(target_world, robot_pose)

    # 转换到 VFH 坐标系（x=前, y=左，已与底盘坐标系统一）
    target_vfh = world_to_vfh2(target_world, robot_pose)
"""
from __future__ import annotations

import math
from typing import Sequence, Tuple, Union

import numpy as np
from scipy.spatial.transform import Rotation

# ------------------------------------------------------------------------------
# 类型别名
# ------------------------------------------------------------------------------
Point2 = Union[Sequence[float], np.ndarray]
Pose2 = Union[Sequence[float], np.ndarray]  # (x, y, yaw)  yaw 单位：弧度
Point3 = Union[Sequence[float], np.ndarray]
Pose3 = Union[Sequence[float], np.ndarray]  # (x, y, z, roll, pitch, yaw)  弧度
Matrix4 = np.ndarray  # 4x4 齐次变换矩阵


# ------------------------------------------------------------------------------
# 二维变换（x, y, yaw）
# ------------------------------------------------------------------------------

def pose2_to_matrix(pose: Pose2) -> Matrix4:
    """将二维位姿 (x, y, yaw) 转为 4x4 齐次变换矩阵。

    变换矩阵将**机器人坐标系**下的点映射到**世界坐标系**：
        p_world = T @ p_robot

    Args:
        pose: (x, y, yaw) 单位：米、米、弧度

    Returns:
        4x4 齐次变换矩阵 T
    """
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.array([
        [c, -s, 0.0, x],
        [s,  c, 0.0, y],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)
    return T


def matrix_to_pose2(T: Matrix4) -> Tuple[float, float, float]:
    """从 4x4 齐次变换矩阵提取二维位姿 (x, y, yaw)。

    Args:
        T: 4x4 齐次变换矩阵

    Returns:
        (x, y, yaw) 单位：米、米、弧度，yaw 范围 [-pi, pi]
    """
    T = np.asarray(T, dtype=np.float64)
    x = float(T[0, 3])
    y = float(T[1, 3])
    yaw = math.atan2(float(T[1, 0]), float(T[0, 0]))
    return x, y, yaw


def inverse_matrix(T: Matrix4) -> Matrix4:
    """求齐次变换矩阵的逆。

    对于齐次变换矩阵 T = [R | t; 0 | 1]，有：
        T^{-1} = [R^T | -R^T @ t; 0 | 1]

    Args:
        T: 4x4 齐次变换矩阵

    Returns:
        T 的逆矩阵
    """
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3:4]
    R_inv = R.T
    t_inv = -R_inv @ t
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3:4] = t_inv
    return T_inv


def transform_point2(T: Matrix4, point: Point2) -> Tuple[float, float]:
    """用齐次变换矩阵变换二维点。

    Args:
        T: 4x4 齐次变换矩阵
        point: (x, y) 或 [x, y]

    Returns:
        变换后的 (x, y)
    """
    T = np.asarray(T, dtype=np.float64)
    p = np.array([float(point[0]), float(point[1]), 0.0, 1.0], dtype=np.float64)
    p_out = T @ p
    return float(p_out[0]), float(p_out[1])


def transform_pose2(T: Matrix4, pose: Pose2) -> Tuple[float, float, float]:
    """用齐次变换矩阵变换二维位姿。

    Args:
        T: 4x4 齐次变换矩阵
        pose: (x, y, yaw) 单位：米、米、弧度

    Returns:
        变换后的 (x, y, yaw)
    """
    T = np.asarray(T, dtype=np.float64)
    p = np.array([float(pose[0]), float(pose[1]), 0.0, 1.0], dtype=np.float64)
    p_out = T @ p
    # 旋转部分：提取原始 yaw，叠加 T 的旋转
    yaw_in = float(pose[2])
    yaw_T = math.atan2(float(T[1, 0]), float(T[0, 0]))
    yaw_out = yaw_T + yaw_in
    yaw_out = math.atan2(math.sin(yaw_out), math.cos(yaw_out))
    return float(p_out[0]), float(p_out[1]), yaw_out


# ------------------------------------------------------------------------------
# 世界坐标系 ↔ 机器人坐标系（底盘驱动约定：x=前进, y=左侧）
# ------------------------------------------------------------------------------

def world_to_robot2(point_world: Point2, robot_pose: Pose2) -> Tuple[float, float]:
    """将世界坐标系下的点转换到机器人坐标系（底盘约定）。

    Args:
        point_world: 世界坐标系下的点 (x, y)
        robot_pose: 机器人在世界坐标系下的位姿 (x, y, yaw)

    Returns:
        机器人坐标系下的点 (x_robot, y_robot)
        - x_robot: 前进方向为正
        - y_robot: 左侧方向为正
    """
    T_world_to_robot = inverse_matrix(pose2_to_matrix(robot_pose))
    return transform_point2(T_world_to_robot, point_world)


def robot_to_world2(point_robot: Point2, robot_pose: Pose2) -> Tuple[float, float]:
    """将机器人坐标系下的点（底盘约定）转换到世界坐标系。

    Args:
        point_robot: 机器人坐标系下的点 (x, y)
        robot_pose: 机器人在世界坐标系下的位姿 (x, y, yaw)

    Returns:
        世界坐标系下的点 (x_world, y_world)
    """
    T_robot_to_world = pose2_to_matrix(robot_pose)
    return transform_point2(T_robot_to_world, point_robot)


def world_to_robot_pose2(pose_world: Pose2, robot_pose: Pose2) -> Tuple[float, float, float]:
    """将世界坐标系下的位姿转换到机器人坐标系（底盘约定）。

    Args:
        pose_world: 世界坐标系下的位姿 (x, y, yaw)
        robot_pose: 机器人在世界坐标系下的位姿 (x, y, yaw)

    Returns:
        机器人坐标系下的位姿 (x, y, yaw)
    """
    T_world_to_robot = inverse_matrix(pose2_to_matrix(robot_pose))
    return transform_pose2(T_world_to_robot, pose_world)


def robot_to_world_pose2(pose_robot: Pose2, robot_pose: Pose2) -> Tuple[float, float, float]:
    """将机器人坐标系下的位姿（底盘约定）转换到世界坐标系。

    Args:
        pose_robot: 机器人坐标系下的位姿 (x, y, yaw)
        robot_pose: 机器人在世界坐标系下的位姿 (x, y, yaw)

    Returns:
        世界坐标系下的位姿 (x, y, yaw)
    """
    T_robot_to_world = pose2_to_matrix(robot_pose)
    return transform_pose2(T_robot_to_world, pose_robot)


# ------------------------------------------------------------------------------
# 世界坐标系 ↔ VFH 坐标系（VFH 已与底盘坐标系统一：x=前进, y=左侧）
# ------------------------------------------------------------------------------

def robot_to_vfh2(point_robot: Point2) -> Tuple[float, float]:
    """将底盘坐标系下的点转换到 VFH 坐标系。

    注：VFH 坐标系现已与底盘坐标系统一，此函数为恒等映射，保留以兼容旧代码。

    Args:
        point_robot: 底盘坐标系下的点 (x, y)

    Returns:
        VFH 坐标系下的点 (x_vfh, y_vfh)，与输入相同
    """
    x_r, y_r = float(point_robot[0]), float(point_robot[1])
    return x_r, y_r


def vfh_to_robot2(point_vfh: Point2) -> Tuple[float, float]:
    """将 VFH 坐标系下的点转换到底盘坐标系。

    注：VFH 坐标系现已与底盘坐标系统一，此函数为恒等映射，保留以兼容旧代码。

    Args:
        point_vfh: VFH 坐标系下的点 (x, y)

    Returns:
        底盘坐标系下的点 (x_robot, y_robot)，与输入相同
    """
    x_v, y_v = float(point_vfh[0]), float(point_vfh[1])
    return x_v, y_v


def world_to_vfh2(point_world: Point2, robot_pose: Pose2) -> Tuple[float, float]:
    """将世界坐标系下的点直接转换到 VFH 坐标系。

    Args:
        point_world: 世界坐标系下的点 (x, y)
        robot_pose: 机器人在世界坐标系下的位姿 (x, y, yaw)

    Returns:
        VFH 坐标系下的点 (x_vfh, y_vfh)
        - x_vfh: 前方为正
        - y_vfh: 左侧为正
    """
    return world_to_robot2(point_world, robot_pose)


def vfh_to_world2(point_vfh: Point2, robot_pose: Pose2) -> Tuple[float, float]:
    """将 VFH 坐标系下的点转换到世界坐标系。

    Args:
        point_vfh: VFH 坐标系下的点 (x, y)
        robot_pose: 机器人在世界坐标系下的位姿 (x, y, yaw)

    Returns:
        世界坐标系下的点 (x_world, y_world)
    """
    return robot_to_world2(point_vfh, robot_pose)


# ------------------------------------------------------------------------------
# 三维变换（x, y, z, roll, pitch, yaw）
# ------------------------------------------------------------------------------

def pose3_to_matrix(pose: Pose3) -> Matrix4:
    """将三维位姿 (x, y, z, roll, pitch, yaw) 转为 4x4 齐次变换矩阵。

    旋转顺序：ZYX（yaw-pitch-roll，即先 roll 绕 X，再 pitch 绕 Y，最后 yaw 绕 Z）
    这与 scipy Rotation.from_euler('ZYX', [yaw, pitch, roll]) 一致。

    Args:
        pose: (x, y, z, roll, pitch, yaw) 单位：米、弧度

    Returns:
        4x4 齐次变换矩阵 T
    """
    x, y, z = float(pose[0]), float(pose[1]), float(pose[2])
    roll, pitch, yaw = float(pose[3]), float(pose[4]), float(pose[5])
    R = Rotation.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def matrix_to_pose3(T: Matrix4) -> Tuple[float, float, float, float, float, float]:
    """从 4x4 齐次变换矩阵提取三维位姿 (x, y, z, roll, pitch, yaw)。

    旋转顺序：ZYX（yaw-pitch-roll）

    Args:
        T: 4x4 齐次变换矩阵

    Returns:
        (x, y, z, roll, pitch, yaw) 单位：米、弧度
    """
    T = np.asarray(T, dtype=np.float64)
    x, y, z = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
    R = T[:3, :3]
    yaw, pitch, roll = Rotation.from_matrix(R).as_euler('ZYX')
    return x, y, z, roll, pitch, yaw


def transform_point3(T: Matrix4, point: Point3) -> Tuple[float, float, float]:
    """用齐次变换矩阵变换三维点。

    Args:
        T: 4x4 齐次变换矩阵
        point: (x, y, z) 或 [x, y, z]

    Returns:
        变换后的 (x, y, z)
    """
    T = np.asarray(T, dtype=np.float64)
    p = np.array([float(point[0]), float(point[1]), float(point[2]), 1.0], dtype=np.float64)
    p_out = T @ p
    return float(p_out[0]), float(p_out[1]), float(p_out[2])


def transform_pose3(T: Matrix4, pose: Pose3) -> Tuple[float, float, float, float, float, float]:
    """用齐次变换矩阵变换三维位姿。

    Args:
        T: 4x4 齐次变换矩阵
        pose: (x, y, z, roll, pitch, yaw)

    Returns:
        变换后的 (x, y, z, roll, pitch, yaw)
    """
    T = np.asarray(T, dtype=np.float64)
    p = np.array([float(pose[0]), float(pose[1]), float(pose[2]), 1.0], dtype=np.float64)
    p_out = T @ p

    # 旋转部分：T 的旋转与 pose 的旋转复合
    R_T = T[:3, :3]
    R_pose = Rotation.from_euler('ZYX', [float(pose[5]), float(pose[4]), float(pose[3])]).as_matrix()
    R_out = R_T @ R_pose
    yaw, pitch, roll = Rotation.from_matrix(R_out).as_euler('ZYX')
    return float(p_out[0]), float(p_out[1]), float(p_out[2]), roll, pitch, yaw


# ------------------------------------------------------------------------------
# 便捷构造
# ------------------------------------------------------------------------------

def translation_matrix(x: float, y: float, z: float = 0.0) -> Matrix4:
    """构造纯平移齐次变换矩阵。"""
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = [x, y, z]
    return T


def rotation_matrix_z(yaw: float) -> Matrix4:
    """构造绕 Z 轴旋转的齐次变换矩阵。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([
        [c, -s, 0.0, 0.0],
        [s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)


def rotation_matrix_from_euler(roll: float, pitch: float, yaw: float, seq: str = 'ZYX') -> Matrix4:
    """构造欧拉角旋转的齐次变换矩阵。

    Args:
        roll, pitch, yaw: 弧度
        seq: 旋转顺序，默认 'ZYX'

    Returns:
        4x4 齐次变换矩阵（纯旋转，无平移）
    """
    R = Rotation.from_euler(seq, [yaw, pitch, roll]).as_matrix()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    return T


# ------------------------------------------------------------------------------
# 验证与测试
# ------------------------------------------------------------------------------

def _test() -> None:
    """内部自测"""
    print("=" * 60)
    print("common.transform 自测")
    print("=" * 60)

    # --- 二维测试（底盘坐标系：x=前, y=左） ---
    print("\n--- 二维测试（底盘坐标系） ---")
    robot = (1.0, 2.0, math.radians(90))  # 在 (1,2)，朝北
    target_world = (1.0, 5.0)  # 正前方 3 米

    # 世界 → 机器人（底盘）
    tr = world_to_robot2(target_world, robot)
    print(f"世界 {target_world} → 机器人(底盘): {tr}")
    assert abs(tr[0] - 3.0) < 1e-6, "x 应为 3（前进 3 米）"
    assert abs(tr[1] - 0.0) < 1e-6, "y 应为 0（左移 0 米）"

    # 机器人 → 世界
    tw = robot_to_world2(tr, robot)
    print(f"机器人(底盘) {tr} → 世界: {tw}")
    assert abs(tw[0] - target_world[0]) < 1e-6
    assert abs(tw[1] - target_world[1]) < 1e-6

    # 世界 → VFH
    tv = world_to_vfh2(target_world, robot)
    print(f"世界 {target_world} → VFH: {tv}")
    assert abs(tv[0] - 3.0) < 1e-6, "vfh_x 应为 3（前方 3 米）"
    assert abs(tv[1] - 0.0) < 1e-6, "vfh_y 应为 0（左侧 0）"

    # VFH → 世界
    tw2 = vfh_to_world2(tv, robot)
    print(f"VFH {tv} → 世界: {tw2}")
    assert abs(tw2[0] - target_world[0]) < 1e-6
    assert abs(tw2[1] - target_world[1]) < 1e-6

    # 位姿变换
    pose_world = (3.0, 4.0, 0.0)
    pose_robot = world_to_robot_pose2(pose_world, robot)
    print(f"世界位姿 {pose_world} → 机器人位姿: {pose_robot}")
    pose_back = robot_to_world_pose2(pose_robot, robot)
    print(f"机器人位姿 {pose_robot} → 世界位姿: {pose_back}")
    assert abs(pose_back[0] - pose_world[0]) < 1e-6
    assert abs(pose_back[1] - pose_world[1]) < 1e-6
    assert abs(pose_back[2] - pose_world[2]) < 1e-6

    # 齐次矩阵求逆验证
    T = pose2_to_matrix(robot)
    T_inv = inverse_matrix(T)
    T_identity = T_inv @ T
    print(f"\nT_inv @ T ≈ I: {np.allclose(T_identity, np.eye(4))}")
    assert np.allclose(T_identity, np.eye(4))

    # --- 三维测试 ---
    print("\n--- 三维测试 ---")
    pose3 = (1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    T3 = pose3_to_matrix(pose3)
    pose3_back = matrix_to_pose3(T3)
    print(f"原始 3D 位姿: {pose3}")
    print(f"矩阵还原后:   {pose3_back}")
    assert np.allclose(np.array(pose3), np.array(pose3_back))

    p3_world = (10.0, 0.0, 0.0)
    p3_robot = transform_point3(inverse_matrix(T3), p3_world)
    print(f"世界点 {p3_world} → 机器人坐标系: {p3_robot}")

    print("\n" + "=" * 60)
    print("所有测试通过 (ok)")
    print("=" * 60)


if __name__ == "__main__":
    _test()
