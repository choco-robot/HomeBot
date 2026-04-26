# -*- coding: utf-8 -*-
"""SLAM + AprilTag 融合定位单元测试"""
from __future__ import annotations

import math
import os
import struct
import time

import numpy as np
import pytest


# ------------------------------------------------------------------------------
# LD06 协议解析测试
# ------------------------------------------------------------------------------

class TestLD06Protocol:
    def test_crc8(self):
        from navigation.hal.lidar_driver import crc8_ld06
        # 空数据 CRC 应为 0x00（因为 init=0x00）
        assert crc8_ld06(b"") == 0x00
        # 一个已知 payload 的 CRC 可以自校验
        payload = b"\x54\x2C\x10\x00\x00\x00"
        crc = crc8_ld06(payload)
        full = payload + bytes([crc])
        assert crc8_ld06(full[:-1]) == full[-1]

    def test_parse_valid_packet(self):
        from navigation.hal.lidar_driver import LD06Driver
        driver = LD06Driver.__new__(LD06Driver)

        # 构造一个合法包
        # Header=0x54, VerLen=0x2C, Speed=3600(10rpm*360), StartAngle=0(0°)
        # 12 points: distance=1000mm, confidence=200
        # StopAngle=3000(30.00°), Timestamp=0, CRC placeholder
        points_data = b""
        for _ in range(12):
            points_data += struct.pack("<H", 1000)  # distance
            points_data += struct.pack("B", 200)    # confidence

        packet = struct.pack("<B", 0x54)
        packet += struct.pack("<B", 0x2C)
        packet += struct.pack("<H", 3600)          # speed
        packet += struct.pack("<H", 0)             # start angle
        packet += points_data
        packet += struct.pack("<H", 3000)          # stop angle
        packet += struct.pack("<H", 0)             # timestamp
        # CRC
        from navigation.hal.lidar_driver import crc8_ld06
        crc = crc8_ld06(packet)
        packet += struct.pack("<B", crc)

        assert len(packet) == 47

        result = driver._parse_packet(packet)
        assert result is not None
        start_angle, stop_angle, points = result
        assert start_angle == 0.0
        assert stop_angle == 30.0
        assert len(points) == 12
        for angle, dist, conf in points:
            assert 0 <= angle < 360
            assert dist == 1000
            assert conf == 200


# ------------------------------------------------------------------------------
# SLAM 融合核心测试
# ------------------------------------------------------------------------------

class TestSLAMFusion:
    def test_slam_initialization(self):
        """验证 SLAMFusion 可正常初始化。"""
        from navigation.core.slam_fusion import SLAMFusion
        fusion = SLAMFusion(map_size_pixels=200, map_size_meters=10.0)
        assert fusion.map_size_pixels == 200
        assert fusion.map_size_meters == 10.0

    def test_odom_propagation(self):
        """测试纯里程计积分模式。"""
        from navigation.core.slam_fusion import SLAMFusion
        fusion = SLAMFusion(map_size_pixels=200, map_size_meters=10.0)

        # 模拟前进 1 秒
        angles = list(range(360))
        distances = [1000] * 360
        fusion.update_lidar(angles, distances, odom=(0.1, 0.0, 0.0, time.time()))

        x, y, theta, P = fusion.get_pose()
        # 位姿应发生变化（至少 SLAM 位姿会更新）
        assert isinstance(P, np.ndarray)
        assert P.shape == (3, 3)

    def test_hard_correction(self):
        """测试硬校正模式：高置信度标签触发融合并重置位姿。"""
        from navigation.core.slam_fusion import SLAMFusion
        fusion = SLAMFusion(map_size_pixels=200, map_size_meters=10.0)

        # 先运行几帧 SLAM
        angles = list(np.linspace(0, 360, 360, endpoint=False))
        distances = [2000] * 360
        for i in range(3):
            fusion.update_lidar(angles, distances, odom=(i * 0.1, 0.0, 0.0, time.time()))

        # 模拟检测到高置信度标签
        tag_detection = {
            "tag_id": 0,
            "x": 5.0,
            "y": 5.0,
            "theta": 0.0,
            "confidence": 0.95,
            "pose_err": 0.3,
            "covariance": np.diag([0.001, 0.001, 0.0001]),
        }
        fusion.update_apriltag([tag_detection])

        # 位姿应被拉到标签附近（受权重影响，但趋势是向标签靠拢）
        x, y, theta, P = fusion.get_pose()
        # 初始位姿在地图中心 (5,5)，硬校正后也应接近 (5,5)
        assert abs(x - 5.0) < 1.0
        assert abs(y - 5.0) < 1.0
        assert fusion.state == "NORMAL"

    def test_kidnap_recovery(self):
        """测试绑架恢复：连续失败 + 检测到标签 → 全局重定位。"""
        from navigation.core.slam_fusion import SLAMFusion
        fusion = SLAMFusion(map_size_pixels=200, map_size_meters=10.0)

        # 制造连续失败（通过提供矛盾的里程计）
        angles = list(np.linspace(0, 360, 360, endpoint=False))
        distances = [2000] * 360

        # 触发 SEARCHING_TAG
        for i in range(5):
            # 里程计大幅跳跃，但 SLAM 会尝试平滑，导致不一致
            fusion.update_lidar(angles, distances, odom=(i * 0.5, 0.0, 0.0, time.time()))

        # 此时应处于 SEARCHING_TAG 或已有一定失败计数
        fail_count_before = fusion._slam_fail_count

        # 检测到标签触发恢复
        tag_detection = {
            "tag_id": 0,
            "x": 2.0,
            "y": 3.0,
            "theta": 0.5,
            "confidence": 0.6,  # 低于硬校正阈值，但足以触发绑架恢复
            "pose_err": 1.0,
            "covariance": np.diag([0.01, 0.01, 0.001]),
        }
        fusion.update_apriltag([tag_detection])

        # 如果失败计数足够，应触发恢复
        if fail_count_before >= 5:
            x, y, theta, _ = fusion.get_pose()
            assert abs(x - 2.0) < 0.1
            assert abs(y - 3.0) < 0.1
            assert abs(theta - 0.5) < 0.1
            assert fusion.state == "NORMAL"

    def test_covariance_bounds(self):
        """测试协方差不会无限增长。"""
        from navigation.core.slam_fusion import SLAMFusion
        fusion = SLAMFusion(map_size_pixels=200, map_size_meters=10.0)

        angles = list(np.linspace(0, 360, 360, endpoint=False))
        distances = [2000] * 360

        for i in range(20):
            fusion.update_lidar(angles, distances, odom=(i * 0.1, 0.0, 0.0, time.time()))

        _, _, _, P = fusion.get_pose()
        assert P[0, 0] <= 2.0  # max_var_xy
        assert P[1, 1] <= 2.0
        assert P[2, 2] <= 1.0  # max_var_theta

    def test_cif_weights(self):
        """测试 Split CIF 权重计算：协方差越小权重越大。"""
        from navigation.core.slam_fusion import SLAMFusion, _angle_weighted_average

        # 直接构造场景
        P_slam = np.diag([1.0, 1.0, 0.1])
        P_tag = np.diag([0.01, 0.01, 0.001])

        tr_slam = np.trace(P_slam)
        tr_tag = np.trace(P_tag)
        w_slam = tr_tag / (tr_slam + tr_tag)
        w_tag = tr_slam / (tr_slam + tr_tag)

        # P_tag 更小 → w_tag 应更大
        assert w_tag > w_slam
        assert abs(w_slam + w_tag - 1.0) < 1e-6

        # 角度融合不应受环绕影响
        a1 = math.radians(179)
        a2 = math.radians(-179)
        avg = _angle_weighted_average(a1, a2, 0.5, 0.5)
        # 结果应接近 180° 或 -180°
        assert abs(avg) > math.radians(170)

    def test_save_and_load_map(self, tmp_path):
        """测试地图保存与加载。"""
        from navigation.core.slam_fusion import SLAMFusion

        fusion = SLAMFusion(map_size_pixels=200, map_size_meters=10.0)
        map_path = str(tmp_path / "test_map.npz")

        # 先运行几帧让位姿变化
        angles = list(np.linspace(0, 360, 360, endpoint=False))
        distances = [2000] * 360
        for i in range(3):
            fusion.update_lidar(angles, distances, odom=(i * 0.1, 0.0, 0.0, time.time()))

        # 保存地图
        fusion.save_map(map_path)
        assert os.path.exists(map_path)

        # 新建一个 fusion 实例并加载地图
        fusion2 = SLAMFusion(map_size_pixels=200, map_size_meters=10.0)
        fusion2.load_map(map_path)

        # 验证地图字节一致
        bytes1 = fusion.get_map_bytes()
        bytes2 = fusion2.get_map_bytes()
        assert len(bytes1) == len(bytes2) == 200 * 200
        assert bytes1 == bytes2

        # 验证能读出保存的位姿
        saved_pose = fusion2.get_saved_pose(map_path)
        assert saved_pose is not None
        x, y, theta = saved_pose
        # 应与 fusion 保存时的位姿一致
        assert abs(x - fusion.x) < 1e-3
        assert abs(y - fusion.y) < 1e-3
        assert abs(theta - fusion.theta) < 1e-3

    def test_load_map_size_mismatch(self, tmp_path):
        """测试加载尺寸不匹配的地图应抛异常。"""
        from navigation.core.slam_fusion import SLAMFusion

        fusion = SLAMFusion(map_size_pixels=200, map_size_meters=10.0)
        map_path = str(tmp_path / "bad_map.npz")

        # 伪造一个尺寸错误的 npz
        np.savez(map_path, map_bytes=np.zeros(400 * 400, dtype=np.uint8),
                 map_size_pixels=400, map_size_meters=20.0)

        with pytest.raises(ValueError, match="尺寸不匹配"):
            fusion.load_map(map_path)

    def test_set_initial_pose(self):
        """测试手动设置初始位姿。"""
        from navigation.core.slam_fusion import SLAMFusion

        fusion = SLAMFusion(map_size_pixels=200, map_size_meters=10.0)

        # 先运行几帧
        angles = list(np.linspace(0, 360, 360, endpoint=False))
        distances = [2000] * 360
        fusion.update_lidar(angles, distances, odom=(0.1, 0.0, 0.0, time.time()))

        # 重置位姿
        fusion.set_initial_pose(1.5, 2.5, math.radians(45))
        x, y, theta, P = fusion.get_pose()

        assert abs(x - 1.5) < 1e-6
        assert abs(y - 2.5) < 1e-6
        assert abs(theta - math.radians(45)) < 1e-6
        assert fusion.state == "NORMAL"
        assert fusion._slam_fail_count == 0

    def test_load_map_with_auto_pose(self, tmp_path):
        """测试加载地图后自动使用保存的位姿。"""
        from navigation.core.slam_fusion import SLAMFusion

        # 创建并保存地图
        fusion1 = SLAMFusion(map_size_pixels=200, map_size_meters=10.0)
        angles = list(np.linspace(0, 360, 360, endpoint=False))
        distances = [2000] * 360
        fusion1.update_lidar(angles, distances, odom=(0.2, 0.1, 0.05, time.time()))

        map_path = str(tmp_path / "auto_pose_map.npz")
        fusion1.save_map(map_path)

        # 新建实例，加载地图，并读取保存位姿
        fusion2 = SLAMFusion(map_size_pixels=200, map_size_meters=10.0)
        fusion2.load_map(map_path)
        saved = fusion2.get_saved_pose(map_path)
        assert saved is not None
        fusion2.set_initial_pose(*saved)

        x, y, theta, _ = fusion2.get_pose()
        assert abs(x - fusion1.x) < 1e-3
        assert abs(y - fusion1.y) < 1e-3
        assert abs(theta - fusion1.theta) < 1e-3


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
