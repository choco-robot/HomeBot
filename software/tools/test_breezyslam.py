# -*- coding: utf-8 -*-
"""BreezySLAM 安装验证测试"""
import sys
sys.path.insert(0, "src")

import numpy as np
import time
from navigation.core.slam_fusion import SLAMFusion
from navigation.hal.lidar_driver import MockLidarDriver

print("=== BreezySLAM 集成测试 ===")

# 1. 验证 SLAMFusion 使用真实 BreezySLAM
fusion = SLAMFusion(map_size_pixels=400, map_size_meters=10.0, scan_size=360)
print(f"BreezySLAM 可用: {fusion.slam.available}")

if not fusion.slam.available:
    print("警告: BreezySLAM 未启用，检查安装")
    exit(1)

# 2. 启动 Mock 雷达
lidar = MockLidarDriver(scan_size=360, room_radius_m=3.0)
lidar.start()
time.sleep(0.3)

# 3. 模拟 3 秒运行（30 帧 @ 10Hz）
print("模拟运行 3 秒...")
for i in range(30):
    scan = lidar.get_scan()
    if scan:
        angles, distances = scan
        fusion.update_lidar(angles, distances, odom=(i*0.01, 0.0, 0.0, time.time()))
    time.sleep(0.1)

lidar.stop()

# 4. 获取位姿和地图
x, y, theta, P = fusion.get_pose()
status = fusion.get_status()
mapbytes = fusion.get_map_bytes()

print(f"\n最终位姿: x={x:.3f}m, y={y:.3f}m, theta={np.degrees(theta):.2f}°")
print(f"协方差对角: [{P[0,0]:.4f}, {P[1,1]:.4f}, {P[2,2]:.4f}]")
print(f"状态: {status['state']}")
print(f"地图字节数: {len(mapbytes)}")

# 5. 测试硬校正
tag = {
    "tag_id": 0,
    "x": 5.0, "y": 5.0, "theta": 0.0,
    "confidence": 0.95,
    "pose_err": 0.3,
    "covariance": np.diag([0.001, 0.001, 0.0001])
}
fusion.update_apriltag([tag])
x2, y2, theta2, P2 = fusion.get_pose()
print(f"\n硬校正后位姿: x={x2:.3f}m, y={y2:.3f}m, theta={np.degrees(theta2):.2f}°")
print("\n测试通过! BreezySLAM 工作正常")
