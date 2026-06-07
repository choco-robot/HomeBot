#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""BreezySLAM 建图仿真演示 - 实时可视化

在仿真环境中运行 BreezySLAM，实时构建栅格地图。
支持键盘手动控制机器人探索环境，同时对比显示真实地图与 SLAM 地图。

运行方式：
    cd E:/develop/HomeBot/homebot/software
    python demo_slam_sim.py

键盘控制：
    W / ↑   前进
    S / ↓   后退
    A / ←   左转
    D / →   右转
    空格     停止
    R       重置位姿
    M       保存 SLAM 地图
    Q / ESC 退出
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

import time
import math
import numpy as np
from typing import Tuple, Optional

# 导航模块
from navigation.simulation import Simulator, MapEnvironment, LaserConfig
from navigation.core.slam_fusion import SLAMFusion

# 可视化
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

print("=" * 70)
print("BreezySLAM 建图仿真演示 - 实时可视化")
print("=" * 70)

# ------------------------------------------------------------------------------
# 配置
# ------------------------------------------------------------------------------

# 外部地图配置（JSON 格式，支持 PNG 引用或障碍物列表重建）
# 示例：EXTERNAL_MAP_JSON = "maps/my_map.json"
# 如果设为 None 或文件不存在，则回退到内置地图
EXTERNAL_MAP_JSON = 'E:\\develop\\HomeBot\\homebot\\software\\maps\\map_2026-06-05T08-22-35.json'  # type: Optional[str]

# 地图类型：'maze', 'simple_room', 'cluttered'
# 仅在没有配置外部地图时生效
MAP_TYPE = "simple_room"

# 机器人起始位姿
# 建议设为 (0,0)，与 BreezySLAM 默认地图中心对齐，避免空地图时 RMHC 搜索漂移
START_POSE = (0.0, 0.0, 0.0)  # (x, y, theta_rad)

# SLAM 配置
SLAM_MAP_SIZE_PIXELS = 400  # 地图分辨率（像素）
SLAM_MAP_SIZE_METERS = 10.0  # 地图物理尺寸（米）

# 仿真参数
UPDATE_INTERVAL_MS = 50  # 可视化更新间隔（毫秒）
LINEAR_SPEED = 0.5       # 线速度（m/s）
ANGULAR_SPEED = 1.0      # 角速度（rad/s）

# 保存路径
SAVE_MAP_PATH = "slam_map.npz"

# ------------------------------------------------------------------------------
# 辅助函数
# ------------------------------------------------------------------------------


def _load_external_map(json_path: str) -> MapEnvironment:
    """加载外部地图。"""
    return MapEnvironment(map_file=json_path)


def scan_to_points(
    scan: np.ndarray, pose: Tuple[float, float, float], max_range: float = 10.0
) -> np.ndarray:
    """将激光扫描转换为世界坐标系点云。"""
    rx, ry, rtheta = pose
    points = []
    for i, dist in enumerate(scan):
        if dist >= max_range or dist < 0.1:
            continue
        angle = rtheta + (i / len(scan)) * 2 * math.pi - math.pi
        x = rx + dist * math.cos(angle)
        y = ry + dist * math.sin(angle)
        points.append([x, y])
    return np.array(points) if points else np.empty((0, 2))


# ------------------------------------------------------------------------------
# 键盘控制器
# ------------------------------------------------------------------------------

class KeyboardController:
    """基于 Matplotlib 键盘事件的机器人控制器。"""

    def __init__(self, simulator: Simulator):
        self.sim = simulator
        self.linear = 0.0
        self.angular = 0.0
        self._pressed = set()

    def on_key_press(self, event):
        key = event.key.lower() if event.key else ""
        self._pressed.add(key)
        self._update_velocity()

        if key == "r":
            print("   [CMD] 重置机器人位姿")
            self.sim.reset_robot(*START_POSE)
        elif key == "m":
            print("   [CMD] 保存地图")
            return "save_map"
        elif key == "f":
            print("   [CMD] 切换地图更新模式")
            return "toggle_map_update"
        elif key in ("q", "escape"):
            print("   [CMD] 退出")
            return "quit"
        return None

    def on_key_release(self, event):
        key = event.key.lower() if event.key else ""
        self._pressed.discard(key)
        self._update_velocity()

    def _update_velocity(self):
        """根据当前按键状态更新速度指令。"""
        v = 0.0
        w = 0.0

        if "w" in self._pressed or "up" in self._pressed:
            v += LINEAR_SPEED
        if "s" in self._pressed or "down" in self._pressed:
            v -= LINEAR_SPEED
        if "a" in self._pressed or "left" in self._pressed:
            w += ANGULAR_SPEED
        if "d" in self._pressed or "right" in self._pressed:
            w -= ANGULAR_SPEED

        self.linear = v
        self.angular = w
        self.sim.set_velocity(linear=v, angular=w)


# ------------------------------------------------------------------------------
# 初始化
# ------------------------------------------------------------------------------

# 1. 创建仿真器和地图
print("\n1. 创建仿真器和地图...")

if EXTERNAL_MAP_JSON and os.path.isfile(EXTERNAL_MAP_JSON):
    print(f"   加载外部地图: {EXTERNAL_MAP_JSON}")
    map_env = _load_external_map(EXTERNAL_MAP_JSON)
else:
    print(f"   使用内置地图: {MAP_TYPE}")
    if MAP_TYPE == "maze":
        map_env = MapEnvironment.create_maze()
    elif MAP_TYPE == "simple_room":
        map_env = MapEnvironment.create_simple_room()
    else:
        map_env = MapEnvironment.create_cluttered_room()

print(f"   [OK] 地图: {map_env.width:.1f}m x {map_env.height:.1f}m")

# 创建仿真器，调整激光配置以匹配 SLAM 扫描大小
laser_cfg = LaserConfig(num_rays=360, max_range=10.0, fov=2 * math.pi)
sim = Simulator()
sim.set_map(map_env)

# 重新初始化激光配置（Simulator 默认创建 LaserConfig()，需要覆盖）
from navigation.simulation.laser_scanner import LaserScanner
sim.laser = LaserScanner(config=laser_cfg, robot_radius=sim.robot.config.radius)

# 2. 创建 BreezySLAM 融合核心
print("\n2. 初始化 BreezySLAM...")
try:
    slam = SLAMFusion(
        map_size_pixels=SLAM_MAP_SIZE_PIXELS,
        map_size_meters=SLAM_MAP_SIZE_METERS,
        scan_size=laser_cfg.num_rays,
    )
    # 设置初始位姿（与仿真器对齐）
    slam.set_initial_pose(START_POSE[0], START_POSE[1], START_POSE[2])
    print(f"   [OK] SLAM 初始化完成: {SLAM_MAP_SIZE_PIXELS}px / {SLAM_MAP_SIZE_METERS}m")
except RuntimeError as e:
    print(f"   [FAIL] {e}")
    print("   请安装 BreezySLAM: pip install breezyslam")
    sys.exit(1)

# 3. 创建可视化窗口
print("\n3. 创建可视化窗口...")
plt.ion()

fig, axes = plt.subplots(1, 2, figsize=(14, 7))
fig.canvas.manager.set_window_title("HomeBot SLAM Simulation")

# --- 左图: Ground Truth ---
ax_gt = axes[0]
ax_gt.set_aspect("equal")
ax_gt.set_title("Ground Truth", fontsize=14, fontweight="bold")
ax_gt.set_xlabel("X (m)")
ax_gt.set_ylabel("Y (m)")

# 绘制真实地图
gt_grid = map_env.grid
gt_display = gt_grid.data.astype(np.float32).copy()
gt_display[gt_display < 0] = 0.5

extent = [
    gt_grid.origin[0],
    gt_grid.origin[0] + gt_grid.width * gt_grid.resolution,
    gt_grid.origin[1],
    gt_grid.origin[1] + gt_grid.height * gt_grid.resolution,
]
ax_gt.imshow(gt_display, cmap="gray", origin="lower", extent=extent, alpha=0.8)
ax_gt.set_xlim(extent[0], extent[1])
ax_gt.set_ylim(extent[2], extent[3])

# 真实机器人标记
gt_robot_circle = Circle((START_POSE[0], START_POSE[1]), 0.15, color="blue", fill=True, alpha=0.6)
ax_gt.add_patch(gt_robot_circle)
gt_robot_arrow = ax_gt.arrow(
    START_POSE[0], START_POSE[1], 0.3, 0, head_width=0.1, head_length=0.05, fc="red", ec="red"
)
gt_scan_scatter = ax_gt.scatter([], [], c="orange", s=5, alpha=0.5, label="Laser")
ax_gt.legend(loc="upper right")

# --- 右图: SLAM Map ---
ax_slam = axes[1]
ax_slam.set_aspect("equal")
ax_slam.set_title("BreezySLAM Map", fontsize=14, fontweight="bold")
ax_slam.set_xlabel("X (m)")
ax_slam.set_ylabel("Y (m)")

# SLAM 地图初始为空（未知区域）
slam_map_size = SLAM_MAP_SIZE_METERS
slam_origin = -slam_map_size / 2
slam_extent = [slam_origin, slam_origin + slam_map_size, slam_origin, slam_origin + slam_map_size]

# 用灰色（128）初始化显示
slam_display = np.full((SLAM_MAP_SIZE_PIXELS, SLAM_MAP_SIZE_PIXELS), 128, dtype=np.uint8)
slam_img = ax_slam.imshow(slam_display, cmap="gray", origin="lower", extent=slam_extent, vmin=0, vmax=255)
ax_slam.set_xlim(slam_extent[0], slam_extent[1])
ax_slam.set_ylim(slam_extent[2], slam_extent[3])

# SLAM 估计位姿标记
slam_robot_circle = Circle((START_POSE[0], START_POSE[1]), 0.15, color="green", fill=True, alpha=0.6)
ax_slam.add_patch(slam_robot_circle)
slam_robot_arrow = ax_slam.arrow(
    START_POSE[0], START_POSE[1], 0.3, 0, head_width=0.1, head_length=0.05, fc="cyan", ec="cyan"
)

# 信息文本
info_text = fig.text(
    0.5,
    0.02,
    "",
    ha="center",
    fontsize=10,
    bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.9),
)

# 键盘控制器
controller = KeyboardController(sim)
fig.canvas.mpl_connect("key_press_event", lambda e: _handle_key_press(e, controller))
fig.canvas.mpl_connect("key_release_event", controller.on_key_release)


def _handle_key_press(event, ctrl: KeyboardController):
    result = ctrl.on_key_press(event)
    if result == "quit":
        global _running
        _running = False
    elif result == "save_map":
        global _save_requested
        _save_requested = True
    elif result == "toggle_map_update":
        global _update_map_enabled
        _update_map_enabled = not _update_map_enabled
        print(f"   [CMD] 地图更新: {'开启' if _update_map_enabled else '关闭（纯定位模式）'}")


print("   [OK] 可视化窗口已创建")
print("   键盘控制: W/A/S/D=移动, 空格=停止, R=重置, M=保存地图, Q=退出")

# 4. 启动系统
print("\n4. 启动系统...")
sim.reset_robot(*START_POSE)
sim.start()
time.sleep(0.5)  # 等待仿真器稳定
# 获取传感器数据
scan = sim.get_laser_scan()
true_pose = sim.get_robot_pose()
odom_pose = sim.get_odom_pose()
print("   [OK] 系统已启动")

# ------------------------------------------------------------------------------
# 主循环
# ------------------------------------------------------------------------------

_running = True
_save_requested = False
_update_map_enabled = True
frame_count = 0
last_scan = None
last_odom = None

print("\n5. 开始建图仿真...")

try:
    angles_deg = (np.degrees(sim.laser.get_angles()+math.pi) % 360).tolist()
    while _running:
        # 检查窗口是否关闭
        if not plt.fignum_exists(fig.number):
            print("\n   窗口已关闭")
            break

        # 获取传感器数据
        scan = sim.get_laser_scan()
        true_pose = sim.get_robot_pose()
        odom_pose = sim.get_odom_pose()

        if scan is None or true_pose is None or odom_pose is None:
            time.sleep(0.01)
            continue

        last_scan = scan
        last_odom = odom_pose

        # --- 更新 BreezySLAM ---
        # 转换扫描数据为 BreezySLAM 格式
        # BreezySLAM 激光雷达约定：0°=-X(后方)，角度逆时针(CCW)递增
        #   0°=后方(-X), 90°=右方(-Y), 180°=前方(+X), 270°=左方(+Y)
        # Simulator 的 _angles[i] 为标准角度（0°=+X前方，CCW），局部角度从 -180° 开始
        # 映射关系：breezy = (standard + 180) % 360
        
        distances_mm = [float(d * 1000.0) for d in scan]

        # 里程计 (x, y, theta, timestamp)
        now = time.time()
        odom = (odom_pose[0], odom_pose[1], odom_pose[2], now)

        # 如果是第一次更新，先重置里程计基准
        if frame_count == 0:
            slam.reset_odom(odom)

        slam.update_lidar(angles_deg, distances_mm, odom, update_map=_update_map_enabled)

        # 获取 SLAM 位姿和地图
        slam_x, slam_y, slam_theta, slam_cov = slam.get_pose()
        mapbytes = slam.get_map_bytes()

        # --- 更新可视化 ---

        # 左图: 更新真实位姿和激光点
        gt_robot_circle.center = (true_pose[0], true_pose[1])

        # 重绘箭头
        gt_robot_arrow.remove()
        dx = 0.3 * math.cos(true_pose[2])
        dy = 0.3 * math.sin(true_pose[2])
        gt_robot_arrow = ax_gt.arrow(
            true_pose[0], true_pose[1], dx, dy,
            head_width=0.1, head_length=0.05, fc="red", ec="red"
        )

        # 更新激光点
        scan_pts = scan_to_points(scan, true_pose, max_range=laser_cfg.max_range)
        if len(scan_pts) > 0:
            gt_scan_scatter.set_offsets(scan_pts)
        else:
            gt_scan_scatter.set_offsets(np.empty((0, 2)))

        # 右图: 更新 SLAM 地图和估计位姿
        slam_map_arr = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
            (SLAM_MAP_SIZE_PIXELS, SLAM_MAP_SIZE_PIXELS)
        )
        # BreezySLAM 地图值语义：0=未知, 1~127=空闲, 128~255=障碍
        # 映射到灰度显示：未知=128(灰), 空闲=255(白), 障碍=0(黑)
        slam_display_gray = np.where(slam_map_arr == 0, 128, 255 - slam_map_arr)
        slam_img.set_array(slam_display_gray)

        # 更新 SLAM 位姿标记
        slam_robot_circle.center = (slam_x, slam_y)
        slam_robot_arrow.remove()
        sdx = 0.3 * math.cos(slam_theta)
        sdy = 0.3 * math.sin(slam_theta)
        slam_robot_arrow = ax_slam.arrow(
            slam_x, slam_y, sdx, sdy,
            head_width=0.1, head_length=0.05, fc="cyan", ec="cyan"
        )

        # 更新信息文本
        status = slam.get_status()
        elapsed = frame_count * UPDATE_INTERVAL_MS / 1000.0
        info_lines = [
            f"Time: {elapsed:.1f}s",
            f"True Pose: ({true_pose[0]:.2f}, {true_pose[1]:.2f}, {math.degrees(true_pose[2]):.1f}°)",
            f"SLAM Pose: ({slam_x:.2f}, {slam_y:.2f}, {math.degrees(slam_theta):.1f}°)",
            f"State: {status['state']}",
            f"Map Update: {'ON' if _update_map_enabled else 'OFF (Localization Only)'}",
            f"Control: v={controller.linear:.2f}m/s, ω={controller.angular:.2f}rad/s",
            "Keys: W/A/S/D=Move, Space=Stop, F=ToggleMap, R=Reset, M=Save, Q=Quit",
        ]
        info_text.set_text("\n".join(info_lines))

        # 刷新
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        # 保存地图请求
        if _save_requested:
            _save_requested = False
            try:
                slam.save_map(SAVE_MAP_PATH)
                print(f"   [OK] SLAM 地图已保存: {SAVE_MAP_PATH}")
            except Exception as e:
                print(f"   [FAIL] 保存地图失败: {e}")

        # 控制帧率
        time.sleep(UPDATE_INTERVAL_MS / 1000.0)
        frame_count += 1

except KeyboardInterrupt:
    print("\n   用户中断")
except Exception as e:
    print(f"\n   运行异常: {e}")
    import traceback
    traceback.print_exc()

# ------------------------------------------------------------------------------
# 清理
# ------------------------------------------------------------------------------
print("\n6. 清理...")
sim.stop()
plt.close(fig)
print("   [OK] 系统已停止")

print("\n" + "=" * 70)
print("演示完成！")
print("=" * 70)
