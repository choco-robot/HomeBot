#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""导航仿真演示 - 实时可视化

运行导航演示并实时显示GUI窗口，观察机器人运动过程。

运行方式：
    cd E:/develop/homeBOT/homebot/software
    python demo_realtime.py
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

import time
import math
import json
import numpy as np
from typing import List, Tuple, Optional
from PIL import Image

# 导航模块
from navigation.simulation import Simulator, MapEnvironment
from navigation.coordinator import NavigationCoordinator, NavigationState
from navigation.perception.obstacle_detector import DepthObstacle
from navigation.core.occupancy_grid import COST_LETHAL

# 可视化 - 使用GUI后端
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

print("=" * 70)
print("导航仿真演示 - 实时可视化")
print("=" * 70)

# ------------------------------------------------------------------------------
# 配置
# ------------------------------------------------------------------------------

# 外部地图配置（JSON 格式，支持 PNG 引用或障碍物列表重建）
# 示例：EXTERNAL_MAP_JSON = "maps/my_map.json"
# 如果设为 None 或文件不存在，则回退到内置地图
EXTERNAL_MAP_JSON = 'map_2026-06-04T14-45-34.json'  # type: Optional[str]

# 地图类型：'maze', 'simple_room', 'cluttered'
# 仅在没有配置外部地图时生效
MAP_TYPE = "cluttered"

# 起点和终点（默认，当外部地图未包含 markers 时使用）
DEFAULT_START = (-2.5, -1.5)
DEFAULT_GOAL = (2.0, 1.0)

# 仿真参数
MAX_DURATION = 9999.0  # 最大运行时长（秒）
UPDATE_INTERVAL = 100  # 更新间隔（毫秒）

# 避障参数调整
OBSTACLE_EMERGENCY_DISTANCE = 0.2

# 是否保存仿真过程为 GIF
SAVE_GIF = False

# ------------------------------------------------------------------------------
# 辅助函数
# ------------------------------------------------------------------------------


def _load_external_map_and_goals(json_path: str) -> Tuple[MapEnvironment, Tuple[float, float], Tuple[float, float]]:
    """加载外部地图，并尝试从 markers 中提取起点/终点。

    Returns:
        (map_env, start, goal)
    """
    map_env = MapEnvironment(map_file=json_path)

    start = DEFAULT_START
    goal = DEFAULT_GOAL

    try:
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        data = {}

    markers = data.get("markers", [])
    for m in markers:
        mtype = m.get("type", "").lower()
        mx = m.get("x")
        my = m.get("y")
        if mx is None or my is None:
            continue
        if mtype == "start":
            start = (float(mx), float(my))
        elif mtype == "goal":
            goal = (float(mx), float(my))

    return map_env, start, goal


def scan_to_obstacles(
    scan: np.ndarray, pose: Tuple[float, float, float], max_range: float = 10.0
) -> List[DepthObstacle]:
    """将激光扫描转换为障碍物列表"""
    obstacles = []
    rx, ry, rtheta = pose

    step = max(1, len(scan) // 72)

    for i in range(0, len(scan), step):
        dist = scan[i]
        if dist >= max_range or dist < 0.5:
            continue

        local_angle = (i / len(scan)) * 2 * math.pi - math.pi
        cam_x = dist
        cam_y = -dist * math.sin(local_angle)

        obstacles.append(
            DepthObstacle(
                x=cam_y, y=0.0, z=cam_x, width=0.1, height=0.1, confidence=0.8
            )
        )

    return obstacles


# ------------------------------------------------------------------------------
# 主程序
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# 初始化地图、起点、终点
# ------------------------------------------------------------------------------

START = DEFAULT_START
GOAL = DEFAULT_GOAL

if EXTERNAL_MAP_JSON and os.path.isfile(EXTERNAL_MAP_JSON):
    print(f"\n配置:")
    print(f"  外部地图: {EXTERNAL_MAP_JSON}")
    print(f"  最大时长: {MAX_DURATION}秒")

    print("\n1. 创建仿真器（外部地图）...")
    map_env, START, GOAL = _load_external_map_and_goals(EXTERNAL_MAP_JSON)
    print(f"   [OK] 地图: {map_env.width:.1f}m x {map_env.height:.1f}m")
    print(f"   [OK] 起点: {START}, 终点: {GOAL}")
else:
    if EXTERNAL_MAP_JSON:
        print(f"   [WARN] 外部地图不存在: {EXTERNAL_MAP_JSON}，回退到内置地图")

    print(f"\n配置:")
    print(f"  地图类型: {MAP_TYPE}")
    print(f"  最大时长: {MAX_DURATION}秒")

    print("\n1. 创建仿真器...")
    if MAP_TYPE == "maze":
        map_env = MapEnvironment.create_maze()
        START = (-3.5, -3.5)
        GOAL = (-0.5, 2.0)
    elif MAP_TYPE == "simple_room":
        map_env = MapEnvironment.create_simple_room()
    else:
        map_env = MapEnvironment.create_cluttered_room()
    print(f"   [OK] 地图: {map_env.width:.1f}m x {map_env.height:.1f}m")

sim = Simulator()
sim.set_map(map_env)

# 2. 创建协调器
print("\n2. 创建协调器...")
coordinator = NavigationCoordinator(
    {
        "goal_reached_distance": 0.7,
        "control_frequency": 10.0,
        "max_replan_attempts": 8,
        "obstacle_emergency_distance": OBSTACLE_EMERGENCY_DISTANCE,
        "replan_distance_threshold": 0.8,
        "inflation_radius": 0.2,  # 障碍物膨胀半径（米）
        "robot_radius": 0.2,  # 机器人半径（米）
    }
)

# 3. 连接接口
print("\n3. 连接接口...")
coordinator.set_pose_provider(sim.get_odom_pose)
coordinator.set_map_provider(sim.get_map)
coordinator.set_velocity_sender(sim.set_velocity)

# 启用障碍物检测：将激光扫描转换为 DepthObstacle 列表
def get_obstacles():
    scan = sim.get_laser_scan()
    pose = sim.get_odom_pose()
    if scan is not None and pose is not None:
        return scan_to_obstacles(scan, pose)
    return []

coordinator.set_obstacle_provider(get_obstacles)  # 启用障碍物检测
print("   [OK] 接口已连接")

# 4. 创建可视化窗口
print("\n4. 创建可视化窗口...")
plt.ion()  # 开启交互模式

fig, ax = plt.subplots(figsize=(10, 8))
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)
ax.set_xlabel("X (m)", fontsize=12)
ax.set_ylabel("Y (m)", fontsize=12)
ax.set_title(
    "HomeBot Navigation - Real-time Visualization", fontsize=14, fontweight="bold"
)

# 绘制地图
grid = sim.get_map()
display_data = grid.data.astype(np.float32).copy()
display_data[display_data < 0] = 0.5

extent = [
    grid.origin[0],
    grid.origin[0] + grid.width * grid.resolution,
    grid.origin[1],
    grid.origin[1] + grid.height * grid.resolution,
]

ax.imshow(display_data, cmap="gray", origin="lower", extent=extent, alpha=0.8)

# 标记起点和终点
ax.plot(
    *START,
    "go",
    markersize=15,
    label="Start",
    markeredgecolor="darkgreen",
    markeredgewidth=2,
)
ax.plot(
    *GOAL,
    "r*",
    markersize=20,
    label="Goal",
    markeredgecolor="darkred",
    markeredgewidth=2,
)
ax.legend(loc="upper right", fontsize=12)

# 若使用外部地图，根据地图边界自适应视图
ax.set_xlim(grid.origin[0], grid.origin[0] + grid.width * grid.resolution)
ax.set_ylim(grid.origin[1], grid.origin[1] + grid.height * grid.resolution)

# 创建动态元素
robot_circle = plt.Circle(START, 0.15, color="blue", fill=True, alpha=0.6)
ax.add_patch(robot_circle)

robot_arrow = ax.arrow(
    START[0],
    START[1],
    0.3,
    0,
    head_width=0.1,
    head_length=0.05,
    fc="red",
    ec="red",
    linewidth=2,
)

(path_line,) = ax.plot([], [], "g-", linewidth=2, alpha=0.7, label="Path")
scan_scatter = ax.scatter([], [], c="orange", s=10, alpha=0.5, label="Laser Scan")

info_text = ax.text(
    0.02,
    0.98,
    "",
    transform=ax.transAxes,
    fontsize=10,
    verticalalignment="top",
    bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
)

print("   [OK] 可视化窗口已创建")

# 5. 启动系统
print("\n5. 启动系统...")
sim.start()
sim.reset_robot(x=START[0], y=START[1], theta=0.0)
coordinator.start()
print("   [OK] 系统已启动")

# 等待稳定
time.sleep(1.0)

# 6. 发送导航目标
print(f"\n6. 发送导航目标: {GOAL}")
goal_id = coordinator.navigate_to_async(x=GOAL[0], y=GOAL[1], timeout=MAX_DURATION)
print(f"   目标ID: {goal_id}")

# 7. 实时更新循环
print("\n7. 开始实时可视化...")
print("   按Ctrl+C或关闭窗口停止")

start_time = time.time()
last_state = None
frame_count = 0
navigation_started = False  # 标记导航是否已开始
frames: List[Image.Image] = [] if SAVE_GIF else []  # 保存仿真帧用于生成 GIF

# 等待协调器开始处理目标
print("   等待导航启动...")
time.sleep(0.5)

try:
    while True:
        # 检查超时
        elapsed_time = time.time() - start_time
        if elapsed_time > MAX_DURATION:
            print("\n   达到最大运行时长")
            break

        # 获取数据
        pose = sim.get_odom_pose()
        scan = sim.get_laser_scan()
        feedback = coordinator.get_feedback(goal_id)
        state = coordinator.get_current_state()

        if pose is None or scan is None:
            time.sleep(0.05)
            continue

        # 检查导航是否已开始（状态不再是IDLE）
        if state in [
            NavigationState.PLANNING,
            NavigationState.NAVIGATING,
            NavigationState.OBSTACLE_AVOIDING,
            NavigationState.RECOVERY,
        ]:
            navigation_started = True

        # 更新机器人位置
        robot_circle.center = (pose[0], pose[1])

        # 更新机器人朝向
        robot_arrow.remove()
        arrow_length = 0.3
        dx = arrow_length * math.cos(pose[2])
        dy = arrow_length * math.sin(pose[2])
        robot_arrow = ax.arrow(
            pose[0],
            pose[1],
            dx,
            dy,
            head_width=0.1,
            head_length=0.05,
            fc="red",
            ec="red",
            linewidth=2,
        )

        # 更新路径
        if coordinator.global_path:
            path_x = [p[0] for p in coordinator.global_path]
            path_y = [p[1] for p in coordinator.global_path]
            path_line.set_data(path_x, path_y)

        # 更新激光扫描
        if scan is not None and len(scan) > 0:
            scan_points = []
            for i in range(0, len(scan), 10):
                dist = scan[i]
                if dist < 10.0:
                    local_angle = (i / len(scan)) * 2 * math.pi - math.pi
                    x = pose[0] + dist * math.cos(pose[2] + local_angle)
                    y = pose[1] + dist * math.sin(pose[2] + local_angle)
                    scan_points.append([x, y])

            if scan_points:
                scan_scatter.set_offsets(scan_points)

        # 更新信息文本
        progress = feedback.progress if feedback else 0
        distance = feedback.distance_to_goal if feedback else 0

        info_str = f"State: {state.value.upper()}\n"
        info_str += f"Time: {elapsed_time:.1f}s\n"
        info_str += f"Progress: {progress:.0%}\n"
        info_str += f"Distance: {distance:.2f}m\n"
        info_str += f"Position: ({pose[0]:.2f}, {pose[1]:.2f})"

        info_text.set_text(info_str)

        # 状态变化时打印
        if state != last_state:
            print(
                f"   [{elapsed_time:.1f}s] 状态: {state.value} | 进度: {progress:.0%} | 距离: {distance:.2f}m"
            )
            last_state = state

        # 刷新显示
        fig.canvas.draw()
        fig.canvas.flush_events()

        # 保存当前帧（仅在 SAVE_GIF=True 时）
        if SAVE_GIF:
            w, h = fig.canvas.get_width_height()
            try:
                buf = fig.canvas.tostring_rgb()
                img = Image.frombytes("RGB", (w, h), buf)
            except AttributeError:
                # 某些后端（如 TkAgg）没有 tostring_rgb，使用 tostring_argb
                buf = np.frombuffer(fig.canvas.tostring_argb(), dtype=np.uint8).reshape((h, w, 4))
                # ARGB -> RGB（去掉 Alpha 通道）
                img = Image.fromarray(buf[:, :, 1:])
            frames.append(img.convert("RGB"))

        # 只有导航开始后才检查完成状态
        if navigation_started and state == NavigationState.IDLE:
            error_msg = feedback.error_msg if feedback else None
            if error_msg is None:
                print("\n   [OK] 导航成功！")
            else:
                print(f"\n   [FAIL] 导航失败: {error_msg}")

        # 控制更新频率
        time.sleep(0.05)
        frame_count += 1

except KeyboardInterrupt:
    print("\n   用户中断")

# 8. 保存 GIF（仅在 SAVE_GIF=True 时）
if SAVE_GIF:
    print("\n8. 保存仿真 GIF...")
    if frames:
        output_path = "navigation_simulation.gif"
        frames[0].save(
            output_path,
            save_all=True,
            append_images=frames[1:],
            duration=100,
            loop=0,
        )
        print(f"   [OK] GIF 已保存: {output_path} ({len(frames)} 帧, 10 fps)")
    else:
        print("   [WARN] 没有捕获到帧")

# 9. 清理
print("\n9. 清理...")
coordinator.stop()
sim.stop()
plt.close(fig)
print("   [OK] 系统已停止")

print("\n" + "=" * 70)
print("演示完成！")
print("=" * 70)
