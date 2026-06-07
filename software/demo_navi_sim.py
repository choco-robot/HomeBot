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
from navigation.simulation.simulator import SimulatorConfig
from navigation.simulation.robot_model import RobotConfig
from navigation.coordinator import NavigationCoordinator, NavigationState
from navigation.perception.obstacle_detector import DepthObstacle
from navigation.core.occupancy_grid import COST_LETHAL

# 配置
from configs import get_config

# ZeroMQ
import zmq
from common.zmq_helper import create_socket

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
EXTERNAL_MAP_JSON = 'maps\map_2026-06-07T14-32-45.json'  # type: Optional[str]

# 地图类型：'maze', 'simple_room', 'cluttered'
# 仅在没有配置外部地图时生效
MAP_TYPE = "maze"

# 起点和终点（默认，当外部地图未包含 markers 时使用）
DEFAULT_START = (-2.5, -2.5)
DEFAULT_GOAL = (2.0, 1.0)

# 仿真参数
MAX_DURATION = 9999.0  # 最大运行时长（秒）
UPDATE_INTERVAL = 100  # 更新间隔（毫秒）

# 从全局配置加载导航参数
_nav_config = get_config().navigation

# 是否保存仿真过程为 GIF
SAVE_GIF = False

# 命令行参数
import argparse
_parser = argparse.ArgumentParser(description="HomeBot 导航仿真器")
_parser.add_argument("--viser", action="store_true", help="启用 Viser 交互模式：到达终点后不退出，持续监听新目标")
_args = _parser.parse_args()
ENABLE_VISER = _args.viser

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
        START = (-3.5, -1.0)
        GOAL = (-0.5, 2.0)
    elif MAP_TYPE == "simple_room":
        map_env = MapEnvironment.create_simple_room()
    else:
        map_env = MapEnvironment.create_cluttered_room()
    print(f"   [OK] 地图: {map_env.width:.1f}m x {map_env.height:.1f}m")

# 创建与导航配置同步的仿真器
sim_config = SimulatorConfig(
    robot_config=RobotConfig(
        max_linear_vel=_nav_config.max_linear_speed,
        max_angular_vel=_nav_config.max_angular_speed,
    )
)
sim = Simulator(config=sim_config)
sim.set_map(map_env)

# 2. 创建协调器
print("\n2. 创建协调器...")
coordinator = NavigationCoordinator(
    {
        "goal_reached_distance": _nav_config.arrival_distance_threshold_m,
        "goal_reached_angle": _nav_config.arrival_angle_threshold_rad,
        "control_frequency": _nav_config.control_rate_hz,
        "max_replan_attempts": _nav_config.max_replan_attempts,
        "obstacle_emergency_distance": _nav_config.emergency_obstacle_distance_m,
        "replan_distance_threshold": _nav_config.max_path_deviation_m,
        "inflation_radius": _nav_config.inflation_radius_m,
        "robot_radius": _nav_config.robot_radius_m,
        "lookahead_distance": _nav_config.lookahead_distance_m,
        "max_angular_accel_rad": _nav_config.max_angular_accel_rad,
        "velocity_filter_alpha": _nav_config.velocity_filter_alpha,
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

# 速度历史记录
velocity_history: List[Tuple[float, float, float]] = []  # (time, linear, angular)

# ------------------------------------------------------------------------------
# ZeroMQ 发布与命令接收（供 Viser 可视化器订阅和交互）
# ------------------------------------------------------------------------------

# 地址配置（与真实系统保持一致）
ODOM_PUB_ADDR = "tcp://*:5559"
LIDAR_SCAN_PUB_ADDR = "tcp://*:5565"
SLAM_POSE_PUB_ADDR = "tcp://*:5563"
SLAM_MAP_PUB_ADDR = "tcp://*:5564"
GLOBAL_PATH_PUB_ADDR = "tcp://*:5569"
NAV_STATUS_PUB_ADDR = "tcp://*:5570"
GOAL_SUB_ADDR = "tcp://localhost:5566"
ODOM_CMD_REP_ADDR = "tcp://*:5567"
SLAM_CMD_REP_ADDR = "tcp://*:5568"

# PUB socket: 对外发布传感器和位姿数据
odom_pub = create_socket(zmq.PUB, bind=True, address=ODOM_PUB_ADDR)
lidar_scan_pub = create_socket(zmq.PUB, bind=True, address=LIDAR_SCAN_PUB_ADDR)
slam_pose_pub = create_socket(zmq.PUB, bind=True, address=SLAM_POSE_PUB_ADDR)
slam_map_pub = create_socket(zmq.PUB, bind=True, address=SLAM_MAP_PUB_ADDR)
global_path_pub = create_socket(zmq.PUB, bind=True, address=GLOBAL_PATH_PUB_ADDR)
nav_status_pub = create_socket(zmq.PUB, bind=True, address=NAV_STATUS_PUB_ADDR)

# SUB socket: 接收 Viser 下发的导航目标
goal_sub = create_socket(zmq.SUB, bind=False, address=GOAL_SUB_ADDR)
goal_sub.setsockopt(zmq.SUBSCRIBE, b"")
goal_sub.setsockopt(zmq.RCVTIMEO, 0)

# REP socket: 响应重置命令（里程计 / SLAM）
odom_cmd_rep = create_socket(zmq.REP, bind=True, address=ODOM_CMD_REP_ADDR)
odom_cmd_rep.setsockopt(zmq.RCVTIMEO, 0)
slam_cmd_rep = create_socket(zmq.REP, bind=True, address=SLAM_CMD_REP_ADDR)
slam_cmd_rep.setsockopt(zmq.RCVTIMEO, 0)

# 状态
_last_map_pub_time = 0.0
_last_global_path: Optional[List[Tuple[float, float]]] = None
_map_pub_interval = 2.0  # 地图发布间隔（秒）


def _publish_odom(pose: Tuple[float, float, float], velocity: Tuple[float, float]) -> None:
    """发布里程计数据"""
    x, y, theta = pose
    v, w = velocity
    msg = {
        "x": float(round(x, 4)),
        "y": float(round(y, 4)),
        "yaw": float(round(theta, 4)),
        "vx": float(round(v, 4)),
        "vy": 0.0,
        "vz": float(round(w, 4)),
        "timestamp": time.time(),
    }
    try:
        odom_pub.send_json(msg, flags=zmq.NOBLOCK)
    except zmq.Again:
        pass


def _publish_lidar_scan(scan: np.ndarray, angles: np.ndarray) -> None:
    """发布激光雷达扫描数据"""
    try:
        msg = {
            "angles_deg": np.degrees(angles).tolist(),
            "distances_m": scan.tolist(),
            "timestamp": time.time(),
        }
        lidar_scan_pub.send_json(msg, flags=zmq.NOBLOCK)
    except zmq.Again:
        pass


def _publish_slam_pose(pose: Tuple[float, float, float], state: str = "tracking") -> None:
    """发布 SLAM 位姿（仿真中直接使用真实位姿）"""
    x, y, theta = pose
    msg = {
        "x": float(round(x, 4)),
        "y": float(round(y, 4)),
        "theta": float(round(theta, 4)),
        "covariance": [
            [0.01, 0.0, 0.0],
            [0.0, 0.01, 0.0],
            [0.0, 0.0, 0.001],
        ],
        "state": state,
        "timestamp": time.time(),
    }
    try:
        slam_pose_pub.send_json(msg, flags=zmq.NOBLOCK)
    except zmq.Again:
        pass


def _publish_slam_map(grid) -> None:
    """发布栅格地图（BreezySLAM 风格灰度字节数组）"""
    global _last_map_pub_time
    if grid is None:
        return
    try:
        size_pixels = grid.width
        size_meters = grid.width * grid.resolution
        # 将 OccupancyGrid cost 值映射为 BreezySLAM 灰度约定
        # 0=空闲 -> 0, 未知 -> 127, 占用 -> 255
        # 注意：grid.data 是 int16，不能先转 uint8（-1 会变成 255）
        data = grid.data
        gray = np.zeros((grid.height, grid.width), dtype=np.uint8)
        gray[data == 0] = 0       # COST_FREE
        gray[data == -1] = 127    # COST_UNKNOWN
        gray[data >= 100] = 255   # COST_OCCUPIED / COST_LETHAL
        map_bytes = gray.tobytes()

        meta = {
            "size_pixels": size_pixels,
            "size_meters": size_meters,
            "timestamp": time.time(),
        }
        slam_map_pub.send_json(meta, flags=zmq.SNDMORE)
        slam_map_pub.send(map_bytes, flags=zmq.NOBLOCK)
        _last_map_pub_time = time.time()
    except zmq.Again:
        pass
    except Exception as e:
        print(f"   [WARN] 发布地图失败: {e}")


def _publish_global_path(path: List[Tuple[float, float]]) -> None:
    """发布全局路径"""
    try:
        msg = {
            "path": [[float(p[0]), float(p[1])] for p in path],
            "timestamp": time.time(),
        }
        global_path_pub.send_json(msg, flags=zmq.NOBLOCK)
    except zmq.Again:
        pass


# 当前导航目标 ID（用于外部命令管理）
_current_goal_id: Optional[str] = None


def _handle_goal_msg(goal_msg: dict) -> None:
    """处理 Viser 下发的导航目标"""
    global _current_goal_id
    cmd = goal_msg.get("cmd", "")
    if cmd == "stop":
        print("   [ZMQ] 收到停止导航命令")
        sim.set_velocity(0.0, 0.0)
        if _current_goal_id:
            coordinator.cancel_goal(_current_goal_id)
            _current_goal_id = None
        return

    # 途径点模式
    if "waypoints" in goal_msg and "final_goal" in goal_msg:
        waypoints = goal_msg["waypoints"]
        final = goal_msg["final_goal"]
        print(f"   [ZMQ] 收到途径点导航任务: {len(waypoints)} 个途径点 -> 最终目标 ({final['x']:.2f}, {final['y']:.2f})")
        # 依次将途径点加入 coordinator 队列
        for wp in waypoints:
            coordinator.navigate_to_async(x=wp["x"], y=wp["y"], timeout=MAX_DURATION)
        # 最后加入最终目标
        _current_goal_id = coordinator.navigate_to_async(x=final["x"], y=final["y"], yaw=final.get("theta"), timeout=MAX_DURATION)
        return

    # 单目标模式
    x = goal_msg.get("x")
    y = goal_msg.get("y")
    theta = goal_msg.get("theta", 0.0)
    if x is not None and y is not None:
        print(f"   [ZMQ] 收到导航目标: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)")
        _current_goal_id = coordinator.navigate_to_async(x=x, y=y, yaw=theta, timeout=MAX_DURATION)


def _handle_cmd_req(sock: zmq.Socket) -> None:
    """非阻塞处理 REP 命令请求"""
    try:
        req = sock.recv_json(flags=zmq.NOBLOCK)
        cmd = req.get("cmd", "")
        if cmd == "reset_pose":
            x = req.get("x", 0.0)
            y = req.get("y", 0.0)
            yaw = req.get("yaw", req.get("theta", 0.0))
            sim.reset_robot(x=x, y=y, theta=yaw)
            sock.send_json({"success": True, "message": f"已重置为 ({x}, {y}, {yaw})"})
        else:
            sock.send_json({"success": False, "message": f"未知命令: {cmd}"})
    except zmq.Again:
        pass
    except Exception as e:
        try:
            sock.send_json({"success": False, "message": str(e)})
        except Exception:
            pass


# 4. 创建可视化窗口
print("\n4. 创建可视化窗口...")
plt.ion()  # 开启交互模式

fig = plt.figure(figsize=(14, 8))
gs = fig.add_gridspec(2, 2, width_ratios=[2, 1], height_ratios=[1, 1])

# 左侧：地图可视化
ax = fig.add_subplot(gs[:, 0])
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

# 窗口关闭标志
window_closed = False

def on_close(event):
    global window_closed
    window_closed = True

fig.canvas.mpl_connect('close_event', on_close)

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
path_points_scatter = ax.scatter([], [], c="cyan", s=30, zorder=5, label="Path Points")
local_goal_scatter = ax.scatter([], [], c="yellow", s=150, marker="*", zorder=6, edgecolors="black", linewidths=1, label="Local Goal")
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

# 右侧：速度曲线
ax_v = fig.add_subplot(gs[0, 1])
ax_v.set_title("Linear Velocity (m/s)", fontsize=12)
ax_v.set_xlabel("Time (s)", fontsize=10)
ax_v.set_ylabel("v (m/s)", fontsize=10)
ax_v.grid(True, alpha=0.3)
(v_line,) = ax_v.plot([], [], "b-", linewidth=1.5, label="Linear")
ax_v.legend(loc="upper right")

ax_w = fig.add_subplot(gs[1, 1])
ax_w.set_title("Angular Velocity (rad/s)", fontsize=12)
ax_w.set_xlabel("Time (s)", fontsize=10)
ax_w.set_ylabel("ω (rad/s)", fontsize=10)
ax_w.grid(True, alpha=0.3)
(w_line,) = ax_w.plot([], [], "r-", linewidth=1.5, label="Angular")
ax_w.legend(loc="upper right")

fig.tight_layout()
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
_current_goal_id = coordinator.navigate_to_async(x=GOAL[0], y=GOAL[1], timeout=MAX_DURATION)
goal_id = _current_goal_id
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
        # 检查窗口是否关闭
        if window_closed:
            print("\n   窗口已关闭")
            break

        # 检查超时
        elapsed_time = time.time() - start_time
        if elapsed_time > MAX_DURATION:
            print("\n   达到最大运行时长")
            break

        # 获取数据
        pose = sim.get_odom_pose()
        scan = sim.get_laser_scan()
        feedback = coordinator.get_feedback(_current_goal_id)
        state = coordinator.get_current_state()

        if pose is None or scan is None:
            time.sleep(0.05)
            continue

        # ---- ZeroMQ 发布 ----
        # 1. 发布里程计
        if sim.robot:
            _publish_odom(pose, sim.robot.velocity)

        # 2. 发布激光雷达扫描
        if sim.laser:
            _publish_lidar_scan(scan, sim.laser.get_angles())

        # 3. 发布 SLAM 位姿（仿真中直接使用真实位姿）
        true_pose = sim.get_robot_pose()
        nav_state = state.value if state else "tracking"
        if true_pose:
            _publish_slam_pose(true_pose, state=nav_state)

        # 3.5 发布导航状态（供 Viser 订阅）
        if feedback:
            try:
                nav_status_pub.send_json(
                    {
                        "state": nav_state,
                        "distance_to_goal": float(feedback.distance_to_goal),
                        "progress": float(feedback.progress),
                        "goal_id": _current_goal_id or "",
                        "timestamp": time.time(),
                    },
                    flags=zmq.NOBLOCK,
                )
            except zmq.Again:
                pass

        # 4. 低频发布栅格地图
        grid = sim.get_map()
        if grid and (time.time() - _last_map_pub_time >= _map_pub_interval):
            _publish_slam_map(grid)

        # 5. 发布全局路径（变化时）
        if coordinator.global_path != _last_global_path:
            _last_global_path = coordinator.global_path
            if _last_global_path:
                _publish_global_path(_last_global_path)

        # 6. 处理外部命令（非阻塞）
        _handle_cmd_req(odom_cmd_rep)
        _handle_cmd_req(slam_cmd_rep)

        # 7. 接收 Viser 下发的导航目标
        try:
            goal_msg = goal_sub.recv_json(flags=zmq.NOBLOCK)
            if goal_msg:
                _handle_goal_msg(goal_msg)
        except zmq.Again:
            pass
        except Exception:
            pass

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
            path_points_scatter.set_offsets(np.column_stack([path_x, path_y]))

        # 更新局部目标点
        local_goal = coordinator.get_local_goal()
        if local_goal:
            local_goal_scatter.set_offsets([[local_goal[0], local_goal[1]]])
        else:
            local_goal_scatter.set_offsets(np.empty((0, 2)))

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

        # 记录速度
        if sim.robot:
            v, w = sim.robot.velocity
            velocity_history.append((elapsed_time, v, w))

        # 更新速度曲线
        if velocity_history:
            t_vals = [v[0] for v in velocity_history]
            v_vals = [v[1] for v in velocity_history]
            w_vals = [v[2] for v in velocity_history]
            v_line.set_data(t_vals, v_vals)
            w_line.set_data(t_vals, w_vals)
            max_t = max(t_vals[-1], 1)
            ax_v.set_xlim(0, max_t)
            ax_v.set_ylim(max(0, min(v_vals) - 0.05), max(v_vals) + 0.05)
            ax_w.set_xlim(0, max_t)
            ax_w.set_ylim(min(w_vals) - 0.1, max(w_vals) + 0.1)

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

            if not ENABLE_VISER:
                break
            else:
                # Viser 模式：重置状态，停止机器人，继续监听新目标
                navigation_started = False
                sim.stop_robot()
                print("   [Viser 模式] 等待新目标...")
                continue

        # 控制更新频率
        time.sleep(0.02)
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

# 关闭 ZeroMQ socket
for sock in (odom_pub, lidar_scan_pub, slam_pose_pub, slam_map_pub, global_path_pub,
             nav_status_pub, goal_sub, odom_cmd_rep, slam_cmd_rep):
    try:
        sock.close()
    except Exception:
        pass

coordinator.stop()
sim.stop()
plt.close(fig)
print("   [OK] 系统已停止")

print("\n" + "=" * 70)
print("演示完成！")
print("=" * 70)
