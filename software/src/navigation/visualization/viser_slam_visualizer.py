# -*- coding: utf-8 -*-
"""Viser SLAM 可视化器

基于 Viser (https://viser.studio) 的实时 3D 可视化，类似 RViz。
订阅以下 ZeroMQ 话题：
    - SLAM 位姿    tcp://localhost:5563  (JSON)
    - SLAM 地图    tcp://localhost:5564  (multipart: json_meta + map_bytes)
    - 里程计       tcp://localhost:5559  (JSON)
    - 激光雷达     tcp://localhost:5565  (JSON)
    - 摄像头图像   tcp://localhost:5560  (multipart: frame_id + jpeg_bytes)

可视化内容：
    - 坐标系树: map -> base_link -> laser / camera
    - 机器人模型: 圆柱底盘 + 方向箭头
    - 激光雷达点云: 实时扫描点 (红绿渐变)
    - 栅格地图: SLAM 地图纹理 (水平平面)
    - 摄像头视锥: 带实时图像的相机 frustum
    - 轨迹: odom 轨迹(黄色) + slam 轨迹(青色)
    - GUI 面板: 状态显示、图层控制、视角切换

启动方式:
    cd software/src
    python -m navigation.visualization
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Any, Deque, Optional, Tuple

import cv2
import numpy as np
import viser
import viser.transforms as tf
import zmq

from common.logging import get_logger
from common.zmq_helper import create_socket
from common.zmq_subscriber import ZMQJsonSubscriber, ZMQMultipartImageSubscriber
from configs import get_config

logger = get_logger(__name__)

# ------------------------------------------------------------------------------
# 颜色常量
# ------------------------------------------------------------------------------
COLOR_RED = np.array([255, 50, 50], dtype=np.uint8)
COLOR_GREEN = np.array([50, 255, 50], dtype=np.uint8)
COLOR_BLUE = np.array([50, 50, 255], dtype=np.uint8)
COLOR_CYAN = np.array([0, 255, 255], dtype=np.uint8)
COLOR_YELLOW = np.array([255, 255, 0], dtype=np.uint8)
COLOR_WHITE = np.array([255, 255, 255], dtype=np.uint8)
COLOR_ORANGE = np.array([255, 165, 0], dtype=np.uint8)


def yaw_to_wxyz(yaw: float) -> Tuple[float, float, float, float]:
    """将 2D 航向角转为 viser 四元数 wxyz (绕 z 轴旋转)。"""
    q = tf.SO3.from_z_radians(yaw)
    return tuple(q.wxyz)


# ------------------------------------------------------------------------------
# SLAM 地图订阅者（自定义 multipart 订阅）
# ------------------------------------------------------------------------------
class SLAMMapSubscriber:
    """订阅 SLAM 栅格地图 (multipart: json_meta + map_bytes)。"""

    def __init__(self, sub_addr: str = "tcp://localhost:5564"):
        self._sub = create_socket(zmq.SUB, bind=False, address=sub_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.setsockopt(zmq.RCVTIMEO, 500)

        self._latest_meta: Optional[dict] = None
        self._latest_map_bytes: Optional[bytes] = None
        self._lock = threading.Lock()
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._recv_count = 0
        self._start_receiver()

    def _start_receiver(self) -> None:
        self._running = True
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

    def _receive_loop(self) -> None:
        while self._running:
            try:
                parts = self._sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) >= 2:
                    meta = None
                    try:
                        import json
                        meta = json.loads(parts[0].decode("utf-8"))
                    except Exception:
                        pass
                    if meta and isinstance(meta, dict) and "size_pixels" in meta:
                        with self._lock:
                            self._latest_meta = meta
                            self._latest_map_bytes = parts[1]
                            self._recv_count += 1
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"[SLAMMapSubscriber] 接收异常: {e}")
            time.sleep(0.01)

    def read(self) -> Tuple[Optional[dict], Optional[bytes]]:
        with self._lock:
            return (
                self._latest_meta.copy() if self._latest_meta else None,
                self._latest_map_bytes,
            )

    def get_stats(self) -> dict:
        with self._lock:
            return {"recv_count": self._recv_count, "has_data": self._latest_meta is not None}

    def close(self) -> None:
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        self._sub.close()


# ------------------------------------------------------------------------------
# 主可视化器
# ------------------------------------------------------------------------------
class ViserSLAMVisualizer:
    """Viser SLAM 实时可视化器。"""

    def __init__(self):
        cfg = get_config().viser
        self._odom_addr = cfg.odom_sub_addr
        self._slam_pose_addr = cfg.slam_pose_sub_addr
        self._slam_map_addr = cfg.slam_map_sub_addr
        self._lidar_scan_addr = cfg.lidar_scan_sub_addr
        self._vision_addr = cfg.vision_sub_addr
        self._goal_pub_addr = cfg.goal_pub_addr
        self._odom_cmd_addr = cfg.odom_cmd_addr
        self._slam_cmd_addr = cfg.slam_cmd_addr
        self._global_path_sub_addr = cfg.global_path_sub_addr

        self._max_traj_points = cfg.max_trajectory_points
        self._point_size = cfg.point_size
        self._map_update_interval = cfg.map_update_interval
        self._follow_robot = cfg.follow_robot

        # Viser 服务器
        self._server = viser.ViserServer(host=cfg.host, port=cfg.port)
        self._server.gui.configure_theme(dark_mode=True)
        logger.info(f"Viser server started at http://{cfg.host}:{cfg.port}")

        # ZeroMQ 订阅者
        self._odom_sub = ZMQJsonSubscriber(self._odom_addr, required_keys=("x", "y", "yaw"))
        self._slam_pose_sub = ZMQJsonSubscriber(self._slam_pose_addr, required_keys=("x", "y", "theta"))
        self._lidar_scan_sub = ZMQJsonSubscriber(self._lidar_scan_addr, required_keys=("angles_deg", "distances_m"))
        self._vision_sub = ZMQMultipartImageSubscriber(self._vision_addr)
        self._map_sub = SLAMMapSubscriber(self._slam_map_addr)
        self._path_sub = ZMQJsonSubscriber(self._global_path_sub_addr, required_keys=("path",))

        # ZeroMQ 发布者/请求者（导航控制）
        self._goal_pub = create_socket(zmq.PUB, bind=True, address=self._goal_pub_addr)
        logger.info(f"Goal PUB: {self._goal_pub_addr}")
        self._odom_cmd = create_socket(zmq.REQ, bind=False, address=self._odom_cmd_addr)
        self._odom_cmd.setsockopt(zmq.RCVTIMEO, 1000)
        logger.info(f"Odom CMD REQ: {self._odom_cmd_addr}")
        self._slam_cmd = create_socket(zmq.REQ, bind=False, address=self._slam_cmd_addr)
        self._slam_cmd.setsockopt(zmq.RCVTIMEO, 1000)
        logger.info(f"SLAM CMD REQ: {self._slam_cmd_addr}")

        # 轨迹历史
        self._odom_traj: Deque[Tuple[float, float]] = deque(maxlen=self._max_traj_points)
        self._slam_traj: Deque[Tuple[float, float]] = deque(maxlen=self._max_traj_points)

        # 状态
        self._running = False
        self._last_map_update_time = 0.0
        self._last_lidar_time = 0.0

        # 目标点状态
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._goal_theta = 0.0
        self._goal_marker_visible = False

        # 场景节点 handle 缓存
        self._handles: dict[str, Any] = {}

        # 构建场景和 GUI
        self._setup_scene()
        self._setup_gui()
        # 初始化默认目标点标记
        self._update_goal_marker()

    # --------------------------------------------------------------------------
    # 场景初始化
    # --------------------------------------------------------------------------
    def _setup_scene(self) -> None:
        s = self._server.scene

        # 地图固定坐标系
        self._handles["/map"] = s.add_frame("/map", show_axes=True, axes_length=0.3, axes_radius=0.02)

        # 地面网格
        s.add_grid("/map/ground_grid", width=10, height=10, cell_size=1.0)

        # 里程计坐标系
        self._handles["/map/odom_frame"] = s.add_frame(
            "/map/odom_frame", show_axes=True, axes_length=0.2, axes_radius=0.015, visible=False
        )

        # 机器人本体坐标系
        self._handles["/map/base_link"] = s.add_frame(
            "/map/base_link", show_axes=True, axes_length=0.25, axes_radius=0.02
        )

        # 机器人底盘（圆柱）
        s.add_cylinder(
            "/map/base_link/body",
            radius=0.18,
            height=0.10,
            position=(0.0, 0.0, 0.05),
            color=(100, 180, 255),
        )

        # 方向箭头杆
        s.add_cylinder(
            "/map/base_link/arrow_shaft",
            radius=0.03,
            height=0.30,
            position=(0.10, 0.0, 0.05),
            wxyz=tf.SO3.from_y_radians(math.pi / 2).wxyz,
            color=(255, 200, 50),
        )

        # 位姿文字标签（显示在机器人上方）
        self._handles["/map/base_link/pose_label"] = s.add_label(
            "/map/base_link/pose_label",
            text="x: 0.00, y: 0.00, θ: 0°",
            position=(0.0, 0.0, 0.35),
        )

        # 激光雷达坐标系（相对于 base_link）
        self._handles["/map/base_link/laser"] = s.add_frame(
            "/map/base_link/laser", show_axes=True, axes_length=0.15, axes_radius=0.01
        )
        self._handles["/map/base_link/laser"].position = np.array([0.0, 0.0, 0.10])

        # 摄像头坐标系
        self._handles["/map/base_link/camera"] = s.add_frame(
            "/map/base_link/camera", show_axes=True, axes_length=0.15, axes_radius=0.01
        )
        self._handles["/map/base_link/camera"].position = np.array([0.10, 0.0, 0.65])
        self._handles["/map/base_link/camera"].wxyz = tf.SO3.from_x_radians(math.pi / 2).wxyz

        # 相机视锥（默认朝向 +z，绕 y 轴 +90° 转向 +x，即机器人前方）
        self._handles["/map/base_link/camera/frustum"] = s.add_camera_frustum(
            "/map/base_link/camera/frustum",
            fov=60.0,
            aspect=16.0 / 9.0,
            scale=0.15,
            color=(200, 200, 200),
            wxyz=tf.SO3.from_y_radians(math.pi / 2).wxyz,
        )

        logger.info("Scene initialized")

    # --------------------------------------------------------------------------
    # GUI 初始化
    # --------------------------------------------------------------------------
    def _setup_gui(self) -> None:
        g = self._server.gui

        # 标题
        g.add_markdown("# 🤖 HomeBot SLAM 可视化")

        # 状态面板
        with g.add_folder("📊 机器人状态", expand_by_default=True):
            self._gui_status_x = g.add_number("X (m)", 0.0, step=0.01, disabled=True)
            self._gui_status_y = g.add_number("Y (m)", 0.0, step=0.01, disabled=True)
            self._gui_status_theta = g.add_number("Theta (deg)", 0.0, step=0.01, disabled=True)
            self._gui_status_vx = g.add_number("Vx (m/s)", 0.0, step=0.01, disabled=True)
            self._gui_status_vz = g.add_number("Vz (rad/s)", 0.0, step=0.01, disabled=True)
            self._gui_status_state = g.add_text("SLAM 状态", "等待数据...")

        # 图层控制
        with g.add_folder("🎨 图层控制", expand_by_default=False):
            self._gui_show_map = g.add_checkbox("栅格地图", True)
            self._gui_show_lidar = g.add_checkbox("激光雷达", True)
            self._gui_show_odom_traj = g.add_checkbox("里程计轨迹", True)
            self._gui_show_slam_traj = g.add_checkbox("SLAM 轨迹", True)
            self._gui_show_camera = g.add_checkbox("摄像头图像", True)
            self._gui_show_odom_frame = g.add_checkbox("里程计坐标系", False)
            self._gui_show_global_path = g.add_checkbox("全局路径", True)

        # 视角控制
        with g.add_folder("🎥 视角控制", expand_by_default=False):
            self._gui_follow_robot = g.add_checkbox("跟随机器人", self._follow_robot)
            self._gui_top_view = g.add_button("🔝 顶视图")
            self._gui_reset_view = g.add_button("🔄 重置视角")
            self._gui_clear_traj = g.add_button("🗑️ 清除轨迹")

        # 导航控制
        with g.add_folder("🎯 导航控制", expand_by_default=True):
            self._gui_goal_x = g.add_number("目标 X (m)", 0.0, step=0.1)
            self._gui_goal_y = g.add_number("目标 Y (m)", 0.0, step=0.1)
            self._gui_goal_theta = g.add_number("目标 Theta (deg)", 0.0, step=5.0)
            self._gui_set_goal = g.add_button("📍 设置目标点")
            self._gui_reset_odom = g.add_button("🔄 重置里程计")
            self._gui_reset_slam = g.add_button("🔄 重置 SLAM 位姿")
            self._gui_nav_status = g.add_text("操作状态", "就绪")

        # 数据流状态
        with g.add_folder("📡 数据流", expand_by_default=False):
            self._gui_odom_hz = g.add_number("Odom 接收数", 0, disabled=True)
            self._gui_slam_hz = g.add_number("SLAM 接收数", 0, disabled=True)
            self._gui_lidar_hz = g.add_number("Lidar 接收数", 0, disabled=True)
            self._gui_map_hz = g.add_number("Map 接收数", 0, disabled=True)
            self._gui_vision_hz = g.add_number("Vision 接收数", 0, disabled=True)
            self._gui_path_hz = g.add_number("Path 接收数", 0, disabled=True)

        # 回调绑定
        self._gui_top_view.on_click(lambda _: self._set_top_view())
        self._gui_reset_view.on_click(lambda _: self._reset_view())
        self._gui_clear_traj.on_click(lambda _: self._clear_trajectories())
        self._gui_set_goal.on_click(lambda _: self._on_set_goal())
        self._gui_reset_odom.on_click(lambda _: self._on_reset_odom())
        self._gui_reset_slam.on_click(lambda _: self._on_reset_slam())

        # 图层控制回调
        self._gui_show_map.on_update(lambda _: self._update_layer_visibility())
        self._gui_show_lidar.on_update(lambda _: self._update_layer_visibility())
        self._gui_show_odom_traj.on_update(lambda _: self._update_layer_visibility())
        self._gui_show_slam_traj.on_update(lambda _: self._update_layer_visibility())
        self._gui_show_camera.on_update(lambda _: self._update_layer_visibility())
        self._gui_show_odom_frame.on_update(lambda _: self._update_layer_visibility())
        self._gui_show_global_path.on_update(lambda _: self._update_layer_visibility())

        logger.info("GUI initialized")

    # --------------------------------------------------------------------------
    # 视角控制
    # --------------------------------------------------------------------------
    def _set_top_view(self) -> None:
        """切换到顶视图。"""
        for client in self._server.get_clients().values():
            client.camera.wxyz = (0.707, 0.707, 0.0, 0.0)  # 俯视角
            client.camera.position = (0.0, 0.0, 15.0)
            client.camera.look_at = (0.0, 0.0, 0.0)

    def _reset_view(self) -> None:
        """重置到默认视角。"""
        for client in self._server.get_clients().values():
            client.camera.wxyz = (0.624, 0.331, 0.412, 0.586)
            client.camera.position = (5.0, -5.0, 5.0)
            client.camera.look_at = (0.0, 0.0, 0.0)

    def _clear_trajectories(self) -> None:
        """清除轨迹历史。"""
        self._odom_traj.clear()
        self._slam_traj.clear()
        self._remove_node("/map/odom_trajectory")
        self._remove_node("/map/slam_trajectory")

    def _update_layer_visibility(self) -> None:
        """根据 GUI 图层控制更新各节点可见性。"""
        # 静态节点
        h = self._handles.get("/map/odom_frame")
        if h is not None:
            h.visible = self._gui_show_odom_frame.value
        h = self._handles.get("/map/base_link/camera/frustum")
        if h is not None:
            h.visible = self._gui_show_camera.value

        # 动态节点：如果当前被关闭，立即移除
        if not self._gui_show_map.value:
            self._remove_node("/map/occupancy_map")
        if not self._gui_show_lidar.value:
            self._remove_node("/map/base_link/laser/scan")
        if not self._gui_show_odom_traj.value:
            self._remove_node("/map/odom_trajectory")
        if not self._gui_show_slam_traj.value:
            self._remove_node("/map/slam_trajectory")

    # --------------------------------------------------------------------------
    # 节点管理辅助
    # --------------------------------------------------------------------------
    def _remove_node(self, name: str) -> None:
        """安全删除场景节点（如果存在）。"""
        if name in self._handles:
            try:
                self._handles[name].remove()
            except Exception:
                pass
            del self._handles[name]

    # --------------------------------------------------------------------------
    # 主循环
    # --------------------------------------------------------------------------
    def start(self) -> None:
        """启动可视化主循环。"""
        self._running = True
        logger.info("ViserSLAMVisualizer 主循环启动 (10Hz)")

        try:
            while self._running:
                t0 = time.perf_counter()

                # 1. 读取所有订阅者的最新数据
                odom = self._odom_sub.read()
                slam_pose = self._slam_pose_sub.read()
                lidar_scan = self._lidar_scan_sub.read()
                vision_frame_id, vision_img = self._vision_sub.read_frame()
                map_meta, map_bytes = self._map_sub.read()

                # 2. 更新可视化
                if slam_pose:
                    self._update_robot_pose(slam_pose)
                    self._update_slam_trajectory(slam_pose)
                    self._update_status_panel(slam_pose, odom)

                if odom:
                    self._update_odom_frame(odom)
                    self._update_odom_trajectory(odom)

                if lidar_scan and self._gui_show_lidar.value:
                    self._update_lidar_point_cloud(lidar_scan)

                if vision_img is not None and self._gui_show_camera.value:
                    self._update_camera_image(vision_img)

                if map_bytes is not None and map_meta is not None and self._gui_show_map.value:
                    self._update_occupancy_map(map_meta, map_bytes)

                # 读取并更新全局路径
                path_msg = self._path_sub.read()
                if path_msg is not None and self._gui_show_global_path.value:
                    self._update_global_path(path_msg)

                # 3. 更新 GUI 统计
                self._update_stream_stats()

                # 4. 帧率控制 (~10Hz)
                elapsed = time.perf_counter() - t0
                rem = 0.1 - elapsed
                if rem > 0:
                    time.sleep(rem)

        except KeyboardInterrupt:
            logger.info("ViserSLAMVisualizer 被用户中断")
        except Exception as e:
            logger.error(f"ViserSLAMVisualizer 异常: {e}")
            import traceback
            logger.error(traceback.format_exc())
        finally:
            self.stop()

    def stop(self) -> None:
        """停止并释放资源。"""
        self._running = False
        self._odom_sub.close()
        self._slam_pose_sub.close()
        self._lidar_scan_sub.close()
        self._vision_sub.close()
        self._map_sub.close()
        self._path_sub.close()
        if self._goal_pub:
            self._goal_pub.close()
        if self._odom_cmd:
            self._odom_cmd.close()
        if self._slam_cmd:
            self._slam_cmd.close()
        logger.info("ViserSLAMVisualizer 已停止")

    # --------------------------------------------------------------------------
    # 各组件更新逻辑
    # --------------------------------------------------------------------------
    def _update_robot_pose(self, slam_pose: dict) -> None:
        """更新机器人 base_link 位姿。"""
        x = slam_pose.get("x", 0.0)
        y = slam_pose.get("y", 0.0)
        theta = slam_pose.get("theta", 0.0)

        h = self._handles.get("/map/base_link")
        if h is not None:
            h.position = np.array([x, y, 0.0])
            h.wxyz = yaw_to_wxyz(theta)

        # 更新位姿文字标签（跟随机器人，显示在上方）
        label = self._handles.get("/map/base_link/pose_label")
        if label is not None:
            theta_deg = math.degrees(theta)
            label.text = f"x: {x:.2f}, y: {y:.2f}, θ: {theta_deg:.1f}°"
            label.position = np.array([x, y, 0.35])

        # 跟随机器人模式：更新相机 look_at
        if self._gui_follow_robot.value:
            for client in self._server.get_clients().values():
                client.camera.look_at = np.array([x, y, 0.0])

    def _update_odom_frame(self, odom: dict) -> None:
        """更新里程计坐标系位置。"""
        h = self._handles.get("/map/odom_frame")
        if h is None:
            return

        visible = self._gui_show_odom_frame.value
        h.visible = visible
        if not visible:
            return

        x = odom.get("x", 0.0)
        y = odom.get("y", 0.0)
        yaw = odom.get("yaw", 0.0)
        h.position = np.array([x, y, 0.0])
        h.wxyz = yaw_to_wxyz(yaw)

    def _update_lidar_point_cloud(self, lidar_scan: dict) -> None:
        """将激光雷达扫描数据转为 3D 点云显示。"""
        if not self._gui_show_lidar.value:
            self._remove_node("/map/base_link/laser/scan")
            return

        angles_deg = lidar_scan.get("angles_deg", [])
        distances_m = lidar_scan.get("distances_m", [])
        if len(angles_deg) == 0 or len(distances_m) == 0:
            return

        angles = np.deg2rad(np.array(angles_deg, dtype=np.float32))
        dists = np.array(distances_m, dtype=np.float32)

        # 过滤无效点
        valid = (dists > 0.05) & (dists < 12.0) & np.isfinite(dists)
        angles = angles[valid]
        dists = dists[valid]

        if len(dists) == 0:
            return

        # 极坐标 -> 笛卡尔坐标（在 laser 坐标系下）
        pts = np.zeros((len(dists), 3), dtype=np.float32)
        pts[:, 0] = dists * np.cos(angles)
        pts[:, 1] = dists * np.sin(angles)
        pts[:, 2] = 0.0

        # 颜色：按距离渐变（近绿远红）
        colors = np.zeros((len(dists), 3), dtype=np.uint8)
        max_d = min(dists.max(), 5.0)
        min_d = dists.min()
        denom = max_d - min_d if max_d > min_d else 1.0
        ratio = (dists - min_d) / denom
        ratio = np.clip(ratio, 0.0, 1.0)
        colors[:, 0] = (ratio * 255).astype(np.uint8)      # R
        colors[:, 1] = ((1.0 - ratio) * 255).astype(np.uint8)  # G
        colors[:, 2] = 30  # B

        # 更新或创建点云
        h = self._handles.get("/map/base_link/laser/scan")
        if h is not None and hasattr(h, "points"):
            h.points = pts
            h.colors = colors
        else:
            self._handles["/map/base_link/laser/scan"] = self._server.scene.add_point_cloud(
                "/map/base_link/laser/scan",
                points=pts,
                colors=colors,
                point_size=self._point_size,
                wxyz=tf.SO3.from_z_radians(math.pi).wxyz,  # 点云绕 z 轴旋转 180°
            )
        self._last_lidar_time = time.time()

    def _update_occupancy_map(self, meta: dict, map_bytes: bytes) -> None:
        """将栅格地图渲染为场景中的纹理图像。"""
        if not self._gui_show_map.value:
            self._remove_node("/map/occupancy_map")
            return

        now = time.time()
        if now - self._last_map_update_time < self._map_update_interval:
            return
        self._last_map_update_time = now

        size_pixels = meta.get("size_pixels", 800)
        size_meters = meta.get("size_meters", 20.0)

        if len(map_bytes) < size_pixels * size_pixels:
            return

        # 转为 numpy 并 reshape
        grid = np.frombuffer(map_bytes, dtype=np.uint8).reshape((size_pixels, size_pixels))

        # 渲染为 RGB 图像：
        # 0(空闲)=白, 127(未知)=灰, 255(占据)=黑
        img = np.zeros((size_pixels, size_pixels, 3), dtype=np.uint8)
        img[grid < 50] = [255, 255, 255]       # 空闲 -> 白
        img[(grid >= 50) & (grid < 200)] = [180, 180, 180]  # 未知 -> 灰
        img[grid >= 200] = [40, 40, 40]        # 占据 -> 深灰

        if self._handles.get("/map/occupancy_map") is not None:
            self._handles.get("/map/occupancy_map").image = img
            return
        self._handles["/map/occupancy_map"] = self._server.scene.add_image(
            "/map/occupancy_map",
            image=img,
            render_width=size_meters,
            render_height=size_meters,
            position=(0.0, 0.0, -0.1),  # 略低于地面
        )

    def _update_camera_image(self, img_bgr: np.ndarray) -> None:
        """更新摄像头视锥中的图像。"""
        if not self._gui_show_camera.value:
            return
        # BGR -> RGB
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        h = self._handles.get("/map/base_link/camera/frustum")
        if h is not None and hasattr(h, "image"):
            h.image = img_rgb

    def _update_slam_trajectory(self, slam_pose: dict) -> None:
        """更新 SLAM 轨迹。"""
        if not self._gui_show_slam_traj.value:
            self._remove_node("/map/slam_trajectory")
            return

        x = slam_pose.get("x", 0.0)
        y = slam_pose.get("y", 0.0)
        self._slam_traj.append((x, y))

        if len(self._slam_traj) < 2:
            return

        self._draw_trajectory("/map/slam_trajectory", self._slam_traj, COLOR_CYAN)

    def _update_odom_trajectory(self, odom: dict) -> None:
        """更新里程计轨迹。"""
        if not self._gui_show_odom_traj.value:
            self._remove_node("/map/odom_trajectory")
            return

        x = odom.get("x", 0.0)
        y = odom.get("y", 0.0)
        if self._odom_traj and (x, y) == self._odom_traj[-1]:
            return
        self._odom_traj.append((x, y))

        if len(self._odom_traj) < 2:
            return

        self._draw_trajectory("/map/odom_trajectory", self._odom_traj, COLOR_YELLOW)

    def _update_global_path(self, path_msg: dict) -> None:
        """更新全局路径可视化（橙色线段）。"""
        if not self._gui_show_global_path.value:
            self._remove_node("/map/global_path")
            return

        path = path_msg.get("path", [])
        if len(path) < 2:
            return

        pts = np.array(path, dtype=np.float32)
        n_segments = len(pts) - 1

        # Viser add_line_segments 需要 (N, 2, 3) 形状
        segments = np.zeros((n_segments, 2, 3), dtype=np.float32)
        segments[:, 0, :2] = pts[:-1]
        segments[:, 1, :2] = pts[1:]
        segments[:, :, 2] = 0.05  # 略高于地面

        colors = np.zeros((n_segments, 2, 3), dtype=np.uint8)
        colors[:, :, :] = COLOR_ORANGE

        self._remove_node("/map/global_path")
        self._handles["/map/global_path"] = self._server.scene.add_line_segments(
            "/map/global_path",
            points=segments,
            colors=colors,
            line_width=4.0,
        )

    def _draw_trajectory(self, name: str, traj: Deque[Tuple[float, float]], color: np.ndarray) -> None:
        """使用点云绘制轨迹。"""
        pts = np.array(list(traj), dtype=np.float32)
        if len(pts) < 2:
            return

        # 点云: (N, 3)
        points = np.zeros((len(pts), 3), dtype=np.float32)
        points[:, :2] = pts
        points[:, 2] = 0.05  # 略高于地面，避免被遮挡

        # 颜色: (N, 3) — 每个点一个颜色
        colors = np.zeros((len(pts), 3), dtype=np.uint8)
        colors[:, :] = color

        # 使用 handle 直接更新，避免频繁 remove/recreate
        h = self._handles.get(name)
        if h is not None and hasattr(h, "points"):
            h.points = points
            h.colors = colors
        else:
            self._handles[name] = self._server.scene.add_point_cloud(
                name,
                points=points,
                colors=colors,
                point_size=self._point_size,
            )

    def _update_status_panel(self, slam_pose: dict, odom: Optional[dict]) -> None:
        """更新 GUI 状态面板。"""
        self._gui_status_x.value = round(slam_pose.get("x", 0.0), 3)
        self._gui_status_y.value = round(slam_pose.get("y", 0.0), 3)
        theta_deg = math.degrees(slam_pose.get("theta", 0.0))
        self._gui_status_theta.value = round(theta_deg, 2)
        self._gui_status_state.value = slam_pose.get("state", "UNKNOWN")

        if odom:
            self._gui_status_vx.value = round(odom.get("vx", 0.0), 3)
            self._gui_status_vz.value = round(odom.get("vz", 0.0), 3)

    def _update_stream_stats(self) -> None:
        """更新数据流统计。"""
        self._gui_odom_hz.value = self._odom_sub.get_stats()["recv_count"]
        self._gui_slam_hz.value = self._slam_pose_sub.get_stats()["recv_count"]
        self._gui_lidar_hz.value = self._lidar_scan_sub.get_stats()["recv_count"]
        self._gui_map_hz.value = self._map_sub.get_stats()["recv_count"]
        self._gui_vision_hz.value = self._vision_sub.get_stats()["recv_count"]
        self._gui_path_hz.value = self._path_sub.get_stats()["recv_count"]

    # --------------------------------------------------------------------------
    # 导航控制
    # --------------------------------------------------------------------------
    def _on_set_goal(self) -> None:
        """设置目标点并发布。"""
        self._goal_x = float(self._gui_goal_x.value)
        self._goal_y = float(self._gui_goal_y.value)
        self._goal_theta = math.radians(float(self._gui_goal_theta.value))

        goal_msg = {
            "x": self._goal_x,
            "y": self._goal_y,
            "theta": self._goal_theta,
            "timestamp": time.time(),
        }
        try:
            self._goal_pub.send_json(goal_msg, flags=zmq.NOBLOCK)
            self._gui_nav_status.value = f"目标点已发布: ({self._goal_x:.2f}, {self._goal_y:.2f})"
            logger.info(f"目标点已发布: {goal_msg}")
        except Exception as e:
            self._gui_nav_status.value = f"发布失败: {e}"
            logger.warning(f"发布目标点失败: {e}")

        self._update_goal_marker()

    def _on_reset_odom(self) -> None:
        """发送里程计重置命令。"""
        result = self._send_cmd_req(self._odom_cmd, {"cmd": "reset_pose", "x": 0.0, "y": 0.0, "yaw": 0.0})
        self._gui_nav_status.value = result
        logger.info(result)

    def _on_reset_slam(self) -> None:
        """发送 SLAM 位姿重置命令。"""
        result = self._send_cmd_req(self._slam_cmd, {"cmd": "reset_pose", "x": 0.0, "y": 0.0, "theta": 0.0})
        self._gui_nav_status.value = result
        logger.info(result)

    def _send_cmd_req(self, sock: zmq.Socket, req: dict) -> str:
        """发送 REQ 命令并等待响应。"""
        try:
            sock.send_json(req)
            rep = sock.recv_json()
            if rep.get("success"):
                return f"✅ {rep.get('message', '成功')}"
            else:
                return f"❌ {rep.get('message', '失败')}"
        except zmq.Again:
            return "❌ 请求超时"
        except Exception as e:
            return f"❌ 请求异常: {e}"

    def _update_goal_marker(self) -> None:
        """在场景中更新目标点标记。"""
        s = self._server.scene
        x, y, theta = self._goal_x, self._goal_y, self._goal_theta

        # 目标点坐标系
        name_frame = "/map/goal"
        h = self._handles.get(name_frame)
        if h is None:
            self._handles[name_frame] = s.add_frame(
                name_frame, show_axes=True, axes_length=0.2, axes_radius=0.015
            )
        self._handles[name_frame].position = np.array([x, y, 0.0])
        self._handles[name_frame].wxyz = yaw_to_wxyz(theta)

        # 目标点圆柱标记
        name_body = "/map/goal/body"
        if self._handles.get(name_body) is None:
            s.add_cylinder(
                name_body,
                radius=0.08,
                height=0.15,
                position=(0.0, 0.0, 0.075),
                color=(255, 50, 50),
            )

        # 目标点方向箭头
        name_arrow = "/map/goal/arrow"
        if self._handles.get(name_arrow) is None:
            s.add_cylinder(
                name_arrow,
                radius=0.02,
                height=0.15,
                position=(0.08, 0.0, 0.05),
                wxyz=tf.SO3.from_y_radians(math.pi / 2).wxyz,
                color=(255, 200, 50),
            )

        # 目标点标签
        name_label = "/map/goal/label"
        h_label = self._handles.get(name_label)
        theta_deg = math.degrees(theta)
        text = f"Goal: ({x:.2f}, {y:.2f}, {theta_deg:.1f}°)"
        if h_label is None:
            self._handles[name_label] = s.add_label(
                name_label, text=text, position=(0.0, 0.0, 0.35)
            )
        else:
            h_label.text = text
            h_label.position = np.array([0.0, 0.0, 0.35])

        # 交互式拖拽控制器（首次创建时绑定回调）
        name_tc = "/map/goal/_tc"
        if self._handles.get(name_tc) is None:
            tc = s.add_transform_controls(
                name_tc,
                scale=1.2,
                active_axes=(True, True, False),                  # 只启用 X、Y 轴平移
                disable_sliders=True,                             # 禁用平面滑块
                disable_rotations=False,                          # 启用旋转
                rotation_limits=((0, 0), (0, 0), (-1000, 1000)),  # 只允许绕 Z 轴旋转
                depth_test=False,                                 # 始终可见
            )
            tc.on_update(lambda _: self._on_goal_drag_update())
            tc.on_drag_end(lambda _: self._on_goal_drag_end())
            self._handles[name_tc] = tc

    def _quat_to_yaw(self, wxyz) -> float:
        """从四元数安全提取绕 Z 轴的旋转角度。"""
        wxyz = np.array(wxyz, dtype=np.float64)
        norm = np.linalg.norm(wxyz)
        if norm < 1e-6:
            return 0.0
        qw, qx, qy, qz = wxyz / norm
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _on_goal_drag_update(self) -> None:
        """拖拽目标点时实时更新内部状态和 GUI 数值（不移动场景节点）。"""
        tc = self._handles.get("/map/goal/_tc")
        goal = self._handles.get("/map/goal")
        if tc is None or goal is None:
            return
        # 角度 = goal 当前角度 + tc 局部增量
        goal_theta = self._quat_to_yaw(goal.wxyz)
        tc_theta = self._quat_to_yaw(tc.wxyz)
        self._goal_theta = math.atan2(
            math.sin(goal_theta + tc_theta), math.cos(goal_theta + tc_theta)
        )
        # 计算世界坐标：tc 的局部偏移需要先旋转 goal_theta 再叠加
        ct = math.cos(goal_theta)
        st = math.sin(goal_theta)
        dx = float(tc.position[0])
        dy = float(tc.position[1])
        self._goal_x = float(goal.position[0]) + dx * ct - dy * st
        self._goal_y = float(goal.position[1]) + dx * st + dy * ct
        self._gui_goal_x.value = round(self._goal_x, 2)
        self._gui_goal_y.value = round(self._goal_y, 2)
        self._gui_goal_theta.value = round(math.degrees(self._goal_theta), 1)

    def _on_goal_drag_end(self) -> None:
        """拖拽结束：同步场景节点到 on_update 记录的最终值。"""
        tc = self._handles.get("/map/goal/_tc")
        goal = self._handles.get("/map/goal")
        if tc is None or goal is None:
            return
        # 直接使用 on_update 中已经记录好的最终值
        # （on_drag_end 时 tc.position 可能已被 Viser 重置，不能重新计算）
        # 重置控件局部偏移和旋转
        tc.position = np.array([0.0, 0.0, 0.0])
        tc.wxyz = np.array([1.0, 0.0, 0.0, 0.0])
        # 同步 goal 到最终位置
        goal.position = np.array([self._goal_x, self._goal_y, 0.0])
        goal.wxyz = yaw_to_wxyz(self._goal_theta)
        # 更新标签
        self._update_goal_marker_from_state()
        # 更新 GUI
        self._gui_goal_x.value = round(self._goal_x, 2)
        self._gui_goal_y.value = round(self._goal_y, 2)
        self._gui_goal_theta.value = round(math.degrees(self._goal_theta), 1)
        self._gui_nav_status.value = "目标点已调整，点击 📍 设置目标点 发布"

    def _update_goal_marker_from_state(self) -> None:
        """仅更新场景标记（不重新创建 TransformControls）。"""
        s = self._server.scene
        x, y, theta = self._goal_x, self._goal_y, self._goal_theta
        name_frame = "/map/goal"
        h = self._handles.get(name_frame)
        if h is not None:
            h.position = np.array([x, y, 0.0])
            h.wxyz = yaw_to_wxyz(theta)
        name_label = "/map/goal/label"
        h_label = self._handles.get(name_label)
        if h_label is not None:
            theta_deg = math.degrees(theta)
            h_label.text = f"Goal: ({x:.2f}, {y:.2f}, {theta_deg:.1f}°)"
            h_label.position = np.array([0.0, 0.0, 0.35])


# ------------------------------------------------------------------------------
# 入口
# ------------------------------------------------------------------------------
def main():
    import argparse

    parser = argparse.ArgumentParser(description="HomeBot Viser SLAM 可视化器")
    parser.add_argument("--host", default="0.0.0.0", help="Viser 服务器绑定地址")
    parser.add_argument("--port", type=int, default=8080, help="Viser 服务器端口")
    parser.add_argument("--odom", default="tcp://localhost:5559", help="里程计 SUB 地址")
    parser.add_argument("--slam-pose", default="tcp://localhost:5563", help="SLAM 位姿 SUB 地址")
    parser.add_argument("--slam-map", default="tcp://localhost:5564", help="SLAM 地图 SUB 地址")
    parser.add_argument("--lidar-scan", default="tcp://localhost:5565", help="激光雷达扫描 SUB 地址")
    parser.add_argument("--vision", default="tcp://localhost:5560", help="摄像头图像 SUB 地址")
    parser.add_argument("--goal-pub", default=None, help="目标点 PUB 地址 (默认 tcp://*:5566)")
    parser.add_argument("--odom-cmd", default=None, help="里程计命令 REQ 地址 (默认 tcp://localhost:5567)")
    parser.add_argument("--slam-cmd", default=None, help="SLAM 命令 REQ 地址 (默认 tcp://localhost:5568)")
    parser.add_argument("--global-path", default=None, help="全局路径 SUB 地址 (默认 tcp://localhost:5569)")
    args = parser.parse_args()

    # 用命令行参数覆盖配置
    cfg = get_config()
    cfg.viser.host = args.host
    cfg.viser.port = args.port
    cfg.viser.odom_sub_addr = args.odom
    cfg.viser.slam_pose_sub_addr = args.slam_pose
    cfg.viser.slam_map_sub_addr = args.slam_map
    cfg.viser.lidar_scan_sub_addr = args.lidar_scan
    cfg.viser.vision_sub_addr = args.vision
    if args.goal_pub:
        cfg.viser.goal_pub_addr = args.goal_pub
    if args.odom_cmd:
        cfg.viser.odom_cmd_addr = args.odom_cmd
    if args.slam_cmd:
        cfg.viser.slam_cmd_addr = args.slam_cmd
    if args.global_path:
        cfg.viser.global_path_sub_addr = args.global_path

    visualizer = ViserSLAMVisualizer()
    visualizer.start()


if __name__ == "__main__":
    main()
