# -*- coding: utf-8 -*-
"""SLAM 融合定位服务

整合 LD06 激光雷达、里程计、AprilTag 视觉定位，
通过 BreezySLAM + AprilTag 融合算法输出高精度位姿和栅格地图。

通信接口:
    SUB tcp://localhost:5560   VisionService 图像流
    SUB tcp://localhost:5559   OdomService 里程计
    PUB tcp://*:5563           SLAM 位姿 (x, y, theta, covariance, status)
    PUB tcp://*:5564           SLAM 栅格地图 (bytearray)
"""
from __future__ import annotations

import atexit
import math
import signal
import sys
import time
from pathlib import Path
from threading import Lock, Thread
from typing import Optional, Tuple

import cv2
import numpy as np
import zmq

from common.logging import get_logger
from common.zmq_helper import create_socket
from configs import get_config

logger = get_logger(__name__)

_DEFAULT_CONFIG = get_config()

DEFAULT_VISION_ADDR = _DEFAULT_CONFIG.viser.vision_sub_addr
DEFAULT_ODOM_ADDR = _DEFAULT_CONFIG.viser.odom_sub_addr
DEFAULT_SLAM_POSE_ADDR = _DEFAULT_CONFIG.zmq.slam_pose_pub_addr
DEFAULT_SLAM_MAP_ADDR = _DEFAULT_CONFIG.zmq.slam_map_pub_addr
DEFAULT_LIDAR_SCAN_ADDR = _DEFAULT_CONFIG.zmq.lidar_scan_pub_addr
DEFAULT_SLAM_CMD_ADDR = "tcp://*:5568"


class SLAMService:
    """SLAM 融合定位服务主类。

    三线程架构:
        - 主循环线程 (10Hz): 雷达→SLAM更新→融合→发布位姿
        - 图像接收线程: 订阅 VisionService，保留最新帧
        - 里程计接收线程: 订阅 OdomService，保留最新里程计
    """

    def __init__(
        self,
        vision_addr: str = DEFAULT_VISION_ADDR,
        odom_addr: str = DEFAULT_ODOM_ADDR,
        pose_pub_addr: str = DEFAULT_SLAM_POSE_ADDR,
        map_pub_addr: str = DEFAULT_SLAM_MAP_ADDR,
        lidar_scan_pub_addr: str = DEFAULT_LIDAR_SCAN_ADDR,
        slam_cmd_addr: str = DEFAULT_SLAM_CMD_ADDR,
        lidar_port: Optional[str] = None,
        scan_size: Optional[int] = None,
        map_size_pixels: Optional[int] = None,
        map_size_meters: Optional[float] = None,
        tag_map: Optional[dict] = None,
        camera_matrix: Optional[np.ndarray] = None,
        tag_size_m: Optional[float] = None,
        publish_rate_hz: Optional[float] = None,
        load_map_path: Optional[str] = None,
        save_map_path: Optional[str] = None,
        export_editor_path: Optional[str] = None,
        init_x: Optional[float] = None,
        init_y: Optional[float] = None,
        init_theta: Optional[float] = None,
    ):
        config = get_config()
        slam_cfg = config.slam

        self.vision_addr = vision_addr
        self.odom_addr = odom_addr
        self.pose_pub_addr = pose_pub_addr
        self.map_pub_addr = map_pub_addr
        self.lidar_scan_pub_addr = lidar_scan_pub_addr
        self.slam_cmd_addr = slam_cmd_addr
        self.publish_interval = 1.0 / (publish_rate_hz if publish_rate_hz is not None else 10.0)

        # 地图加载/保存参数
        self._load_map_path = load_map_path
        self._save_map_path = save_map_path
        self._export_editor_path = export_editor_path
        self._init_x = init_x
        self._init_y = init_y
        self._init_theta = init_theta

        # ------------------------------------------------------------------
        # ZeroMQ sockets
        # ------------------------------------------------------------------
        # 图像订阅
        self._vision_sub = create_socket(zmq.SUB, bind=False, address=self.vision_addr)
        self._vision_sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._vision_sub.setsockopt(zmq.RCVTIMEO, 500)
        self._vision_sub.setsockopt(zmq.CONFLATE, 1)
        logger.info(f"SLAMService Vision SUB: {self.vision_addr}")

        # 里程计订阅
        self._odom_sub = create_socket(zmq.SUB, bind=False, address=self.odom_addr)
        self._odom_sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._odom_sub.setsockopt(zmq.RCVTIMEO, 500)
        self._odom_sub.setsockopt(zmq.CONFLATE, 1)
        logger.info(f"SLAMService Odom SUB: {self.odom_addr}")

        # 位姿发布
        self._pose_pub = create_socket(zmq.PUB, bind=True, address=self.pose_pub_addr)
        logger.info(f"SLAMService Pose PUB: {self.pose_pub_addr}")

        # 地图发布（低频，如 0.5Hz）
        self._map_pub = create_socket(zmq.PUB, bind=True, address=self.map_pub_addr)
        logger.info(f"SLAMService Map PUB: {self.map_pub_addr}")

        # 激光雷达扫描数据发布（供可视化器订阅）
        self._lidar_scan_pub = create_socket(zmq.PUB, bind=True, address=self.lidar_scan_pub_addr)
        logger.info(f"SLAMService LidarScan PUB: {self.lidar_scan_pub_addr}")

        # REP socket 接收命令（重置位姿等）
        self._cmd_socket = create_socket(zmq.REP, bind=True, address=self.slam_cmd_addr)
        logger.info(f"SLAMService REP cmd: {self.slam_cmd_addr}")

        # ------------------------------------------------------------------
        # 雷达驱动
        # ------------------------------------------------------------------
        lidar_port = lidar_port if lidar_port is not None else slam_cfg.lidar_port
        scan_size = scan_size if scan_size is not None else slam_cfg.lidar_scan_size
        map_size_pixels = map_size_pixels if map_size_pixels is not None else slam_cfg.map_size_pixels
        map_size_meters = map_size_meters if map_size_meters is not None else slam_cfg.map_size_meters
        tag_size_m = tag_size_m if tag_size_m is not None else slam_cfg.tag_size_m
        tag_map = tag_map if tag_map is not None else slam_cfg.tag_map

        if camera_matrix is None:
            camera_matrix = np.array([
                [slam_cfg.camera_fx, 0.0, slam_cfg.camera_cx],
                [0.0, slam_cfg.camera_fy, slam_cfg.camera_cy],
                [0.0, 0.0, 1.0],
            ], dtype=np.float64)

        from navigation.hal.lidar_driver import create_lidar_driver
        self._lidar = create_lidar_driver(
            port=lidar_port,
            scan_size=scan_size,
            min_distance_m=slam_cfg.lidar_min_distance_m,
        )

        # ------------------------------------------------------------------
        # SLAM 融合核心
        # ------------------------------------------------------------------
        from navigation.core.slam_fusion import SLAMFusion
        self._fusion = SLAMFusion(
            map_size_pixels=map_size_pixels,
            map_size_meters=map_size_meters,
            scan_size=scan_size,
        )

        # ------------------------------------------------------------------
        # 地图加载与初始位姿设置
        # ------------------------------------------------------------------
        self._setup_map_and_pose()

        # ------------------------------------------------------------------
        # AprilTag 检测器
        # ------------------------------------------------------------------
        from navigation.perception.apriltag_detector import create_apriltag_detector
        self._tag_detector = create_apriltag_detector(
            camera_matrix=camera_matrix,
            tag_map=tag_map,
            tag_size_m=tag_size_m,
        )

        # ------------------------------------------------------------------
        # 共享内存（最新数据）
        # ------------------------------------------------------------------
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_odom: Optional[Tuple[float, float, float, float]] = None  # x, y, yaw, ts

        self._frame_lock = Lock()
        self._odom_lock = Lock()

        self._running = False
        self._vision_thread: Optional[Thread] = None
        self._odom_thread: Optional[Thread] = None

        # 运行模式
        self._localization_only = False

        # 统计
        self._loop_count = 0
        self._tag_detect_count = 0
        self._last_log_time = time.time()
        self._last_map_pub_time = 0.0

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def _setup_map_and_pose(self) -> None:
        """根据构造参数加载地图并设置初始位姿。"""
        # 1. 加载已有地图
        if self._load_map_path:
            try:
                self._fusion.load_map(self._load_map_path)
            except Exception as e:
                logger.error(f"加载地图失败: {e}")

        # 2. 设置初始位姿
        init_pose = None
        if self._init_x is not None and self._init_y is not None and self._init_theta is not None:
            init_pose = (self._init_x, self._init_y, self._init_theta)
        elif self._load_map_path:
            # 若未手动指定，尝试读取地图中保存的位姿
            saved = self._fusion.get_saved_pose(self._load_map_path)
            if saved:
                init_pose = saved
                logger.info(f"使用地图保存的位姿作为初始位姿: ({saved[0]:.3f}, {saved[1]:.3f}, {math.degrees(saved[2]):.2f}°)")

        if init_pose:
            self._fusion.set_initial_pose(*init_pose)

        # 3. 注册退出保存（如指定了 save_map_path）
        if self._save_map_path:
            atexit.register(self._save_map_on_exit)
            # 同时捕获 SIGINT/SIGTERM 做优雅保存
            for sig in (signal.SIGINT, signal.SIGTERM):
                signal.signal(sig, self._signal_handler)

    def _save_map_on_exit(self) -> None:
        """服务退出时自动保存地图。"""
        # 1. 保存 .npz（如有指定）
        if self._save_map_path:
            try:
                path = Path(self._save_map_path)
                path.parent.mkdir(parents=True, exist_ok=True)
                self._fusion.save_map(str(path))
                logger.info(f"退出时自动保存地图成功: {path}")
            except Exception as e:
                logger.error(f"退出保存 .npz 地图失败: {e}")

        # 2. 导出编辑器格式（如有指定）
        if self._export_editor_path:
            try:
                path = Path(self._export_editor_path)
                path.parent.mkdir(parents=True, exist_ok=True)
                self._fusion.export_to_editor_format(str(path.parent), path.stem)
                logger.info(f"退出时导出编辑器格式地图成功: {path.parent}/{path.stem}.png+json")
            except Exception as e:
                logger.error(f"退出导出编辑器格式地图失败: {e}")

    def _signal_handler(self, signum, frame) -> None:
        """信号处理：保存地图后退出。"""
        logger.info(f"接收到信号 {signum}，准备保存地图并退出...")
        self._save_map_on_exit()
        self.stop()
        sys.exit(0)

    def start(self) -> None:
        """启动 SLAM 服务。无可用雷达时直接结束并提示错误。"""
        self._running = True
        logger.info("SLAMService 启动中...")

        # 启动雷达
        try:
            self._lidar.start()
        except Exception as e:
            logger.error(f"雷达启动失败，服务终止: {e}")
            self.stop()
            sys.exit(1)

        # 启动后台接收线程
        self._vision_thread = Thread(target=self._vision_loop, daemon=True)
        self._vision_thread.start()
        self._odom_thread = Thread(target=self._odom_loop, daemon=True)
        self._odom_thread.start()

        # 等待接收线程初始化
        time.sleep(0.3)

        # 主循环
        logger.info("SLAMService 主循环启动 (10Hz)")
        try:
            loop_counter = 0
            while self._running:
                t0 = time.perf_counter()

                # 1. 获取最新雷达数据
                scan = self._lidar.get_scan()
                if scan is None:
                    time.sleep(0.01)
                    continue

                angles_deg, distances_mm = scan

                # 1.5 发布激光雷达扫描数据（供可视化器使用）
                self._publish_lidar_scan(angles_deg, distances_mm)

                # 2. 获取最新里程计
                with self._odom_lock:
                    odom = self._latest_odom

                # 3. SLAM Lidar 更新
                self._fusion.update_lidar(angles_deg, distances_mm, odom, update_map=not self._localization_only)

                # 4. 每 5 轮（2Hz）执行 AprilTag 检测与融合
                loop_counter += 1
                if loop_counter >= 5:
                    loop_counter = 0
                    self._process_apriltag()

                # 5. 发布位姿
                self._publish_pose()

                # 6. 低频发布地图（0.5Hz）
                now = time.time()
                if now - self._last_map_pub_time >= 2.0:
                    self._publish_map()
                    self._last_map_pub_time = now

                # 7. 处理命令请求（非阻塞轮询）
                self._handle_command()

                # 8. 统计与帧率控制
                self._update_stats()
                elapsed = time.perf_counter() - t0
                rem = self.publish_interval - elapsed
                if rem > 0:
                    time.sleep(rem)

        except KeyboardInterrupt:
            logger.info("SLAMService 被用户中断")
        except Exception as e:
            logger.error(f"SLAMService 主循环异常: {e}")
            import traceback
            logger.error(traceback.format_exc())
        finally:
            self.stop()

    def _handle_command(self) -> None:
        """处理 REP 命令请求（非阻塞）。"""
        try:
            req = self._cmd_socket.recv_json(flags=zmq.NOBLOCK)
            cmd = req.get("cmd", "")
            if cmd == "reset_pose":
                x = req.get("x", 0.0)
                y = req.get("y", 0.0)
                theta = req.get("theta", 0.0)
                self._fusion.reset_pose(x, y, theta)
                self._cmd_socket.send_json({"success": True, "message": f"SLAM 已重置为 ({x}, {y}, {theta})"})
            elif cmd == "set_mode":
                mode = req.get("mode", "")
                if mode in ("localization_only", "navigation"):
                    self._localization_only = True
                    self._cmd_socket.send_json({"success": True, "message": "已切换到纯定位模式"})
                elif mode == "mapping":
                    self._localization_only = False
                    self._cmd_socket.send_json({"success": True, "message": "已切换到建图模式"})
                else:
                    self._cmd_socket.send_json({"success": False, "message": f"未知模式: {mode}"})
            else:
                self._cmd_socket.send_json({"success": False, "message": f"未知命令: {cmd}"})
        except zmq.Again:
            pass
        except Exception as e:
            logger.warning(f"处理命令失败: {e}")
            try:
                self._cmd_socket.send_json({"success": False, "message": str(e)})
            except Exception:
                pass

    def stop(self) -> None:
        """停止服务。"""
        self._running = False
        if self._lidar:
            self._lidar.stop()
        for t in (self._vision_thread, self._odom_thread):
            if t:
                t.join(timeout=1.0)
        for sock in (self._vision_sub, self._odom_sub, self._pose_pub, self._map_pub, self._lidar_scan_pub, self._cmd_socket):
            if sock:
                sock.close()
        logger.info("SLAMService 已停止")

    # ------------------------------------------------------------------
    # 后台接收线程
    # ------------------------------------------------------------------
    def _vision_loop(self) -> None:
        """持续接收 VisionService 图像，保留最新帧。"""
        logger.info("SLAMService 图像接收线程已启动")
        while self._running:
            try:
                parts = self._vision_sub.recv_multipart()
                if len(parts) >= 2:
                    buf = np.frombuffer(parts[1], dtype=np.uint8)
                    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self._frame_lock:
                            self._latest_frame = frame
            except zmq.Again:
                continue
            except Exception as e:
                logger.warning(f"接收图像异常: {e}")
                time.sleep(0.01)

    def _odom_loop(self) -> None:
        """持续接收 OdomService 里程计，保留最新数据。"""
        logger.info("SLAMService 里程计接收线程已启动")
        _first_odom = True
        while self._running:
            try:
                msg = self._odom_sub.recv_json(flags=zmq.NOBLOCK)
                if msg:
                    x = msg.get("x", 0.0)
                    y = msg.get("y", 0.0)
                    yaw = msg.get("yaw", 0.0)
                    ts = msg.get("timestamp", time.time())
                    with self._odom_lock:
                        self._latest_odom = (x, y, yaw, ts)
                    if _first_odom:
                        _first_odom = False
                        self._fusion.reset_odom((x, y, yaw, ts))
                        logger.info(f"里程计基准已初始化: ({x:.3f}, {y:.3f}, {yaw:.3f})")
            except zmq.Again:
                time.sleep(0.005)
            except Exception as e:
                logger.warning(f"接收里程计异常: {e}")
                time.sleep(0.01)

    # ------------------------------------------------------------------
    # AprilTag 处理
    # ------------------------------------------------------------------
    def _process_apriltag(self) -> None:
        """检测 AprilTag 并更新融合核心。"""
        with self._frame_lock:
            frame = self._latest_frame.copy() if self._latest_frame is not None else None

        if frame is None:
            return

        detections = self._tag_detector.detect(frame)
        if detections:
            self._tag_detect_count += 1
            self._fusion.update_apriltag(detections)

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------
    def _publish_pose(self) -> None:
        """发布融合位姿和状态。"""
        x, y, theta, P = self._fusion.get_pose()
        status = self._fusion.get_status()

        msg = {
            "x": float(round(x, 4)),
            "y": float(round(y, 4)),
            "theta": float(round(theta, 4)),
            "covariance": P.tolist(),
            "state": status["state"],
            "slam_fail_count": status["slam_fail_count"],

            "timestamp": time.time(),
        }
        try:
            self._pose_pub.send_json(msg, flags=zmq.NOBLOCK)
        except zmq.Again:
            pass

    def _publish_lidar_scan(self, angles_deg, distances_mm) -> None:
        """发布激光雷达扫描数据（JSON格式，供可视化器订阅）。"""
        try:
            # 将数据转为米，并过滤无效值
            # 注意：angles_deg 和 distances_mm 可能是 list 或 np.ndarray
            import numpy as np
            if not isinstance(distances_mm, np.ndarray):
                distances_mm = np.array(distances_mm, dtype=np.float32)
            distances_m = distances_mm.astype(np.float32) / 1000.0
            if not isinstance(angles_deg, np.ndarray):
                angles_deg = np.array(angles_deg, dtype=np.float32)
            msg = {
                "angles_deg": angles_deg.tolist(),
                "distances_m": distances_m.tolist(),
                "timestamp": time.time(),
            }
            self._lidar_scan_pub.send_json(msg, flags=zmq.NOBLOCK)
        except zmq.Again:
            pass
        except Exception as e:
            logger.warning(f"发布激光雷达扫描数据失败: {e}")

    def _publish_map(self) -> None:
        """发布栅格地图字节数组。"""
        try:
            mapbytes = self._fusion.get_map_bytes()
            # 附带地图元信息
            meta = {
                "size_pixels": self._fusion.map_size_pixels,
                "size_meters": self._fusion.map_size_meters,
                "timestamp": time.time(),
            }
            self._map_pub.send_json(meta, flags=zmq.SNDMORE)
            self._map_pub.send(mapbytes, flags=zmq.NOBLOCK)
        except zmq.Again:
            pass
        except Exception as e:
            logger.warning(f"发布地图失败: {e}")

    # ------------------------------------------------------------------
    # 统计
    # ------------------------------------------------------------------
    def _update_stats(self) -> None:
        self._loop_count += 1
        now = time.time()
        if now - self._last_log_time >= 5.0:
            fps = self._loop_count / (now - self._last_log_time)
            x, y, theta, _ = self._fusion.get_pose()
            logger.info(
                f"SLAMService 性能: {fps:.1f} Hz, "
                f"位姿=({x:.3f}, {y:.3f}, {math.degrees(theta):.2f}°), "
                f"状态={self._fusion.state}, "
                f"标签检测={self._tag_detect_count}"
            )
            self._loop_count = 0
            self._tag_detect_count = 0
            self._last_log_time = now


# ------------------------------------------------------------------------------
# 入口
# ------------------------------------------------------------------------------

def main():
    import argparse
    parser = argparse.ArgumentParser(description="HomeBot SLAM 融合定位服务")
    parser.add_argument("--vision", default=DEFAULT_VISION_ADDR, help="VisionService 地址")
    parser.add_argument("--odom", default=DEFAULT_ODOM_ADDR, help="OdomService 地址")
    parser.add_argument("--pose-pub", default=DEFAULT_SLAM_POSE_ADDR, help="位姿 PUB 地址")
    parser.add_argument("--map-pub", default=DEFAULT_SLAM_MAP_ADDR, help="地图 PUB 地址")
    parser.add_argument("--lidar-scan-pub", default=DEFAULT_LIDAR_SCAN_ADDR, help="激光雷达扫描 PUB 地址")
    parser.add_argument("--lidar-port", default=None, help="雷达串口")
    parser.add_argument("--rate", type=float, default=None, help="主循环频率 Hz")
    parser.add_argument("--load-map", default=None, help="启动时加载已有地图 (.npz)")
    parser.add_argument("--save-map", default=None, help="退出时保存地图到指定路径 (.npz)")
    parser.add_argument("--export-editor", default=None, help="退出时导出编辑器格式地图 (PNG+JSON) 到指定目录/前缀")
    parser.add_argument("--init-x", type=float, default=None, help="初始位姿 X (m)")
    parser.add_argument("--init-y", type=float, default=None, help="初始位姿 Y (m)")
    parser.add_argument("--init-theta", type=float, default=None, help="初始位姿朝向 (rad)")
    args = parser.parse_args()

    service = SLAMService(
        vision_addr=args.vision,
        odom_addr=args.odom,
        pose_pub_addr=args.pose_pub,
        map_pub_addr=args.map_pub,
        lidar_scan_pub_addr=args.lidar_scan_pub,
        lidar_port=args.lidar_port,
        publish_rate_hz=args.rate,
        load_map_path=args.load_map,
        save_map_path=args.save_map,
        export_editor_path=args.export_editor,
        init_x=args.init_x,
        init_y=args.init_y,
        init_theta=args.init_theta,
    )
    service.start()


if __name__ == "__main__":
    main()
