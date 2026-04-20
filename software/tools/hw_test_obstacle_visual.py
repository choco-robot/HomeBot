# -*- coding: utf-8 -*-
"""硬件测试：障碍物检测一维直方图可视化工具

功能：
  - 订阅 DepthService 发布的深度图和障碍物距离直方图
  - 左窗：RGB 原图
  - 右窗：深度图 + 距离直方图柱状叠加
  - 底部：20个条带的最近距离直方图曲线

Usage:
    cd software/src
    python ../tools/hw_test_obstacle_visual.py

依赖服务：
  - VisionService (tcp://localhost:5560) 提供 RGB 图像流
  - DepthService (tcp://localhost:5561) 提供伪彩色深度图
  - DepthService (tcp://localhost:5562) 提供障碍物距离直方图

按键：
  [Q/ESC] 退出    [S] 保存截图
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from common.logging import get_logger
from common.zmq_helper import create_socket

logger = get_logger(__name__)

DEFAULT_VISION_ADDR = "tcp://localhost:5560"
DEFAULT_DEPTH_ADDR = "tcp://localhost:5561"
DEFAULT_OBSTACLE_ADDR = "tcp://localhost:5562"


def put_text_bg(img, text, pos, font=cv2.FONT_HERSHEY_SIMPLEX, scale=0.5, color=(0, 255, 0), thickness=1):
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    x, y = pos
    cv2.rectangle(img, (x, y - th - 4), (x + tw, y + 4), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


class HistogramVisualizer:
    """实时订阅 DepthService 的深度图和障碍物直方图，可视化"""

    def __init__(
        self,
        vision_addr: str = DEFAULT_VISION_ADDR,
        depth_addr: str = DEFAULT_DEPTH_ADDR,
        obstacle_addr: str = DEFAULT_OBSTACLE_ADDR,
        display_size: tuple = (640, 480),
    ):
        self.vision_addr = vision_addr
        self.depth_addr = depth_addr
        self.obstacle_addr = obstacle_addr
        self.disp_w, self.disp_h = display_size

        # ZeroMQ 上下文
        self._ctx = zmq.Context()

        # 订阅 RGB
        self._rgb_sub = create_socket(zmq.SUB, bind=False, address=self.vision_addr)
        self._rgb_sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._rgb_sub.setsockopt(zmq.RCVTIMEO, 500)

        # 订阅深度图（伪彩色 JPEG）
        self._depth_sub = create_socket(zmq.SUB, bind=False, address=self.depth_addr)
        self._depth_sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._depth_sub.setsockopt(zmq.RCVTIMEO, 500)

        # 订阅障碍物直方图（JSON）
        self._obstacle_sub = create_socket(zmq.SUB, bind=False, address=self.obstacle_addr)
        self._obstacle_sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._obstacle_sub.setsockopt(zmq.RCVTIMEO, 500)

        # 最新数据锁
        self._data_lock = threading.Lock()
        self._latest_rgb: np.ndarray | None = None
        self._latest_depth_color: np.ndarray | None = None
        self._latest_histogram: np.ndarray | None = None
        self._latest_estimator_type: str = ""
        self._latest_inference_ms: float = 0.0
        self._latest_frame_id: int = -1

        self._running = False
        self._recv_threads: list[threading.Thread] = []

    def start(self) -> None:
        self._running = True

        # 启动三个接收线程
        for target in (self._receive_rgb_loop, self._receive_depth_loop, self._receive_obstacle_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._recv_threads.append(t)

        cv2.namedWindow("Histogram Visualizer", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Histogram Visualizer", 1280, 800)

        try:
            while self._running:
                canvas = self._compose_frame()
                if canvas is not None:
                    cv2.imshow("Histogram Visualizer", canvas)

                key = cv2.waitKey(30) & 0xFF
                if key == 27 or key == ord('q'):
                    break
                if key == ord('s'):
                    fname = f"hist_debug_{time.strftime('%H%M%S')}.png"
                    cv2.imwrite(fname, canvas)
                    print(f"[截图已保存] {fname}")

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def _receive_rgb_loop(self) -> None:
        while self._running:
            try:
                parts = self._rgb_sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) >= 2:
                    frame_id = int(parts[0].decode())
                    buf = np.frombuffer(parts[1], dtype=np.uint8)
                    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self._data_lock:
                            self._latest_rgb = frame
                            self._latest_frame_id = frame_id
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"RGB接收异常: {e}")
            time.sleep(0.001)

    def _receive_depth_loop(self) -> None:
        while self._running:
            try:
                parts = self._depth_sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) >= 2:
                    buf = np.frombuffer(parts[1], dtype=np.uint8)
                    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self._data_lock:
                            self._latest_depth_color = frame
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"深度图接收异常: {e}")
            time.sleep(0.001)

    def _receive_obstacle_loop(self) -> None:
        while self._running:
            try:
                parts = self._obstacle_sub.recv_multipart(flags=zmq.NOBLOCK)
                if len(parts) >= 2:
                    data = json.loads(parts[1].decode("utf-8"))
                    hist_list = data.get("histogram", [])
                    # 将 None 还原为 inf
                    histogram = np.array([
                        float(v) if v is not None else math.inf
                        for v in hist_list
                    ], dtype=np.float32)
                    with self._data_lock:
                        self._latest_histogram = histogram
                        self._latest_estimator_type = data.get("estimator_type", "")
                        self._latest_inference_ms = data.get("inference_ms", 0.0)
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"直方图接收异常: {e}")
            time.sleep(0.001)

    def _compose_frame(self) -> np.ndarray | None:
        with self._data_lock:
            rgb = self._latest_rgb
            depth_color = self._latest_depth_color
            histogram = self._latest_histogram
            estimator_type = self._latest_estimator_type
            inference_ms = self._latest_inference_ms
            frame_id = self._latest_frame_id

        if rgb is None:
            canvas = np.zeros((self.disp_h, self.disp_w * 2, 3), dtype=np.uint8)
            cv2.putText(canvas, "Waiting for RGB stream...", (400, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            return canvas

        rgb_disp = cv2.resize(rgb, (self.disp_w, self.disp_h))

        # 左上：RGB
        left = rgb_disp.copy()
        put_text_bg(left, f"[F]DepthSrc  [S]Save  [Q]Quit", (10, 30), scale=0.6)

        # 右上：深度图 + 直方图柱状叠加
        if depth_color is not None:
            right = cv2.resize(depth_color, (self.disp_w, self.disp_h))
            right = self._draw_histogram_on_depth(right, histogram)
        else:
            right = np.zeros_like(left)
            cv2.putText(right, "No depth", (self.disp_w // 2 - 80, self.disp_h // 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        info_text = f"id:{frame_id}  {inference_ms:.1f}ms  {estimator_type[:10]}"
        put_text_bg(right, info_text, (10, 30), scale=0.6)

        top = np.hstack([left, right])

        # 底部：直方图曲线
        bottom = self._draw_histogram_curve(histogram, width=self.disp_w * 2, height=200)

        canvas = np.vstack([top, bottom])
        return canvas

    def _draw_histogram_on_depth(self, depth_bgr: np.ndarray, histogram: np.ndarray | None) -> np.ndarray:
        """在深度图上叠加距离直方图柱状条"""
        overlay = depth_bgr.copy()
        if histogram is None or len(histogram) == 0:
            return overlay

        h, w = overlay.shape[:2]
        n = len(histogram)
        col_w = w // n
        max_dist = 3.0  # 最大显示距离 3m

        for i in range(n):
            dist = float(histogram[i])
            if np.isinf(dist):
                continue
            x = i * col_w
            cx = x + col_w // 2
            # 柱子高度：距离越近，柱子越高（从底部向上）
            bar_h = int(min(dist, max_dist) / max_dist * (h * 0.4))
            y1 = h - bar_h
            # 颜色：近=红，远=绿
            ratio = min(dist, max_dist) / max_dist
            color = (0, int(255 * ratio), int(255 * (1 - ratio)))
            cv2.rectangle(overlay, (x + 2, y1), (x + col_w - 2, h), color, -1)
            # 标注距离值
            if bar_h > 15:
                put_text_bg(overlay, f"{dist:.1f}", (x + 2, h - 5), color=(255, 255, 255), scale=0.3)

        return overlay

    def _draw_histogram_curve(self, histogram: np.ndarray | None, width: int, height: int) -> np.ndarray:
        """绘制底部直方图曲线"""
        panel = np.zeros((height, width, 3), dtype=np.uint8)
        if histogram is None or len(histogram) == 0:
            cv2.putText(panel, "No histogram data", (width // 2 - 100, height // 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            return panel

        n = len(histogram)
        max_dist = 3.0
        margin = 40
        plot_w = width - margin * 2
        plot_h = height - margin * 2
        col_w = plot_w // n

        # 标题
        cv2.putText(panel, "Distance Histogram (x=column/direction, y=distance in meters)",
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # 坐标轴
        cv2.line(panel, (margin, height - margin), (width - margin, height - margin), (128, 128, 128), 1)
        cv2.line(panel, (margin, margin), (margin, height - margin), (128, 128, 128), 1)

        # Y轴刻度
        for d in [0, 1, 2, 3]:
            y = int(height - margin - d / max_dist * plot_h)
            cv2.line(panel, (margin - 5, y), (margin, y), (128, 128, 128), 1)
            cv2.putText(panel, f"{d}m", (5, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

        # 画柱状图
        for i in range(n):
            dist = float(histogram[i])
            if np.isinf(dist):
                continue
            x = margin + i * col_w
            bar_h = int(min(dist, max_dist) / max_dist * plot_h)
            y = height - margin - bar_h
            ratio = min(dist, max_dist) / max_dist
            color = (0, int(255 * ratio), int(255 * (1 - ratio)))
            cv2.rectangle(panel, (x + 1, y), (x + col_w - 1, height - margin), color, -1)

        # 画安全距离线 (0.5m)
        safe_y = int(height - margin - 0.5 / max_dist * plot_h)
        cv2.line(panel, (margin, safe_y), (width - margin, safe_y), (0, 0, 255), 2)
        cv2.putText(panel, "safety=0.5m", (width - margin - 80, safe_y - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        # X轴标签
        for i in range(0, n, 5):
            x = margin + i * col_w
            cv2.putText(panel, str(i), (x, height - margin + 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

        return panel

    def stop(self) -> None:
        self._running = False
        for t in self._recv_threads:
            t.join(timeout=1.0)
        self._rgb_sub.close()
        self._depth_sub.close()
        self._obstacle_sub.close()
        self._ctx.term()
        cv2.destroyAllWindows()
        logger.info("可视化工具已停止")


def main():
    parser = argparse.ArgumentParser(description="HomeBot 障碍物检测直方图可视化工具（订阅深度服务）")
    parser.add_argument("--vision", default=DEFAULT_VISION_ADDR, help="VisionService RGB 地址")
    parser.add_argument("--depth", default=DEFAULT_DEPTH_ADDR, help="DepthService 深度图 PUB 地址")
    parser.add_argument("--obstacle", default=DEFAULT_OBSTACLE_ADDR, help="DepthService 障碍物直方图 PUB 地址")
    args = parser.parse_args()

    print("=" * 60)
    print("HomeBot 障碍物检测一维直方图可视化")
    print("=" * 60)
    print("模式：订阅 DepthService（零本地推理）")
    print("")
    print(f"  VisionService  RGB : {args.vision}")
    print(f"  DepthService  深度图: {args.depth}")
    print(f"  DepthService  直方图: {args.obstacle}")
    print("")
    print("窗口布局：")
    print("  左上 = RGB 原图")
    print("  右上 = 深度图 + 距离柱状图叠加（红=近，绿=远）")
    print("  底部 = 20个条带的最近距离直方图曲线")
    print("")
    print("按键：")
    print("  [Q/ESC] 退出    [S] 保存截图")
    print("=" * 60)

    viz = HistogramVisualizer(
        vision_addr=args.vision,
        depth_addr=args.depth,
        obstacle_addr=args.obstacle,
    )
    viz.start()


if __name__ == "__main__":
    main()
