# -*- coding: utf-8 -*-
"""硬件测试：里程计轨迹可视化工具

功能：
  - 订阅 OdomService 发布的里程计数据
  - 实时绘制机器人运动轨迹（俯视图）
  - 显示当前位姿、速度、轨迹长度等信息

Usage:
    cd software/src
    python ../tools/hw_test_odom_visual.py

依赖服务：
  - OdomService (tcp://localhost:5559) 提供里程计数据

按键：
  [Q/ESC] 退出      [R] 重置/清除轨迹
  [S] 保存截图      [+] 放大    [-] 缩小
  [↑↓←→] 平移视图   [0] 恢复自动缩放
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from common.logging import get_logger
from common.zmq_helper import create_socket

logger = get_logger(__name__)

DEFAULT_ODOM_ADDR = "tcp://localhost:5559"
MAX_TRAIL_POINTS = 5000  # 最大轨迹点数
CANVAS_SIZE = (900, 900)  # (width, height)


def put_text_bg(
    img,
    text,
    pos,
    font=cv2.FONT_HERSHEY_SIMPLEX,
    scale=0.5,
    color=(0, 255, 0),
    thickness=1,
    bg_color=(0, 0, 0),
):
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    x, y = pos
    cv2.rectangle(img, (x, y - th - 4), (x + tw, y + 4), bg_color, -1)
    cv2.putText(img, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


class OdomVisualizer:
    """订阅里程计数据，实时可视化轨迹"""

    def __init__(
        self,
        odom_addr: str = DEFAULT_ODOM_ADDR,
        canvas_size: tuple = CANVAS_SIZE,
        max_trail: int = MAX_TRAIL_POINTS,
    ):
        self.odom_addr = odom_addr
        self.canvas_w, self.canvas_h = canvas_size
        self.max_trail = max_trail

        # ZeroMQ 订阅里程计
        self._ctx = zmq.Context()
        self._odom_sub = create_socket(zmq.SUB, bind=False, address=self.odom_addr)
        self._odom_sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._odom_sub.setsockopt(zmq.RCVTIMEO, 500)

        # 数据锁
        self._lock = threading.Lock()
        self._trail: deque[tuple[float, float]] = deque(maxlen=max_trail)  # (x, y)
        self._current_pose: dict = {}
        self._receive_count = 0
        self._last_recv_time = 0.0

        # 视图参数
        self._zoom = 50.0  # 像素/米
        self._offset_x = self.canvas_w // 2
        self._offset_y = self.canvas_h // 2
        self._auto_fit = True
        self._margin_m = 0.5  # 自动缩放边距（米）

        self._running = False
        self._recv_thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

        cv2.namedWindow("Odom Trajectory", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Odom Trajectory", self.canvas_w, self.canvas_h)

        try:
            while self._running:
                canvas = self._draw_canvas()
                cv2.imshow("Odom Trajectory", canvas)

                key = cv2.waitKey(30) & 0xFF
                if key == 27 or key == ord("q"):
                    break
                if key == ord("r"):
                    with self._lock:
                        self._trail.clear()
                        self._current_pose = {}
                    print("[轨迹已重置]")
                if key == ord("s"):
                    fname = f"odom_{time.strftime('%H%M%S')}.png"
                    cv2.imwrite(fname, canvas)
                    print(f"[截图已保存] {fname}")
                if key == ord("+") or key == ord("="):
                    self._zoom *= 1.2
                    self._auto_fit = False
                if key == ord("-"):
                    self._zoom *= 0.8
                    self._auto_fit = False
                if key == ord("0"):
                    self._auto_fit = True
                    print("[自动缩放开启]")
                # 方向键平移（OpenCV waitKey 不直接支持方向键，用 WASD 替代）
                if key == ord("w"):
                    self._offset_y += int(self.canvas_h * 0.1)
                    self._auto_fit = False
                if key == ord("s"):
                    self._offset_y -= int(self.canvas_h * 0.1)
                    self._auto_fit = False
                if key == ord("a"):
                    self._offset_x += int(self.canvas_w * 0.1)
                    self._auto_fit = False
                if key == ord("d"):
                    self._offset_x -= int(self.canvas_w * 0.1)
                    self._auto_fit = False

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def _receive_loop(self) -> None:
        while self._running:
            try:
                odom = self._odom_sub.recv_json(flags=zmq.NOBLOCK)
                x = odom.get("x", 0.0)
                y = odom.get("y", 0.0)
                with self._lock:
                    self._trail.append((x, y))
                    self._current_pose = odom
                    self._receive_count += 1
                    self._last_recv_time = time.time()
            except zmq.Again:
                pass
            except Exception as e:
                logger.warning(f"里程计接收异常: {e}")
            time.sleep(0.001)

    def _world_to_screen(self, x: float, y: float) -> tuple[int, int]:
        """世界坐标 (米) -> 屏幕坐标 (像素)。
        
        OpenCV 坐标系：X 向右，Y 向下。
        为了让机器人前进方向朝上，我们将世界 Y 轴映射为屏幕 -Y 方向。
        """
        sx = int(self._offset_x + x * self._zoom)
        sy = int(self._offset_y - y * self._zoom)
        return sx, sy

    def _auto_fit_view(self) -> None:
        """根据轨迹自动计算缩放和平移，使轨迹居中显示。"""
        with self._lock:
            trail = list(self._trail)
        if not trail:
            self._offset_x = self.canvas_w // 2
            self._offset_y = self.canvas_h // 2
            self._zoom = 50.0
            return

        xs = [p[0] for p in trail]
        ys = [p[1] for p in trail]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        # 包含当前位姿
        with self._lock:
            pose = self._current_pose
        if pose:
            min_x = min(min_x, pose.get("x", min_x))
            max_x = max(max_x, pose.get("x", max_x))
            min_y = min(min_y, pose.get("y", min_y))
            max_y = max(max_y, pose.get("y", max_y))

        # 添加边距
        range_x = max(max_x - min_x, 0.1) + self._margin_m * 2
        range_y = max(max_y - min_y, 0.1) + self._margin_m * 2

        zoom_x = self.canvas_w / range_x
        zoom_y = self.canvas_h / range_y
        self._zoom = min(zoom_x, zoom_y) * 0.9  # 留 10% 边距

        cx = (min_x + max_x) / 2
        cy = (min_y + max_y) / 2
        self._offset_x = int(self.canvas_w / 2 - cx * self._zoom)
        self._offset_y = int(self.canvas_h / 2 + cy * self._zoom)

    def _draw_canvas(self) -> np.ndarray:
        canvas = np.zeros((self.canvas_h, self.canvas_w, 3), dtype=np.uint8)

        # 自动缩放
        if self._auto_fit:
            self._auto_fit_view()

        with self._lock:
            trail = list(self._trail)
            pose = self._current_pose.copy() if self._current_pose else {}
            recv_count = self._receive_count

        # 绘制网格
        self._draw_grid(canvas)

        # 绘制坐标轴
        origin = self._world_to_screen(0.0, 0.0)
        cv2.line(canvas, (origin[0], 0), (origin[0], self.canvas_h), (40, 40, 40), 1)
        cv2.line(canvas, (0, origin[1]), (self.canvas_w, origin[1]), (40, 40, 40), 1)

        # 绘制轨迹
        if len(trail) >= 2:
            points = np.array([self._world_to_screen(x, y) for x, y in trail], dtype=np.int32)
            # 渐变色轨迹：旧=暗蓝，新=亮青
            for i in range(1, len(points)):
                ratio = i / len(points)
                color = (int(128 + 127 * ratio), int(128 * ratio), int(64))
                thickness = max(1, int(1 + 2 * ratio))
                cv2.line(canvas, tuple(points[i - 1]), tuple(points[i]), color, thickness)

        # 绘制当前位姿（机器人图标）
        if pose:
            x = pose.get("x", 0.0)
            y = pose.get("y", 0.0)
            yaw = pose.get("yaw", 0.0)
            sx, sy = self._world_to_screen(x, y)

            # 机器人本体圆
            robot_radius = max(4, int(0.12 * self._zoom))  # 0.12m 半径
            cv2.circle(canvas, (sx, sy), robot_radius, (0, 255, 0), 2)
            cv2.circle(canvas, (sx, sy), robot_radius - 2, (0, 100, 0), -1)

            # 朝向箭头
            arrow_len = robot_radius * 2
            ax = int(sx + arrow_len * math.cos(-yaw))
            ay = int(sy + arrow_len * math.sin(-yaw))
            cv2.arrowedLine(canvas, (sx, sy), (ax, ay), (0, 0, 255), 2, tipLength=0.3)

        # 信息面板（左上角）
        self._draw_info_panel(canvas, pose, recv_count, len(trail))

        # 右下角图例
        self._draw_legend(canvas)

        return canvas

    def _draw_grid(self, canvas: np.ndarray) -> None:
        """绘制世界坐标网格"""
        # 计算合适的网格间距（0.1m, 0.5m, 1m, 2m, 5m...）
        base_spacing = 1.0  # 米
        while base_spacing * self._zoom < 40:
            base_spacing *= 2
        while base_spacing * self._zoom > 150:
            base_spacing /= 2

        # 找到需要绘制的网格线范围
        # 从屏幕坐标反推世界坐标
        def screen_to_world(sx: int, sy: int) -> tuple[float, float]:
            x = (sx - self._offset_x) / self._zoom
            y = -(sy - self._offset_y) / self._zoom
            return x, y

        wx1, wy1 = screen_to_world(0, 0)
        wx2, wy2 = screen_to_world(self.canvas_w, self.canvas_h)

        x_start = math.floor(min(wx1, wx2) / base_spacing) * base_spacing
        x_end = math.ceil(max(wx1, wx2) / base_spacing) * base_spacing
        y_start = math.floor(min(wy1, wy2) / base_spacing) * base_spacing
        y_end = math.ceil(max(wy1, wy2) / base_spacing) * base_spacing

        color = (30, 30, 30)
        for xi in np.arange(x_start, x_end + base_spacing, base_spacing):
            sx, _ = self._world_to_screen(xi, 0)
            if 0 <= sx <= self.canvas_w:
                cv2.line(canvas, (sx, 0), (sx, self.canvas_h), color, 1)
        for yi in np.arange(y_start, y_end + base_spacing, base_spacing):
            _, sy = self._world_to_screen(0, yi)
            if 0 <= sy <= self.canvas_h:
                cv2.line(canvas, (0, sy), (self.canvas_w, sy), color, 1)

        # 标注网格尺度
        sx, _ = self._world_to_screen(base_spacing, 0)
        s0, _ = self._world_to_screen(0, 0)
        pixel_len = abs(sx - s0)
        put_text_bg(canvas, f"grid={base_spacing}m ({pixel_len:.0f}px)", (10, self.canvas_h - 10),
                   scale=0.4, color=(100, 100, 100), bg_color=(0, 0, 0))

    def _draw_info_panel(self, canvas: np.ndarray, pose: dict, recv_count: int, trail_len: int) -> None:
        x_base = 10
        y_base = 25
        line_h = 22
        color = (0, 255, 0)
        scale = 0.55

        lines = [
            "=== Odom Trajectory ===",
            f"addr: {self.odom_addr}",
            "",
        ]

        if pose:
            lines.extend([
                f"x:    {pose.get('x', 0.0):+.4f} m",
                f"y:    {pose.get('y', 0.0):+.4f} m",
                f"yaw:  {math.degrees(pose.get('yaw', 0.0)):+.2f} deg",
                f"vx:   {pose.get('vx', 0.0):+.3f} m/s",
                f"vy:   {pose.get('vy', 0.0):+.3f} m/s",
                f"vz:   {pose.get('vz', 0.0):+.3f} rad/s",
                "",
                f"zoom: {self._zoom:.1f} px/m",
                f"trail: {trail_len} pts",
                f"recv: {recv_count}",
            ])
        else:
            lines.append("Waiting for odom data...")

        for i, text in enumerate(lines):
            put_text_bg(canvas, text, (x_base, y_base + i * line_h), scale=scale, color=color)

    def _draw_legend(self, canvas: np.ndarray) -> None:
        """绘制右下角图例"""
        x = self.canvas_w - 200
        y = self.canvas_h - 80
        line_h = 20

        items = [
            ((0, 255, 0), "Robot position"),
            ((0, 0, 255), "Heading (yaw)"),
            ((200, 150, 50), "Trail (newer=brighter)"),
        ]
        for i, (color, label) in enumerate(items):
            cy = y + i * line_h
            cv2.line(canvas, (x, cy - 5), (x + 20, cy - 5), color, 2)
            put_text_bg(canvas, label, (x + 28, cy), scale=0.4, color=(180, 180, 180))

        # 控制提示
        hint = "[R]Reset  [S]Save  [+/-]Zoom  [WASD]Pan  [0]Auto"
        put_text_bg(canvas, hint, (x - 100, y + 60), scale=0.4, color=(150, 150, 150))

    def stop(self) -> None:
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        self._odom_sub.close()
        self._ctx.term()
        cv2.destroyAllWindows()
        logger.info("里程计可视化工具已停止")


def main():
    parser = argparse.ArgumentParser(description="HomeBot 里程计轨迹可视化工具")
    parser.add_argument("--odom", default=DEFAULT_ODOM_ADDR, help="OdomService PUB 地址")
    parser.add_argument("--size", type=int, default=900, help="画布大小（像素）")
    parser.add_argument("--max-trail", type=int, default=MAX_TRAIL_POINTS, help="最大轨迹点数")
    args = parser.parse_args()

    print("=" * 60)
    print("HomeBot 里程计轨迹可视化")
    print("=" * 60)
    print(f"订阅地址: {args.odom}")
    print("")
    print("功能:")
    print("  - 实时绘制 XY 平面运动轨迹")
    print("  - 显示机器人朝向（红色箭头）")
    print("  - 自动缩放使轨迹居中")
    print("")
    print("按键:")
    print("  [Q/ESC] 退出     [R] 重置轨迹")
    print("  [S] 保存截图     [+/-] 缩放")
    print("  [WASD] 平移视图  [0] 恢复自动缩放")
    print("=" * 60)

    viz = OdomVisualizer(
        odom_addr=args.odom,
        canvas_size=(args.size, args.size),
        max_trail=args.max_trail,
    )
    viz.start()


if __name__ == "__main__":
    main()
