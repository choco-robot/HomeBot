# -*- coding: utf-8 -*-
"""SLAM Real-time Monitor (High-FPS OpenCV Version)

Subscribes to:
    - VisionService image stream (tcp://localhost:5560)
    - SLAMService pose         (tcp://localhost:5563)
    - SLAMService map          (tcp://localhost:5564)

Display layout (side-by-side):
    Left : Real-time camera feed
    Right: SLAM occupancy grid + robot trajectory overlay

Uses background threads for ZMQ reception; main loop runs at ~30 FPS.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np
import zmq

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from common.logging import get_logger

logger = get_logger(__name__)

DEFAULT_VISION_ADDR = "tcp://localhost:5560"
DEFAULT_POSE_ADDR = "tcp://localhost:5563"
DEFAULT_MAP_ADDR = "tcp://localhost:5564"
HISTORY_LEN = 2000


def colorize_map(map_gray: np.ndarray) -> np.ndarray:
    """Colorize occupancy grid: unknown=gray, free=white, occupied=red."""
    h, w = map_gray.shape
    colored = np.zeros((h, w, 3), dtype=np.uint8)
    mask_unknown = (map_gray > 100) & (map_gray < 150)
    colored[mask_unknown] = [128, 128, 128]
    mask_free = map_gray <= 100
    colored[mask_free] = [255, 255, 255]
    mask_occ = map_gray >= 150
    colored[mask_occ] = [0, 0, 200]
    return colored


class DataReceiver:
    """Background threads that continuously receive ZMQ streams."""

    def __init__(self, vision_addr: str, pose_addr: str, map_addr: str):
        self.vision_addr = vision_addr
        self.pose_addr = pose_addr
        self.map_addr = map_addr

        self._ctx = zmq.Context()

        self._vision_sock = self._ctx.socket(zmq.SUB)
        self._vision_sock.connect(vision_addr)
        self._vision_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._vision_sock.setsockopt(zmq.RCVTIMEO, 100)

        self._pose_sock = self._ctx.socket(zmq.SUB)
        self._pose_sock.connect(pose_addr)
        self._pose_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._pose_sock.setsockopt(zmq.RCVTIMEO, 100)

        self._map_sock = self._ctx.socket(zmq.SUB)
        self._map_sock.connect(map_addr)
        self._map_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._map_sock.setsockopt(zmq.RCVTIMEO, 100)

        self.lock = threading.Lock()
        self.latest_frame: np.ndarray | None = None
        self.latest_pose: dict | None = None
        self.latest_map: np.ndarray | None = None
        self.map_meta: dict | None = None

        # Counters for debugging
        self.vision_count = 0
        self.pose_count = 0
        self.map_count = 0

        self._running = False
        self._threads: list[threading.Thread] = []

    def start(self):
        self._running = True
        self._threads = [
            threading.Thread(target=self._vision_loop, daemon=True),
            threading.Thread(target=self._pose_loop, daemon=True),
            threading.Thread(target=self._map_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=1.0)
        for s in (self._vision_sock, self._pose_sock, self._map_sock):
            s.close()
        self._ctx.term()

    def _vision_loop(self):
        while self._running:
            try:
                parts = self._vision_sock.recv_multipart()
                if len(parts) >= 2:
                    buf = np.frombuffer(parts[1], dtype=np.uint8)
                    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self.lock:
                            self.latest_frame = frame
                            self.vision_count += 1
            except zmq.Again:
                continue
            except Exception:
                time.sleep(0.01)

    def _pose_loop(self):
        while self._running:
            try:
                msg = self._pose_sock.recv_json(flags=zmq.NOBLOCK)
                if msg:
                    with self.lock:
                        self.latest_pose = msg
                        self.pose_count += 1
            except zmq.Again:
                time.sleep(0.005)
            except Exception:
                time.sleep(0.01)

    def _map_loop(self):
        while self._running:
            try:
                parts = self._map_sock.recv_multipart()
                if len(parts) >= 2:
                    meta = json.loads(parts[0])
                    mapbytes = np.frombuffer(parts[1], dtype=np.uint8)
                    size = meta.get("size_pixels", 800)
                    if len(mapbytes) == size * size:
                        map_img = mapbytes.reshape(size, size)
                        with self.lock:
                            self.latest_map = map_img
                            self.map_meta = meta
                            self.map_count += 1
            except zmq.Again:
                time.sleep(0.05)
            except Exception:
                time.sleep(0.05)

    def print_stats(self):
        with self.lock:
            print(f"[Stats] Vision={self.vision_count}  Pose={self.pose_count}  Map={self.map_count}")


def main():
    parser = argparse.ArgumentParser(description="SLAM Real-time Monitor")
    parser.add_argument("--vision", default=DEFAULT_VISION_ADDR, help="VisionService address")
    parser.add_argument("--pose", default=DEFAULT_POSE_ADDR, help="Pose PUB address")
    parser.add_argument("--map", default=DEFAULT_MAP_ADDR, help="Map PUB address")
    parser.add_argument("--map-size", type=float, default=20.0, help="Map physical size (m)")
    args = parser.parse_args()

    receiver = DataReceiver(args.vision, args.pose, args.map)
    receiver.start()

    history = deque(maxlen=HISTORY_LEN)
    fps_history = deque(maxlen=30)
    last_stat_time = time.time()

    print("SLAM Monitor started. Press Q or ESC to exit.")
    running = True

    try:
        while running:
            t0 = time.perf_counter()

            with receiver.lock:
                frame = receiver.latest_frame.copy() if receiver.latest_frame is not None else None
                pose = receiver.latest_pose
                map_gray = receiver.latest_map.copy() if receiver.latest_map is not None else None
                meta = receiver.map_meta

            # ---------- Camera ----------
            if frame is not None:
                cam_display = frame.copy()
            else:
                cam_display = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(cam_display, "Waiting for VisionService...", (120, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # ---------- Map ----------
            if map_gray is not None:
                map_display = colorize_map(map_gray)
                scale = 600 / map_display.shape[0] if map_display.shape[0] > 600 else 1.0
                if scale < 1.0:
                    map_display = cv2.resize(map_display, None, fx=scale, fy=scale)
            else:
                map_display = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(map_display, "Waiting for SLAM map...", (140, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Draw trajectory on map
            if map_gray is not None and len(history) > 1:
                h, w = map_gray.shape
                s = map_display.shape[0] / h
                for i in range(1, len(history)):
                    x1, y1, _ = history[i - 1]
                    x2, y2, _ = history[i]
                    px1 = int((x1 / args.map_size + 0.5) * w * s)
                    py1 = int((1.0 - (y1 / args.map_size + 0.5)) * h * s)
                    px2 = int((x2 / args.map_size + 0.5) * w * s)
                    py2 = int((1.0 - (y2 / args.map_size + 0.5)) * h * s)
                    cv2.line(map_display, (px1, py1), (px2, py2), (0, 255, 255), 1)

                cx, cy, ctheta = history[-1]
                pcx = int((cx / args.map_size + 0.5) * w * s)
                pcy = int((1.0 - (cy / args.map_size + 0.5)) * h * s)
                cv2.circle(map_display, (pcx, pcy), 4, (0, 255, 0), -1)
                dx = int(15 * math.cos(ctheta))
                dy = int(-15 * math.sin(ctheta))
                cv2.arrowedLine(map_display, (pcx, pcy), (pcx + dx, pcy + dy), (0, 255, 0), 2)

            # Update history
            if pose:
                history.append((pose["x"], pose["y"], pose["theta"]))

            # ---------- Info text on camera ----------
            h_cam, w_cam = cam_display.shape[:2]
            info_lines = []
            if pose:
                state = pose.get("state", "UNKNOWN")
                info_lines.append(f"State: {state}")
                info_lines.append(f"X: {pose['x']:+.3f}  Y: {pose['y']:+.3f}")
                info_lines.append(f"Theta: {math.degrees(pose['theta']):.1f}deg")
                cov = pose.get("covariance", [[0]])
                info_lines.append(f"CovXY: {cov[0][0]:.4f}  Fails: {pose.get('slam_fail_count', 0)}")
            else:
                info_lines.append("Waiting for pose data...")

            y_off = h_cam - 10
            for line in reversed(info_lines):
                cv2.putText(cam_display, line, (10, y_off), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 1)
                y_off -= 20

            # FPS
            fps_history.append(1.0 / max(time.perf_counter() - t0, 0.001))
            fps = sum(fps_history) / len(fps_history)
            cv2.putText(cam_display, f"FPS: {fps:.1f}", (w_cam - 120, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # ---------- Display ----------
            target_h = 480
            cam_h, cam_w = cam_display.shape[:2]
            map_h, map_w = map_display.shape[:2]

            cam_resized = cv2.resize(cam_display, (int(target_h * cam_w / cam_h), target_h))
            map_resized = cv2.resize(map_display, (int(target_h * map_w / map_h), target_h))

            # Ensure same height
            if cam_resized.shape[0] != target_h:
                cam_resized = cv2.resize(cam_resized, (cam_resized.shape[1], target_h))
            if map_resized.shape[0] != target_h:
                map_resized = cv2.resize(map_resized, (map_resized.shape[1], target_h))

            combined = np.hstack([cam_resized, map_resized])
            cv2.imshow("SLAM Monitor (Camera | Map)", combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                running = False

    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
