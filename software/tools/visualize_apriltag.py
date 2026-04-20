# -*- coding: utf-8 -*-
"""AprilTag Detection Real-time Visualization (High-FPS OpenCV Version)

Image source: subscribes to VisionService ZMQ stream by default.
Background thread receives frames continuously; main loop refreshes at ~30 FPS.

Usage:
    # Default: subscribe to VisionService
    python tools/visualize_apriltag.py

    # Mock mode (no real tag needed)
    python tools/visualize_apriltag.py --mock

    # Direct camera bypass
    python tools/visualize_apriltag.py --device 0
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import zmq

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

DEFAULT_VISION_ADDR = "tcp://localhost:5560"


class VisionSubscriber:
    """Background thread that subscribes to VisionService and keeps the latest frame."""

    def __init__(self, address: str = DEFAULT_VISION_ADDR):
        self.address = address
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.connect(address)
        self._sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._sock.setsockopt(zmq.RCVTIMEO, 100)
        self.lock = threading.Lock()
        self.latest_frame: np.ndarray | None = None
        self.frame_count = 0
        self._running = False
        self._thread: threading.Thread | None = None
        print(f"[ZMQ] Subscribed to VisionService: {address}")

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self):
        while self._running:
            try:
                parts = self._sock.recv_multipart()
                if len(parts) >= 2:
                    buf = np.frombuffer(parts[1], dtype=np.uint8)
                    frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self.lock:
                            self.latest_frame = frame
                            self.frame_count += 1
            except zmq.Again:
                continue
            except Exception as e:
                print(f"[VisionSubscriber] Error: {e}")
                time.sleep(0.01)

    def read(self) -> tuple:
        with self.lock:
            if self.latest_frame is not None:
                return True, self.latest_frame.copy()
        return False, None

    def release(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self._sock.close()
        self._ctx.term()


def draw_tag_axis(frame, K, rvec, tvec, axis_length=0.1, thickness=2, dist_coeffs=None):
    axis_3d = np.array([
        [0, 0, 0],
        [axis_length, 0, 0],
        [0, axis_length, 0],
        [0, 0, axis_length],
    ], dtype=np.float64)
    dist = dist_coeffs if dist_coeffs is not None else np.zeros((4, 1), dtype=np.float64)
    imgpts, _ = cv2.projectPoints(axis_3d, rvec, tvec, K.astype(np.float64), dist)
    imgpts = imgpts.astype(int)
    origin = tuple(imgpts[0][0])
    cv2.line(frame, origin, tuple(imgpts[1][0]), (0, 0, 255), thickness)
    cv2.line(frame, origin, tuple(imgpts[2][0]), (0, 255, 0), thickness)
    cv2.line(frame, origin, tuple(imgpts[3][0]), (255, 0, 0), thickness)
    cv2.putText(frame, 'X', tuple(imgpts[1][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(frame, 'Y', tuple(imgpts[2][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(frame, 'Z', tuple(imgpts[3][0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    return frame


def draw_tag_info(frame, detection, pos=(10, 30), color=(0, 255, 255)):
    lines = [
        f"ID: {detection['tag_id']}",
        f"Conf: {detection['confidence']:.2f}",
        f"X: {detection['x']:.3f}m",
        f"Y: {detection['y']:.3f}m",
        f"Theta: {math.degrees(detection['theta']):.1f}deg",
        f"Err: {detection.get('pose_err', 0):.2f}px",
    ]
    y = pos[1]
    for line in lines:
        cv2.putText(frame, line, (pos[0], y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        y += 20
    return frame


def main():
    parser = argparse.ArgumentParser(description="AprilTag Detection Visualization")
    parser.add_argument("--vision", default=DEFAULT_VISION_ADDR, help="VisionService address")
    parser.add_argument("--device", type=int, default=None, help="Direct camera device ID")
    parser.add_argument("--image", default=None, help="Image file for offline test")
    parser.add_argument("--mock", action="store_true", help="Mock detection mode")
    parser.add_argument("--tag-size", type=float, default=0.05, help="Tag edge length (m)")
    parser.add_argument("--fx", type=float, default=600.0, help="Camera fx")
    parser.add_argument("--fy", type=float, default=600.0, help="Camera fy")
    parser.add_argument("--cx", type=float, default=320.0, help="Camera cx")
    parser.add_argument("--cy", type=float, default=240.0, help="Camera cy")
    args = parser.parse_args()

    K = np.array([[args.fx, 0, args.cx], [0, args.fy, args.cy], [0, 0, 1]], dtype=np.float64)

    # Initialize detector
    if args.mock:
        from navigation.perception.apriltag_detector import MockAprilTagDetector
        tag_map = {0: (1.0, 0.0, 0.0), 1: (2.0, 1.0, 1.57)}
        detector = MockAprilTagDetector(tag_map=tag_map, fov_deg=75.0, max_range_m=3.0)
        print("[Mock Mode] Simulated AprilTag detection")
    else:
        from navigation.perception.apriltag_detector import create_apriltag_detector
        detector = create_apriltag_detector(camera_matrix=K, tag_size_m=args.tag_size)
        print("[Real Mode] pupil-apriltags detection (tag36h11)")

    # Image source
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"Error: cannot read image {args.image}")
            return
        source = "image"
    elif args.device is not None:
        cap = cv2.VideoCapture(args.device)
        if not cap.isOpened():
            print(f"Error: cannot open camera {args.device}")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        source = "camera"
        print(f"[Camera Mode] Device {args.device} opened")
    else:
        cap = VisionSubscriber(args.vision)
        cap.start()
        source = "zmq"
        for _ in range(30):
            ok, _ = cap.read()
            if ok:
                break
            time.sleep(0.1)
        if not ok:
            print("Error: no frame from VisionService. Is it running?")
            cap.release()
            return

    fps_history = []
    print("AprilTag visualization started. Press Q or ESC to exit.")

    while True:
        t0 = time.perf_counter()

        if source == "image":
            pass
        elif source == "camera":
            ret, frame = cap.read()
            if not ret:
                break
        else:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

        if args.mock:
            detector.set_robot_pose(0.5, 0.0, 0.0)

        detections = detector.detect(frame)

        for det in detections:
            if det.get("corners") is not None:
                corners = np.array(det["corners"], dtype=int).reshape(-1, 2)
                cv2.polylines(frame, [corners], True, (0, 255, 255), 2)
            else:
                cx, cy = int(K[0, 2]), int(K[1, 2])
                pts = np.array([[cx-80, cy-80], [cx+80, cy-80], [cx+80, cy+80], [cx-80, cy+80]], dtype=int)
                cv2.polylines(frame, [pts], True, (0, 255, 255), 2)

            if "rvec" in det and "tvec" in det:
                frame = draw_tag_axis(frame, K, det["rvec"], det["tvec"], axis_length=0.1)

            info_pos = (10, 30 + det["tag_id"] * 120)
            frame = draw_tag_info(frame, det, pos=info_pos)

        fps_history.append(1.0 / max(time.perf_counter() - t0, 0.001))
        if len(fps_history) > 30:
            fps_history.pop(0)
        avg_fps = sum(fps_history) / len(fps_history)

        mode_str = "MOCK" if args.mock else "REAL"
        src_str = {"image": "IMG", "camera": "CAM", "zmq": "ZMQ"}[source]
        h, w = frame.shape[:2]
        info = f"Src:{src_str} Mode:{mode_str} Tags:{len(detections)} FPS:{avg_fps:.1f}"
        cv2.putText(frame, info, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if not args.mock and len(detections) == 0:
            cv2.putText(frame, "No tag. Check tag36h11 / tag_size / lighting",
                        (10, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow("AprilTag Detection", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        if source == "image":
            break

    if source == "camera":
        cap.release()
    elif source == "zmq":
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
