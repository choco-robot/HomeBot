# -*- coding: utf-8 -*-
"""LD06 / LD19 激光雷达驱动

基于 LDRobot LD06 协议实现的纯 Python 串口驱动。
无可用雷达连接时直接抛出异常，不模拟。

协议格式（每包 47 字节，小端）：
    Header      1B  0x54
    VerLen      1B  0x2C
    Speed       2B  deg/s
    StartAngle  2B  0.01°
    Data[12]    36B 每点: Distance(mm, 2B) + Confidence(1B)
    StopAngle   2B  0.01°
    Timestamp   2B  ms
    CRC         1B  poly=0x4D, init=0x00
"""
from __future__ import annotations

import math
import struct
import threading
import time
from typing import List, Optional, Tuple

import numpy as np

from common.logging import get_logger

logger = get_logger(__name__)

# ------------------------------------------------------------------------------
# 协议常量
# ------------------------------------------------------------------------------
PACKET_LENGTH = 47
MEASUREMENT_LENGTH = 12
HEADER_BYTE = 0x54
VER_LEN_BYTE = 0x2C

# struct format: skip header, then length(1), speed(2), start_angle(2),
# 12 x (distance(2) + confidence(1)), stop_angle(2), timestamp(2), crc(1)
MESSAGE_FORMAT = f"<xBHH{'HB' * MEASUREMENT_LENGTH}HHB"

# CRC8 参数 (poly=0x4D, init=0x00, final_xor=0x00, reflect_in=False, reflect_out=False)
CRC8_TABLE = None


def _init_crc8_table() -> List[int]:
    """初始化 CRC8 查找表"""
    table = []
    poly = 0x4D
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
            crc &= 0xFF
        table.append(crc)
    return table


def crc8_ld06(data: bytes) -> int:
    """计算 LD06 协议 CRC8"""
    global CRC8_TABLE
    if CRC8_TABLE is None:
        CRC8_TABLE = _init_crc8_table()
    crc = 0x00
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc


# ------------------------------------------------------------------------------
# LD06 真实驱动
# ------------------------------------------------------------------------------

class LD06Driver:
    """LD06 激光雷达串口驱动。

    输出格式与 BreezySLAM 兼容：固定分辨率的距离数组（毫米）。
    """

    def __init__(
        self,
        port: str = "COM3",
        baudrate: int = 230400,
        scan_size: int = 360,
        max_distance_m: float = 12.0,
        min_distance_m: float = 0.2,
        timeout: float = 1.0,
    ):
        self.port = port
        self.baudrate = baudrate
        self.scan_size = scan_size
        self.max_distance_mm = int(max_distance_m * 1000)
        self.min_distance_mm = int(min_distance_m * 1000)
        self.timeout = timeout

        self._serial: Optional[object] = None
        self._running = False
        self._read_thread: Optional[threading.Thread] = None

        # 最新一圈扫描数据（插值后）
        self._latest_scan: Optional[Tuple[List[float], List[float]]] = None
        self._lock = threading.Lock()

        # 统计
        self._packet_count = 0
        self._frame_count = 0
        self._crc_error_count = 0
        self._last_log_time = time.time()

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def start(self) -> None:
        import serial

        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,  # 非阻塞读取，50ms 超时
        )
        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        logger.info(f"LD06Driver 已启动: {self.port}@{self.baudrate}")

    def stop(self) -> None:
        self._running = False
        if self._read_thread:
            self._read_thread.join(timeout=2.0)
        if self._serial:
            self._serial.close()
            self._serial = None
        logger.info("LD06Driver 已停止")

    # ------------------------------------------------------------------
    # 数据获取
    # ------------------------------------------------------------------
    def get_scan(self) -> Optional[Tuple[List[float], List[float]]]:
        """获取最新一圈扫描数据。

        Returns:
            (angles_degrees, distances_mm) 或 None
        """
        with self._lock:
            return self._latest_scan

    # ------------------------------------------------------------------
    # 内部读取循环
    # ------------------------------------------------------------------
    def _read_loop(self) -> None:
        """后台线程：持续读取串口并解析完整 360° 扫描。"""
        buf = bytearray()
        measurements: List[Tuple[float, int, int]] = []  # (angle, dist_mm, conf)
        last_angle = -1.0

        while self._running:
            try:
                chunk = self._serial.read(max(1, self._serial.in_waiting))
            except Exception as e:
                logger.warning(f"串口读取异常: {e}")
                time.sleep(0.1)
                continue

            if not chunk:
                continue

            buf.extend(chunk)

            # 解析缓冲区中的所有完整包
            while len(buf) >= PACKET_LENGTH:
                # 查找包头
                if buf[0] != HEADER_BYTE:
                    buf.pop(0)
                    continue
                if len(buf) < 2 or buf[1] != VER_LEN_BYTE:
                    buf.pop(0)
                    continue
                if len(buf) < PACKET_LENGTH:
                    break

                packet = bytes(buf[:PACKET_LENGTH])
                buf = buf[PACKET_LENGTH:]

                # CRC 校验
                payload = packet[:-1]
                expected_crc = packet[-1]
                if crc8_ld06(payload) != expected_crc:
                    self._crc_error_count += 1
                    continue

                self._packet_count += 1
                parsed = self._parse_packet(packet)
                if parsed is None:
                    continue

                start_angle, stop_angle, points = parsed

                # 检测跳变（完成一圈）
                if last_angle > 300.0 and start_angle < 60.0 and len(measurements) > 100:
                    self._finalize_scan(measurements)
                    measurements = []

                last_angle = stop_angle

                # 累积测量点
                for angle, dist_mm, conf in points:
                    measurements.append((angle, dist_mm, conf))

            self._log_stats()

    def _parse_packet(
        self, packet: bytes
    ) -> Optional[Tuple[float, float, List[Tuple[float, int, int]]]]:
        """解析单包数据。"""
        try:
            unpacked = struct.unpack(MESSAGE_FORMAT, packet)
        except struct.error:
            return None

        # unpacked: length, speed, start_angle, *pos_data, stop_angle, timestamp, crc
        length = unpacked[0]
        speed = unpacked[1]
        start_angle = unpacked[2] / 100.0
        stop_angle = unpacked[-3] / 100.0
        # pos_data 是 24 个值: distance0, conf0, distance1, conf1, ...
        pos_data = unpacked[3:-3]

        # 角度 unwrap
        if stop_angle < start_angle:
            stop_angle += 360.0

        step = (stop_angle - start_angle) / (MEASUREMENT_LENGTH - 1) if MEASUREMENT_LENGTH > 1 else 0

        points = []
        for i in range(MEASUREMENT_LENGTH):
            dist_mm = pos_data[i * 2]
            conf = pos_data[i * 2 + 1]
            angle = start_angle + step * i
            # 规范化到 [0, 360)
            angle = angle % 360.0
            points.append((angle, dist_mm, conf))

        return start_angle, stop_angle, points

    def _finalize_scan(self, measurements: List[Tuple[float, int, int]]) -> None:
        """将累积的原始点插值为固定分辨率的扫描数据。"""
        if not measurements:
            return

        angles = np.array([m[0] for m in measurements])
        distances = np.array([m[1] for m in measurements])
        confidences = np.array([m[2] for m in measurements])

        # 过滤无效距离、过近距离（机器人本体结构）和置信度过低的点
        valid_mask = (
            (distances >= self.min_distance_mm)
            & (distances <= self.max_distance_mm)
            & (confidences > 0)
        )
        if not np.any(valid_mask):
            return

        angles = angles[valid_mask]
        distances = distances[valid_mask]

        # 按角度排序
        sort_idx = np.argsort(angles)
        angles = angles[sort_idx]
        distances = distances[sort_idx]

        # 插值到固定分辨率
        target_angles = np.linspace(0.0, 360.0, self.scan_size, endpoint=False)
        interp_dists = np.interp(
            target_angles,
            angles,
            distances,
            period=360.0,
        )

        # 将 0（interp 外推可能产生）替换为 max_distance
        interp_dists = np.where(interp_dists <= 0, self.max_distance_mm, interp_dists)

        with self._lock:
            self._latest_scan = (target_angles.tolist(), interp_dists.tolist())

        self._frame_count += 1

    def _log_stats(self) -> None:
        now = time.time()
        if now - self._last_log_time >= 5.0:
            fps = self._frame_count / (now - self._last_log_time)
            logger.info(
                f"LD06Driver 性能: {fps:.1f} FPS, "
                f"包数={self._packet_count}, CRC错误={self._crc_error_count}"
            )
            self._frame_count = 0
            self._packet_count = 0
            self._crc_error_count = 0
            self._last_log_time = now


# ------------------------------------------------------------------------------
# 工厂函数
# ------------------------------------------------------------------------------

def create_lidar_driver(
    port: Optional[str] = None,
    scan_size: int = 360,
    min_distance_m: float = 0.2,
) -> LD06Driver:
    """创建激光雷达驱动实例。

    Args:
        port: 串口号，None 时使用默认（Windows=COM3, Linux=/dev/ttyUSB0）
        scan_size: 扫描分辨率（点数）
        min_distance_m: 最小有效距离（米），小于此距离的点会被过滤

    Returns:
        LD06Driver 实例
    """
    if port is None:
        import sys
        if sys.platform.startswith("win"):
            port = "COM3"
        else:
            port = "/dev/ttyUSB0"

    return LD06Driver(port=port, scan_size=scan_size, min_distance_m=min_distance_m)
