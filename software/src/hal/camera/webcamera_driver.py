"""WebCamera driver - 网络摄像头驱动，支持从 URL/RTSP/HTTP 获取视频流."""

import time
from common.logging import get_logger

logger = get_logger(__name__)


class WebCameraDriver:
    """网络摄像头驱动，支持多种视频流协议.
    
    支持的 URL 格式:
        - HTTP MJPEG: http://ip:port/path (推荐用于网络摄像头)
        - RTSP: rtsp://username:password@ip:port/path
        - 本地文件: file:///path/to/video.mp4
    
    对于 HTTP MJPEG 流，使用 requests 直接解析 multipart/x-mixed-replace，
    比 OpenCV VideoCapture 更稳定。
    """
    
    # 常见 MJPEG 端点路径（自动探测用）
    _COMMON_MJPEG_PATHS = [
        "",
        "/video",
        "/mjpeg",
        "/stream",
        "/?action=stream",
        "/cgi-bin/mjpg/video.cgi",
        "/videostream.cgi",
        "/api/video",
        "/live",
        "/camera",
    ]
    
    def __init__(self, url: str, width: int = 0, height: int = 0, fps: int = 0):
        """初始化网络摄像头驱动.
        
        Args:
            url: 视频流地址 (RTSP/HTTP/文件路径等)
            width: 期望宽度 (0=不设置)
            height: 期望高度 (0=不设置)
            fps: 期望帧率 (0=不设置)
        """
        self._url = url
        self._width = width
        self._height = height
        self._fps = fps
        self._cap = None          # OpenCV VideoCapture (RTSP/文件用)
        self._mjpeg_session = None  # requests Session (MJPEG HTTP 用)
        self._mjpeg_stream = None   # requests Response
        self._mjpeg_boundary = None # multipart boundary bytes
        self._use_mjpeg_http = False
        
        # 解析 URL 判断协议
        url_lower = url.lower().strip()
        
        if url_lower.startswith("http://") or url_lower.startswith("https://"):
            self._init_mjpeg_http(url)
        else:
            self._init_opencv(url)
    
    def _init_mjpeg_http(self, url: str):
        """初始化 MJPEG HTTP 流."""
        import requests
        
        base_url = url.rstrip("/")
        urls_to_try = [base_url]
        
        # 如果 URL 没有路径，追加常见端点
        path_part = base_url.split("/", 3)[-1] if "/" in base_url.replace("//", "") else ""
        if not path_part:
            urls_to_try = [base_url + p for p in self._COMMON_MJPEG_PATHS]
        
        last_err = None
        for attempt_url in urls_to_try:
            for retry in range(3):
                try:
                    logger.info(f"connecting MJPEG stream: {attempt_url} (attempt {retry+1})")
                    session = requests.Session()
                    # 发送 Connection: close 帮助服务端感知断开
                    resp = session.get(
                        attempt_url,
                        stream=True,
                        timeout=(5, 10),  # (connect_timeout, read_timeout)
                        headers={"Connection": "close"}
                    )
                    
                    if resp.status_code != 200:
                        resp.close()
                        session.close()
                        logger.debug(f"{attempt_url} returned {resp.status_code}")
                        break  # 换下一个 URL
                    
                    ct = resp.headers.get("Content-Type", "").lower()
                    if "multipart" not in ct:
                        resp.close()
                        session.close()
                        logger.debug(f"{attempt_url} not multipart: {ct}")
                        break  # 换下一个 URL
                    
                    # 成功建立 MJPEG 连接
                    self._use_mjpeg_http = True
                    self._mjpeg_session = session
                    self._mjpeg_stream = resp
                    self._mjpeg_boundary = self._parse_boundary(ct)
                    logger.info(f"MJPEG HTTP stream connected: {attempt_url}, boundary={self._mjpeg_boundary}")
                    return
                    
                except Exception as e:
                    last_err = e
                    logger.warning(f"connect {attempt_url} attempt {retry+1} failed: {e}")
                    time.sleep(1.5)
        
        raise RuntimeError(
            f"WebCamera open failed: {url}\n"
            f"last error: {last_err}\n"
            f"提示：ESP32 只支持一个客户端，请确保没有其他程序占用该 IP 的 80 端口"
        )
    
    def _parse_boundary(self, content_type: str) -> bytes:
        """从 Content-Type 头解析 multipart boundary."""
        import re
        match = re.search(r'boundary=([\w\-]+)', content_type, re.IGNORECASE)
        if match:
            return ("--" + match.group(1)).encode()
        return b"--frame"
    
    def _init_opencv(self, url: str):
        """使用 OpenCV VideoCapture 初始化."""
        import cv2
        self._cap = cv2.VideoCapture(url)
        
        if not self._cap.isOpened():
            logger.error(f"failed to open webcamera: {url}")
            raise RuntimeError(f"WebCamera open failed: {url}")
        
        if self._width > 0:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        if self._height > 0:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        if self._fps > 0:
            self._cap.set(cv2.CAP_PROP_FPS, self._fps)
        
        actual_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
        
        logger.info(f"webcamera opened (OpenCV): url={url}, resolution={actual_width}x{actual_height}, fps={actual_fps:.1f}")
    
    def capture_frame(self):
        """捕获单帧图像，返回 numpy array (BGR)."""
        import cv2
        import numpy as np
        
        if self._use_mjpeg_http:
            jpeg_data = self._read_mjpeg_frame()
            if jpeg_data is None:
                logger.warning("failed to read MJPEG frame, trying reconnect...")
                self._reconnect_mjpeg()
                jpeg_data = self._read_mjpeg_frame()
            if jpeg_data is None:
                return None
            
            frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                logger.debug("captured frame from MJPEG HTTP")
            return frame
        else:
            # OpenCV 模式
            if self._cap is None:
                raise RuntimeError("webcamera not initialized")
            ret, frame = self._cap.read()
            if not ret:
                logger.warning("failed to read frame from webcamera, trying reconnect...")
                self._cap.release()
                self._cap = cv2.VideoCapture(self._url)
                if self._cap.isOpened():
                    ret, frame = self._cap.read()
                if not ret:
                    logger.warning("reconnect failed")
                    return None
            logger.debug("captured frame from webcamera (OpenCV)")
            return frame
    
    def _read_mjpeg_frame(self):
        """从 MJPEG HTTP 流读取单帧 JPEG 数据.
        
        multipart/x-mixed-replace 格式：
            --boundary\r\n
            Content-Type: image/jpeg\r\n
            \r\n
            <JPEG data>
            --boundary\r\n
        """
        if self._mjpeg_stream is None:
            return None
        
        try:
            # 使用底层 raw 流读取，避免重复创建 iter_content 生成器
            raw = self._mjpeg_stream.raw
            boundary = self._mjpeg_boundary
            buffer = b""
            
            # 第一步：读取到第一个 boundary（跳过 preamble）
            while True:
                idx = buffer.find(boundary)
                if idx != -1:
                    buffer = buffer[idx + len(boundary):]
                    break
                chunk = raw.read(4096)
                if not chunk:
                    return None
                buffer += chunk
            
            # 第二步：读取到下一个 boundary
            while True:
                idx = buffer.find(boundary)
                if idx != -1:
                    frame_data = buffer[:idx]
                    buffer = buffer[idx + len(boundary):]
                    break
                chunk = raw.read(4096)
                if not chunk:
                    return None
                buffer += chunk
            
            # 去掉前后可能的 \r\n 或 \n
            frame_data = frame_data.strip(b"\r\n").strip(b"\n")
            
            # 如果有 HTTP headers，去掉它们（找到第一个空行）
            header_end = frame_data.find(b"\r\n\r\n")
            if header_end != -1:
                frame_data = frame_data[header_end + 4:]
            else:
                header_end = frame_data.find(b"\n\n")
                if header_end != -1:
                    frame_data = frame_data[header_end + 2:]
            
            return frame_data if frame_data else None
        except Exception as e:
            logger.warning(f"MJPEG stream read error: {e}")
            return None
    
    def _reconnect_mjpeg(self):
        """MJPEG HTTP 流重连."""
        try:
            self.release()
            time.sleep(1.0)
            self._init_mjpeg_http(self._url)
            logger.info("MJPEG HTTP stream reconnected")
        except Exception as e:
            logger.warning(f"MJPEG reconnect failed: {e}")
    
    def release(self):
        """释放摄像头资源."""
        if self._cap:
            self._cap.release()
            self._cap = None
        if self._mjpeg_stream:
            try:
                self._mjpeg_stream.close()
            except Exception:
                pass
            self._mjpeg_stream = None
        if self._mjpeg_session:
            try:
                self._mjpeg_session.close()
            except Exception:
                pass
            self._mjpeg_session = None
        logger.info("webcamera released")
