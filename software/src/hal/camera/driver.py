"""Camera driver implementation using OpenCV."""

from common.logging import get_logger

logger = get_logger(__name__)

def get_optimal_backend():
    """Get the optimal OpenCV backend for camera capture."""
    import platform,cv2
    system=platform.system()
    if system == "Linux":
        return cv2.CAP_V4L2  # Use V4L2 backend for Linux
    elif system == "Windows":
        # 不用 CAP_MSMF: 实测在非主线程中打开会死锁（摄像头驱动被卡死需重新插拔）
        return cv2.CAP_DSHOW  # Use DirectShow backend for Windows
    elif system == "Darwin":
        return cv2.CAP_AVFOUNDATION  # Use AVFoundation backend for macOS
    else:
        logger.warning(f"Unsupported platform {system}, using default backend")
        return cv2.CAP_ANY  # Use default backend for other platforms

class CameraDriver:
    def __init__(self, device: int = 0):
        """Open the camera device index (default 0)."""
        import cv2
        self._device = device
        self._cap = cv2.VideoCapture(device, get_optimal_backend())  # Use optimal backend
        if not self._cap.isOpened():
            logger.error(f"failed to open camera device {device}")
            raise RuntimeError(f"Camera {device} open failed")
        # 最小化驱动缓冲以降低采集延迟 (部分驱动不支持, 失败静默忽略)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        logger.info(f"camera {device} opened with backend {self._cap.getBackendName()}")

    def capture_frame(self):
        """Capture a single frame and return as numpy array (BGR)."""
        import cv2
        if self._cap is None:
            raise RuntimeError("camera not initialized")
        ret, frame = self._cap.read()
        if not ret:
            logger.warning("failed to read frame")
            return None
        logger.debug("captured frame")
        return frame

    def release(self):
        """Release the camera resource."""
        if self._cap:
            self._cap.release()
            self._cap = None
            logger.info("camera released")
