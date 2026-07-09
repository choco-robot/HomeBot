"""
跨平台键盘热键监听模块

Windows 使用 msvcrt，Unix/Linux/macOS 使用 termios + select。
在非终端环境（如 CI、IDE 运行）下会自动禁用热键。
"""
import sys
import threading
import time
from typing import Callable, Optional

from common.logging import get_logger

logger = get_logger(__name__)


class KeyboardListener:
    """
    键盘监听器

    在后台线程中捕获单个字符按键，并通过回调函数通知应用。
    """

    def __init__(self, callback: Callable[[str], None], poll_interval: float = 0.05):
        """
        初始化键盘监听器

        Args:
            callback: 按键回调，接收单个字符
            poll_interval: 轮询间隔（秒）
        """
        self.callback = callback
        self.poll_interval = poll_interval
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._available = self._check_available()

    def _check_available(self) -> bool:
        """检查当前环境是否支持键盘监听"""
        if sys.platform == "win32":
            return True
        try:
            import termios
            import tty
            return sys.stdin.isatty()
        except Exception:
            return False

    def start(self) -> None:
        """启动监听线程"""
        if not self._available:
            logger.warning("当前终端不支持键盘热键，热键功能已禁用")
            return

        self._running = True
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()
        logger.debug("键盘监听器已启动")

    def stop(self) -> None:
        """停止监听线程"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=0.5)
            self._thread = None
        logger.debug("键盘监听器已停止")

    def _listen(self) -> None:
        """监听循环"""
        while self._running:
            try:
                key = self._get_key()
                if key:
                    try:
                        self.callback(key)
                    except Exception as e:
                        logger.error(f"热键处理异常: {e}")
            except Exception as e:
                logger.warning(f"键盘监听异常: {e}")
                break
            time.sleep(self.poll_interval)

    def _get_key(self) -> Optional[str]:
        """获取单个按键"""
        if sys.platform == "win32":
            return self._get_key_windows()
        else:
            return self._get_key_unix()

    def _get_key_windows(self) -> Optional[str]:
        """Windows 平台使用 msvcrt"""
        import msvcrt
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            try:
                return ch.decode("utf-8", errors="ignore")
            except Exception:
                return None
        return None

    def _get_key_unix(self) -> Optional[str]:
        """Unix 平台使用 termios + select"""
        import termios
        import tty
        import select

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            if select.select([sys.stdin], [], [], self.poll_interval)[0]:
                ch = sys.stdin.read(1)
                if ch:
                    return ch
        except Exception:
            return None
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass
        return None


