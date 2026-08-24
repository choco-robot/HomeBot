"""简单日志封装，可替换为更复杂的日志系统（如 structlog）。

日志级别通过环境变量 HOMEBOT_LOG_LEVEL 控制，默认 INFO。

注意：不要在此模块中导入 configs —— configs.config 依赖 configs.secrets，
而 configs.secrets 又依赖本模块，直接导入会形成循环，导致级别回退到
DEBUG 并在 import 时产生日志噪音。
"""
import logging
import os


def get_logger(name: str) -> logging.Logger:
    level = os.environ.get("HOMEBOT_LOG_LEVEL", "INFO")
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    return logger
