# -*- coding: utf-8 -*-
"""单目深度估计器 - 基于 MiDaS-small ONNX"""
from __future__ import annotations

import math
import os
from typing import Optional, Tuple

import cv2
import numpy as np

from common.logging import get_logger

logger = get_logger(__name__)

DEFAULT_MODEL_PATH = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "models", "midas", "midas_small.onnx"
)
DEFAULT_MODEL_PATH = os.path.abspath(DEFAULT_MODEL_PATH)


def _check_onnxruntime() -> bool:
    try:
        import onnxruntime as ort
        return True
    except ImportError:
        return False


class DepthEstimator:
    """MiDaS-small ONNX 深度估计器。

    输入：OpenCV BGR 图像 (H, W, 3)
    输出：相对深度图 (H, W)，值越大表示越远
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        inference_size: Tuple[int, int] = (256, 256),
        providers: Optional[list] = None,
    ):
        """
        Args:
            model_path: ONNX 模型路径，默认使用 models/midas/midas_small.onnx
            inference_size: 模型输入分辨率，默认 256x256
            providers: ONNX Runtime execution providers，默认按优先级自动选择
        """
        self.model_path = model_path or DEFAULT_MODEL_PATH
        self.inference_size = inference_size
        self._session = None
        self._input_name = None
        self._available = False

        if not _check_onnxruntime():
            logger.warning("ONNX Runtime 未安装，DepthEstimator 将无法运行真实推理")
            return

        if not os.path.exists(self.model_path):
            logger.warning(
                f"深度估计模型未找到: {self.model_path}\n"
                f"请运行 'python tools/download_depth_models.py' 下载模型，"
                f"或手动放置 ONNX 模型到该路径。"
            )
            return

        import onnxruntime as ort

        if providers is None:
            providers = ort.get_available_providers()
            # 优先使用 GPU（CUDA/DirectML），否则 CPU
            preferred = ["CUDAExecutionProvider", "DmlExecutionProvider", "CPUExecutionProvider"]
            providers = [p for p in preferred if p in providers] or ["CPUExecutionProvider"]

        try:
            self._session = ort.InferenceSession(self.model_path, providers=providers)
            self._input_name = self._session.get_inputs()[0].name
            self._available = True
            logger.info(
                f"DepthEstimator 初始化成功: {self.model_path}, "
                f"providers={providers}, input_size={inference_size}"
            )
        except Exception as e:
            logger.error(f"加载 ONNX 模型失败: {e}")

    @property
    def is_available(self) -> bool:
        return self._available

    def estimate(self, bgr_image: np.ndarray) -> np.ndarray:
        """对输入 BGR 图像进行深度估计。

        Args:
            bgr_image: OpenCV BGR 图像，shape (H, W, 3)

        Returns:
            深度图，shape (H, W)，dtype float32，值域任意（相对深度）
        """
        if not self._available or self._session is None:
            raise RuntimeError(
                "DepthEstimator 不可用。请检查模型文件是否存在，以及 ONNX Runtime 是否已安装。"
            )

        orig_h, orig_w = bgr_image.shape[:2]

        # 1. 预处理：BGR -> RGB，resize，归一化
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        inp = cv2.resize(rgb, self.inference_size, interpolation=cv2.INTER_CUBIC)
        inp = inp.astype(np.float32) / 255.0

        # MiDaS 使用 ImageNet 标准化
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        inp = (inp - mean) / std
        inp = np.transpose(inp, (2, 0, 1))  # HWC -> CHW
        inp = np.expand_dims(inp, axis=0)   # add batch dim

        # 2. 推理
        depth = self._session.run(None, {self._input_name: inp})[0]

        # 3. 后处理
        depth = np.squeeze(depth)  # (H, W)
        depth = cv2.resize(depth, (orig_w, orig_h), interpolation=cv2.INTER_CUBIC)

        # 4. 归一化到 0~1（越远值越小）
        depth_min = depth.min()
        depth_max = depth.max()
        if depth_max - depth_min > 1e-6:
            depth = 1 - (depth - depth_min) / (depth_max - depth_min)
        else:
            depth = np.zeros_like(depth)

        return depth.astype(np.float32)

    def estimate_safe(self, bgr_image: np.ndarray) -> Optional[np.ndarray]:
        """安全版本的 estimate，失败时返回 None 而不是抛出异常。"""
        try:
            return self.estimate(bgr_image)
        except Exception as e:
            logger.error(f"深度估计失败: {e}")
            return None

    @staticmethod
    def colorize(depth: np.ndarray, colormap: int = cv2.COLORMAP_JET) -> np.ndarray:
        """将深度图转为伪彩色图（8-bit BGR），用于可视化。"""
        depth_u8 = 255-(depth * 255).clip(0, 255).astype(np.uint8)
        colored = cv2.applyColorMap(depth_u8, colormap)
        return colored


def create_depth_estimator(
    model_path: Optional[str] = None,
    inference_size: Tuple[int, int] = (256, 256),
    allow_fake: bool = True,
) -> DepthEstimator:
    """工厂函数：创建 DepthEstimator，"""
    real = DepthEstimator(model_path=model_path, inference_size=inference_size)
    if real.is_available:
        return real
    raise RuntimeError("深度估计器不可用")
