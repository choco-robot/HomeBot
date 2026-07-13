"""
麻将牌检测器 - SVP NNN C 后端适配层（单次调用模式）

适用场景：海思 SD3403 平台，C 后端为单次运行程序。
调用流程：
    1. Python 将图像帧写入临时文件
    2. 生成 JSON 文件列表（C 后端要求的输入格式）
    3. 调用 C 后端子进程（阻塞等待退出）
    4. 读取 C 后端输出的 TXT 结果文件
    5. 解析为 MahjongTile 列表

C 后端调用示例：
    ./main --model ../model/yolo11s-mj.om --input ../data/file_list_1.json

JSON 文件列表格式：
    {"loop": 1, "fileList": [["/path/to/image.jpg"]]}

C 后端结果文件格式示例：
    Class 3 | Score: 0.932617 | Box: [254.906, 43.375, 398.719, 212.125]
    Class 4 | Score: 0.874023 | Box: [1755, 70.75, 1899, 244.75]
"""

import json
import os
import re
import subprocess
import tempfile
from typing import List, Tuple, Optional
from dataclasses import dataclass, field
from pathlib import Path
from threading import Lock
import numpy as np

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from applications.mahjong_bot.detector import MahjongTile, TILE_CLASSES, TILE_CLASSES_ROBOFLOW
from common.logging import get_logger

logger = get_logger(__name__)


# 正则表达式：匹配单行检测结果
# Class {class_id} | Score: {confidence} | Box: [x1, y1, x2, y2]
_RESULT_LINE_RE = re.compile(
    r"Class\s+(\d+)\s+\|\s+Score:\s+([\d.eE+-]+)\s+\|\s+Box:\s+\[([^\]]+)\]"
)


@dataclass
class SvpNnnConfig:
    """SVP NNN 检测器配置"""
    # C 后端可执行文件路径
    executable_path: str = "./main"
    # C 后端模型路径（传给 --model）
    model_path: str = "../model/yolo11s-mj.om"
    # Python 写入临时输入图像的路径
    # 如果未指定（空字符串），默认使用 working_dir 下的 svp_nnn_input.jpg
    input_image_path: str = ""
    # JSON 文件列表路径（C 后端通过 --input 读取）
    # 如果未指定（空字符串），默认使用 working_dir 下的 svp_nnn_input.json
    json_file_path: str = ""
    # C 后端输出 TXT 结果文件的路径
    # 如果未指定（空字符串），自动推断为:
    #   {working_dir}/out/result/txt/{input_stem}_result.txt
    output_txt_path: str = ""
    # C 后端工作目录（如果 C 程序内部使用相对路径）
    # 临时文件默认会放在这个目录下
    working_dir: Optional[str] = None
    # 子进程调用超时（秒）
    exec_timeout: float = 10.0
    # 置信度阈值
    conf_threshold: float = 0.5
    # 是否使用 Roboflow 类别映射
    use_roboflow_classes: bool = True
    # ROI 配置
    roi_enabled: bool = False
    roi_x: int = 0
    roi_y: int = 0
    roi_width: int = 0
    roi_height: int = 0
    # C 后端推理时使用的输入图像分辨率（用于坐标缩放适配）
    # 设为 None 表示不缩放，使用原始坐标
    backend_width: Optional[int] = None
    backend_height: Optional[int] = None
    # 输入图片编码质量（JPG，0-100）
    jpeg_quality: int = 95
    # 是否清理临时文件
    cleanup_temp_files: bool = True


class SvpNnnDetector:
    """
    SVP NNN C 后端检测器（单次调用子进程模式）

    接口与 MahjongDetector 保持一致，可直接替换使用：
        detector = SvpNnnDetector(...)
        tiles = detector.detect(frame)  # 同步调用 C 后端
    """

    def __init__(self, config: Optional[SvpNnnConfig] = None, **kwargs):
        """
        初始化 SVP NNN 检测器

        Args:
            config: SvpNnnConfig 配置对象，优先级高于 kwargs
            **kwargs: 支持直接传入配置字段覆盖默认值
        """
        if config is None:
            config = SvpNnnConfig()

        # 允许 kwargs 覆盖 config 中的字段
        for key in vars(config):
            if key in kwargs:
                setattr(config, key, kwargs[key])

        self.config = config

        # 展开 ~ 为用户主目录
        def _expand(p):
            return Path(p).expanduser() if p else Path(p)

        # C 后端路径配置（先展开 ~）
        self.executable_path = _expand(config.executable_path)
        # 修复：保留 ./ 前缀（Path("./main") 会变成 "main"，导致 subprocess 找不到）
        if config.executable_path.startswith("./") and not str(self.executable_path).startswith("./"):
            self.executable_path = Path(".") / self.executable_path
        self.model_path = str(_expand(config.model_path)) if config.model_path else config.model_path

        # 确定工作目录：
        # 1. 如果用户指定了，就用用户的
        # 2. 如果用户没指定，用可执行文件所在目录
        # 3. 如果连可执行文件路径都没法解析，用当前目录
        if config.working_dir:
            self.working_dir = _expand(config.working_dir)
        elif self.executable_path.parent.exists():
            self.working_dir = self.executable_path.parent
        else:
            self.working_dir = Path.cwd()

        # 确定临时文件路径（未指定时默认放到工作目录下）
        if config.input_image_path:
            self.input_image_path = _expand(config.input_image_path)
        else:
            self.input_image_path = self.working_dir / "svp_nnn_input.jpg"

        if config.json_file_path:
            self.json_file_path = _expand(config.json_file_path)
        else:
            self.json_file_path = self.working_dir / "svp_nnn_input.json"

        # 确定输出 TXT 路径
        if config.output_txt_path:
            self.output_txt_path = _expand(config.output_txt_path)
        else:
            # 自动推断：根据输入图片文件名 + C 后端命名规则
            # C 后端命名规则：{input_stem}_result.txt，放在 out/result/txt/ 下
            input_stem = self.input_image_path.stem
            self.output_txt_path = self.working_dir / "result" / "txt" / f"{input_stem}_result.txt"
        self.exec_timeout = config.exec_timeout
        self.jpeg_quality = config.jpeg_quality
        self.cleanup_temp_files = config.cleanup_temp_files

        # 检测参数
        self.conf_threshold = config.conf_threshold
        self.use_roboflow_classes = config.use_roboflow_classes
        self.tile_classes = TILE_CLASSES_ROBOFLOW if config.use_roboflow_classes else TILE_CLASSES

        # ROI 配置
        self.roi_enabled = config.roi_enabled
        self.roi_x = config.roi_x
        self.roi_y = config.roi_y
        self.roi_width = config.roi_width
        self.roi_height = config.roi_height

        # 坐标缩放配置
        self.backend_width = config.backend_width
        self.backend_height = config.backend_height

        # 内部状态
        self._initialized = False
        self._call_lock = Lock()  # 防止并发调用 C 后端

        logger.info("SvpNnnDetector 初始化:")
        logger.info(f"  可执行文件: {self.executable_path}")
        logger.info(f"  模型路径: {self.model_path}")
        logger.info(f"  工作目录: {self.working_dir}")
        logger.info(f"  输入图像: {self.input_image_path}")
        logger.info(f"  JSON 文件: {self.json_file_path}")
        logger.info(f"  输出 TXT: {self.output_txt_path} (自动推断)" if not config.output_txt_path else f"  输出 TXT: {self.output_txt_path}")
        logger.info(f"  调用超时: {self.exec_timeout}s")
        logger.info(f"  置信度阈值: {self.conf_threshold}")
        logger.info(f"  类别映射: {'Roboflow' if self.use_roboflow_classes else 'Standard'}")
        logger.info(f"  ROI 启用: {self.roi_enabled}")
        if self.roi_enabled:
            logger.info(f"  ROI 区域: x={self.roi_x}, y={self.roi_y}, w={self.roi_width}, h={self.roi_height}")
        logger.info(f"  后端图像分辨率: {self.backend_width}x{self.backend_height}")

    def _is_within_roi(self, tile: MahjongTile) -> bool:
        """判断检测结果是否位于 ROI 区域内（使用中心点判断）"""
        if not self.roi_enabled or self.roi_width <= 0 or self.roi_height <= 0:
            return True
        cx, cy = tile.center
        return (self.roi_x <= cx <= self.roi_x + self.roi_width and
                self.roi_y <= cy <= self.roi_y + self.roi_height)

    def _write_input_image(self, frame: np.ndarray) -> bool:
        """
        将图像帧写入临时文件，供 C 后端读取

        Args:
            frame: OpenCV 图像帧 (BGR 格式)

        Returns:
            bool: 是否写入成功
        """
        import cv2
        try:
            # 确保父目录存在
            self.input_image_path.parent.mkdir(parents=True, exist_ok=True)
            # 使用 JPG 编码，质量可配置
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            success, buf = cv2.imencode(".jpg", frame, encode_params)
            if not success:
                logger.error("图像编码失败")
                return False
            self.input_image_path.write_bytes(buf.tobytes())
            logger.debug(f"输入图像已写入: {self.input_image_path} ({self.input_image_path.stat().st_size} bytes)")
            return True
        except Exception as e:
            logger.error(f"写入输入图像失败: {e}")
            return False

    def _build_json_file(self) -> bool:
        """
        生成 C 后端要求的 JSON 文件列表

        Returns:
            bool: 是否生成成功
        """
        try:
            data = {
                "loop": 1,
                "fileList": [
                    [str(self.input_image_path.resolve())]
                ]
            }
            self.json_file_path.parent.mkdir(parents=True, exist_ok=True)
            self.json_file_path.write_text(json.dumps(data, indent=4), encoding="utf-8")
            logger.debug(f"JSON 文件已写入: {self.json_file_path}")
            return True
        except Exception as e:
            logger.error(f"生成 JSON 文件失败: {e}")
            return False

    def _call_backend(self) -> bool:
        """
        调用 C 后端子进程，阻塞等待其完成

        Returns:
            bool: C 后端是否成功执行
        """
        if not self.executable_path.exists():
            logger.error(f"C 后端可执行文件不存在: {self.executable_path}")
            return False

        cmd = [
            str(self.executable_path),
            "--model", self.model_path,
            "--input", str(self.json_file_path.resolve()),
        ]

        logger.info(f"调用 C 后端: {' '.join(cmd)}")
        logger.info(f"工作目录: {self.working_dir}")

        try:
            result = subprocess.run(
                cmd,
                cwd=str(self.working_dir),
                capture_output=True,
                text=True,
                timeout=self.exec_timeout,
            )

            if result.returncode != 0:
                logger.error(f"C 后端返回非零退出码: {result.returncode}")
                if result.stderr:
                    logger.error(f"C 后端 stderr: {result.stderr[:1000]}")
                if result.stdout:
                    logger.error(f"C 后端 stdout: {result.stdout[:1000]}")
                return False

            logger.info("C 后端执行成功")
            if result.stdout:
                logger.debug(f"C 后端 stdout: {result.stdout[:500]}")
            return True

        except subprocess.TimeoutExpired:
            logger.error(f"C 后端调用超时（>{self.exec_timeout}s）")
            return False
        except Exception as e:
            logger.error(f"调用 C 后端失败: {e}")
            return False

    def _parse_result_file(self, file_path: Path) -> List[MahjongTile]:
        """
        解析 C 后端写入的 TXT 结果文件

        Args:
            file_path: 结果文件路径

        Returns:
            List[MahjongTile]: 解析后的检测结果列表
        """
        detections = []

        if not file_path.exists():
            logger.warning(f"结果文件不存在: {file_path}")
            return []

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
        except Exception as e:
            logger.warning(f"读取结果文件失败: {e}")
            return []

        for line in content.strip().splitlines():
            line = line.strip()
            if not line:
                continue

            match = _RESULT_LINE_RE.match(line)
            if not match:
                logger.debug(f"无法解析的行: {line}")
                continue

            try:
                class_id = int(match.group(1))
                confidence = float(match.group(2))
                box_str = match.group(3)

                # 解析 Box 坐标 [x1, y1, x2, y2]
                coords = [float(x.strip()) for x in box_str.split(",")]
                if len(coords) != 4:
                    logger.debug(f"Box 坐标数量不对: {box_str}")
                    continue

                x1, y1, x2, y2 = coords

                # 置信度过滤
                if confidence < self.conf_threshold:
                    continue

                # 转换为整数 bbox
                bbox = (int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2)))

                # 类别名称映射
                cls_name = self.tile_classes.get(class_id, f"unknown-{class_id}")

                detections.append(MahjongTile(
                    bbox=bbox,
                    confidence=confidence,
                    class_id=class_id,
                    class_name=cls_name,
                    track_id=None  # C 后端目前不提供跟踪 ID
                ))

            except Exception as e:
                logger.debug(f"解析单行结果失败: {e}")
                continue

        return detections

    def _apply_scale(self, detections: List[MahjongTile], frame: Optional[np.ndarray]) -> List[MahjongTile]:
        """
        如果 C 后端推理分辨率与当前 frame 分辨率不一致，缩放 bbox 坐标

        Args:
            detections: 原始检测结果（基于 backend_width/backend_height）
            frame: Python 层当前图像帧

        Returns:
            缩放后的检测结果
        """
        if frame is None or frame.size == 0:
            return detections

        if self.backend_width is None or self.backend_height is None:
            return detections

        frame_h, frame_w = frame.shape[:2]

        # 如果分辨率已经一致，无需缩放
        if frame_w == self.backend_width and frame_h == self.backend_height:
            return detections

        scale_x = frame_w / self.backend_width
        scale_y = frame_h / self.backend_height

        scaled = []
        for tile in detections:
            x1, y1, x2, y2 = tile.bbox
            new_bbox = (
                int(round(x1 * scale_x)),
                int(round(y1 * scale_y)),
                int(round(x2 * scale_x)),
                int(round(y2 * scale_y)),
            )
            scaled.append(MahjongTile(
                bbox=new_bbox,
                confidence=tile.confidence,
                class_id=tile.class_id,
                class_name=tile.class_name,
                track_id=tile.track_id
            ))

        if scaled:
            logger.debug(f"坐标缩放: {self.backend_width}x{self.backend_height} -> {frame_w}x{frame_h} "
                        f"(scale_x={scale_x:.3f}, scale_y={scale_y:.3f})")

        return scaled

    def _cleanup_temp_files(self):
        """清理临时文件（输入图像、JSON 文件）"""
        if not self.cleanup_temp_files:
            return
        for path in [self.input_image_path, self.json_file_path]:
            try:
                if path.exists():
                    path.unlink()
                    logger.debug(f"清理临时文件: {path}")
            except Exception:
                pass

    def initialize(self) -> bool:
        """初始化检测器（检查可执行文件是否存在）"""
        if self._initialized:
            return True

        if not self.executable_path.exists():
            logger.warning(f"C 后端可执行文件暂不存在: {self.executable_path}")
            logger.info("请确认 C 后端程序已编译并路径正确")
        else:
            logger.info(f"✓ C 后端可执行文件就绪: {self.executable_path}")

        self._initialized = True
        logger.info("SvpNnnDetector 初始化完成")
        return True

    def detect(self, frame: np.ndarray = None) -> List[MahjongTile]:
        """
        检测图像中的麻将牌（同步调用 C 后端子进程）

        执行流程：
            1. 将 frame 写入临时图片文件
            2. 生成 JSON 文件列表
            3. 调用 C 后端子进程（阻塞等待）
            4. 读取并解析 TXT 结果
            5. 返回 MahjongTile 列表

        Args:
            frame: OpenCV 图像帧 (BGR 格式)。如果为 None，则跳过图像写入
                   （仅读取已有的 C 后端结果，用于调试）

        Returns:
            List[MahjongTile]: 检测结果列表，按 x 坐标从左到右排序
        """
        if not self._initialized:
            if not self.initialize():
                return []

        if frame is not None and frame.size == 0:
            logger.warning("输入图像为空")
            return []

        # 加锁防止并发调用 C 后端（单次运行程序不支持并发）
        with self._call_lock:
            # 1. 写入输入图像（如果提供了 frame）
            if frame is not None:
                if not self._write_input_image(frame):
                    return []

                # 2. 生成 JSON 文件列表
                if not self._build_json_file():
                    return []

                # 3. 调用 C 后端
                if not self._call_backend():
                    return []

            # 4. 读取并解析结果
            detections = self._parse_result_file(self.output_txt_path)

            # 5. 可选清理临时文件
            if frame is not None:
                self._cleanup_temp_files()

        # 坐标缩放适配
        detections = self._apply_scale(detections, frame)

        # ROI 过滤
        if self.roi_enabled and self.roi_width > 0 and self.roi_height > 0:
            detections = [t for t in detections if self._is_within_roi(t)]

        # 按 x 坐标从左到右排序
        detections.sort(key=lambda t: t.center[0])
        
        if detections:
            names = [t.class_name for t in detections]
            logger.info(f"检测到 {len(detections)} 张牌: {names}")
        else:
            logger.info("未检测到牌")

        logger.debug(f"detect() 返回 {len(detections)} 个目标")
        return detections

    def detect_and_draw(self, frame: np.ndarray) -> Tuple[np.ndarray, List[MahjongTile]]:
        """
        检测并在图像上绘制结果

        Returns:
            (绘制后的图像, 检测结果列表)
        """
        import cv2

        detections = self.detect(frame)
        output = frame.copy() if frame is not None else None

        if output is not None:
            for tile in detections:
                x1, y1, x2, y2 = tile.bbox
                cx, cy = tile.center

                cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(output, (cx, cy), 4, (0, 0, 255), -1)

                label = f"{tile.class_name}: {tile.confidence:.2f}"
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                label_y = max(y1 - 10, label_size[1] + 10)

                cv2.rectangle(output,
                             (x1, label_y - label_size[1] - 5),
                             (x1 + label_size[0], label_y + 5),
                             (0, 255, 0), -1)
                cv2.putText(output, label, (x1, label_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            info_text = f"Tiles: {len(detections)}"
            cv2.putText(output, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return output, detections

    def release(self):
        """释放资源"""
        self._initialized = False
        self._cleanup_temp_files()
        logger.info("SvpNnnDetector 已释放")
