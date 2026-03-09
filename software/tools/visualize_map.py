# -*- coding: utf-8 -*-
"""SLAM 地图文件可视化工具（灰度原始数据）

将 SLAMService 保存的 `.npz` 地图文件以原始灰度值渲染，
不做任何颜色主题映射，直接显示 BreezySLAM 原始地图数据（0-255）。

在图像上标注：
- 保存时的机器人位姿（红点 + 朝向箭头）
- 0=黑(空闲)、127=灰(未知)、255=白(占据) 的色标
- 地图尺寸、分辨率、保存时间等元信息

Usage:
    # 直接显示
    python tools/visualize_map.py maps/home_map.npz --show

    # 保存为 PNG
    python tools/visualize_map.py maps/home_map.npz --output home_map.png

    # 批量处理
    python tools/visualize_map.py maps/*.npz --output-dir map_images/
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from common.logging import get_logger

logger = get_logger(__name__)

COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)
COLOR_YELLOW = (0, 255, 255)


def render_map(
    map_gray: np.ndarray,
    pose: Optional[Tuple[float, float, float]] = None,
    map_size_meters: Optional[float] = None,
    show_grid: bool = True,
    grid_interval_m: float = 1.0,
) -> np.ndarray:
    """将原始灰度栅格地图渲染为 BGR 图像。

    map_gray 直接作为灰度值复制到 BGR 三通道，不做任何阈值映射。
    0=黑(空闲), 127=灰(未知), 255=白(占据) 的色标显示在右侧。
    """
    h, w = map_gray.shape

    # 原始灰度复制到 BGR 三通道
    img = np.stack([map_gray, map_gray, map_gray], axis=2).astype(np.uint8)

    # 绘制网格
    if show_grid and map_size_meters is not None and map_size_meters > 0:
        resolution = map_size_meters / w
        grid_px = int(grid_interval_m / resolution)
        if grid_px >= 5:
            grid_color = (128, 128, 128)
            for i in range(0, w, grid_px):
                cv2.line(img, (i, 0), (i, h - 1), grid_color, 1)
            for j in range(0, h, grid_px):
                cv2.line(img, (0, j), (w - 1, j), grid_color, 1)

    # 在地图上标注保存时的位姿
    if pose is not None and map_size_meters is not None:
        x_m, y_m, theta = pose
        resolution = map_size_meters / w
        origin = -map_size_meters / 2.0

        # 世界坐标 -> 图像像素坐标
        # BreezySLAM y 向下增加，与图像 y 轴一致
        px = int((x_m - origin) / resolution)
        py = int((y_m - origin) / resolution)

        # 裁剪到图像范围内
        px = max(0, min(w - 1, px))
        py = max(0, min(h - 1, py))

        arrow_len = max(15, int(w * 0.03))
        dx = int(arrow_len * math.cos(theta))
        dy = int(arrow_len * math.sin(theta))

        cv2.circle(img, (px, py), 5, COLOR_RED, -1)
        cv2.arrowedLine(
            img, (px, py), (px + dx, py + dy), COLOR_GREEN, 2, tipLength=0.3
        )

    # 右侧添加色标条
    bar_w = max(30, int(w * 0.04))
    bar_h = h
    colorbar = np.zeros((bar_h, bar_w, 3), dtype=np.uint8)
    for y in range(bar_h):
        val = int(255 * (1.0 - y / bar_h))
        colorbar[y, :] = (val, val, val)

    # 色标标签
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = max(0.3, bar_w / 80.0)
    thickness = 1
    labels = [
        ("255 occupied", 0, COLOR_RED),
        ("200", int(bar_h * 0.2), COLOR_YELLOW),
        ("127 unknown", int(bar_h * 0.5), COLOR_YELLOW),
        ("50", int(bar_h * 0.8), COLOR_YELLOW),
        ("0 free", bar_h - 5, COLOR_GREEN),
    ]
    for text, y_pos, color in labels:
        cv2.putText(colorbar, text, (2, y_pos), font, font_scale, color, thickness)

    # 拼接主图 + 色标条
    img = np.hstack([img, colorbar])
    return img


def load_npz_map(path: str) -> dict:
    """加载 SLAMService 保存的 .npz 地图文件。"""
    data = np.load(path)
    map_bytes = data["map_bytes"]
    size_pixels = int(data["map_size_pixels"])
    size_meters = float(data["map_size_meters"])

    map_gray = map_bytes.astype(np.uint8).reshape((size_pixels, size_pixels))

    pose = None
    if "pose_x" in data and "pose_y" in data and "pose_theta" in data:
        pose = (
            float(data["pose_x"]),
            float(data["pose_y"]),
            float(data["pose_theta"]),
        )

    timestamp = float(data["timestamp"]) if "timestamp" in data else None

    return {
        "map_gray": map_gray,
        "map_size_pixels": size_pixels,
        "map_size_meters": size_meters,
        "pose": pose,
        "timestamp": timestamp,
    }


def visualize_map(
    npz_path: str,
    output_path: Optional[str] = None,
    show: bool = False,
    show_grid: bool = True,
    grid_interval_m: float = 1.0,
    scale: Optional[int] = None,
) -> Optional[np.ndarray]:
    """加载并渲染单个 .npz 地图文件。"""
    npz_path = Path(npz_path)
    if not npz_path.exists():
        logger.error(f"文件不存在: {npz_path}")
        return None

    try:
        info = load_npz_map(str(npz_path))
    except Exception as e:
        logger.error(f"加载地图失败 [{npz_path}]: {e}")
        return None

    map_gray = info["map_gray"]
    size_pixels = info["map_size_pixels"]
    size_meters = info["map_size_meters"]
    pose = info["pose"]
    timestamp = info["timestamp"]

    img = render_map(
        map_gray,
        pose=pose,
        map_size_meters=size_meters,
        show_grid=show_grid,
        grid_interval_m=grid_interval_m,
    )

    # 可选放大
    if scale and scale > 1:
        img = cv2.resize(
            img, (img.shape[1] * scale, img.shape[0] * scale), interpolation=cv2.INTER_NEAREST
        )

    # 叠加信息文字（左上角黑底白字）
    text_lines = [
        f"Map: {size_pixels}x{size_pixels} ({size_meters}m)",
        f"Res: {size_meters / size_pixels:.3f} m/px",
        f"Min/Max/Mean: {map_gray.min()}/{map_gray.max()}/{map_gray.mean():.1f}",
    ]
    if pose:
        text_lines.append(
            f"Pose: ({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.1f}deg)"
        )
    if timestamp:
        import time
        time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp))
        text_lines.append(f"Saved: {time_str}")

    # 画黑色背景条
    line_h = 18
    pad = 4
    bg_h = len(text_lines) * line_h + pad * 2
    bg_w = max(len(t) for t in text_lines) * 9 + pad * 2
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (bg_w, bg_h), (0, 0, 0), -1)
    img = cv2.addWeighted(img, 0.3, overlay, 0.7, 0)

    y_offset = line_h
    for line in text_lines:
        cv2.putText(img, line, (pad, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        y_offset += line_h

    # 保存
    if output_path:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_path), img)
        logger.info(f"图片已保存: {output_path}")

    # 显示
    if show:
        window_name = f"Map: {npz_path.name}"
        cv2.imshow(window_name, img)
        logger.info("按任意键关闭窗口...")
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)

    return img


def main():
    parser = argparse.ArgumentParser(description="SLAM 地图 .npz 文件可视化工具（灰度原始数据）")
    parser.add_argument("input", nargs="+", help="输入 .npz 地图文件路径（支持多个）")
    parser.add_argument("-o", "--output", help="输出图片路径（单文件时有效）")
    parser.add_argument("--output-dir", help="输出目录（多文件批量处理时有效）")
    parser.add_argument("--show", action="store_true", help="用 OpenCV 显示图片")
    parser.add_argument("--no-grid", action="store_true", help="不绘制网格线")
    parser.add_argument("--grid-interval", type=float, default=1.0, help="网格间隔（米）")
    parser.add_argument("--scale", type=int, default=None, help="输出缩放倍数（如 2=放大2倍）")
    args = parser.parse_args()

    input_paths = args.input
    show_grid = not args.no_grid

    # 批量处理
    if len(input_paths) > 1 or args.output_dir:
        output_dir = Path(args.output_dir) if args.output_dir else Path(".")
        output_dir.mkdir(parents=True, exist_ok=True)

        for p in input_paths:
            src = Path(p)
            dst = output_dir / f"{src.stem}.png"
            visualize_map(
                str(src),
                output_path=str(dst),
                show=False,
                show_grid=show_grid,
                grid_interval_m=args.grid_interval,
                scale=args.scale,
            )
        logger.info(f"批量处理完成，共 {len(input_paths)} 个文件")
        return

    # 单文件处理
    visualize_map(
        input_paths[0],
        output_path=args.output,
        show=args.show,
        show_grid=show_grid,
        grid_interval_m=args.grid_interval,
        scale=args.scale,
    )


if __name__ == "__main__":
    main()
