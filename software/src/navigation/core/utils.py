# -*- coding: utf-8 -*-
"""导航通用工具函数"""
from typing import Tuple


def world_to_grid(
    wx: float,
    wy: float,
    resolution: float,
    origin: Tuple[float, float] = (0.0, 0.0),
) -> Tuple[int, int]:
    """世界坐标转栅格坐标"""
    gx = int((wx - origin[0]) / resolution)
    gy = int((wy - origin[1]) / resolution)
    return gx, gy


def grid_to_world(
    gx: int,
    gy: int,
    resolution: float,
    origin: Tuple[float, float] = (0.0, 0.0),
) -> Tuple[float, float]:
    """栅格坐标转世界坐标（栅格中心点）"""
    wx = origin[0] + (gx + 0.5) * resolution
    wy = origin[1] + (gy + 0.5) * resolution
    return wx, wy
