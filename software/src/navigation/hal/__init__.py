# -*- coding: utf-8 -*-
"""导航硬件抽象层 (HAL)

包含激光雷达等导航专用硬件驱动。
"""
from navigation.hal.lidar_driver import LD06Driver, create_lidar_driver

__all__ = ["LD06Driver", "create_lidar_driver"]
