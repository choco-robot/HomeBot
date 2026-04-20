# -*- coding: utf-8 -*-
"""Viser SLAM 可视化器入口

Usage:
    cd software/src
    python -m navigation.visualization

或带参数:
    python -m navigation.visualization --port 8080 --lidar-scan tcp://localhost:5565
"""
from navigation.visualization.viser_slam_visualizer import main

if __name__ == "__main__":
    main()
