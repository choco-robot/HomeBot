# -*- coding: utf-8 -*-
"""演示如何加载地图编辑器导出的JSON文件

使用方法：
1. 在地图编辑器中创建地图
2. 点击"保存地图"按钮，保存为 JSON + PNG 文件
3. 运行本脚本，指定 JSON 文件路径（PNG 会自动加载）

示例：
    python demo_map_editor_loader.py map_2026-01-15.json
"""

import os
import sys

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.navigation.simulation.map_environment import MapEnvironment

def visualize_map_only(json_file: str):
    """Load and visualize map only (no navigation)

    Args:
        json_file: JSON file path
    """
    import matplotlib.pyplot as plt
    import numpy as np

    print(f"\nLoading map: {json_file}")

    # Create map environment
    map_env = MapEnvironment(map_file=json_file)

    # 可视化
    fig, ax = plt.subplots(figsize=(10, 10))

    # 绘制栅格地图
    grid_data = map_env.get_grid_data()

    ax.imshow(
        grid_data,
        cmap="gray_r",
        origin="lower",
        extent=[
            map_env.origin[0],
            map_env.origin[0] + map_env.width,
            map_env.origin[1],
            map_env.origin[1] + map_env.height,
        ],
    )

    # 绘制障碍物边界
    for obs in map_env.obstacles.values():
        if obs.type == "circle":
            x, y, radius = obs.data
            circle = plt.Circle((x, y), radius, fill=False, color="red", linewidth=2)
            ax.add_patch(circle)
        elif obs.type == "polygon":
            vertices = obs.data
            polygon = plt.Polygon(vertices, fill=False, color="red", linewidth=2)
            ax.add_patch(polygon)

    # 设置图形属性
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"Map from Editor\nSize: {map_env.width:.1f}m x {map_env.height:.1f}m")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def main():
    """Main function"""
    # Check command line arguments
    if len(sys.argv) < 2:
        print("\nUsage:")
        print(f"  python {os.path.basename(__file__)} <map_json_file> [mode]")
        print("\nExamples:")
        print(f"  python {os.path.basename(__file__)} map_2026-01-15.json")
        return

    json_file = sys.argv[1]

    # Check if file exists
    if not os.path.exists(json_file):
        print(f"\nError: File not found: {json_file}")
        return
    
    visualize_map_only(json_file)


if __name__ == "__main__":
    main()
