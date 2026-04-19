# -*- coding: utf-8 -*-
"""基于 Matplotlib 的 2D 导航模拟器"""
from __future__ import annotations

from typing import Callable, List, Optional, Tuple

import numpy as np

from common.logging import get_logger
from navigation.core.occupancy_grid import COST_FREE, COST_LETHAL, COST_UNKNOWN, OccupancyGrid

logger = get_logger(__name__)

# 尝试导入 matplotlib，未安装时给出友好提示
try:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    _MATPLOTLIB_AVAILABLE = True
except ImportError:
    _MATPLOTLIB_AVAILABLE = False
    plt = None  # type: ignore
    Circle = None  # type: ignore


def _check_matplotlib() -> None:
    if not _MATPLOTLIB_AVAILABLE:
        raise ImportError(
            "运行模拟器需要 matplotlib，请执行: pip install matplotlib"
        )


class SimVisualizer:
    """导航模拟器可视化工具。

    功能：
    - 显示栅格地图、起点、终点、规划路径
    - 支持点击地图设置起点/终点并触发规划回调
    - 显示机器人当前位姿
    """

    def __init__(
        self,
        grid: OccupancyGrid,
        title: str = "HomeBot Navigation Simulator",
    ):
        _check_matplotlib()
        self.grid = grid
        self.title = title
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self._setup_plot()

        # 交互状态
        self.start_point: Optional[Tuple[float, float]] = None
        self.goal_point: Optional[Tuple[float, float]] = None
        self.path: Optional[List[Tuple[float, float]]] = None
        self.robot_pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw
        self._click_callback: Optional[
            Callable[[Tuple[float, float], Tuple[float, float]], None]
        ] = None
        self._click_mode: str = "start"  # 'start' -> 'goal'

        # 图形元素引用（用于增量更新）
        self._start_scatter = None
        self._goal_scatter = None
        self._path_line = None
        self._robot_patch = None

    def _setup_plot(self) -> None:
        """初始化 matplotlib 坐标系和地图背景"""
        self.ax.clear()
        self.ax.set_title(self.title)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")

        # 构造可显示的地图数组
        display_data = self.grid.data.astype(np.float32).copy()
        display_data[display_data == COST_UNKNOWN] = np.nan

        #  extent: [left, right, bottom, top]
        ox, oy = self.grid.origin
        w = self.grid.width * self.grid.resolution
        h = self.grid.height * self.grid.resolution
        extent = [ox, ox + w, oy + h, oy]

        self.ax.imshow(
            display_data,
            cmap="gray_r",
            vmin=COST_FREE,
            vmax=COST_LETHAL,
            extent=extent,
            origin="upper",
            interpolation="nearest",
        )
        self.ax.set_xlim(ox, ox + w)
        self.ylim_bottom = oy
        self.ylim_top = oy + h
        self.ax.set_ylim(oy, oy + h)

    def set_click_callback(
        self,
        callback: Callable[[Tuple[float, float], Tuple[float, float]], None],
    ) -> None:
        """设置点击回调：当起点和终点都被点击后，调用 callback(start, goal)"""
        self._click_callback = callback
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)

    def _on_click(self, event) -> None:
        if event.inaxes != self.ax:
            return
        wx, wy = event.xdata, event.ydata
        if wx is None or wy is None:
            return

        if self._click_mode == "start":
            self.start_point = (wx, wy)
            self._draw_start()
            self._click_mode = "goal"
            logger.info(f"模拟器：设置起点 ({wx:.2f}, {wy:.2f})")
        else:
            self.goal_point = (wx, wy)
            self._draw_goal()
            self._click_mode = "start"
            logger.info(f"模拟器：设置终点 ({wx:.2f}, {wy:.2f})")
            if self._click_callback and self.start_point:
                self._click_callback(self.start_point, self.goal_point)

        self.fig.canvas.draw_idle()

    def _draw_start(self) -> None:
        if self._start_scatter:
            self._start_scatter.remove()
        self._start_scatter = self.ax.scatter(
            [self.start_point[0]],
            [self.start_point[1]],
            c="green",
            s=100,
            marker="o",
            label="Start",
            zorder=5,
        )

    def _draw_goal(self) -> None:
        if self._goal_scatter:
            self._goal_scatter.remove()
        self._goal_scatter = self.ax.scatter(
            [self.goal_point[0]],
            [self.goal_point[1]],
            c="red",
            s=100,
            marker="X",
            label="Goal",
            zorder=5,
        )

    def update_path(self, path: Optional[List[Tuple[float, float]]]) -> None:
        """更新并显示路径"""
        self.path = path
        if self._path_line:
            self._path_line.remove()
            self._path_line = None

        if path and len(path) > 1:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            (self._path_line,) = self.ax.plot(
                xs, ys, "b-", linewidth=2, label="Path", zorder=4
            )
            # 添加路径点标记
            self.ax.plot(xs, ys, "b.", markersize=4, zorder=4)
        self.fig.canvas.draw_idle()

    def update_robot(self, x: float, y: float, yaw: float = 0.0, radius_m: float = 0.15) -> None:
        """更新机器人位姿显示"""
        self.robot_pose = (x, y, yaw)
        if self._robot_patch:
            self._robot_patch.remove()

        # 绘制机器人圆盘 + 方向指示线
        self._robot_patch = Circle(
            (x, y),
            radius_m,
            color="cyan",
            ec="black",
            linewidth=1.5,
            zorder=6,
        )
        self.ax.add_patch(self._robot_patch)

        # 方向线
        dx = radius_m * np.cos(yaw)
        dy = radius_m * np.sin(yaw)
        self.ax.arrow(
            x, y, dx, dy,
            head_width=radius_m * 0.3,
            head_length=radius_m * 0.3,
            fc="black",
            ec="black",
            zorder=7,
        )
        self.fig.canvas.draw_idle()

    def show(self, block: bool = True) -> None:
        """显示窗口"""
        self.ax.legend(loc="upper right")
        plt.tight_layout()
        plt.show(block=block)

    def save_screenshot(self, path: str) -> None:
        """保存当前画面到文件"""
        self.fig.savefig(path, dpi=150)
        logger.info(f"模拟器截图已保存: {path}")


def run_demo() -> None:
    """运行一个内置 Demo：随机地图 + A* 规划 + 交互式点击"""
    from navigation.core.astar_planner import AStarPlanner

    # 创建 5m x 5m 地图，分辨率 5cm
    grid = OccupancyGrid(width=100, height=100, resolution=0.05, origin=(-2.5, -2.5))
    # 添加一些固定障碍物
    grid.set_rectangle(30, 30, 40, 10, COST_LETHAL)   # 横墙
    grid.set_rectangle(20, 60, 10, 25, COST_LETHAL)   # 左竖墙
    grid.set_rectangle(70, 50, 10, 30, COST_LETHAL)   # 右竖墙
    grid.inflate_obstacles(0.1)

    planner = AStarPlanner(grid, allow_diagonal=True)
    viz = SimVisualizer(grid, title="HomeBot A* Simulator (Click to set start/goal)")

    def on_plan(start, goal):
        path = planner.plan_with_simplification(start, goal)
        viz.update_path(path)
        if path:
            viz.update_robot(path[0][0], path[0][1])

    viz.set_click_callback(on_plan)

    # 预设一个起点和机器人
    viz.start_point = (-1.5, -1.5)
    viz._draw_start()
    viz.update_robot(-1.5, -1.5, yaw=0.0)

    logger.info("模拟器已启动，请在地图上依次点击设置起点和终点")
    viz.show()


if __name__ == "__main__":
    run_demo()
