#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轮速曲线可视化测试工具

直接连接底盘硬件，执行一系列标准运动测试，
绘制命令速度 vs 实际轮速的对比曲线，用于标定和验证。

用法:
    cd software
    python tools/wheel_speed_visualizer.py --port /dev/ttyUSB0
    python tools/wheel_speed_visualizer.py --port COM3 --save plot.png
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, List, Optional, Tuple

# 将 src 加入路径
sys.path.insert(0, "src")

from configs import get_config
from hal.chassis.driver import ChassisDriver


try:
    import matplotlib

    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import numpy as np

    _MATPLOTLIB_AVAILABLE = True
except ImportError:
    _MATPLOTLIB_AVAILABLE = False
    plt = None
    np = None


class WheelSpeedRecord:
    """单次采样记录"""

    def __init__(
        self,
        t: float,
        test_name: str,
        cmd_vx: float,
        cmd_vy: float,
        cmd_omega: float,
        cmd_wheels: Tuple[float, float, float],
        act_wheels: Tuple[Optional[float], Optional[float], Optional[float]],
        raw_speeds: Tuple[Optional[int], Optional[int], Optional[int]],
        act_robot: Tuple[Optional[float], Optional[float], Optional[float]],
    ):
        self.t = t
        self.test_name = test_name
        self.cmd_vx = cmd_vx
        self.cmd_vy = cmd_vy
        self.cmd_omega = cmd_omega
        self.cmd_lf, self.cmd_rf, self.cmd_re = cmd_wheels
        self.act_lf, self.act_rf, self.act_re = act_wheels
        self.raw_lf, self.raw_rf, self.raw_re = raw_speeds
        self.act_vx, self.act_vy, self.act_omega = act_robot


class WheelSpeedVisualizer:
    """轮速测试与可视化"""

    def __init__(self, port: Optional[str] = None, sample_rate: float = 20.0):
        config = get_config().chassis
        if port:
            config.serial_port = port

        self.driver = ChassisDriver(config)
        self.sample_interval = 1.0 / sample_rate
        self.records: List[WheelSpeedRecord] = []

    def connect(self) -> bool:
        print("[Visualizer] 正在连接底盘...")
        if not self.driver.initialize():
            print("[Visualizer] 底盘连接失败！")
            return False
        print("[Visualizer] 底盘连接成功")
        print(f"[Visualizer] 轮子: {self.driver.wheel_ids}")
        print(f"[Visualizer] 轮半径: {self.driver.config.wheel_radius} m")
        print(f"[Visualizer] 底盘半径: {self.driver.config.chassis_radius} m")
        print(f"[Visualizer] 采样频率: {1.0/self.sample_interval:.0f} Hz")
        return True

    def run_test_sequence(self) -> None:
        """执行标准测试序列"""
        tests: List[Tuple[str, float, float, float, float]] = [
            ("前进", 0.2, 0.0, 0.0, 2.0),
            ("停止", 0.0, 0.0, 0.0, 1.0),
            ("后退", -0.2, 0.0, 0.0, 2.0),
            ("停止", 0.0, 0.0, 0.0, 1.0),
            ("左移", 0.0, 0.2, 0.0, 2.0),
            ("停止", 0.0, 0.0, 0.0, 1.0),
            ("右移", 0.0, -0.2, 0.0, 2.0),
            ("停止", 0.0, 0.0, 0.0, 1.0),
            ("逆时针转", 0.0, 0.0, 0.5, 2.0),
            ("停止", 0.0, 0.0, 0.0, 1.0),
            ("顺时针转", 0.0, 0.0, -0.5, 2.0),
            ("停止", 0.0, 0.0, 0.0, 1.0),
        ]

        print("\n[Visualizer] 开始测试序列，请确保机器人周围有足够空间...")
        time.sleep(1.0)

        for name, vx, vy, omega, duration in tests:
            print(f"[TEST] {name:12s} | vx={vx:+.2f}  vy={vy:+.2f}  ω={omega:+.2f}  | 持续 {duration}s")
            self.driver.set_velocity(vx, vy, omega)
            t_start = time.time()

            while time.time() - t_start < duration:
                t0 = time.time()

                # 命令轮速（逆运动学）
                cmd_wheels = self.driver._inverse_kinematics(
                    self.driver._current_vx,
                    self.driver._current_vy,
                    self.driver._current_omega,
                )

                # 实际轮速
                act_wheels = self.driver.read_wheel_speeds()

                # 原始舵机读数
                raw = self.driver.bus.sync_read_speeds(self.driver.wheel_ids)
                raw_speeds = (
                    raw.get(self.driver.wheel_ids[0]),
                    raw.get(self.driver.wheel_ids[1]),
                    raw.get(self.driver.wheel_ids[2]),
                )

                # 实际机器人速度
                act_robot = self.driver.get_actual_velocity()

                self.records.append(
                    WheelSpeedRecord(
                        t=t0,
                        test_name=name,
                        cmd_vx=vx,
                        cmd_vy=vy,
                        cmd_omega=omega,
                        cmd_wheels=cmd_wheels,  # type: ignore[arg-type]
                        act_wheels=act_wheels,  # type: ignore[arg-type]
                        raw_speeds=raw_speeds,  # type: ignore[arg-type]
                        act_robot=act_robot,  # type: ignore[arg-type]
                    )
                )

                elapsed = time.time() - t0
                sleep_time = self.sample_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        self.driver.stop()
        print("[Visualizer] 测试序列完成")

    def _to_array(self) -> Dict[str, List[float]]:
        """将记录转为数组"""
        base_t = self.records[0].t if self.records else 0.0
        return {
            "t": [r.t - base_t for r in self.records],
            "cmd_vx": [r.cmd_vx for r in self.records],
            "cmd_vy": [r.cmd_vy for r in self.records],
            "cmd_omega": [r.cmd_omega for r in self.records],
            "cmd_lf": [r.cmd_lf for r in self.records],
            "cmd_rf": [r.cmd_rf for r in self.records],
            "cmd_re": [r.cmd_re for r in self.records],
            "act_lf": [r.act_lf if r.act_lf is not None else math.nan for r in self.records],
            "act_rf": [r.act_rf if r.act_rf is not None else math.nan for r in self.records],
            "act_re": [r.act_re if r.act_re is not None else math.nan for r in self.records],
            "raw_lf": [r.raw_lf if r.raw_lf is not None else math.nan for r in self.records],
            "raw_rf": [r.raw_rf if r.raw_rf is not None else math.nan for r in self.records],
            "raw_re": [r.raw_re if r.raw_re is not None else math.nan for r in self.records],
            "act_vx": [r.act_vx if r.act_vx is not None else math.nan for r in self.records],
            "act_vy": [r.act_vy if r.act_vy is not None else math.nan for r in self.records],
            "act_omega": [r.act_omega if r.act_omega is not None else math.nan for r in self.records],
        }

    def print_statistics(self) -> None:
        """打印各测试阶段的统计信息"""
        print("\n" + "=" * 70)
        print("测试统计（命令均值 vs 实际均值）")
        print("=" * 70)

        test_names = sorted({r.test_name for r in self.records})
        for name in test_names:
            subset = [r for r in self.records if r.test_name == name]
            if not subset:
                continue

            def avg(vals):
                valid = [v for v in vals if v is not None and not math.isnan(v)]
                return sum(valid) / len(valid) if valid else math.nan

            cmd_lf = avg([r.cmd_lf for r in subset])
            act_lf = avg([r.act_lf for r in subset])
            cmd_rf = avg([r.cmd_rf for r in subset])
            act_rf = avg([r.act_rf for r in subset])
            cmd_re = avg([r.cmd_re for r in subset])
            act_re = avg([r.act_re for r in subset])

            ratio_lf = act_lf / cmd_lf if cmd_lf != 0 and not math.isnan(cmd_lf) else math.nan
            ratio_rf = act_rf / cmd_rf if cmd_rf != 0 and not math.isnan(cmd_rf) else math.nan
            ratio_re = act_re / cmd_re if cmd_re != 0 and not math.isnan(cmd_re) else math.nan

            print(f"\n{name}:")
            print(f"  左前轮  命令={cmd_lf:+.4f}  实际={act_lf:+.4f}  比例={ratio_lf:.3f}")
            print(f"  右前轮  命令={cmd_rf:+.4f}  实际={act_rf:+.4f}  比例={ratio_rf:.3f}")
            print(f"  后轮    命令={cmd_re:+.4f}  实际={act_re:+.4f}  比例={ratio_re:.3f}")

        print("\n" + "=" * 70)
        print("提示：")
        print("  - 若各轮比例接近 1.0：轮速读取正确，距离误差可能来自轮子半径不准")
        print("  - 若某轮比例明显偏离 1.0：该轮速度读取或安装方向可能有问题")
        print("  - 若实际值为 0：该轮速度读取失败（检查舵机ID和连接）")
        print("=" * 70)

    def plot(self, save_path: Optional[str] = None) -> None:
        """绘制轮速对比曲线"""
        if not _MATPLOTLIB_AVAILABLE:
            print("[Visualizer] 未安装 matplotlib，跳过绘图。请执行: pip install matplotlib")
            return

        if not self.records:
            print("[Visualizer] 没有记录数据")
            return

        d = self._to_array()
        t = np.array(d["t"])

        # 为不同测试阶段着色
        test_names = [r.test_name for r in self.records]
        colors = {"前进": "#e8f5e9", "后退": "#ffebee", "左移": "#e3f2fd",
                  "右移": "#fff3e0", "逆时针转": "#f3e5f5", "顺时针转": "#fce4ec",
                  "停止": "#f5f5f5"}

        fig, axes = plt.subplots(3, 2, figsize=(14, 12))
        fig.suptitle("HomeBot 轮速测试曲线", fontsize=14, fontweight="bold")

        def add_bg(ax):
            """为背景添加测试阶段色块"""
            if len(test_names) == 0:
                return
            start_idx = 0
            current_name = test_names[0]
            for i in range(1, len(test_names)):
                if test_names[i] != current_name:
                    ax.axvspan(t[start_idx], t[i - 1], alpha=0.3, color=colors.get(current_name, "gray"))
                    start_idx = i
                    current_name = test_names[i]
            ax.axvspan(t[start_idx], t[-1], alpha=0.3, color=colors.get(current_name, "gray"))

        # 1. 机器人线速度 vx
        ax = axes[0, 0]
        add_bg(ax)
        ax.plot(t, d["cmd_vx"], "b-", linewidth=2, label="命令 vx")
        ax.plot(t, d["act_vx"], "r--", linewidth=1.5, label="实际 vx")
        ax.set_ylabel("vx (m/s)")
        ax.set_title("前进/后退速度")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

        # 2. 机器人线速度 vy
        ax = axes[0, 1]
        add_bg(ax)
        ax.plot(t, d["cmd_vy"], "b-", linewidth=2, label="命令 vy")
        ax.plot(t, d["act_vy"], "r--", linewidth=1.5, label="实际 vy")
        ax.set_ylabel("vy (m/s)")
        ax.set_title("左移/右移速度")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

        # 3. 机器人角速度
        ax = axes[1, 0]
        add_bg(ax)
        ax.plot(t, d["cmd_omega"], "b-", linewidth=2, label="命令 ω")
        ax.plot(t, d["act_omega"], "r--", linewidth=1.5, label="实际 ω")
        ax.set_ylabel("ω (rad/s)")
        ax.set_title("旋转角速度")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

        # 4. 三个轮子的线速度对比
        ax = axes[1, 1]
        add_bg(ax)
        ax.plot(t, d["cmd_lf"], "b-", linewidth=1.5, alpha=0.7, label="cmd 左前")
        ax.plot(t, d["cmd_rf"], "g-", linewidth=1.5, alpha=0.7, label="cmd 右前")
        ax.plot(t, d["cmd_re"], "m-", linewidth=1.5, alpha=0.7, label="cmd 后轮")
        ax.plot(t, d["act_lf"], "b--", linewidth=1.5, label="act 左前")
        ax.plot(t, d["act_rf"], "g--", linewidth=1.5, label="act 右前")
        ax.plot(t, d["act_re"], "m--", linewidth=1.5, label="act 后轮")
        ax.set_ylabel("线速度 (m/s)")
        ax.set_title("各轮子线速度对比")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        # 5. 原始舵机读数
        ax = axes[2, 0]
        add_bg(ax)
        ax.plot(t, d["raw_lf"], "b-", linewidth=1.5, label="左前 raw")
        ax.plot(t, d["raw_rf"], "g-", linewidth=1.5, label="右前 raw")
        ax.plot(t, d["raw_re"], "m-", linewidth=1.5, label="后轮 raw")
        ax.set_ylabel("舵机读数")
        ax.set_xlabel("时间 (s)")
        ax.set_title("原始舵机速度读数")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

        # 6. 命令 vs 实际 散点图（验证线性关系）
        ax = axes[2, 1]
        all_cmd = np.array(d["cmd_lf"] + d["cmd_rf"] + d["cmd_re"])
        all_act = np.array(d["act_lf"] + d["act_rf"] + d["act_re"])
        valid = ~(np.isnan(all_cmd) | np.isnan(all_act))
        ax.scatter(all_cmd[valid], all_act[valid], s=5, alpha=0.5, c="blue")

        # 拟合 y = kx
        if np.any(valid):
            k = np.sum(all_cmd[valid] * all_act[valid]) / np.sum(all_cmd[valid] ** 2)
            x_line = np.linspace(all_cmd[valid].min(), all_cmd[valid].max(), 100)
            ax.plot(x_line, k * x_line, "r-", linewidth=2, label=f"拟合 y={k:.3f}x")
            ax.plot(x_line, x_line, "k--", linewidth=1, alpha=0.5, label="y=x (理想)")
        ax.set_xlabel("命令轮速 (m/s)")
        ax.set_ylabel("实际轮速 (m/s)")
        ax.set_title("命令 vs 实际 线性拟合")
        ax.legend(loc="upper left")
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="box")

        plt.tight_layout(rect=[0, 0, 1, 0.96])

        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"[Visualizer] 图表已保存: {save_path}")
        else:
            plt.show()

    def close(self) -> None:
        self.driver.close()


def main():
    parser = argparse.ArgumentParser(description="HomeBot 轮速可视化测试工具")
    parser.add_argument("--port", default=None, help="舵机串口，如 /dev/ttyUSB0 或 COM3")
    parser.add_argument("--rate", type=float, default=20.0, help="采样频率 Hz (默认 20)")
    parser.add_argument("--save", default=None, help="保存图表路径（如 plot.png）")
    args = parser.parse_args()

    viz = WheelSpeedVisualizer(port=args.port, sample_rate=args.rate)
    if not viz.connect():
        sys.exit(1)

    try:
        viz.run_test_sequence()
        viz.print_statistics()
        viz.plot(save_path=args.save)
    except KeyboardInterrupt:
        print("\n[Visualizer] 被用户中断")
    finally:
        viz.close()


if __name__ == "__main__":
    main()
