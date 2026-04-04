"""
麻将机械臂调试工具（合并版）

整合了原 arm_debug_tool.py（ZMQ高层调试）和 debug_arm.py（底层串口控制）的功能。

用法:
    cd software
    python tools/arm_debug_tool.py

命令行快捷模式（底层串口）:
    python tools/arm_debug_tool.py --disable      # 一键失能扭矩
    python tools/arm_debug_tool.py --reset        # 一键复位
    python tools/arm_debug_tool.py --status       # 查看当前状态
    python tools/arm_debug_tool.py --port COM4 --reset

交互模式菜单:
    1-6  - ZMQ高层功能（关节测试、精度、轨迹、序列）
    7-11 - 笛卡尔运动（PTP、直线插补、工作空间）
    D    - 底层串口调试（扭矩/复位/状态）
    H    - 一键复位 Home
    E    - 紧急停止
    0    - 退出
"""

import argparse
import sys
import json
import time
import math
from pathlib import Path
from typing import Dict, List, Optional
from dataclasses import dataclass

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import numpy as np

from applications.mahjong_bot.arm_client import ArmServiceClient, SafeArmController
from applications.mahjong_bot.motion_planner import MotionPlanner
from hal.arm.Kinematics import Arm3DKinematics
from configs.config import get_config
from hal.ftservo_driver import FTServoBus


# ========== 底层串口常量 ==========
ARM_JOINTS = {
    1: ("base", "基座旋转"),
    2: ("shoulder", "肩关节"),
    3: ("elbow", "肘关节"),
    4: ("wrist_flex", "腕关节屈伸"),
    5: ("wrist_roll", "腕关节旋转"),
    6: ("gripper", "夹爪"),
}


def angle_to_position(angle: float) -> int:
    return int(2048 + angle * 11.377)


def position_to_angle(position: int) -> float:
    return (position - 2048) / 11.377


# ========== 数据类 ==========
@dataclass
class CartesianPoint:
    """笛卡尔空间点位"""
    x: float
    y: float
    z: float
    orientation: float = 0.0
    wrist_roll: float = 0.0

    def to_tuple(self) -> tuple:
        return (self.x, self.y, self.z)

    def __str__(self) -> str:
        return f"({self.x:.1f}, {self.y:.1f}, {self.z:.1f}) 姿态={self.orientation:.1f}°"


# ========== 主工具类 ==========
class ArmDebugTool:
    def __init__(self, arm_addr: str = "tcp://localhost:5557", port: str = None):
        self.arm_addr = arm_addr
        self.port = port or get_config().arm.serial_port
        self.baudrate = get_config().arm.baudrate

        # ZMQ 高层连接对象
        self.arm_client: Optional[ArmServiceClient] = None
        self.safe_controller: Optional[SafeArmController] = None

        # 底层串口对象
        self.bus: Optional[FTServoBus] = None

        self.kinematics = Arm3DKinematics()
        self.motion_planner = MotionPlanner()

    # ---------- 连接管理 ----------
    def connect_zmq(self) -> bool:
        self.arm_client = ArmServiceClient(self.arm_addr)
        ok = self.arm_client.connect()
        if ok:
            self.safe_controller = SafeArmController(self.arm_client)
        return ok

    def disconnect_zmq(self):
        if self.arm_client:
            self.arm_client.disconnect()
            self.arm_client = None
            self.safe_controller = None

    def connect_bus(self) -> bool:
        self.bus = FTServoBus(self.port, self.baudrate)
        return self.bus.connect()

    def disconnect_bus(self):
        if self.bus:
            self.bus.disconnect()
            self.bus = None

    # ---------- 底层串口操作 ----------
    def _ensure_bus(self) -> bool:
        if self.bus is None:
            print("\n[底层] 正在连接串口...")
            if not self.connect_bus():
                print("[错误] 串口连接失败")
                return False
            print(f"[OK] 串口已连接 ({self.port})")
            if self.bus._simulation:
                print("[注意] 运行在模拟模式")
        return True

    def disable_all_torque(self) -> bool:
        if not self._ensure_bus():
            return False
        print("\n" + "=" * 50)
        print("一键失能扭矩")
        print("=" * 50)
        try:
            if self.bus.torque_disable(-1):
                print("[成功] 所有舵机扭矩已失能，可手动调整位置")
                return True
            else:
                print("[错误] 失能失败")
                return False
        except Exception as e:
            print(f"[错误] {e}")
            return False

    def enable_all_torque(self) -> bool:
        if not self._ensure_bus():
            return False
        try:
            if self.bus.torque_enable(-1):
                print("[成功] 扭矩已使能")
                return True
            else:
                print("[错误] 使能失败")
                return False
        except Exception as e:
            print(f"[错误] {e}")
            return False

    def reset_all_joints(self, speed: int = 800, acc: int = 50) -> bool:
        if not self._ensure_bus():
            return False
        print("\n" + "=" * 50)
        print("一键复位机械臂")
        print("=" * 50)
        rest_pos = get_config().arm.rest_position

        print("目标位置:")
        for jid, (name, desc) in ARM_JOINTS.items():
            angle = rest_pos[name]
            print(f"  {name} ({desc}): {angle}°")

        try:
            self.bus.torque_enable(-1)
            time.sleep(0.1)
            positions = {}
            for jid, (name, desc) in ARM_JOINTS.items():
                pos = angle_to_position(rest_pos[name])
                positions[jid] = (pos, speed, acc)
            self.bus.sync_write_positions(positions)
            print("[成功] 复位命令已发送")
            time.sleep(2.0)

            print(f"\n{'ID':<4} {'名称':<12} {'描述':<12} {'目标':<8} {'当前':<8} {'状态'}")
            print("-" * 60)
            all_ok = True
            for jid, (name, desc) in ARM_JOINTS.items():
                target = angle_to_position(rest_pos[name])
                cur = self.bus.read_position(jid)
                if cur is not None:
                    diff = abs(cur - target)
                    status = "OK" if diff < 50 else "偏差"
                    if diff >= 50:
                        all_ok = False
                    print(f"{jid:<4} {name:<12} {desc:<12} {target:<8} {cur:<8} [{status}]")
                else:
                    all_ok = False
                    print(f"{jid:<4} {name:<12} {desc:<12} {'N/A':<8} {'N/A':<8} [无法读取]")
            return all_ok
        except Exception as e:
            print(f"[错误] 复位异常: {e}")
            return False

    def show_bus_status(self):
        if not self._ensure_bus():
            return
        print("\n" + "=" * 50)
        print("机械臂当前状态")
        print("=" * 50)
        rest_pos = get_config().arm.rest_position
        print(f"{'ID':<4} {'名称':<12} {'当前位置':<10} {'当前角度':<10} {'复位角度':<10} {'偏差'}")
        print("-" * 65)
        for jid, (name, desc) in ARM_JOINTS.items():
            pos = self.bus.read_position(jid)
            rest_angle = rest_pos[name]
            rest_val = angle_to_position(rest_angle)
            if pos is not None:
                angle = position_to_angle(pos)
                print(f"{jid:<4} {name:<12} {pos:<10} {angle:>6.1f}°   {rest_angle:>6.1f}°   {pos - rest_val:+d}")
            else:
                print(f"{jid:<4} {name:<12} {'N/A':<10} {'N/A':<10} {rest_angle:>6.1f}°   N/A")

    # ---------- ZMQ 高层操作 ----------
    def go_home(self, speed: int = 600) -> bool:
        print(f"\n{'='*50}\n一键复位到 Home 位置\n{'='*50}")
        config = get_config()
        home_pos = getattr(config.arm, "home_position", {
            "base": 0, "shoulder": 0, "elbow": 180,
            "wrist_flex": 0, "wrist_roll": 0, "gripper": 45
        })
        print(f"目标: {home_pos}, 速度: {speed}")
        confirm = input("确认执行? (yes/no): ")
        if confirm.lower() != "yes":
            print("已取消")
            return False
        ok = self.safe_controller.move_joints_safe(home_pos, speed=speed)
        print("✓ 复位完成" if ok else "✗ 复位失败")
        return ok

    def test_joint_movement(self):
        joint = input("关节名称 (base/shoulder/elbow/wrist_flex/wrist_roll/gripper): ").strip()
        angles_str = input("测试角度 (逗号分隔，如 0,30,60): ").strip()
        try:
            angles = [float(a.strip()) for a in angles_str.split(",")]
        except ValueError:
            print("输入无效")
            return
        print(f"\n{'='*50}\n测试关节: {joint}\n{'='*50}")
        for angle in angles:
            print(f"\n移动到 {angle}°...")
            ok = self.safe_controller.move_joints_safe({joint: angle}, speed=500)
            if ok:
                time.sleep(0.8)
                state = self.arm_client.get_state()
                if state:
                    actual = state.joint_angles.get(joint)
                    if actual is not None:
                        print(f"  目标: {angle:.1f}°, 实际: {actual:.1f}°, 误差: {abs(actual-angle):.2f}°")
            else:
                print("  ✗ 失败")

    def test_position_accuracy(self):
        print("输入测试位置 (每行 x,y,z，空行结束):")
        positions = []
        while True:
            line = input().strip()
            if not line:
                break
            try:
                x, y, z = map(float, line.split(","))
                positions.append((x, y, z))
            except ValueError:
                print("格式错误")
        for i, (x, y, z) in enumerate(positions, 1):
            print(f"\n测试点 {i}: ({x}, {y}, {z})")
            joints = self.kinematics.solve_for_position(x, y, z, target_orientation=0.0, elbow_up=True)
            if joints is None:
                print("  ✗ 逆运动学求解失败")
                continue
            joints["wrist_roll"] = joints.get("base", 0)
            print(f"  关节解: { {k:round(v,1) for k,v in joints.items()} }")
            ok = self.safe_controller.move_joints_safe(joints, speed=600)
            print("  ✓ 运动完成" if ok else "  ✗ 运动失败")
            if ok:
                time.sleep(1.0)

    def move_to_cartesian(self, point: CartesianPoint, speed: int = 600, wait: bool = True) -> bool:
        joints = self.kinematics.solve_for_position(
            point.x, point.y, point.z,
            target_orientation=point.orientation,
            target_yaw=point.wrist_roll,
            elbow_up=True
        )
        if joints is None:
            print(f"✗ 逆解失败: {point}")
            return False
        ok = self.safe_controller.move_joints_safe(joints, speed=speed)
        if ok and wait:
            time.sleep(0.5)
        return ok

    def test_ptp_motion(self):
        print("输入起点 (x,y,z[ori]): ")
        start_vals = [float(x) for x in input().strip().split(",")]
        print("输入终点 (x,y,z[ori]): ")
        end_vals = [float(x) for x in input().strip().split(",")]
        speed = int(input("速度 (默认600): ") or "600")
        sp = CartesianPoint(*start_vals)
        ep = CartesianPoint(*end_vals)
        print(f"\nPTP 运动: {sp} -> {ep}")
        if not self.move_to_cartesian(sp, speed=speed):
            return
        time.sleep(1.0)
        t0 = time.time()
        ok = self.move_to_cartesian(ep, speed=speed)
        dt = time.time() - t0
        print(f"✓ PTP完成 (耗时 {dt:.2f}s)" if ok else "✗ PTP失败")

    def test_linear_interpolation(self):
        print("输入起点 (x,y,z[ori]): ")
        start_vals = [float(x) for x in input().strip().split(",")]
        print("输入终点 (x,y,z[ori]): ")
        end_vals = [float(x) for x in input().strip().split(",")]
        speed = int(input("速度 (默认400): ") or "400")
        step = float(input("插补步长mm (默认10): ") or "10")
        sp = CartesianPoint(*start_vals)
        ep = CartesianPoint(*end_vals)
        print(f"\n直线插补: {sp} -> {ep}, 步长={step}mm")
        if not self.move_to_cartesian(sp, speed=speed):
            return
        time.sleep(1.0)
        dx, dy, dz = ep.x - sp.x, ep.y - sp.y, ep.z - sp.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist < 0.1:
            print("距离过近")
            return
        n = max(2, int(dist / step) + 1)
        print(f"总距离 {dist:.1f}mm, 插补点数 {n}")
        t0 = time.time()
        for i in range(1, n):
            t = i / (n - 1)
            cp = CartesianPoint(
                x=sp.x + dx * t,
                y=sp.y + dy * t,
                z=sp.z + dz * t,
                orientation=sp.orientation + (ep.orientation - sp.orientation) * t,
                wrist_roll=sp.wrist_roll + (ep.wrist_roll - sp.wrist_roll) * t,
            )
            joints = self.kinematics.solve_for_position(cp.x, cp.y, cp.z, target_orientation=cp.orientation, elbow_up=True)
            if joints is None:
                print(f"  ✗ 第{i}点逆解失败")
                break
            self.arm_client.move_joints(joints, speed=speed)
            time.sleep(0.05)
        else:
            dt = time.time() - t0
            print(f"✓ 直线插补完成 (耗时 {dt:.2f}s)")

    def test_complete_sequence(self):
        print(f"\n{'='*50}\n测试完整出牌序列\n{'='*50}")
        tx, ty, tz = 250, 0, 100
        print(f"目标: ({tx}, {ty}, {tz})")
        if input("开始? (yes/no): ").lower() != "yes":
            return
        seq = self.motion_planner.plan_pick_and_place(tx, ty, tz)
        print(f"共 {len(seq)} 步:")
        for i, step in enumerate(seq):
            print(f"  {i+1}. {step.name}: {step.description}")
        print("\n按回车执行...")
        input()
        # 简化执行：逐关节发送
        for i, step in enumerate(seq):
            print(f"  [{i+1}/{len(seq)}] {step.name}")
            if step.joint_angles:
                self.arm_client.move_joints(step.joint_angles, speed=600)
                time.sleep(1.5)
        print("✓ 序列执行完成")

    def interactive_cartesian_mode(self):
        print(f"\n{'='*50}\n笛卡尔空间控制\n{'='*50}")
        state = self.arm_client.get_state()
        if state and state.joint_angles:
            j = state.joint_angles
            pos = self.kinematics.forward_kinematics(j.get("base", 0), j.get("shoulder", 0), j.get("elbow", 0))
            cp = CartesianPoint(x=pos[0], y=pos[1], z=pos[2])
            print(f"当前位置: {cp}")
        else:
            cp = CartesianPoint(x=200, y=0, z=100)

        speed = 600
        while True:
            print(f"\n命令: ptp x,y,z | line x,y,z | current | home | back")
            cmd = input("输入: ").strip().lower()
            if cmd == "back":
                break
            elif cmd == "current":
                state = self.arm_client.get_state()
                if state and state.joint_angles:
                    j = state.joint_angles
                    pos = self.kinematics.forward_kinematics(j.get("base", 0), j.get("shoulder", 0), j.get("elbow", 0))
                    print(f"当前: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
            elif cmd == "home":
                self.safe_controller.move_joints_safe(
                    {"base": 0, "shoulder": 45, "elbow": 90, "wrist_flex": 0, "wrist_roll": 0}, speed=600
                )
            elif cmd.startswith("ptp ") or cmd.startswith("line "):
                parts = cmd.split()
                vals = [float(v) for v in parts[1].split(",")]
                target = CartesianPoint(*vals)
                if parts[0] == "ptp":
                    self.move_to_cartesian(target, speed=speed)
                else:
                    # 简化为直接发送目标点
                    self.move_to_cartesian(target, speed=speed)
                cp = target
            else:
                print("未知命令")

    # ---------- 菜单 ----------
    def interactive_low_level_menu(self):
        """底层串口调试子菜单"""
        while True:
            print(f"\n{'='*50}\n底层串口调试 (直接控制舵机)\n{'='*50}")
            print("1. 使能扭矩")
            print("2. 失能扭矩")
            print("3. 一键复位")
            print("4. 查看当前状态")
            print("0. 返回上级")
            c = input("选择: ").strip()
            if c == "1":
                self.enable_all_torque()
            elif c == "2":
                self.disable_all_torque()
            elif c == "3":
                speed = int(input("速度 (默认800): ") or "800")
                self.reset_all_joints(speed=speed)
            elif c == "4":
                self.show_bus_status()
            elif c == "0":
                break

    def interactive_menu(self):
        while True:
            print(f"\n{'='*50}")
            print("麻将机械臂调试工具")
            print(f"{'='*50}")
            print("1. 测试单个关节运动")
            print("2. 测试定位精度")
            print("3. 测试完整出牌序列")
            print("4. PTP 点到点运动")
            print("5. 直线插补运动")
            print("6. 工作空间测试")
            print("7. 交互式笛卡尔控制")
            print("D. 底层串口调试")
            print("H. 一键复位 Home")
            print("E. 紧急停止")
            print("0. 退出")
            c = input("选择: ").strip().lower()

            if c == "1":
                self.test_joint_movement()
            elif c == "2":
                self.test_position_accuracy()
            elif c == "3":
                self.test_complete_sequence()
            elif c == "4":
                self.test_ptp_motion()
            elif c == "5":
                self.test_linear_interpolation()
            elif c == "6":
                ws = self.kinematics.get_workspace()
                print(f"\n工作空间: r={ws['r_min']:.0f}~{ws['r_max']:.0f}mm, z={ws['z_min']:.0f}~{ws['z_max']:.0f}mm")
                r_vals = np.linspace(ws["r_min"] + 10, ws["r_max"] - 10, 5)
                z_vals = np.linspace(max(0, ws["z_min"]), ws["z_max"] - 20, 5)
                ok_cnt = 0
                for r in r_vals:
                    for z in z_vals:
                        reachable = self.kinematics.is_reachable(r, 0, z)
                        ok_cnt += reachable
                        print(f"  ({r:>6.0f}, 0, {z:>6.0f}): {'✓' if reachable else '✗'}")
                print(f"可达: {ok_cnt}/25")
            elif c == "7":
                self.interactive_cartesian_mode()
            elif c == "d":
                self.interactive_low_level_menu()
            elif c == "h":
                speed = int(input("速度 (默认600): ") or "600")
                self.go_home(speed)
            elif c == "e":
                if input("确认紧急停止? (yes/no): ").lower() == "yes":
                    self.arm_client.emergency_stop()
                    print("✓ 已执行")
            elif c == "0":
                break
            else:
                print("无效选择")


def main():
    parser = argparse.ArgumentParser(
        description="麻将机械臂调试工具（高层ZMQ + 底层串口）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
快捷命令（底层串口模式，无需启动 ArmService）:
  python tools/arm_debug_tool.py --disable
  python tools/arm_debug_tool.py --reset
  python tools/arm_debug_tool.py --status
  python tools/arm_debug_tool.py --port COM4 --reset

交互模式（需启动 ArmService）:
  python tools/arm_debug_tool.py --arm tcp://localhost:5557
        """
    )
    parser.add_argument("--arm", default="tcp://localhost:5557", help="机械臂服务地址")
    parser.add_argument("--port", default=None, help="串口设备（底层模式使用）")
    parser.add_argument("--disable", action="store_true", help="一键失能扭矩（底层）")
    parser.add_argument("--reset", action="store_true", help="一键复位（底层）")
    parser.add_argument("--status", action="store_true", help="查看当前状态（底层）")
    parser.add_argument("--speed", type=int, default=800, help="复位速度（底层）")
    parser.add_argument("--acc", type=int, default=50, help="复位加速度（底层）")
    args = parser.parse_args()

    tool = ArmDebugTool(arm_addr=args.arm, port=args.port)

    # 底层快捷模式
    if args.disable or args.reset or args.status:
        try:
            if args.disable:
                tool.disable_all_torque()
            elif args.reset:
                tool.reset_all_joints(speed=args.speed, acc=args.acc)
            elif args.status:
                tool.show_bus_status()
        finally:
            tool.disconnect_bus()
        return

    # 交互模式
    print("=" * 50)
    print("麻将机械臂调试工具")
    print("=" * 50)
    print("\n连接机械臂服务...")
    if not tool.connect_zmq():
        print("连接失败，请检查 ArmService 是否已启动")
        print("提示: 若只需底层调试，请使用 --disable / --reset / --status")
        return

    try:
        tool.interactive_menu()
    finally:
        tool.disconnect_zmq()
        tool.disconnect_bus()
        print("\n已退出")


if __name__ == "__main__":
    main()
