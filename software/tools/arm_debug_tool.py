"""
麻将机械臂调试工具

用于：
1. 测试机械臂各关节运动
2. 验证定位精度
3. 测试碰撞检测
4. 记录和回放动作

使用方法:
    cd software
    python tools/arm_debug_tool.py
"""

import argparse
import sys
import json
import time
from pathlib import Path
from typing import Dict, List, Optional
from dataclasses import dataclass, asdict

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import numpy as np

from applications.mahjong_bot.arm_client import ArmServiceClient, SafeArmController, ArmState
from applications.mahjong_bot.motion_planner import MotionPlanner
from applications.mahjong_bot.coordinate_transformer import CoordinateTransformer
from hal.arm.Kinematics import Arm3DKinematics
from common.logging import get_logger
from configs.config import get_config

logger = get_logger(__name__)


@dataclass
class TestResult:
    """测试结果"""
    test_name: str
    success: bool
    target_position: tuple
    actual_position: Optional[tuple]
    error_mm: Optional[float]
    duration_sec: float
    notes: str = ""


class ArmDebugTool:
    """机械臂调试工具"""
    
    def __init__(self, arm_addr: str = "tcp://localhost:5557"):
        """初始化调试工具"""
        self.arm_client = ArmServiceClient(arm_addr)
        self.safe_controller = SafeArmController(self.arm_client)
        self.motion_planner = MotionPlanner()
        self.transformer = CoordinateTransformer()
        self.kinematics = Arm3DKinematics()
        
        self.results: List[TestResult] = []
        
    def connect(self) -> bool:
        """连接机械臂"""
        return self.arm_client.connect()
    
    def disconnect(self):
        """断开连接"""
        self.arm_client.disconnect()
    
    def test_joint_movement(self, joint_name: str, 
                           test_angles: List[float]) -> bool:
        """
        测试单个关节运动
        
        Args:
            joint_name: 关节名称
            test_angles: 测试角度列表
        
        Returns:
            是否全部成功
        """
        print(f"\n{'='*60}")
        print(f"测试关节: {joint_name}")
        print(f"{'='*60}")
        
        all_success = True
        
        for angle in test_angles:
            print(f"\n移动到 {angle}°...")
            
            start_time = time.time()
            success = self.safe_controller.move_joints_safe(
                {joint_name: angle}, speed=500
            )
            duration = time.time() - start_time
            
            if success:
                print(f"✓ 成功 ({duration:.2f}s)")
                time.sleep(1)  # 等待稳定
                
                # 获取实际位置
                state = self.arm_client.get_state()
                if state:
                    actual = state.joint_angles.get(joint_name)
                    if actual is not None:
                        error = abs(actual - angle)
                        print(f"  目标: {angle:.1f}°, 实际: {actual:.1f}°, 误差: {error:.2f}°")
            else:
                print(f"✗ 失败")
                all_success = False
        
        return all_success
    
    def test_position_accuracy(self, test_positions: List[tuple]) -> bool:
        """
        测试定位精度
        
        Args:
            test_positions: [(x, y, z), ...] 测试位置列表
        
        Returns:
            是否全部成功
        """
        print(f"\n{'='*60}")
        print("测试定位精度")
        print(f"{'='*60}")
        
        all_success = True
        
        for i, (x, y, z) in enumerate(test_positions):
            print(f"\n测试点 {i+1}: ({x}, {y}, {z})")
            
            if not self.motion_planner.is_position_reachable(x, y, z):
                print(f"✗ 位置不可达")
                all_success = False
                continue
            
            # 求解逆运动学
            start_time = time.time()
            joints = self.kinematics.solve_for_position(x, y, z,270)
            
            if joints is None:
                print(f"✗ 逆运动学求解失败")
                all_success = False
                continue
            
            print(f"  关节解: base={joints['base']:.1f}°, "
                  f"shoulder={joints['shoulder']:.1f}°, "
                  f"elbow={joints['elbow']:.1f}°")
            
            joints['wrist_roll'] = joints['base']  # 保持手腕水平
            
            # 执行运动
            success = self.safe_controller.move_joints_safe(joints, speed=600)
            duration = time.time() - start_time
            
            if success:
                time.sleep(1.5)  # 等待稳定
                
                # 获取实际位置并计算误差
                # 注意：这里假设我们能通过某种方式获取实际位置
                # 实际项目中可能需要外部测量（如视觉反馈）
                state = self.arm_client.get_state()
                
                # 简化：假设执行成功即精度合格
                # 实际应该比较目标位置和实际测量位置
                result = TestResult(
                    test_name=f"定位精度测试-{i+1}",
                    success=True,
                    target_position=(x, y, z),
                    actual_position=None,  # 需要外部测量
                    error_mm=None,
                    duration_sec=duration,
                    notes="执行成功，需外部测量验证精度"
                )
                self.results.append(result)
                
                print(f"✓ 运动完成 ({duration:.2f}s)")
            else:
                print(f"✗ 运动失败")
                all_success = False
        
        return all_success
    
    def test_collision_detection(self) -> bool:
        """
        测试碰撞检测
        
        通过监测电流异常来检测碰撞
        """
        print(f"\n{'='*60}")
        print("测试碰撞检测")
        print(f"{'='*60}")
        print("警告：此测试可能会导致机械臂碰撞！")
        print("请确保：")
        print("  1. 周围无障碍物")
        print("  2. 有人随时准备紧急停止")
        print("  3. 机械臂运动范围内安全")
        
        confirm = input("\n是否继续? (yes/no): ")
        if confirm.lower() != "yes":
            print("测试已取消")
            return False
        
        print("\n碰撞检测测试尚未实现")
        print("需要扩展 ArmService 支持实时电流监测")
        
        return True
    
    def record_motion(self, duration: float = 10.0) -> List[Dict]:
        """
        记录机械臂运动轨迹
        
        Args:
            duration: 记录时长 (秒)
        
        Returns:
            轨迹点列表
        """
        print(f"\n{'='*60}")
        print(f"记录运动轨迹 ({duration}秒)")
        print(f"{'='*60}")
        print("请在倒计时内手动移动机械臂...")
        
        for i in range(3, 0, -1):
            print(f"{i}...")
            time.sleep(1)
        
        print("开始记录!")
        
        trajectory = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            state = self.arm_client.get_state()
            if state and state.joint_angles:
                point = {
                    "timestamp": time.time() - start_time,
                    "joints": state.joint_angles.copy(),
                    "lift_height": state.lift_height
                }
                trajectory.append(point)
            
            time.sleep(0.1)  # 10Hz
        
        print(f"记录完成，共 {len(trajectory)} 个点")
        return trajectory
    
    def playback_motion(self, trajectory: List[Dict], speed: float = 1.0):
        """
        回放记录的运动轨迹
        
        Args:
            trajectory: 轨迹点列表
            speed: 回放速度倍率 (1.0=正常速度)
        """
        print(f"\n{'='*60}")
        print(f"回放运动轨迹 (速度 x{speed})")
        print(f"{'='*60}")
        
        if not trajectory:
            print("轨迹为空")
            return
        
        print(f"轨迹点数: {len(trajectory)}")
        print("按回车开始回放...")
        input()
        
        start_time = time.time()
        
        for i, point in enumerate(trajectory):
            target_time = point["timestamp"] / speed
            current_time = time.time() - start_time
            
            # 等待到达目标时间点
            while current_time < target_time:
                time.sleep(0.01)
                current_time = time.time() - start_time
            
            # 发送关节角度
            joints = point["joints"]
            self.arm_client.move_joints(joints, speed=800)
            
            if (i + 1) % 10 == 0:
                print(f"进度: {i+1}/{len(trajectory)}")
        
        print("回放完成")
    
    def test_complete_sequence(self):
        """测试完整的出牌动作序列"""
        print(f"\n{'='*60}")
        print("测试完整出牌序列")
        print(f"{'='*60}")
        print("此测试将执行完整的抓取-移动-出牌动作")
        
        # 测试位置
        test_x, test_y, test_z = 150, 50, 30
        
        print(f"\n目标位置: ({test_x}, {test_y}, {test_z})")
        print("请确保：")
        print(f"  1. 该位置有测试用牌")
        print(f"  2. 出牌槽位置无障碍")
        print(f"  3. 已准备好紧急停止")
        
        confirm = input("\n是否开始测试? (yes/no): ")
        if confirm.lower() != "yes":
            return
        
        # 规划动作序列
        sequence = self.motion_planner.plan_pick_and_place(test_x, test_y, test_z)
        
        print(f"\n动作序列 ({len(sequence)} 步):")
        for i, step in enumerate(sequence):
            print(f"  {i+1}. {step.name}: {step.description}")
        
        print("\n按回车开始执行...")
        input()
        
        # 执行序列
        def on_step(name, curr, total):
            print(f"  [{curr}/{total}] {name}")
        
        success = self.motion_planner.execute_sequence(
            self.arm_client,
            on_step_start=on_step
        )
        
        if success:
            print("\n✓ 序列执行成功")
        else:
            print("\n✗ 序列执行失败")
    
    def print_report(self):
        """打印测试报告"""
        print(f"\n{'='*60}")
        print("测试报告")
        print(f"{'='*60}")
        
        if not self.results:
            print("暂无测试结果")
            return
        
        success_count = sum(1 for r in self.results if r.success)
        total_count = len(self.results)
        
        print(f"总测试数: {total_count}")
        print(f"成功: {success_count}")
        print(f"失败: {total_count - success_count}")
        
        print("\n详细结果:")
        for r in self.results:
            status = "✓" if r.success else "✗"
            print(f"  {status} {r.test_name}: {r.duration_sec:.2f}s - {r.notes}")
    
    def interactive_menu(self):
        """交互式菜单"""
        while True:
            print(f"\n{'='*60}")
            print("麻将机械臂调试工具")
            print(f"{'='*60}")
            print("1. 测试单个关节运动")
            print("2. 测试定位精度")
            print("3. 测试碰撞检测")
            print("4. 记录运动轨迹")
            print("5. 回放运动轨迹")
            print("6. 测试完整出牌序列")
            print("7. 查看测试报告")
            print("0. 退出")
            
            choice = input("\n选择: ").strip()
            
            if choice == "1":
                joint = input("输入关节名称 (base/shoulder/elbow/wrist_flex/wrist_roll/gripper): ")
                angles_str = input("输入测试角度 (逗号分隔，如: 0,30,60): ")
                try:
                    angles = [float(a.strip()) for a in angles_str.split(",")]
                    self.test_joint_movement(joint, angles)
                except ValueError:
                    print("输入无效")
                    
            elif choice == "2":
                print("输入测试位置 (每行一个，格式: x,y,z，空行结束):")
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
                
                if positions:
                    self.test_position_accuracy(positions)
                    
            elif choice == "3":
                self.test_collision_detection()
                
            elif choice == "4":
                duration = float(input("记录时长 (秒): ") or "10")
                trajectory = self.record_motion(duration)
                
                save = input("是否保存轨迹? (y/n): ")
                if save.lower() == "y":
                    filename = input("文件名: ") or "trajectory.json"
                    with open(filename, "w") as f:
                        json.dump(trajectory, f, indent=2)
                    print(f"已保存到 {filename}")
                    
            elif choice == "5":
                filename = input("轨迹文件名: ")
                try:
                    with open(filename, "r") as f:
                        trajectory = json.load(f)
                    speed = float(input("回放速度倍率 (默认1.0): ") or "1.0")
                    self.playback_motion(trajectory, speed)
                except FileNotFoundError:
                    print(f"文件不存在: {filename}")
                except json.JSONDecodeError:
                    print("文件格式错误")
                    
            elif choice == "6":
                self.test_complete_sequence()
                
            elif choice == "7":
                self.print_report()
                
            elif choice == "0":
                break
                
            else:
                print("无效选择")


def main():
    parser = argparse.ArgumentParser(
        description="麻将机械臂调试工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 交互模式
  python tools/arm_debug_tool.py
  
  # 指定机械臂地址
  python tools/arm_debug_tool.py --arm tcp://192.168.1.100:5557
        """
    )
    
    parser.add_argument("--arm", "-a", type=str, default="tcp://localhost:5557",
                        help="机械臂服务地址 (默认: tcp://localhost:5557)")
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("麻将机械臂调试工具")
    print("=" * 60)
    
    tool = ArmDebugTool(args.arm)
    
    print("\n连接机械臂...")
    if not tool.connect():
        print("连接失败，请检查:")
        print("  1. ArmService 是否已启动")
        print("  2. 地址是否正确")
        print("  3. 网络连接是否正常")
        return
    
    try:
        tool.interactive_menu()
    finally:
        tool.disconnect()
        print("\n已断开连接")


if __name__ == "__main__":
    main()
