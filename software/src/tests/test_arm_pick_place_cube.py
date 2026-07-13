"""
机械臂定点抓取立方体测试脚本

使用 mahjong_bot.motion_planner 中的 plan_pick_and_place_cube 生成完整抓取动作序列，
并通过 ArmServiceClient 控制真实机械臂执行。

运行前请确保：
1. 已启动机械臂服务：
       python -m services.motion_service.arm_service
2. 已安装项目依赖并激活虚拟环境

使用方法：
    cd software/src
    python -m tests.test_arm_pick_place_cube --x 200 --y 0 --z 100

参数说明：
    --x, --y, --z   目标立方体在机械臂坐标系中的位置 (mm)
    --addr          机械臂服务地址，默认 tcp://localhost:5557
    --dry-run       只规划并打印动作序列，不连接机械臂
"""
import sys
import os
import argparse
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from applications.mahjong_bot.motion_planner import MotionPlanner
from applications.mahjong_bot.arm_client import ArmServiceClient
from configs.config import get_config
from common.logging import get_logger

logger = get_logger(__name__)


def parse_args():
    parser = argparse.ArgumentParser(description="机械臂定点抓取立方体测试")
    parser.add_argument("--x", type=float, default=200.0, help="目标 X 坐标 (mm)，默认 200")
    parser.add_argument("--y", type=float, default=0.0, help="目标 Y 坐标 (mm)，默认 0")
    parser.add_argument("--z", type=float, default=100.0, help="目标 Z 坐标 (mm)，默认 100")
    parser.add_argument(
        "--addr",
        type=str,
        default=None,
        help="机械臂服务地址，默认从配置读取 tcp://localhost:5557",
    )
    parser.add_argument(
        "--linear-step",
        type=float,
        default=None,
        help="启用位姿间直线插补并设置最大步长 (mm)，例如 10。不设置则按原始步骤执行",
    )
    parser.add_argument(
        "--interp-duration",
        type=float,
        default=0.0,
        help="插补中间点的统一等待时长 (秒)，0.0 表示等待舵机稳定，大于0表示固定等待",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只进行运动规划并打印步骤，不连接真实机械臂",
    )
    return parser.parse_args()


def print_sequence(sequence):
    """打印运动序列"""
    print(f"\n共规划 {len(sequence)} 个步骤:")
    for i, step in enumerate(sequence):
        print(f"\n步骤 {i + 1}: {step.name}")
        print(f"  描述: {step.description}")
        print(f"  夹爪状态: {'张开' if step.gripper_open else '闭合'}")
        print(f"  预计耗时: {step.duration}s")
        if step.target_pose:
            p = step.target_pose
            print(f"  目标位姿: x={p.x:.1f}, y={p.y:.1f}, z={p.z:.1f}, orientation={p.orientation:.1f}")
        if step.joint_angles:
            angles_str = ", ".join([f"{k}={v:.1f}°" for k, v in step.joint_angles.items()])
            print(f"  关节角度: {angles_str}")


def main():
    args = parse_args()

    print("=" * 60)
    print("机械臂定点抓取立方体测试")
    print("=" * 60)
    print(f"目标位置: x={args.x:.1f} mm, y={args.y:.1f} mm, z={args.z:.1f} mm")
    print(f"运行模式: {'仅规划 (dry-run)' if args.dry_run else '实际控制机械臂'}")
    if args.linear_step is not None:
        print(f"直线插补步长: {args.linear_step:.1f} mm")
        print(f"插补点等待时长: {args.interp_duration:.2f} s")

    # 初始化运动规划器
    planner = MotionPlanner()

    # 检查可达性
    print("\n1. 可达性检查")
    print("-" * 40)
    if not planner.is_position_reachable(args.x, args.y, args.z):
        print(f"[FAIL] 目标位置 ({args.x}, {args.y}, {args.z}) 不可达，请调整目标坐标")
        return 1
    print(f"[OK] 目标位置 ({args.x}, {args.y}, {args.z}) 可达")

    # 规划动作序列
    print("\n2. 运动规划")
    print("-" * 40)
    sequence = planner.plan_pick_and_place_cube(
        args.x,
        args.y,
        args.z,
        linear_step_mm=args.linear_step,
        interp_duration=args.interp_duration,
    )
    if not sequence:
        print("[FAIL] 运动规划失败，未生成任何步骤")
        return 1

    print_sequence(sequence)

    if args.dry_run:
        print("\n[Dry-run] 不连接机械臂，测试结束")
        return 0

    # 确定服务地址
    if args.addr:
        arm_addr = args.addr
    else:
        config = get_config()
        arm_addr = config.zmq.arm_service_addr.replace("*", "localhost")

    print(f"\n3. 连接机械臂服务")
    print("-" * 40)
    print(f"服务地址: {arm_addr}")

    client = ArmServiceClient(arm_addr)
    if not client.connect():
        print("[FAIL] 无法连接到机械臂服务，请确认服务已启动")
        return 1

    print("[OK] 机械臂服务连接成功")

    try:
        print("\n4. 执行抓取动作序列")
        print("-" * 40)

        def on_step_start(step, idx):
            print(f"\n--> 执行 {idx + 1}/{len(sequence)}: {step.name} | {step.description}")

        success = planner.execute_sequence(client, on_step_start=on_step_start)

        if success:
            print("\n[OK] 抓取动作序列执行完成")
        else:
            print("\n[FAIL] 抓取动作序列执行失败")
            return 1

        # 读取最终状态
        print("\n5. 读取最终关节状态")
        print("-" * 40)
        time.sleep(0.5)
        state = client.get_state()
        if state and state.joint_angles:
            print("当前关节角度:")
            for name, angle in state.joint_angles.items():
                print(f"  {name}: {angle:.1f}°")
        else:
            print("[WARN] 未能读取到最终关节状态")

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断测试")
        client.emergency_stop()
        return 130
    finally:
        client.disconnect()

    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
