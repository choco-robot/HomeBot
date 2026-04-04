"""
发送机械臂关节角度指令测试脚本

用法:
    cd software/src
    python -m tests.test_arm_joints [0,0,0,0,0]
    python -m tests.test_arm_joints --speed 500 [30,45,-30,0,10]

参数:
    joints          5个关节角度，格式如 [0,0,0,0,0]
    --speed         运动速度，默认 800
    --source        控制源，默认 "test"
    --priority      优先级，默认 1
    --addr          服务地址，默认从配置读取

关节映射 (1-5号关节):
    [0] -> base        (基座旋转)
    [1] -> shoulder    (肩关节)
    [2] -> elbow       (肘关节)
    [3] -> wrist_flex  (腕关节屈伸)
    [4] -> wrist_roll  (腕关节旋转)
"""
import sys
import os
import argparse
import json
import ast

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import zmq
from configs.config import get_config


def parse_joint_list(value: str) -> list:
    """解析关节角度列表"""
    try:
        joints = ast.literal_eval(value)
        if not isinstance(joints, list):
            raise ValueError("关节角度必须是列表格式")
        if len(joints) != 5:
            raise ValueError(f"期望5个关节角度，实际得到 {len(joints)} 个")
        return [float(v) for v in joints]
    except Exception as e:
        raise argparse.ArgumentTypeError(f"无法解析关节列表 '{value}': {e}")


def main():
    parser = argparse.ArgumentParser(
        description="发送机械臂关节角度指令到机械臂服务",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n  python -m tests.test_arm_joints [0,0,0,0,0]\n  python -m tests.test_arm_joints --speed 500 [30,45,-30,0,10]"
    )
    parser.add_argument(
        "joints",
        type=parse_joint_list,
        help='5个关节角度列表，如 [0,0,0,0,0]'
    )
    parser.add_argument("--speed", type=int, default=800, help="运动速度 (默认: 800)")
    parser.add_argument("--source", type=str, default="test", help="控制源 (默认: test)")
    parser.add_argument("--priority", type=int, default=1, help="优先级 (默认: 1)")
    parser.add_argument("--addr", type=str, default=None, help="机械臂服务 ZeroMQ 地址")

    args = parser.parse_args()

    # 获取服务地址
    config = get_config()
    arm_addr = (args.addr or config.zmq.arm_service_addr).replace("*", "localhost")

    # 关节映射说明
    joint_names = ["base", "shoulder", "elbow", "wrist_flex", "wrist_roll"]
    print("=" * 60)
    print("机械臂关节角度指令发送测试")
    print("=" * 60)
    print(f"服务地址: {arm_addr}")
    print(f"控制源: {args.source}, 优先级: {args.priority}, 速度: {args.speed}")
    print("关节映射:")
    for i, name in enumerate(joint_names, 1):
        print(f"  J{i}: {name:12s} = {args.joints[i-1]:.1f}°")
    print("-" * 60)

    # 构建指令
    command = {
        "source": args.source,
        "priority": args.priority,
        "speed": args.speed,
        "joints": args.joints  # 列表格式，服务端按索引映射到 1-6 号关节
    }

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.SNDTIMEO, 5000)
    socket.setsockopt(zmq.RCVTIMEO, 5000)
    socket.setsockopt(zmq.LINGER, 0)

    try:
        socket.connect(arm_addr)
        print(f"发送命令: {json.dumps(command, indent=2, ensure_ascii=False)}")
        socket.send_json(command)
        response = socket.recv_json()
        print(f"\n收到响应:\n{json.dumps(response, indent=2, ensure_ascii=False)}")

        if response.get("success"):
            print("\n[OK] 指令已接受")
        else:
            print(f"\n[FAIL] 指令被拒绝: {response.get('message', '未知错误')}")
    except zmq.Again:
        print("\n[FAIL] 请求超时，机械臂服务无响应")
    except Exception as e:
        print(f"\n[FAIL] 请求异常: {e}")
    finally:
        socket.close()
        context.term()


if __name__ == "__main__":
    main()
