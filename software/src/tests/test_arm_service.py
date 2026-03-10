# -*- coding: utf-8 -*-
"""
机械臂服务测试脚本
测试机械臂运动控制服务的功能
"""
import sys
import os
import time

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from services.motion_service.chassis_arbiter import ArmArbiterClient, PRIORITIES


def test_arm_client():
    """测试机械臂客户端"""
    print("=" * 60)
    print("机械臂服务客户端测试")
    print("=" * 60)
    
    # 创建客户端连接到机械臂服务
    client = ArmArbiterClient(service_addr="tcp://127.0.0.1:5557", timeout_ms=2000)
    
    print("\n[测试1] 发送6关节角度指令 [0, 45, 90, 0, 0, 30]")
    response = client.send_joint_command(
        joints=[0, 45, 90, 0, 0, 30],  # [base, shoulder, elbow, wrist_flex, wrist_roll, gripper]
        source="test",
        priority=1,
        speed=1000
    )
    
    if response:
        print(f"  响应: success={response.success}, message={response.message}")
        print(f"  当前控制者: {response.current_owner}, 优先级: {response.current_priority}")
        if response.joint_states:
            print(f"  关节状态: {response.joint_states}")
    else:
        print("  错误: 无响应（服务可能未启动）")
    
    print("\n[测试2] 使用关节名称字典发送指令")
    response = client.send_joint_dict(
        joints_dict={
            "base": 10,
            "shoulder": 30,
            "elbow": 60,
            "wrist_flex": 0,
            "wrist_roll": 0,
            "gripper": 45
        },
        source="test",
        priority=1,
        speed=800
    )
    
    if response:
        print(f"  响应: success={response.success}, message={response.message}")
    else:
        print("  错误: 无响应")
    
    print("\n[测试3] 高优先级抢占测试")
    # 先以低优先级发送
    client.send_joint_command([0, 0, 0, 0, 0, 0], source="web", priority=1)
    
    # 再以高优先级发送
    response = client.send_joint_command(
        [20, 20, 20, 0, 0, 0],
        source="auto",
        priority=3,  # 高优先级
        speed=1000
    )
    
    if response:
        print(f"  高优先级响应: {response.message}")
    
    client.close()
    print("\n[完成] 测试结束")


def print_usage():
    """打印使用说明"""
    print("""
使用方法:
1. 首先启动运动控制服务（包含机械臂服务）:
   cd software/src
   python3 -m services.motion_service --service both

   或只启动机械臂服务:
   python3 -m services.motion_service.arm_service

2. 然后运行测试:
   python3 -m tests.test_arm_service

机械臂关节映射:
  J1 (ID=1): base       - 基座旋转 (-180 to 180)
  J2 (ID=2): shoulder   - 肩关节 (-90 to 90)
  J3 (ID=3): elbow      - 肘关节 (-120 to 120)
  J4 (ID=4): wrist_flex - 腕关节屈伸 (-90 to 90)
  J5 (ID=5): wrist_roll - 腕关节旋转 (-180 to 180)
  J6 (ID=6): gripper    - 夹爪 (0 to 90)

客户端使用示例:
  from services.motion_service.chassis_arbiter import ArmArbiterClient
  
  client = ArmArbiterClient("tcp://127.0.0.1:5557")
  
  # 方式1: 数组发送6个关节角度
  client.send_joint_command([0, 45, 90, 0, 0, 30])
  
  # 方式2: 字典发送指定关节
  client.send_joint_dict({"base": 0, "shoulder": 45, "elbow": 90})
  
  client.close()
""")


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--help':
        print_usage()
    else:
        test_arm_client()
