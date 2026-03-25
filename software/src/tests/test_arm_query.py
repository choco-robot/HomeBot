"""
测试机械臂关节状态查询功能

使用方法:
    cd software/src
    python -m tests.test_arm_query

需要先启动机械臂服务（修改后的版本）:
    python -m services.motion_service.arm_service

注意: 如果服务端没有重启，query 功能将无法使用
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import zmq
import json
import time
from configs.config import get_config

def test_arm_query():
    """测试查询机械臂关节状态"""
    print("=" * 60)
    print("测试机械臂关节状态查询")
    print("=" * 60)
    
    config = get_config()
    arm_addr = config.zmq.arm_service_addr.replace("*", "localhost")
    
    print(f"连接地址: {arm_addr}")
    print(f"\n[提示] 如果服务端没有重启，query 功能将不会生效")
    print(f"[提示] 请确保已使用最新代码启动 arm_service")
    print()
    
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.SNDTIMEO, 3000)
    socket.setsockopt(zmq.RCVTIMEO, 3000)
    socket.setsockopt(zmq.LINGER, 0)
    
    try:
        socket.connect(arm_addr)
        print("[OK] 已连接到机械臂服务")
    except Exception as e:
        print(f"[FAIL] 连接失败: {e}")
        return
    
    # 测试 1: 查询状态（带 query 标记）
    print("\n测试 1: 查询关节状态 (query=True)")
    print("-" * 40)
    
    command = {
        "source": "test",
        "priority": 1,
        "speed": 0,
        "joints": {},
        "query": True
    }
    
    print(f"发送命令: {json.dumps(command, indent=2, ensure_ascii=False)}")
    
    try:
        socket.send_json(command)
        response = socket.recv_json()
        print(f"\n收到响应:")
        print(json.dumps(response, indent=2, ensure_ascii=False))
        
        # 诊断
        if response.get("message") == "查询成功":
            print("\n[OK] 服务端正确识别了 query 请求")
        elif response.get("message") == "指令已接受":
            print("\n[WARN] 服务端没有识别 query 请求！")
            print("       可能原因：服务端代码未更新或未重启 arm_service")
        
        joint_states = response.get("joint_states")
        if joint_states and isinstance(joint_states, dict):
            print(f"\n[OK] 成功获取关节状态:")
            for name, angle in joint_states.items():
                print(f"  {name}: {angle:.1f}°")
        else:
            print(f"\n[WARN] 响应中没有 joint_states 字段 (值为: {joint_states})")
            
    except zmq.Again:
        print("[FAIL] 请求超时，机械臂服务无响应")
    except Exception as e:
        print(f"[FAIL] 请求失败: {e}")
    
    # 测试 2: 移动一个关节，再查询验证
    print("\n\n测试 2: 移动基座后再查询")
    print("-" * 40)
    
    socket.close()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.SNDTIMEO, 3000)
    socket.setsockopt(zmq.RCVTIMEO, 3000)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(arm_addr)
    
    # 先移动基座到 30 度
    move_cmd = {
        "source": "test",
        "priority": 1,
        "speed": 800,
        "joints": {"base": 30}
    }
    
    print(f"1. 移动基座到 30 度")
    print(f"   发送: {json.dumps(move_cmd, indent=2, ensure_ascii=False)}")
    
    try:
        socket.send_json(move_cmd)
        response = socket.recv_json()
        print(f"   响应: {response.get('message')}")
    except Exception as e:
        print(f"   [FAIL] {e}")
    
    time.sleep(0.5)  # 等待运动完成
    
    # 查询当前状态
    socket.close()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.SNDTIMEO, 3000)
    socket.setsockopt(zmq.RCVTIMEO, 3000)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(arm_addr)
    
    query_cmd = {
        "source": "test",
        "priority": 1,
        "speed": 0,
        "joints": {},
        "query": True
    }
    
    print(f"\n2. 查询当前状态")
    print(f"   发送: {json.dumps(query_cmd, indent=2, ensure_ascii=False)}")
    
    try:
        socket.send_json(query_cmd)
        response = socket.recv_json()
        print(f"   响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
        
        joint_states = response.get("joint_states")
        if joint_states and isinstance(joint_states, dict):
            base_angle = joint_states.get("base", "未知")
            print(f"\n   [OK] 当前 base 角度: {base_angle}°")
            if base_angle == 30 or (isinstance(base_angle, (int, float)) and abs(base_angle - 30) < 5):
                print("   [OK] 查询结果与预期一致（base ≈ 30°）")
            else:
                print("   [WARN] 查询结果可能与预期不符")
        else:
            print(f"\n   [WARN] 未获取到关节状态")
    except Exception as e:
        print(f"   [FAIL] {e}")
    
    # 测试 3: 复位
    print("\n\n测试 3: 机械臂复位")
    print("-" * 40)
    
    socket.close()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.SNDTIMEO, 3000)
    socket.setsockopt(zmq.RCVTIMEO, 3000)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(arm_addr)
    
    home_cmd = {
        "source": "test",
        "priority": 1,
        "speed": 800,
        "joints": {}  # 空 joints 不带 query = home
    }
    
    print(f"发送: {json.dumps(home_cmd, indent=2, ensure_ascii=False)}")
    print("(这将使机械臂回到 home 位置)")
    
    try:
        socket.send_json(home_cmd)
        response = socket.recv_json()
        print(f"响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
    except Exception as e:
        print(f"[FAIL] {e}")
    
    socket.close()
    context.term()
    
    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)
    print("\n诊断:")
    print("- 如果测试 1 返回 'message': '查询成功'，说明 query 功能正常")
    print("- 如果测试 1 返回 'message': '指令已接受'，说明服务端未更新")
    print("  请重启 arm_service: python -m services.motion_service.arm_service")

if __name__ == "__main__":
    test_arm_query()
