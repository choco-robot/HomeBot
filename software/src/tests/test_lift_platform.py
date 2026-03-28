#!/usr/bin/env python
"""
升降平台测试脚本
测试机械臂服务中的升降平台功能
"""
import sys
import os
import time

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import zmq
from configs import get_config


def test_lift_platform():
    """测试升降平台功能"""
    config = get_config()
    arm_addr = config.zmq.arm_service_addr.replace("*", "localhost")
    
    print("=" * 60)
    print("升降平台功能测试")
    print("=" * 60)
    print(f"连接地址: {arm_addr}")
    
    # 创建ZMQ上下文和socket
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5秒超时
    socket.connect(arm_addr)
    
    try:
        # 1. 查询当前状态
        print("\n[1] 查询当前状态...")
        socket.send_json({
            "source": "test",
            "priority": 2,
            "query": True
        })
        response = socket.recv_json()
        print(f"响应: {response}")
        
        # 2. 测试下降到低位置（新坐标系：负数表示向下）
        print("\n[2] 升降平台下降到 -100mm...")
        socket.send_json({
            "source": "test",
            "priority": 2,
            "lift": -100  # 单位: mm，新坐标系：0=最高点，负数=向下
        })
        response = socket.recv_json()
        print(f"响应: {response}")
        time.sleep(3)
        
        # 3. 测试下降到更低位置
        print("\n[3] 升降平台下降到 -180mm...")
        socket.send_json({
            "source": "test",
            "priority": 2,
            "lift_height": -180  # 也可以用 lift_height
        })
        response = socket.recv_json()
        print(f"响应: {response}")
        time.sleep(3)
        
        # 4. 测试升降到0mm（最高点）
        print("\n[4] 升降平台回到最高点 0mm...")
        socket.send_json({
            "source": "test",
            "priority": 2,
            "lift": 0
        })
        response = socket.recv_json()
        print(f"响应: {response}")
        time.sleep(3)
        
        # 5. 再次查询状态
        print("\n[5] 再次查询状态...")
        socket.send_json({
            "source": "test",
            "priority": 2,
            "query": True
        })
        response = socket.recv_json()
        print(f"响应: {response}")
        
        print("\n" + "=" * 60)
        print("测试完成!")
        print("=" * 60)
        
    except zmq.Again:
        print("[ERROR] 连接超时，请确保机械臂服务已启动")
        print(f"请运行: python -m services.motion_service.arm_service")
    except Exception as e:
        print(f"[ERROR] 测试失败: {e}")
    finally:
        socket.close()
        context.term()


def test_config():
    """测试配置读取"""
    print("=" * 60)
    print("升降平台配置检查")
    print("=" * 60)
    
    config = get_config()
    lift_cfg = config.lift_platform
    
    print(f"舵机ID1: {lift_cfg.servo_id_1}")
    print(f"舵机ID2: {lift_cfg.servo_id_2}")
    print(f"丝杆导程: {lift_cfg.lead}mm")
    print(f"传动比: {lift_cfg.gear_ratio}")
    print(f"角度分辨率: {lift_cfg.angle_resolution}")
    print(f"行程范围: {lift_cfg.min_height}mm - {lift_cfg.max_height}mm")
    print(f"默认速度: {lift_cfg.default_speed}")
    print(f"默认加速度: {lift_cfg.default_acc}")
    print(f"舵机偏移: {lift_cfg.servo_offset}")
    
    # 验证参数计算
    print("\n参数验证:")
    print("  新坐标系: 0=最高点, -stroke=最低点")
    test_heights = [0, -50, -100, -150, -200]  # mm
    lead = lift_cfg.lead
    ratio = lift_cfg.gear_ratio
    a_res = lift_cfg.angle_resolution
    for h in test_heights:
        steps = int(-h * 4096 / lead / ratio / a_res)
        print(f"  高度 {h:6.1f}mm = {steps:8d} 步")
    
    return True


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='升降平台测试')
    parser.add_argument('--config-only', action='store_true', help='只检查配置')
    
    args = parser.parse_args()
    
    if args.config_only:
        test_config()
    else:
        # 先检查配置
        if test_config():
            print("\n")
            test_lift_platform()
