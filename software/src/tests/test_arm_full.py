# -*- coding: utf-8 -*-
"""
完整机械臂服务测试
测试从客户端到舵机的完整链路
"""
import sys
import os
import time
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from configs import get_config
from services.motion_service.servo_bus_manager import ServoBusManager
from services.motion_service.arm_service import ArmService
from services.motion_service.chassis_arbiter import ArmArbiterClient


def test_service_with_real_hardware():
    """测试完整服务链路（连接真实硬件）"""
    print("=" * 60)
    print("完整机械臂服务测试（真实硬件）")
    print("=" * 60)
    
    config = get_config()
    
    # 步骤1: 初始化共享总线
    print("\n[步骤1] 初始化共享总线...")
    bus_manager = ServoBusManager()
    if not bus_manager.initialize(config.chassis.serial_port, config.chassis.baudrate):
        print("[错误] 共享总线初始化失败")
        return False
    
    print(f"[OK] 串口已连接: {config.chassis.serial_port}")
    print(f"  总线实例: {bus_manager.get_bus()}")
    
    # 步骤2: 创建并启动机械臂服务
    print("\n[步骤2] 启动机械臂服务...")
    arm_service = ArmService()
    
    # 在新线程中启动服务
    service_thread = threading.Thread(target=arm_service.start, daemon=True)
    service_thread.start()
    
    # 等待服务启动
    time.sleep(2)
    
    if not service_thread.is_alive():
        print("[错误] 服务线程已退出")
        bus_manager.close()
        return False
    
    print("[OK] 机械臂服务已启动")
    
    # 步骤3: 创建客户端并发送指令
    print("\n[步骤3] 客户端发送指令...")
    client = ArmArbiterClient(service_addr="tcp://127.0.0.1:5557", timeout_ms=3000)
    
    # 测试指令1: 数组格式
    print("\n[测试] 发送6关节角度 [0, 30, 60, 0, 0, 45]...")
    response = client.send_joint_command(
        joints=[0, 30, 60, 0, 0, 45],
        source="test",
        priority=1,
        speed=500
    )
    
    if response:
        print(f"  响应: success={response.success}")
        print(f"  message: {response.message}")
        if response.joint_states:
            print(f"  当前关节状态: {response.joint_states}")
    else:
        print("  [错误] 无响应")
    
    time.sleep(2)  # 等待运动完成
    
    # 测试指令2: 回到初始位置
    print("\n[测试] 发送归零指令 [0, 0, 90, 0, 0, 45]...")
    response = client.send_joint_command(
        joints=[0, 0, 90, 0, 0, 45],
        source="test",
        priority=1,
        speed=500
    )
    
    if response:
        print(f"  响应: success={response.success}")
    else:
        print("  [错误] 无响应")
    
    # 清理
    print("\n[步骤5] 清理...")
    try:
        client.close()
    except Exception as e:
        print(f"  关闭客户端: {e}")
    try:
        arm_service._running = False
    except:
        pass
    try:
        bus_manager.close()
    except Exception as e:
        print(f"  关闭总线: {e}")
    
    print("[完成]")
    return True


def quick_test_direct_call():
    """直接调用 ArmDriver 测试"""
    print("=" * 60)
    print("直接调用 ArmDriver 测试")
    print("=" * 60)
    
    from services.motion_service.arm_service import create_arm_config_from_global
    from hal.arm.driver import ArmDriver
    
    config = get_config()
    
    # 初始化共享总线
    bus_manager = ServoBusManager()
    if not bus_manager.initialize(config.chassis.serial_port, config.chassis.baudrate):
        print("[错误] 总线初始化失败")
        return
    
    # 创建 ArmDriver
    arm_config = create_arm_config_from_global()
    bus = bus_manager.get_bus()
    driver = ArmDriver(arm_config, bus=bus)
    
    print("\n初始化机械臂...")
    if not driver.initialize():
        print("[错误] 初始化失败")
        bus_manager.close()
        return
    
    print("[OK] 初始化成功")
    
    # # 测试运动
    # print("\n测试运动...")
    # print("  设置 base = 30°")
    # driver.set_joint_angle("base", 30, speed=500, wait=True)
    # time.sleep(1)
    
    # print("  设置 shoulder = 45°")
    # driver.set_joint_angle("shoulder", 45, speed=500, wait=True)
    # time.sleep(1)
    
    # print("  设置 elbow = 60°")
    # driver.set_joint_angle("elbow", 60, speed=500, wait=True)
    # time.sleep(1)
    
    # print("\n读取当前位置...")
    # angles = driver.get_all_joint_angles()
    # for name, angle in angles.items():
    #     print(f"  {name}: {angle:.1f}°")
    
    # print("\n回到初始位置...")
    # driver.move_to_home()
    
    driver.close()
    bus_manager.close()
    print("\n[完成]")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['direct', 'service'], default='direct',
                       help='测试模式: direct=直接调用, service=完整服务')
    args = parser.parse_args()
    
    if args.mode == 'direct':
        quick_test_direct_call()
    else:
        test_service_with_real_hardware()
