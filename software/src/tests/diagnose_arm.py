# -*- coding: utf-8 -*-
"""
机械臂诊断脚本
用于排查舵机不运动的问题
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from configs import get_config
from services.motion_service.servo_bus_manager import ServoBusManager
from hal.ftservo_driver import FTServoBus
from hal.arm.driver import ArmDriver, ArmConfig as HalArmConfig


def test_raw_servo_control():
    """测试原始舵机控制"""
    print("=" * 60)
    print("测试1: 原始舵机控制")
    print("=" * 60)
    
    config = get_config()
    bus = FTServoBus(config.arm.serial_port, config.arm.baudrate)
    
    if not bus.connect():
        print("[错误] 无法连接串口")
        return False
    
    print("[OK] 串口已连接: " + config.arm.serial_port)
    print("  模拟模式: " + str(bus._simulation))
    
    # 测试 ping 每个机械臂舵机
    arm_ids = [1, 2, 3, 4, 5, 6]
    print("\n[测试] Ping 机械臂舵机:")
    for sid in arm_ids:
        found, model = bus.ping(sid)
        status = "OK" if found else "FAIL"
        print(f"  ID {sid}: {status} (model: {model})")
    
    # 测试使能扭矩
    print("\n[测试] 广播使能扭矩...")
    bus.torque_enable()
    time.sleep(0.1)
    
    # 测试写入位置
    test_positions = [2048, 2100, 2000, 2048, 2048, 2048]
    print("\n[测试] 写入测试位置:")
    for sid, pos in zip(arm_ids, test_positions):
        success = bus.write_position(sid, pos, speed=500, acc=50)
        status = "OK" if success else "FAIL"
        print(f"  ID {sid} -> {pos}: {status}")
        time.sleep(0.05)
    
    # 读取当前位置
    print("\n[测试] 读取当前位置:")
    time.sleep(0.5)
    for sid in arm_ids:
        pos = bus.read_position(sid)
        print(f"  ID {sid}: {pos}")
    
    bus.disconnect()
    print("\n[完成]")
    return True


def main():
    print("\n" + "=" * 60)
    print("机械臂诊断工具")
    print("=" * 60 + "\n")
    
    test_raw_servo_control()


if __name__ == '__main__':
    main()
