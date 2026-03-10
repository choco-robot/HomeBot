# -*- coding: utf-8 -*-
"""
测试机械臂休息位置功能
验证服务启动时自动恢复到休息位置
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from configs import get_config
from services.motion_service.servo_bus_manager import ServoBusManager
from services.motion_service.arm_service import create_arm_config_from_global
from hal.arm.driver import ArmDriver


def test_rest_position():
    """测试休息位置配置"""
    print("=" * 60)
    print("测试机械臂休息位置功能")
    print("=" * 60)
    
    config = get_config()
    
    # 显示配置
    print("\n[配置] 休息位置 (rest_position):")
    for joint, angle in config.arm.rest_position.items():
        print(f"  {joint}: {angle}°")
    
    # 转换为 HAL 配置
    hal_config = create_arm_config_from_global()
    print("\n[配置] HAL home_position (来自 rest_position):")
    for joint, angle in hal_config.home_position.items():
        print(f"  {joint}: {angle}°")
    
    # 初始化总线
    print("\n[步骤] 初始化硬件...")
    bus_mgr = ServoBusManager()
    if not bus_mgr.initialize(config.chassis.serial_port, config.chassis.baudrate):
        print("[错误] 总线初始化失败")
        return False
    
    # 创建驱动并初始化（会自动移动到休息位置）
    print("\n[步骤] 初始化机械臂（将自动移动到休息位置）...")
    driver = ArmDriver(hal_config, bus=bus_mgr.get_bus())
    
    if not driver.initialize():
        print("[错误] 机械臂初始化失败")
        bus_mgr.close()
        return False
    
    # 验证位置
    print("\n[验证] 当前关节角度:")
    current = driver.get_all_joint_angles()
    all_ok = True
    for joint, angle in current.items():
        target = hal_config.home_position.get(joint, 0)
        diff = abs(angle - target)
        ok = diff < 5  # 允许5度误差
        status = "✓" if ok else "✗"
        print(f"  {joint}: {angle:>6.1f}° (目标: {target:>6.1f}°, 偏差: {diff:>5.1f}°) {status}")
        if not ok:
            all_ok = False
    
    if all_ok:
        print("\n[成功] 机械臂已成功移动到休息位置！")
    else:
        print("\n[警告] 部分关节位置偏差较大")
    
    # 关闭
    print("\n[步骤] 关闭机械臂（将回到休息位置）...")
    driver.close()
    bus_mgr.close()
    
    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)
    return all_ok


if __name__ == '__main__':
    success = test_rest_position()
    sys.exit(0 if success else 1)
