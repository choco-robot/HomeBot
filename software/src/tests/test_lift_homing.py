#!/usr/bin/env python
"""
升降平台零点初始化测试与校准工具

功能:
1. 测试找零功能
2. 校准电流阈值
3. 测量实际行程
4. 验证坐标系

使用方法:
    python -m tests.test_lift_homing --calibrate    # 校准电流阈值
    python -m tests.test_lift_homing --test         # 测试找零功能
    python -m tests.test_lift_homing --read         # 读取当前电流值
"""
import sys
import os
import time
import argparse

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from configs import get_config


def calibrate_current_threshold():
    """
    校准电流阈值
    
    通过监测正常运行和堵转时的电流差异，确定合适的阈值。
    """
    from services.motion_service.servo_bus_manager import ServoBusManager
    
    config = get_config()
    lift_cfg = config.lift_platform
    servo_id = lift_cfg.servo_id_1
    
    print("=" * 60)
    print("电流阈值校准")
    print("=" * 60)
    print(f"舵机ID: {servo_id}")
    print()
    print("校准步骤:")
    print("1. 升降平台将先向下运动一小段")
    print("2. 记录正常运行时的电流")
    print("3. 然后继续运动直到碰到限位")
    print("4. 记录堵转时的电流")
    print("5. 计算合适的阈值")
    print()
    
    # 获取或初始化总线
    bus_manager = ServoBusManager()
    if not bus_manager.is_initialized():
        port = config.arm.serial_port
        baudrate = config.arm.baudrate
        print(f"[INFO] 初始化舵机总线: {port} @ {baudrate}bps")
        if not bus_manager.initialize(port, baudrate):
            print("[ERROR] 舵机总线初始化失败")
            return
    
    bus = bus_manager.get_bus()
    if bus is None or not bus.is_connected():
        print("[ERROR] 舵机总线未连接")
        return
    
    # 读取初始电流
    print("[1] 读取初始状态...")
    initial_currents = []
    for _ in range(10):
        current = bus.read_current(servo_id)
        if current is not None:
            initial_currents.append(current)
        time.sleep(0.1)
    
    if not initial_currents:
        print("[ERROR] 无法读取电流值")
        return
    
    avg_initial = sum(initial_currents) / len(initial_currents)
    print(f"    静态电流: {avg_initial:.0f}mA")
    
    # 向下运动，记录运行电流
    print("[2] 向下运动，记录运行电流...")
    print("    (按 Ctrl+C 停止)")
    
    # 发送向下运动指令
    test_steps = 1000  # 向下运动
    positions = {
        servo_id: (test_steps, 3000, 0),
        lift_cfg.servo_id_2: (test_steps + lift_cfg.servo_offset, 3000, 0)
    }
    bus.sync_write_positions(positions)
    
    running_currents = []
    try:
        while True:
            current = bus.read_current(servo_id)
            if current is not None:
                running_currents.append(current)
                print(f"    电流: {current:.0f}mA    ", end='\r')
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    
    print()
    
    # 停止运动
    stop_positions = {
        servo_id: (0, 0, 0),
        lift_cfg.servo_id_2: (0, 0, 0)
    }
    bus.sync_write_positions(stop_positions)
    
    if running_currents:
        avg_running = sum(running_currents) / len(running_currents)
        max_running = max(running_currents)
        print(f"    平均运行电流: {avg_running:.0f}mA")
        print(f"    最大运行电流: {max_running:.0f}mA")
    
    print()
    print("建议阈值设置:")
    print(f"  保守值: {max_running * 1.5:.0f}mA (运行最大值的1.5倍)")
    print(f"  推荐值: {max_running * 2.0:.0f}mA (运行最大值的2倍)")
    print()
    print(f"当前配置: {lift_cfg.homing_current_threshold}mA")


def test_homing():
    """测试找零功能"""
    print("=" * 60)
    print("升降平台找零测试")
    print("=" * 60)
    
    from services.motion_service.arm_service import ArmService
    from services.motion_service.servo_bus_manager import ServoBusManager
    
    # 创建临时服务实例
    service = ArmService()
    
    # 获取或初始化总线
    bus_manager = ServoBusManager()
    if not bus_manager.is_initialized():
        # 从配置获取串口参数并初始化
        config = get_config()
        port = config.arm.serial_port
        baudrate = config.arm.baudrate
        print(f"[INFO] 初始化舵机总线: {port} @ {baudrate}bps")
        if not bus_manager.initialize(port, baudrate):
            print("[ERROR] 舵机总线初始化失败，请检查串口连接")
            return
    
    service._bus = bus_manager.get_bus()
    
    if service._bus is None or not service._bus.is_connected():
        print("[ERROR] 舵机总线未连接")
        return
    
    # 执行找零
    print("开始找零...")
    success = service._perform_homing()
    
    if success:
        print("[OK] 找零成功")
        print(f"    当前高度: {service._current_lift_height:.1f}mm")
    else:
        print("[FAIL] 找零失败")


def read_current():
    """持续读取电流值"""
    from services.motion_service.servo_bus_manager import ServoBusManager
    
    config = get_config()
    lift_cfg = config.lift_platform
    servo_id = lift_cfg.servo_id_1
    
    print("=" * 60)
    print("实时电流监测")
    print("=" * 60)
    print(f"舵机ID: {servo_id}")
    print("按 Ctrl+C 停止")
    print()
    
    # 获取或初始化总线
    bus_manager = ServoBusManager()
    if not bus_manager.is_initialized():
        port = config.arm.serial_port
        baudrate = config.arm.baudrate
        print(f"[INFO] 初始化舵机总线: {port} @ {baudrate}bps")
        if not bus_manager.initialize(port, baudrate):
            print("[ERROR] 舵机总线初始化失败")
            return
    
    bus = bus_manager.get_bus()
    if bus is None or not bus.is_connected():
        print("[ERROR] 舵机总线未连接")
        return
    
    try:
        while True:
            # 读取电流、负载、位置
            current = bus.read_current(servo_id)
            load = bus.read_load(servo_id)
            pos = bus.read_position(servo_id)
            
            if current is not None and load is not None and pos is not None:
                # 转换为高度
                from services.motion_service.arm_service import ArmService
                service = ArmService()
                height = service._steps_to_height(pos)
                
                print(f"电流: {current:4.0f}mA | 负载: {load:4d} | 位置: {pos:6d} | 高度: {height:6.1f}mm    ", end='\r')
            else:
                print("读取失败...", end='\r')
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n停止监测")


def test_coordinate_system():
    """测试坐标系转换"""
    print("=" * 60)
    print("坐标系转换测试")
    print("=" * 60)
    
    config = get_config()
    lift_cfg = config.lift_platform
    
    print(f"行程长度: {lift_cfg.stroke_length}mm")
    print(f"最高点(坐标0): {lift_cfg.max_height}mm")
    print(f"最低点(坐标-{lift_cfg.stroke_length}): {lift_cfg.min_height}mm")
    print()
    
    from services.motion_service.arm_service import ArmService
    service = ArmService()
    
    print("坐标转换验证:")
    test_heights = [0, -50, -100, -150, -200]
    
    for height in test_heights:
        steps = service._height_to_steps(height)
        height_back = service._steps_to_height(steps)
        print(f"  高度 {height:6.1f}mm -> {steps:8d} 步 -> {height_back:6.1f}mm")
    
    print()
    print("[OK] 如果往返值一致，则坐标转换正确")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='升降平台零点初始化测试与校准')
    parser.add_argument('--calibrate', action='store_true', help='校准电流阈值')
    parser.add_argument('--test', action='store_true', help='测试找零功能')
    parser.add_argument('--read', action='store_true', help='持续读取电流')
    parser.add_argument('--coordinate', action='store_true', help='测试坐标系转换')
    
    args = parser.parse_args()
    
    if args.calibrate:
        calibrate_current_threshold()
    elif args.test:
        test_homing()
    elif args.read:
        read_current()
    elif args.coordinate:
        test_coordinate_system()
    else:
        # 默认执行所有测试
        test_coordinate_system()
        print()
        read_current()
