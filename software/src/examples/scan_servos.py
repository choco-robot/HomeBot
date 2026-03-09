"""
舵机扫描工具 - 查找连接的飞特舵机
"""
import sys
sys.path.insert(0, 'software/src')

from hal.ftservo_driver import FTServoBus


def scan_servos(port: str = "COM15", baudrate: int = 1000000):
    """
    扫描指定串口上的所有舵机
    """
    print(f"扫描串口 {port} @ {baudrate}bps...")
    print("-" * 50)
    
    bus = FTServoBus(port, baudrate)
    if not bus.connect():
        print("无法连接串口")
        return []
    
    found_servos = []
    
    # 扫描 ID 1-20 (通常使用的范围)
    for servo_id in range(1, 21):
        success, model = bus.ping(servo_id)
        if success:
            # 读取当前位置
            pos = bus.read_position(servo_id)
            print(f"  ID={servo_id:2d} | 型号={model:5d} | 位置={pos}")
            found_servos.append((servo_id, model, pos))
    
    bus.disconnect()
    
    print("-" * 50)
    if found_servos:
        print(f"发现 {len(found_servos)} 个舵机:")
        for sid, model, pos in found_servos:
            print(f"  - ID {sid}")
    else:
        print("未发现任何舵机，请检查:")
        print("  1. 舵机是否已上电")
        print("  2. USB转串口模块是否正确连接")
        print("  3. 串口号是否正确")
        print("  4. 波特率是否匹配 (默认 1Mbps)")
    
    return found_servos


def test_wheel_mode(port: str, servo_id: int):
    """
    测试指定舵机的轮式模式
    """
    print(f"\n测试舵机 ID={servo_id} 的轮式模式...")
    
    bus = FTServoBus(port)
    if not bus.connect():
        return False
    
    # 设置轮式模式
    if bus.set_wheel_mode(servo_id):
        print("  轮式模式设置成功")
    else:
        print("  轮式模式设置失败")
        bus.disconnect()
        return False
    
    # 使能扭矩
    bus.torque_enable(servo_id)
    
    # 测试转动
    print("  正向转动...")
    bus.write_speed(servo_id, 500)
    import time
    time.sleep(1)
    
    print("  停止...")
    bus.write_speed(servo_id, 0)
    time.sleep(0.5)
    
    print("  反向转动...")
    bus.write_speed(servo_id, -500)
    time.sleep(1)
    
    print("  停止...")
    bus.write_speed(servo_id, 0)
    
    bus.disconnect()
    print("  测试完成")
    return True


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="扫描飞特舵机")
    parser.add_argument("--port", default="COM15", help="串口号 (默认 COM15)")
    parser.add_argument("--baud", type=int, default=1000000, help="波特率 (默认 1000000)")
    parser.add_argument("--test", type=int, help="测试指定ID的轮式模式")
    
    args = parser.parse_args()
    
    if args.test:
        test_wheel_mode(args.port, args.test)
    else:
        servos = scan_servos(args.port, args.baud)
        
