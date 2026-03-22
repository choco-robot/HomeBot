# -*- coding: utf-8 -*-
"""
机械臂零点校准工具

使用方法:
    python3 -m tools.calibrate_arm [选项]

功能:
    - 校准指定关节，将当前位置设为2048（中点/零点）
    - 支持单个关节或全部关节校准
    - 支持手动输入目标位置

注意:
    校准前请确保机械臂处于正确的机械零点位置
    校准会立即生效并保存到舵机EEPROM
"""
import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src'))

from configs import get_config
from hal.ftservo_driver import FTServoBus


# 机械臂关节定义
ARM_JOINTS = {
    1: ("base", "基座旋转"),
    2: ("shoulder", "肩关节"),
    3: ("elbow", "肘关节"),
    4: ("wrist_flex", "腕关节屈伸"),
    5: ("wrist_roll", "腕关节旋转"),
    6: ("gripper", "夹爪"),
}

# 飞特舵机寄存器地址
SMS_STS_TORQUE_ENABLE = 40  # 扭矩使能/校准寄存器
SMS_STS_MIN_ANGLE_LIMIT_L = 9   # 最小角度限制低字节
SMS_STS_MIN_ANGLE_LIMIT_H = 10  # 最小角度限制高字节
SMS_STS_MAX_ANGLE_LIMIT_L = 11  # 最大角度限制低字节
SMS_STS_MAX_ANGLE_LIMIT_H = 12  # 最大角度限制高字节
SMS_STS_LOCK = 55  # EEPROM锁定寄存器


def set_angle_limits(bus: FTServoBus, servo_id: int, min_pos: int = 0, max_pos: int = 4096) -> bool:
    """
    设置舵机最大/最小角度限制
    
    步骤:
        1. 解锁EEPROM
        2. 写入最小角度限制 (0)
        3. 写入最大角度限制 (4096)
        4. 锁定EEPROM
    
    Args:
        bus: 舵机总线实例
        servo_id: 舵机ID
        min_pos: 最小位置值 (默认0)
        max_pos: 最大位置值 (默认4096)
        
    Returns:
        是否设置成功
    """
    try:
        print(f"  设置角度限制 [{min_pos}, {max_pos}]...")
        
        if bus.packet_handler and not bus._simulation:
            # 解锁EEPROM
            comm_result, error = bus.packet_handler.write1ByteTxRx(servo_id, SMS_STS_LOCK, 0)
            if comm_result != 0:
                print(f"  [错误] EEPROM解锁失败: {comm_result}")
                return False
            time.sleep(0.05)
            
            # 写入最小角度限制 (2字节，低字节在前)
            min_l = min_pos & 0xFF
            min_h = (min_pos >> 8) & 0xFF
            comm_result, error = bus.packet_handler.write1ByteTxRx(servo_id, SMS_STS_MIN_ANGLE_LIMIT_L, min_l)
            if comm_result != 0:
                print(f"  [错误] 最小角度限制(低字节)写入失败: {comm_result}")
                return False
            comm_result, error = bus.packet_handler.write1ByteTxRx(servo_id, SMS_STS_MIN_ANGLE_LIMIT_H, min_h)
            if comm_result != 0:
                print(f"  [错误] 最小角度限制(高字节)写入失败: {comm_result}")
                return False
            time.sleep(0.05)
            
            # 写入最大角度限制 (2字节，低字节在前)
            max_l = max_pos & 0xFF
            max_h = (max_pos >> 8) & 0xFF
            comm_result, error = bus.packet_handler.write1ByteTxRx(servo_id, SMS_STS_MAX_ANGLE_LIMIT_L, max_l)
            if comm_result != 0:
                print(f"  [错误] 最大角度限制(低字节)写入失败: {comm_result}")
                return False
            comm_result, error = bus.packet_handler.write1ByteTxRx(servo_id, SMS_STS_MAX_ANGLE_LIMIT_H, max_h)
            if comm_result != 0:
                print(f"  [错误] 最大角度限制(高字节)写入失败: {comm_result}")
                return False
            time.sleep(0.05)
            
            # 锁定EEPROM
            comm_result, error = bus.packet_handler.write1ByteTxRx(servo_id, SMS_STS_LOCK, 1)
            if comm_result != 0:
                print(f"  [警告] EEPROM锁定失败: {comm_result}")
            time.sleep(0.05)
            
            print(f"  [成功] 角度限制设置完成 [{min_pos}, {max_pos}]")
            return True
        else:
            print(f"  [模拟模式] 跳过角度限制设置")
            return True
            
    except Exception as e:
        print(f"  [错误] 设置角度限制异常: {e}")
        return False


def calibrate_servo(bus: FTServoBus, servo_id: int, target_pos: int = 2048) -> bool:
    """
    校准单个舵机，将当前位置设为指定值（默认2048中点）
    
    原理:
        向扭矩使能寄存器(40)写入128，当前位置会被设为2048
        这是飞特舵机的零点校准功能
    
    Args:
        bus: 舵机总线实例
        servo_id: 舵机ID
        target_pos: 目标位置值（默认2048，实际由硬件决定）
        
    Returns:
        是否校准成功
    """
    try:
        # 先失能扭矩
        print(f"  步骤1: 失能舵机 {servo_id} 扭矩...")
        bus.torque_disable(servo_id)
        time.sleep(0.1)
        
        # 发送校准命令（向寄存器40写入128）
        print(f"  步骤2: 发送校准命令 (寄存器{SMS_STS_TORQUE_ENABLE} = 128)...")
        if bus.packet_handler and not bus._simulation:
            comm_result, error = bus.packet_handler.write1ByteTxRx(
                servo_id, SMS_STS_TORQUE_ENABLE, 128
            )
            if comm_result != 0:  # COMM_SUCCESS = 0
                print(f"  [错误] 校准命令发送失败: {comm_result}")
                return False
        else:
            print(f"  [模拟模式] 跳过实际校准")
            return True
        
        time.sleep(0.2)
        
        # 重新使能扭矩
        print(f"  步骤3: 重新使能扭矩...")
        bus.torque_enable(servo_id)
        time.sleep(0.1)
        
        # 验证校准结果
        print(f"  步骤4: 验证当前位置...")
        current_pos = bus.read_position(servo_id)
        if current_pos is not None:
            print(f"  当前位置: {current_pos} (目标: 2048)")
            if abs(current_pos - 2048) < 10:
                print(f"  [成功] 舵机 {servo_id} 校准完成！")
                
                # 步骤5: 设置角度限制
                print(f"  步骤5: 设置最大/最小角度限制...")
                if set_angle_limits(bus, servo_id, min_pos=0, max_pos=4096):
                    return True
                else:
                    print(f"  [警告] 角度限制设置失败")
                    return False
            else:
                print(f"  [警告] 位置偏差较大，请检查")
                return False
        else:
            print(f"  [警告] 无法读取位置")
            return False
            
    except Exception as e:
        print(f"  [错误] 校准异常: {e}")
        return False


def interactive_calibration(bus: FTServoBus):
    """交互式校准模式"""
    print("\n" + "=" * 60)
    print("交互式机械臂校准")
    print("=" * 60)
    print("\n关节列表:")
    for sid, (name, desc) in ARM_JOINTS.items():
        print(f"  {sid}: {name} ({desc})")
    print("  0: 校准所有关节")
    print("  q: 退出")
    
    while True:
        print()
        choice = input("请选择要校准的关节 (0-6, q): ").strip().lower()
        
        if choice == 'q':
            break
        
        if choice == '0':
            print("\n[批量校准] 将依次校准所有关节...")
            print("请确保机械臂处于正确的零点位置！")
            confirm = input("确认继续? (y/n): ").strip().lower()
            if confirm != 'y':
                continue
            
            success_count = 0
            for sid, (name, desc) in ARM_JOINTS.items():
                print(f"\n[{sid}/6] 校准 {name} ({desc})...")
                if calibrate_servo(bus, sid):
                    success_count += 1
                time.sleep(0.5)
            
            print(f"\n[完成] {success_count}/6 个关节校准成功")
            
        elif choice in ['1', '2', '3', '4', '5', '6']:
            sid = int(choice)
            name, desc = ARM_JOINTS[sid]
            print(f"\n校准关节: {name} ({desc})")
            print("请手动将该关节移动到机械零点位置")
            input("准备好后按回车继续...")
            
            calibrate_servo(bus, sid)
            
        else:
            print("[错误] 无效选择")


def quick_calibration(bus: FTServoBus, joint_ids: list = None):
    """快速校准指定关节"""
    if joint_ids is None:
        joint_ids = list(ARM_JOINTS.keys())
    
    print("\n" + "=" * 60)
    print("快速校准模式")
    print("=" * 60)
    print(f"\n将校准以下关节: {joint_ids}")
    print("警告: 这会立即将当前位置设为2048（中点）！")
    
    success_count = 0
    for sid in joint_ids:
        if sid in ARM_JOINTS:
            name, desc = ARM_JOINTS[sid]
            print(f"\n[校准] ID {sid} - {name} ({desc})...")
            if calibrate_servo(bus, sid):
                success_count += 1
            time.sleep(0.3)
    
    print(f"\n[完成] {success_count}/{len(joint_ids)} 个关节校准成功")
    return success_count == len(joint_ids)


def verify_positions(bus: FTServoBus):
    """验证所有关节当前位置"""
    print("\n" + "=" * 60)
    print("当前关节位置")
    print("=" * 60)
    
    print(f"\n{'ID':<4} {'名称':<12} {'描述':<16} {'位置':<8} {'角度(约)'}")
    print("-" * 60)
    
    for sid, (name, desc) in ARM_JOINTS.items():
        pos = bus.read_position(sid)
        if pos is not None:
            # 粗略角度转换 (0-4095 -> -180~180度)
            angle = (pos - 2048) / 11.377
            status = "OK" if abs(pos - 2048) < 100 else "偏离"
            print(f"{sid:<4} {name:<12} {desc:<16} {pos:<8} {angle:>6.1f}° [{status}]")
        else:
            print(f"{sid:<4} {name:<12} {desc:<16} {'N/A':<8} {'N/A':>6}")


def main():
    parser = argparse.ArgumentParser(
        description='机械臂零点校准工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 交互式校准
  python3 -m tools.calibrate_arm
  
  # 快速校准所有关节
  python3 -m tools.calibrate_arm --quick
  
  # 只校准特定关节（如基座和肩关节）
  python3 -m tools.calibrate_arm --quick --joints 1 2
  
  # 只查看当前位置
  python3 -m tools.calibrate_arm --verify
        """
    )
    
    parser.add_argument('--port', default=None, 
                       help='串口设备 (默认使用配置文件)')
    parser.add_argument('--quick', action='store_true',
                       help='快速模式：直接校准，不询问')
    parser.add_argument('--joints', nargs='+', type=int,
                       help='指定要校准的关节ID列表 (如: 1 2 3)')
    parser.add_argument('--verify', action='store_true',
                       help='只查看当前位置，不校准')
    
    args = parser.parse_args()
    
    # 获取串口配置
    config = get_config()
    port = args.port or config.arm.serial_port
    baudrate = config.arm.baudrate
    
    print("=" * 60)
    print("机械臂零点校准工具")
    print("=" * 60)
    print(f"\n串口: {port}")
    print(f"波特率: {baudrate}")
    
    # 连接串口
    print("\n连接串口...")
    bus = FTServoBus(port, baudrate)
    if not bus.connect():
        print("[错误] 无法连接串口，请检查:")
        print(f"  1. 串口 {port} 是否正确")
        print(f"  2. 串口是否被其他程序占用")
        print(f"  3. 波特率是否正确 ({baudrate})")
        sys.exit(1)
    
    print("[OK] 串口连接成功")
    
    if bus._simulation:
        print("\n[注意] 运行中模拟模式，不会实际控制舵机")
    
    try:
        if args.verify:
            # 只查看位置
            verify_positions(bus)
        elif args.quick:
            # 快速校准
            joint_ids = args.joints if args.joints else list(ARM_JOINTS.keys())
            quick_calibration(bus, joint_ids)
            print()
            verify_positions(bus)
        else:
            # 交互式校准
            interactive_calibration(bus)
            print()
            verify_positions(bus)
            
    except KeyboardInterrupt:
        print("\n\n用户取消")
    finally:
        print("\n关闭串口...")
        bus.disconnect()
        print("[完成]")


if __name__ == '__main__':
    main()
