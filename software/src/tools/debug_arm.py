# -*- coding: utf-8 -*-
"""
机械臂调试工具

使用方法:
    python3 -m tools.debug_arm [选项]

功能:
    - 一键失能: 让所有舵机失去扭矩，可手动调整机械臂位置
    - 一键复位: 让机械臂回到配置的复位位置
    - 查看状态: 查看所有关节当前位置

示例:
    # 交互式菜单
    python3 -m tools.debug_arm
    
    # 一键失能
    python3 -m tools.debug_arm --disable
    
    # 一键复位
    python3 -m tools.debug_arm --reset
    
    # 查看当前状态
    python3 -m tools.debug_arm --status
"""
import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

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


def angle_to_position(angle: float) -> int:
    """将角度转换为舵机位置值 (0-4095)"""
    # 2048 对应 0度，11.377 为角度到位置的转换系数
    return int(2048 + angle * 11.377)


def position_to_angle(position: int) -> float:
    """将舵机位置值转换为角度"""
    return (position - 2048) / 11.377


def disable_all_torque(bus: FTServoBus) -> bool:
    """
    一键失能所有舵机扭矩
    
    失能后可以手动调整机械臂位置
    """
    print("\n" + "=" * 60)
    print("一键失能扭矩")
    print("=" * 60)
    
    try:
        print("\n正在失能所有舵机扭矩...")
        if bus.torque_disable(-1):  # -1 表示广播到所有舵机
            print("[成功] 所有舵机扭矩已失能")
            print("提示: 现在可以手动调整机械臂位置")
            return True
        else:
            print("[错误] 失能扭矩失败")
            return False
    except Exception as e:
        print(f"[错误] 失能扭矩异常: {e}")
        return False


def reset_all_joints(bus: FTServoBus, speed: int = 800, acc: int = 50) -> bool:
    """
    一键复位所有关节到休息位置
    
    Args:
        bus: 舵机总线实例
        speed: 运动速度
        acc: 加速度
    """
    print("\n" + "=" * 60)
    print("一键复位机械臂")
    print("=" * 60)
    
    config = get_config()
    rest_pos = config.arm.rest_position
    
    print("\n目标位置:")
    for joint_id, (name, desc) in ARM_JOINTS.items():
        angle = rest_pos[name]
        pos = angle_to_position(angle)
        print(f"  {name} ({desc}): {angle}° (位置值: {pos})")
    
    print(f"\n运动参数: 速度={speed}, 加速度={acc}")
    print("\n开始复位...")
    
    try:
        # 先使能扭矩
        print("步骤1: 使能扭矩...")
        if not bus.torque_enable(-1):
            print("[警告] 使能扭矩失败，继续尝试...")
        time.sleep(0.1)
        
        # 同步写入所有关节位置
        print("步骤2: 发送复位命令...")
        positions = {}
        for joint_id, (name, desc) in ARM_JOINTS.items():
            angle = rest_pos[name]
            pos = angle_to_position(angle)
            positions[joint_id] = (pos, speed, acc)
        
        if bus.sync_write_positions(positions):
            print("[成功] 复位命令已发送")
        else:
            print("[错误] 复位命令发送失败")
            return False
        
        # 等待运动完成
        print("步骤3: 等待运动完成...")
        time.sleep(2.0)
        
        # 验证位置
        print("步骤4: 验证当前位置...")
        print(f"\n{'ID':<4} {'名称':<12} {'描述':<16} {'目标角度':<10} {'当前位置':<10} {'当前角度':<10} {'状态'}")
        print("-" * 80)
        
        all_ok = True
        for joint_id, (name, desc) in ARM_JOINTS.items():
            target_angle = rest_pos[name]
            target_pos = angle_to_position(target_angle)
            
            current_pos = bus.read_position(joint_id)
            if current_pos is not None:
                current_angle = position_to_angle(current_pos)
                diff = abs(current_pos - target_pos)
                status = "OK" if diff < 50 else "偏差"
                if diff >= 50:
                    all_ok = False
                print(f"{joint_id:<4} {name:<12} {desc:<16} {target_angle:<10} {current_pos:<10} {current_angle:<10.1f} [{status}]")
            else:
                all_ok = False
                print(f"{joint_id:<4} {name:<12} {desc:<16} {target_angle:<10} {'N/A':<10} {'N/A':<10} [无法读取]")
        
        if all_ok:
            print("\n[成功] 机械臂复位完成！")
        else:
            print("\n[警告] 部分关节位置有偏差，请检查")
        
        return all_ok
        
    except Exception as e:
        print(f"[错误] 复位异常: {e}")
        return False


def show_status(bus: FTServoBus):
    """显示所有关节当前状态"""
    print("\n" + "=" * 60)
    print("机械臂当前状态")
    print("=" * 60)
    
    config = get_config()
    rest_pos = config.arm.rest_position
    
    print(f"\n{'ID':<4} {'名称':<12} {'描述':<16} {'当前位置':<10} {'当前角度':<10} {'复位角度':<10} {'偏差'}")
    print("-" * 85)
    
    for joint_id, (name, desc) in ARM_JOINTS.items():
        pos = bus.read_position(joint_id)
        rest_angle = rest_pos[name]
        rest_pos_val = angle_to_position(rest_angle)
        
        if pos is not None:
            angle = position_to_angle(pos)
            diff = pos - rest_pos_val
            diff_str = f"{diff:+d}"
            print(f"{joint_id:<4} {name:<12} {desc:<16} {pos:<10} {angle:>6.1f}°   {rest_angle:>6.1f}°   {diff_str}")
        else:
            print(f"{joint_id:<4} {name:<12} {desc:<16} {'N/A':<10} {'N/A':<10} {rest_angle:>6.1f}°   N/A")


def interactive_menu(bus: FTServoBus):
    """交互式菜单模式"""
    print("\n" + "=" * 60)
    print("机械臂调试工具 - 交互式菜单")
    print("=" * 60)
    
    while True:
        print("\n功能选项:")
        print("  1. 一键失能扭矩 (可手动调整位置)")
        print("  2. 一键复位机械臂")
        print("  3. 查看当前状态")
        print("  4. 使能扭矩")
        print("  q. 退出")
        
        choice = input("\n请选择功能 (1-4, q): ").strip().lower()
        
        if choice == 'q':
            break
        elif choice == '1':
            disable_all_torque(bus)
        elif choice == '2':
            reset_all_joints(bus)
        elif choice == '3':
            show_status(bus)
        elif choice == '4':
            print("\n使能扭矩...")
            if bus.torque_enable(-1):
                print("[成功] 扭矩已使能")
            else:
                print("[错误] 使能失败")
        else:
            print("[错误] 无效选择")


def main():
    parser = argparse.ArgumentParser(
        description='机械臂调试工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 交互式菜单
  python3 -m tools.debug_arm
  
  # 一键失能扭矩
  python3 -m tools.debug_arm --disable
  
  # 一键复位
  python3 -m tools.debug_arm --reset
  
  # 查看状态
  python3 -m tools.debug_arm --status
  
  # 指定串口
  python3 -m tools.debug_arm --port COM3 --reset
        """
    )
    
    parser.add_argument('--port', default=None, 
                       help='串口设备 (默认使用配置文件)')
    parser.add_argument('--disable', action='store_true',
                       help='一键失能所有舵机扭矩')
    parser.add_argument('--reset', action='store_true',
                       help='一键复位机械臂到休息位置')
    parser.add_argument('--status', action='store_true',
                       help='查看当前状态')
    parser.add_argument('--speed', type=int, default=800,
                       help='复位速度 (默认800)')
    parser.add_argument('--acc', type=int, default=50,
                       help='复位加速度 (默认50)')
    
    args = parser.parse_args()
    
    # 获取串口配置
    config = get_config()
    port = args.port or config.arm.serial_port
    baudrate = config.arm.baudrate
    
    print("=" * 60)
    print("机械臂调试工具")
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
        # 根据参数执行功能
        if args.disable:
            disable_all_torque(bus)
        elif args.reset:
            reset_all_joints(bus, args.speed, args.acc)
        elif args.status:
            show_status(bus)
        else:
            # 无参数，进入交互式菜单
            interactive_menu(bus)
            
    except KeyboardInterrupt:
        print("\n\n用户取消")
    finally:
        print("\n关闭串口...")
        bus.disconnect()
        print("[完成]")


if __name__ == '__main__':
    main()
