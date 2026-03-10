"""
motion_service 模块入口
可以同时启动底盘和机械臂服务，支持共享串口总线
"""
import sys
import os
import argparse
import time
import threading

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from configs import get_config
from services.motion_service.servo_bus_manager import ServoBusManager


def main():
    parser = argparse.ArgumentParser(description='HomeBot 运动控制服务')
    parser.add_argument('--service', choices=['chassis', 'arm', 'both'], 
                       default='both', help='启动哪个服务 (默认: both)')
    parser.add_argument('--port', default=None, help='串口设备（覆盖配置）')
    parser.add_argument('--chassis-addr', default=None, help='底盘服务ZeroMQ地址')
    parser.add_argument('--arm-addr', default=None, help='机械臂服务ZeroMQ地址')
    args = parser.parse_args()
    
    print("=" * 60)
    print("HomeBot 运动控制服务启动器")
    print("=" * 60)
    
    # 获取配置
    config = get_config()
    
    # 覆盖串口配置
    if args.port:
        config.chassis.serial_port = args.port
        config.arm.serial_port = args.port  # 确保机械臂使用同一串口
        print(f"[配置] 使用指定串口: {args.port}")
    else:
        print(f"[配置] 底盘串口: {config.chassis.serial_port}")
        print(f"[配置] 机械臂串口: {config.arm.serial_port}")
    
    # 初始化共享串口总线
    bus_manager = ServoBusManager()
    if not bus_manager.initialize(config.chassis.serial_port, config.chassis.baudrate):
        print("[错误] 串口总线初始化失败，退出")
        sys.exit(1)
    
    print("[系统] 共享串口总线已初始化")
    
    # 存储服务和线程
    services = {}
    threads = []
    
    # 启动底盘服务
    if args.service in ('chassis', 'both'):
        from services.motion_service.chassis_service import ChassisService
        
        chassis_addr = args.chassis_addr or config.chassis.service_addr
        chassis_service = ChassisService(rep_addr=chassis_addr, use_shared_bus=True)
        services['chassis'] = chassis_service
        
        chassis_thread = threading.Thread(target=chassis_service.start, daemon=True)
        threads.append(('chassis', chassis_thread))
        chassis_thread.start()
        
        print(f"[服务] 底盘服务已启动 ({chassis_addr})")
    
    # 启动机械臂服务
    if args.service in ('arm', 'both'):
        from services.motion_service.arm_service import ArmService
        
        arm_addr = args.arm_addr or config.zmq.arm_service_addr
        arm_service = ArmService(rep_addr=arm_addr)
        services['arm'] = arm_service
        
        arm_thread = threading.Thread(target=arm_service.start, daemon=True)
        threads.append(('arm', arm_thread))
        arm_thread.start()
        
        print(f"[服务] 机械臂服务已启动 ({arm_addr})")
    
    print("=" * 60)
    print("按 Ctrl+C 停止所有服务")
    print("=" * 60)
    
    # 等待所有线程
    try:
        while True:
            time.sleep(0.1)
            # 检查是否有线程异常退出
            for name, thread in threads:
                if not thread.is_alive():
                    print(f"[警告] {name} 服务线程已退出")
                    # 如果只启动了一个服务，退出主程序
                    if len(threads) == 1:
                        print("[系统] 服务已停止")
                        return
    except KeyboardInterrupt:
        print("\n[系统] 收到停止信号，正在关闭服务...")
    finally:
        # 停止所有服务
        for name, service in services.items():
            print(f"[服务] 正在停止 {name}...")
            try:
                service.stop()
            except Exception as e:
                print(f"[警告] 停止 {name} 时出错: {e}")
        
        # 关闭共享总线
        print("[系统] 正在关闭串口总线...")
        bus_manager.close()
        
        print("[系统] 所有服务已停止")


if __name__ == '__main__':
    main()
