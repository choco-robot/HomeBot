"""
电池状态订阅测试程序
用于验证底盘服务发布的电池状态

运行方式:
    cd software/src
    python -m tests.test_battery_subscriber
"""
import sys
import time
import zmq
from configs import get_config


def main():
    """订阅并显示电池状态"""
    config = get_config()
    battery_addr = config.battery.pub_addr.replace("*", "localhost")
    
    print("=" * 60)
    print("电池状态订阅测试")
    print("=" * 60)
    print(f"订阅地址: {battery_addr}")
    print("按 Ctrl+C 退出")
    print("=" * 60)
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(battery_addr)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    
    try:
        while True:
            try:
                message = socket.recv_json(flags=zmq.NOBLOCK)
                msg_type = message.get("type", "unknown")
                data = message.get("data", {})
                timestamp = message.get("timestamp", 0)
                
                if msg_type == "sensor.battery":
                    voltage = data.get("voltage", 0)
                    percentage = data.get("percentage", 0)
                    status = data.get("status", "unknown")
                    temp = data.get("temperature", None)
                    servo_id = data.get("servo_id", 0)
                    
                    temp_str = f"{temp}°C" if temp is not None else "N/A"
                    
                    print(f"\r[{time.strftime('%H:%M:%S')}] "
                          f"电压: {voltage:.2f}V | "
                          f"电量: {percentage:.1f}% | "
                          f"状态: {status:10} | "
                          f"温度: {temp_str:6} | "
                          f"舵机ID: {servo_id}", end="", flush=True)
                    
            except zmq.Again:
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("\n\n正在退出...")
    finally:
        socket.close()
        context.term()


if __name__ == "__main__":
    main()
