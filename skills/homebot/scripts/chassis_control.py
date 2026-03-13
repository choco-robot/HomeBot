#!/usr/bin/env python3
"""
HomeBot Controller - LAN Robot Chassis Control
Control HomeBot robot chassis movement over local area network.

Author: Picoclaw
Date: 2026-03-10
"""

import zmq
import math
import sys
import time
from typing import Optional

# Configuration - Modify this for your robot
ROBOT_IP = "192.168.1.13"
ROBOT_PORT = 5556
DEFAULT_SPEED = 0.3       # m/s
DEFAULT_ANGULAR_SPEED = 0.5  # rad/s


class HomeBotController:
    """HomeBot机器人控制器"""
    
    def __init__(self, ip: str = ROBOT_IP, port: int = ROBOT_PORT):
        self.ip = ip
        self.port = port
        self.context: Optional[zmq.Context] = None
        self.socket: Optional[zmq.Socket] = None
        self.connected = False
    
    def connect(self) -> bool:
        """连接到机器人"""
        try:
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.REQ)
            self.socket.connect(f"tcp://{self.ip}:{self.port}")
            self.connected = True
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def send_command(self, vx: float, vz: float, priority: int = 2) -> dict:
        """发送速度命令"""
        if not self.connected:
            return {"success": False, "message": "Not connected"}
        
        request = {
            "type": "set_velocity",
            "vx": vx,
            "vz": vz,
            "priority": priority,
            "source": "picoclaw"
        }
        self.socket.send_json(request)
        response = self.socket.recv_json()
        return response
    
    def stop(self) -> dict:
        """停止机器人"""
        return self.send_command(0.0, 0.0)
    
    def forward_cm(self, distance_cm: float, speed: float = DEFAULT_SPEED) -> bool:
        """前进指定距离（厘米）"""
        distance = abs(distance_cm) / 100.0  # convert to meters
        duration = distance / abs(speed) * 2  # 加倍运动时间（实际距离不足）
        
        direction = 1 if distance_cm > 0 else -1
        target_vx = speed * direction
        print(f"Moving {abs(distance_cm)}cm {'forward' if distance_cm > 0 else 'backward'} at {abs(speed) * direction}m/s (duration: {duration:.2f}s - 加倍时间补偿)")
        
        start_time = time.time()
        last_send_time = 0
        
        while time.time() - start_time < duration:
            # Resend command every 200ms to avoid timeout (timeout is 1s)
            current_time = time.time()
            if current_time - last_send_time >= 0.2:
                resp = self.send_command(target_vx, 0.0)
                if not resp.get("success"):
                    print(f"Failed to send: {resp}")
                    self.stop()
                    return False
                last_send_time = current_time
            time.sleep(0.05)  # Small sleep to reduce CPU usage
        
        resp_stop = self.stop()
        print(f"Stopped: {resp_stop}")
        return resp_stop.get("success", False)
    
    def backward_cm(self, distance_cm: float, speed: float = DEFAULT_SPEED) -> bool:
        """后退指定距离（厘米）"""
        return self.forward_cm(-distance_cm, speed)
    
    def left_deg(self, angle_deg: float, angular_speed: float = DEFAULT_ANGULAR_SPEED) -> bool:
        """左转指定角度（度）"""
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad / angular_speed) * 2  # 加倍运动时间（实际角度不足）
        
        target_vz = -angular_speed  # 左转vz为负
        print(f"Turning {abs(angle_deg)} degrees left at {abs(angular_speed)}rad/s (duration: {duration:.2f}s - 加倍时间补偿)")
        
        start_time = time.time()
        last_send_time = 0
        
        while time.time() - start_time < duration:
            # Resend command every 200ms to avoid timeout (timeout is 1s)
            current_time = time.time()
            if current_time - last_send_time >= 0.2:
                resp = self.send_command(0.0, target_vz)
                if not resp.get("success"):
                    print(f"Failed to send: {resp}")
                    self.stop()
                    return False
                last_send_time = current_time
            time.sleep(0.05)  # Small sleep to reduce CPU usage
        
        resp_stop = self.stop()
        print(f"Stopped: {resp_stop}")
        return resp_stop.get("success", False)
    
    def right_deg(self, angle_deg: float, angular_speed: float = DEFAULT_ANGULAR_SPEED) -> bool:
        """右转指定角度（度）"""
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad / angular_speed) * 2  # 加倍运动时间（实际角度不足）
        
        target_vz = angular_speed  # 右转vz为正（用户要求）
        print(f"Turning {abs(angle_deg)} degrees right at {abs(angular_speed)}rad/s (duration: {duration:.2f}s - 加倍时间补偿)")
        
        start_time = time.time()
        last_send_time = 0
        
        while time.time() - start_time < duration:
            # Resend command every 200ms to avoid timeout (timeout is 1s)
            current_time = time.time()
            if current_time - last_send_time >= 0.2:
                resp = self.send_command(0.0, target_vz)
                if not resp.get("success"):
                    print(f"Failed to send: {resp}")
                    self.stop()
                    return False
                last_send_time = current_time
            time.sleep(0.05)  # Small sleep to reduce CPU usage
        
        resp_stop = self.stop()
        print(f"Stopped: {resp_stop}")
        return resp_stop.get("success", False)
    
    def set_velocity(self, vx: float, vz: float) -> dict:
        """设置线速度和角速度"""
        return self.send_command(vx, vz)
    
    def close(self):
        """关闭连接"""
        if self.connected and self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        self.connected = False
    
    def interactive(self):
        """交互式控制"""
        print("\n=== HomeBot Interactive Control ===")
        print("Commands:")
        print("  w - forward 5cm")
        print("  s - backward 5cm")
        print("  a - left 15 degrees")
        print("  d - right 15 degrees")
        print("  space - stop")
        print("  f <cm> - forward N cm (e.g., 'f 10')")
        print("  r <deg> - right N degrees (e.g., 'r 90')")
        print("  l <deg> - left N degrees (e.g., 'l 45')")
        print("  q - quit")
        print("\nConnected to robot. Enter command:")
        
        try:
            while True:
                cmd = input("> ").strip().lower()
                if not cmd:
                    continue
                
                if cmd == 'q':
                    print("Quitting...")
                    break
                elif cmd == 'w':
                    self.forward_cm(5)
                elif cmd == 's':
                    self.backward_cm(5)
                elif cmd == 'a':
                    self.left_deg(15)
                elif cmd == 'd':
                    self.right_deg(15)
                elif cmd == ' ' or cmd == 'space':
                    self.stop()
                    print("Stopped")
                elif cmd.startswith('f '):
                    try:
                        dist = float(cmd.split()[1])
                        self.forward_cm(dist)
                    except:
                        print("Usage: f <distance_cm>")
                elif cmd.startswith('r '):
                    try:
                        angle = float(cmd.split()[1])
                        self.right_deg(angle)
                    except:
                        print("Usage: r <angle_deg>")
                elif cmd.startswith('l '):
                    try:
                        angle = float(cmd.split()[1])
                        self.left_deg(angle)
                    except:
                        print("Usage: l <angle_deg>")
                else:
                    print(f"Unknown command: {cmd}")
        except KeyboardInterrupt:
            print("\nInterrupted")
        finally:
            self.stop()
            self.close()
            print("Disconnected")


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python robot_control.py interactive        - Interactive control")
        print("  python robot_control.py forward <cm>       - Move forward N cm")
        print("  python robot_control.py backward <cm>      - Move backward N cm")
        print("  python robot_control.py left <deg>         - Turn left N degrees")
        print("  python robot_control.py right <deg>        - Turn right N degrees")
        print("  python robot_control.py stop               - Stop robot")
        print("  python robot_control.py velocity <vx> <vz> - Set velocity (m/s, rad/s)")
        sys.exit(1)
    
    command = sys.argv[1]
    bot = HomeBotController()
    
    if not bot.connect():
        print("Failed to connect to robot")
        sys.exit(1)
    
    try:
        if command == "interactive":
            bot.interactive()
        elif command == "forward":
            if len(sys.argv) != 3:
                print("Usage: forward <distance_cm>")
                sys.exit(1)
            dist = float(sys.argv[2])
            success = bot.forward_cm(dist)
            print(f"Done: {'SUCCESS' if success else 'FAILED'}")
        elif command == "backward":
            if len(sys.argv) != 3:
                print("Usage: backward <distance_cm>")
                sys.exit(1)
            dist = float(sys.argv[2])
            success = bot.backward_cm(dist)
            print(f"Done: {'SUCCESS' if success else 'FAILED'}")
        elif command == "left":
            if len(sys.argv) != 3:
                print("Usage: left <angle_deg>")
                sys.exit(1)
            angle = float(sys.argv[2])
            success = bot.left_deg(angle)
            print(f"Done: {'SUCCESS' if success else 'FAILED'}")
        elif command == "right":
            if len(sys.argv) != 3:
                print("Usage: right <angle_deg>")
                sys.exit(1)
            angle = float(sys.argv[2])
            success = bot.right_deg(angle)
            print(f"Done: {'SUCCESS' if success else 'FAILED'}")
        elif command == "stop":
            resp = bot.stop()
            print(f"Response: {resp}")
        elif command == "velocity":
            if len(sys.argv) != 4:
                print("Usage: velocity <vx> <vz>")
                sys.exit(1)
            vx = float(sys.argv[2])
            vz = float(sys.argv[3])
            resp = bot.set_velocity(vx, vz)
            print(f"Response: {resp}")
        else:
            print(f"Unknown command: {command}")
            sys.exit(1)
    finally:
        bot.close()


if __name__ == "__main__":
    main()
