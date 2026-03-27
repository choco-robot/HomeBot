"""
HomeBot 机械臂姿态动作脚本
支持挥挥手、点点头、摇摇头三个动作
"""

import time
import sys
import argparse
from arm_control import HomeBotArmController
from robot_config import ROBOT_IP, ARM_PORT


class ArmGestureController:
    """机械臂姿态控制器"""
    
    def __init__(self, robot_ip=None, robot_port=None):
        self.arm = HomeBotArmController(
            robot_ip=robot_ip or ROBOT_IP, 
            robot_port=robot_port or ARM_PORT
        )
    
    def wave(self, times=3):
        """
        挥挥手动作
        通过手腕摆动模拟挥手动作
        """
        print("=== 开始挥挥手 ===")
        
        # 1. 预备姿势 - 抬起手臂
        print("[预备] 抬起手臂...")
        self.arm.set_joint_angles({
            "base": 0,
            "shoulder": 20,
            "elbow": 90,
            "wrist_flex": 45
        }, speed=800)
        time.sleep(2)
        
        # 2. 挥手动作 - 手腕来回摆动
        for i in range(times):
            print(f"[挥手 {i+1}/{times}] 向左...")
            self.arm.set_joint_angle("wrist_flex", 0, speed=1200)
            time.sleep(0.4)
            
            print(f"[挥手 {i+1}/{times}] 向右...")
            self.arm.set_joint_angle("wrist_flex", 90, speed=1200)
            time.sleep(0.4)
        
        # 3. 恢复手腕居中
        self.arm.set_joint_angle("wrist_flex", 45, speed=800)
        time.sleep(0.5)
        
        print("=== 挥挥手完成 ===")
    
    def nod(self, times=3):
        """
        点点头动作
        通过肩膀上下运动模拟点头动作
        """
        print("=== 开始点点头 ===")
        
        # 1. 预备姿势 - 手臂前伸
        print("[预备] 调整姿势...")
        self.arm.set_joint_angles({
            "base": 0,
            "shoulder": 30,
            "elbow": 100,
            "wrist_flex": 30
        }, speed=800)
        time.sleep(2)
        
        # 2. 点头动作 - 肩膀上下运动
        for i in range(times):
            print(f"[点头 {i+1}/{times}] 低下...")
            self.arm.set_joint_angle("shoulder", 60, speed=1000)
            time.sleep(0.5)
            
            print(f"[点头 {i+1}/{times}] 抬起...")
            self.arm.set_joint_angle("shoulder", 20, speed=1000)
            time.sleep(0.5)
        
        print("=== 点点头完成 ===")
    
    def shake_head(self, times=3):
        """
        摇摇头动作
        通过基座左右旋转模拟摇头动作
        """
        print("=== 开始摇摇头 ===")
        
        # 1. 预备姿势
        print("[预备] 调整姿势...")
        self.arm.set_joint_angles({
            "base": 0,
            "shoulder": 35,
            "elbow": 90,
            "wrist_flex": 45
        }, speed=800)
        time.sleep(2)
        
        # 2. 摇头动作 - 基座左右旋转
        for i in range(times):
            print(f"[摇头 {i+1}/{times}] 向左...")
            self.arm.set_joint_angle("base", -45, speed=1200)
            time.sleep(0.5)
            
            print(f"[摇头 {i+1}/{times}] 向右...")
            self.arm.set_joint_angle("base", 45, speed=1200)
            time.sleep(0.5)
        
        # 3. 恢复居中
        self.arm.set_joint_angle("base", 0, speed=800)
        time.sleep(0.5)
        
        print("=== 摇摇头完成 ===")
    
    def close(self):
        """关闭控制器"""
        self.arm.close()


def main():
    parser = argparse.ArgumentParser(
        description="HomeBot 机械臂姿态动作 - 挥挥手、点点头、摇摇头",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python arm_gestures.py wave          # 挥挥手3次
  python arm_gestures.py wave --times 5 # 挥挥手5次
  python arm_gestures.py nod           # 点点头
  python arm_gestures.py shake         # 摇摇头
  python arm_gestures.py all           # 依次执行全部动作
        """
    )
    parser.add_argument("--ip", default=ROBOT_IP, help=f"机器人IP (默认: {ROBOT_IP})")
    parser.add_argument("--port", type=int, default=ARM_PORT, help=f"端口 (默认: {ARM_PORT})")
    parser.add_argument("--times", type=int, default=3, help="重复次数 (默认: 3)")
    parser.add_argument("action", choices=["wave", "nod", "shake", "all"], 
                        help="动作: wave(挥手), nod(点头), shake(摇头), all(全部)")
    
    args = parser.parse_args()
    
    controller = ArmGestureController(args.ip, args.port)
    
    try:
        if args.action == "wave":
            controller.wave(times=args.times)
        elif args.action == "nod":
            controller.nod(times=args.times)
        elif args.action == "shake":
            controller.shake_head(times=args.times)
        elif args.action == "all":
            controller.wave(times=args.times)
            time.sleep(1)
            controller.nod(times=args.times)
            time.sleep(1)
            controller.shake_head(times=args.times)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        controller.close()
        print("控制器已关闭")


if __name__ == "__main__":
    main()
