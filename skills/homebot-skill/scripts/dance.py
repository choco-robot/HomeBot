"""
HomeBot机械臂舞蹈脚本
简单的关节舞蹈序列
通过依次移动各个关节创造舞蹈效果
"""

import time
from arm_control import HomeBotArmController

def dance():
    # 连接机械臂
    arm = HomeBotArmController()
    print("=== HomeBot 机械臂舞蹈开始 ===")
    
    # 先回原点
    print("\n[Step 0] 回到起点")
    resp = arm.move_home(speed=800)
    time.sleep(3)
    
    # 舞蹈序列
    dance_steps = [
        # (描述, 关节角度字典, 等待时间秒)
        ("[Step 1] 基座左右摆动", {"base": -60}, 1.5),
        ("[Step 2] 基座向右", {"base": 60}, 1.5),
        ("[Step 3] 基座回中", {"base": 0}, 1.5),
        
        ("[Step 4] 肩膀上下", {"shoulder": 10}, 1.2),
        ("[Step 5] 肩膀下", {"shoulder": 60}, 1.2),
        ("[Step 6] 肩膀回", {"shoulder": 35}, 1.2),
        
        ("[Step 7] 手肘弯曲", {"elbow": 60}, 1.2),
        ("[Step 8] 手肘伸展", {"elbow": 120}, 1.2),
        ("[Step 9] 手肘回中", {"elbow": 90}, 1.2),
        
        ("[Step 10] 腕部摆动", {"wrist_flex": 0}, 1.0),
        ("[Step 11] 腕部上抬", {"wrist_flex": 80}, 1.0),
        ("[Step 12] 腕部回中", {"wrist_flex": 45}, 1.0),
        
        ("[Step 13] 夹爪开合", {"gripper": 0}, 0.8),
        ("[Step 14] 夹爪打开", {"gripper": 90}, 0.8),
        ("[Step 15] 夹爪半开", {"gripper": 45}, 0.8),
        
        ("[Step 16] 同步波浪 - 基座左+肩下", {"base": -45, "shoulder": 60}, 1.8),
        ("[Step 17] 同步波浪 - 基座右+肩上", {"base": 45, "shoulder": 10}, 1.8),
        ("[Step 18] 波浪回中", {"base": 0, "shoulder": 35}, 1.8),
        
        ("[Step 19] 大波浪全体", {"base": -30, "shoulder": 50, "elbow": 110, "wrist_flex": 70}, 2.0),
        ("[Step 20] 反向大波浪", {"base": 30, "shoulder": 20, "elbow": 70, "wrist_flex": 20}, 2.0),
        
        ("[Step 21] 结束姿势", {"base": 0, "shoulder": 30, "elbow": 90, "wrist_flex": 45, "gripper": 90}, 2.5),
    ]
    
    # 执行舞蹈序列
    for i, (desc, angles, wait) in enumerate(dance_steps, 1):
        print(f"\n{i}/{len(dance_steps)} {desc}")
        print(f"   目标角度: {angles}")
        resp = arm.set_joint_angles(angles, speed=600)
        if resp and resp.success:
            print(f"   OK: 指令已接受，等待 {wait} 秒...")
            time.sleep(wait)
        else:
            print("   FAIL: 指令发送失败")
    
    print("\n=== 舞蹈序列完成，回到原点 ===")
    arm.move_home(speed=500)
    time.sleep(3)
    arm.close()
    print("=== 表演完成 ===")

if __name__ == "__main__":
    dance()
