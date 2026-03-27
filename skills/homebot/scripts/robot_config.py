"""
机器人连接配置
支持从环境变量读取配置，便于 OpenClaw 等 Agent 使用

优先级: 环境变量 > 本文件默认值

环境变量设置示例:
    Windows:
        set HOMEBOT_IP=192.168.1.13
        set HOMEBOT_CHASSIS_PORT=5556
        set HOMEBOT_ARM_PORT=5557
        set HOMEBOT_VIDEO_PORT=5560
    
    Linux/Mac:
        export HOMEBOT_IP=192.168.1.13
        export HOMEBOT_CHASSIS_PORT=5556
        export HOMEBOT_ARM_PORT=5557
        export HOMEBOT_VIDEO_PORT=5560
"""

import os

<<<<<<< HEAD
# 视频流ZeroMQ PUB端口 - 默认5560
VIDEO_PORT = 5560
=======
# 机器人IP地址 - 可通过 HOMEBOT_IP 环境变量覆盖
ROBOT_IP = os.getenv("HOMEBOT_IP", "192.168.1.13")

# ZeroMQ 端口配置 - 可通过环境变量覆盖
CHASSIS_PORT = int(os.getenv("HOMEBOT_CHASSIS_PORT", "5556"))
ARM_PORT = int(os.getenv("HOMEBOT_ARM_PORT", "5557"))
VIDEO_PORT = int(os.getenv("HOMEBOT_VIDEO_PORT", "5560"))
>>>>>>> f737069 (homebot-skill更新)

# 捕获超时时间（秒）
CAPTURE_TIMEOUT = float(os.getenv("HOMEBOT_CAPTURE_TIMEOUT", "10.0"))

# 图像保存目录（相对脚本目录）
OUTPUT_DIR = os.getenv("HOMEBOT_OUTPUT_DIR", ".")


def get_config():
    """
    获取完整配置字典，便于调试和打印
    
    Returns:
        dict: 包含所有配置项的字典
    """
    return {
        "robot_ip": ROBOT_IP,
        "chassis_port": CHASSIS_PORT,
        "arm_port": ARM_PORT,
        "video_port": VIDEO_PORT,
        "capture_timeout": CAPTURE_TIMEOUT,
        "output_dir": OUTPUT_DIR,
        "chassis_addr": f"tcp://{ROBOT_IP}:{CHASSIS_PORT}",
        "arm_addr": f"tcp://{ROBOT_IP}:{ARM_PORT}",
        "video_addr": f"tcp://{ROBOT_IP}:{VIDEO_PORT}",
    }


def print_config():
    """打印当前配置信息"""
    print("=== HomeBot 当前配置 ===")
    for key, value in get_config().items():
        print(f"  {key}: {value}")
    print("=" * 30)


if __name__ == "__main__":
    print_config()
