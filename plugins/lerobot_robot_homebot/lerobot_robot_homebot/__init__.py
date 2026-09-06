# -*- coding: utf-8 -*-
"""LeRobot 第三方插件入口：注册 HomeBot 机器人（type="homebot"）

本包被 lerobot 的插件发现机制自动 import（包名前缀 lerobot_robot_，
要求 lerobot>=0.4.0），import 即触发 @RobotConfig.register_subclass("homebot")
完成注册，之后 lerobot.record / lerobot.teleoperate 可直接使用
--robot.type=homebot，无需改动 lerobot 源码。

前置条件：HomeBot 代码可导入（PYTHONPATH 指向 software/src），
详见 requirements-lerobot.txt 与 docs/LeRobot 生态接入.md。
"""
try:
    from applications.imitation_learning.robot import HomeBotRobot, HomeBotRobotConfig
except ImportError as e:
    raise ImportError(
        "lerobot_robot_homebot 需要 HomeBot 代码可导入，请设置 PYTHONPATH：\n"
        "  set PYTHONPATH=<homebot>/software/src    (Windows)\n"
        "  export PYTHONPATH=<homebot>/software/src (Linux/macOS)\n"
        f"原始错误: {e}"
    ) from e

__all__ = ["HomeBotRobot", "HomeBotRobotConfig"]
__version__ = "0.1.0"
