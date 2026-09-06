# -*- coding: utf-8 -*-
"""模仿学习应用：LeRobot 集成

- robot.py: HomeBotRobot（LeRobot Robot 接口适配器，type="homebot"）
- chassis_adapter.py: 底盘双形态适配（omni3 全向 / diff2 差动 / none）
- joint_map.py: HomeBot ↔ LeRobot 关节命名与单位映射
- policy_runner.py: 策略推理部署（服务级，经仲裁器下发）
- verify_calibration.py: 校准后两套坐标系一致性验证

数据采集用 lerobot 官方 CLI（lerobot.record），详见 AGENTS.md。
"""

# lerobot 第三方设备查找约定（make_device_from_device_class）：在配置类的
# 父包中按"配置类名去掉 Config 后缀"查找设备类，必须在此暴露
from .robot import HomeBotRobot, HomeBotRobotConfig

__all__ = ["HomeBotRobot", "HomeBotRobotConfig"]
