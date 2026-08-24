# -*- coding: utf-8 -*-
"""服务注册表：名称 → 启动模块 / 参数 / 端口

供 start/stop/status 命令和 tab 补全共用。
"""
from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class ServiceInfo:
    """单个服务的注册信息"""
    name: str
    module: str                      # python -m 启动的模块
    args: List[str] = field(default_factory=list)
    ports: List[int] = field(default_factory=list)
    desc: str = ""


SERVICES: Dict[str, ServiceInfo] = {
    "bus": ServiceInfo(
        name="bus",
        module="services.message_bus",
        ports=[5590, 5591],
        desc="通用消息总线 (XPUB-XSUB broker)",
    ),
    "motion": ServiceInfo(
        name="motion",
        module="services.motion_service",
        args=["--service", "both"],
        ports=[5556, 5557],
        desc="运动控制服务 (底盘+机械臂)",
    ),
    "vision": ServiceInfo(
        name="vision",
        module="services.vision_service",
        ports=[5560],
        desc="视觉服务 (图像采集发布)",
    ),
    "speech": ServiceInfo(
        name="speech",
        module="services.speech_service",
        args=["wakeup"],
        ports=[5571],
        desc="语音唤醒+ASR (PUB)",
    ),
    "speech_app": ServiceInfo(
        name="speech_app",
        module="applications.speech_interaction",
        ports=[],
        desc="语音交互应用 (SUB，无端口)",
    ),
    "web": ServiceInfo(
        name="web",
        module="applications.remote_control",
        ports=[5000],
        desc="网页遥控端 (Flask)",
    ),
}

# 不带参数时 `homebot start` 启动的核心服务
CORE_SERVICES: List[str] = ["bus", "motion", "vision", "web"]
