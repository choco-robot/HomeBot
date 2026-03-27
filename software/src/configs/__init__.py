# -*- coding: utf-8 -*-
"""配置管理模块

使用方式:
    from configs import get_config, get_secrets, check_secrets
    
    # 获取配置
    config = get_config()
    print(config.chassis.serial_port)
    
    # 检查密钥配置状态
    check_secrets()
"""
from configs.config import (
    get_config,
    set_config,
    Config,
    CameraConfig,
    ArmConfig,
    ChassisConfig,
    ZMQConfig,
    LoggingConfig,
    SpeechConfig,
    TTSConfig,
    LLMConfig,
    VisionConfig,
    GamepadConfig,
    HumanFollowConfig,
    BatteryConfig,
)

from configs.secrets import (
    get_secrets,
    reload_secrets,
    check_secrets,
    require_secrets,
    Secrets,
    TTSSecrets,
    LLMSecrets,
    VisionSecrets,
)

__all__ = [
    # 配置
    "get_config",
    "set_config",
    "Config",
    "CameraConfig",
    "ArmConfig",
    "ChassisConfig",
    "ZMQConfig",
    "LoggingConfig",
    "SpeechConfig",
    "TTSConfig",
    "LLMConfig",
    "VisionConfig",
    "GamepadConfig",
    "HumanFollowConfig",
    "BatteryConfig",
    # 密钥
    "get_secrets",
    "reload_secrets",
    "check_secrets",
    "require_secrets",
    "Secrets",
    "TTSSecrets",
    "LLMSecrets",
    "VisionSecrets",
]
