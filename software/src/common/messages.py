# -*- coding: utf-8 -*-
"""消息类型与序列化工具

消息统一采用 {"type", "data", "timestamp"} 信封格式。

内置消息类型使用 MessageType 枚举，命名采用点分层级（cmd.*、sensor.*、detection.*）。
用户自定义消息类型直接使用字符串（如 "user.temperature"、"ext.custom_event"），
无需修改本模块即可扩展。
"""
from enum import Enum
from typing import Any, Dict, Union
import json


class MessageType(str, Enum):
    CMD_VELOCITY = "cmd.velocity"
    CMD_ARM_JOINT = "cmd.arm.joint"
    DETECTION_HUMAN = "detection.human"
    BATTERY_STATE = "sensor.battery"  # 电池状态消息


def resolve_type(msg_type: Union[MessageType, str]) -> str:
    """将消息类型统一解析为字符串

    Args:
        msg_type: MessageType 枚举或自定义类型字符串（如 "user.xxx"）

    Returns:
        消息类型字符串
    """
    if isinstance(msg_type, MessageType):
        return msg_type.value
    return str(msg_type)


def serialize(
    msg_type: Union[MessageType, str],
    data: Dict[str, Any],
    timestamp: float = None,
) -> Dict[str, Any]:
    """Return a JSON-ready dictionary payload.

    Args:
        msg_type: 内置 MessageType 枚举或用户自定义类型字符串（如 "user.temperature"）
        data: 消息数据
        timestamp: 可选时间戳
    """
    payload: Dict[str, Any] = {"type": resolve_type(msg_type), "data": data}
    if timestamp is not None:
        payload["timestamp"] = timestamp
    return payload


def deserialize(raw: str) -> Dict[str, Any]:
    return json.loads(raw)
