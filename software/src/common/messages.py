"""消息类型与序列化工具"""
from enum import Enum
from typing import Any, Dict
import json


class MessageType(str, Enum):
    CMD_VELOCITY = "cmd.velocity"
    CMD_ARM_JOINT = "cmd.arm.joint"
    DETECTION_HUMAN = "detection.human"
    BATTERY_STATE = "sensor.battery"  # 电池状态消息
    # TODO: add more types


def serialize(msg_type: MessageType, data: Dict[str, Any], timestamp: float = None) -> Dict[str, Any]:
    """Return a JSON-ready dictionary payload."""
    payload: Dict[str, Any] = {"type": msg_type.value, "data": data}
    if timestamp is not None:
        payload["timestamp"] = timestamp
    return payload


def deserialize(raw: str) -> Dict[str, Any]:
    return json.loads(raw)
