# -*- coding: utf-8 -*-
"""HomeBot 与 LeRobot 之间的关节命名 / 单位映射

坐标系约定（校准迁移）：
- 机械臂采用 LeRobot 标准校准，校准时将机械臂摆放在 HomeBot 零位姿态，
  使 LeRobot 校准后的 0° 与 HomeBot 的 0°（raw 2048）重合
- 因此两侧关节角度（度）数值一致，只有命名和夹爪单位不同

LeRobot 侧约定：
- 关节名: shoulder_pan / shoulder_lift / elbow_flex / wrist_flex / wrist_roll / gripper
- 观测与动作键: "{joint}.pos"
- 手臂关节单位为度，夹爪为 0-100（0=闭合，100=张开）

HomeBot 侧约定：
- 关节名: base / shoulder / elbow / wrist_flex / wrist_roll / gripper
- 单位全部为度（夹爪 0-90，0=闭合，90=张开）
"""
from typing import Dict

# LeRobot 标准关节顺序（数据集 observation.state / action 的元素顺序）
LEROBOT_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# HomeBot 关节名 -> LeRobot 关节名
HOME2LEROBOT: Dict[str, str] = {
    "base": "shoulder_pan",
    "shoulder": "shoulder_lift",
    "elbow": "elbow_flex",
    "wrist_flex": "wrist_flex",
    "wrist_roll": "wrist_roll",
    "gripper": "gripper",
}

# LeRobot 关节名 -> HomeBot 关节名
LEROBOT2HOME: Dict[str, str] = {v: k for k, v in HOME2LEROBOT.items()}

# HomeBot 夹爪角度范围（度）
GRIPPER_DEG_MAX = 90.0


def gripper_deg_to_lerobot(deg: float) -> float:
    """HomeBot 夹爪角度（0-90 度）→ LeRobot 夹爪值（0-100）"""
    return deg / GRIPPER_DEG_MAX * 100.0


def gripper_lerobot_to_deg(value: float) -> float:
    """LeRobot 夹爪值（0-100）→ HomeBot 夹爪角度（0-90 度）"""
    return value / 100.0 * GRIPPER_DEG_MAX


def home_to_lerobot_state(joint_angles_deg: Dict[str, float]) -> Dict[str, float]:
    """HomeBot 关节角度字典（度）→ LeRobot 观测键值 {"{joint}.pos": value}

    夹爪自动从度转换为 0-100。
    """
    state = {}
    for home_name, deg in joint_angles_deg.items():
        lerobot_name = HOME2LEROBOT.get(home_name, home_name)
        value = gripper_deg_to_lerobot(deg) if lerobot_name == "gripper" else deg
        state[f"{lerobot_name}.pos"] = value
    return state


def lerobot_to_home_action(action: Dict[str, float]) -> Dict[str, float]:
    """LeRobot 动作 {"{joint}.pos": value} → HomeBot 关节角度字典（度）

    夹爪自动从 0-100 转换为度。非 .pos 键（如底盘速度键）被忽略。
    """
    joints = {}
    for key, value in action.items():
        if not key.endswith(".pos"):
            continue
        lerobot_name = key.removesuffix(".pos")
        home_name = LEROBOT2HOME.get(lerobot_name, lerobot_name)
        joints[home_name] = gripper_lerobot_to_deg(value) if lerobot_name == "gripper" else value
    return joints


def clip_action(action: Dict[str, float], limits: Dict[str, tuple]) -> Dict[str, float]:
    """按关节限幅截断 LeRobot 格式的动作 {"{joint}.pos": value}

    Args:
        action: LeRobot 格式动作
        limits: {"{joint}.pos": (min, max)} 限幅表
    """
    clipped = {}
    for key, value in action.items():
        if key in limits:
            lo, hi = limits[key]
            value = max(lo, min(hi, value))
        clipped[key] = value
    return clipped
