# -*- coding: utf-8 -*-
"""配置管理 - 集中管理所有硬件和系统配置"""
import os
from typing import Optional
from dataclasses import dataclass, field, asdict


@dataclass
class CameraConfig:
    """摄像头配置"""
    device_id: int = 0
    width: int = 1920     # 摄像头原始分辨率
    height: int = 1080
    fps: int = 30


@dataclass
class ArmConfig:
    """机械臂配置"""
    serial_port: str = "/dev/tty.usbmodem5AE60527771"  # 与底盘共用串口
    baudrate: int = 1000000
    # 舵机ID映射 (1-6号关节)
    base_id: int = 1
    shoulder_id: int = 2
    elbow_id: int = 3
    wrist_flex_id: int = 4
    wrist_roll_id: int = 5
    gripper_id: int = 6
    # 关节角度限制 (度) 人工设置，AI勿动
    joint_limits: dict = field(default_factory=lambda: {
        "base": (-180, 180),
        "shoulder": (0, 180),
        "elbow": (0, 180),
        "wrist_flex": (-90, 90),
        "wrist_roll": (-180, 180),
        "gripper": (0, 90),
    })
    # 默认速度/加速度
    default_speed: int = 1000
    default_acc: int = 50
    # 休息位置/待机位置 (度) - 服务启动时自动恢复到此位置 人工设置，AI勿动
    rest_position: dict = field(default_factory=lambda: {
        "base": -90,         # J1: 基座旋转
        "shoulder": 0,   # J2: 肩关节（自然下垂）
        "elbow": 150,       # J3: 肘关节
        "wrist_flex": 30,   # J4: 腕关节屈伸
        "wrist_roll": -90,   # J5: 腕关节旋转
        "gripper": 45,     # J6: 夹爪（半开）
    })


@dataclass
class ChassisConfig:
    """底盘配置 - 从机器人配置文件读取"""
    # 串口配置（Windows: COM3, Linux: /dev/ttyUSB0）
    serial_port: str = "/dev/tty.usbmodem5AE60527771"
    baudrate: int = 1000000
    
    # 舵机ID映射
    left_front_id: int = 7
    right_front_id: int = 9
    rear_id: int = 8
    
    # 物理参数
    wheel_radius: float = 0.08      # 轮子半径 (m)
    chassis_radius: float = 0.18     # 底盘半径 (m)
    
    # 运动限制
    max_linear_speed: float = 0.5    # 最大线速度 (m/s)
    max_angular_speed: float = 1.0   # 最大角速度 (rad/s)
    default_wheel_speed: int = 3250  # 舵机最大速度
    
    # ZeroMQ地址
    service_addr: str = "tcp://*:5556"


@dataclass
class ZMQConfig:
    """ZeroMQ网络配置"""
    chassis_service_addr: str = "tcp://*:5556"
    arm_service_addr: str = "tcp://*:5557"      # 机械臂服务地址
    vision_pub_addr: str = "tcp://*:5560"
    speech_service_addr: str = "tcp://*:5570"


@dataclass
class LoggingConfig:
    """日志配置"""
    level: str = "INFO"


@dataclass
class HumanFollowConfig:
    """人体跟随配置（YOLO26版）"""
    # 模型配置
    model_path: str = "models/yolo26n.onnx"     # YOLO26 nano (~2.4MB)
    conf_threshold: float = 0.5               # 检测置信度阈值
    
    # 跟踪配置
    max_tracking_age: int = 30                # 最大丢失帧数
    min_iou_threshold: float = 0.3            # IoU匹配阈值
    target_selection: str = "center"          # 目标选择策略: center/largest/closest
    
    # 推理优化（边缘设备）
    inference_size: int = 320                 # 输入分辨率 320x320
    use_half_precision: bool = False          # FP16半精度推理（需GPU支持）
    
    # 跟随控制配置
    target_distance: float = 1.0              # 目标距离（米）
    kp_linear: float = 0.5                    # 线速度P系数（归一化误差后）
    kp_angular: float = 1.0                   # 角速度P系数（归一化误差后）
    max_linear_speed: float = 0.3             # 最大线速度 (m/s)
    max_angular_speed: float = 0.8            # 最大角速度 (rad/s)
    dead_zone_x: int = 150                    # 水平死区（像素），约8%画面宽度
    dead_zone_area: float = 0.1               # 面积死区（相对值）
    
    # 安全配置
    timeout_ms: int = 1000                    # 通信超时
    stop_on_lost: bool = True                 # 丢失目标时是否停止
    search_on_lost: bool = False               # 丢失时是否旋转搜索
    lost_patience: int = 30                   # 丢失容忍帧数（约2秒@30fps）
    
    # ZeroMQ配置
    chassis_service_addr: str = "tcp://localhost:5556"
    vision_sub_addr: str = "tcp://localhost:5560"


@dataclass
class Config:
    """全局配置"""
    camera: CameraConfig = field(default_factory=CameraConfig)
    arm: ArmConfig = field(default_factory=ArmConfig)
    chassis: ChassisConfig = field(default_factory=ChassisConfig)
    zmq: ZMQConfig = field(default_factory=ZMQConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    human_follow: HumanFollowConfig = field(default_factory=HumanFollowConfig)
    
    def to_dict(self) -> dict:
        """转换为字典"""
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: dict) -> "Config":
        """从字典创建配置"""
        return cls(
            camera=CameraConfig(**data.get("camera", {})),
            arm=ArmConfig(**data.get("arm", {})),
            chassis=ChassisConfig(**data.get("chassis", {})),
            zmq=ZMQConfig(**data.get("zmq", {})),
            logging=LoggingConfig(**data.get("logging", {})),
            human_follow=HumanFollowConfig(**data.get("human_follow", {}))
        )


# 全局配置实例
_config_instance: Optional[Config] = None


def get_config() -> Config:
    """获取全局配置实例"""
    global _config_instance
    if _config_instance is None:
        _config_instance = Config()
    return _config_instance


def set_config(config: Config):
    """设置全局配置实例"""
    global _config_instance
    _config_instance = config
