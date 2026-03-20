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
    serial_port: str = "COM15"  # 与底盘共用串口
    baudrate: int = 1000000
    # 舵机ID映射 (1-6号关节)
    base_id: int = 1
    shoulder_id: int = 2
    elbow_id: int = 3
    wrist_flex_id: int = 4
    wrist_roll_id: int = 5
    gripper_id: int = 6
    # 连杆长度 (mm) 人工设置，AI勿动
    upper_arm_length: float = 115.0  # 大臂长度 (L1)
    forearm_length: float = 130.0    # 小臂长度 (L2)
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
    serial_port: str = "COM15"
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
    speech_service_addr: str = "tcp://*:5570"   # 语音服务地址（备用）
    wakeup_pub_addr: str = "tcp://*:5571"       # 唤醒+ASR PUB地址


@dataclass
class LoggingConfig:
    """日志配置"""
    level: str = "DEBUG"


@dataclass
class SpeechConfig:
    """语音引擎配置"""
    # 模型路径
    wakeup_model_path: str = "models/wakeup"
    asr_model_path: str = "models/asr"
    cache_dir: str = "cache"
    
    # ASR模型文件
    asr_encoder_file: str = "encoder.int8.onnx"
    asr_decoder_file: str = "decoder.onnx"
    asr_joiner_file: str = "joiner.int8.onnx"
    
    # 唤醒模型文件
    wakeup_encoder_file: str = "encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx"
    wakeup_decoder_file: str = "decoder-epoch-13-avg-2-chunk-16-left-64.onnx"
    wakeup_joiner_file: str = "joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx"
    wakeup_keyword_file: str = "keywords.txt"
    
    # 音频参数
    sample_rate: int = 16000
    channels: int = 1
    mic_index: int = 1
    
    # 唤醒词配置
    wakeup_keyword: str = "你好小白"
    wakeup_sensitivity: float = 0.3
    
    # ASR监听超时（秒）
    listen_timeout: float = 1.5


@dataclass
class TTSConfig:
    """火山引擎TTS配置"""
    appid: str = ""                           # 应用ID
    access_token: str = ""                    # 访问令牌
    resource_id: str = "seed-tts-2.0"         # 资源ID
    voice_type: str = "zh_female_vv_uranus_bigtts"  # 音色类型
    encoding: str = "pcm"                     # 音频编码
    endpoint: str = "wss://openspeech.bytedance.com/api/v3/tts/bidirection"
    sample_rate: int = 16000                  # 输出采样率


@dataclass
class LLMConfig:
    """LLM API配置"""
    provider: str = "deepseek"                # 提供商: deepseek/qwen
    api_key: str = ""                         # API密钥
    api_url: str = "https://api.deepseek.com/v1"  # API地址
    model: str = "deepseek-chat"              # 模型名称
    temperature: float = 0.7                  # 温度参数
    max_tokens: int = 512                     # 最大token数


@dataclass
class GamepadConfig:
    """游戏手柄控制配置 - 同时控制底盘和机械臂"""
    
    # ========== 底盘控制参数 ==========
    max_linear_speed: float = 0.5          # 最大线速度 (m/s)
    max_angular_speed: float = 1.0         # 最大角速度 (rad/s)
    trigger_deadzone: float = 0.1          # 扳机键死区
    left_stick_deadzone: float = 0.15      # 左摇杆死区
    
    # ========== 机械臂控制参数 ==========
    arm_base_step: float = 3.0             # 基座关节步进 (度/帧)
    arm_elbow_step: float = 2.0            # 肘关节步进 (度/帧)
    arm_shoulder_step: float = 2.0         # 肩关节步进 (度/帧)
    arm_wrist_flex_step: float = 3.0       # 腕屈伸步进 (度/次)
    arm_wrist_roll_step: float = 3.0       # 腕旋转步进 (度/帧)
    arm_gripper_open: float = 90.0         # 夹爪打开角度
    arm_gripper_close: float = 0.0         # 夹爪关闭角度
    arm_speed: int = 800                   # 机械臂运动速度
    right_stick_deadzone: float = 0.15     # 右摇杆死区
    
    # ========== 通信配置 ==========
    chassis_service_addr: str = "tcp://localhost:5556"
    arm_service_addr: str = "tcp://localhost:5557"
    
    # ========== 轮询配置 ==========
    polling_interval: float = 0.02         # 50Hz (20ms)


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
    kp_linear: float = 0.8                    # 线速度P系数（归一化误差后）
    kp_angular: float = 1.5                   # 角速度P系数（归一化误差后）
    max_linear_speed: float = 0.5             # 最大线速度 (m/s)
    max_angular_speed: float = 2.0            # 最大角速度 (rad/s)
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
    speech: SpeechConfig = field(default_factory=SpeechConfig)
    tts: TTSConfig = field(default_factory=TTSConfig)
    llm: LLMConfig = field(default_factory=LLMConfig)
    gamepad: GamepadConfig = field(default_factory=GamepadConfig)
    
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
            human_follow=HumanFollowConfig(**data.get("human_follow", {})),
            speech=SpeechConfig(**data.get("speech", {})),
            tts=TTSConfig(**data.get("tts", {})),
            llm=LLMConfig(**data.get("llm", {})),
            gamepad=GamepadConfig(**data.get("gamepad", {}))
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
