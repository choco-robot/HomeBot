# -*- coding: utf-8 -*-
"""配置管理 - 集中管理所有硬件和系统配置

敏感配置（API密钥等）从 secrets 模块加载，不直接存储在此文件
"""
import os
from typing import Optional
from dataclasses import dataclass, field, asdict

import logging

# 导入密钥管理模块
from configs.secrets import get_secrets, Secrets

logger = logging.getLogger(__name__)


@dataclass
class CameraConfig:
    """摄像头配置"""
    device_id: int = 1
    width: int = 1920     # 摄像头原始分辨率
    height: int = 1080
    fps: int = 30


@dataclass
class LiftPlatformConfig:
    """升降平台配置 (Plus版本)
    
    坐标系定义:
    - 最高点设为 0 mm
    - 向下为负方向
    - 最低点为 -stroke_length mm (如 -200mm)
    """
    # 舵机ID
    servo_id_1: int = 10       # 升降舵机1
    servo_id_2: int = 11       # 升降舵机2
    
    # 机械参数
    lead: float = 20.0         # 丝杆导程 (mm/转)
    gear_ratio: float = 1.0    # 传动比 (电机转数:丝杆转数)
    angle_resolution: float = 4.0  # 角度分辨率
    
    # 行程长度 (mm)，用于坐标转换
    stroke_length: float = 200.0  # 总行程 (最高点 to 最低点)
    
    # 坐标系定义 (新坐标系: 最高点为0，向下为负)
    # height = 0 表示最高点
    # height = -stroke_length 表示最低点
    min_height: float = -200.0   # 最低位置坐标 (通常为 -stroke_length)
    max_height: float = 0.0      # 最高位置坐标 (通常为 0)
    
    # 运动参数
    default_speed: int = 0     # 0=最大速度
    default_acc: int = 0       # 0=默认加速度
    
    # 零位偏移 (步数)，用于校准两个舵机的同步
    servo_offset: int = 0      # 舵机2相对于舵机1的偏移
    
    # ========== 零点初始化参数 ==========
    auto_homing_on_startup: bool = True   # 启动时自动找零
    homing_direction: str = "up"          # 找零方向: "up"(向上找最高点) 或 "down"(向下找最低点)
    homing_speed: int = 300               # 找零速度 (不宜过快)
    homing_current_threshold: int = 300   # 电流阈值，超过此值判定为碰到限位 (需根据实际校准)
    homing_timeout: float = 15.0          # 找零超时时间(秒)
    homing_backoff_steps: int = 100       # 找到限位后回退步数 (释放压力)


@dataclass
class ArmConfig:
    """机械臂配置"""
    serial_port: str = "COM23"  # 与底盘共用串口
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
        "wrist_roll": 0,   # J5: 腕关节旋转
        "gripper": 45,     # J6: 夹爪（半开）
    })
    # 升降平台配置 (Plus版本)
    lift_platform: LiftPlatformConfig = field(default_factory=LiftPlatformConfig)


@dataclass
class ChassisConfig:
    """底盘配置 - 从机器人配置文件读取"""
    # 串口配置（Windows: COM3, Linux: /dev/ttyUSB0）
    serial_port: str = "COM23"
    baudrate: int = 1000000
    
    # 舵机ID映射
    left_front_id: int = 9
    right_front_id: int = 8
    rear_id: int = 7
    
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
    level: str = "INFO"


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
    wakeup_sensitivity: float = 0.2
    
    # ASR监听超时（秒）
    listen_timeout: float = 1.5


@dataclass
class TTSConfig:
    """火山引擎TTS配置
    
    敏感信息（appid, access_token）从 secrets 模块加载
    如需修改，请在 .env.local 文件中设置
    """
    # 以下配置从环境变量/密钥管理加载
    appid: str = ""                           # 应用ID
    access_token: str = ""                    # 访问令牌
    resource_id: str = "seed-tts-2.0"         # 资源ID
    voice_type: str = "zh_female_vv_uranus_bigtts"  # 音色类型
    encoding: str = "pcm"                     # 音频编码
    endpoint: str = "wss://openspeech.bytedance.com/api/v3/tts/bidirection"
    sample_rate: int = 16000                  # 输出采样率
    
    def __post_init__(self):
        """从密钥管理加载敏感配置"""
        if not self.appid or not self.access_token:
            secrets = get_secrets()
            if not self.appid:
                self.appid = secrets.tts.appid
            if not self.access_token:
                self.access_token = secrets.tts.access_token
            # 非敏感配置也可以从环境变量覆盖
            if secrets.tts.resource_id:
                self.resource_id = secrets.tts.resource_id
            if secrets.tts.voice_type:
                self.voice_type = secrets.tts.voice_type


@dataclass
class LLMConfig:
    """LLM API配置
    
    敏感信息（api_key）从 secrets 模块加载
    如需修改，请在 .env.local 文件中设置
    """
    provider: str = "volcano"                 # 提供商: volcano/deepseek/qwen
    api_key: str = ""                         # API密钥
    api_url: str = "https://ark.cn-beijing.volces.com/api/v3"  # API地址
    model: str = ""                           # 模型名称（火山Ark需要填写模型ID，如 ep-20250324123456-abcdef）
    temperature: float = 0.1                  # 温度参数（低温度=更确定性回复，响应更快）
    max_tokens: int = 256                     # 最大token数（限制回复长度，提升速度）
    top_p: float = 0.9                        # 核采样（控制输出多样性）
    
    def __post_init__(self):
        """从密钥管理加载敏感配置"""
        secrets = get_secrets()
        if not self.api_key:
            self.api_key = secrets.llm.api_key
        # 非敏感配置可以从环境变量覆盖
        if secrets.llm.api_url:
            self.api_url = secrets.llm.api_url
        if secrets.llm.model:
            self.model = secrets.llm.model
        # 如果没有配置model，给出警告
        if not self.model:
            logger.warning("LLM模型未配置，请在.env.local中设置 ARK_MODEL_ID 或 LLM_MODEL")


@dataclass
class VisionConfig:
    """图片理解/Vision API配置
    
    支持多提供商: deepseek/qwen/openai
    敏感信息从 secrets 模块加载
    """
    provider: str = "deepseek"                # 提供商
    api_key: str = ""                         # API密钥
    api_url: str = ""                         # API地址
    model: str = ""                           # 模型名称
    temperature: float = 0.7                  # 温度参数
    max_tokens: int = 1024                    # 最大token数
    
    def __post_init__(self):
        """从密钥管理加载配置"""
        secrets = get_secrets()
        
        # 如果未指定provider，使用环境变量的配置
        env_provider = secrets.vision.provider
        if env_provider:
            self.provider = env_provider
        
        # 加载密钥和URL
        if secrets.vision.api_key:
            self.api_key = secrets.vision.api_key
        if secrets.vision.api_url:
            self.api_url = secrets.vision.api_url
        if secrets.vision.model:
            self.model = secrets.vision.model
        
        # 如果没有单独配置Vision，复用DeepSeek LLM配置
        if self.provider == "deepseek":
            if not self.api_key:
                self.api_key = secrets.llm.api_key
            if not self.api_url:
                self.api_url = secrets.llm.api_url or "https://api.deepseek.com/v1"
            if not self.model:
                self.model = "deepseek-chat"
        
        # 提供商特定的默认配置
        elif self.provider == "qwen":
            if not self.api_url:
                self.api_url = "https://dashscope.aliyuncs.com/compatible-mode/v1"
            if not self.model:
                self.model = "qwen-vl-plus"
        
        elif self.provider == "openai":
            if not self.api_url:
                self.api_url = "https://api.openai.com/v1"
            if not self.model:
                self.model = "gpt-4o"


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
    target_width_ratio: float = 0.4          # 1米处人体占画面宽度比例（0.25=25%）
    target_height_ratio: float = 1.0          # 1米处人体占画面高度比例（1.0=100%）
    kp_linear: float = 0.8                    # 线速度P系数（归一化误差后）
    kp_angular: float = 1.5                   # 角速度P系数（归一化误差后）
    max_linear_speed: float = 0.5             # 最大线速度 (m/s)
    max_angular_speed: float = 2.0            # 最大角速度 (rad/s)
    dead_zone_x: float = 0.15                 # 水平死区（比例值，0.15=15%画面宽度）
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
class BatteryConfig:
    """电池监测配置"""
    # 用于读取电压的舵机ID列表（按优先级排序）
    servo_ids: list = field(default_factory=lambda: [1])  # 默认使用ID 1
    
    # 电压阈值配置 (3S锂电池)
    full_voltage: float = 12.6     # 满电电压 (V)
    low_voltage: float = 10.5      # 低电量阈值 (V)
    critical_voltage: float = 9.5  # 严重低电量阈值 (V)
    min_voltage: float = 9.0       # 最低工作电压 (V)
    
    # 发布配置
    publish_interval: float = 5.0  # 电压信息发布间隔 (秒)
    pub_addr: str = "tcp://*:5555"  # 电池状态PUB地址


@dataclass
class Config:
    """全局配置"""
    camera: CameraConfig = field(default_factory=CameraConfig)
    arm: ArmConfig = field(default_factory=ArmConfig)
    lift_platform: LiftPlatformConfig = field(default_factory=LiftPlatformConfig)
    chassis: ChassisConfig = field(default_factory=ChassisConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    zmq: ZMQConfig = field(default_factory=ZMQConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    human_follow: HumanFollowConfig = field(default_factory=HumanFollowConfig)
    speech: SpeechConfig = field(default_factory=SpeechConfig)
    tts: TTSConfig = field(default_factory=TTSConfig)
    llm: LLMConfig = field(default_factory=LLMConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
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
            battery=BatteryConfig(**data.get("battery", {})),
            zmq=ZMQConfig(**data.get("zmq", {})),
            logging=LoggingConfig(**data.get("logging", {})),
            human_follow=HumanFollowConfig(**data.get("human_follow", {})),
            speech=SpeechConfig(**data.get("speech", {})),
            tts=TTSConfig(**data.get("tts", {})),
            llm=LLMConfig(**data.get("llm", {})),
            vision=VisionConfig(**data.get("vision", {})),
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
