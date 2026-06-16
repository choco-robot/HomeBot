# -*- coding: utf-8 -*-
"""配置管理 - 集中管理所有硬件和系统配置

敏感配置（API密钥等）从 secrets 模块加载，不直接存储在此文件
"""
import os
from typing import Optional, Dict, Tuple
from dataclasses import dataclass, field, asdict

import logging

# 导入密钥管理模块
from configs.secrets import get_secrets, Secrets

logger = logging.getLogger(__name__)

# ------------------------------------------------------------------------------
# 项目路径宏（便于配置文件中写相对路径）
# ------------------------------------------------------------------------------

# config.py 位于 software/src/configs/，向上回溯三级到项目根目录
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SOFTWARE_DIR = os.path.join(PROJECT_ROOT, "software")
HARDWARE_DIR = os.path.join(PROJECT_ROOT, "hardware")
DOCS_DIR = os.path.join(PROJECT_ROOT, "docs")
MAPS_DIR = os.path.join(PROJECT_ROOT, "software", "maps")
MODELS_DIR = os.path.join(PROJECT_ROOT, "software", "models")


@dataclass
class CameraConfig:
    """摄像头配置"""
    device_id: int = 0
    width: int = 1920     # 摄像头原始分辨率
    height: int = 1080
    fps: int = 30


@dataclass
class WebCameraConfig:
    """网络摄像头配置"""
    enabled: bool = False                         # 是否启用网络摄像头
    url: str = "rtsp://admin:admin@192.168.1.100:554/live"  # 视频流地址
    width: int = 0                                # 0=使用原始分辨率
    height: int = 0                               # 0=使用原始分辨率
    fps: int = 0                                  # 0=使用原始帧率
    reconnect_interval: float = 5.0               # 断线重连间隔 (秒)


@dataclass
class ArmConfig:
    """机械臂配置"""
    serial_port: str = "/usr/local/dev/servobus"  # 与底盘共用串口
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
        "base": (-90, 90),
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
        "base": 0,         # J1: 基座旋转
        "shoulder": 15,   # J2: 肩关节（自然下垂）
        "elbow": 150,       # J3: 肘关节
        "wrist_flex": 0,   # J4: 腕关节屈伸
        "wrist_roll": 0,   # J5: 腕关节旋转
        "gripper": 45,     # J6: 夹爪（半开）
    })


@dataclass
class ChassisConfig:
    """底盘配置 - 从机器人配置文件读取"""
    # 底盘类型: "omni" = 三轮全向轮, "diff" = 双轮差动
    chassis_type: str = "diff"
    
    # 串口配置（Windows: COM3, Linux: /dev/ttyUSB0）
    serial_port: str = "/usr/local/dev/servobus"
    baudrate: int = 1000000
    
    # 舵机ID映射
    left_front_id: int = 7
    right_front_id: int = 8
    rear_id: int = 9
    
    # 物理参数
    wheel_radius: float = 0.04      # 轮子半径 (m)
    chassis_radius: float = 0.14     # 底盘半径 (m)
    
    # 运动限制
    max_linear_speed: float = 0.3    # 最大线速度 (m/s)，已根据舵机满速 47.45 RPM 校准
    max_angular_speed: float = 1.0   # 最大角速度 (rad/s)
    default_wheel_speed: int = 3250  # 舵机最大速度读数（与 servo_speed_scale 一致）
    
    # 舵机速度物理参数（已根据数据手册确认：1单位 = 0.0146 RPM）
    servo_speed_scale: int = 3250    # 舵机速度满量程读数（100%输出 = 3250）
    servo_max_rpm: float = 47.45     # 3250读数对应的实际转速 RPM（3250 * 0.0146）
    
    # ZeroMQ地址
    service_addr: str = "tcp://*:5556"


@dataclass
class ZMQConfig:
    """ZeroMQ网络配置"""
    chassis_service_addr: str = "tcp://*:5556"
    chassis_state_pub_addr: str = "tcp://*:5558"  # 底盘状态 PUB 地址
    arm_service_addr: str = "tcp://*:5557"      # 机械臂服务地址
    vision_pub_addr: str = "tcp://*:5560"
    depth_pub_addr: str = "tcp://*:5561"        # 深度图 PUB 地址
    obstacle_pub_addr: str = "tcp://*:5562"     # 障碍物信息 PUB 地址
    odom_pub_addr: str = "tcp://*:5559"          # 里程计 PUB 地址
    slam_pose_pub_addr: str = "tcp://*:5563"    # SLAM 位姿 PUB 地址
    slam_map_pub_addr: str = "tcp://*:5564"     # SLAM 地图 PUB 地址
    lidar_scan_pub_addr: str = "tcp://*:5565"   # 激光雷达扫描数据 PUB 地址
    goal_pub_addr: str = "tcp://*:5566"          # 目标点 PUB 地址
    odom_cmd_addr: str = "tcp://*:5567"          # 里程计命令 REP 地址
    slam_cmd_addr: str = "tcp://*:5568"          # SLAM 命令 REP 地址
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
        # 同步提供商
        if secrets.llm.provider:
            self.provider = secrets.llm.provider.lower()
        if not self.api_key:
            self.api_key = secrets.llm.api_key
        
        # 非敏感配置可以从环境变量覆盖
        if secrets.llm.api_url:
            self.api_url = secrets.llm.api_url
        if secrets.llm.model:
            self.model = secrets.llm.model
        
        # 根据 provider 设置默认 URL 和 model
        if self.provider == "deepseek":
            if not self.api_url:
                self.api_url = "https://api.deepseek.com/v1"
            if not self.model:
                self.model = "deepseek-chat"
        elif self.provider == "volcano":
            if not self.api_url:
                self.api_url = "https://ark.cn-beijing.volces.com/api/v3"
        elif self.provider == "qwen":
            if not self.api_url:
                self.api_url = "https://dashscope.aliyuncs.com/compatible-mode/v1"
            if not self.model:
                self.model = "qwen-turbo"
        
        # 如果没有配置model，给出警告
        if not self.model:
            if self.provider == "deepseek":
                logger.warning("LLM模型未配置，将使用默认模型 deepseek-chat")
                self.model = "deepseek-chat"
            else:
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
class NavigationConfig:
    """导航配置（全局路径规划 + 局部避障）"""
    # 速度限制
    max_linear_speed: float = 0.3        # 导航时最大线速度 (m/s)
    max_angular_speed: float = 0.8       # 导航时最大角速度 (rad/s)
    
    # 到达判断阈值
    arrival_distance_threshold_m: float = 0.15   # 到达目标点的距离阈值（米）
    arrival_angle_threshold_rad: float = 0.15    # 到达目标点的角度阈值（弧度）
    
    # 避障配置
    use_depth_obstacle: bool = False      # 是否启用基于深度视觉的避障
    emergency_obstacle_distance_m: float = 0.3   # 紧急避障距离（米）
    safety_distance_m: float = 0.5       # VFH 安全距离（米）
    
    # 规划配置
    max_replan_attempts: int = 5         # 全局路径规划最大重试次数
    replan_interval_s: float = 3.0       # 重规划间隔（秒）
    lookahead_distance_m: float = 0.5    # 路径跟踪前瞻距离（米）
    max_path_deviation_m: float = 1.0    # 允许偏离全局路径的最大距离（米）
    inflation_radius_m: float = 0.2      # 障碍物膨胀半径（米）
    robot_radius_m: float = 0.15         # 机器人半径（米）
    
    # 功能开关
    enable_apriltag: bool = False        # 是否启用 AprilTag 检测（用于视觉定位校正）
    
    # 控制频率
    control_rate_hz: float = 10.0        # 导航控制循环频率（Hz）
    
    # 角速度变化率限制
    max_angular_accel_rad: float = 2.0   # 最大角加速度（rad/s²），用于平滑转向突变
    
    # 速度低通滤波器
    velocity_filter_alpha: float = 0.7   # 一阶低通滤波系数（0~1），越小越平滑


@dataclass
class SLAMConfig:
    """SLAM 与视觉定位配置"""
    # 雷达配置
    lidar_port: str = "/usr/local/dev/lidar"                     # 激光雷达串口
    lidar_scan_size: int = 360                   # 扫描分辨率（点数）
    lidar_max_distance_m: float = 12.0           # 最大检测距离
    lidar_min_distance_m: float = 0.2            # 最小检测距离（过滤机器人本体结构，20cm）
    
    # 地图配置
    map_size_pixels: int = 800                   # 栅格地图像素尺寸
    map_size_meters: float = 20.0                # 地图物理尺寸（米）
    
    # AprilTag 配置
    tag_family: str = "tag36h11"
    tag_size_m: float = 0.165                    # 标签边长（米）
    tag_map: dict = field(default_factory=lambda: {
        # 示例：标签ID → 世界位姿 (x_m, y_m, theta_rad)
        # 0: (1.0, 0.0, 0.0),
        # 1: (3.0, 2.0, 1.5708),
    })
    
    # 相机内参（需根据实际标定结果替换）
    camera_fx: float = 600.0
    camera_fy: float = 600.0
    camera_cx: float = 320.0
    camera_cy: float = 240.0
    
    # 融合参数
    confidence_threshold: float = 0.8            # 硬校正触发阈值
    odom_consistency_threshold: float = 9.21     # 里程计一致性卡方阈值
    
    # 地图持久化
    map_save_dir: str = "maps"                   # 默认地图保存目录


@dataclass
class ViserConfig:
    """Viser SLAM 可视化配置"""
    host: str = "0.0.0.0"
    port: int = 8080
    # 订阅地址
    odom_sub_addr: str = "tcp://localhost:5559"
    slam_pose_sub_addr: str = "tcp://localhost:5563"
    slam_map_sub_addr: str = "tcp://localhost:5564"
    lidar_scan_sub_addr: str = "tcp://localhost:5565"
    vision_sub_addr: str = "tcp://localhost:5560"
    # 发布地址
    goal_pub_addr: str = "tcp://*:5566"          # 目标点 PUB 地址
    odom_cmd_addr: str = "tcp://localhost:5567"  # 里程计命令 REQ 地址
    slam_cmd_addr: str = "tcp://localhost:5568"  # SLAM 命令 REQ 地址
    global_path_sub_addr: str = "tcp://localhost:5569"  # 全局路径 SUB 地址
    nav_status_sub_addr: str = "tcp://localhost:5570"  # 导航状态 SUB 地址
    maps_dir: str = ""  # 地图文件夹路径（空则自动探测 software/maps）
    urdf_path: str = os.path.join(HARDWARE_DIR, "structure", "URDF", "双轮差动小推车", "双轮差动小推车.urdf")  # 机器人 URDF 文件路径（空则使用简易圆柱造型）
    urdf_color_override: Optional[Tuple[float, float, float]] = None  # URDF 模型颜色覆盖 (R, G, B)，None 则使用 URDF 自带颜色
    # 激光雷达坐标适配（度）
    # 不同雷达的 0° 零位方向不同：LD06 0°=后方(180°)，标准极坐标 0°=前方(0°)
    lidar_rotation_offset_deg: float = 180.0
    
    # 可视化参数
    max_trajectory_points: int = 5000   # 最大轨迹点数
    point_size: float = 0.03            # 激光点大小
    map_update_interval: float = 2.0    # 地图更新间隔（秒）
    follow_robot: bool = True           # 默认跟随机器人


@dataclass
class GamepadConfig:
    """游戏手柄控制配置 - 同时控制底盘和机械臂"""
    
    # ========== 底盘控制参数 ==========
    max_linear_speed: float = 0.3          # 最大线速度 (m/s)
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
class ArmTeleopConfig:
    """机械臂 WLAN 遥操作配置"""
    # 从端 arm_service 地址（WLAN 内从端机器人 IP）
    slave_arm_addr: str = "tcp://192.168.137.159:5557"

    # 主臂读取频率 / 下发频率
    read_rate: float = 50.0            # Hz
    send_rate: float = 30.0            # Hz

    # 主臂扭矩：True=关闭扭矩方便拖动，False=保持使能
    torque_off: bool = True

    # 启动时是否默认开启遥操作
    enabled_by_default: bool = False

    # 通信超时（ms）
    timeout_ms: int = 800

    # 默认/最小/最大下发速度（舵机速度单位）
    default_speed: int = 800
    min_speed: int = 100
    max_speed: int = 2000
    hold_speed: int = 200              # 关闭遥操作时保持位置用的慢速

    # 速度比例因子：deg/s -> servo speed，需根据 SO101 标定微调
    speed_scale: float = 15.0

    # 角度死区（度）：小于该值不发新命令，减少抖动
    deadband_deg: float = 0.5

    # 默认回放轨迹文件路径（热键 'p' 使用）
    default_playback_file: str = ""

    # 录制轨迹默认保存目录
    trajectory_dir: str = "trajectories"

    # 关节映射：master_joint -> (slave_joint, sign)
    # 默认恒等映射；示例："shoulder": ("shoulder", -1)
    joint_mapping: Dict[str, Tuple[str, int]] = field(default_factory=lambda: {
        "base": ("base", 1),
        "shoulder": ("shoulder", 1),
        "elbow": ("elbow", 1),
        "wrist_flex": ("wrist_flex", 1),
        "wrist_roll": ("wrist_roll", 1),
        "gripper": ("gripper", 1),
    })


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
    webcamera: WebCameraConfig = field(default_factory=WebCameraConfig)
    arm: ArmConfig = field(default_factory=ArmConfig)
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
    arm_teleop: ArmTeleopConfig = field(default_factory=ArmTeleopConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    slam: SLAMConfig = field(default_factory=SLAMConfig)
    viser: ViserConfig = field(default_factory=ViserConfig)
    
    def to_dict(self) -> dict:
        """转换为字典"""
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: dict) -> "Config":
        """从字典创建配置"""
        return cls(
            camera=CameraConfig(**data.get("camera", {})),
            webcamera=WebCameraConfig(**data.get("webcamera", {})),
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
            gamepad=GamepadConfig(**data.get("gamepad", {})),
            arm_teleop=ArmTeleopConfig(**data.get("arm_teleop", {})),
            navigation=NavigationConfig(**data.get("navigation", {})),
            slam=SLAMConfig(**data.get("slam", {})),
            viser=ViserConfig(**data.get("viser", {})),
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
