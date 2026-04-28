"""
机械臂人脸跟踪核心逻辑

功能：
1. 订阅视觉服务的图像流
2. 检测人脸并选择主要目标
3. 通过PID控制计算 base 和 wrist_flex 的角度调整
4. 发送控制指令到机械臂服务

只控制两个关节：
- base: 基座旋转，控制相机左右
- wrist_flex: 腕关节屈伸，控制相机上下俯仰
"""
import sys
import os
import time
import threading
from typing import Optional, Dict, Tuple
from dataclasses import dataclass
from enum import Enum

import numpy as np
import cv2

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from common.logging import get_logger
from configs import get_config
from services.vision_service import VisionSubscriber
from services.motion_service.chassis_arbiter import ArmArbiterClient

logger = get_logger(__name__)


class TrackerState(Enum):
    """跟踪状态"""
    IDLE = "idle"
    TRACKING = "tracking"
    SEARCHING = "searching"
    ERROR = "error"


@dataclass
class FaceTrackerConfig:
    """人脸跟踪配置"""
    # 视觉服务配置
    vision_sub_addr: str = "tcp://localhost:5560"
    
    # 机械臂服务配置
    arm_service_addr: str = "tcp://localhost:5557"
    
    # 人脸检测配置
    model_path: str = "models/yolov8n-face-lindevs.pt"
    conf_threshold: float = 0.5
    inference_size: int = 640
    device: str = "cpu"            # 推理设备: cpu/cuda/mps
    
    # 控制参数
    kp_base: float = 0.01          # 水平方向比例系数 (度/像素偏差)
    kp_wrist: float = 0.01         # 垂直方向比例系数 (度/像素偏差)
    max_base_step: float = 5.0     # 每帧最大base角度变化 (度)
    max_wrist_step: float = 5.0    # 每帧最大wrist_flex角度变化 (度)
    dead_zone_pixels: int = 20     # 画面中心死区 (像素)
    
    # 丢失处理
    lost_patience: int = 30        # 丢失容忍帧数
    
    # 搜索配置（慢速舵机扫描）
    search_speed: int = 300        # 搜索时舵机速度（慢速，数值越小越慢）
    search_angle_range: float = 90.0  # 搜索角度范围边界（±度）
    search_switch_interval: float = 4.0  # 扫描方向切换间隔（秒）
    startup_search_timeout: float = 3.0  # 启动后等待人脸超时（秒）
    
    # 机械臂速度
    arm_speed: int = 800           # 正常运动速度
    arm_priority: int = 3          # 控制优先级 (auto=3)
    arm_source: str = "auto"       # 控制源标识
    
    # 初始/复位关节角度
    initial_base: float = 0.0
    initial_wrist_flex: float = 0.0
    initial_shoulder: float = 90.0
    initial_elbow: float = 90.0
    initial_wrist_roll: float = 0.0
    initial_gripper: float = 45.0


class PIDController:
    """简单PID控制器"""
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
    
    def compute(self, error: float, dt: Optional[float] = None) -> float:
        now = time.time()
        if dt is None:
            dt = now - self.prev_time
        if dt <= 0:
            dt = 0.033  # 默认30fps
        
        self.integral += error * dt
        # 积分限幅
        self.integral = max(-100, min(100, self.integral))
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error
        self.prev_time = now
        
        return output


class FaceTrackerApp:
    """
    机械臂人脸跟踪应用
    
    控制手腕相机的 base 和 wrist_flex，使检测到的主要人脸保持在画面中心。
    """
    
    def __init__(self, config: Optional[FaceTrackerConfig] = None):
        self.config = config or FaceTrackerConfig()
        
        # 组件
        self.vision_sub: Optional[VisionSubscriber] = None
        self.arm_client: Optional[ArmArbiterClient] = None
        self.detector = None
        
        # 状态
        self.state = TrackerState.IDLE
        self.running = False
        self._stop_event = threading.Event()
        
        # 从全局配置的 rest_position 同步初始角度，保持一致
        try:
            global_config = get_config()
            rest = global_config.arm.rest_position
            self.config.initial_base = rest.get("base", self.config.initial_base)
            self.config.initial_shoulder = rest.get("shoulder", self.config.initial_shoulder)
            self.config.initial_elbow = rest.get("elbow", self.config.initial_elbow)
            self.config.initial_wrist_flex = rest.get("wrist_flex", self.config.initial_wrist_flex)
            self.config.initial_wrist_roll = rest.get("wrist_roll", self.config.initial_wrist_roll)
            self.config.initial_gripper = rest.get("gripper", self.config.initial_gripper)
            self.joint_limits = global_config.arm.joint_limits
        except Exception as e:
            logger.warning(f"无法读取全局配置，使用默认值: {e}")
            self.joint_limits = {
                "base": (-180, 180),
                "shoulder": (-90, 90),
                "elbow": (-120, 120),
                "wrist_flex": (-90, 90),
                "wrist_roll": (-180, 180),
                "gripper": (0, 90),
            }
        
        # 机械臂当前关节状态
        self.arm_state: Dict[str, float] = {
            "base": self.config.initial_base,
            "shoulder": self.config.initial_shoulder,
            "elbow": self.config.initial_elbow,
            "wrist_flex": self.config.initial_wrist_flex,
            "wrist_roll": self.config.initial_wrist_roll,
            "gripper": self.config.initial_gripper,
        }
        
        # PID控制器
        self.base_pid = PIDController(kp=self.config.kp_base, ki=0.0, kd=0.0)
        self.wrist_pid = PIDController(kp=self.config.kp_wrist, ki=0.0, kd=0.0)
        
        # 丢失计数
        self.lost_count = 0
        
        # 性能统计
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0
        
        # 搜索状态
        self._startup_time = 0.0
        self._search_last_switch_time = 0.0
        self._search_direction = 1
        self._search_target_base = 0.0   # 当前搜索目标角度（用于切换到跟踪时同步基准）
        self._tracking_warmup = 0        # 跟踪软启动帧数
        
        logger.info("FaceTrackerApp 初始化完成")
    
    def initialize(self) -> bool:
        """初始化所有组件"""
        logger.info("=" * 60)
        logger.info("初始化人脸跟踪应用")
        logger.info("=" * 60)
        
        try:
            # 1. 连接视觉服务
            logger.info("连接视觉服务...")
            self.vision_sub = VisionSubscriber(self.config.vision_sub_addr)
            self.vision_sub.start()
            logger.info(f"✓ 视觉订阅已连接: {self.config.vision_sub_addr}")
            
            # 2. 连接机械臂服务
            logger.info("连接机械臂服务...")
            self.arm_client = ArmArbiterClient(
                service_addr=self.config.arm_service_addr,
                timeout_ms=500
            )
            logger.info(f"✓ 机械臂客户端已连接: {self.config.arm_service_addr}")
            
            # 3. 初始化人脸检测器
            logger.info("加载人脸检测模型...")
            # 复用 human_follow 的 HumanDetector，使用 face 模式
            from applications.human_follow.detector import HumanDetector
            
            self.detector = HumanDetector(
                model_path=self.config.model_path,
                conf_threshold=self.config.conf_threshold,
                inference_size=self.config.inference_size,
                device=self.config.device,
                detect_mode="face"
            )
            if not self.detector.initialize():
                logger.error("✗ 人脸检测器初始化失败")
                return False
            logger.info("✓ 人脸检测器已加载")
            
            # 4. 尝试同步当前机械臂状态
            self._sync_arm_state()
            
            logger.info("=" * 60)
            logger.info("初始化完成")
            logger.info("=" * 60)
            return True
            
        except Exception as e:
            logger.error(f"初始化失败: {e}")
            self.state = TrackerState.ERROR
            return False
    
    def _sync_arm_state(self):
        """尝试从机械臂服务获取当前关节状态"""
        try:
            if self.arm_client:
                # 通过 send_joint_dict 的查询模式或直接查询
                # ArmArbiterClient 没有直接的查询接口，但我们可以通过发送空命令或
                # 使用底层 socket 查询。这里暂时使用配置的 rest_position 作为初始值
                config = get_config()
                rest = config.arm.rest_position
                self.arm_state.update({
                    "base": rest.get("base", 0),
                    "shoulder": rest.get("shoulder", 90),
                    "elbow": rest.get("elbow", 90),
                    "wrist_flex": rest.get("wrist_flex", 0),
                    "wrist_roll": rest.get("wrist_roll", 0),
                    "gripper": rest.get("gripper", 45),
                })
                logger.info(f"机械臂初始状态: base={self.arm_state['base']:.1f}, "
                           f"wrist_flex={self.arm_state['wrist_flex']:.1f}")
        except Exception as e:
            logger.warning(f"同步机械臂状态失败: {e}")
    
    def _send_arm_command(self, joints: Dict[str, float], speed: int = None) -> bool:
        """
        发送机械臂关节角度指令
        
        Args:
            joints: 关节角度字典
            speed: 运动速度（None则使用配置默认值）
            
        Returns:
            bool: 是否成功
        """
        if self.arm_client is None:
            return False
        
        if speed is None:
            speed = self.config.arm_speed
        
        try:
            response = self.arm_client.send_joint_dict(
                joints_dict=joints,
                source=self.config.arm_source,
                priority=self.config.arm_priority,
                speed=speed
            )
            return response.success if response else False
        except Exception as e:
            logger.warning(f"发送机械臂指令失败: {e}")
            return False
    
    def _query_arm_state(self) -> Optional[Dict[str, float]]:
        """
        查询机械臂当前实际关节状态
        
        Returns:
            关节角度字典，如 {"base": 10.5, "shoulder": 45.0, ...}，失败返回 None
        """
        if self.arm_client is None:
            return None
        
        try:
            import time
            
            command = {
                "source": self.config.arm_source,
                "joints": {},
                "speed": 0,
                "priority": self.config.arm_priority,
                "query": True,
                "timestamp": time.time()
            }
            
            # 直接使用底层 socket 发送查询请求
            socket = self.arm_client._socket
            socket.send_json(command)
            response_data = socket.recv_json()
            
            if response_data.get("success"):
                joint_states = response_data.get("joint_states")
                if joint_states:
                    return joint_states
            
            return None
        except Exception as e:
            logger.warning(f"查询机械臂状态失败: {e}")
            return None
    
    def _clamp_angle(self, angle: float, joint_name: str) -> float:
        """将角度限制在关节限制范围内"""
        limits = self.joint_limits.get(joint_name, (-180, 180))
        return max(limits[0], min(limits[1], angle))
    
    def _reset_wrist_flex(self):
        """wrist_flex 归零，保持其他关节不变"""
        self.arm_state["wrist_flex"] = self.config.initial_wrist_flex
        self.arm_state["wrist_flex"] = self._clamp_angle(self.arm_state["wrist_flex"], "wrist_flex")
        logger.info(f"wrist_flex 归零: {self.arm_state['wrist_flex']:.1f}°")
        self._send_arm_command(self.arm_state, speed=self.config.arm_speed)
    
    def _select_face(self, detections, frame_width: int, frame_height: int) -> Optional:
        """
        从检测结果中选择主要人脸
        
        策略：优先选择最大的人脸（面积最大），也可以改为选择画面中心最近的
        
        Args:
            detections: 检测结果列表
            frame_width: 画面宽度
            frame_height: 画面高度
            
        Returns:
            选中的 Detection 对象或 None
        """
        if not detections:
            return None
        
        # 选择面积最大的人脸
        best_face = max(detections, key=lambda d: d.area)
        return best_face
    
    def _compute_control(self, face_cx: int, face_cy: int, 
                         frame_width: int, frame_height: int) -> Tuple[float, float]:
        """
        根据人脸中心与画面中心的偏差，计算关节角度调整量
        
        Args:
            face_cx, face_cy: 人脸中心坐标
            frame_width, frame_height: 画面尺寸
            
        Returns:
            (delta_base, delta_wrist) 角度调整量（度）
        """
        center_x = frame_width // 2
        center_y = frame_height // 2
        
        error_x = face_cx - center_x
        error_y = face_cy - center_y
        
        # 死区检查
        if abs(error_x) < self.config.dead_zone_pixels:
            error_x = 0
        if abs(error_y) < self.config.dead_zone_pixels:
            error_y = 0
        
        # PID计算
        delta_base = -self.base_pid.compute(error_x)   # 负号：人脸偏右->base左转
        delta_wrist = self.wrist_pid.compute(error_y)   # 人脸偏上->wrist向上转
        
        # 软启动：切换后前几帧限制步长，让人脸平滑收敛到中心
        if self._tracking_warmup > 0:
            warmup_ratio = 0.3  # 软启动期间只使用 30% 最大步长
            max_base = self.config.max_base_step * warmup_ratio
            max_wrist = self.config.max_wrist_step * warmup_ratio
            self._tracking_warmup -= 1
        else:
            max_base = self.config.max_base_step
            max_wrist = self.config.max_wrist_step
        
        # 限幅
        delta_base = max(-max_base, min(max_base, delta_base))
        delta_wrist = max(-max_wrist, min(max_wrist, delta_wrist))
        
        return delta_base, delta_wrist
    
    def _update_fps(self):
        """更新FPS统计"""
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed >= 1.0:
            self.current_fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def _process_frame(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """
        处理单帧图像
        
        Args:
            frame: 输入图像
            
        Returns:
            可视化后的图像（调试用）
        """
        if frame is None:
            return None
        
        h, w = frame.shape[:2]
        
        # 1. 人脸检测
        detections = self.detector.detect(frame)
        
        # 2. 选择目标人脸
        face = self._select_face(detections, w, h)
        
        # 3. 状态机与控制
        if face:
            # 目标存在
            self.lost_count = 0
            
            if self.state in (TrackerState.IDLE, TrackerState.SEARCHING):
                prev_state = self.state
                logger.info("检测到人脸，开始跟踪")
                self.state = TrackerState.TRACKING
                self.base_pid.reset()
                self.wrist_pid.reset()
                
                # 从搜索切换时，先读取当前实际 base 角度作为基准
                if prev_state == TrackerState.SEARCHING:
                    joint_states = self._query_arm_state()
                    if joint_states and "base" in joint_states:
                        actual_base = joint_states["base"]
                        self.arm_state["base"] = actual_base
                        logger.info(f"同步实际 base 角度: {actual_base:.1f}°")
                    else:
                        # 查询失败则回退到搜索目标估算值
                        self.arm_state["base"] = self._search_target_base
                        self.arm_state["base"] = self._clamp_angle(self.arm_state["base"], "base")
                        logger.warning(f"查询实际角度失败，使用估算值: {self.arm_state['base']:.1f}°")
                    self._tracking_warmup = 10  # 软启动 10 帧，防止人脸在边缘时猛甩
            
            # 计算控制量
            face_cx, face_cy = face.center
            delta_base, delta_wrist = self._compute_control(face_cx, face_cy, w, h)
            
            # 更新关节角度
            new_base = self.arm_state["base"] + delta_base
            new_wrist = self.arm_state["wrist_flex"] + delta_wrist
            
            # 限幅
            new_base = self._clamp_angle(new_base, "base")
            new_wrist = self._clamp_angle(new_wrist, "wrist_flex")
            
            # 更新状态
            self.arm_state["base"] = new_base
            self.arm_state["wrist_flex"] = new_wrist
            
            # 发送指令
            self._send_arm_command(self.arm_state)
            
            # 绘制可视化
            output = self._visualize(frame, face, w, h, delta_base, delta_wrist)
            
        else:
            # 目标丢失
            now = time.time()
            
            if self.state == TrackerState.IDLE:
                # 启动后等待人脸，超时则进入搜索
                elapsed = now - self._startup_time
                if elapsed > self.config.startup_search_timeout:
                    logger.info(f"启动后 {self.config.startup_search_timeout} 秒未检测到人脸，进入搜索模式")
                    self.state = TrackerState.SEARCHING
                    self._search_last_switch_time = 0.0
                    self._reset_wrist_flex()
            
            elif self.state == TrackerState.TRACKING:
                self.lost_count += 1
                if self.lost_count > self.config.lost_patience:
                    logger.info("人脸丢失，进入搜索模式")
                    self.state = TrackerState.SEARCHING
                    self.lost_count = 0
                    self._search_last_switch_time = 0.0
                    self._reset_wrist_flex()
            
            elif self.state == TrackerState.SEARCHING:
                # 慢速扫描：通过设置舵机速度和远端目标，让舵机自动转动
                # 无需每帧发送指令，只需定期切换方向
                if now - self._search_last_switch_time > self.config.search_switch_interval:
                    target = self.config.search_angle_range * self._search_direction
                    target = self._clamp_angle(target, "base")
                    
                    # 记录搜索目标，但不污染 arm_state（跟踪基准值保持独立）
                    self._search_target_base = target
                    
                    search_joints = self.arm_state.copy()
                    search_joints["base"] = target
                    
                    logger.info(f"搜索扫描: base -> {target:.0f}°, speed={self.config.search_speed}")
                    self._send_arm_command(search_joints, speed=self.config.search_speed)
                    
                    # 切换方向
                    self._search_direction *= -1
                    self._search_last_switch_time = now
            
            output = self._visualize(frame, None, w, h, 0, 0)
        
        # 更新FPS
        self._update_fps()
        
        return output
    
    def _visualize(self, frame: np.ndarray, face, frame_width: int, frame_height: int,
                   delta_base: float, delta_wrist: float) -> np.ndarray:
        """
        绘制可视化信息
        
        Args:
            frame: 原始图像
            face: 检测到的人脸对象或None
            frame_width, frame_height: 画面尺寸
            delta_base, delta_wrist: 当前控制量
            
        Returns:
            绘制后的图像
        """
        output = frame.copy()
        h, w = output.shape[:2]
        
        cx = w // 2
        cy = h // 2
        
        # 绘制画面中心十字线
        color_cross = (0, 255, 0)  # 绿色
        cv2.line(output, (cx, cy - 20), (cx, cy + 20), color_cross, 2)
        cv2.line(output, (cx - 20, cy), (cx + 20, cy), color_cross, 2)
        
        # 绘制死区
        dead = self.config.dead_zone_pixels
        cv2.rectangle(output, (cx - dead, cy - dead), (cx + dead, cy + dead), 
                     (0, 100, 0), 1)
        
        if face:
            # 绘制人脸框
            x1, y1, x2, y2 = face.bbox
            cv2.rectangle(output, (x1, y1), (x2, y2), (255, 0, 255), 2)
            
            # 绘制人脸中心
            fx, fy = face.center
            cv2.circle(output, (fx, fy), 5, (0, 255, 255), -1)
            
            # 绘制连线
            cv2.line(output, (cx, cy), (fx, fy), (255, 255, 0), 1)
            
            # 绘制信息
            info = f"Face: ({fx},{fy}) conf={face.confidence:.2f}"
            cv2.putText(output, info, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        # 绘制状态信息
        status_text = [
            f"State: {self.state.value}",
            f"FPS: {self.current_fps:.1f}",
            f"base: {self.arm_state['base']:.1f} deg",
            f"wrist_flex: {self.arm_state['wrist_flex']:.1f} deg",
            f"d_base: {delta_base:+.2f}",
            f"d_wrist: {delta_wrist:+.2f}",
        ]
        
        y_offset = 25
        for text in status_text:
            cv2.putText(output, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            y_offset += 22
        
        return output
    
    def run(self, display: bool = False):
        """
        主循环
        
        Args:
            display: 是否显示调试窗口
        """
        if not self.initialize():
            logger.error("初始化失败，无法运行")
            return
        
        # 启动复位
        logger.info("发送机械臂复位指令...")
        self._send_arm_command(self.arm_state, speed=self.config.arm_speed)
        time.sleep(0.5)  # 给舵机一点时间开始运动
        
        self.running = True
        self._stop_event.clear()
        self._startup_time = time.time()
        
        logger.info("=" * 60)
        logger.info("人脸跟踪已启动")
        logger.info("控制关节: base (左右), wrist_flex (上下)")
        logger.info(f"启动搜索超时: {self.config.startup_search_timeout} 秒")
        logger.info(f"扫描范围: ±{self.config.search_angle_range}°, 扫描速度: {self.config.search_speed}")
        logger.info("按 Ctrl+C 停止")
        logger.info("=" * 60)
        
        try:
            while self.running and not self._stop_event.is_set():
                # 读取图像
                frame_id, frame = self.vision_sub.read_frame()
                
                if frame is None:
                    time.sleep(0.01)
                    continue
                
                # 处理帧
                output = self._process_frame(frame)
                
                # 显示（调试用）
                if display and output is not None:
                    cv2.imshow("Face Tracking", output)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                        
        except KeyboardInterrupt:
            logger.info("收到中断信号")
        except Exception as e:
            logger.error(f"运行异常: {e}")
            self.state = TrackerState.ERROR
        finally:
            self.stop()
            if display:
                cv2.destroyAllWindows()
    
    def stop(self):
        """停止应用"""
        logger.info("停止人脸跟踪应用")
        self.running = False
        self._stop_event.set()
        
        # 机械臂复位
        if self.arm_client:
            logger.info("发送机械臂复位指令...")
            self.arm_state["base"] = self.config.initial_base
            self.arm_state["wrist_flex"] = self.config.initial_wrist_flex
            self.arm_state["shoulder"] = self.config.initial_shoulder
            self.arm_state["elbow"] = self.config.initial_elbow
            self.arm_state["wrist_roll"] = self.config.initial_wrist_roll
            self.arm_state["gripper"] = self.config.initial_gripper
            self._send_arm_command(self.arm_state, speed=self.config.arm_speed)
            time.sleep(0.3)  # 给舵机一点时间开始运动
        
        # 释放资源
        if self.vision_sub:
            self.vision_sub.stop()
        if self.arm_client:
            self.arm_client.close()
        if self.detector:
            self.detector.release()
        
        logger.info("应用已停止")


def main():
    """主入口"""
    import argparse
    
    parser = argparse.ArgumentParser(description='机械臂人脸跟踪应用')
    parser.add_argument('--display', '-d', action='store_true',
                       help='显示调试窗口')
    parser.add_argument('--vision', default='tcp://localhost:5560',
                       help='视觉服务地址')
    parser.add_argument('--arm', default='tcp://localhost:5557',
                       help='机械臂服务地址')
    parser.add_argument('--kp-base', type=float, default=0.01,
                       help='水平方向比例系数 (默认: 0.01)')
    parser.add_argument('--kp-wrist', type=float, default=0.01,
                       help='垂直方向比例系数 (默认: 0.01)')
    parser.add_argument('--max-step', type=float, default=5.0,
                       help='每帧最大角度变化 (默认: 5.0度)')
    parser.add_argument('--dead-zone', type=int, default=20,
                       help='画面中心死区像素 (默认: 20)')
    parser.add_argument('--device', default='cpu',
                       help='推理设备: cpu/cuda/mps (默认: cpu)')
    parser.add_argument('--search-speed', type=int, default=300,
                       help='搜索扫描舵机速度，数值越小越慢 (默认: 300)')
    parser.add_argument('--search-range', type=float, default=90.0,
                       help='搜索扫描角度范围 (默认: 90.0度)')
    parser.add_argument('--search-interval', type=float, default=4.0,
                       help='扫描方向切换间隔 (默认: 4.0秒)')
    parser.add_argument('--startup-timeout', type=float, default=3.0,
                       help='启动后等待人脸超时时间 (默认: 3.0秒)')
    
    args = parser.parse_args()
    
    config = FaceTrackerConfig(
        vision_sub_addr=args.vision,
        arm_service_addr=args.arm,
        kp_base=args.kp_base,
        kp_wrist=args.kp_wrist,
        max_base_step=args.max_step,
        max_wrist_step=args.max_step,
        dead_zone_pixels=args.dead_zone,
        device=args.device,
        search_speed=args.search_speed,
        search_angle_range=args.search_range,
        search_switch_interval=args.search_interval,
        startup_search_timeout=args.startup_timeout,
    )
    
    app = FaceTrackerApp(config=config)
    app.run(display=args.display)


if __name__ == "__main__":
    main()
