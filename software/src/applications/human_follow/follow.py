"""
人体跟随主应用
整合检测、跟踪、控制和底盘通信
"""
import time
import threading
from typing import Optional
from dataclasses import dataclass
from enum import Enum

import numpy as np

from common.logging import get_logger
from configs import get_config, HumanFollowConfig
from services.vision_service import VisionSubscriber

from .detector import HumanDetector
from .tracker import TargetTracker
from .controller import FollowController, VelocityCommand

logger = get_logger(__name__)


class FollowMode(Enum):
    """跟随模式"""
    IDLE = "idle"           # 空闲
    FOLLOWING = "following" # 跟随中
    SEARCHING = "searching" # 搜索中
    PAUSED = "paused"       # 暂停
    ERROR = "error"         # 错误


@dataclass
class FollowStatus:
    """跟随状态"""
    mode: FollowMode
    target_id: Optional[int]
    target_confidence: float
    velocity: VelocityCommand
    fps: float
    error_message: Optional[str] = None


class HumanFollowApp:
    """
    人体跟随主应用
    
    功能：
    1. 订阅视觉服务的图像流
    2. 检测并跟踪人体目标
    3. 计算跟随速度
    4. 发送控制指令到底盘服务
    """
    
    def __init__(self, config: Optional[HumanFollowConfig] = None):
        """
        初始化跟随应用
        
        Args:
            config: 跟随配置，默认从全局配置获取
        """
        self.config = config or get_config().human_follow
        
        # 组件
        self.detector: Optional[HumanDetector] = None
        self.tracker: Optional[TargetTracker] = None
        self.controller: Optional[FollowController] = None
        self.vision_sub: Optional[VisionSubscriber] = None
        self.chassis_client = None  # 底盘客户端
        
        # 状态
        self.mode = FollowMode.IDLE
        self.running = False
        self._stop_event = threading.Event()
        
        # 性能统计
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0
        
        # 当前速度（用于平滑）
        self.current_velocity = VelocityCommand(0.0, 0.0, 0.0)
        
        logger.info("HumanFollowApp初始化完成")
    
    def initialize(self) -> bool:
        """初始化所有组件"""
        logger.info("=" * 60)
        logger.info("初始化人体跟随应用")
        logger.info("=" * 60)
        
        try:
            # 1. 初始化视觉订阅
            logger.info("连接视觉服务...")
            self.vision_sub = VisionSubscriber(self.config.vision_sub_addr)
            self.vision_sub.start()  # 启动后台接收线程
            logger.info(f"✓ 视觉订阅已连接: {self.config.vision_sub_addr}")
            
            # 2. 初始化检测器
            logger.info("加载检测模型...")
            self.detector = HumanDetector(
                model_path=self.config.model_path,
                conf_threshold=self.config.conf_threshold,
                inference_size=self.config.inference_size,
                use_half=self.config.use_half_precision
            )
            if not self.detector.initialize():
                logger.error("✗ 检测器初始化失败")
                return False
            logger.info("✓ 检测器已加载")
            
            # 3. 初始化跟踪器
            logger.info("初始化跟踪器...")
            self.tracker = TargetTracker(
                max_age=self.config.max_tracking_age,
                min_iou=self.config.min_iou_threshold,
                selection_strategy=self.config.target_selection
            )
            logger.info("✓ 跟踪器已初始化")
            
            # 4. 初始化控制器
            logger.info("初始化控制器...")
            # 从配置获取相机分辨率
            from configs.config import get_config
            cam_config = get_config().camera
            self.controller = FollowController(
                target_distance=self.config.target_distance,
                kp_linear=self.config.kp_linear,
                kp_angular=self.config.kp_angular,
                max_linear_speed=self.config.max_linear_speed,
                max_angular_speed=self.config.max_angular_speed,
                dead_zone_x=self.config.dead_zone_x,
                dead_zone_area=self.config.dead_zone_area,
                frame_width=cam_config.width,
                frame_height=cam_config.height
            )
            logger.info("✓ 控制器已初始化")
            
            # 5. 初始化底盘客户端
            logger.info("连接底盘服务...")
            from services.motion_service.chassis_arbiter import ChassisArbiterClient
            self.chassis_client = ChassisArbiterClient(self.config.chassis_service_addr)
            logger.info(f"✓ 底盘客户端已连接: {self.config.chassis_service_addr}")
            
            logger.info("=" * 60)
            logger.info("初始化完成")
            logger.info("=" * 60)
            return True
            
        except Exception as e:
            logger.error(f"初始化失败: {e}")
            self.mode = FollowMode.ERROR
            return False
    
    def _update_fps(self):
        """更新FPS统计"""
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed >= 1.0:
            self.current_fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def _send_velocity(self, velocity: VelocityCommand) -> bool:
        """
        发送速度指令到底盘
        
        Args:
            velocity: 速度指令
            
        Returns:
            bool: 是否成功
        """
        if self.chassis_client is None:
            return False
        
        try:
            response = self.chassis_client.send_command(
                vx=velocity.vx,
                vy=velocity.vy,
                vz=velocity.vz,
                source="auto",      # 自动跟随控制源
                priority=3          # 优先级3（auto）
            )
            return response.success if response else False
        except Exception as e:
            logger.warning(f"发送速度指令失败: {e}")
            return False
    
    def _process_frame(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """
        处理单帧图像
        
        Args:
            frame: 输入图像
            
        Returns:
            处理后的图像（调试用）
        """
        if frame is None:
            return None
        
        # 1. 检测人体
        detections = self.detector.detect(frame)
        
        # 2. 更新跟踪
        target = self.tracker.update(detections)
        
        # 3. 状态机处理
        if target:
            # 目标存在
            if self.mode in (FollowMode.FOLLOWING, FollowMode.SEARCHING):
                # 从搜索恢复或继续跟随
                if self.mode == FollowMode.SEARCHING:
                    logger.info("目标重新出现，恢复跟随")
                    self.mode = FollowMode.FOLLOWING
                    # 重置 controller 的丢失计数
                    self.controller.target_lost_count = 0
                
                # 计算跟随速度
                cmd = self.controller.compute_velocity(target)
                if cmd:
                    self.current_velocity = self.controller.smooth_velocity(
                        self.current_velocity, cmd, alpha=0.3
                    )
                    self._send_velocity(self.current_velocity)
                    
            elif self.mode == FollowMode.PAUSED:
                # 暂停模式，保持停止
                self._send_velocity(VelocityCommand(0.0, 0.0, 0.0))
            # IDLE 模式下不发送指令
            
        else:
            # 目标丢失
            self.controller.compute_velocity(None)
            
            if self.mode == FollowMode.FOLLOWING:
                if self.controller.is_searching() and self.config.search_on_lost:
                    # 进入搜索模式
                    self.mode = FollowMode.SEARCHING
                    logger.info("目标丢失，开始搜索")
                    
                elif self.controller.is_target_lost() or self.config.stop_on_lost:
                    # 完全丢失，停止
                    self.current_velocity = self.controller.smooth_velocity(
                        self.current_velocity,
                        VelocityCommand(0.0, 0.0, 0.0),
                        alpha=0.5
                    )
                    self._send_velocity(self.current_velocity)
                    
                    if self.controller.is_target_lost():
                        logger.info("目标完全丢失，停止跟随")
                        self.mode = FollowMode.IDLE
                        
            elif self.mode == FollowMode.SEARCHING:
                # 搜索中继续旋转
                if self.controller.is_target_lost():
                    # 搜索超时，停止
                    logger.info("搜索超时，停止")
                    self.mode = FollowMode.IDLE
                    self.current_velocity = VelocityCommand(0.0, 0.0, 0.0)
                    self._send_velocity(self.current_velocity)
                else:
                    # 继续搜索旋转
                    search_cmd = self.controller.compute_search_velocity()
                    self.current_velocity = self.controller.smooth_velocity(
                        self.current_velocity, search_cmd, alpha=0.3
                    )
                    self._send_velocity(self.current_velocity)
        
        # 更新FPS
        self._update_fps()
        
        # 返回可视化图像（可选）
        return self._visualize(frame, target, detections)
    
    def _visualize(self, frame: np.ndarray, target, detections) -> np.ndarray:
        """
        绘制可视化信息
        
        Args:
            frame: 原始图像
            target: 当前跟踪目标
            detections: 所有检测
            
        Returns:
            绘制后的图像
        """
        import cv2
        
        output = frame.copy()
        h, w = output.shape[:2]
        
        # 绘制所有检测框（半透明）
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            overlay = output.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 1)
            output = cv2.addWeighted(output, 0.7, overlay, 0.3, 0)
        
        # 绘制主要目标
        if target:
            x1, y1, x2, y2 = target.bbox
            cx, cy = target.center
            
            # 目标框
            cv2.rectangle(output, (x1, y1), (x2, y2), (0, 0, 255), 3)
            
            # 中心点
            cv2.circle(output, (cx, cy), 5, (0, 255, 255), -1)
            
            # 到画面中心的连线
            center_x = self.controller.frame_center_x if self.controller else w // 2
            center_y = self.controller.frame_center_y if self.controller else h // 2
            cv2.line(output, (cx, cy), (center_x, cy), (255, 0, 0), 2)
            cv2.line(output, (cx, cy), (cx, center_y), (255, 0, 0), 2)
            
            # 标签
            label = f"Target {target.id}: {target.confidence:.2f}"
            cv2.putText(output, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # 绘制画面中心
        center_x = self.controller.frame_center_x if self.controller else w // 2
        center_y = self.controller.frame_center_y if self.controller else h // 2
        cv2.drawMarker(output, (center_x, center_y), (0, 255, 0), 
                      cv2.MARKER_CROSS, 20, 2)
        
        # 绘制信息面板
        info_lines = [
            f"Mode: {self.mode.value}",
            f"FPS: {self.current_fps:.1f}",
            f"Velocity: vx={self.current_velocity.vx:+.2f}, vz={self.current_velocity.vz:+.2f}",
            f"Targets: {len(self.tracker.targets) if self.tracker else 0}",
        ]
        
        if target:
            info_lines.append(f"Target ID: {target.id}, Conf: {target.confidence:.2f}")
        
        # 绘制背景
        panel_height = len(info_lines) * 25 + 10
        cv2.rectangle(output, (5, 5), (350, panel_height), (0, 0, 0), -1)
        cv2.rectangle(output, (5, 5), (350, panel_height), (0, 255, 0), 1)
        
        # 绘制文字
        for i, line in enumerate(info_lines):
            cv2.putText(output, line, (10, 25 + i * 22),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        
        return output
    
    def start_following(self):
        """开始跟随"""
        if self.mode == FollowMode.ERROR:
            logger.error("应用处于错误状态，无法开始")
            return False
        
        logger.info("开始跟随模式")
        self.mode = FollowMode.FOLLOWING
        return True
    
    def stop_following(self):
        """停止跟随"""
        logger.info("停止跟随模式")
        self.mode = FollowMode.IDLE
        
        # 发送停止指令
        if self.chassis_client:
            try:
                self.chassis_client.send_command(
                    vx=0.0, vy=0.0, vz=0.0,
                    source="auto", priority=3
                )
            except Exception as e:
                logger.warning(f"发送停止指令失败: {e}")
        
        self.current_velocity = VelocityCommand(0.0, 0.0, 0.0)
    
    def pause(self):
        """暂停"""
        logger.info("暂停跟随")
        self.mode = FollowMode.PAUSED
        # 发送停止指令
        if self.chassis_client:
            self.chassis_client.send_command(
                vx=0.0, vy=0.0, vz=0.0,
                source="auto", priority=3
            )
    
    def resume(self):
        """恢复"""
        logger.info("恢复跟随")
        self.mode = FollowMode.FOLLOWING
    
    def run(self, display: bool = False):
        """
        主循环
        
        Args:
            display: 是否显示调试窗口
        """
        if not self.initialize():
            logger.error("初始化失败，无法运行")
            return
        
        self.running = True
        self._stop_event.clear()
        
        # 自动开始跟随
        self.start_following()
        
        logger.info("=" * 60)
        logger.info("人体跟随已启动")
        logger.info("按 Ctrl+C 停止")
        logger.info("=" * 60)
        
        try:
            while self.running and not self._stop_event.is_set():
                # 读取图像
                frame_id, frame = self.vision_sub.read_frame()
                
                if frame is None:
                    logger.warning("未收到图像帧")
                    time.sleep(0.01)
                    continue
                
                # 处理帧
                output = self._process_frame(frame)
                
                # 显示（调试用）
                if display and output is not None:
                    import cv2
                    cv2.imshow("Human Follow", output)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
        except KeyboardInterrupt:
            logger.info("收到中断信号")
        except Exception as e:
            logger.error(f"运行异常: {e}")
            self.mode = FollowMode.ERROR
        finally:
            self.stop()
            if display:
                import cv2
                cv2.destroyAllWindows()
    
    def stop(self):
        """停止应用"""
        logger.info("停止人体跟随应用")
        self.running = False
        self._stop_event.set()
        
        # 停止底盘
        self.stop_following()
        
        # 释放资源
        if self.vision_sub:
            self.vision_sub.stop()
        if self.detector:
            self.detector.release()
        
        logger.info("应用已停止")
    
    def get_status(self) -> FollowStatus:
        """获取当前状态"""
        target = self.tracker.get_primary_target() if self.tracker else None
        return FollowStatus(
            mode=self.mode,
            target_id=target.id if target else None,
            target_confidence=target.confidence if target else 0.0,
            velocity=self.current_velocity,
            fps=self.current_fps
        )


# 入口函数
def main():
    """主入口"""
    import argparse
    
    parser = argparse.ArgumentParser(description='人体跟随应用')
    parser.add_argument('--display', '-d', action='store_true',
                       help='显示调试窗口')
    parser.add_argument('--config', '-c', type=str,
                       help='配置文件路径（可选）')
    
    args = parser.parse_args()
    
    # 创建并运行应用
    app = HumanFollowApp()
    app.run(display=args.display)


if __name__ == "__main__":
    main()
