"""
跟随控制器
实现视觉伺服控制算法，根据目标位置计算底盘速度
"""
from typing import Tuple, Optional
from dataclasses import dataclass
import time

from common.logging import get_logger
from .tracker import Target

logger = get_logger(__name__)


@dataclass
class VelocityCommand:
    """速度指令"""
    vx: float      # 线速度 X (m/s) - 前进/后退
    vy: float      # 线速度 Y (m/s) - 左右平移（全向底盘）
    vz: float      # 角速度 Z (rad/s) - 旋转
    
    def __post_init__(self):
        """确保速度值合理"""
        self.vx = float(self.vx)
        self.vy = float(self.vy)
        self.vz = float(self.vz)
    
    def __repr__(self):
        return f"Velocity(vx={self.vx:+.3f}, vy={self.vy:+.3f}, vz={self.vz:+.3f})"


class FollowController:
    """
    跟随控制器
    
    使用视觉伺服（Visual Servoing）控制算法
    目标：保持目标在画面中央，维持固定距离
    """
    
    def __init__(self,
                 target_distance: float = 1.0,      # 目标距离（米）
                 kp_linear: float = 0.001,          # 线速度P系数
                 ki_linear: float = 0.0,            # 线速度I系数
                 kd_linear: float = 0.0,            # 线速度D系数
                 kp_angular: float = 0.003,         # 角速度P系数
                 ki_angular: float = 0.0,           # 角速度I系数
                 kd_angular: float = 0.0,           # 角速度D系数
                 max_linear_speed: float = 0.3,     # 最大线速度 (m/s)
                 max_angular_speed: float = 0.8,    # 最大角速度 (rad/s)
                 dead_zone_x: int = 50,             # 水平死区（像素）
                 dead_zone_area: float = 0.1,       # 面积死区（相对值）
                 frame_width: int = 640,            # 画面宽度
                 frame_height: int = 480):          # 画面高度
        """
        初始化控制器
        
        Args:
            target_distance: 期望保持的目标距离（通过目标面积估算）
            kp_linear: 线速度比例系数（误差归一化到-1~1）
            ki_linear: 线速度积分系数
            kd_linear: 线速度微分系数
            kp_angular: 角速度比例系数（误差归一化到-1~1）
            ki_angular: 角速度积分系数
            kd_angular: 角速度微分系数
            max_linear_speed: 最大线速度限制
            max_angular_speed: 最大角速度限制
            dead_zone_x: 水平方向死区（像素），内部会转换为相对值
            dead_zone_area: 面积死区（相对比例，如0.1表示10%）
            frame_width: 画面宽度
            frame_height: 画面高度
        """
        # PID参数
        self.kp_linear = kp_linear
        self.ki_linear = ki_linear
        self.kd_linear = kd_linear
        self.kp_angular = kp_angular
        self.ki_angular = ki_angular
        self.kd_angular = kd_angular
        
        # 限制参数
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        
        # 死区参数
        self.dead_zone_x = dead_zone_x
        self.dead_zone_area = dead_zone_area
        
        # 画面尺寸
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.frame_center_x = frame_width // 2
        self.frame_center_y = frame_height // 2
        
        # 目标距离参数（米）
        self.target_distance = target_distance
        # 根据目标距离计算期望的目标面积
        # 假设：在 reference_distance（1米）时，reference_area 是参考面积
        # 目标面积与距离平方成反比
        reference_distance = 1.0  # 参考距离 1 米
        # 在 1 米处，假设人体占画面约 50% 宽 × 80% 高（更合理的比例）
        reference_area = frame_width * 0.5 * frame_height * 0.8
        # 根据目标距离反推期望面积
        self.target_area = reference_area * (reference_distance / target_distance) ** 2
        
        # PID状态
        self.error_x_integral = 0.0
        self.error_x_prev = 0.0
        self.error_area_integral = 0.0
        self.error_area_prev = 0.0
        
        # 时间
        self.last_time = time.time()
        
        # 状态
        self.target_lost_count = 0
        self.max_lost_count = 60  # 约2秒@30fps
        self.is_initialized = False
        
        logger.info(f"FollowController初始化:")
        logger.info(f"  目标距离: {target_distance}m")
        logger.info(f"  参考面积(1m): {reference_area:.0f}px")
        logger.info(f"  目标面积({target_distance}m): {self.target_area:.0f}px")
        logger.info(f"  线速度PID: P={kp_linear}, I={ki_linear}, D={kd_linear}")
        logger.info(f"  角速度PID: P={kp_angular}, I={ki_angular}, D={kd_angular}")
        logger.info(f"  速度限制: linear={max_linear_speed}, angular={max_angular_speed}")
        dead_zone_x_rel = dead_zone_x / (frame_width / 2) * 100
        logger.info(f"  死区: x={dead_zone_x}px ({dead_zone_x_rel:.1f}%), area={dead_zone_area*100:.0f}%")
    
    def reset(self):
        """重置控制器状态"""
        self.error_x_integral = 0.0
        self.error_x_prev = 0.0
        self.error_area_integral = 0.0
        self.error_area_prev = 0.0
        self.target_lost_count = 0
        self.is_initialized = False
        logger.info("FollowController已重置")
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """限制数值范围"""
        return max(min_val, min(value, max_val))
    
    def _compute_pid(self, error: float, error_prev: float, error_integral: float,
                     kp: float, ki: float, kd: float, dt: float) -> Tuple[float, float]:
        """
        计算PID输出
        
        Returns:
            (output, new_integral)
        """
        # 积分
        new_integral = error_integral + error * dt
        # 积分限幅
        new_integral = self._clamp(new_integral, -10.0, 10.0)
        
        # 微分
        derivative = (error - error_prev) / dt if dt > 0 else 0.0
        
        # PID计算
        output = kp * error + ki * new_integral + kd * derivative
        
        return output, new_integral
    
    def compute_velocity(self, target: Target) -> Optional[VelocityCommand]:
        """
        根据目标位置计算跟随速度
        
        Args:
            target: 跟踪目标
            
        Returns:
            VelocityCommand: 速度指令，如果目标无效返回None
        """
        if target is None:
            self.target_lost_count += 1
            if self.target_lost_count > self.max_lost_count:
                # 目标长期丢失，停止
                return VelocityCommand(0.0, 0.0, 0.0)
            # 暂时丢失，保持之前的速度（可由上层处理）
            return None
        
        # 目标存在，重置丢失计数
        self.target_lost_count = 0
        self.is_initialized = True
        
        # 计算时间差
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 限制dt避免异常值
        dt = min(dt, 0.1)  # 最大100ms
        
        # 获取目标信息
        cx, cy = target.center
        area = target.area
        
        # ========== 水平控制（角速度 vz）==========
        # 计算水平误差（归一化到 -1.0 ~ 1.0，避免分辨率影响）
        error_x = (cx - self.frame_center_x) / self.frame_center_x
        
        # 应用死区（归一化值）
        dead_zone_norm = self.dead_zone_x / self.frame_center_x
        if abs(error_x) < dead_zone_norm:
            error_x = 0.0
        
        # PID计算
        vz, self.error_x_integral = self._compute_pid(
            error_x, self.error_x_prev, self.error_x_integral,
            self.kp_angular, self.ki_angular, self.kd_angular, dt
        )
        self.error_x_prev = error_x
        
        # 角速度方向：目标在左(error_x负)→需要右转(vz负)才能对准目标
        # 目标在右(error_x正)→需要左转(vz正)才能对准目标
        # 所以vz与error_x同号
        vz = vz  # 直接使用PID输出
        
        # ========== 距离控制（线速度 vx）==========
        # 计算面积误差（归一化到 -1.0 ~ 1.0，目标面积 vs 实际面积）
        # error > 0: 目标太小（距离太远），需要靠近
        # error < 0: 目标太大（距离太近），需要后退
        if area > 0:
            error_area = (self.target_area - area) / self.target_area
        else:
            error_area = 0.0
        
        # 应用死区
        if abs(error_area) < self.dead_zone_area:
            error_area = 0.0
        
        # PID计算
        vx, self.error_area_integral = self._compute_pid(
            error_area, self.error_area_prev, self.error_area_integral,
            self.kp_linear, self.ki_linear, self.kd_linear, dt
        )
        self.error_area_prev = error_area
        
        # 线速度直接使用PID输出（已归一化）
        # error_area > 0（目标小/远）→ vx > 0（前进）
        # error_area < 0（目标大/近）→ 不后退，保持静止
        
        # ========== 速度限制 ==========
        # 只限制前进速度，禁止后退（vx >= 0）
        vx = self._clamp(vx, 0.0, self.max_linear_speed)
        vz = self._clamp(vz, -self.max_angular_speed, self.max_angular_speed)
        
        # vy = 0（不使用左右平移）
        vy = 0.0
        
        return VelocityCommand(vx, vy, vz)
    
    def compute_search_velocity(self) -> VelocityCommand:
        """
        计算搜索速度
        当目标丢失时原地旋转搜索
        
        Returns:
            VelocityCommand: 旋转速度指令
        """
        # 原地旋转，速度适中
        search_vz = self.max_angular_speed * 0.5
        return VelocityCommand(0.0, 0.0, search_vz)
    
    def is_target_lost(self) -> bool:
        """判断目标是否丢失"""
        return self.target_lost_count > self.max_lost_count
    
    def is_searching(self) -> bool:
        """是否正在搜索目标"""
        return 0 < self.target_lost_count <= self.max_lost_count
    
    def get_status(self) -> dict:
        """获取控制器状态"""
        return {
            "initialized": self.is_initialized,
            "target_lost_count": self.target_lost_count,
            "target_lost": self.is_target_lost(),
            "searching": self.is_searching(),
            "error_x_integral": self.error_x_integral,
            "error_area_integral": self.error_area_integral,
            "target_area": self.target_area,
            "frame_center": (self.frame_center_x, self.frame_center_y)
        }
    
    def smooth_velocity(self, current: VelocityCommand, 
                       target: VelocityCommand,
                       alpha: float = 0.3) -> VelocityCommand:
        """
        速度平滑（低通滤波）
        
        Args:
            current: 当前速度
            target: 目标速度
            alpha: 平滑系数 (0-1)，越小越平滑
            
        Returns:
            VelocityCommand: 平滑后的速度
        """
        vx = alpha * target.vx + (1 - alpha) * current.vx
        vy = alpha * target.vy + (1 - alpha) * current.vy
        vz = alpha * target.vz + (1 - alpha) * current.vz
        return VelocityCommand(vx, vy, vz)


# 测试代码
if __name__ == "__main__":
    import cv2
    import numpy as np
    from .detector import HumanDetector
    from .tracker import TargetTracker
    
    # 初始化
    detector = HumanDetector(model_path="models/yolo26n.pt")
    tracker = TargetTracker(selection_strategy="center")
    controller = FollowController(
        target_distance=1.0,
        kp_linear=0.001,
        kp_angular=0.003,
        max_linear_speed=0.3,
        max_angular_speed=0.5
    )
    
    if not detector.initialize():
        logger.error("检测器初始化失败")
        exit(1)
    
    # 测试摄像头
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    current_velocity = VelocityCommand(0.0, 0.0, 0.0)
    
    logger.info("按 'q' 退出测试，按 'r' 重置")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 检测和跟踪
        detections = detector.detect(frame)
        target = tracker.update(detections)
        
        # 计算速度
        if target:
            cmd = controller.compute_velocity(target)
            if cmd:
                # 速度平滑
                current_velocity = controller.smooth_velocity(current_velocity, cmd, alpha=0.3)
        else:
            # 目标丢失，搜索或停止
            if controller.is_searching():
                search_cmd = controller.compute_search_velocity()
                current_velocity = controller.smooth_velocity(current_velocity, search_cmd, alpha=0.3)
            else:
                # 完全丢失，减速停止
                current_velocity = controller.smooth_velocity(
                    current_velocity, 
                    VelocityCommand(0.0, 0.0, 0.0), 
                    alpha=0.1
                )
        
        # 绘制
        output = frame.copy()
        
        # 绘制目标
        if target:
            x1, y1, x2, y2 = target.bbox
            cx, cy = target.center
            cv2.rectangle(output, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.circle(output, (cx, cy), 5, (0, 255, 0), -1)
            cv2.line(output, (cx, cy), (controller.frame_center_x, cy), (255, 0, 0), 2)
        
        # 绘制画面中心
        cv2.drawMarker(output, (controller.frame_center_x, controller.frame_center_y),
                      (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        
        # 显示速度信息
        status = controller.get_status()
        info_lines = [
            f"Velocity: vx={current_velocity.vx:+.2f}, vz={current_velocity.vz:+.2f}",
            f"Status: {'Lost' if status['target_lost'] else 'Searching' if status['searching'] else 'Tracking'}",
            f"Targets: {len(tracker.get_all_targets())}"
        ]
        
        for i, line in enumerate(info_lines):
            cv2.putText(output, line, (10, 30 + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow("Follow Controller", output)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            tracker.reset()
            controller.reset()
            current_velocity = VelocityCommand(0.0, 0.0, 0.0)
            logger.info("已重置")
    
    cap.release()
    cv2.destroyAllWindows()
