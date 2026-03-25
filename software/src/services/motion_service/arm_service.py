"""
机械臂运动控制服务
基于底盘服务的架构实现，使用共享串口总线
"""
import sys
import os
import time
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, asdict, field
from threading import Lock

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

import zmq

from hal.arm.driver import ArmDriver, ArmConfig as HalArmConfig
from configs import get_config
from .servo_bus_manager import get_servo_bus


def create_arm_config_from_global() -> HalArmConfig:
    """从全局配置创建机械臂驱动配置"""
    global_config = get_config()
    arm_cfg = global_config.arm
    
    # 创建 HAL 层 ArmConfig
    return HalArmConfig(
        # 舵机ID映射 (1-6号关节)
        joint_ids={
            "base": arm_cfg.base_id,           # J1
            "shoulder": arm_cfg.shoulder_id,   # J2
            "elbow": arm_cfg.elbow_id,         # J3
            "wrist_flex": arm_cfg.wrist_flex_id,  # J4
            "wrist_roll": arm_cfg.wrist_roll_id,  # J5
            "gripper": arm_cfg.gripper_id,     # J6
        },
        # 角度限制
        joint_limits=getattr(arm_cfg, 'joint_limits', {
            "base": (-180, 180),
            "shoulder": (-90, 90),
            "elbow": (-120, 120),
            "wrist_flex": (-90, 90),
            "wrist_roll": (-180, 180),
            "gripper": (0, 90),
        }),
        # 默认速度/加速度
        default_speed=getattr(arm_cfg, 'default_speed', 1000),
        default_acc=getattr(arm_cfg, 'default_acc', 50),
        # 初始位置（使用休息位置）
        home_position=getattr(arm_cfg, 'rest_position', {
            "base": 0,
            "shoulder": -30,
            "elbow": 90,
            "wrist_flex": 0,
            "wrist_roll": 0,
            "gripper": 45,
        }),
        # 串口配置
        port=arm_cfg.serial_port,
        baudrate=arm_cfg.baudrate,
    )


@dataclass
class ArmCommand:
    """机械臂指令数据结构"""
    joint_angles: Dict[str, float]  # {joint_name: angle, ...}
    speed: int                      # 运动速度
    source: str                     # 控制源
    priority: int                   # 优先级
    timestamp: float                # 时间戳
    query: bool = False             # True 表示仅查询状态，不执行运动


@dataclass
class ArmResponse:
    """机械臂服务响应"""
    success: bool
    message: str
    current_owner: str
    current_priority: int
    joint_states: Optional[Dict[str, float]] = None


# 控制源优先级
PRIORITIES = {
    "emergency": 4,
    "auto": 3,
    "voice": 2,
    "web": 1,
}


class ArmService:
    """
    机械臂运动控制服务
    - ZeroMQ REP 模式监听控制指令
    - 优先级-based 控制权管理
    - 使用共享串口总线
    """
    
    TIMEOUT_MS = 2000  # 机械臂指令超时时间稍长
    
    # 关节名称映射（1-6号关节）
    JOINT_NAMES = {
        1: "base",        # 基座旋转
        2: "shoulder",    # 肩关节
        3: "elbow",       # 肘关节
        4: "wrist_flex",  # 腕关节屈伸
        5: "wrist_roll",  # 腕关节旋转
        6: "gripper",     # 夹爪
    }
    
    # 关节ID映射（从名称到ID）
    JOINT_IDS = {v: k for k, v in JOINT_NAMES.items()}
    
    def __init__(self, rep_addr: Optional[str] = None):
        # 从配置读取地址
        config = get_config()
        self.rep_addr = rep_addr or config.zmq.arm_service_addr
        
        # 创建机械臂驱动配置（从全局配置转换）
        self._arm_config = create_arm_config_from_global()
        self._bus = None  # 延迟到 start() 时再获取
        self.arm = None   # 延迟初始化
        
        # 控制权状态
        self._current_owner: Optional[str] = None
        self._current_priority: int = 0
        self._last_command_time: float = 0.0
        
        self._lock = Lock()
        self._context: Optional[zmq.Context] = None
        self._rep_socket: Optional[zmq.Socket] = None
        self._running = False
    
    def _check_timeout(self) -> None:
        """检查控制权是否超时"""
        if self._current_owner is None:
            return
            
        elapsed_ms = (time.time() - self._last_command_time) * 1000
        if elapsed_ms > self.TIMEOUT_MS:
            print(f"[ARM_SVC] 控制权超时释放: {self._current_owner}")
            self._current_owner = None
            self._current_priority = 0
    
    def _get_current_joint_states(self) -> Dict[str, float]:
        """获取当前关节状态"""
        if not self.arm or not self.arm._initialized:
            return {}
        
        joint_states = {}
        for joint_name, servo_id in self.arm.config.joint_ids.items():
            try:
                # 读取当前位置
                pos = self.arm.bus.read_position(servo_id)
                if pos is not None:
                    angle = self.arm._pos_to_angle(pos)
                    joint_states[joint_name] = angle
                else:
                    # 读取失败，使用缓存值
                    joint_states[joint_name] = self.arm._current_angles.get(joint_name, 0)
            except Exception as e:
                # 读取失败，使用缓存值
                joint_states[joint_name] = self.arm._current_angles.get(joint_name, 0)
        
        return joint_states
    
    def _arbitrate(self, cmd: ArmCommand) -> ArmResponse:
        """仲裁核心逻辑 - 优化版本，不读取关节状态以减少延迟"""
        # 处理查询请求（只读关节状态，不执行运动）
        if cmd.query:
            joint_states = self._get_current_joint_states()
            return ArmResponse(
                success=True,
                message="查询成功",
                current_owner=self._current_owner or "none",
                current_priority=self._current_priority,
                joint_states=joint_states
            )
        
        with self._lock:
            self._check_timeout()
            
            new_priority = cmd.priority
            
            if self._current_owner is None:
                self._current_owner = cmd.source
                self._current_priority = new_priority
                self._last_command_time = time.time()
                success = self._execute_to_hardware(cmd)
                
                return ArmResponse(
                    success=success,
                    message="指令已接受" if success else "执行失败",
                    current_owner=cmd.source,
                    current_priority=new_priority,
                    joint_states=None  # 不读取关节状态，减少延迟
                )
            
            elif new_priority >= self._current_priority:
                old_owner = self._current_owner
                self._current_owner = cmd.source
                self._current_priority = new_priority
                self._last_command_time = time.time()
                success = self._execute_to_hardware(cmd)
                
                msg = "抢占控制权" if old_owner != cmd.source else "续期控制权"
                return ArmResponse(
                    success=success,
                    message=f"指令已接受（{msg}）" if success else "执行失败",
                    current_owner=cmd.source,
                    current_priority=new_priority,
                    joint_states=None  # 不读取关节状态，减少延迟
                )
            else:
                return ArmResponse(
                    success=False,
                    message=f"优先级不足，当前被 {self._current_owner} 占用",
                    current_owner=self._current_owner,
                    current_priority=self._current_priority
                )
    
    def _execute_to_hardware(self, cmd: ArmCommand) -> bool:
        """执行指令到机械臂硬件 - 使用批量写入优化性能"""
        if not cmd.joint_angles:
            print("[ARM_SVC] [SKIP] 空关节角度指令")
            return True
        
        # 使用批量写入替代逐个写入，性能提升约6倍
        success = self._sync_write_joints(cmd.joint_angles, speed=cmd.speed)
        
        status = "OK" if success else "FAIL"
        angles_str = ", ".join([f"{k}={v:.1f}" for k, v in cmd.joint_angles.items()])
        print(f"[ARM_SVC] [{status}] {angles_str} [from {cmd.source}]")
        return success
    
    def _sync_write_joints(self, joint_angles: Dict[str, float], speed: int) -> bool:
        """
        批量写入关节角度 - 性能优化版本
        使用 sync_write_positions 替代逐个写入
        """
        if not self.arm._initialized:
            return False
        
        # 构建批量写入参数: {servo_id: (position, speed, acc), ...}
        positions = {}
        
        for joint_name, angle in joint_angles.items():
            if joint_name not in self.arm.config.joint_ids:
                continue
            
            # 限制角度范围
            angle = self.arm._clamp_angle(joint_name, angle)
            
            # 获取舵机ID和目标位置
            servo_id = self.arm.config.joint_ids[joint_name]
            position = self.arm._angle_to_pos(angle)
            
            # 添加到批量写入字典
            positions[servo_id] = (position, speed, self.arm.config.default_acc)
            
            # 更新缓存
            self.arm._current_angles[joint_name] = angle
        
        if not positions:
            return True
        
        # 批量写入所有舵机（只有一次串口通信）
        return self.arm.bus.sync_write_positions(positions)
    
    def _parse_request(self, data: Dict[str, Any]) -> Optional[ArmCommand]:
        """解析REQ请求数据"""
        try:
            source = data.get("source", "")
            priority = int(data.get("priority", 0))
            speed = int(data.get("speed", 0))
            
            # 支持三种格式：
            # 1. 1-6号关节角度数组: {"joints": [0, 45, 90, 0, 0, 30]}
            # 2. 关节名称字典: {"joints": {"base": 0, "shoulder": 45, ...}}
            # 3. 兼容旧格式: {"j1": 0, "j2": 45, ...} 或 {"base": 0, ...}
            
            joint_angles = {}
            joints_data = data.get("joints", None)
            
            query = data.get("query", False)
            
            if joints_data is not None:
                if isinstance(joints_data, list):
                    # 数组格式，按索引映射到关节名
                    for i, angle in enumerate(joints_data[:6], 1):
                        if i in self.JOINT_NAMES:
                            joint_angles[self.JOINT_NAMES[i]] = float(angle)
                elif isinstance(joints_data, dict):
                    if joints_data == {} and not query:
                        # 空字典且非查询模式 = 移动到 home 位置
                        joint_angles = self._arm_config.home_position
                    elif joints_data == {} and query:
                        # 空字典且查询模式 = 只查询状态
                        joint_angles = {}
                    else:
                        # 字典格式，直接使用
                        joint_angles = {k: float(v) for k, v in joints_data.items()}
            else:
                # 尝试从顶层解析关节角度
                for i in range(1, 7):
                    key = f"j{i}"
                    if key in data:
                        joint_angles[self.JOINT_NAMES[i]] = float(data[key])
                # 也支持关节名称
                for name in ["base", "shoulder", "elbow", "wrist_flex", "wrist_roll", "gripper"]:
                    if name in data:
                        joint_angles[name] = float(data[name])
            
            if priority == 0 and source in PRIORITIES:
                priority = PRIORITIES[source]
            
            return ArmCommand(
                joint_angles=joint_angles,
                speed=speed,
                source=source,
                priority=priority,
                timestamp=time.time(),
                query=query
            )
        except (KeyError, ValueError, TypeError) as e:
            print(f"[ARM_SVC] 解析请求失败: {e}, data={data}")
            return None
    
    def start(self) -> None:
        """启动机械臂服务"""
        print("=" * 60)
        print("HomeBot 机械臂运动控制服务")
        print("=" * 60)
        
        # 延迟初始化 ArmDriver（确保共享总线已准备好）
        if self.arm is None:
            self._bus = get_servo_bus()
            self.arm = ArmDriver(self._arm_config, bus=self._bus)
        
        # 初始化机械臂硬件（使用已连接的共享总线，默认不复位）
        if not self.arm.initialize(auto_home=False):
            print("[ARM_SVC] 机械臂硬件初始化失败，退出")
            return
        
        # 启动ZeroMQ
        self._context = zmq.Context()
        self._rep_socket = self._context.socket(zmq.REP)
        self._rep_socket.setsockopt(zmq.LINGER, 0)
        self._rep_socket.bind(self.rep_addr)
        
        self._running = True
        print(f"[ARM_SVC] 机械臂服务已启动，监听: {self.rep_addr}")
        print("=" * 60)
        
        try:
            while self._running:
                try:
                    request_data = self._rep_socket.recv_json(flags=zmq.NOBLOCK)
                    cmd = self._parse_request(request_data)
                    
                    if cmd is None:
                        response = ArmResponse(
                            success=False,
                            message="请求格式错误",
                            current_owner=self._current_owner or "none",
                            current_priority=self._current_priority
                        )
                    else:
                        response = self._arbitrate(cmd)
                    
                    self._rep_socket.send_json(asdict(response))
                    
                except zmq.Again:
                    with self._lock:
                        self._check_timeout()
                    time.sleep(0.001)
                    continue
                    
        except KeyboardInterrupt:
            print("\n[ARM_SVC] 正在关闭...")
        finally:
            self.stop()
    
    def stop(self) -> None:
        """停止机械臂服务"""
        if not getattr(self, '_stopped', False):
            self._stopped = True
            self._running = False
            # 注意：不关闭共享总线，由底盘服务或主程序管理
            if self._rep_socket:
                self._rep_socket.close()
                self._rep_socket = None
            if self._context:
                self._context.term()
                self._context = None
            print("[ARM_SVC] 已关闭")


def main():
    """主入口"""
    import argparse
    
    parser = argparse.ArgumentParser(description='HomeBot 机械臂运动控制服务')
    parser.add_argument('--addr', default=None, help='ZeroMQ地址')
    
    args = parser.parse_args()
    
    service = ArmService(rep_addr=args.addr)
    service.start()


if __name__ == '__main__':
    main()
