"""
底盘服务
从配置文件读取硬件参数
"""
import os
import sys
import time
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass, asdict
from threading import Lock

import zmq
from .chassis_arbiter import ControlCommand, ArbiterResponse, PRIORITIES
from .servo_bus_manager import ServoBusManager, get_servo_bus
from hal.chassis.driver import ChassisDriver
from hal.battery.driver import BatteryDriver
from common.messages import MessageType, serialize
from configs import get_config, ChassisConfig, BatteryConfig


@dataclass
class ChassisCommand:
    """底盘指令数据结构"""
    vx: float
    vy: float
    vz: float
    source: str
    priority: int
    timestamp: float


class RealChassisController:
    """真实底盘控制器 - 从配置文件读取串口等参数"""
    
    def __init__(self, config: Optional[ChassisConfig] = None, bus=None):
        # 从全局配置读取，或使用传入的配置
        self.config = config or get_config().chassis
        
        # 根据底盘类型选择驱动类
        if self.config.chassis_type == "diff":
            from hal.chassis.diff_driver import DiffChassisDriver
            self.driver = DiffChassisDriver(self.config, bus=bus)
            print(f"[RealChassis] 使用双轮差动底盘驱动 (type=diff)")
        else:
            # 默认使用三轮全向轮底盘
            from hal.chassis.driver import OmniChassisDriver
            self.driver = OmniChassisDriver(self.config, bus=bus)
            print(f"[RealChassis] 使用三轮全向底盘驱动 (type={self.config.chassis_type})")
        
        self._initialized = False
        
    def initialize(self) -> bool:
        """初始化底盘硬件"""
        print("[RealChassis] 正在初始化底盘硬件...")
        print(f"[RealChassis] 串口: {self.config.serial_port}, 波特率: {self.config.baudrate}")
        
        if self._initialized:
            return True
            
        if not self.driver.initialize():
            print("[RealChassis] 底盘初始化失败！")
            return False
            
        self._initialized = True
        print("[RealChassis] 底盘初始化成功")
        return True
    
    def set_velocity(self, vx: float, vy: float, vz: float) -> bool:
        """设置底盘速度"""
        if not self._initialized:
            print("[RealChassis] 错误：底盘未初始化")
            return False
        
        return self.driver.set_velocity(vx, vy, vz)
    
    def get_actual_velocity(self) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        """读取轮子实际转速并计算机器人实际速度"""
        if not self._initialized:
            return (None, None, None)
        return self.driver.get_actual_velocity()
    
    def stop(self) -> None:
        """停止底盘运动"""
        if self._initialized:
            self.driver.stop()
    
    def close(self) -> None:
        """关闭底盘驱动"""
        if self._initialized:
            self.driver.close()
            self._initialized = False


class ChassisService:
    """
    底盘服务 - 合并仲裁器和执行端
    从 configs/config.py 读取硬件配置
    集成电池状态监测和发布
    新增底盘状态发布（速度、控制源）供 OdomService 订阅
    """
    
    TIMEOUT_MS = 1000
    STATE_PUBLISH_INTERVAL = 0.05  # 50Hz 状态发布频率
    
    def __init__(self, 
                 rep_addr: Optional[str] = None, 
                 pub_addr: Optional[str] = None,
                 use_shared_bus: bool = False):
        """
        初始化底盘服务
        
        Args:
            rep_addr: ZeroMQ REP地址，默认从配置文件读取
            pub_addr: 电池状态PUB地址，默认从配置文件读取
            use_shared_bus: 是否使用共享串口总线（与机械臂共用）
        """
        # 从配置文件读取地址
        config = get_config().chassis
        battery_config = get_config().battery
        self.rep_addr = rep_addr or config.service_addr
        self.pub_addr = pub_addr or battery_config.pub_addr
        
        # 电池配置
        self._battery_config = battery_config
        self._battery_publish_interval = battery_config.publish_interval
        
        # 创建真实底盘控制器
        self._servo_bus = None
        if use_shared_bus:
            # 使用共享总线
            self._servo_bus = get_servo_bus()
            self.chassis = RealChassisController(config, bus=self._servo_bus)
        else:
            # 独立模式 - 暂时不创建，初始化时获取bus
            self.chassis = RealChassisController(config)
            self._servo_bus = self.chassis.driver.bus  # 获取底盘的舵机总线用于电池读取
        
        # 创建电池驱动
        self.battery = BatteryDriver(
            servo_bus=self._servo_bus,
            servo_ids=battery_config.servo_ids,
            full_voltage=battery_config.full_voltage,
            low_voltage=battery_config.low_voltage,
            critical_voltage=battery_config.critical_voltage,
            min_voltage=battery_config.min_voltage
        )
        
        # 控制权状态
        self._current_owner: Optional[str] = None
        self._current_priority: int = 0
        self._last_command_time: float = 0.0
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_vz = 0.0
        
        # 紧急停止锁定状态 - 一旦触发，必须通过归位解除
        self._emergency_locked: bool = False
        
        # 电池状态发布
        self._last_battery_publish_time: float = 0.0
        self._last_battery_state = None
        
        # 底盘状态发布
        self._state_pub_addr = getattr(get_config().zmq, 'chassis_state_pub_addr', 'tcp://*:5558')
        self._last_state_publish_time: float = 0.0
        
        self._lock = Lock()
        self._context: Optional[zmq.Context] = None
        self._rep_socket: Optional[zmq.Socket] = None
        self._pub_socket: Optional[zmq.Socket] = None
        self._state_pub_socket: Optional[zmq.Socket] = None
        self._running = False
        
    def _check_timeout(self) -> None:
        """检查当前控制权是否超时"""
        if self._current_owner is None:
            return
            
        elapsed_ms = (time.time() - self._last_command_time) * 1000
        if elapsed_ms > self.TIMEOUT_MS:
            print(f"[CHASSIS_SVC] 控制权超时释放: {self._current_owner}")
            self._current_owner = None
            self._current_priority = 0
            self._last_vx = self._last_vy = self._last_vz = 0.0
            self.chassis.stop()
    
    def _publish_battery_state(self, force: bool = False) -> None:
        """
        发布电池状态
        
        Args:
            force: 是否强制发布（忽略时间间隔）
        """
        current_time = time.time()
        
        # 检查是否需要发布（根据时间间隔）
        if not force:
            elapsed = current_time - self._last_battery_publish_time
            if elapsed < self._battery_publish_interval:
                return
        
        # 读取电池状态
        battery_state = self.battery.read_state()
        self._last_battery_state = battery_state
        
        if battery_state.is_valid and self._pub_socket:
            # 构建消息
            msg_data = {
                "voltage": round(battery_state.voltage, 2),
                "percentage": round(battery_state.percentage, 1),
                "status": battery_state.status.value,
                "temperature": battery_state.temperature,
                "servo_id": battery_state.servo_id
            }
            
            message = serialize(
                msg_type=MessageType.BATTERY_STATE,
                data=msg_data,
                timestamp=current_time
            )
            
            try:
                self._pub_socket.send_json(message, flags=zmq.NOBLOCK)
                self._last_battery_publish_time = current_time
                
                # 只在状态变化或低电量时打印日志
                if (battery_state.status.value in ["low", "critical"] or force):
                    print(f"[CHASSIS_SVC] [BATTERY] {battery_state.voltage:.1f}V "
                          f"({battery_state.percentage:.0f}%) - {battery_state.status.value}")
            except zmq.Again:
                # 发送缓冲区满，忽略
                pass
            except Exception as e:
                print(f"[CHASSIS_SVC] 电池状态发布失败: {e}")
        
        # 低电量警告
        if battery_state.is_valid and self.battery.is_critical_battery():
            print(f"[CHASSIS_SVC] [WARNING] 电池电量严重不足！请立即充电！")
    
    def _publish_chassis_state(self, force: bool = False) -> None:
        """发布底盘状态（命令速度、实际轮速、控制源）供 OdomService 订阅"""
        current_time = time.time()
        if not force and (current_time - self._last_state_publish_time) < self.STATE_PUBLISH_INTERVAL:
            return
        
        if self._state_pub_socket:
            state = {
                "vx": float(self._last_vx),
                "vy": float(self._last_vy),
                "vz": float(self._last_vz),
                "source": self._current_owner or "none",
                "priority": int(self._current_priority),
                "timestamp": current_time,
                "emergency_locked": self._emergency_locked,
            }
            
            # 读取实际轮速并加入状态
            actual_vx, actual_vy, actual_vz = self.chassis.get_actual_velocity()
            if actual_vx is not None:
                state["actual_vx"] = float(round(actual_vx, 4))
                state["actual_vy"] = float(round(actual_vy, 4))
                state["actual_vz"] = float(round(actual_vz, 4))
            
            try:
                self._state_pub_socket.send_json(state, flags=zmq.NOBLOCK)
                self._last_state_publish_time = current_time
            except zmq.Again:
                pass
            except Exception as e:
                print(f"[CHASSIS_SVC] 底盘状态发布失败: {e}")
    
    def _arbitrate(self, cmd: ControlCommand) -> ArbiterResponse:
        """仲裁核心逻辑"""
        with self._lock:
            self._check_timeout()
            
            # 处理紧急停止 - 最高优先级，会锁定底盘
            if cmd.source == "emergency" or (cmd.vx == 0 and cmd.vy == 0 and cmd.vz == 0 and cmd.priority >= 4):
                self._emergency_locked = True
                self._current_owner = "emergency"
                self._current_priority = 4
                self._last_command_time = time.time()
                self._last_vx = self._last_vy = self._last_vz = 0.0
                self.chassis.stop()
                print("[CHASSIS_SVC] [EMERGENCY] 紧急停止已触发，底盘已锁定")
                return ArbiterResponse(
                    success=True,
                    message="紧急停止已触发，底盘已锁定",
                    current_owner="emergency",
                    current_priority=4
                )
            
            # 处理归位命令 - 解除紧急停止锁定
            if cmd.source == "home" or getattr(cmd, 'command', None) == 'home':
                was_locked = self._emergency_locked
                self._emergency_locked = False
                self._current_owner = None
                self._current_priority = 0
                self._last_vx = self._last_vy = self._last_vz = 0.0
                self.chassis.stop()
                msg = "紧急停止已解除，底盘归位" if was_locked else "底盘归位"
                print(f"[CHASSIS_SVC] [HOME] {msg}")
                return ArbiterResponse(
                    success=True,
                    message=msg,
                    current_owner="none",
                    current_priority=0
                )
            
            # 如果处于紧急停止锁定状态，拒绝所有运动命令
            if self._emergency_locked:
                return ArbiterResponse(
                    success=False,
                    message="底盘处于紧急停止锁定状态，请先点击归位解除",
                    current_owner="emergency",
                    current_priority=4
                )
            
            new_priority = cmd.priority
            
            if self._current_owner is None:
                self._current_owner = cmd.source
                self._current_priority = new_priority
                self._last_command_time = time.time()
                self._last_vx, self._last_vy, self._last_vz = cmd.vx, cmd.vy, cmd.vz
                self._execute_to_hardware(cmd)
                
                return ArbiterResponse(
                    success=True,
                    message="指令已接受",
                    current_owner=cmd.source,
                    current_priority=new_priority
                )
            
            elif new_priority >= self._current_priority:
                old_owner = self._current_owner
                self._current_owner = cmd.source
                self._current_priority = new_priority
                self._last_command_time = time.time()
                self._last_vx, self._last_vy, self._last_vz = cmd.vx, cmd.vy, cmd.vz
                self._execute_to_hardware(cmd)
                
                msg = "抢占控制权" if old_owner != cmd.source else "续期控制权"
                return ArbiterResponse(
                    success=True,
                    message=f"指令已接受（{msg}）",
                    current_owner=cmd.source,
                    current_priority=new_priority
                )
            else:
                return ArbiterResponse(
                    success=False,
                    message=f"优先级不足，当前被 {self._current_owner} 占用",
                    current_owner=self._current_owner,
                    current_priority=self._current_priority
                )
    
    def _execute_to_hardware(self, cmd: ControlCommand) -> None:
        """执行指令到底盘硬件"""
        success = self.chassis.set_velocity(cmd.vx, cmd.vy, cmd.vz)
        status = "OK" if success else "FAIL"
        print(f"[CHASSIS_SVC] [{status}] vx={cmd.vx:+.2f}, vz={cmd.vz:+.2f} [from {cmd.source}]")

    def _handle_command(self, cmd: Dict[str, Any]) -> ArbiterResponse:
        """处理非速度控制类命令（如传感器复位）。"""
        command = cmd.get("cmd", "")

        if command == "reset_sensors":
            reset_odom = bool(cmd.get("odom", True))
            reset_imu = bool(cmd.get("imu", True))
            messages: List[str] = []
            success = True

            if reset_odom:
                if hasattr(self.chassis.driver, 'reset_odometry'):
                    if self.chassis.driver.reset_odometry():
                        messages.append("编码器里程计已清零")
                    else:
                        success = False
                        messages.append("编码器里程计清零失败")
                else:
                    messages.append("当前底盘不支持编码器里程计清零")

            if reset_imu:
                if hasattr(self.chassis.driver, 'imu_clear_attitude'):
                    if self.chassis.driver.imu_clear_attitude():
                        messages.append("IMU 姿态已清零")
                    else:
                        success = False
                        messages.append("IMU 姿态清零失败")
                else:
                    messages.append("当前底盘不支持 IMU 清零")

            message = "; ".join(messages) if messages else "无操作"
            return ArbiterResponse(
                success=success,
                message=message,
                current_owner=self._current_owner or "none",
                current_priority=self._current_priority
            )

        return ArbiterResponse(
            success=False,
            message=f"未知命令: {command}",
            current_owner=self._current_owner or "none",
            current_priority=self._current_priority
        )

    def _parse_request(self, data: Dict[str, Any]) -> Optional[Any]:
        """解析REQ请求数据。

        支持两种风格：
        - 速度控制命令：{"source": "web", "vx": 0.5, "vy": 0.0, "vz": 0.3, "priority": 1}
        - 硬件管理命令：{"cmd": "reset_sensors", "odom": true, "imu": true}
        """
        try:
            if "cmd" in data:
                # 硬件管理命令，直接返回 dict
                return data

            # 速度控制命令
            source = data.get("source", "")
            vx = float(data.get("vx", 0.0))
            vy = float(data.get("vy", 0.0))
            vz = float(data.get("vz", 0.0))
            priority = int(data.get("priority", 0))

            if priority == 0 and source in PRIORITIES:
                priority = PRIORITIES[source]

            return ControlCommand(
                source=source, vx=vx, vy=vy, vz=vz,
                priority=priority, timestamp=time.time()
            )
        except (KeyError, ValueError, TypeError) as e:
            print(f"[CHASSIS_SVC] 解析请求失败: {e}")
            return None
    
    def start(self) -> None:
        """启动底盘服务"""
        print("=" * 60)
        print("HomeBot 底盘服务")
        print("=" * 60)
        
        # 从配置读取的信息
        config = get_config().chassis
        battery_config = get_config().battery
        print(f"[配置] 串口: {config.serial_port}")
        print(f"[配置] 波特率: {config.baudrate}")
        print(f"[配置] 最大线速度: {config.max_linear_speed} m/s")
        print(f"[配置] 服务地址: {self.rep_addr}")
        print(f"[配置] 电池发布地址: {self.pub_addr}")
        print(f"[配置] 电池发布间隔: {self._battery_publish_interval}s")
        
        # 初始化底盘硬件
        if not self.chassis.initialize():
            print("[CHASSIS_SVC] 底盘硬件初始化失败，退出")
            return
        
        # 更新电池驱动的舵机总线引用（如果之前是独立模式，现在才获得bus）
        if self._servo_bus is None:
            self._servo_bus = self.chassis.driver.bus
            self.battery.set_servo_bus(self._servo_bus)
        
        # 启动ZeroMQ
        self._context = zmq.Context()
        
        # REP socket用于接收控制命令
        self._rep_socket = self._context.socket(zmq.REP)
        self._rep_socket.setsockopt(zmq.LINGER, 0)
        self._rep_socket.bind(self.rep_addr)
        
        # PUB socket用于发布电池状态
        self._pub_socket = self._context.socket(zmq.PUB)
        self._pub_socket.setsockopt(zmq.LINGER, 0)
        self._pub_socket.bind(self.pub_addr)
        
        # PUB socket用于发布底盘状态（速度、控制源）
        self._state_pub_socket = self._context.socket(zmq.PUB)
        self._state_pub_socket.setsockopt(zmq.LINGER, 0)
        self._state_pub_socket.bind(self._state_pub_addr)
        print(f"[CHASSIS_SVC] 底盘状态发布地址: {self._state_pub_addr}")
        
        self._running = True
        print("[CHASSIS_SVC] 底盘服务已启动，等待控制源连接...")
        print("=" * 60)
        
        # 发布一次初始电池状态
        self._publish_battery_state(force=True)
        
        try:
            while self._running:
                try:
                    request_data = self._rep_socket.recv_json(flags=zmq.NOBLOCK)
                    cmd = self._parse_request(request_data)

                    if cmd is None:
                        response = ArbiterResponse(
                            success=False, message="请求格式错误",
                            current_owner=self._current_owner or "none",
                            current_priority=self._current_priority
                        )
                    elif isinstance(cmd, dict):
                        # 硬件管理命令
                        response = self._handle_command(cmd)
                    else:
                        response = self._arbitrate(cmd)

                    self._rep_socket.send_json(asdict(response))
                    
                except zmq.Again:
                    with self._lock:
                        self._check_timeout()
                        self._publish_battery_state()
                        self._publish_chassis_state()
                    time.sleep(0.001)
                    continue
                    
        except KeyboardInterrupt:
            print("\n[CHASSIS_SVC] 正在关闭...")
        finally:
            self.stop()
    
    def stop(self) -> None:
        """停止底盘服务"""
        if not getattr(self, '_stopped', False):
            self._stopped = True
            self._running = False
            # close() 内部已包含 stop() 逻辑，避免重复写入舵机速度
            self.chassis.close()
            if self._rep_socket:
                self._rep_socket.close()
                self._rep_socket = None
            if self._pub_socket:
                self._pub_socket.close()
                self._pub_socket = None
            if self._state_pub_socket:
                self._state_pub_socket.close()
                self._state_pub_socket = None
            if self._context:
                self._context.term()
                self._context = None
            print("[CHASSIS_SVC] 已关闭")


def main():
    """主入口 - 支持命令行参数覆盖配置文件"""
    import argparse
    
    parser = argparse.ArgumentParser(description='HomeBot 底盘服务')
    parser.add_argument('--port', default=None, help='覆盖配置文件的串口')
    parser.add_argument('--addr', default=None, help='覆盖配置文件的ZeroMQ REP地址')
    parser.add_argument('--battery-addr', default=None, help='覆盖配置文件的电池状态PUB地址')
    parser.add_argument('--shared-bus', action='store_true', help='使用共享串口总线（与机械臂共用）')
    
    args = parser.parse_args()
    
    # 如果有命令行参数，修改配置
    if args.port:
        get_config().chassis.serial_port = args.port
        print(f"[命令行] 覆盖串口为: {args.port}")
    
    rep_addr = args.addr or get_config().chassis.service_addr
    battery_addr = args.battery_addr or get_config().battery.pub_addr
    
    service = ChassisService(
        rep_addr=rep_addr, 
        pub_addr=battery_addr,
        use_shared_bus=args.shared_bus
    )
    service.start()


if __name__ == '__main__':
    main()
