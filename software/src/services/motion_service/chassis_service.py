"""
底盘服务
从配置文件读取硬件参数
"""
import os
import sys
import time
from typing import Optional, Dict, Any
from dataclasses import dataclass, asdict
from threading import Lock

import zmq
from .chassis_arbiter import ControlCommand, ArbiterResponse, PRIORITIES
from .servo_bus_manager import ServoBusManager, get_servo_bus
from hal.chassis.driver import ChassisDriver
from configs import get_config, ChassisConfig


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
        # 支持传入外部总线（共享模式）
        self.driver = ChassisDriver(self.config, bus=bus)
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
    """
    
    TIMEOUT_MS = 1000
    
    def __init__(self, rep_addr: Optional[str] = None, use_shared_bus: bool = False):
        """
        初始化底盘服务
        
        Args:
            rep_addr: ZeroMQ REP地址，默认从配置文件读取
            use_shared_bus: 是否使用共享串口总线（与机械臂共用）
        """
        # 从配置文件读取地址
        config = get_config().chassis
        self.rep_addr = rep_addr or config.service_addr
        
        # 创建真实底盘控制器
        if use_shared_bus:
            # 使用共享总线
            bus = get_servo_bus()
            self.chassis = RealChassisController(config, bus=bus)
        else:
            # 独立模式
            self.chassis = RealChassisController(config)
        
        # 控制权状态
        self._current_owner: Optional[str] = None
        self._current_priority: int = 0
        self._last_command_time: float = 0.0
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_vz = 0.0
        
        # 紧急停止锁定状态 - 一旦触发，必须通过归位解除
        self._emergency_locked: bool = False
        
        self._lock = Lock()
        self._context: Optional[zmq.Context] = None
        self._rep_socket: Optional[zmq.Socket] = None
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
    
    def _parse_request(self, data: Dict[str, Any]) -> Optional[ControlCommand]:
        """解析REQ请求数据"""
        try:
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
        print(f"[配置] 串口: {config.serial_port}")
        print(f"[配置] 波特率: {config.baudrate}")
        print(f"[配置] 最大线速度: {config.max_linear_speed} m/s")
        print(f"[配置] 服务地址: {self.rep_addr}")
        
        # 初始化底盘硬件
        if not self.chassis.initialize():
            print("[CHASSIS_SVC] 底盘硬件初始化失败，退出")
            return
        
        # 启动ZeroMQ
        self._context = zmq.Context()
        self._rep_socket = self._context.socket(zmq.REP)
        self._rep_socket.setsockopt(zmq.LINGER, 0)
        self._rep_socket.bind(self.rep_addr)
        
        self._running = True
        print("[CHASSIS_SVC] 底盘服务已启动，等待控制源连接...")
        print("=" * 60)
        
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
                    else:
                        response = self._arbitrate(cmd)
                    
                    self._rep_socket.send_json(asdict(response))
                    
                except zmq.Again:
                    with self._lock:
                        self._check_timeout()
                    time.sleep(0.001)
                    continue
                    
        except KeyboardInterrupt:
            print("\n[CHASSIS_SVC] 正在关闭...")
        finally:
            self.stop()
    
    def stop(self) -> None:
        """停止底盘服务"""
        self._running = False
        self.chassis.stop()
        self.chassis.close()
        if self._rep_socket:
            self._rep_socket.close()
        if self._context:
            self._context.term()
        print("[CHASSIS_SVC] 已关闭")


def main():
    """主入口 - 支持命令行参数覆盖配置文件"""
    import argparse
    
    parser = argparse.ArgumentParser(description='HomeBot 底盘服务')
    parser.add_argument('--port', default=None, help='覆盖配置文件的串口')
    parser.add_argument('--addr', default=None, help='覆盖配置文件的ZeroMQ地址')
    parser.add_argument('--shared-bus', action='store_true', help='使用共享串口总线（与机械臂共用）')
    
    args = parser.parse_args()
    
    # 如果有命令行参数，修改配置
    if args.port:
        get_config().chassis.serial_port = args.port
        print(f"[命令行] 覆盖串口为: {args.port}")
    
    rep_addr = args.addr or get_config().chassis.service_addr
    
    service = ChassisService(rep_addr=rep_addr, use_shared_bus=args.shared_bus)
    service.start()


if __name__ == '__main__':
    main()
