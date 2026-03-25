"""
底盘仲裁器 - 核心数据结构
用于ChassisService和ArmService的仲裁逻辑
"""
from dataclasses import dataclass, field
from typing import Optional, Dict


@dataclass
class ControlCommand:
    """控制指令数据结构"""
    source: str
    vx: float
    vy: float
    vz: float
    priority: int
    timestamp: float = 0.0


@dataclass
class ArbiterResponse:
    """仲裁器响应数据结构"""
    success: bool
    message: str
    current_owner: str
    current_priority: int


@dataclass
class ArmResponse:
    """机械臂服务响应数据结构"""
    success: bool
    message: str
    current_owner: str
    current_priority: int
    joint_states: Optional[Dict[str, float]] = None


# 控制源优先级定义
# 数值越大优先级越高
PRIORITIES = {
    "emergency": 4,   # 紧急停止（最高）
    "gamepad": 3,     # 游戏手柄控制
    "auto": 3,        # 自动模式（人体跟随等）
    "voice": 2,       # 语音控制
    "web": 1,         # Web 遥控（最低）
}


class ChassisArbiterClient:
    """
    底盘仲裁器客户端
    用于向底盘服务发送控制指令
    """
    
    def __init__(self, service_addr: str = "tcp://127.0.0.1:5556", timeout_ms: int = 1000):
        """
        初始化客户端
        
        Args:
            service_addr: 底盘服务ZeroMQ地址
            timeout_ms: 请求超时时间
        """
        import zmq
        
        self.service_addr = service_addr
        self.timeout_ms = timeout_ms
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(service_addr)
    
    def send_command(self, vx: float, vy: float, vz: float,
                    source: str = "auto", priority: int = 0) -> Optional[ArbiterResponse]:
        """
        发送控制指令
        
        Args:
            vx: 线速度X (m/s)
            vy: 线速度Y (m/s)
            vz: 角速度Z (rad/s)
            source: 控制源
            priority: 优先级（0表示自动根据source获取）
            
        Returns:
            ArbiterResponse: 仲裁器响应，失败返回None
        """
        import time
        from dataclasses import asdict
        
        # 自动获取优先级
        if priority == 0 and source in PRIORITIES:
            priority = PRIORITIES[source]
        
        # 构建命令
        command = {
            "source": source,
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "priority": priority,
            "timestamp": time.time()
        }
        
        try:
            # 发送请求
            self._socket.send_json(command)
            
            # 接收响应
            response_data = self._socket.recv_json()
            
            return ArbiterResponse(
                success=response_data.get("success", False),
                message=response_data.get("message", ""),
                current_owner=response_data.get("current_owner", ""),
                current_priority=response_data.get("current_priority", 0)
            )
            
        except Exception as e:
            # 超时或错误，重建socket
            import zmq
            self._socket.close()
            self._socket = self._context.socket(zmq.REQ)
            self._socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.connect(self.service_addr)
            return None
    
    def close(self):
        """关闭客户端"""
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
            self._socket = None
        # 注意：不要调用 _context.term()，因为使用的是单例模式
        # zmq.Context.instance() 返回的全局上下文不应被单个客户端终止


class ArmArbiterClient:
    """
    机械臂仲裁器客户端
    用于向机械臂服务发送控制指令
    """
    
    def __init__(self, service_addr: str = "tcp://127.0.0.1:5557", timeout_ms: int = 1000):
        """
        初始化客户端
        
        Args:
            service_addr: 机械臂服务ZeroMQ地址
            timeout_ms: 请求超时时间
        """
        import zmq
        
        self.service_addr = service_addr
        self.timeout_ms = timeout_ms
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(service_addr)
    
    def send_joint_command(self, joints: list, source: str = "web", 
                          priority: int = 0, speed: int = 1000) -> Optional[ArmResponse]:
        """
        发送关节角度指令
        
        Args:
            joints: 6个关节的角度数组 [j1, j2, j3, j4, j5, j6]
            source: 控制源
            priority: 优先级（0表示自动根据source获取）
            speed: 运动速度
            
        Returns:
            ArmResponse: 响应，失败返回None
        """
        import time
        from dataclasses import asdict
        
        if len(joints) != 6:
            print("[ArmClient] 错误：需要提供6个关节角度")
            return None
        
        # 自动获取优先级
        if priority == 0 and source in PRIORITIES:
            priority = PRIORITIES[source]
        
        command = {
            "source": source,
            "joints": joints,  # [j1, j2, j3, j4, j5, j6]
            "speed": speed,
            "priority": priority,
            "timestamp": time.time()
        }
        
        try:
            self._socket.send_json(command)
            response_data = self._socket.recv_json()
            
            return ArmResponse(
                success=response_data.get("success", False),
                message=response_data.get("message", ""),
                current_owner=response_data.get("current_owner", ""),
                current_priority=response_data.get("current_priority", 0),
                joint_states=response_data.get("joint_states")
            )
            
        except Exception as e:
            # 超时或错误，重建socket
            import zmq
            self._socket.close()
            self._socket = self._context.socket(zmq.REQ)
            self._socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.connect(self.service_addr)
            return None
    
    def send_joint_dict(self, joints_dict: Dict[str, float], source: str = "web",
                       priority: int = 0, speed: int = 1000) -> Optional[ArmResponse]:
        """
        使用关节名称字典发送指令
        
        Args:
            joints_dict: 关节角度字典，如 {"base": 0, "shoulder": 45, ...}
            source: 控制源
            priority: 优先级
            speed: 运动速度
            
        Returns:
            ArmResponse: 响应，失败返回None
        """
        import time
        
        if priority == 0 and source in PRIORITIES:
            priority = PRIORITIES[source]
        
        command = {
            "source": source,
            "joints": joints_dict,
            "speed": speed,
            "priority": priority,
            "timestamp": time.time()
        }
        
        try:
            self._socket.send_json(command)
            response_data = self._socket.recv_json()
            
            return ArmResponse(
                success=response_data.get("success", False),
                message=response_data.get("message", ""),
                current_owner=response_data.get("current_owner", ""),
                current_priority=response_data.get("current_priority", 0),
                joint_states=response_data.get("joint_states")
            )
            
        except Exception as e:
            import zmq
            self._socket.close()
            self._socket = self._context.socket(zmq.REQ)
            self._socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.connect(self.service_addr)
            return None
    
    def close(self):
        """关闭客户端"""
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
            self._socket = None
        # 注意：不要调用 _context.term()，因为使用的是单例模式
        # zmq.Context.instance() 返回的全局上下文不应被单个客户端终止
