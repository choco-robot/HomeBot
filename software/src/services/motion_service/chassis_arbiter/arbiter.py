"""
底盘仲裁器 - 核心数据结构
用于ChassisService的仲裁逻辑
"""
from dataclasses import dataclass
from typing import Optional


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


# 控制源优先级定义
PRIORITIES = {
    "emergency": 4,
    "auto": 3,
    "voice": 2,
    "web": 1,
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
        from common.zmq_helper import create_socket
        
        self.service_addr = service_addr
        self.timeout_ms = timeout_ms
        self._context = zmq.Context()
        self._socket = create_socket(zmq.REQ, bind=False, address=service_addr)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
    
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
            self._socket.close()
        if self._context:
            self._context.term()
