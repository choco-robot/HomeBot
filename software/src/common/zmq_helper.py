"""ZeroMQ socket helper utilities"""
import zmq
from typing import Any, Optional


def create_context() -> zmq.Context:
    return zmq.Context.instance()


def create_socket(
    socket_type: int, 
    bind: bool, 
    address: str,
    context: Optional[zmq.Context] = None
) -> zmq.Socket:
    """创建 ZeroMQ socket
    
    Args:
        socket_type: socket 类型 (zmq.PUB, zmq.SUB, zmq.REP, etc.)
        bind: 是否绑定 (True=bind, False=connect)
        address: 地址 (如 "tcp://*:5571")
        context: 可选的上下文，不传则使用全局实例
        
    Returns:
        zmq.Socket: 创建的 socket
    """
    ctx = context or create_context()
    sock = ctx.socket(socket_type)
    if bind:
        sock.bind(address)
    else:
        sock.connect(address)
    return sock


def send_json(socket: zmq.Socket, payload: Any) -> None:
    socket.send_json(payload)


def recv_json(socket: zmq.Socket) -> Any:
    return socket.recv_json()
