"""
从臂客户端模块

通过 ZeroMQ REQ-REP 连接远端机械臂服务，将主臂角度作为目标角度下发。
"""
import time
from typing import Dict, Optional

import zmq

from services.motion_service.chassis_arbiter import ArmArbiterClient
from common.logging import get_logger

logger = get_logger(__name__)


class SlaveArmClient:
    """
    从臂客户端

    封装到远端 arm_service 的 ZeroMQ 连接，固定控制源为 "teleop"。
    """

    def __init__(self, service_addr: str, timeout_ms: int = 800):
        """
        初始化从臂客户端

        Args:
            service_addr: 远端 arm_service 地址，如 tcp://192.168.1.100:5557
            timeout_ms: ZeroMQ 请求超时（毫秒）
        """
        self.service_addr = service_addr
        self.timeout_ms = timeout_ms
        self.client = ArmArbiterClient(service_addr, timeout_ms)
        self.source = "teleop"
        self.priority = 0  # 自动从 PRIORITIES 解析为 3

    def send_joint_angles(self, angles: Dict[str, float], speed: int) -> bool:
        """
        发送关节目标角度到从臂

        Args:
            angles: 关节角度字典，如 {"base": 0, "shoulder": 45, ...}
            speed: 舵机运动速度

        Returns:
            是否发送并执行成功
        """
        if not angles:
            logger.warning("跳过空角度指令")
            return False

        resp = self.client.send_joint_dict(
            angles,
            source=self.source,
            priority=self.priority,
            speed=speed,
        )

        if resp is None:
            logger.warning("发送从臂命令超时或网络错误")
            return False

        if not resp.success:
            logger.warning(f"从臂拒绝命令: {resp.message} (当前控制源: {resp.current_owner})")
            return False

        return True

    def send_query(self) -> Optional[Dict[str, float]]:
        """
        查询从臂当前关节状态（连通性检测）

        Returns:
            关节状态字典，失败返回 None
        """
        command = {
            "source": self.source,
            "priority": self.priority,
            "joints": {},
            "query": True,
            "timestamp": time.time(),
        }
        try:
            self.client._socket.send_json(command)
            response_data = self.client._socket.recv_json()
            return response_data.get("joint_states")
        except Exception as e:
            logger.warning(f"查询从臂状态失败: {e}")
            self._reset_socket()
            return None

    def _reset_socket(self) -> None:
        """重置 ZeroMQ REQ socket，避免死锁"""
        try:
            self.client._socket.close()
        except Exception:
            pass

        self.client._socket = zmq.Context.instance().socket(zmq.REQ)
        self.client._socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.client._socket.setsockopt(zmq.LINGER, 0)
        self.client._socket.connect(self.service_addr)

    def close(self) -> None:
        """关闭客户端连接"""
        self.client.close()
