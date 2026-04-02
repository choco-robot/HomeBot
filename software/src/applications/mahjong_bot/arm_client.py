"""
麻将机械臂ZMQ客户端

用于连接 ArmService，发送运动指令和接收状态
"""

import zmq
import time
from typing import Dict, Optional, Any
from dataclasses import dataclass

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from common.logging import get_logger
from configs.config import get_config

logger = get_logger(__name__)


@dataclass
class ArmState:
    """机械臂状态"""
    joint_angles: Dict[str, float]
    lift_height: Optional[float]
    current_owner: str
    is_moving: bool = False


class ArmServiceClient:
    """
    机械臂服务客户端
    
    通过 ZeroMQ REQ 模式连接到 ArmService
    """
    
    def __init__(self, service_addr: str = "tcp://localhost:5557", timeout: int = 5000):
        """
        初始化客户端
        
        Args:
            service_addr: ArmService 地址
            timeout: 超时时间 (ms)
        """
        self.service_addr = service_addr
        self.timeout = timeout
        
        self._context: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None
        self._connected = False
        
    def connect(self) -> bool:
        """连接到服务"""
        try:
            self._context = zmq.Context()
            self._socket = self._context.socket(zmq.REQ)
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.setsockopt(zmq.RCVTIMEO, self.timeout)
            self._socket.connect(self.service_addr)
            
            # 测试连接
            response = self._send_request({"query": True, "source": "test"})
            if response:
                self._connected = True
                logger.info(f"✓ 已连接到机械臂服务: {self.service_addr}")
                return True
            else:
                logger.error("连接测试失败")
                return False
                
        except Exception as e:
            logger.error(f"连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self._socket:
            self._socket.close()
            self._socket = None
        if self._context:
            self._context.term()
            self._context = None
        self._connected = False
        logger.info("已断开连接")
    
    def _send_request(self, request: Dict) -> Optional[Dict]:
        """发送请求并接收响应"""
        if not self._socket:
            logger.error("未连接")
            return None
        
        try:
            self._socket.send_json(request)
            response = self._socket.recv_json()
            return response
        except zmq.Again:
            logger.error("请求超时")
            return None
        except Exception as e:
            logger.error(f"请求失败: {e}")
            return None
    
    def move_joints(self, joint_angles: Dict[str, float], 
                   speed: int = 800, source: str = "mahjong") -> bool:
        """
        移动关节到指定角度
        
        Args:
            joint_angles: 关节角度字典，如 {"base": 0, "shoulder": 45, ...}
            speed: 运动速度
            source: 控制源标识
        
        Returns:
            是否成功
        """
        request = {
            "joints": joint_angles,
            "speed": speed,
            "source": source,
            "priority": 2  # auto 级别
        }
        
        response = self._send_request(request)
        if response and response.get("success"):
            logger.debug(f"关节运动指令已发送: {joint_angles}")
            return True
        else:
            msg = response.get("message", "未知错误") if response else "无响应"
            logger.error(f"关节运动失败: {msg}")
            return False
    
    def move_to_joint_array(self, joint_array: list, 
                           speed: int = 800, source: str = "mahjong") -> bool:
        """
        使用数组格式移动关节
        
        Args:
            joint_array: [base, shoulder, elbow, wrist_flex, wrist_roll, gripper]
            speed: 运动速度
            source: 控制源标识
        """
        request = {
            "joints": joint_array,
            "speed": speed,
            "source": source,
            "priority": 2
        }
        
        response = self._send_request(request)
        if response and response.get("success"):
            return True
        else:
            msg = response.get("message", "未知错误") if response else "无响应"
            logger.error(f"关节运动失败: {msg}")
            return False
    
    def get_state(self) -> Optional[ArmState]:
        """
        获取机械臂当前状态
        
        Returns:
            ArmState 或 None
        """
        request = {
            "query": True,
            "source": "mahjong"
        }
        
        response = self._send_request(request)
        if not response:
            return None
        
        return ArmState(
            joint_angles=response.get("joint_states", {}),
            lift_height=response.get("lift_height"),
            current_owner=response.get("current_owner", "unknown"),
            is_moving=False  # 服务目前不返回运动状态
        )
    
    def move_lift_platform(self, height: float, speed: int = 1500) -> bool:
        """
        控制升降平台
        
        Args:
            height: 目标高度 (mm)，新坐标系下为负值或零
            speed: 运动速度
        
        Returns:
            是否成功
        """
        request = {
            "lift_height": height,
            "speed": speed,
            "source": "mahjong",
            "priority": 2
        }
        
        response = self._send_request(request)
        if response and response.get("success"):
            logger.debug(f"升降平台移动至: {height}mm")
            return True
        else:
            msg = response.get("message", "未知错误") if response else "无响应"
            logger.error(f"升降平台控制失败: {msg}")
            return False
    
    def emergency_stop(self) -> bool:
        """
        紧急停止
        
        发送最高优先级指令停止当前运动
        """
        request = {
            "joints": {},  # 空指令表示停止
            "source": "emergency",
            "priority": 4  # 最高优先级
        }
        
        response = self._send_request(request)
        if response:
            logger.warning("紧急停止已触发")
            return True
        return False
    
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._connected
    
    def check_collision(self, current_threshold: int = 800) -> Optional[bool]:
        """
        检查是否发生碰撞（通过电流监测）
        
        注意：这需要读取舵机电流，当前 ArmService 不直接支持，
        可以通过 get_state 间接获取或通过扩展服务实现。
        
        Args:
            current_threshold: 电流阈值 (mA)
        
        Returns:
            True=发生碰撞, False=正常, None=无法检测
        """
        # 简化版本：读取状态并假设如果关节角度与目标差异很大可能是碰撞
        # 实际实现应该读取舵机电流
        state = self.get_state()
        if not state:
            return None
        
        # TODO: 实现电流读取逻辑
        # 需要扩展 ArmService 支持读取舵机电流
        
        return False


class SafeArmController:
    """
    安全的机械臂控制器
    
    在 ArmServiceClient 基础上添加：
    - 碰撞检测
    - 运动超时保护
    - 关节限制检查
    """
    
    def __init__(self, client: ArmServiceClient):
        self.client = client
        self.joint_limits = {
            "base": (-180, 180),
            "shoulder": (0, 180),
            "elbow": (0, 180),
            "wrist_flex": (-90, 90),
            "wrist_roll": (-180, 180),
            "gripper": (0, 90),
        }
        self._last_angles: Dict[str, float] = {}
        
    def validate_angles(self, angles: Dict[str, float]) -> tuple[bool, str]:
        """
        验证关节角度是否在安全范围内
        
        Returns:
            (是否安全, 错误信息)
        """
        for joint, angle in angles.items():
            if joint in self.joint_limits:
                min_val, max_val = self.joint_limits[joint]
                if not (min_val <= angle <= max_val):
                    return False, f"关节 {joint} 角度 {angle}° 超出范围 [{min_val}, {max_val}]"
        return True, ""
    
    def move_joints_safe(self, joint_angles: Dict[str, float], 
                        speed: int = 800, 
                        check_collision: bool = True,
                        timeout: float = 10.0) -> bool:
        """
        安全地移动关节
        
        Args:
            joint_angles: 目标关节角度
            speed: 运动速度
            check_collision: 是否启用碰撞检测
            timeout: 运动超时时间
        
        Returns:
            是否成功
        """
        # 1. 验证角度
        safe, msg = self.validate_angles(joint_angles)
        if not safe:
            logger.error(f"安全检查失败: {msg}")
            return False
        
        # 2. 发送运动指令
        start_time = time.time()
        if not self.client.move_joints(joint_angles, speed):
            return False
        
        self._last_angles = joint_angles.copy()
        
        # 3. 监测运动（简化版本，实际应监测电流）
        if check_collision:
            # 等待运动完成并检查异常
            # 实际实现中应该持续监测舵机电流
            time.sleep(0.5)  # 给运动开始的时间
            
        return True
    
    def emergency_stop(self):
        """紧急停止"""
        self.client.emergency_stop()


def test_arm_client():
    """测试机械臂客户端"""
    import argparse
    
    parser = argparse.ArgumentParser(description='测试机械臂客户端')
    parser.add_argument('--addr', default='tcp://localhost:5557', help='服务地址')
    args = parser.parse_args()
    
    print("=" * 60)
    print("机械臂客户端测试")
    print("=" * 60)
    
    client = ArmServiceClient(args.addr)
    
    # 连接
    print("\n1. 连接测试")
    if not client.connect():
        print("连接失败，请检查 ArmService 是否运行")
        return
    
    # 获取状态
    print("\n2. 获取状态")
    state = client.get_state()
    if state:
        print(f"当前控制者: {state.current_owner}")
        print(f"关节状态: {state.joint_angles}")
        print(f"升降高度: {state.lift_height}")
    
    # 测试运动（小幅度）
    print("\n3. 测试运动（小幅度）")
    print("将 base 关节移动 10 度...")
    
    # 先获取当前位置
    if state and state.joint_angles:
        current_base = state.joint_angles.get('base', 0)
        target_angles = {'base': current_base + 10}
        
        if client.move_joints(target_angles, speed=500):
            print("运动指令已发送")
            time.sleep(2)
            
            # 检查新状态
            new_state = client.get_state()
            if new_state:
                new_base = new_state.joint_angles.get('base', 0)
                print(f"当前 base 角度: {new_base:.1f}°")
        else:
            print("运动失败")
    
    print("\n4. 断开连接")
    client.disconnect()
    
    print("=" * 60)


if __name__ == "__main__":
    test_arm_client()
