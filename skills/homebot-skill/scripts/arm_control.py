"""
HomeBot Arm Controller
机械臂局域网控制器 - 客户端
通过ZeroMQ REQ-REP协议发送关节角度指令
参考HomeBot机械臂服务协议，支持优先级仲裁和自动重连

用法:
    python arm_control.py joint <joint_name> <angle>
    python arm_control.py joints "<name1>:<angle1>,<name2>:<angle2>..."
    python arm_control.py gripper open|close|<0-90>
    python arm_control.py home
    python arm_control.py status
"""

import sys
import time
import argparse
import zmq
from typing import Dict, Optional
from dataclasses import dataclass

# ============ 配置 ============
ROBOT_IP = "192.168.1.13"
ROBOT_PORT = 5557  # 与HomeBot配置一致: arm_service_addr = "tcp://*:5557"
DEFAULT_SOURCE = "picoclaw"
DEFAULT_PRIORITY = 2
DEFAULT_SPEED = 1000
# ==============================

# 控制源优先级定义（与服务端保持一致）
PRIORITIES = {
    "emergency": 4,
    "auto": 3,
    "voice": 2,
    "web": 1,
}


@dataclass
class ArbiterResponse:
    """仲裁器响应数据结构"""
    success: bool
    message: str
    current_owner: str
    current_priority: int
    joint_states: Optional[Dict[str, float]] = None


class HomeBotArmController:
    """
    机械臂服务客户端
    用于向HomeBot机械臂服务发送关节控制指令
    完全匹配HomeBot arm_service.py 协议
    
    协议格式（请求）:
    {
        "source": "picoclaw",
        "priority": 2,
        "speed": 1000,
        "joints": {"base": 0, "shoulder": -30, ...},
        "timestamp": 1234567890.123
    }
    
    协议格式（响应）:
    {
        "success": true,
        "message": "指令已接受",
        "current_owner": "picoclaw",
        "current_priority": 2,
        "joint_states": {"base": 0, "shoulder": -30, ...}
    }
    """
    
    def __init__(self, service_addr: str = None, timeout_ms: int = 3000, 
                 robot_ip: str = ROBOT_IP, robot_port: int = ROBOT_PORT):
        """
        初始化客户端
        
        Args:
            service_addr: 机械臂服务ZeroMQ地址 (例如 tcp://192.168.0.12:5557)
            timeout_ms: 请求超时时间
            robot_ip: 机器人IP地址，如果service_addr为None则使用此参数构建地址
            robot_port: 机器人端口，如果service_addr为None则使用此参数构建地址
        """
        if service_addr is None:
            service_addr = f"tcp://{robot_ip}:{robot_port}"
        
        self.service_addr = service_addr
        self.timeout_ms = timeout_ms
        self._context = zmq.Context()
        self._socket = self._create_socket()
    
    def _create_socket(self) -> zmq.Socket:
        """创建并配置socket"""
        socket = self._context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        socket.setsockopt(zmq.LINGER, 0)
        socket.connect(self.service_addr)
        return socket
    
    def send_command(self, joint_angles: Dict[str, float], 
                    speed: int = DEFAULT_SPEED,
                    source: str = DEFAULT_SOURCE, 
                    priority: int = 0) -> Optional[ArbiterResponse]:
        """
        发送关节角度命令并等待响应
        
        完全匹配HomeBot服务端协议:
        - source, priority, speed 放在顶层
        - joints 是关节角度字典 {关节名: 角度}
        
        Args:
            joint_angles: 关节角度字典 {joint_name: angle_degrees}
            speed: 运动速度 (默认 1000)
            source: 控制源
            priority: 优先级（0表示自动根据source获取）
            
        Returns:
            ArbiterResponse: 服务端响应，连接失败返回None
        """
        # 自动获取优先级
        if priority == 0 and source in PRIORITIES:
            priority = PRIORITIES[source]
        
        # 构建完整命令 - 完全匹配服务端期望格式
        command = {
            "source": source,
            "priority": priority,
            "speed": speed,
            "joints": joint_angles,
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
                current_priority=response_data.get("current_priority", 0),
                joint_states=response_data.get("joint_states", None)
            )
            
        except zmq.ZMQError as e:
            # 超时或错误，重建socket
            self._socket.close()
            self._socket = self._create_socket()
            return None
    
    def set_joint_angle(self, joint_name: str, angle: float,
                        source: str = DEFAULT_SOURCE,
                        priority: int = 0,
                        speed: int = DEFAULT_SPEED) -> Optional[ArbiterResponse]:
        """设置单个关节角度"""
        return self.send_command({joint_name: angle}, speed, source, priority)
    
    def set_joint_angles(self, angles: Dict[str, float],
                         source: str = DEFAULT_SOURCE,
                         priority: int = 0,
                         speed: int = DEFAULT_SPEED) -> Optional[ArbiterResponse]:
        """同时设置多个关节角度"""
        return self.send_command(angles, speed, source, priority)
    
    def set_gripper(self, angle: float,
                   source: str = DEFAULT_SOURCE,
                   priority: int = 0,
                   speed: int = DEFAULT_SPEED) -> Optional[ArbiterResponse]:
        """
        设置夹爪角度（直接使用度数，0-90度）
        
        Args:
            angle: 夹爪角度 (0-90度，通常0=关闭，90=打开)
        """
        return self.send_command({"gripper": angle}, speed, source, priority)
    
    def open_gripper(self, source: str = DEFAULT_SOURCE,
                    priority: int = 0,
                    speed: int = DEFAULT_SPEED) -> Optional[ArbiterResponse]:
        """打开夹爪（90度）"""
        return self.set_gripper(90, source, priority, speed)
    
    def close_gripper(self, source: str = DEFAULT_SOURCE,
                     priority: int = 0,
                     speed: int = DEFAULT_SPEED) -> Optional[ArbiterResponse]:
        """关闭夹爪（0度）"""
        return self.set_gripper(0, source, priority, speed)
    
    def move_home(self, source: str = DEFAULT_SOURCE,
                  priority: int = 0,
                  speed: int = DEFAULT_SPEED) -> Optional[ArbiterResponse]:
        """
        移动机械臂到休息位置（Home位置）
        从全局配置读取rest_position，服务端会自动应用
        """
        # Home位置由服务端根据配置自动处理，这里发送空joints
        # 服务端在启动时已经会自动go home，这个命令用于手动回原点
        return self.send_command({}, speed, source, priority)
    
    def get_status(self) -> Optional[ArbiterResponse]:
        """获取当前状态（所有关节角度）"""
        # 获取状态使用system高优先级，不会抢占控制除非空闲
        return self.send_command({}, speed=DEFAULT_SPEED, source="system", priority=4)
    
    def move_lift(self, direction: int, step: float = 20.0,
                  source: str = DEFAULT_SOURCE,
                  priority: int = 0,
                  speed: int = DEFAULT_SPEED) -> Optional[ArbiterResponse]:
        """
        控制升降平台移动
        
        Args:
            direction: 1=上升(向0靠近，高度增加), -1=下降(向负值移动，高度减少)
            step: 移动步长 (mm)，默认20mm
            source: 控制源
            priority: 优先级
            speed: 运动速度
            
        Returns:
            ArbiterResponse: 服务端响应
        """
        # 自动获取优先级
        if priority == 0 and source in PRIORITIES:
            priority = PRIORITIES[source]
        
        # 构建命令，包含 lift_height 字段
        command = {
            "source": source,
            "priority": priority,
            "speed": speed,
            "joints": {},
            "lift_direction": direction,
            "lift_step": step,
            "timestamp": time.time()
        }
        
        try:
            self._socket.send_json(command)
            response_data = self._socket.recv_json()
            
            return ArbiterResponse(
                success=response_data.get("success", False),
                message=response_data.get("message", ""),
                current_owner=response_data.get("current_owner", ""),
                current_priority=response_data.get("current_priority", 0),
                joint_states=response_data.get("joint_states", None)
            )
        except zmq.ZMQError as e:
            self._socket.close()
            self._socket = self._create_socket()
            return None
    
    def lift_up(self, step: float = 20.0,
                source: str = DEFAULT_SOURCE,
                priority: int = 0,
                speed: int = DEFAULT_SPEED) -> Optional[ArbiterResponse]:
        """升降平台上升"""
        return self.move_lift(1, step, source, priority, speed)
    
    def lift_down(self, step: float = 20.0,
                  source: str = DEFAULT_SOURCE,
                  priority: int = 0,
                  speed: int = DEFAULT_SPEED) -> Optional[ArbiterResponse]:
        """升降平台下降"""
        return self.move_lift(-1, step, source, priority, speed)
    
    def close(self):
        """关闭客户端"""
        if self._socket:
            self._socket.close()
        if self._context:
            self._context.term()


def print_response(response: Optional[ArbiterResponse]):
    """打印响应结果"""
    if response is None:
        print("[ERROR] 无响应（连接超时或网络错误，请检查机械臂服务是否启动）")
    elif response.success:
        print(f"[OK] {response.message}")
        print(f"[INFO] 当前控制权: {response.current_owner} (优先级: {response.current_priority})")
        if response.joint_states:
            print("[INFO] 当前关节角度:")
            for joint, angle in response.joint_states.items():
                print(f"   {joint}: {angle:.1f}°")
    else:
        print(f"[FAIL] 失败: {response.message}")
        print(f"[INFO] 当前控制权: {response.current_owner} (优先级: {response.current_priority})")


def main():
    parser = argparse.ArgumentParser(description="HomeBot机械臂控制器")
    parser.add_argument("--ip", default=ROBOT_IP, help=f"机器人IP地址 (默认: {ROBOT_IP})")
    parser.add_argument("--port", type=int, default=ROBOT_PORT, help=f"ZeroMQ端口 (默认: {ROBOT_PORT})")

    subparsers = parser.add_subparsers(dest="command", required=True)

    # joint 命令
    p_joint = subparsers.add_parser("joint", help="设置单个关节角度")
    p_joint.add_argument("name", help="关节名称 (base/shoulder/elbow/wrist_flex/wrist_roll/gripper)")
    p_joint.add_argument("angle", type=float, help="目标角度 (度)")
    p_joint.add_argument("--source", default=DEFAULT_SOURCE, help="控制源")
    p_joint.add_argument("--priority", type=int, default=0, help="优先级 (0=自动)")
    p_joint.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="运动速度")

    # joints 命令
    p_joints = subparsers.add_parser("joints", help="同时设置多个关节角度，格式: name1:angle1,name2:angle2")
    p_joints.add_argument("angles_str", help="多个关节角度描述")
    p_joints.add_argument("--source", default=DEFAULT_SOURCE, help="控制源")
    p_joints.add_argument("--priority", type=int, default=0, help="优先级 (0=自动)")
    p_joints.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="运动速度")

    # gripper 命令
    p_gripper = subparsers.add_parser("gripper", help="控制夹爪")
    p_gripper.add_argument("arg", help="open/close 或 角度(0-90)")
    p_gripper.add_argument("--source", default=DEFAULT_SOURCE, help="控制源")
    p_gripper.add_argument("--priority", type=int, default=0, help="优先级 (0=自动)")
    p_gripper.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="运动速度")

    # home 命令
    p_home = subparsers.add_parser("home", help="移动到原点/休息位置")
    p_home.add_argument("--source", default=DEFAULT_SOURCE, help="控制源")
    p_home.add_argument("--priority", type=int, default=0, help="优先级 (0=自动)")
    p_home.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="运动速度")

    # status 命令
    subparsers.add_parser("status", help="获取当前关节角度")

    # stop 命令
    subparsers.add_parser("stop", help="紧急停止（清空指令，超时释放控制权）")
    
    # lift 命令 - 升降平台控制
    p_lift = subparsers.add_parser("lift", help="控制升降平台")
    p_lift.add_argument("direction", choices=["up", "down"], help="方向 (up=上升, down=下降)")
    p_lift.add_argument("--step", type=float, default=20.0, help="移动步长 (mm)，默认20mm")
    p_lift.add_argument("--source", default=DEFAULT_SOURCE, help="控制源")
    p_lift.add_argument("--priority", type=int, default=0, help="优先级 (0=自动)")
    p_lift.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="运动速度")

    args = parser.parse_args()

    # 创建控制器
    arm = HomeBotArmController(robot_ip=args.ip, robot_port=args.port)

    try:
        if args.command == "joint":
            resp = arm.set_joint_angle(args.name, args.angle, args.source, args.priority, args.speed)
            print_response(resp)
        elif args.command == "joints":
            # 解析 "base:0,shoulder:30,elbow:45"
            angles: Dict[str, float] = {}
            for part in args.angles_str.replace('"', '').replace("'", "").split(","):
                if ":" not in part:
                    continue
                name, ang = part.split(":")
                angles[name.strip()] = float(ang.strip())
            resp = arm.set_joint_angles(angles, args.source, args.priority, args.speed)
            print_response(resp)
        elif args.command == "gripper":
            if hasattr(args, 'source') and hasattr(args, 'priority') and hasattr(args, 'speed'):
                if args.arg.lower() == "open":
                    resp = arm.open_gripper(args.source, args.priority, args.speed)
                elif args.arg.lower() == "close":
                    resp = arm.close_gripper(args.source, args.priority, args.speed)
                else:
                    try:
                        angle = float(args.arg)
                        # 钳位到0-90度
                        angle = max(0, min(90, angle))
                        resp = arm.set_gripper(angle, args.source, args.priority, args.speed)
                    except ValueError:
                        print("参数错误: 必须是 open/close 或 0-90 之间的角度")
                        sys.exit(1)
                print_response(resp)
        elif args.command == "home":
            resp = arm.move_home(args.source, args.priority, args.speed)
            print_response(resp)
        elif args.command == "status":
            resp = arm.get_status()
            print_response(resp)
        elif args.command == "stop":
            # 紧急停止使用最高优先级，发送空指令让控制权释放
            # 实际上只需要清空当前命令并设置超时，这里发送空指令
            resp = arm.send_command({}, source="emergency", priority=4)
            print_response(resp)
        elif args.command == "lift":
            # 升降平台控制
            if args.direction == "up":
                resp = arm.lift_up(step=args.step, source=args.source, priority=args.priority, speed=args.speed)
                print_response(resp)
            elif args.direction == "down":
                resp = arm.lift_down(step=args.step, source=args.source, priority=args.priority, speed=args.speed)
                print_response(resp)
        else:
            print(f"未知命令: {args.command}")

    finally:
        arm.close()


if __name__ == "__main__":
    main()