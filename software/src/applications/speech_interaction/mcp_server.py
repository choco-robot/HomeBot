"""MCP Server - 提供机器人控制工具

通过 ZeroMQ 调用底盘服务和机械臂服务
与 dialogue_manager 集成，支持 LLM 工具调用
"""
import json
import zmq
import time
import asyncio
from typing import Optional
from fastmcp import FastMCP

from common.logging import get_logger
from common.zmq_helper import create_socket
from configs.config import get_config

logger = get_logger(__name__)

# 创建全局 FastMCP 服务器实例
mcp = FastMCP("HomeBot Voice Interaction MCP Server")


class RobotControllerClient:
    """机器人控制器客户端 - 通过 ZeroMQ 调用服务"""
    
    def __init__(self):
        config = get_config()
        self.chassis_addr = config.zmq.chassis_service_addr.replace("*", "localhost")
        self.arm_addr = config.zmq.arm_service_addr.replace("*", "localhost")
        self.context = zmq.Context()
        self.chassis_socket = None
        self.arm_socket = None
    
    def _get_chassis_socket(self):
        """获取底盘服务 socket（惰性创建）"""
        if self.chassis_socket is None:
            self.chassis_socket = create_socket(
                zmq.REQ, 
                bind=False, 
                address=self.chassis_addr,
                context=self.context
            )
        return self.chassis_socket
    
    def _get_arm_socket(self):
        """获取机械臂服务 socket（惰性创建）"""
        if self.arm_socket is None:
            self.arm_socket = create_socket(
                zmq.REQ,
                bind=False,
                address=self.arm_addr,
                context=self.context
            )
        return self.arm_socket
    
    def send_chassis_command(self, vx: float, vy: float, vz: float, duration_ms: int = 1000) -> dict:
        """发送底盘控制命令
        
        Args:
            vx: X方向线速度（m/s）
            vy: Y方向线速度（m/s）
            vz: Z方向角速度（rad/s）
            duration_ms: 持续时间（毫秒），默认1秒后自动停止
            
        Returns:
            dict: 命令执行结果
        """
        try:
            socket = self._get_chassis_socket()
            command = {
                "source": "voice",
                "vx": vx,
                "vy": vy,
                "vz": vz,
                "priority": 2
            }
            socket.send_json(command)
            response = socket.recv_json()
            
            # 如果指定了持续时间，等待后发送停止命令
            if duration_ms > 0:
                time.sleep(duration_ms / 1000.0)
                stop_command = {
                    "source": "voice",
                    "vx": 0,
                    "vy": 0,
                    "vz": 0,
                    "priority": 2
                }
                socket.send_json(stop_command)
                socket.recv_json()
            
            return {"status": "success", "data": response}
        except Exception as e:
            logger.error(f"发送底盘命令失败: {e}")
            return {"status": "error", "message": str(e)}
    
    def stop_chassis(self) -> dict:
        """停止底盘运动"""
        try:
            socket = self._get_chassis_socket()
            command = {
                "source": "voice",
                "vx": 0,
                "vy": 0,
                "vz": 0,
                "priority": 2
            }
            socket.send_json(command)
            response = socket.recv_json()
            return {"status": "success", "data": response}
        except Exception as e:
            logger.error(f"停止底盘失败: {e}")
            return {"status": "error", "message": str(e)}
    
    def send_arm_command(self, action: str, params: dict = None) -> dict:
        """发送机械臂控制命令（预留）
        
        Args:
            action: 动作名称
            params: 动作参数
            
        Returns:
            dict: 命令执行结果
        """
        try:
            socket = self._get_arm_socket()
            command = {
                "source": "voice",
                "action": action,
                "params": params or {},
                "priority": 2
            }
            socket.send_json(command)
            response = socket.recv_json()
            return {"status": "success", "data": response}
        except Exception as e:
            logger.error(f"发送机械臂命令失败: {e}")
            return {"status": "error", "message": str(e)}
    
    def close(self):
        """关闭连接"""
        if self.chassis_socket:
            self.chassis_socket.close()
        if self.arm_socket:
            self.arm_socket.close()
        self.context.term()


# 全局控制器客户端实例
_controller_client: Optional[RobotControllerClient] = None


def get_controller() -> RobotControllerClient:
    """获取机器人控制器客户端（单例）"""
    global _controller_client
    if _controller_client is None:
        _controller_client = RobotControllerClient()
    return _controller_client


# ==================== MCP 工具定义 ====================

@mcp.tool
def move_forward(distance: float, speed: float = 0.3) -> dict:
    """控制机器人向前移动指定距离
    
    Args:
        distance: 移动距离，单位：米，范围 0.1-2.0
        speed: 移动速度，范围 0.1-0.5 m/s
    
    Returns:
        移动结果
    """
    try:
        controller = get_controller()
        # 计算持续时间（毫秒）
        duration_ms = int((distance / speed) * 1000) if speed > 0 else 1000
        # 限制最大持续时间（安全）
        duration_ms = min(duration_ms, 5000)  # 最多5秒
        
        result = controller.send_chassis_command(speed, 0, 0, duration_ms)
        success = result.get("status") == "success"
        
        return {
            "status": "success" if success else "failed",
            "message": f"机器人已向前移动 {distance} 米" if success else f"移动失败: {result.get('message', '')}"
        }
    except Exception as e:
        logger.error(f"移动机器人失败: {e}")
        return {"status": "error", "message": str(e)}


@mcp.tool
def move_backward(distance: float, speed: float = 0.3) -> dict:
    """控制机器人向后移动指定距离
    
    Args:
        distance: 移动距离，单位：米，范围 0.1-2.0
        speed: 移动速度，范围 0.1-0.5 m/s
    
    Returns:
        移动结果
    """
    try:
        controller = get_controller()
        duration_ms = int((distance / speed) * 1000) if speed > 0 else 1000
        duration_ms = min(duration_ms, 5000)
        
        result = controller.send_chassis_command(-speed, 0, 0, duration_ms)
        success = result.get("status") == "success"
        
        return {
            "status": "success" if success else "failed",
            "message": f"机器人已向后移动 {distance} 米" if success else f"移动失败: {result.get('message', '')}"
        }
    except Exception as e:
        logger.error(f"移动机器人失败: {e}")
        return {"status": "error", "message": str(e)}


@mcp.tool
def turn_left(angle: float, speed: float = 0.5) -> dict:
    """控制机器人向左旋转指定角度
    
    Args:
        angle: 旋转角度，单位：度，范围 15-360
        speed: 旋转速度，范围 0.3-1.0 rad/s
    
    Returns:
        旋转结果
    """
    try:
        controller = get_controller()
        # 角度转弧度，计算持续时间
        angle_rad = angle * 3.14159 / 180.0
        duration_ms = int((angle_rad / speed) * 1000)
        duration_ms = min(duration_ms, 5000)
        
        result = controller.send_chassis_command(0, 0, speed, duration_ms)
        success = result.get("status") == "success"
        
        return {
            "status": "success" if success else "failed",
            "message": f"机器人已向左旋转 {angle} 度" if success else f"旋转失败: {result.get('message', '')}"
        }
    except Exception as e:
        logger.error(f"旋转机器人失败: {e}")
        return {"status": "error", "message": str(e)}


@mcp.tool
def turn_right(angle: float, speed: float = 0.5) -> dict:
    """控制机器人向右旋转指定角度
    
    Args:
        angle: 旋转角度，单位：度，范围 15-360
        speed: 旋转速度，范围 0.3-1.0 rad/s
    
    Returns:
        旋转结果
    """
    try:
        controller = get_controller()
        angle_rad = angle * 3.14159 / 180.0
        duration_ms = int((angle_rad / speed) * 1000)
        duration_ms = min(duration_ms, 5000)
        
        result = controller.send_chassis_command(0, 0, -speed, duration_ms)
        success = result.get("status") == "success"
        
        return {
            "status": "success" if success else "failed",
            "message": f"机器人已向右旋转 {angle} 度" if success else f"旋转失败: {result.get('message', '')}"
        }
    except Exception as e:
        logger.error(f"旋转机器人失败: {e}")
        return {"status": "error", "message": str(e)}


@mcp.tool
def stop_robot() -> dict:
    """停止机器人当前动作
    
    Returns:
        停止结果
    """
    try:
        controller = get_controller()
        result = controller.stop_chassis()
        success = result.get("status") == "success"
        
        return {
            "status": "success" if success else "failed",
            "message": "机器人已停止" if success else f"停止失败: {result.get('message', '')}"
        }
    except Exception as e:
        logger.error(f"停止机器人失败: {e}")
        return {"status": "error", "message": str(e)}


@mcp.tool
def get_robot_status() -> dict:
    """获取机器人当前状态
    
    Returns:
        机器人状态信息
    """
    try:
        controller = get_controller()
        # 通过发送一个停止命令来检查连接状态
        result = controller.stop_chassis()
        
        return {
            "status": "success",
            "data": {
                "chassis_connected": result.get("status") == "success",
                "chassis": "ready" if result.get("status") == "success" else "offline",
                "arm": "ready",  # 预留
                "speech": "active"
            }
        }
    except Exception as e:
        logger.error(f"获取机器人状态失败: {e}")
        return {
            "status": "success",  # 即使失败也返回成功，避免中断对话
            "data": {
                "chassis": "unknown",
                "arm": "unknown",
                "speech": "active",
                "note": "无法获取底盘状态，可能服务未启动"
            }
        }


@mcp.tool
def move_arm_to_position(joint_angles: dict) -> dict:
    """控制机械臂移动到指定关节角度（预留功能）
    
    Args:
        joint_angles: 关节角度字典，如 {"base": 0, "shoulder": 45, "elbow": 90, "wrist_flex": 0, "wrist_roll": 0, "gripper": 45}
    
    Returns:
        动作执行结果
    """
    logger.info(f"机械臂移动到: {joint_angles}")
    return {
        "status": "success",
        "message": "机械臂控制功能预留，当前版本暂不支持"
    }


@mcp.tool
def grab_object() -> dict:
    """控制机械臂执行抓取动作（预留功能）
    
    Returns:
        抓取结果
    """
    return {
        "status": "success",
        "message": "抓取功能预留，当前版本暂不支持"
    }


@mcp.tool
def release_object() -> dict:
    """控制机械臂执行释放动作（预留功能）
    
    Returns:
        释放结果
    """
    return {
        "status": "success",
        "message": "释放功能预留，当前版本暂不支持"
    }


# ==================== MCP 客户端集成 ====================

class MCPClientWrapper:
    """MCP 客户端包装器 - 供 DialogueManager 使用"""
    
    def __init__(self):
        self.tools = self._get_tools_schema()
    
    def _get_tools_schema(self) -> list:
        """获取工具列表（OpenAI Function Calling 格式）"""
        return [
            {
                "type": "function",
                "function": {
                    "name": "move_forward",
                    "description": "控制机器人向前移动指定距离",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "distance": {"type": "number", "description": "移动距离（米）"},
                            "speed": {"type": "number", "description": "移动速度（m/s）"}
                        },
                        "required": ["distance"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "move_backward",
                    "description": "控制机器人向后移动指定距离",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "distance": {"type": "number", "description": "移动距离（米）"},
                            "speed": {"type": "number", "description": "移动速度（m/s）"}
                        },
                        "required": ["distance"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "turn_left",
                    "description": "控制机器人向左旋转指定角度",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "angle": {"type": "number", "description": "旋转角度（度）"},
                            "speed": {"type": "number", "description": "旋转速度（rad/s）"}
                        },
                        "required": ["angle"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "turn_right",
                    "description": "控制机器人向右旋转指定角度",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "angle": {"type": "number", "description": "旋转角度（度）"},
                            "speed": {"type": "number", "description": "旋转速度（rad/s）"}
                        },
                        "required": ["angle"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "stop_robot",
                    "description": "停止机器人当前动作",
                    "parameters": {"type": "object", "properties": {}}
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_robot_status",
                    "description": "获取机器人当前状态",
                    "parameters": {"type": "object", "properties": {}}
                }
            }
        ]
    
    async def call_tool(self, tool_name: str, arguments: dict) -> dict:
        """调用工具"""
        logger.info(f"MCP调用工具: {tool_name}, 参数: {arguments}")
        
        # 工具映射
        tool_map = {
            "move_forward": move_forward,
            "move_backward": move_backward,
            "turn_left": turn_left,
            "turn_right": turn_right,
            "stop_robot": stop_robot,
            "get_robot_status": get_robot_status,
            "move_arm_to_position": move_arm_to_position,
            "grab_object": grab_object,
            "release_object": release_object,
        }
        
        if tool_name in tool_map:
            try:
                # 同步工具在异步环境中运行
                result = await asyncio.get_event_loop().run_in_executor(
                    None, 
                    lambda: tool_map[tool_name](**arguments)
                )
                return result
            except Exception as e:
                logger.error(f"工具调用异常: {e}")
                return {"status": "error", "message": str(e)}
        else:
            return {"status": "error", "message": f"未知工具: {tool_name}"}


def get_mcp_client():
    """获取 MCP 客户端实例（供 DialogueManager 使用）"""
    return MCPClientWrapper()


if __name__ == "__main__":
    # 运行 MCP 服务器，使用 STDIO 传输
    logger.info("启动 MCP 服务器...")
    mcp.run()
