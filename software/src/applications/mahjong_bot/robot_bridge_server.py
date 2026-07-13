"""
HomeBot 麻将机器人本地桥接服务器

功能:
1. 作为 ZMQ 和 MQTT 之间的桥接，连接本地硬件与远程云服务器
2. 托管 robot_trtc.html 静态页面和 TRTC UserSig API
3. 订阅 MQTT command topic，执行机械臂/YOLO 检测相关指令
4. 将 YOLO 检测结果和系统状态发布到 MQTT status topic
5. 连接前置摄像头 ZMQ 流用于检测，连接机械臂服务执行动作

使用方法:
    cd software/src
    python -m applications.mahjong_bot.robot_bridge_server --host 0.0.0.0 --port 5200

访问:
    http://localhost:5200/static/robot_trtc.html
"""

import os
import sys
import time
import json
import signal
import atexit
from threading import Thread, Lock
from typing import Optional, Dict, Any, List, Tuple
from pathlib import Path

# Windows 平台兼容性处理
if sys.platform == 'win32':
    import ctypes
    kernel32 = ctypes.windll.kernel32
    kernel32.SetConsoleCtrlHandler.argtypes = [ctypes.c_void_p, ctypes.c_bool]
    kernel32.SetConsoleCtrlHandler.restype = ctypes.c_bool

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from flask import Flask, send_from_directory, request, jsonify

# MQTT
import paho.mqtt.client as mqtt

# 导入现有组件
from services.vision_service.vision import VisionSubscriber
from applications.mahjong_bot.arm_client import ArmServiceClient
from applications.mahjong_bot.detector import MahjongDetector
from applications.mahjong_bot.svp_nnn_detector import SvpNnnDetector
from applications.mahjong_bot.arm_controller import MahjongArmController
from applications.mahjong_bot.game_state_manager import GameStateManager
from applications.mahjong_bot.detector import TILE_CLASSES_CN
from applications.face_tracking import FaceTrackerApp
from applications.face_tracking.tracker import FaceTrackerConfig
from configs import get_config

from common.logging import get_logger
logger = get_logger(__name__)

# 语音引擎（可选，失败不影响主功能）
try:
    from services.speech_service.voice_engine import VoiceEngine
except ImportError:
    VoiceEngine = None

# ========== Flask 应用（轻量，用于托管静态页面和 API） ==========
app = Flask(__name__)


@app.route('/static/robot_trtc.html')
def robot_trtc_page():
    """托管机器人端 TRTC 页面"""
    static_dir = Path(__file__).parent / 'static'
    return send_from_directory(static_dir, 'robot_trtc.html')


@app.route('/')
def index():
    """重定向到 TRTC 页面"""
    return robot_trtc_page()


@app.route('/api/trtc/usersig')
def api_trtc_usersig():
    """
    生成 TRTC UserSig（供 robot_trtc.html 使用）
    """
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'common'))
    
    config = get_config()
    userid = request.args.get('userid', 'robot')
    expire = request.args.get('expire', 86400, type=int)
    
    if not config.trtc.sdk_app_id or not config.trtc.secret_key:
        return jsonify({"success": False, "error": "TRTC not configured"}), 500
    
    try:
        from TLSSigAPIv2 import TLSSigAPIv2
        api = TLSSigAPIv2(config.trtc.sdk_app_id, config.trtc.secret_key)
        user_sig = api.genUserSig(userid, expire)
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500
    
    return jsonify({
        "success": True,
        "sdkAppId": config.trtc.sdk_app_id,
        "userSig": user_sig,
        "userId": userid,
        "roomId": config.trtc.room_id,
        "expire": expire
    })


@app.route('/api/tools')
def api_list_tools():
    """
    获取所有可用的 MQTT/机器人控制工具列表（类似 MCP tools/list）
    
    返回 JSON 格式的工具定义列表，包含名称、描述和参数 Schema，
    供外部 Agent 自动发现可调用能力。
    """
    return jsonify({
        "success": True,
        "tools": _get_tool_definitions()
    })


def _get_tool_definitions() -> List[Dict[str, Any]]:
    """
    返回所有 MQTT 控制指令的工具定义（MCP tools/list 风格）
    
    Returns:
        工具定义列表，每个工具包含 name、description、inputSchema
    """
    return [
        {
            "name": "list_tools",
            "description": "获取机器人所有可用的控制工具列表",
            "inputSchema": {
                "type": "object",
                "properties": {},
                "required": []
            }
        },
        {
            "name": "select_tile",
            "description": "选中指定索引的麻将牌，为后续出牌做准备",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "index": {
                        "type": "integer",
                        "description": "牌的索引，对应最近一次检测到的牌列表"
                    }
                },
                "required": ["index"]
            }
        },
        {
            "name": "play_tile",
            "description": "将当前选中的麻将牌抓起并放置到出牌区",
            "inputSchema": {
                "type": "object",
                "properties": {},
                "required": []
            }
        },
        {
            "name": "arm_home",
            "description": "控制所有机械臂回到休息位",
            "inputSchema": {
                "type": "object",
                "properties": {},
                "required": []
            }
        },
        {
            "name": "gripper_toggle",
            "description": "控制夹爪打开或闭合",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "closed": {
                        "type": "boolean",
                        "description": "True=闭合夹爪，False=打开夹爪"
                    },
                    "arm_id": {
                        "type": ["integer", "null"],
                        "description": "机械臂 ID，1=左侧，2=右侧，null=同时控制两侧"
                    }
                },
                "required": ["closed"]
            }
        },
        {
            "name": "push_tiles",
            "description": "推倒指定的若干张麻将牌，用于执行吃、碰、杠等操作",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "indices": {
                        "type": "array",
                        "items": {"type": "integer"},
                        "description": "需要推倒的牌索引列表"
                    }
                },
                "required": ["indices"]
            }
        },
        {
            "name": "start_face_tracking",
            "description": "启动人脸跟踪应用，机械臂会自动跟随画面中的人脸",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "device": {
                        "type": "string",
                        "description": "推理设备，如 cpu 或 cuda"
                    },
                    "kp_base": {
                        "type": "number",
                        "description": "底座 PID 比例系数"
                    },
                    "kp_wrist": {
                        "type": "number",
                        "description": "腕部 PID 比例系数"
                    },
                    "max_step": {
                        "type": "number",
                        "description": "单步最大角度"
                    }
                },
                "required": []
            }
        },
        {
            "name": "stop_face_tracking",
            "description": "停止人脸跟踪应用",
            "inputSchema": {
                "type": "object",
                "properties": {},
                "required": []
            }
        },
        {
            "name": "speak",
            "description": "远程调用 TTS 语音播报指定文本",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "text": {
                        "type": "string",
                        "description": "需要播报的中文文本"
                    }
                },
                "required": ["text"]
            }
        }
    ]


# ========== 机器人桥接服务 ==========
class RobotBridgeService:
    """机器人本地桥接服务：ZMQ + MQTT 桥接"""
    
    def __init__(self):
        config = get_config()
        self.mahjong_cfg = config.mahjong
        self.trtc_cfg = config.trtc
        
        # MQTT 配置
        self.mqtt_broker = self.mahjong_cfg.mqtt_broker
        self.mqtt_port = self.mahjong_cfg.mqtt_port
        self.mqtt_use_tls = getattr(self.mahjong_cfg, 'mqtt_use_tls', False)
        self.mqtt_username = self.mahjong_cfg.mqtt_username
        self.mqtt_password = self.mahjong_cfg.mqtt_password
        self.command_topic = self.mahjong_cfg.mqtt_command_topic
        self.status_topic = self.mahjong_cfg.mqtt_status_topic
        self.mqtt_client_id = self.mahjong_cfg.mqtt_client_id_robot
        
        # 生成唯一的 client ID 避免冲突
        import random
        unique_id = f"{self.mqtt_client_id}_{random.randint(1000, 9999)}_{int(time.time())}"
        self.mqtt_client = mqtt.Client(client_id=unique_id)
        
        # 启用 TLS/SSL（EMQX Cloud 要求）
        if self.mqtt_use_tls:
            import ssl
            ca_cert = Path(__file__).parent / 'certs' / 'emqxsl-ca.crt'
            if ca_cert.exists():
                self.mqtt_client.tls_set(
                    ca_certs=str(ca_cert),
                    cert_reqs=ssl.CERT_REQUIRED,
                    tls_version=ssl.PROTOCOL_TLS_CLIENT
                )
                print(f"[MQTT] 已启用 TLS/SSL 加密连接 (CA: {ca_cert})")
            else:
                self.mqtt_client.tls_set()
                print(f"[MQTT] 已启用 TLS/SSL 加密连接 (系统默认 CA)")
        
        if self.mqtt_username:
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        
        # ZMQ 客户端
        self.front_video = VisionSubscriber(self.mahjong_cfg.front_vision_addr)
        self.arm_client = ArmServiceClient(self.mahjong_cfg.arm_service_addr)
        self.arm_client2 = ArmServiceClient(self.mahjong_cfg.arm2_service_addr)
        
        # 机械臂控制器（纯运动控制层）- 双机械臂
        self.arm_controller = MahjongArmController(self.mahjong_cfg.arm_service_addr, arm_id=1)
        self.arm_controller2 = MahjongArmController(self.mahjong_cfg.arm2_service_addr, arm_id=2)
        
        # 手牌状态管理器（业务逻辑层）
        self.game_state = GameStateManager(arm_id=1)
        
        # 语音引擎（TTS-only 模式，用于出牌播报）
        self.voice_engine = None
        if VoiceEngine is not None:
            try:
                self.voice_engine = VoiceEngine(mode="tts_only")
                print("[Bridge] 语音引擎初始化成功（TTS-only 模式）")
            except Exception as e:
                print(f"[Bridge] 语音引擎初始化失败，语音播报功能将不可用: {e}")
        
        # 分界线（图像宽度的百分比，小于此值为左侧，用arm1）
        self.center_split_ratio = 0.5
        
        # TRTC RTMP 推流（替代浏览器方案）
        self.trtc_streamer = None
        if getattr(self.trtc_cfg, 'rtmp_enabled', False) and self.trtc_cfg.sdk_app_id:
            try:
                from services.trtc_service.streamer import TrtcStreamer
                self.trtc_streamer = TrtcStreamer(
                    sdk_app_id=self.trtc_cfg.sdk_app_id,
                    secret_key=self.trtc_cfg.secret_key,
                    room_id=self.trtc_cfg.room_id,
                    user_id=self.trtc_cfg.rtmp_user_id,
                    video_device=self.trtc_cfg.rtmp_video_device,
                    video_width=self.trtc_cfg.rtmp_video_width,
                    video_height=self.trtc_cfg.rtmp_video_height,
                    video_fps=self.trtc_cfg.rtmp_video_fps,
                    video_bitrate=self.trtc_cfg.rtmp_video_bitrate,
                    video_input_format=self.trtc_cfg.rtmp_video_input_format,
                    audio_device=self.trtc_cfg.rtmp_audio_device or None,
                    audio_bitrate=self.trtc_cfg.rtmp_audio_bitrate,
                    use_hw_encoder=self.trtc_cfg.rtmp_use_hw_encoder,
                    hw_encoder=self.trtc_cfg.rtmp_hw_encoder,
                    ffmpeg_path=self.trtc_cfg.rtmp_ffmpeg_path,
                    auto_restart=self.trtc_cfg.rtmp_auto_restart,
                    max_retries=self.trtc_cfg.rtmp_max_retries,
                )
                logger.info("TRTC RTMP 推流模块已初始化")
            except Exception as e:
                logger.error(f"TRTC RTMP 推流模块初始化失败: {e}")
                self.trtc_streamer = None
        
        # 检测器（根据配置自动切换 YOLO / SVP NNN C 后端）
        if getattr(self.mahjong_cfg, 'detector_backend', 'yolo') == 'svp_nnn':
            logger.info("使用 SVP NNN C 后端检测器")
            self.detector = SvpNnnDetector(
                executable_path=self.mahjong_cfg.svp_nnn_executable,
                model_path=self.mahjong_cfg.svp_nnn_model_path,
                input_image_path=self.mahjong_cfg.svp_nnn_input_image,
                json_file_path=self.mahjong_cfg.svp_nnn_json_file,
                output_txt_path=self.mahjong_cfg.svp_nnn_output_txt,
                working_dir=self.mahjong_cfg.svp_nnn_working_dir or None,
                exec_timeout=self.mahjong_cfg.svp_nnn_exec_timeout,
                conf_threshold=self.mahjong_cfg.detector_conf_threshold,
                use_roboflow_classes=True,
                roi_enabled=self.mahjong_cfg.detector_roi_enabled,
                roi_x=self.mahjong_cfg.detector_roi_x,
                roi_y=self.mahjong_cfg.detector_roi_y,
                roi_width=self.mahjong_cfg.detector_roi_width,
                roi_height=self.mahjong_cfg.detector_roi_height,
                backend_width=self.mahjong_cfg.svp_nnn_backend_width or None,
                backend_height=self.mahjong_cfg.svp_nnn_backend_height or None,
            )
        else:
            logger.info("使用 YOLO Python 检测器")
            software_dir = Path(__file__).parent.parent.parent.parent
            model_path = str(software_dir / self.mahjong_cfg.detector_model_path)
            import torch
            self.detector = MahjongDetector(
                model_path=model_path,
                conf_threshold=self.mahjong_cfg.detector_conf_threshold,
                inference_size=self.mahjong_cfg.detector_inference_size,
                device="cuda" if torch.cuda.is_available() else "cpu",
                use_roboflow_classes=True,
                roi_enabled=self.mahjong_cfg.detector_roi_enabled,
                roi_x=self.mahjong_cfg.detector_roi_x,
                roi_y=self.mahjong_cfg.detector_roi_y,
                roi_width=self.mahjong_cfg.detector_roi_width,
                roi_height=self.mahjong_cfg.detector_roi_height
            )
        
        # 状态
        self._running = False
        self._lock = Lock()
        self._last_detected_tiles: list = []
        self._selected_tile_index: int = -1
        self._system_status: str = "idle"
        self._mqtt_connected = False
        
        # 人脸跟踪应用
        self.face_tracker = None
        self._face_tracker_thread = None
        
    def start(self) -> bool:
        """启动所有服务"""
        print("=" * 60)
        print("HomeBot 麻将机器人本地桥接服务")
        print("=" * 60)
        
        # 记录各服务启动状态
        startup_status = {}
        
        # 连接 MQTT
        try:
            print(f"[MQTT] 连接到 {self.mqtt_broker}:{self.mqtt_port}...")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
            self.mqtt_client.loop_start()
            startup_status['MQTT'] = True
        except Exception as e:
            print(f"[MQTT] 连接失败: {e}")
            startup_status['MQTT'] = False
            self._announce_startup(startup_status)
            return False
        
        # 连接前置视频
        self.front_video.start()
        startup_status['前置视频'] = True
        print(f"[Bridge] 前置视频: OK")
        
        # 连接机械臂1（左侧）
        ok2 = self.arm_client.connect()
        startup_status['机械臂1客户端'] = ok2
        print(f"[Bridge] 机械臂1客户端: {'OK' if ok2 else 'FAIL'}")
        
        # 连接机械臂2（右侧）
        ok2_2 = self.arm_client2.connect()
        startup_status['机械臂2客户端'] = ok2_2
        print(f"[Bridge] 机械臂2客户端: {'OK' if ok2_2 else 'FAIL'}")
        
        # 连接机械臂控制器
        ok3 = self.arm_controller.connect_arm()
        startup_status['机械臂1控制器'] = ok3
        print(f"[Bridge] 机械臂1控制器: {'OK' if ok3 else 'FAIL'}")
        
        ok3_2 = self.arm_controller2.connect_arm()
        startup_status['机械臂2控制器'] = ok3_2
        print(f"[Bridge] 机械臂2控制器: {'OK' if ok3_2 else 'FAIL'}")
        
        # 初始化检测器
        ok4 = self.detector.initialize()
        startup_status['检测器'] = ok4
        print(f"[Bridge] 检测器: {'OK' if ok4 else 'FAIL'}")
        
        # 启动 TRTC RTMP 推流
        if self.trtc_streamer:
            ok5 = self.trtc_streamer.start()
            startup_status['TRTC推流'] = ok5
            print(f"[Bridge] TRTC RTMP 推流: {'OK' if ok5 else 'FAIL'}")
        
        self._running = True
        
        # 启动检测循环
        self._detect_thread = Thread(target=self._detection_loop, daemon=True)
        self._detect_thread.start()
        
        # 系统启动语音播报
        self._announce_startup(startup_status)
        
        return True
    
    def _announce_startup(self, status: dict) -> None:
        """
        系统启动语音播报
        
        Args:
            status: 各服务启动状态字典
        """
        if not getattr(self, 'voice_engine', None):
            return
        
        # 收集失败项
        failed = [name for name, ok in status.items() if not ok]
        
        if not failed:
            # 全部成功
            self.voice_engine.speak("麻将机器人已启动，所有服务正常")
            print("[Bridge] 语音播报: 麻将机器人已启动，所有服务正常")
        elif len(failed) == len(status):
            # 全部失败（通常是 MQTT 连接失败导致提前返回）
            self.voice_engine.speak("启动失败，请检查网络连接")
            print("[Bridge] 语音播报: 启动失败，请检查网络连接")
        else:
            # 部分失败
            failed_text = "，".join(failed)
            speak_text = f"麻将机器人已启动，注意：{failed_text}未连接"
            self.voice_engine.speak(speak_text)
            print(f"[Bridge] 语音播报: {speak_text}")
    
    def stop(self):
        """停止所有服务"""
        if not self._running:
            return  # 已经停止，避免重复调用
            
        print("\n[Bridge] 正在关闭...")
        self._running = False
        
        # 等待检测线程结束
        if hasattr(self, '_detect_thread') and self._detect_thread and self._detect_thread.is_alive():
            self._detect_thread.join(timeout=2.0)
        
        # 关闭 MQTT（带异常处理）
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except Exception as e:
            pass  # 忽略关闭错误
        
        # 关闭视频流
        try:
            self.front_video.stop()
        except Exception:
            pass
            
        # 断开机械臂连接
        try:
            self.arm_client.disconnect()
        except Exception:
            pass
        
        try:
            self.arm_client2.disconnect()
        except Exception:
            pass
            
        try:
            self.arm_controller.disconnect_arm()
        except Exception:
            pass
        
        try:
            self.arm_controller2.disconnect_arm()
        except Exception:
            pass
            
        # 释放检测器
        try:
            self.detector.release()
        except Exception:
            pass
        
        # 释放语音引擎
        try:
            if getattr(self, 'voice_engine', None):
                self.voice_engine.release()
        except Exception:
            pass
        
        # 停止人脸跟踪
        try:
            if self.face_tracker and getattr(self.face_tracker, 'running', False):
                self.face_tracker.stop()
            if self._face_tracker_thread and self._face_tracker_thread.is_alive():
                self._face_tracker_thread.join(timeout=2.0)
        except Exception:
            pass
        
        # 停止 TRTC RTMP 推流
        try:
            if self.trtc_streamer:
                self.trtc_streamer.stop()
        except Exception:
            pass
            
        print("[Bridge] 已关闭")
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"[MQTT] 已连接，订阅 {self.command_topic}")
            self._mqtt_connected = True
            client.subscribe(self.command_topic, qos=1)
        else:
            print(f"[MQTT] 连接失败，返回码: {rc}")
            self._mqtt_connected = False
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        print(f"[MQTT] 断开连接，返回码: {rc}")
        self._mqtt_connected = False
    
    def _on_mqtt_message(self, client, userdata, msg):
        """处理 MQTT 控制指令"""
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            cmd = payload.get('cmd')
            data = payload.get('data', {})
            print(f"[MQTT] 收到指令: {cmd}, data: {data}")
            
            if cmd == 'select_tile':
                self._handle_select_tile(data)
            elif cmd == 'play_tile':
                self._handle_play_tile(data)
            elif cmd == 'arm_home':
                self._handle_arm_home(data)
            elif cmd == 'gripper_toggle':
                self._handle_gripper_toggle(data)
            elif cmd == 'push_tiles':
                self._handle_push_tiles(data)
            elif cmd == 'start_face_tracking':
                self._handle_start_face_tracking(data)
            elif cmd == 'stop_face_tracking':
                self._handle_stop_face_tracking(data)
            elif cmd == 'speak':
                self._handle_speak(data)
            elif cmd == 'list_tools':
                self._handle_list_tools(data)
            else:
                print(f"[MQTT] 未知指令: {cmd}")
                
        except Exception as e:
            print(f"[MQTT] 处理指令异常: {e}")
    
    def _publish_status(self, status_type: str, data: dict):
        """发布状态到 MQTT"""
        payload = json.dumps({
            "type": status_type,
            "data": data,
            "timestamp": time.time()
        })
        if self._mqtt_connected:
            self.mqtt_client.publish(self.status_topic, payload, qos=0)
    
    # ========== 指令处理器 ==========
    
    def _handle_select_tile(self, data):
        index = data.get('index', -1)
        with self._lock:
            tiles = self._last_detected_tiles
            if 0 <= index < len(tiles):
                self._selected_tile_index = index
                tile = tiles[index]
                
                # 根据牌的位置选择对应的机械臂控制器
                x1, y1, x2, y2 = tile.bbox
                cx = (x1 + x2) // 2
                frame_width = getattr(self.mahjong_cfg, 'frame_width', 1920)
                arm_id = self._select_arm_by_position(cx, frame_width)
                controller = self.arm_controller if arm_id == 1 else self.arm_controller2
                arm_name = "arm1" if arm_id == 1 else "arm2"
                
                # 更新控制器中的选中状态
                # MahjongArmController 使用 tile_id 来选牌
                # 这里我们使用索引作为 tile_id（简化）
                if tile and hasattr(tile, 'tile_id'):
                    controller.select_tile(tile.tile_id)
                
                self._publish_status("tile_selected", {
                    "success": True,
                    "index": index,
                    "arm": arm_name,
                    "tile": {
                        "name": tile.class_name,
                        "bbox": [int(x) for x in tile.bbox],
                        "confidence": round(float(tile.confidence), 2)
                    }
                })
                print(f'[Bridge] 选中第 {index} 张牌: {tile.class_name} (使用 {arm_name})')
            else:
                self._publish_status("tile_selected", {
                    "success": False,
                    "error": "无效的牌索引"
                })
    
    def _handle_play_tile(self, data):
        with self._lock:
            index = self._selected_tile_index
            tiles = self._last_detected_tiles
            
            if index < 0 or index >= len(tiles):
                self._publish_status("play_result", {
                    "success": False,
                    "error": "未选择有效的牌"
                })
                return
            
            tile = tiles[index]
            self._system_status = "executing"
            print(f'[Bridge] 执行出牌: {tile.class_name} @ {tile.bbox}')
            
            self._publish_status("play_result", {
                "success": True,
                "message": f"开始执行出牌: {tile.class_name}",
                "tile": {
                    "name": tile.class_name,
                    "center": tile.center
                }
            })
            
            self._publish_status("system_status", {
                "status": "executing",
                "message": f"正在打出 {tile.class_name}..."
            })
        
        # 在锁外执行机械臂动作（避免阻塞 MQTT 回调线程太久）
        def execute_arm():
            try:
                # 获取选中的牌
                tile = None
                with self._lock:
                    if 0 <= self._selected_tile_index < len(self._last_detected_tiles):
                        tile = self._last_detected_tiles[self._selected_tile_index]
                
                if tile is None:
                    print("[Bridge] 没有选中的牌")
                    self._publish_status("play_result", {
                        "success": False,
                        "error": "没有选中的牌"
                    })
                    return
                
                # 使用 bottom_center 坐标（出牌接触点）
                bx, by = tile.bottom_center
                
                # 根据牌的 x 坐标选择机械臂
                frame_width = getattr(self.mahjong_cfg, 'frame_width', 1920)
                arm_id = self._select_arm_by_position(bx, frame_width)
                controller = self.arm_controller if arm_id == 1 else self.arm_controller2
                arm_name = "arm1" if arm_id == 1 else "arm2"
                
                print(f"[Bridge] 出牌位置 ({bx}, {by}) -> 使用 {arm_name}")
                
                # 更新手牌状态管理器
                with self._lock:
                    detected_data = []
                    for i, t in enumerate(self._last_detected_tiles):
                        tbx, tby = t.bottom_center
                        detected_data.append({
                            'tile_id': i,
                            'class_name': t.class_name,
                            'image_x': tbx,
                            'image_y': tby,
                            'confidence': t.confidence
                        })
                    self.game_state.initialize_hand(detected_data)
                
                # 找到最近的牌并选中
                selected_tile_id = self._select_tile_by_position(self.game_state, bx, by)
                if selected_tile_id is not None:
                    self.game_state.select_tile(selected_tile_id)
                
                # 语音播报出牌名（在机械臂动作之前播报）
                if getattr(self, 'voice_engine', None):
                    try:
                        cn_name = TILE_CLASSES_CN.get(tile.class_name, tile.class_name)
                        self.voice_engine.speak(f"{cn_name}")
                        print(f"[Bridge] 语音播报: {cn_name}")
                    except Exception as e:
                        print(f"[Bridge] 语音播报失败: {e}")
                
                # 执行出牌动作（直接使用坐标）
                success = controller.pick_and_place(
                    image_x=bx,
                    image_y=by,
                    on_step=lambda name, cur, total: print(f"[{arm_name}] {name} ({cur}/{total})")
                )
                
                print(f"[Bridge] 出牌执行结果 ({arm_name}): {'成功' if success else '失败'}")
                
                if success:
                    # 更新游戏状态
                    self.game_state.discard_selected()
                
                self._publish_status("play_result", {
                    "success": success,
                    "message": f"出牌{'成功' if success else '失败'}",
                    "arm": arm_name,
                    "tile": {
                        "name": tile.class_name,
                        "center": tile.center
                    }
                })
                
            except Exception as e:
                print(f"[Bridge] 出牌执行异常: {e}")
                import traceback
                traceback.print_exc()
            finally:
                with self._lock:
                    self._system_status = "idle"
                    self._selected_tile_index = -1
                
                self._publish_status("system_status", {
                    "status": "idle",
                    "message": "出牌完成"
                })
        
        Thread(target=execute_arm, daemon=True).start()
    
    def _select_arm_by_position(self, image_x: int, frame_width: int = 1920) -> int:
        """
        根据图像 x 坐标选择机械臂
        
        Args:
            image_x: 图像X坐标
            frame_width: 图像宽度（用于计算分界线）
            
        Returns:
            1=左侧机械臂(arm1), 2=右侧机械臂(arm2)
        """
        split_x = int(frame_width * self.center_split_ratio)
        
        if image_x < split_x:
            return 1  # 左侧，使用 arm1
        else:
            return 2  # 右侧，使用 arm2
    
    def _select_tile_by_position(self, game_state: GameStateManager, 
                                 image_x: float, image_y: float,
                                 distance_threshold: float = 100.0) -> Optional[int]:
        """
        根据图像位置选择最近的牌
        
        Args:
            game_state: 游戏状态管理器
            image_x: 图像X坐标
            image_y: 图像Y坐标
            distance_threshold: 距离阈值（像素）
        
        Returns:
            选中的牌ID，如果没有找到返回None
        """
        min_dist = float('inf')
        nearest_tile = None
        
        for tile in game_state.hand:
            dist = ((tile.image_x - image_x)**2 + (tile.image_y - image_y)**2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                nearest_tile = tile
        
        if nearest_tile and min_dist < distance_threshold:
            return nearest_tile.tile_id
        
        return None
    
    def _classify_push_action(self, tiles: List) -> Tuple[Optional[str], Optional[str]]:
        """
        根据推倒的牌判断动作类型（吃/碰/杠）
        
        Args:
            tiles: 推倒的牌列表（MahjongTile 对象）
            
        Returns:
            (动作名称, 目标牌名) 目标牌名用于碰/杠时播报
        """
        if len(tiles) == 2:
            name1 = tiles[0].class_name
            name2 = tiles[1].class_name
            if name1 == name2:
                return "碰", name1
            else:
                return "吃", None
        elif len(tiles) == 3:
            names = [t.class_name for t in tiles]
            if names[0] == names[1] == names[2]:
                return "杠", names[0]
        return None, None
    
    def _handle_arm_home(self, data):
        """两个机械臂都归位"""
        from configs import get_config
        config = get_config()
        results = []
        
        # arm1 归位
        if self.arm_client and self.arm_client.is_connected():
            rest_pos = config.arm.rest_position
            success1 = self.arm_client.move_joints(rest_pos, speed=800)
            results.append(f"arm1:{'OK' if success1 else 'FAIL'}")
        else:
            results.append("arm1:未连接")
        
        # arm2 归位
        if self.arm_client2 and self.arm_client2.is_connected():
            rest_pos2 = config.arm2.rest_position
            success2 = self.arm_client2.move_joints(rest_pos2, speed=800)
            results.append(f"arm2:{'OK' if success2 else 'FAIL'}")
        else:
            results.append("arm2:未连接")
        
        success = 'OK' in results or 'OK' in results
        self._publish_status("arm_update", {
            "success": success,
            "message": f"机械臂归位完成 ({', '.join(results)})"
        })
    
    def _handle_push_tiles(self, data):
        """
        推倒指定的若干张牌
        
        Args:
            data: {"indices": [0, 2, 5]} 牌的序号列表
        """
        indices = data.get('indices', [])
        if not indices or not isinstance(indices, list):
            self._publish_status("push_result", {
                "success": False,
                "error": "无效的牌序号列表"
            })
            return
        
        with self._lock:
            tiles = self._last_detected_tiles
            # 验证所有序号是否有效
            valid_indices = [i for i in indices if 0 <= i < len(tiles)]
            invalid_indices = [i for i in indices if i not in valid_indices]
            
            if not valid_indices:
                self._publish_status("push_result", {
                    "success": False,
                    "error": "没有有效的牌序号"
                })
                return
            
            if invalid_indices:
                print(f"[Bridge] 警告: 忽略无效的牌序号: {invalid_indices}")
            
            self._system_status = "executing"
            print(f'[Bridge] 开始推倒 {len(valid_indices)} 张牌: {valid_indices}')
            
            # 发布开始状态
            self._publish_status("push_result", {
                "success": True,
                "message": f"开始推倒 {len(valid_indices)} 张牌",
                "total": len(valid_indices),
                "indices": valid_indices
            })
            
            self._publish_status("system_status", {
                "status": "executing",
                "message": f"正在推倒 {len(valid_indices)} 张牌..."
            })
        
        # 在锁外执行机械臂动作（避免阻塞 MQTT 回调线程）
        def execute_push():
            # 获取推倒的牌（用于判断吃/碰/杠）
            push_tiles = []
            with self._lock:
                for idx in valid_indices:
                    if 0 <= idx < len(self._last_detected_tiles):
                        push_tiles.append(self._last_detected_tiles[idx])
            
            # 语音播报吃/碰/杠（机械臂动作之前）
            action, target_name = self._classify_push_action(push_tiles)
            if action and getattr(self, 'voice_engine', None):
                try:
                    if action in ("碰", "杠") and target_name:
                        cn_name = TILE_CLASSES_CN.get(target_name, target_name)
                        self.voice_engine.speak(f"{action}，{cn_name}")
                        print(f"[Bridge] 语音播报: {action}，{cn_name}")
                    elif action == "吃":
                        names = [TILE_CLASSES_CN.get(t.class_name, t.class_name) for t in push_tiles]
                        speak_text = f"吃，{'，'.join(names)}"
                        self.voice_engine.speak(speak_text)
                        print(f"[Bridge] 语音播报: {speak_text}")
                except Exception as e:
                    print(f"[Bridge] 语音播报失败: {e}")
            
            success_count = 0
            failed_count = 0
            results = []
            
            # 获取图像宽度
            frame_width = getattr(self.mahjong_cfg, 'frame_width', 1920)
            
            last_arm=None
            for idx in valid_indices[::-1]:
                try:
                    tile = self._last_detected_tiles[idx]
                    print(f'[Bridge] 推倒第 {idx} 张牌: {tile.class_name}')
                    
                    # 使用 bottom_center 坐标（推倒接触点）
                    bx, by = tile.bottom_center
                    # 根据牌的 x 坐标选择机械臂
                    arm_id = self._select_arm_by_position(bx, frame_width)
                    controller = self.arm_controller if arm_id == 1 else self.arm_controller2
                    arm_name = "arm1" if arm_id == 1 else "arm2"
                    
                    print(f"[Bridge] 牌 {idx} 底部中心 ({bx}, {by}) -> 使用 {arm_name}")
                    if last_arm and last_arm!=controller:
                        last_arm.move_to_rest()
                    last_arm=controller
                    
                    # 直接使用坐标执行推倒，无需更新手牌状态
                    success = controller.push_tile(
                        image_x=bx,
                        image_y=by,
                        on_step=lambda name, cur, total: print(f"[{arm_name}] 推倒牌{idx} {name} ({cur}/{total})")
                    )
                    
                    if success:
                        success_count += 1
                        results.append({"index": idx, "success": True, "name": tile.class_name, "arm": arm_name})
                        print(f"[Bridge] 牌 {idx} 推倒成功 ({arm_name})")
                    else:
                        failed_count += 1
                        results.append({"index": idx, "success": False, "error": "推倒失败", "arm": arm_name})
                        print(f"[Bridge] 牌 {idx} 推倒失败 ({arm_name})")

                    
                except Exception as e:
                    print(f"[Bridge] 推倒牌 {idx} 异常: {e}")
                    failed_count += 1
                    results.append({"index": idx, "success": False, "error": str(e)})

            controller.move_to_rest()
            
            # 更新状态
            with self._lock:
                self._system_status = "idle"
            
            # 发布完成状态
            self._publish_status("push_result", {
                "success": True,
                "completed": True,
                "success_count": success_count,
                "failed_count": failed_count,
                "total": len(valid_indices),
                "results": results
            })
            
            self._publish_status("system_status", {
                "status": "idle",
                "message": f"推倒完成: {success_count}成功, {failed_count}失败"
            })
            
            print(f"[Bridge] 推倒完成: {success_count} 成功, {failed_count} 失败")
        
        Thread(target=execute_push, daemon=True).start()
    
    def _handle_gripper_toggle(self, data):
        """
        切换夹爪状态
        
        Args:
            data: {"closed": bool, "arm_id": 1|2|None}
                  arm_id=None 表示同时控制两个机械臂
        """
        closed = data.get('closed', False)
        arm_id = data.get('arm_id', None)  # None=both, 1=arm1, 2=arm2
        angle = 0 if closed else 90
        
        results = []
        
        # 控制 arm1
        if arm_id in (None, 1) and self.arm_client and self.arm_client.is_connected():
            success1 = self.arm_client.move_joints({"gripper": angle}, speed=1000)
            results.append(f"arm1:{'OK' if success1 else 'FAIL'}")
        
        # 控制 arm2
        if arm_id in (None, 2) and self.arm_client2 and self.arm_client2.is_connected():
            success2 = self.arm_client2.move_joints({"gripper": angle}, speed=1000)
            results.append(f"arm2:{'OK' if success2 else 'FAIL'}")
        
        if results:
            arm_desc = "全部" if arm_id is None else (f"arm{arm_id}" if arm_id in (1, 2) else "未知")
            self._publish_status("gripper_update", {
                "success": True,
                "closed": closed,
                "angle": angle,
                "arm": arm_desc,
                "message": f"{arm_desc}夹爪{'闭合' if closed else '打开'} ({', '.join(results)})"
            })
        else:
            self._publish_status("gripper_update", {
                "success": False,
                "message": "机械臂未连接"
            })
    
    def _handle_start_face_tracking(self, data):
        """
        启动人脸跟踪应用
        
        Args:
            data: {"device": "cpu", "kp_base": 0.01, "kp_wrist": 0.01, "max_step": 5.0}
        """
        with self._lock:
            if self._face_tracker_thread is not None and self._face_tracker_thread.is_alive():
                self._publish_status("face_tracking", {
                    "success": False,
                    "message": "人脸跟踪已在运行中或正在启动"
                })
                return
        
        # 创建配置
        config = FaceTrackerConfig(
            vision_sub_addr=self.mahjong_cfg.wrist1_vision_addr,
            arm_service_addr=self.mahjong_cfg.arm_service_addr,
            device=data.get('device', 'cpu'),
            kp_base=data.get('kp_base', 0.01),
            kp_wrist=data.get('kp_wrist', 0.01),
            max_base_step=data.get('max_step', 5.0),
            max_wrist_step=data.get('max_step', 5.0),
        )
        tracker = FaceTrackerApp(config=config)
        
        def run_tracker():
            """运行人脸跟踪主循环"""
            try:
                tracker.run(display=False)
            except Exception as e:
                print(f"[Bridge] 人脸跟踪运行异常: {e}")
            finally:
                with self._lock:
                    if self.face_tracker is tracker:
                        self.face_tracker = None
                    if self._face_tracker_thread is thread:
                        self._face_tracker_thread = None
        
        def monitor_startup():
            """监控启动过程，确认初始化结果"""
            for _ in range(100):  # 最多等待10秒
                time.sleep(0.1)
                if tracker.running:
                    self._publish_status("face_tracking", {
                        "success": True,
                        "message": "人脸跟踪已启动",
                        "state": tracker.state.value
                    })
                    return
                if not thread.is_alive():
                    break
            
            # 启动失败
            with self._lock:
                if self.face_tracker is tracker:
                    self.face_tracker = None
                if self._face_tracker_thread is thread:
                    self._face_tracker_thread = None
            
            self._publish_status("face_tracking", {
                "success": False,
                "message": "人脸跟踪启动失败（初始化失败或模型加载超时）"
            })
        
        thread = Thread(target=run_tracker, daemon=True)
        with self._lock:
            self.face_tracker = tracker
            self._face_tracker_thread = thread
        thread.start()
        
        # 启动后台监控线程确认初始化结果
        Thread(target=monitor_startup, daemon=True).start()
        
        self._publish_status("face_tracking", {
            "success": True,
            "message": "人脸跟踪启动中..."
        })
    
    def _handle_stop_face_tracking(self, data):
        """
        停止人脸跟踪应用
        """
        with self._lock:
            if self.face_tracker is None or not getattr(self.face_tracker, 'running', False):
                self._publish_status("face_tracking", {
                    "success": False,
                    "message": "人脸跟踪未在运行"
                })
                return
            
            tracker = self.face_tracker
            thread = self._face_tracker_thread
        
        tracker.stop()
        
        # 等待线程结束
        if thread and thread.is_alive():
            thread.join(timeout=5.0)
        
        self._publish_status("face_tracking", {
            "success": True,
            "message": "人脸跟踪已停止"
        })
    
    def _handle_speak(self, data):
        """
        远程 TTS 语音播报
        
        Args:
            data: {"text": "需要播报的文本"}
        """
        text = data.get('text', '')
        if not text or not isinstance(text, str):
            self._publish_status("speak_result", {
                "success": False,
                "error": "text 字段为空或格式错误"
            })
            print("[MQTT] TTS 指令缺少有效 text 字段")
            return
        
        if not getattr(self, 'voice_engine', None):
            self._publish_status("speak_result", {
                "success": False,
                "error": "语音引擎未初始化"
            })
            print("[MQTT] TTS 失败：语音引擎未初始化")
            return
        
        # 立即返回已接收状态，避免阻塞 MQTT 回调线程
        self._publish_status("speak_result", {
            "success": True,
            "received": True,
            "text": text,
            "message": "已开始语音播报"
        })
        print(f"[MQTT] 收到 TTS 请求: {text}")
        
        def execute_speak():
            try:
                self.voice_engine.speak(text)
                print(f"[Bridge] TTS 播报完成: {text}")
                self._publish_status("speak_result", {
                    "success": True,
                    "completed": True,
                    "text": text,
                    "message": "语音播报完成"
                })
            except Exception as e:
                print(f"[Bridge] TTS 播报失败: {e}")
                self._publish_status("speak_result", {
                    "success": False,
                    "completed": False,
                    "text": text,
                    "error": str(e)
                })
        
        Thread(target=execute_speak, daemon=True).start()
    
    def _handle_list_tools(self, data):
        """
        返回所有可用的 MQTT 控制工具列表（MCP tools/list 风格）
        
        Args:
            data: 空对象即可
        """
        tools = _get_tool_definitions()
        self._publish_status("tools_list", {
            "success": True,
            "tools": tools,
            "count": len(tools)
        })
        print(f"[MQTT] 返回工具列表，共 {len(tools)} 个工具")
    
    # ========== 检测循环 ==========
    
    def _detection_loop(self):
        import cv2
        import numpy as np
        
        detector = self.detector
        broadcast_interval = 1.0
        
        while self._running:
            try:
                loop_start = time.time()
                detections = []
                
                if detector._initialized:
                    frame_id, img = self.front_video.read_frame()
                    if img is not None:
                        detections = detector.detect(img)
                        logging_info = f"检测到 {len(detections)} 张牌"
                        with self._lock:
                            self._last_detected_tiles = detections
                            if self._system_status != "executing":
                                self._system_status = "detecting" if detections else "idle"
                        
                        # 检测到的牌保存在 self._last_detected_tiles 中
                        # 手牌状态在出牌/推倒前通过 self.game_state 更新
                        pass
                
                with self._lock:
                    tiles_data = []
                    for i, t in enumerate(self._last_detected_tiles):
                        bbox_list = [int(x) for x in t.bbox]
                        tiles_data.append({
                            "index": i,
                            "name": t.class_name,
                            "confidence": round(float(t.confidence), 2),
                            "bbox": bbox_list
                        })
                    current_status = self._system_status
                
                # 发布检测结果
                self._publish_status("tiles_update", {
                    "tiles": tiles_data,
                    "timestamp": time.time()
                })
                
                self._publish_status("system_status", {
                    "status": current_status,
                    "tiles_count": len(tiles_data)
                })
                
                elapsed = time.time() - loop_start
                sleep_time = max(0, broadcast_interval - elapsed)
                time.sleep(sleep_time)
                
            except Exception as e:
                print(f'[_detection_loop] 异常: {e}')
                import traceback
                traceback.print_exc()
                time.sleep(1.0)


# ========== 全局服务实例 ==========
bridge_service: Optional[RobotBridgeService] = None


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='HomeBot Mahjong Robot Bridge')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址')
    parser.add_argument('--port', type=int, default=5200, help='监听端口')
    args = parser.parse_args()
    
    global bridge_service
    bridge_service = RobotBridgeService()
    
    if not bridge_service.start():
        print("[WARN] 部分服务启动失败，但仍继续运行")
    
    print(f"\n[Bridge] 静态页面: http://{args.host}:{args.port}/static/robot_trtc.html")
    print("=" * 60)
    
    # 启动 Flask 服务器（在守护线程中，避免 Windows 上的信号问题）
    from werkzeug.serving import make_server
    
    server = make_server(args.host, args.port, app, threaded=True)
    server_thread = Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    
    print(f"[Bridge] HTTP 服务器已启动 http://{args.host}:{args.port}")
    print("[Bridge] 按 Ctrl+C 退出")
    
    # 设置信号处理器
    def signal_handler(signum, frame):
        print("\n[Bridge] 收到中断信号...")
        raise KeyboardInterrupt
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Windows 平台特殊处理
    if sys.platform == 'win32':
        # 使用轮询方式等待中断，避免 signal 在 Windows 上的问题
        try:
            while server_thread.is_alive():
                server_thread.join(timeout=0.1)
        except KeyboardInterrupt:
            pass
    else:
        try:
            # Unix 平台可以直接等待
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    
    # 优雅关闭
    try:
        server.shutdown()
    except Exception:
        pass
    
    if bridge_service:
        bridge_service.stop()


if __name__ == '__main__':
    main()
