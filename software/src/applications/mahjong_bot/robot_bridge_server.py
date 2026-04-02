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
from threading import Thread, Lock
from typing import Optional, Dict, Any
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from flask import Flask, send_from_directory, request, jsonify

# MQTT
import paho.mqtt.client as mqtt

# 导入现有组件
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'remote_control'))
from web_server import VideoStreamClient, ArmClient

from applications.mahjong_bot.detector import MahjongDetector
from applications.mahjong_bot.arm_controller import MahjongArmController
from configs import get_config

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


# ========== 机器人桥接服务 ==========
class RobotBridgeService:
    """机器人本地桥接服务：ZMQ + MQTT 桥接"""
    
    def __init__(self):
        config = get_config()
        self.mahjong_cfg = config.mahjong
        
        # MQTT 配置
        self.mqtt_broker = self.mahjong_cfg.mqtt_broker
        self.mqtt_port = self.mahjong_cfg.mqtt_port
        self.mqtt_username = self.mahjong_cfg.mqtt_username
        self.mqtt_password = self.mahjong_cfg.mqtt_password
        self.command_topic = self.mahjong_cfg.mqtt_command_topic
        self.status_topic = self.mahjong_cfg.mqtt_status_topic
        self.mqtt_client_id = self.mahjong_cfg.mqtt_client_id_robot
        
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        if self.mqtt_username:
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        
        # ZMQ 客户端
        self.front_video = VideoStreamClient(self.mahjong_cfg.front_vision_addr)
        self.arm_client = ArmClient(self.mahjong_cfg.arm_service_addr)
        
        # 机械臂控制器（用于出牌流程）
        self.arm_controller = MahjongArmController(self.mahjong_cfg.arm_service_addr)
        
        # 检测器
        software_dir = Path(__file__).parent.parent.parent.parent
        model_path = str(software_dir / self.mahjong_cfg.detector_model_path)
        self.detector = MahjongDetector(
            model_path=model_path,
            conf_threshold=self.mahjong_cfg.detector_conf_threshold,
            inference_size=self.mahjong_cfg.detector_inference_size,
            device="cuda",
            use_roboflow_classes=True
        )
        
        # 状态
        self._running = False
        self._lock = Lock()
        self._last_detected_tiles: list = []
        self._selected_tile_index: int = -1
        self._system_status: str = "idle"
        self._mqtt_connected = False
        
    def start(self) -> bool:
        """启动所有服务"""
        print("=" * 60)
        print("HomeBot 麻将机器人本地桥接服务")
        print("=" * 60)
        
        # 连接 MQTT
        try:
            print(f"[MQTT] 连接到 {self.mqtt_broker}:{self.mqtt_port}...")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
            self.mqtt_client.loop_start()
        except Exception as e:
            print(f"[MQTT] 连接失败: {e}")
            return False
        
        # 连接前置视频
        ok1 = self.front_video.start()
        print(f"[Bridge] 前置视频: {'OK' if ok1 else 'FAIL'}")
        
        # 连接机械臂
        ok2 = self.arm_client.connect()
        print(f"[Bridge] 机械臂客户端: {'OK' if ok2 else 'FAIL'}")
        
        # 连接机械臂控制器
        ok3 = self.arm_controller.connect_arm()
        print(f"[Bridge] 机械臂控制器: {'OK' if ok3 else 'FAIL'}")
        
        # 初始化检测器
        ok4 = self.detector.initialize()
        print(f"[Bridge] 检测器: {'OK' if ok4 else 'FAIL'}")
        
        self._running = True
        
        # 启动检测循环
        self._detect_thread = Thread(target=self._detection_loop, daemon=True)
        self._detect_thread.start()
        
        return True
    
    def stop(self):
        """停止所有服务"""
        print("\n[Bridge] 正在关闭...")
        self._running = False
        
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.front_video.stop()
        self.arm_client.disconnect()
        self.arm_controller.disconnect_arm()
        self.detector.release()
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
            elif cmd == 'arm_joystick':
                self._handle_arm_joystick(data)
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
                
                # 更新控制器中的选中状态
                # MahjongArmController 使用 tile_id 来选牌
                # 这里我们使用索引作为 tile_id（简化）
                if tile and hasattr(tile, 'tile_id'):
                    self.arm_controller.select_tile(tile.tile_id)
                
                self._publish_status("tile_selected", {
                    "success": True,
                    "index": index,
                    "tile": {
                        "name": tile.class_name,
                        "bbox": [int(x) for x in tile.bbox],
                        "confidence": round(float(tile.confidence), 2)
                    }
                })
                print(f'[Bridge] 选中第 {index} 张牌: {tile.class_name}')
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
                # 使用 MahjongArmController 执行出牌
                # 先根据图像坐标选中对应的牌
                tile = None
                with self._lock:
                    if 0 <= self._selected_tile_index < len(self._last_detected_tiles):
                        tile = self._last_detected_tiles[self._selected_tile_index]
                
                if tile:
                    # 通过图像位置选中牌
                    cx, cy = tile.center
                    tile_id = self.arm_controller.select_tile_by_image_position(cx, cy)
                    if tile_id is not None:
                        success = self.arm_controller.discard_selected_tile(
                            on_step=lambda name, cur, total: print(f"[Arm] {name} ({cur}/{total})")
                        )
                        print(f"[Bridge] 出牌执行结果: {'成功' if success else '失败'}")
                    else:
                        print(f"[Bridge] 无法根据图像位置选中牌")
                
                with self._lock:
                    self._system_status = "idle"
                    self._selected_tile_index = -1
                
                self._publish_status("system_status", {
                    "status": "idle",
                    "message": "出牌完成"
                })
                
            except Exception as e:
                print(f"[Bridge] 出牌执行异常: {e}")
                import traceback
                traceback.print_exc()
                with self._lock:
                    self._system_status = "idle"
                self._publish_status("system_status", {
                    "status": "idle",
                    "message": f"出牌异常: {e}"
                })
        
        Thread(target=execute_arm, daemon=True).start()
    
    def _handle_arm_home(self, data):
        if self.arm_client and self.arm_client._connected:
            result = self.arm_client.move_to_home()
            self._publish_status("arm_update", {
                "success": result.get('success', False),
                "message": "机械臂已归位"
            })
        else:
            self._publish_status("arm_update", {"success": False, "message": "机械臂未连接"})
    
    def _handle_gripper_toggle(self, data):
        closed = data.get('closed', False)
        if self.arm_client and self.arm_client._connected:
            result = self.arm_client.set_gripper(closed)
            self._publish_status("gripper_update", result)
        else:
            self._publish_status("gripper_update", {"success": False, "message": "机械臂未连接"})
    
    def _handle_arm_joystick(self, data):
        x = data.get('x', 0.0)
        y = data.get('y', 0.0)
        axis = data.get('axis', 'base')
        
        if self.arm_client and self.arm_client._connected:
            result = self.arm_client.process_joystick(x, y, axis)
            self._publish_status("arm_update", {
                "success": result.get('success', False),
                "angles": result.get('angles', {}),
                "message": result.get('message', '')
            })
        else:
            self._publish_status("arm_update", {"success": False, "message": "机械臂未连接"})
    
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
                    frame_bytes = self.front_video.get_frame()
                    if frame_bytes is not None:
                        nparr = np.frombuffer(frame_bytes, np.uint8)
                        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                        if img is not None:
                            detections = detector.detect(img)
                            
                            with self._lock:
                                self._last_detected_tiles = detections
                                if self._system_status != "executing":
                                    self._system_status = "detecting" if detections else "idle"
                            
                            # 更新机械臂控制器中的手牌状态
                            detected_data = []
                            for i, t in enumerate(detections):
                                cx, cy = t.center
                                detected_data.append({
                                    'tile_id': i,
                                    'class_name': t.class_name,
                                    'image_x': cx,
                                    'image_y': cy,
                                    'confidence': t.confidence
                                })
                            if detected_data:
                                self.arm_controller.update_hand_detection(detected_data)
                
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
    
    try:
        app.run(host=args.host, port=args.port, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        print("\n[Bridge] 收到中断信号，正在关闭...")
    finally:
        bridge_service.stop()


if __name__ == '__main__':
    main()
