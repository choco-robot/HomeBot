"""
HomeBot 麻将机器人 Web 服务器（云服务器版）

功能:
1. 提供玩家访问的 Web 页面 (mahjong.html)
2. 腾讯云 TRTC UserSig 生成接口
3. SocketIO 实时通信与前端交互
4. 通过 MQTT 与机器人本地桥接服务通信（控制指令 / 状态同步）
5. 自动 HTTPS（检测到证书时启用）

使用方法:
    cd software/src
    python -m applications.mahjong_bot
    
访问:
    http://<云服务器IP>:5100/mahjong
    
HTTPS 配置:
    将证书文件放入 applications/mahjong_bot/certs/ 目录
"""

import os
import sys
import time
import json
import base64
import hashlib
import hmac
from threading import Thread, Lock
from typing import Optional, Dict, Any
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

import paho.mqtt.client as mqtt
from configs import get_config

# ========== Flask + SocketIO 应用 ==========
app = Flask(__name__)
app.config['SECRET_KEY'] = 'homebot-mahjong-secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ========== TRTC UserSig 生成 ==========
def gen_trtc_usersig(sdkappid: int, secret_key: str, userid: str, expire: int = 86400) -> str:
    """生成腾讯云 TRTC UserSig"""
    try:
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'common'))
        from TLSSigAPIv2 import TLSSigAPIv2
        api = TLSSigAPIv2(sdkappid, secret_key)
        return api.genUserSig(userid, expire)
    except ImportError:
        print("[WARN] TLSSigAPIv2 未找到，使用备用 UserSig 生成")
        return _gen_usersig_fallback(sdkappid, secret_key, userid, expire)
    except AttributeError as e:
        print(f"[WARN] TLSSigAPIv2 方法错误: {e}，使用备用实现")
        return _gen_usersig_fallback(sdkappid, secret_key, userid, expire)


def _gen_usersig_fallback(sdkappid: int, secret_key: str, userid: str, expire: int = 86400) -> str:
    """备用 UserSig 生成"""
    curr_time = int(time.time())
    m = json.dumps({
        "TLS.ver": "2.0",
        "TLS.identifier": userid,
        "TLS.sdkappid": str(sdkappid),
        "TLS.expire": str(expire),
        "TLS.time": str(curr_time)
    })
    
    def hmacsha256(key: str, content: str) -> bytes:
        return hmac.new(key.encode('utf-8'), content.encode('utf-8'), hashlib.sha256).digest()
    
    sig = hmacsha256(secret_key, m)
    sig_base64 = base64.b64encode(sig).decode('utf-8')
    
    return base64.b64encode(json.dumps({
        "TLS.identifier": userid,
        "TLS.sdkappid": str(sdkappid),
        "TLS.time": str(curr_time),
        "TLS.expire": str(expire),
        "TLS.sig": sig_base64,
        "TLS.ver": "2.0"
    }).encode('utf-8')).decode('utf-8')


# ========== MQTT 桥接服务 ==========
class MahjongCloudService:
    """麻将应用云服务：SocketIO ↔ MQTT 桥接"""
    
    def __init__(self):
        config = get_config()
        self.mahjong_cfg = config.mahjong
        
        # MQTT 配置
        self.mqtt_broker = self.mahjong_cfg.mqtt_broker
        self.mqtt_port = self.mahjong_cfg.mqtt_port
        self.mqtt_use_tls = getattr(self.mahjong_cfg, 'mqtt_use_tls', False)
        self.mqtt_username = self.mahjong_cfg.mqtt_username
        self.mqtt_password = self.mahjong_cfg.mqtt_password
        self.command_topic = self.mahjong_cfg.mqtt_command_topic
        self.status_topic = self.mahjong_cfg.mqtt_status_topic
        self.mqtt_client_id = self.mahjong_cfg.mqtt_client_id_cloud
        
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
        
        self._mqtt_connected = False
        self._lock = Lock()
        
        # 本地缓存状态（用于 /api/status）
        self._system_status = "idle"
        self._selected_tile_index = -1
        self._last_detected_tiles = []
        self._arm_connected = False
        
    def start(self) -> bool:
        """启动 MQTT 连接"""
        try:
            print(f"[MQTT] 连接到 {self.mqtt_broker}:{self.mqtt_port}...")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
            self.mqtt_client.loop_start()
            return True
        except Exception as e:
            print(f"[MQTT] 连接失败: {e}")
            return False
    
    def stop(self):
        """停止 MQTT（强制退出，避免卡死）"""
        try:
            self.mqtt_client.disconnect()
        except Exception as e:
            pass
        try:
            self.mqtt_client.loop_stop(force=True)
        except Exception as e:
            pass
        print("[MQTT] 已断开")
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"[MQTT] 已连接，订阅 {self.status_topic}")
            self._mqtt_connected = True
            client.subscribe(self.status_topic, qos=0)
        else:
            print(f"[MQTT] 连接失败，返回码: {rc}")
            self._mqtt_connected = False
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        print(f"[MQTT] 断开连接，返回码: {rc}")
        self._mqtt_connected = False
    
    def _on_mqtt_message(self, client, userdata, msg):
        """收到 MQTT 状态消息，转发到 SocketIO"""
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            msg_type = payload.get('type')
            data = payload.get('data', {})
            
            # 更新本地缓存状态
            with self._lock:
                if msg_type == 'tiles_update':
                    self._last_detected_tiles = data.get('tiles', [])
                elif msg_type == 'system_status':
                    self._system_status = data.get('status', 'idle')
                elif msg_type == 'tile_selected' and data.get('success'):
                    self._selected_tile_index = data.get('index', -1)
                elif msg_type == 'arm_update':
                    self._arm_connected = data.get('success', False)
            
            # 转发到所有 SocketIO 客户端
            if msg_type:
                socketio.emit(msg_type, data)
                
        except Exception as e:
            print(f"[MQTT] 处理状态消息异常: {e}")
    
    def publish_command(self, cmd: str, data: dict):
        """发布控制指令到 MQTT"""
        payload = json.dumps({
            "cmd": cmd,
            "data": data,
            "timestamp": time.time()
        })
        if self._mqtt_connected:
            self.mqtt_client.publish(self.command_topic, payload, qos=1)
        else:
            print(f"[WARN] MQTT 未连接，无法发送指令: {cmd}")


# 全局服务实例
def _create_service():
    try:
        return MahjongCloudService()
    except Exception as e:
        print(f"\n{'='*60}")
        print(f"[FATAL] 初始化 MahjongCloudService 失败: {e}")
        print(f"{'='*60}")
        import traceback
        traceback.print_exc()
        print(f"{'='*60}\n")
        raise

service = _create_service()


# ========== Flask 路由 ==========

@app.route('/mahjong')
def index():
    """麻将主页面"""
    config = get_config()
    return render_template('mahjong.html',
                         trtc_sdkappid=config.trtc.sdk_app_id,
                         trtc_room_id=config.trtc.room_id)


@app.route('/api/trtc/usersig')
def api_trtc_usersig():
    """生成 TRTC UserSig"""
    config = get_config()
    userid = request.args.get('userid', '')
    expire = request.args.get('expire', 86400, type=int)
    
    if not userid:
        return jsonify({"success": False, "error": "userid is required"}), 400
    
    if not config.trtc.sdk_app_id or not config.trtc.secret_key:
        return jsonify({"success": False, "error": "TRTC not configured"}), 500
    
    try:
        user_sig = gen_trtc_usersig(
            config.trtc.sdk_app_id,
            config.trtc.secret_key,
            userid,
            expire
        )
        return jsonify({
            "success": True,
            "sdkAppId": config.trtc.sdk_app_id,
            "userSig": user_sig,
            "userId": userid,
            "roomId": config.trtc.room_id,
            "expire": expire
        })
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route('/api/status')
def api_status():
    """获取系统状态（本地缓存）"""
    with service._lock:
        return jsonify({
            "success": True,
            "system_status": service._system_status,
            "selected_tile": service._selected_tile_index,
            "detected_tiles": [t.get('name', '') for t in service._last_detected_tiles],
            "arm_connected": service._arm_connected,
        })


# ========== SocketIO 事件 ==========

@socketio.on('connect')
def handle_connect():
    print(f'[SocketIO] 客户端已连接: {request.sid}')
    emit('server_hello', {
        "message": "欢迎连接 HomeBot 麻将机器人",
        "status": service._system_status,
        "timestamp": time.time()
    })


@socketio.on('disconnect')
def handle_disconnect():
    print(f'[SocketIO] 客户端已断开: {request.sid}')


@socketio.on('select_tile')
def handle_select_tile(data):
    """用户点击选中某张牌"""
    index = data.get('index', -1)
    with service._lock:
        service._selected_tile_index = index
    service.publish_command('select_tile', {'index': index})


@socketio.on('play_tile')
def handle_play_tile(data):
    """用户确认出牌"""
    service.publish_command('play_tile', {})


@socketio.on('arm_joystick')
def handle_arm_joystick(data):
    """机械臂摇杆控制（调试用）"""
    service.publish_command('arm_joystick', {
        'x': data.get('x', 0.0),
        'y': data.get('y', 0.0),
        'axis': data.get('axis', 'base')
    })


@socketio.on('gripper_toggle')
def handle_gripper_toggle(data):
    """夹爪控制"""
    service.publish_command('gripper_toggle', {
        'closed': data.get('closed', False)
    })


@socketio.on('arm_home')
def handle_arm_home():
    """机械臂归位"""
    service.publish_command('arm_home', {})


@socketio.on('get_status')
def handle_get_status():
    """主动查询状态"""
    with service._lock:
        emit('system_status', {
            "status": service._system_status,
            "selected_tile": service._selected_tile_index,
            "tiles_count": len(service._last_detected_tiles),
            "arm_connected": service._arm_connected,
        })


# ========== 主入口 ==========

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='HomeBot Mahjong Bot Cloud Server')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址')
    parser.add_argument('--port', type=int, default=5100, help='监听端口')
    args = parser.parse_args()
    
    print("=" * 60)
    print("HomeBot 麻将机器人 Web 服务（云服务器版）")
    print("=" * 60)
    
    # 启动 MQTT 连接
    if not service.start():
        print("[WARN] MQTT 连接失败，服务仍会继续运行但无法与机器人通信")
    
    # 准备 SSL 上下文
    ssl_context = None
    cert_dir = Path(__file__).parent / 'certs'
    
    cert_combinations = [
        ('cert.pem', 'key.pem'),
        ('server.crt', 'server.key'),
        ('localhost.crt', 'localhost.key'),
    ]
    
    cert_file = None
    key_file = None
    
    for cert_name, key_name in cert_combinations:
        cf = cert_dir / cert_name
        kf = cert_dir / key_name
        if cf.exists() and kf.exists():
            cert_file = cf
            key_file = kf
            break
    
    if cert_file and key_file:
        ssl_context = (str(cert_file), str(key_file))
        print(f"\n[HTTPS] 已自动启用 SSL")
        print(f"  证书: {cert_file}")
        print(f"  私钥: {key_file}")
        print(f"\n请访问: https://{args.host}:{args.port}/mahjong")
    else:
        print(f"\n[HTTP] 以 HTTP 模式运行")
        print(f"\n请访问: http://{args.host}:{args.port}/mahjong")
    
    print("=" * 60)
    
    try:
        socketio.run(app, host=args.host, port=args.port, debug=True, 
                     use_reloader=True, ssl_context=ssl_context)
    except KeyboardInterrupt:
        print("\n[MahjongBot] 正在关闭...")
    finally:
        service.stop()


if __name__ == '__main__':
    main()
