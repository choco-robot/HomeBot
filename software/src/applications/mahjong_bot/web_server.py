"""
HomeBot 麻将机器人 Web 服务器

功能:
1. 双路 MJPEG 视频流 (顶置摄像头 5560 + 前置摄像头 5562)
2. 腾讯云 TRTC UserSig 生成接口
3. SocketIO 实时通信 (选牌、出牌、状态同步)
4. 集成 ArmClient 与机械臂服务通信

使用方法:
    cd software/src
    python -m applications.mahjong_bot
    
访问:
    http://<机器人IP>:5100/mahjong
"""

import os
import sys
import time
import json
import base64
import hashlib
import hmac
from threading import Thread, Lock
from typing import Optional, Dict, Any, Generator
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from flask import Flask, render_template, Response, request, jsonify
from flask_socketio import SocketIO, emit

import zmq
from configs import get_config

# ========== 导入现有组件 ==========
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'remote_control'))
from web_server import VideoStreamClient, ArmClient

from applications.mahjong_bot.detector import MahjongDetector

DEFAULT_TOP_VISION_ADDR = "tcp://127.0.0.1:5560"
DEFAULT_FRONT_VISION_ADDR = "tcp://127.0.0.1:5562"
DEFAULT_ARM_ADDR = "tcp://127.0.0.1:5557"

# ========== Flask + SocketIO 应用 ==========
app = Flask(__name__)
app.config['SECRET_KEY'] = 'homebot-mahjong-secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')


# ========== TRTC UserSig 生成 ==========
def gen_trtc_usersig(sdkappid: int, secret_key: str, userid: str, expire: int = 86400) -> str:
    """
    生成腾讯云 TRTC UserSig
    
    Args:
        sdkappid: 腾讯云 SDKAppID
        secret_key: 腾讯云 SecretKey
        userid: 用户ID
        expire: 过期时间（秒）
    
    Returns:
        base64 编码的 UserSig
    """
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


# ========== 全局服务实例 ==========
class MahjongService:
    """麻将应用全局服务"""
    
    def __init__(self):
        config = get_config()
        self.mahjong_cfg = config.mahjong
        self.trtc_cfg = config.trtc
        
        # 双路视频客户端
        self.top_video = VideoStreamClient(self.mahjong_cfg.top_vision_addr)
        self.front_video = VideoStreamClient(self.mahjong_cfg.front_vision_addr)
        
        # 机械臂客户端
        self.arm_client = ArmClient(self.mahjong_cfg.arm_service_addr)
        
        # 状态
        self._running = False
        self._lock = Lock()
        self._last_detected_tiles: list = []  # 最近一次检测到的手牌
        self._selected_tile_index: int = -1   # 用户选中的牌索引
        self._system_status: str = "idle"     # idle/detecting/executing
        
        # 检测器（修复模型路径，确保指向 software/models/）
        software_dir = Path(__file__).parent.parent.parent.parent
        model_path = str(software_dir / self.mahjong_cfg.detector_model_path)
        self.detector = MahjongDetector(
            model_path=model_path,
            conf_threshold=self.mahjong_cfg.detector_conf_threshold,
            inference_size=self.mahjong_cfg.detector_inference_size,
            device="cuda",
            use_roboflow_classes=True
        )
        
    def start(self) -> bool:
        """启动所有客户端连接"""
        ok1 = self.top_video.start()
        ok2 = self.front_video.start()
        ok3 = self.arm_client.connect()
        ok4 = self.detector.initialize()
        
        self._running = True
        print(f"[MahjongService] 顶置视频: {'OK' if ok1 else 'FAIL'}")
        print(f"[MahjongService] 前置视频: {'OK' if ok2 else 'FAIL'}")
        print(f"[MahjongService] 机械臂: {'OK' if ok3 else 'FAIL'}")
        print(f"[MahjongService] 检测器: {'OK' if ok4 else 'FAIL'}")
        return ok1 or ok2  # 至少一路视频成功即可启动
    
    def stop(self):
        """停止所有服务"""
        self._running = False
        self.top_video.stop()
        self.front_video.stop()
        self.arm_client.disconnect()
        self.detector.release()


# 全局服务实例
def _create_service():
    """创建服务实例，带错误处理"""
    try:
        return MahjongService()
    except Exception as e:
        print(f"\n{'='*60}")
        print(f"[FATAL] 初始化 MahjongService 失败: {e}")
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


@app.route('/video_feed_top')
def video_feed_top():
    """顶置摄像头 MJPEG 流"""
    # start() 增加引用计数，generate_mjpeg() 结束时会自动调用 stop() 减少引用
    service.top_video.start()
    return Response(service.top_video.generate_mjpeg(), 
                   mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/video_feed_front')
def video_feed_front():
    """前置摄像头 MJPEG 流"""
    service.front_video.start()
    return Response(service.front_video.generate_mjpeg(), 
                   mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/trtc/usersig')
def api_trtc_usersig():
    """
    生成 TRTC UserSig
    
    Query params:
        userid: 用户ID（必填）
        expire: 过期时间秒数（可选，默认86400）
    """
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
    """获取系统状态"""
    with service._lock:
        return jsonify({
            "success": True,
            "system_status": service._system_status,
            "selected_tile": service._selected_tile_index,
            "detected_tiles": [t.class_name for t in service._last_detected_tiles],
            "arm_connected": service.arm_client._connected if service.arm_client else False,
            "top_video_active": service.top_video._connected if service.top_video else False,
            "front_video_active": service.front_video._connected if service.front_video else False,
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
        tiles = service._last_detected_tiles
        if 0 <= index < len(tiles):
            service._selected_tile_index = index
            tile = tiles[index]
            emit('tile_selected', {
                "success": True,
                "index": index,
                "tile": {
                    "name": tile.class_name,
                    "bbox": tile.bbox,
                    "confidence": tile.confidence
                }
            }, broadcast=False)
            print(f'[Mahjong] 选中第 {index} 张牌: {tile.class_name}')
        else:
            emit('tile_selected', {
                "success": False,
                "error": "无效的牌索引"
            }, broadcast=False)


@socketio.on('play_tile')
def handle_play_tile(data):
    """用户确认出牌"""
    with service._lock:
        index = service._selected_tile_index
        tiles = service._last_detected_tiles
        
        if index < 0 or index >= len(tiles):
            emit('play_result', {
                "success": False,
                "error": "未选择有效的牌"
            })
            return
        
        tile = tiles[index]
        service._system_status = "executing"
        
        # TODO: 将图像坐标通过 Homography 转换为机械臂坐标
        # 这里先返回模拟执行结果
        print(f'[Mahjong] 执行出牌: {tile.class_name} @ {tile.bbox}')
        
        emit('play_result', {
            "success": True,
            "message": f"开始执行出牌: {tile.class_name}",
            "tile": {
                "name": tile.class_name,
                "center": tile.center
            }
        })
        
        # 广播给所有客户端
        socketio.emit('system_status', {
            "status": "executing",
            "message": f"正在打出 {tile.class_name}..."
        })
        
        # TODO: 实际调用动作规划器执行机械臂动作
        # 完成后重置状态
        # service._system_status = "idle"
        # service._selected_tile_index = -1


@socketio.on('arm_joystick')
def handle_arm_joystick(data):
    """机械臂摇杆控制（调试用）"""
    x = data.get('x', 0.0)
    y = data.get('y', 0.0)
    axis = data.get('axis', 'base')
    
    if service.arm_client and service.arm_client._connected:
        result = service.arm_client.process_joystick(x, y, axis)
        emit('arm_update', {
            "success": result.get('success', False),
            "angles": result.get('angles', {}),
            "message": result.get('message', '')
        })
    else:
        emit('arm_update', {"success": False, "message": "机械臂未连接"})


@socketio.on('gripper_toggle')
def handle_gripper_toggle(data):
    """夹爪控制"""
    closed = data.get('closed', False)
    if service.arm_client and service.arm_client._connected:
        result = service.arm_client.set_gripper(closed)
        emit('gripper_update', result)
    else:
        emit('gripper_update', {"success": False, "message": "机械臂未连接"})


@socketio.on('arm_home')
def handle_arm_home():
    """机械臂归位"""
    if service.arm_client and service.arm_client._connected:
        result = service.arm_client.move_to_home()
        emit('arm_update', {
            "success": result.get('success', False),
            "message": "机械臂已归位"
        })
    else:
        emit('arm_update', {"success": False, "message": "机械臂未连接"})


@socketio.on('get_status')
def handle_get_status():
    """主动查询状态"""
    with service._lock:
        emit('system_status', {
            "status": service._system_status,
            "selected_tile": service._selected_tile_index,
            "tiles_count": len(service._last_detected_tiles),
            "arm_connected": service.arm_client._connected if service.arm_client else False,
        })


def broadcast_detection_loop():
    """
    后台线程：定期从顶置视频流获取帧，执行麻将牌检测，并广播结果到前端
    """
    import cv2
    import numpy as np
    
    detector = service.detector
    broadcast_interval = 1.0  # 目标广播间隔（秒）
    
    while service._running:
        try:
            loop_start = time.time()
            detections = []
            
            # 若检测器已初始化，执行推理
            if detector._initialized:
                frame_bytes = service.top_video.get_frame()
                if frame_bytes is not None:
                    nparr = np.frombuffer(frame_bytes, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if img is not None:
                        detections = detector.detect(img)
                        
                        # 更新检测结果和状态（在锁内）
                        with service._lock:
                            service._last_detected_tiles = detections
                            # 仅在非执行状态时更新为 detecting/idle
                            if service._system_status != "executing":
                                service._system_status = "detecting" if detections else "idle"
            
            # 构造广播数据（在锁内读取状态）
            with service._lock:
                tiles_data = []
                for i, t in enumerate(service._last_detected_tiles):
                    # 将 numpy int64 转换为 Python int，确保 JSON 可序列化
                    bbox_list = [int(x) for x in t.bbox]
                    tiles_data.append({
                        "index": i,
                        "name": t.class_name,
                        "confidence": round(float(t.confidence), 2),
                        "bbox": bbox_list
                    })
                current_status = service._system_status
            
            # 广播检测结果
            socketio.emit('tiles_update', {
                "tiles": tiles_data,
                "timestamp": time.time()
            })
            
            # 广播当前状态
            socketio.emit('system_status', {
                "status": current_status,
                "tiles_count": len(tiles_data)
            })
            
            # 精确控制广播间隔
            elapsed = time.time() - loop_start
            sleep_time = max(0, broadcast_interval - elapsed)
            time.sleep(sleep_time)
            
        except Exception as e:
            print(f'[broadcast_detection_loop] 异常: {e}')
            time.sleep(1.0)


# ========== 主入口 ==========

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='HomeBot Mahjong Bot')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址')
    parser.add_argument('--port', type=int, default=5100, help='监听端口')
    args = parser.parse_args()
    
    print("=" * 60)
    print("HomeBot 麻将机器人 Web 服务")
    print("=" * 60)
    
    # 启动视频和机械臂连接
    if not service.start():
        print("[WARN] 部分服务启动失败，但仍继续运行 Web 服务器")
    
    # 启动检测广播线程
    broadcast_thread = Thread(target=broadcast_detection_loop, daemon=True)
    broadcast_thread.start()
    
    print(f"\n请访问: http://{args.host}:{args.port}/mahjong")
    print(f"机器人 TRTC 终端: file://{os.path.abspath('static/robot_trtc.html')}")
    print("=" * 60)
    
    try:
        socketio.run(app, host=args.host, port=args.port, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        print("\n[MahjongBot] 正在关闭...")
    finally:
        service.stop()


if __name__ == '__main__':
    main()
