"""
机器人网页控制端 - Web服务器
提供HTTP服务和WebSocket实时通信，桥接前端与底盘服务
支持摄像头图像流显示

使用方法:
    python -m applications.remote_control
    
    python -m applications.remote_control --arbiter tcp://127.0.0.1:5556 --vision tcp://127.0.0.1:5560
"""
import os
import sys
import time
import platform
from threading import Thread, Lock, Event
from typing import Optional, Dict, Any, Generator
import subprocess
import signal

from flask import Flask, render_template, send_from_directory, request, Response
from flask_socketio import SocketIO, emit

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

import zmq

DEFAULT_ARBITER_ADDR = "tcp://127.0.0.1:5556"
DEFAULT_VISION_ADDR = "tcp://127.0.0.1:5560"
DEFAULT_ARM_ADDR = "tcp://127.0.0.1:5557"


class ZMQClient:
    """
    ZeroMQ客户端 - 直接连接ChassisService
    """
    
    SOURCE_NAME = "web"
    PRIORITY = 1
    
    def __init__(self, arbiter_addr: str = DEFAULT_ARBITER_ADDR):
        self.arbiter_addr = arbiter_addr
        self._context: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None
        self._connected = False
        
    def connect(self) -> bool:
        """连接到底盘服务"""
        try:
            self._context = zmq.Context()
            self._socket = self._context.socket(zmq.REQ)
            self._socket.setsockopt(zmq.RCVTIMEO, 100)
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.connect(self.arbiter_addr)
            self._connected = True
            print(f"[ZMQ] 已连接到底盘服务: {self.arbiter_addr}")
            return True
        except Exception as e:
            print(f"[ZMQ] 连接失败: {e}")
            return False
    
    def send_command(self, vx: float, vy: float, vz: float, source: str = None, priority: int = None) -> Dict[str, Any]:
        """发送速度指令 - 使用 poll 确保不会阻塞"""
        if not self._connected or self._socket is None:
            return {"success": False, "message": "未连接"}
        
        request = {
            "source": source or self.SOURCE_NAME,
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "priority": priority if priority is not None else self.PRIORITY
        }
        
        try:
            # 使用 poll 检查 socket 是否可写
            if self._socket.poll(100, zmq.POLLOUT):
                self._socket.send_json(request)
                # 等待响应
                if self._socket.poll(100, zmq.POLLIN):
                    response = self._socket.recv_json()
                    return response
                else:
                    print("[ZMQ] 接收响应超时")
                    self._reset_socket()
                    return {"success": False, "message": "接收超时"}
            else:
                print("[ZMQ] socket 不可写")
                self._reset_socket()
                return {"success": False, "message": "socket 不可写"}
                
        except zmq.error.ZMQError as e:
            err_msg = str(e)
            if "Operation cannot be accomplished in current state" in err_msg:
                print(f"[ZMQ] socket 状态错误，重置连接")
                self._reset_socket()
                # 重试一次
                try:
                    self._socket.send_json(request)
                    response = self._socket.recv_json()
                    return response
                except Exception as e2:
                    print(f"[ZMQ] 重试失败: {e2}")
                    return {"success": False, "message": str(e2)}
            else:
                print(f"[ZMQ] 发送失败: {e}")
                self._reset_socket()
                return {"success": False, "message": err_msg}
        except Exception as e:
            print(f"[ZMQ] 发送异常: {e}")
            self._reset_socket()
            return {"success": False, "message": str(e)}
    
    def send_emergency_stop(self) -> Dict[str, Any]:
        """发送紧急停止命令"""
        return self.send_command(0.0, 0.0, 0.0, source="emergency", priority=4)
    
    def send_home(self) -> Dict[str, Any]:
        """发送归位命令"""
        return self.send_command(0.0, 0.0, 0.0, source="home", priority=0)
    
    def _reset_socket(self):
        """重置socket"""
        if self._socket:
            self._socket.close()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, 100)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(self.arbiter_addr)
    
    def disconnect(self):
        """断开连接"""
        if self._socket:
            self._socket.close()
        if self._context:
            self._context.term()
        self._connected = False


class ArmClient:
    """
    机械臂ZeroMQ客户端 - 连接ArmService
    """
    
    SOURCE_NAME = "web"
    PRIORITY = 1
    
    def __init__(self, arm_addr: str = DEFAULT_ARM_ADDR):
        self.arm_addr = arm_addr
        self._context: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None
        self._connected = False
        
    def connect(self) -> bool:
        """连接到机械臂服务"""
        try:
            self._context = zmq.Context()
            self._socket = self._context.socket(zmq.REQ)
            self._socket.setsockopt(zmq.RCVTIMEO, 500)  # 500ms超时
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.connect(self.arm_addr)
            self._connected = True
            print(f"[Arm] 已连接到机械臂服务: {self.arm_addr}")
            return True
        except Exception as e:
            print(f"[Arm] 连接失败: {e}")
            return False
    
    def send_command(self, joints: Dict[str, float], speed: int = 1000, 
                     source: str = None, priority: int = None) -> Dict[str, Any]:
        """发送机械臂关节角度指令"""
        if not self._connected or self._socket is None:
            return {"success": False, "message": "未连接"}
        
        request = {
            "source": source or self.SOURCE_NAME,
            "joints": joints,
            "speed": speed,
            "priority": priority if priority is not None else self.PRIORITY
        }
        
        try:
            if self._socket.poll(500, zmq.POLLOUT):
                self._socket.send_json(request)
                if self._socket.poll(500, zmq.POLLIN):
                    response = self._socket.recv_json()
                    return response
                else:
                    print("[Arm] 接收响应超时")
                    return {"success": False, "message": "接收超时"}
            else:
                print("[Arm] socket 不可写")
                return {"success": False, "message": "socket 不可写"}
        except Exception as e:
            print(f"[Arm] 发送失败: {e}")
            return {"success": False, "message": str(e)}
    
    def move_to_home(self) -> Dict[str, Any]:
        """
        机械臂回到复位位置（rest_position）
        从配置文件读取休息位置
        """
        # 导入配置获取休息位置
        try:
            import os
            import sys
            current_dir = os.path.dirname(os.path.abspath(__file__))
            sys.path.insert(0, os.path.join(current_dir, '..', '..'))
            from configs import get_config
            config = get_config()
            rest_position = config.arm.rest_position
            print(f"[Arm] 归位到休息位置: {rest_position}")
            return self.send_command(rest_position, speed=800, source="web", priority=2)
        except Exception as e:
            print(f"[Arm] 获取休息位置失败: {e}")
            # 使用默认位置
            default_home = {
                "base": 0,
                "shoulder": 45,
                "elbow": 90,
                "wrist_flex": 0,
                "wrist_roll": 0,
                "gripper": 45
            }
            return self.send_command(default_home, speed=800, source="web", priority=2)
    
    def disconnect(self):
        """断开连接"""
        if self._socket:
            self._socket.close()
        if self._context:
            self._context.term()
        self._connected = False


class VideoStreamClient:
    """
    视频流客户端 - 订阅VisionService的图像流并转换为MJPEG格式
    """
    
    def __init__(self, vision_addr: str = DEFAULT_VISION_ADDR):
        self.vision_addr = vision_addr
        self._context: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None
        self._connected = False
        self._running = False
        self._thread: Optional[Thread] = None
        self._latest_frame: Optional[bytes] = None
        self._frame_lock = Lock()
        
    def connect(self) -> bool:
        """连接到VisionService"""
        try:
            self._context = zmq.Context()
            self._socket = self._context.socket(zmq.SUB)
            self._socket.setsockopt(zmq.RCVTIMEO, 1000)
            self._socket.setsockopt(zmq.SUBSCRIBE, b"")
            self._socket.connect(self.vision_addr)
            self._connected = True
            print(f"[Video] 已连接到视觉服务: {self.vision_addr}")
            return True
        except Exception as e:
            print(f"[Video] 连接失败: {e}")
            return False
    
    def start(self) -> bool:
        """启动视频流接收线程"""
        if not self._connected:
            if not self.connect():
                return False
        self._running = True
        self._thread = Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        print("[Video] 视频流接收已启动")
        return True
    
    def stop(self):
        """停止视频流接收"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._socket:
            self._socket.close()
        if self._context:
            self._context.term()
        self._connected = False
        print("[Video] 视频流接收已停止")
    
    def _receive_loop(self):
        """后台线程接收视频帧"""
        import cv2
        import numpy as np
        
        retry_count = 0
        max_retries = 5
        frame_count = 0
        last_log = time.time()
        
        print(f"[Video] Receive loop started, connecting to {self.vision_addr}")
        
        while self._running:
            try:
                if not self._connected:
                    if retry_count < max_retries:
                        retry_count += 1
                        print(f"[Video] Reconnecting... ({retry_count}/{max_retries})")
                        if self.connect():
                            retry_count = 0
                            print("[Video] Connected successfully")
                        else:
                            time.sleep(2)
                        continue
                    else:
                        print("[Video] Max retries exceeded, stopping receiver")
                        break
                
                # 接收 multipart 消息 [frame_id, jpeg_bytes]
                parts = self._socket.recv_multipart()
                if len(parts) >= 2:
                    jpeg_bytes = parts[1]
                    with self._frame_lock:
                        self._latest_frame = jpeg_bytes
                    frame_count += 1
                    
                    # 每5秒打印一次统计
                    if time.time() - last_log > 5:
                        fps = frame_count / 5
                        print(f"[Video] Receiving {fps:.1f} fps, {len(jpeg_bytes)} bytes/frame")
                        frame_count = 0
                        last_log = time.time()
                
            except zmq.error.Again:
                # 超时，继续
                continue
            except Exception as e:
                print(f"[Video] Receive error: {e}")
                self._connected = False
                time.sleep(1)
    
    def get_frame(self) -> Optional[bytes]:
        """获取最新的JPEG帧数据"""
        with self._frame_lock:
            return self._latest_frame
    
    def generate_mjpeg(self) -> Generator[bytes, None, None]:
        """生成MJPEG流用于HTTP响应"""
        import cv2
        import numpy as np
        
        # 等待第一帧
        timeout = 5.0
        start = time.time()
        while self._running and not self.get_frame():
            if time.time() - start > timeout:
                print("[Video] Timeout waiting for first frame")
                break
            time.sleep(0.1)
        
        frame_count = 0
        last_log = time.time()
        
        while self._running:
            frame = self.get_frame()
            if frame:
                frame_count += 1
                if time.time() - last_log > 5:
                    print(f"[Video] Streaming {frame_count} frames")
                    frame_count = 0
                    last_log = time.time()
                
                # 正确的 MJPEG multipart 格式
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n'
                       b'Content-Length: ' + str(len(frame)).encode() + b'\r\n'
                       b'\r\n' + frame + b'\r\n')
            else:
                # 没有帧时发送占位图片
                img = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(img, "No Signal", (80, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                ret, buf = cv2.imencode('.jpg', img)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n'
                           b'Content-Length: ' + str(len(buf)).encode() + b'\r\n'
                           b'\r\n' + buf.tobytes() + b'\r\n')
                time.sleep(0.5)


class ZMQBridge:
    """ZeroMQ桥接器 - 后台线程发送指令，使用队列避免并发问题"""
    
    def __init__(self, arbiter_addr: Optional[str] = None, arm_addr: Optional[str] = None):
        self.arbiter_addr = arbiter_addr or DEFAULT_ARBITER_ADDR
        self.arm_addr = arm_addr or DEFAULT_ARM_ADDR
        self.client = ZMQClient(self.arbiter_addr)
        self.arm_client = ArmClient(self.arm_addr)
        self._running = False
        self._thread: Optional[Thread] = None
        self._current_cmd = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        self._cmd_lock = Lock()
        self._connected = False
        self._last_response: Optional[Dict[str, Any]] = None
        
        # 命令队列 - 用于紧急命令和归位命令
        import queue
        self._cmd_queue = queue.Queue(maxsize=10)
        
    def connect(self) -> bool:
        chassis_ok = self.client.connect()
        arm_ok = self.arm_client.connect()
        if chassis_ok:
            self._connected = True
            print("[Bridge] 底盘服务连接成功")
        if arm_ok:
            print("[Bridge] 机械臂服务连接成功")
        return chassis_ok
    
    def start(self):
        if not self._connected:
            self.connect()
        self._running = True
        self._thread = Thread(target=self._send_loop, daemon=True)
        self._thread.start()
        print("[Bridge] 已启动")
        return True
    
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        self.client.disconnect()
        self.arm_client.disconnect()
        print("[Bridge] 已停止")
    
    def update_command(self, vx: float, vy: float, vz: float):
        with self._cmd_lock:
            self._current_cmd = {"vx": vx, "vy": vy, "vz": vz}
    
    def send_emergency_stop(self) -> Dict[str, Any]:
        """发送紧急停止命令 - 放入队列"""
        import queue
        result_holder = {'response': None}
        event = Event()
        
        try:
            self._cmd_queue.put_nowait(('emergency', result_holder, event))
            # 等待处理完成，最多等1秒
            if event.wait(timeout=1.0):
                return result_holder['response'] or {"success": False, "message": "无响应"}
            else:
                return {"success": False, "message": "处理超时"}
        except queue.Full:
            return {"success": False, "message": "命令队列已满"}
    
    def send_home(self) -> Dict[str, Any]:
        """发送归位命令 - 放入队列，同时触发机械臂归位"""
        import queue
        result_holder = {'response': None}
        event = Event()
        
        # 同时发送机械臂归位命令（不阻塞）
        try:
            print("[Bridge] 发送机械臂归位命令...")
            arm_response = self.arm_client.move_to_home()
            print(f"[Bridge] 机械臂归位响应: {arm_response}")
        except Exception as e:
            print(f"[Bridge] 机械臂归位失败: {e}")
        
        try:
            self._cmd_queue.put_nowait(('home', result_holder, event))
            # 等待处理完成，最多等1秒
            if event.wait(timeout=1.0):
                return result_holder['response'] or {"success": False, "message": "无响应"}
            else:
                return {"success": False, "message": "处理超时"}
        except queue.Full:
            return {"success": False, "message": "命令队列已满"}
    
    def _send_loop(self):
        interval = 0.05  # 20Hz
        retry_interval = 2.0
        
        while self._running:
            try:
                if not self._connected:
                    if self.connect():
                        print("[Bridge] 重新连接成功")
                    else:
                        time.sleep(retry_interval)
                        continue
                
                # 优先处理队列中的特殊命令
                try:
                    import queue
                    cmd_type, result_holder, event = self._cmd_queue.get_nowait()
                    
                    if cmd_type == 'emergency':
                        result_holder['response'] = self.client.send_emergency_stop()
                        self._last_response = result_holder['response']
                        print(f"[Bridge] Emergency stop sent: {result_holder['response']}")
                    elif cmd_type == 'home':
                        result_holder['response'] = self.client.send_home()
                        self._last_response = result_holder['response']
                        print(f"[Bridge] Home command sent: {result_holder['response']}")
                    
                    event.set()
                    continue  # 处理完队列命令后继续
                except queue.Empty:
                    pass  # 没有队列命令，继续发送常规命令
                
                # 发送常规控制命令
                with self._cmd_lock:
                    cmd = self._current_cmd.copy()
                
                response = self.client.send_command(cmd["vx"], cmd["vy"], cmd["vz"])
                self._last_response = response
                
                if not response.get("success"):
                    err_msg = response.get("message", "")
                    if "超时" in err_msg or "socket" in err_msg.lower():
                        print(f"[Bridge] 连接异常: {err_msg}")
                        self._connected = False
                
                time.sleep(interval)
                
            except Exception as e:
                print(f"[Bridge] 异常: {e}")
                self._connected = False
                time.sleep(retry_interval)
    
    def get_status(self) -> Dict[str, Any]:
        # 从上次响应中提取紧急停止状态
        last_resp = self._last_response or {}
        is_locked = (
            last_resp.get('current_owner') == 'emergency' or
            '锁定' in last_resp.get('message', '') or
            '紧急停止' in last_resp.get('message', '')
        )
        return {
            "connected": self._connected,
            "arbiter_addr": self.arbiter_addr,
            "current_cmd": self._current_cmd.copy(),
            "last_response": self._last_response,
            "emergency_locked": is_locked
        }


# 创建Flask应用
app = Flask(
    __name__,
    template_folder=os.path.join(os.path.dirname(__file__), 'templates'),
    static_folder=os.path.join(os.path.dirname(__file__), 'static')
)
app.config['SECRET_KEY'] = 'homebot-secret-key'

socketio = SocketIO(app, cors_allowed_origins="*")
zmq_bridge: Optional[ZMQBridge] = None
video_client: Optional[VideoStreamClient] = None

# 人体跟随进程
human_follow_process: Optional[subprocess.Popen] = None
human_follow_lock = Lock()


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    """视频流路由 - 提供MJPEG格式的摄像头图像"""
    global video_client
    
    print(f"[HTTP] Video feed request from {request.remote_addr}")
    
    # 检查视频客户端是否已连接（不是_running，而是_connected）
    has_video = (video_client is not None and 
                 video_client._connected and 
                 video_client.get_frame() is not None)
    
    if not has_video:
        print(f"[HTTP] Video not available (client={video_client is not None}, "
              f"connected={getattr(video_client, '_connected', False)}, "
              f"has_frame={video_client.get_frame() is not None if video_client else False})")
        
        def empty_stream():
            import cv2
            import numpy as np
            frame_count = 0
            while True:
                # 创建一个黑色占位图像
                img = np.zeros((240, 320, 3), dtype=np.uint8)
                status_text = "Camera Offline - Check Vision Service"
                cv2.putText(img, status_text, (10, 100), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(img, f"Frame: {frame_count}", (10, 140), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
                ret, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n'
                           b'Content-Length: ' + str(len(buf)).encode() + b'\r\n'
                           b'\r\n' + buf.tobytes() + b'\r\n')
                frame_count += 1
                time.sleep(0.5)
        
        return Response(empty_stream(),
                       mimetype='multipart/x-mixed-replace; boundary=frame',
                       headers={
                           'Cache-Control': 'no-cache, no-store, must-revalidate',
                           'Pragma': 'no-cache',
                           'Expires': '0'
                       })
    
    print("[HTTP] Starting MJPEG stream")
    return Response(video_client.generate_mjpeg(),
                   mimetype='multipart/x-mixed-replace; boundary=frame',
                   headers={
                       'Cache-Control': 'no-cache, no-store, must-revalidate',
                       'Pragma': 'no-cache',
                       'Expires': '0'
                   })


@socketio.on('connect')
def handle_connect():
    print(f"[Socket] 客户端已连接")
    status = zmq_bridge.get_status() if zmq_bridge else {}
    emit('server_response', {
        'status': 'connected',
        'message': '已连接到机器人控制服务器',
        'arbiter_connected': status.get('connected', False)
    })


@socketio.on('disconnect')
def handle_disconnect():
    print("[Socket] 客户端已断开")
    if zmq_bridge:
        zmq_bridge.update_command(0.0, 0.0, 0.0)


@socketio.on('joystick_data')
def handle_joystick(data):
    """处理摇杆数据"""
    try:
        left = data.get('left', {})
        left_x = float(left.get('x', 0.0))
        left_y = float(left.get('y', 0.0))
        
        max_linear = 0.5
        max_angular = 1.0
        
        vx = -left_y * max_linear
        vz = left_x * max_angular
        
        if zmq_bridge:
            zmq_bridge.update_command(vx, 0.0, vz)
        
        emit('command_ack', {'success': True, 'chassis': {'vx': vx, 'vz': vz}})
        
    except Exception as e:
        print(f"[Socket] 处理摇杆数据失败: {e}")
        emit('command_ack', {'success': False, 'error': str(e)})


@socketio.on('emergency_stop')
def handle_emergency_stop():
    print("[Socket] 紧急停止！")
    if zmq_bridge:
        response = zmq_bridge.send_emergency_stop()
        emit('server_response', {
            'status': 'emergency_stop',
            'locked': response.get('success', False),
            'message': response.get('message', '紧急停止已触发')
        })
    else:
        emit('server_response', {'status': 'emergency_stop', 'error': '未连接'})


@socketio.on('home')
def handle_home():
    print("[Socket] 归位命令（底盘+机械臂）")
    if zmq_bridge:
        response = zmq_bridge.send_home()
        emit('server_response', {
            'status': 'home',
            'success': response.get('success', False),
            'message': response.get('message', '归位完成')
        })
    else:
        emit('server_response', {'status': 'home', 'error': '未连接'})


@socketio.on('get_status')
def handle_get_status():
    if zmq_bridge:
        status = zmq_bridge.get_status()
        # 添加人体跟随状态
        with human_follow_lock:
            status['human_follow_active'] = human_follow_process is not None and human_follow_process.poll() is None
        emit('server_status', status)


@socketio.on('toggle_human_follow')
def handle_toggle_human_follow(data):
    """处理人体跟随开关请求"""
    global human_follow_process
    
    requested_state = data.get('active', False)
    print(f"[Socket] 人体跟随请求: {'启动' if requested_state else '停止'}")
    
    with human_follow_lock:
        is_running = (human_follow_process is not None and 
                      human_follow_process.poll() is None)
        
        if requested_state and not is_running:
            # 启动人体跟随
            try:
                # 检查是否在紧急停止状态
                if zmq_bridge:
                    arbiter_status = zmq_bridge.get_status()
                    if arbiter_status.get('emergency_locked', False):
                        emit('server_response', {
                            'status': 'human_follow',
                            'success': False,
                            'active': False,
                            'message': '底盘已锁定，请先归位解锁'
                        })
                        return
                
                # 启动人体跟随进程
                src_dir = os.path.join(os.path.dirname(__file__), '..', '..')
                cmd = [sys.executable, '-m', 'applications.human_follow']
                
                # 设置环境变量
                env = os.environ.copy()
                env['PYTHONPATH'] = src_dir + os.pathsep + env.get('PYTHONPATH', '')
                
                print(f"[HumanFollow] 启动进程: {' '.join(cmd)}")
                
                # 使用subprocess.Popen启动
                human_follow_process = subprocess.Popen(
                    cmd,
                    cwd=src_dir,
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == 'win32' else 0
                )
                
                # 给进程一点时间启动
                import time
                time.sleep(1.0)
                
                # 检查进程是否成功启动
                if human_follow_process.poll() is None:
                    print(f"[HumanFollow] 进程已启动 (PID: {human_follow_process.pid})")
                    emit('server_response', {
                        'status': 'human_follow',
                        'success': True,
                        'active': True,
                        'message': '人体跟随已启动'
                    })
                    # 广播状态给所有客户端
                    socketio.emit('follow_status', {'active': True})
                else:
                    returncode = human_follow_process.returncode
                    stdout, stderr = human_follow_process.communicate(timeout=1)
                    error_msg = stderr.decode('utf-8', errors='ignore') if stderr else f'返回码: {returncode}'
                    print(f"[HumanFollow] 进程启动失败: {error_msg}")
                    human_follow_process = None
                    emit('server_response', {
                        'status': 'human_follow',
                        'success': False,
                        'active': False,
                        'message': f'启动失败: {error_msg}'
                    })
                    
            except Exception as e:
                print(f"[HumanFollow] 启动异常: {e}")
                import traceback
                traceback.print_exc()
                human_follow_process = None
                emit('server_response', {
                    'status': 'human_follow',
                    'success': False,
                    'active': False,
                    'message': f'启动异常: {str(e)}'
                })
                
        elif not requested_state and is_running:
            # 停止人体跟随
            try:
                print(f"[HumanFollow] 停止进程 (PID: {human_follow_process.pid})")
                
                # 发送终止信号
                if sys.platform == 'win32':
                    human_follow_process.send_signal(signal.CTRL_BREAK_EVENT)
                else:
                    human_follow_process.send_signal(signal.SIGTERM)
                
                # 等待进程终止
                try:
                    human_follow_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    print("[HumanFollow] 进程未及时终止，强制杀死")
                    human_follow_process.kill()
                    human_follow_process.wait()
                
                print("[HumanFollow] 进程已停止")
                human_follow_process = None
                
                emit('server_response', {
                    'status': 'human_follow',
                    'success': True,
                    'active': False,
                    'message': '人体跟随已停止'
                })
                # 广播状态给所有客户端
                socketio.emit('follow_status', {'active': False})
                
            except Exception as e:
                print(f"[HumanFollow] 停止异常: {e}")
                emit('server_response', {
                    'status': 'human_follow',
                    'success': False,
                    'active': True,
                    'message': f'停止异常: {str(e)}'
                })
        else:
            # 状态没有变化
            emit('server_response', {
                'status': 'human_follow',
                'success': True,
                'active': is_running,
                'message': '状态未变化'
            })


def run_server(host: str = '0.0.0.0', port: int = 5000,
               arbiter_addr: Optional[str] = None,
               vision_addr: Optional[str] = None,
               arm_addr: Optional[str] = None, debug: bool = False):
    global zmq_bridge, video_client
    
    print("=" * 60)
    print("HomeBot 网页控制端")
    print("=" * 60)
    
    # 初始化底盘控制桥接（同时连接机械臂服务）
    zmq_bridge = ZMQBridge(arbiter_addr=arbiter_addr, arm_addr=arm_addr)
    if not zmq_bridge.start():
        print("[警告] 无法连接到底盘服务")
    
    # 初始化视频流客户端
    video_client = VideoStreamClient(vision_addr=vision_addr or DEFAULT_VISION_ADDR)
    video_ok = video_client.start()
    if not video_ok:
        print("[警告] 无法连接到视觉服务，视频流不可用")
        print("[提示] 请确保 VisionService 已启动:")
        print(f"       python -m services.vision_service")
        print(f"       或检查地址是否正确: {video_client.vision_addr}")
    else:
        print("[OK] 视觉服务连接成功")
    
    print(f"[配置] 底盘服务: {zmq_bridge.arbiter_addr}")
    print(f"[配置] 机械臂服务: {zmq_bridge.arm_addr}")
    print(f"[配置] 视觉服务: {video_client.vision_addr}")
    print(f"[网络] Web服务器: http://{host}:{port}")
    print(f"[视频] 视频流地址: http://{host}:{port}/video_feed")
    
    # 获取本机IP
    try:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        print(f"[访问] 手机访问: http://{local_ip}:{port}")
    except:
        pass
    
    print("=" * 60)
    
    try:
        socketio.run(app, host=host, port=port, debug=debug, use_reloader=False)
    except KeyboardInterrupt:
        print("\n[关闭] 正在关闭...")
    finally:
        # 停止人体跟随进程
        global human_follow_process
        with human_follow_lock:
            if human_follow_process is not None:
                try:
                    if human_follow_process.poll() is None:
                        print("[HumanFollow] 关闭服务器时停止跟随进程")
                        if sys.platform == 'win32':
                            human_follow_process.send_signal(signal.CTRL_BREAK_EVENT)
                        else:
                            human_follow_process.send_signal(signal.SIGTERM)
                        human_follow_process.wait(timeout=2)
                except:
                    try:
                        human_follow_process.kill()
                    except:
                        pass
                human_follow_process = None
        
        zmq_bridge.stop()
        video_client.stop()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='HomeBot 网页控制端')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=5000)
    parser.add_argument('--arbiter', dest='arbiter_addr', default=None,
                       help='底盘服务地址 (默认: tcp://127.0.0.1:5556)')
    parser.add_argument('--vision', dest='vision_addr', default=None,
                       help='视觉服务地址 (默认: tcp://127.0.0.1:5560)')
    parser.add_argument('--arm', dest='arm_addr', default=None,
                       help='机械臂服务地址 (默认: tcp://127.0.0.1:5557)')
    parser.add_argument('--debug', action='store_true')
    
    args = parser.parse_args()
    run_server(**vars(args))


if __name__ == '__main__':
    main()
