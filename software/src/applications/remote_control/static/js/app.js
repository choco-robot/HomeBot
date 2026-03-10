/**
 * HomeBot 网页控制端 - 前端应用
 * 双虚拟摇杆控制 + WebSocket通信
 */

class RobotController {
    constructor() {
        // 摇杆状态
        this.leftJoystick = null;
        this.rightJoystick = null;
        this.joystickData = {
            left: { x: 0, y: 0 },
            right: { x: 0, y: 0 }
        };
        
        // Socket连接
        this.socket = null;
        this.isConnected = false;
        this.isArbiterConnected = false;
        this.emergencyLocked = false;
        
        // 人体跟随状态
        this.isHumanFollowActive = false;
        
        // 控制使能标志
        this.isControlEnabled = true;
        
        // 数据发送控制
        this.sendInterval = null;
        this.lastSentData = null;
        
        // FPS计算
        this.frameCount = 0;
        this.lastFpsTime = Date.now();
        
        // 视频流
        this.videoElement = null;
        this.videoCheckInterval = null;
        this.lastVideoLoadTime = 0;  // 初始化为0，表示尚未收到数据
        this.isVideoActive = false;
        
        // 初始化
        this.init();
    }
    
    init() {
        this.initSocket();
        this.initJoysticks();
        this.initButtons();
        this.initVideo();
        this.startSendLoop();
        this.startStatusPolling();
        this.updateFps();
        this.updateFollowStatus(false);
    }
    
    // ========== 视频流初始化 ==========
    initVideo() {
        this.videoElement = document.getElementById('videoFeed');
        if (!this.videoElement) {
            console.error('[Video] videoFeed element not found');
            return;
        }
        
        console.log('[Video] Initializing video stream...');
        this.lastVideoLoadTime = 0;
        this.isVideoActive = false;
        
        // 记录上次检查时的图像尺寸
        this._lastWidth = 0;
        this._lastHeight = 0;
        
        // 监听首次加载
        this.videoElement.onload = () => {
            console.log('[Video] Image loaded');
            this.lastVideoLoadTime = Date.now();
            if (!this.isVideoActive) {
                this.isVideoActive = true;
                this.updateVideoStatus(true);
            }
        };
        
        this.videoElement.onerror = (e) => {
            console.error('[Video] Stream error:', e);
            this.updateVideoStatus(false);
        };
        
        // 定期检查视频流是否活跃（通过检测图像尺寸变化）
        this.videoCheckInterval = setInterval(() => {
            this.checkVideoStatus();
        }, 1000);
        
        // 设置初始状态为连接中
        this.updateVideoStatus(false);
        
        // 3秒后检查连接状态
        setTimeout(() => {
            if (!this.isVideoActive) {
                console.warn('[Video] Stream not connected after 3s');
                this.updateVideoStatus(false);
            }
        }, 3000);
    }
    
    checkVideoStatus() {
        if (!this.videoElement) return;
        
        const img = this.videoElement;
        const currentTime = Date.now();
        
        // 检查图像是否已加载且有尺寸
        if (img.complete && img.naturalWidth > 0) {
            // 图像已加载
            if (!this.isVideoActive) {
                this.isVideoActive = true;
                this.updateVideoStatus(true);
                console.log('[Video] Stream active, size:', img.naturalWidth + 'x' + img.naturalHeight);
            }
            this.lastVideoLoadTime = currentTime;
            this._lastWidth = img.naturalWidth;
            this._lastHeight = img.naturalHeight;
        } else {
            // 检查是否超时（5秒无数据视为断开）
            const timeSinceLastLoad = currentTime - this.lastVideoLoadTime;
            if (timeSinceLastLoad > 5000 && this.isVideoActive) {
                this.isVideoActive = false;
                this.updateVideoStatus(false);
                console.warn('[Video] Stream timeout');
            }
        }
    }
    
    updateVideoStatus(active) {
        const statusEl = document.getElementById('videoStatus');
        if (!statusEl) return;
        
        if (active) {
            statusEl.textContent = '● LIVE';
            statusEl.classList.remove('offline');
        } else {
            statusEl.textContent = '● OFFLINE';
            statusEl.classList.add('offline');
        }
    }
    
    updateEmergencyLock(locked) {
        this.emergencyLocked = locked;
        this.isControlEnabled = !locked;
        
        // 更新UI显示
        const emergencyBtn = document.getElementById('btnEmergency');
        const homeBtn = document.getElementById('btnHome');
        
        if (locked) {
            // 紧急停止锁定状态
            if (emergencyBtn) {
                emergencyBtn.textContent = '已锁定';
                emergencyBtn.style.background = '#666';
            }
            if (homeBtn) {
                homeBtn.textContent = '点击解锁';
                homeBtn.style.background = 'linear-gradient(135deg, #00ff88, #00cc66)';
                homeBtn.style.animation = 'pulse 1s infinite';
            }
            
            // 重置摇杆数据并发送停止命令
            this.joystickData = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 } };
            if (this.isConnected) {
                this.socket.emit('joystick_data', this.joystickData);
            }
        } else {
            // 正常状态
            if (emergencyBtn) {
                emergencyBtn.textContent = '紧急停止';
                emergencyBtn.style.background = '';
            }
            if (homeBtn) {
                homeBtn.textContent = '归位';
                homeBtn.style.background = '';
                homeBtn.style.animation = '';
            }
        }
    }
    
    // ========== Socket.IO 连接 ==========
    initSocket() {
        // 连接WebSocket服务器
        this.socket = io();
        
        this.socket.on('connect', () => {
            console.log('[Socket] 已连接到服务器');
            this.isConnected = true;
            this.updateConnectionStatus('ws', true);
            this.showToast('已连接到控制服务器', 'success');
        });
        
        this.socket.on('disconnect', () => {
            console.log('[Socket] 与服务器断开连接');
            this.isConnected = false;
            this.isArbiterConnected = false;
            this.updateConnectionStatus('ws', false);
            this.updateConnectionStatus('arbiter', false);
            this.showToast('连接已断开', 'error');
        });
        
        this.socket.on('server_response', (data) => {
            console.log('[Socket] 服务器响应:', data);
            
            // 更新仲裁器连接状态
            if (data.arbiter_connected !== undefined) {
                this.isArbiterConnected = data.arbiter_connected;
                this.updateConnectionStatus('arbiter', this.isArbiterConnected);
                
                if (!this.isArbiterConnected) {
                    this.showToast('仲裁器未连接，无法控制底盘', 'warning');
                }
            }
        });
        
        this.socket.on('command_ack', (data) => {
            if (data.success) {
                this.updateDataDisplay(data.chassis);
            }
        });
        
        this.socket.on('server_status', (status) => {
            console.log('[Socket] 服务器状态:', status);
            
            // 更新仲裁器状态
            if (status.connected !== undefined) {
                this.isArbiterConnected = status.connected;
                this.updateConnectionStatus('arbiter', this.isArbiterConnected);
            }
            
            // 更新紧急停止锁定状态
            if (status.emergency_locked !== undefined) {
                this.updateEmergencyLock(status.emergency_locked);
            }
        });
        
        // 处理服务器响应（紧急停止/归位结果）
        this.socket.on('server_response', (data) => {
            console.log('[Socket] 服务器响应:', data);
            
            if (data.status === 'emergency_stop') {
                if (data.locked) {
                    this.updateEmergencyLock(true);
                    this.showToast('紧急停止！底盘已锁定，点击归位解除', 'error');
                } else {
                    this.showToast('紧急停止失败: ' + (data.message || ''), 'error');
                }
            } else if (data.status === 'home') {
                if (data.success) {
                    this.updateEmergencyLock(false);
                    this.showToast(data.message || '归位完成，底盘已解锁', 'success');
                } else {
                    this.showToast('归位失败: ' + (data.message || ''), 'error');
                }
            } else if (data.status === 'human_follow') {
                // 人体跟随状态变化
                if (data.active !== undefined) {
                    this.isHumanFollowActive = data.active;
                    this.updateFollowStatus(data.active);
                    if (data.success) {
                        this.showToast(data.active ? '人体跟随已启动' : '人体跟随已停止', 'success');
                    } else {
                        this.showToast('人体跟随操作失败: ' + (data.message || ''), 'error');
                    }
                }
            }
        });
        
        // 人体跟随状态更新
        this.socket.on('follow_status', (data) => {
            if (data.active !== undefined) {
                this.isHumanFollowActive = data.active;
                this.updateFollowStatus(data.active);
            }
        });
        
        this.socket.on('connect_error', (error) => {
            console.error('[Socket] 连接错误:', error);
            this.showToast('连接失败，请刷新重试', 'error');
        });
    }
    
    // ========== 状态轮询 ==========
    startStatusPolling() {
        // 每2秒获取一次状态
        setInterval(() => {
            if (this.isConnected) {
                this.socket.emit('get_status');
            }
        }, 2000);
    }
    
    // ========== 虚拟摇杆初始化 ==========
    initJoysticks() {
        const joystickOptions = {
            mode: 'static',
            position: { left: '50%', top: '50%' },
            size: 100,
            threshold: 0.1,  // 死区阈值
            fadeTime: 100,
            multitouch: true
        };
        
        // 左手摇杆 - 底盘控制
        this.leftJoystick = nipplejs.create({
            ...joystickOptions,
            zone: document.getElementById('leftJoystick'),
            color: 'rgba(0, 255, 136, 0.5)'
        });
        
        // 右手摇杆 - 机械臂控制（预留）
        this.rightJoystick = nipplejs.create({
            ...joystickOptions,
            zone: document.getElementById('rightJoystick'),
            color: 'rgba(68, 68, 255, 0.5)'
        });
        
        // 绑定左手摇杆事件
        this.leftJoystick.on('move', (evt, data) => {
            const angle = data.angle.radian;
            const distance = Math.min(data.distance / 50, 1.0); // 归一化到0-1
            
            // 计算X/Y分量（Y轴向上为负）
            const x = Math.cos(angle) * distance;
            const y = -Math.sin(angle) * distance;
            
            this.joystickData.left = { x, y };
            this.frameCount++;
        });
        
        this.leftJoystick.on('end', () => {
            this.joystickData.left = { x: 0, y: 0 };
        });
        
        // 绑定右手摇杆事件（机械臂控制，预留）
        this.rightJoystick.on('move', (evt, data) => {
            const angle = data.angle.radian;
            const distance = Math.min(data.distance / 50, 1.0);
            
            const x = Math.cos(angle) * distance;
            const y = -Math.sin(angle) * distance;
            
            this.joystickData.right = { x, y };
            this.frameCount++;
        });
        
        this.rightJoystick.on('end', () => {
            this.joystickData.right = { x: 0, y: 0 };
        });
    }
    
    // ========== 按钮事件 ==========
    initButtons() {
        // 紧急停止按钮
        document.getElementById('btnEmergency').addEventListener('click', () => {
            this.emergencyStop();
        });
        
        // 归位按钮
        document.getElementById('btnHome').addEventListener('click', () => {
            this.goHome();
        });
        
        // 人体跟随按钮
        document.getElementById('btnFollow').addEventListener('click', () => {
            this.toggleHumanFollow();
        });
    }
    
    // ========== 人体跟随控制 ==========
    toggleHumanFollow() {
        console.log('[Control] 切换人体跟随状态');
        
        if (this.isConnected) {
            const newState = !this.isHumanFollowActive;
            this.socket.emit('toggle_human_follow', { active: newState });
            this.showToast(newState ? '正在启动人体跟随...' : '正在停止人体跟随...', 'info');
        } else {
            this.showToast('未连接，无法控制人体跟随', 'error');
        }
    }
    
    updateFollowStatus(active) {
        const followBtn = document.getElementById('btnFollow');
        const statusDot = document.getElementById('followStatusDot');
        
        if (followBtn) {
            if (active) {
                followBtn.textContent = '关闭跟随';
                followBtn.classList.add('active');
            } else {
                followBtn.textContent = '打开跟随';
                followBtn.classList.remove('active');
            }
        }
        
        if (statusDot) {
            if (active) {
                statusDot.classList.add('connected');
            } else {
                statusDot.classList.remove('connected');
            }
        }
    }
    
    // ========== 数据发送循环 ==========
    startSendLoop() {
        // 以20Hz频率发送数据
        this.sendInterval = setInterval(() => {
            this.sendJoystickData();
        }, 50); // 50ms = 20Hz
    }
    
    sendJoystickData() {
        if (!this.isConnected) return;
        
        // 如果处于紧急停止锁定状态，不发送控制命令
        if (!this.isControlEnabled) {
            // 确保发送一次停止命令
            if (this.lastSentData !== JSON.stringify({left: {x: 0, y: 0}, right: {x: 0, y: 0}})) {
                this.socket.emit('joystick_data', {left: {x: 0, y: 0}, right: {x: 0, y: 0}});
                this.lastSentData = JSON.stringify({left: {x: 0, y: 0}, right: {x: 0, y: 0}});
            }
            return;
        }
        
        // 检查数据是否有变化（减少不必要的传输）
        const currentData = JSON.stringify(this.joystickData);
        if (currentData === this.lastSentData) {
            // 数据未变化，但超过100ms也要发送一次（保活）
            if (!this.lastSendTime || Date.now() - this.lastSendTime > 100) {
                // 继续发送
            } else {
                return;
            }
        }
        
        this.socket.emit('joystick_data', this.joystickData);
        this.lastSentData = currentData;
        this.lastSendTime = Date.now();
    }
    
    // ========== 控制命令 ==========
    emergencyStop() {
        console.log('[Control] 紧急停止！');
        
        // 重置摇杆数据
        this.joystickData = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 } };
        
        if (this.isConnected) {
            this.socket.emit('emergency_stop');
        } else {
            // 离线模式下也显示锁定状态
            this.updateEmergencyLock(true);
            this.showToast('紧急停止！底盘已锁定（离线模式）', 'error');
        }
        
        // 震动反馈（如果支持）
        if (navigator.vibrate) {
            navigator.vibrate([100, 50, 100, 50, 200]);
        }
    }
    
    goHome() {
        console.log('[Control] 归位命令');
        if (this.isConnected) {
            this.socket.emit('home');
            this.showToast('发送归位命令...', 'success');
        } else {
            this.showToast('未连接，无法归位', 'error');
        }
    }
    
    // ========== UI更新 ==========
    updateConnectionStatus(type, connected) {
        if (type === 'ws') {
            const dot = document.getElementById('wsStatusDot');
            if (connected) {
                dot.classList.add('connected');
                dot.classList.remove('warning');
            } else {
                dot.classList.remove('connected', 'warning');
            }
        } else if (type === 'arbiter') {
            const dot = document.getElementById('arbiterStatusDot');
            if (connected) {
                dot.classList.add('connected');
                dot.classList.remove('warning');
            } else {
                dot.classList.remove('connected');
                dot.classList.add('warning');
            }
        }
    }
    
    updateDataDisplay(chassis) {
        if (chassis) {
            document.getElementById('vx').textContent = chassis.vx.toFixed(2);
            document.getElementById('vz').textContent = chassis.vz.toFixed(2);
        }
    }
    
    updateFps() {
        setInterval(() => {
            const now = Date.now();
            const elapsed = (now - this.lastFpsTime) / 1000;
            const fps = Math.round(this.frameCount / elapsed);
            
            document.getElementById('fps').textContent = `${fps} FPS`;
            
            this.frameCount = 0;
            this.lastFpsTime = now;
        }, 1000);
    }
    
    showToast(message, type = 'info') {
        const toast = document.getElementById('toast');
        toast.textContent = message;
        toast.className = 'toast show';
        
        if (type === 'error') {
            toast.classList.add('error');
        } else if (type === 'success') {
            toast.classList.add('success');
        } else if (type === 'warning') {
            toast.classList.add('warning');
        }
        
        setTimeout(() => {
            toast.classList.remove('show');
        }, 3000);
    }
    
    // ========== 清理 ==========
    destroy() {
        if (this.sendInterval) {
            clearInterval(this.sendInterval);
        }
        if (this.videoCheckInterval) {
            clearInterval(this.videoCheckInterval);
        }
        if (this.socket) {
            this.socket.disconnect();
        }
        if (this.leftJoystick) {
            this.leftJoystick.destroy();
        }
        if (this.rightJoystick) {
            this.rightJoystick.destroy();
        }
    }
}

// 页面加载完成后初始化
let controller = null;

document.addEventListener('DOMContentLoaded', () => {
    controller = new RobotController();
});

// 页面关闭时清理
window.addEventListener('beforeunload', () => {
    if (controller) {
        controller.destroy();
    }
});

// 防止移动端页面滚动和缩放
document.addEventListener('touchmove', (e) => {
    if (e.target.closest('.joystick-zone')) {
        e.preventDefault();
    }
}, { passive: false });

// 防止双击缩放
document.addEventListener('dblclick', (e) => {
    e.preventDefault();
});
