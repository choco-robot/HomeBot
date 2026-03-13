# HomeBot 新应用开发指南

本文档指导开发者如何基于 HomeBot 框架开发新的应用模块。

## 目录

1. [快速开始](#快速开始)
2. [应用结构](#应用结构)
3. [核心组件](#核心组件)
4. [开发步骤](#开发步骤)
5. [完整示例](#完整示例)
6. [调试与测试](#调试与测试)

---

## 快速开始

### 前置条件

```bash
# 1. 确保已安装依赖
cd homebot
pip install -e .

# 2. 启动基础服务
cd software/src

# 终端1: 启动底盘服务
python -m services.motion_service.chassis_service

# 终端2: 启动视觉服务（如需要）
python -m services.vision_service
```

### 创建最小应用（3分钟上手）

```bash
cd software/src/applications

# 创建新应用目录
mkdir my_app
cd my_app

# 创建必要文件
touch __init__.py __main__.py app.py
```

**`app.py`** - 应用核心逻辑:

```python
"""我的第一个 HomeBot 应用"""
import time
from common.logging import get_logger
from services.motion_service.chassis_arbiter import ChassisArbiterClient

logger = get_logger(__name__)


class MyFirstApp:
    """示例应用：让机器人画正方形
    
    注意：底盘控制器有1000ms超时机制，需要持续发送指令维持运动
    """
    
    def __init__(self):
        # 初始化底盘客户端
        self.chassis = ChassisArbiterClient("tcp://localhost:5556")
        self.running = False
        # 控制频率（Hz），必须 > 1Hz 以避免超时
        self.control_rate = 20  # 20Hz = 每50ms发送一次
        
    def move_forward(self, duration=2.0, speed=0.3):
        """向前移动指定时间
        
        Args:
            duration: 移动时间（秒）
            speed: 前进速度（m/s）
        """
        logger.info(f"向前移动 {duration} 秒")
        start_time = time.time()
        
        # 持续发送指令以维持控制权（心跳模式）
        while time.time() - start_time < duration:
            self.chassis.send_command(
                vx=speed, vy=0.0, vz=0.0,
                source="auto", priority=3
            )
            time.sleep(1.0 / self.control_rate)
        
        self.stop()
        
    def turn_right(self, duration=1.0, speed=1.57):
        """右转指定时间
        
        Args:
            duration: 转动时间（秒）
            speed: 角速度（rad/s），1.57约90度/秒
        """
        logger.info(f"右转 {duration} 秒")
        start_time = time.time()
        
        # 持续发送指令以维持控制权
        while time.time() - start_time < duration:
            self.chassis.send_command(
                vx=0.0, vy=0.0, vz=-speed,  # 负值为右转
                source="auto", priority=3
            )
            time.sleep(1.0 / self.control_rate)
        
        self.stop()
        
    def stop(self):
        """停止"""
        self.chassis.send_command(
            vx=0.0, vy=0.0, vz=0.0,
            source="auto", priority=3
        )
        
    def run(self):
        """主循环：画正方形"""
        logger.info("开始执行正方形路径")
        self.running = True
        
        try:
            for i in range(4):
                if not self.running:
                    break
                logger.info(f"第 {i+1} 边")
                self.move_forward(2.0)
                time.sleep(0.5)
                self.turn_right(1.0)
                time.sleep(0.5)
                
            logger.info("任务完成")
            
        except KeyboardInterrupt:
            logger.info("用户中断")
        finally:
            self.stop()
            self.running = False
```

> **⚠️ 重要提示**：底盘服务有 **1000ms（1秒）超时机制**。最后一次发送指令后，如果 1 秒内没有新指令，控制权会自动释放，底盘会自动停止。因此移动操作必须使用**持续发送（心跳）模式**，而不是简单的"发送→延时→停止"模式。

**`__main__.py`** - 应用入口:

```python
#!/usr/bin/env python3
"""应用启动入口"""
from .app import MyFirstApp

if __name__ == "__main__":
    app = MyFirstApp()
    app.run()
```

**`__init__.py`** - 包标记（可为空）:

```python
"""My First App 包"""
```

### 运行应用

```bash
cd software/src
python -m applications.my_app
```

---

## 应用结构

### 标准目录结构

```
applications/my_app/
├── __init__.py          # 包初始化（可选导出）
├── __main__.py          # 应用入口点
├── app.py               # 主应用逻辑
├── detector.py          # 检测器（如需要）
├── controller.py        # 控制器（如需要）
├── utils.py             # 工具函数
└── README.md            # 应用说明
```

### 命名规范

| 类型 | 命名风格 | 示例 |
|------|---------|------|
| 目录 | snake_case | `my_application/` |
| 类名 | PascalCase | `MyApplication` |
| 函数/变量 | snake_case | `send_command()` |
| 常量 | UPPER_CASE | `MAX_SPEED = 1.0` |
| 控制源 | 小写字符串 | `"auto"`, `"voice"`, `"web"` |

---

## 核心组件

### 1. 底盘控制（必需）

```python
from services.motion_service.chassis_arbiter import ChassisArbiterClient
import time

# 初始化客户端
chassis = ChassisArbiterClient("tcp://localhost:5556")

# ⚠️ 重要：底盘有1000ms超时机制，需要持续发送指令（心跳模式）
# 错误方式（会超时停止）：
# chassis.send_command(vx=0.5, vy=0, vz=0, source="auto", priority=3)
# time.sleep(2)  # 超过1秒，底盘已自动停止！

# 正确方式（持续发送）：
start_time = time.time()
while time.time() - start_time < 2.0:  # 持续2秒
    chassis.send_command(
        vx=0.5,      # 线速度 X (m/s)，向前为正
        vy=0.0,      # 线速度 Y (m/s)，向左为正
        vz=0.3,      # 角速度 Z (rad/s)，逆时针为正
        source="auto",   # 控制源: web/voice/auto/emergency
        priority=3       # 优先级: 1=web, 2=voice, 3=auto, 4=emergency
    )
    time.sleep(0.05)  # 20Hz，确保不超过1秒超时

# 发送停止指令
chassis.send_command(vx=0, vy=0, vz=0, source="auto", priority=3)
```

**底盘超时机制说明：**
- 超时时间：**1000ms（1秒）**
- 机制：最后一次指令后 1 秒内无新指令 → 自动释放控制权 → 底盘停止
- 解决：**必须以 >1Hz 的频率持续发送指令**（推荐 20-50Hz）

**控制源优先级：**

```
emergency(4) > auto(3) > voice(2) > web(1)
```

### 2. 视觉订阅（可选）

```python
from services.vision_service import VisionSubscriber
import cv2

# 初始化订阅
vision = VisionSubscriber("tcp://localhost:5560")
vision.start()

# 读取图像帧
frame_id, frame = vision.read_frame()
if frame is not None:
    # frame 是 numpy.ndarray (BGR格式)
    cv2.imshow("Camera", frame)
    
# 停止订阅
vision.stop()
```

### 3. 配置管理

```python
from configs import get_config

# 获取全局配置
config = get_config()

# 访问各模块配置
print(config.chassis.serial_port)      # 底盘串口
print(config.camera.width)             # 相机宽度
print(config.human_follow.model_path)  # 模型路径

# 自定义应用配置（推荐）
from dataclasses import dataclass

@dataclass
class MyAppConfig:
    loop_count: int = 5
    speed: float = 0.3
    enable_debug: bool = False
```

### 4. 日志记录

```python
from common.logging import get_logger

# 创建日志器
logger = get_logger(__name__)

# 各级别日志
logger.debug("调试信息")
logger.info("普通信息")
logger.warning("警告信息")
logger.error("错误信息")
logger.critical("严重错误")
```

---

## 开发步骤

### Step 1: 规划应用架构

确定应用需要的组件：

- [ ] 是否需要视觉输入？
- [ ] 是否需要底盘控制？
- [ ] 是否需要语音交互？
- [ ] 是否需要机械臂控制？
- [ ] 状态机设计

### Step 2: 创建应用框架

```bash
mkdir -p software/src/applications/my_app
touch software/src/applications/my_app/__init__.py
touch software/src/applications/my_app/__main__.py
```

### Step 3: 实现主类

参考模板：

```python
"""应用模板"""
import threading
import time
from typing import Optional
from enum import Enum

from common.logging import get_logger
from configs import get_config
from services.motion_service.chassis_arbiter import ChassisArbiterClient
from services.vision_service import VisionSubscriber

logger = get_logger(__name__)


class AppState(Enum):
    """应用状态"""
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    ERROR = "error"


class MyApplication:
    """
    新应用模板
    
    功能描述：
    1. xxx
    2. xxx
    
    使用方式：
        cd software/src
        python -m applications.my_app
    """
    
    def __init__(self, config=None):
        """初始化应用"""
        self.config = config or get_config()
        self.state = AppState.IDLE
        self.running = False
        
        # 组件（延迟初始化）
        self.chassis: Optional[ChassisArbiterClient] = None
        self.vision: Optional[VisionSubscriber] = None
        
        # 线程控制
        self._stop_event = threading.Event()
        
        logger.info("应用初始化完成")
    
    def initialize(self) -> bool:
        """
        初始化所有组件
        
        Returns:
            bool: 初始化是否成功
        """
        try:
            logger.info("正在初始化...")
            
            # 初始化底盘客户端
            self.chassis = ChassisArbiterClient("tcp://localhost:5556")
            logger.info("✓ 底盘客户端已连接")
            
            # 如需视觉，取消注释
            # self.vision = VisionSubscriber("tcp://localhost:5560")
            # self.vision.start()
            # logger.info("✓ 视觉订阅已启动")
            
            logger.info("初始化完成")
            return True
            
        except Exception as e:
            logger.error(f"初始化失败: {e}")
            self.state = AppState.ERROR
            return False
    
    def process(self):
        """
        主处理逻辑（每帧调用）
        子类应重写此方法
        
        注意：如果涉及底盘控制，需要在此方法中持续发送指令
        以避免1000ms超时机制导致自动停止
        """
        pass
    
    def run(self):
        """主循环"""
        if not self.initialize():
            return
        
        self.running = True
        self.state = AppState.RUNNING
        self._stop_event.clear()
        
        logger.info("=" * 50)
        logger.info("应用已启动，按 Ctrl+C 停止")
        logger.info("=" * 50)
        
        # 控制频率（Hz），必须 > 1Hz 以避免底盘超时
        control_rate = 20
        dt = 1.0 / control_rate
        
        try:
            while self.running and not self._stop_event.is_set():
                loop_start = time.time()
                
                self.process()
                
                # 精确控制循环频率
                elapsed = time.time() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            logger.info("用户中断")
        except Exception as e:
            logger.error(f"运行异常: {e}")
            self.state = AppState.ERROR
        finally:
            self.stop()
    
    def pause(self):
        """暂停应用"""
        if self.state == AppState.RUNNING:
            self.state = AppState.PAUSED
            self.stop_chassis()
            logger.info("应用已暂停")
    
    def resume(self):
        """恢复应用"""
        if self.state == AppState.PAUSED:
            self.state = AppState.RUNNING
            logger.info("应用已恢复")
    
    def stop(self):
        """停止应用"""
        logger.info("正在停止应用...")
        self.running = False
        self._stop_event.set()
        self.state = AppState.IDLE
        
        # 停止底盘
        self.stop_chassis()
        
        # 释放资源
        if self.vision:
            self.vision.stop()
        
        logger.info("应用已停止")
    
    def stop_chassis(self):
        """停止底盘运动"""
        if self.chassis:
            try:
                self.chassis.send_command(
                    vx=0, vy=0, vz=0,
                    source="auto", priority=3
                )
            except Exception as e:
                logger.warning(f"停止底盘失败: {e}")


def main():
    """入口函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='我的应用')
    parser.add_argument('--debug', '-d', action='store_true',
                       help='启用调试模式')
    
    args = parser.parse_args()
    
    app = MyApplication()
    app.run()


if __name__ == "__main__":
    main()
```

### Step 4: 添加配置（可选）

编辑 `software/src/configs/config.py`：

```python
@dataclass
class MyAppConfig:
    """我的应用配置"""
    param1: float = 0.5
    param2: int = 100
    enabled: bool = True

@dataclass
class Config:
    """全局配置"""
    # ... 现有配置 ...
    my_app: MyAppConfig = field(default_factory=MyAppConfig)
```

### Step 5: 测试与调试

```bash
# 运行应用
cd software/src
python -m applications.my_app

# 带调试输出
python -m applications.my_app --debug
```

---

## 完整示例

### 示例：自动避障应用

```python
"""自动避障应用 - 使用距离估计"""
import time
import numpy as np
from typing import Optional, List
from dataclasses import dataclass

from common.logging import get_logger
from services.motion_service.chassis_arbiter import ChassisArbiterClient
from services.vision_service import VisionSubscriber

logger = get_logger(__name__)


@dataclass
class Obstacle:
    """障碍物信息"""
    x: int      # 图像坐标x
    y: int      # 图像坐标y
    width: int  # 宽度
    distance: float  # 估计距离(m)


class AvoidanceApp:
    """
    简单避障应用
    
    功能：
    - 向前移动
    - 检测到障碍物时转向
    - 安全距离：0.5米
    
    注意：使用持续控制循环（20Hz）避免底盘超时停止
    """
    
    def __init__(self):
        self.chassis: Optional[ChassisArbiterClient] = None
        self.vision: Optional[VisionSubscriber] = None
        
        # 参数
        self.safe_distance = 0.5   # 安全距离(米)
        self.forward_speed = 0.3   # 前进速度
        self.turn_speed = 0.8      # 转向速度
        self.control_rate = 20     # 控制频率(Hz)，必须 > 1Hz
        
        self.running = False
        # 当前速度（用于平滑）
        self.current_vx = 0.0
        self.current_vz = 0.0
        
    def initialize(self) -> bool:
        """初始化"""
        try:
            self.chassis = ChassisArbiterClient("tcp://localhost:5556")
            self.vision = VisionSubscriber("tcp://localhost:5560")
            self.vision.start()
            logger.info("避障应用初始化完成")
            return True
        except Exception as e:
            logger.error(f"初始化失败: {e}")
            return False
    
    def detect_obstacles(self, frame) -> List[Obstacle]:
        """
        检测障碍物（简化示例）
        实际应用应使用深度相机或立体视觉
        """
        obstacles = []
        h, w = frame.shape[:2]
        
        # 示例：检测画面下方的大面积区域
        # 实际应使用语义分割或深度估计
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 简单的地面检测（示例）
        bottom_region = gray[int(h*0.7):, :]
        
        # 如果下方区域有物体（像素值变化大）
        if np.std(bottom_region) > 50:
            # 估计距离（基于面积）
            distance = 1.0  # 简化估计
            obstacles.append(Obstacle(
                x=w//2, y=int(h*0.8),
                width=w//3, distance=distance
            ))
        
        return obstacles
    
    def decide_action(self, obstacles: List[Obstacle]) -> tuple:
        """
        决策
        
        Returns:
            (vx, vy, vz): 目标速度指令
        """
        if not obstacles:
            # 无障碍，前进
            return self.forward_speed, 0.0, 0.0
        
        # 找到最近的障碍物
        closest = min(obstacles, key=lambda o: o.distance)
        
        if closest.distance > self.safe_distance:
            # 距离安全，继续前进
            return self.forward_speed, 0.0, 0.0
        
        # 需要避障
        logger.info(f"检测到障碍物，距离: {closest.distance:.2f}m")
        
        # 根据障碍物位置决定转向
        frame_center = 640  # 假设分辨率
        if closest.x < frame_center:
            # 障碍物在左边，右转
            return 0.0, 0.0, -self.turn_speed
        else:
            # 障碍物在右边，左转
            return 0.0, 0.0, self.turn_speed
    
    def smooth_velocity(self, target_vx: float, target_vz: float, alpha: float = 0.3) -> tuple:
        """速度平滑，避免突变"""
        self.current_vx = alpha * target_vx + (1 - alpha) * self.current_vx
        self.current_vz = alpha * target_vz + (1 - alpha) * self.current_vz
        return self.current_vx, self.current_vz
    
    def run(self):
        """主循环 - 持续控制模式（避免超时）"""
        if not self.initialize():
            return
        
        self.running = True
        logger.info("避障应用启动")
        
        # 控制周期（秒）
        dt = 1.0 / self.control_rate
        
        try:
            while self.running:
                loop_start = time.time()
                
                # 读取图像
                frame_id, frame = self.vision.read_frame()
                if frame is None:
                    time.sleep(dt)
                    continue
                
                # 检测障碍物
                obstacles = self.detect_obstacles(frame)
                
                # 决策（目标速度）
                target_vx, _, target_vz = self.decide_action(obstacles)
                
                # 速度平滑
                vx, vz = self.smooth_velocity(target_vx, target_vz)
                
                # ⚠️ 关键：持续发送指令以维持控制权（心跳模式）
                self.chassis.send_command(
                    vx=vx, vy=0.0, vz=vz,
                    source="auto", priority=3
                )
                
                # 精确控制循环频率
                elapsed = time.time() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            logger.info("用户中断")
        finally:
            self.stop()
    
    def stop(self):
        """停止应用"""
        logger.info("正在停止避障应用...")
        self.running = False
        
        # 发送停止指令
        if self.chassis:
            self.chassis.send_command(
                vx=0, vy=0, vz=0,
                source="auto", priority=3
            )
        
        if self.vision:
            self.vision.stop()
        
        logger.info("避障应用已停止")


def main():
    app = AvoidanceApp()
    app.run()


if __name__ == "__main__":
    import cv2  # 延迟导入
    main()
```

---

## 调试与测试

### 1. 日志调试

```python
# 设置日志级别
import logging
logging.getLogger().setLevel(logging.DEBUG)

# 关键位置添加日志
logger.debug(f"当前速度: vx={vx}, vz={vz}")
logger.debug(f"检测目标: {len(detections)} 个")
```

### 2. 模拟模式

如果硬件不可用，可以创建模拟客户端：

```python
class MockChassisClient:
    """模拟底盘客户端，用于测试"""
    
    def send_command(self, vx, vy, vz, source, priority):
        print(f"[模拟] 速度: vx={vx}, vy={vy}, vz={vz}")
        return type('Response', (), {'success': True})()
```

### 3. 单元测试

在 `software/src/tests/` 下创建测试：

```python
# tests/test_my_app.py
import unittest
from applications.my_app.app import MyApplication

class TestMyApp(unittest.TestCase):
    
    def test_initialization(self):
        app = MyApplication()
        self.assertEqual(app.state.value, "idle")
    
    def test_state_transition(self):
        app = MyApplication()
        # 测试状态转换
        app.state = AppState.RUNNING
        self.assertEqual(app.state.value, "running")

if __name__ == "__main__":
    unittest.main()
```

运行测试：

```bash
cd software/src
python -m tests.test_my_app
```

### 4. 性能分析

```python
import time

# 测量处理时间
start = time.time()
result = process_frame(frame)
elapsed = (time.time() - start) * 1000
logger.debug(f"处理耗时: {elapsed:.1f}ms")
```

---

## 底盘控制重要说明

### 超时机制

HomeBot 底盘服务实现了**控制权超时保护机制**：

| 参数 | 值 | 说明 |
|------|-----|------|
| 超时时间 | **1000ms (1秒)** | 最后一次指令后的等待时间 |
| 超时行为 | 自动释放控制权 | 控制权归零，底盘停止 |
| 安全频率 | **> 1Hz** | 最小发送频率 |
| 推荐频率 | **20-50Hz** | 平滑控制频率 |

### ❌ 错误示例（会超时停止）

```python
# 错误：发送一次指令后等待
chassis.send_command(vx=0.5, vy=0, vz=0, source="auto", priority=3)
time.sleep(2)  # 超过1秒，底盘已自动停止！
chassis.send_command(vx=0, vy=0, vz=0, source="auto", priority=3)  # 停止指令已无效
```

### ✅ 正确示例（心跳模式）

```python
# 正确：持续发送指令维持运动
import time

start_time = time.time()
duration = 2.0  # 运行2秒
control_rate = 20  # 20Hz

while time.time() - start_time < duration:
    chassis.send_command(vx=0.5, vy=0, vz=0, source="auto", priority=3)
    time.sleep(1.0 / control_rate)  # 每50ms发送一次

# 最后发送停止指令
chassis.send_command(vx=0, vy=0, vz=0, source="auto", priority=3)
```

### 控制模式对比

| 模式 | 适用场景 | 实现方式 |
|------|---------|---------|
| **持续控制** | 实时控制（避障、跟随） | 主循环中以20-50Hz频率发送 |
| **定时控制** | 固定动作（画正方形） | while循环+time.sleep直到时间结束 |
| **事件控制** | 触发式动作（语音指令） | 事件触发时发送，需持续维护 |

---

## 最佳实践

1. **优先级使用规范**
   - `web` (1): 网页遥控
   - `voice` (2): 语音控制
   - `auto` (3): 自动模式（跟随、避障等）
   - `emergency` (4): 紧急停止

2. **底盘控制规范**
   - **永远不要**使用"发送→延时→停止"模式
   - **始终使用**持续发送（心跳）模式，频率 > 1Hz
   - 停止时显式发送零速度指令
   - 使用速度平滑避免突变

3. **资源管理**
   - 使用 `try/finally` 或上下文管理器确保资源释放
   - 异常时确保底盘停止

4. **错误处理**
   - 网络异常时重连
   - 服务不可用时优雅降级
   - 底盘无响应时停止并重试

5. **状态机设计**
   - 明确定义状态转换
   - 避免状态混乱
   - 停止状态时停止发送指令

6. **代码组织**
   - 单一职责原则
   - 检测/跟踪/控制分离
   - 控制逻辑集中在一个循环中

---

## 常见问题

**Q: 底盘不响应指令？**

A: 检查：
1. 底盘服务是否已启动
2. 控制源优先级是否正确
3. 是否有更高优先级的控制源占用

**Q: 机器人运动一下就停了？**

A: 这是**1000ms超时机制**导致的。底盘服务会在最后一次指令后 1 秒自动停止。解决：
- 使用**持续发送（心跳）模式**，控制频率 > 1Hz
- 参考本文档"底盘控制重要说明"章节
- 错误示例：`send_command() → sleep(2) → stop()`
- 正确示例：`while循环中以20Hz频率持续send_command()`

**Q: 如何获取摄像头图像？**

A: 使用 `VisionSubscriber` 订阅视觉服务发布的图像流。

**Q: 应用如何与Web端通信？**

A: 可以：
1. 通过底盘服务间接通信
2. 添加自定义 ZeroMQ 通信
3. 使用共享状态/数据库

**Q: 如何调试视觉算法？**

A: 使用 `--display` 参数显示调试窗口，可视化检测结果。

---

*文档版本: 1.0*
*最后更新: 2026-03-13*
