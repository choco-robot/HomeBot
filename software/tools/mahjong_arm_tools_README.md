# 麻将机械臂工具集

本目录包含麻将机器人机械臂控制相关的调试和标定工具。

## 工具清单

### 1. test_kinematics.py - 运动学测试
测试3DOF逆运动学的正确性和工作空间。

```bash
cd software
python tools/test_kinematics.py

# 交互式测试
python tools/test_kinematics.py -i

# 生成工作空间点云
python tools/test_kinematics.py --pointcloud
```

### 2. calibration_tool.py - 标定工具
交互式四点标定工具，建立图像坐标到机械臂坐标的映射。

**使用方法：**
```bash
cd software
python tools/calibration_tool.py
```

**标定流程：**
1. 确保摄像头和机械臂服务都已启动
2. 在牌桌四个角放置标记牌
3. 按提示在图像中点击每个角点
4. 输入对应的机械臂坐标（或移动机械臂到该位置）
5. 保存标定结果

**参数：**
- `--camera`: 摄像头设备ID (默认: 0)
- `--arm`: 机械臂服务地址 (默认: tcp://localhost:5557)
- `--load`: 加载已有标定文件
- `--test`: 测试模式

### 3. arm_debug_tool.py - 综合调试工具
整合了原高层 ZMQ 调试与底层串口控制功能，支持通过机械臂服务或直连串口两种方式操作。

**交互模式（需启动 ArmService）：**
```bash
cd software
python tools/arm_debug_tool.py
```

**功能菜单：**
1. **测试单个关节运动** - 测试特定关节的运动范围和精度
2. **测试定位精度** - 验证末端定位准确性
3. **测试完整出牌序列** - 执行抓取-移动-出牌全流程
4. **PTP 点到点运动** - 快速笛卡尔空间定位
5. **直线插补运动** - 笛卡尔空间直线轨迹
6. **工作空间测试** - 测试可达空间网格
7. **交互式笛卡尔控制** - 直接输入坐标控制
8. **底层串口调试** - 直连舵机总线（扭矩使能/失能、复位、读状态）

**底层快捷模式（无需启动 ArmService，直连串口）：**
```bash
# 一键失能扭矩
python tools/arm_debug_tool.py --disable

# 一键复位
python tools/arm_debug_tool.py --reset

# 查看当前状态
python tools/arm_debug_tool.py --status

# 指定串口
python tools/arm_debug_tool.py --port COM4 --reset
```

## 快速开始

### 第一步：测试运动学
```bash
python tools/test_kinematics.py --num-tests 20
```

### 第二步：标定
```bash
# 确保 ArmService 已启动
python -m services.motion_service.arm_service

# 在另一个终端运行标定工具
python tools/calibration_tool.py
```

### 第三步：调试机械臂
```bash
# 高层功能（ZMQ）
python tools/arm_debug_tool.py

# 底层功能（串口）
python tools/arm_debug_tool.py --reset
python tools/arm_debug_tool.py --status
```

## 坐标系说明

### 机械臂坐标系
- **原点**: 机械臂基座中心
- **X轴**: 向前（机械臂正前方）
- **Y轴**: 向左
- **Z轴**: 向上

### 牌桌坐标系
- **原点**: 牌桌参考点（通常是一个角）
- 通过 `arm_offset_x/y` 与机械臂坐标系关联

### 图像坐标系
- **原点**: 图像左上角
- **u**: 向右（X轴）
- **v**: 向下（Y轴）

## 配置文件

相关配置项在 `src/configs/config.py` 中的 `MahjongConfig`:

```python
# 机械臂偏移（牌桌原点到机械臂基座）
arm_offset_x: float = 0.0
arm_offset_y: float = -200.0

# Homography矩阵（标定后自动更新）
homography_matrix: list = [1,0,0, 0,1,0, 0,0,1]
```

## 故障排除

### 无法连接到 ArmService
- 检查 ArmService 是否已启动
- 检查地址和端口是否正确
- 检查防火墙设置

### 标定误差过大
- 确保4个点分布合理（覆盖整个工作区域）
- 检查输入的机械臂坐标是否准确
- 确保摄像头与牌桌相对位置固定

### 运动失败
- 检查目标位置是否在可达范围内
- 检查关节角度是否超出限制
- 查看 ArmService 日志

## 安全注意事项

1. **运行前确保：**
   - 机械臂运动范围内无障碍物
   - 紧急停止按钮触手可及
   - 初始速度设置较低（<500）

2. **测试时：**
   - 先进行小幅度运动测试
   - 逐步增加运动范围
   - 观察是否有异常声音或振动

3. **紧急停止：**
   - 按 Ctrl+C 停止程序
   - ArmService 会在超时后自动释放控制权
