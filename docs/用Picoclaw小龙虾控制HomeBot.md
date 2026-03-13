# 用 Picoclaw (小龙虾) 控制 HomeBot 机器人

> "Every bit helps, every bit matters." - Picoclaw

## 概述

[Picoclaw](https://github.com/sipeed/picoclaw) 是一个超轻量级的开源 AI 助手，通过扩展技能系统，可以轻松实现对 HomeBot 机器人的局域网智能控制。本文档介绍如何配置和使用 Picoclaw 控制 HomeBot 底盘和机械臂。

## 前置条件

1. 已经成功部署并运行 HomeBot 底盘和机械臂服务
2. HomeBot 机器人和 Picoclaw 运行在同一个局域网中
3. 已知 HomeBot 机器人的 IP 地址（本文示例：`192.168.1.13`）

## 技能安装

Picoclaw 通过技能系统扩展功能，HomeBot 提供了以下专用控制技能：

| 技能名称 | 功能说明 |
|---------|---------|
| **homebot-controller** | 底盘运动控制（前进/后退/旋转） |
| **homebot-arm-controller** | 机械臂关节控制 |
| **what-does-robot-see** | 一键查询机器人摄像头视角 |

技能文件位于本仓库的 [`skills/`](https://github.com/choco-robot/homebot/tree/main/skills) 目录下。

### 安装方式

#### 方式一：通过 GitHub 链接直接安装（推荐）

在 Picoclaw 中执行以下命令安装：

```bash
# 安装底盘控制技能
install_skill url="https://github.com/choco-robot/homebot/tree/main/skills/homebot"

```

#### 方式二：本地手动安装

1. 克隆 HomeBot 仓库到本地：
```bash
git clone https://github.com/choco-robot/homebot.git
cd homebot/skills
```

2. 将技能目录复制到 Picoclaw 的技能目录中：
```bash
# 假设 Picoclaw 技能目录为 ~/.picoclaw/skills
cp -r homebot ~/.picoclaw/skills/
```

3. 重启 Picoclaw 或刷新技能列表即可使用。

## 配置连接

### 1. 修改 IP 地址

在使用前，请确认技能配置中的 IP 地址与你的实际机器人 IP 一致：

- 底盘默认端口：`5556`
- 机械臂默认端口：`5555`
- 视频服务默认端口：`5560`

### 2. 通信协议

HomeBot 和 Picoclaw 使用 **ZeroMQ REQ-REP** 协议进行局域网通信：

- Picoclaw 作为客户端发起请求
- HomeBot 服务端处理请求并返回执行结果
- 请求执行完成后立即返回，支持同步控制

## 使用方式

### 自然语言控制

Picoclaw 支持自然语言直接控制，你可以直接说：

```
"让机器人前进 20 厘米"
"右转 90 度"
"把机械臂回原点"
"让机械臂跳个舞"
"机器人看到了什么？"
"设置关节1为 90 度，关节3为 45 度"
"紧急停止"
```

Picoclaw 会自动解析你的指令，转换成控制命令发送给机器人。

### 底盘控制命令示例

| 自然语言指令 | 实际执行动作 |
|-------------|-------------|
| "前进 10 厘米" | `forward 10` |
| "后退 15 厘米" | `backward 15` |
| "左转 45 度" | `left 45` |
| "右转 90 度" | `right 90` |
| "停车" / "紧急停止" | 发送停止指令 |

### 机械臂控制命令示例

| 自然语言指令 | 实际执行动作 |
|-------------|-------------|
| "回原点" / "复位" | 所有关节回原点位置 |
| "一号关节 90 度" | 设置关节 1 角度为 90° |
| "关节1 0度，关节2 30度，关节3 -20度" | 同时设置多个关节 |
| "张开夹爪" / "闭合夹爪" | 控制夹爪开合 |
| "跳个舞" | 执行预编好的舞蹈脚本 |

### 查看机器人视角

使用 `what-does-robot-see` 技能，只需一句话：

```
机器人看到了什么？
```

技能会自动：
1. 连接机器人视频服务获取最新一帧图像
2. 保存图像并发送给你
3. 调用多模态大模型自动描述图像内容

## 高级用法：编写自动化脚本

你可以直接在 Picoclaw 中让 AI 帮你编写控制脚本，实现复杂的自动化任务。

### 示例：机械臂舞蹈脚本

```python
from homebot_arm_controller import HomeBotArmController

# 连接机械臂
robot = HomeBotArmController("tcp://192.168.1.13:5555")

# 回原点
robot.go_home()
time.sleep(3)

# 舞蹈动作序列
dance_steps = [
    # (j1, j2, j3, j4, j5, delay)
    (90, 30, 0, 0, 0, 1.5),
    (0, 30, 0, 0, 0, 1.5),
    # ... 更多动作
]

# 执行舞蹈
for step in dance_steps:
    robot.set_all_joints(*step[:-1])
    time.sleep(step[-1])

# 最后回原点
robot.go_home()
```

然后你可以直接在 Picoclaw 环境中运行这个脚本。

### 示例：自动巡线任务组合

```python
from homebot_controller import HomeBotController
import time

robot = HomeBotController("tcp://192.168.1.13:5556")

# 走一个正方形
for _ in range(4):
    robot.forward_cm(50)   # 前进50厘米
    time.sleep(2)
    robot.left_deg(90)     # 左转90度
    time.sleep(2)
```

## 故障排查

### 连接超时

**问题**: 发送命令后一直等待，返回连接超时

**解决方法**:
1. 检查机器人是否开机
2. 检查 IP 地址是否正确
3. 检查服务是否已经启动
4. 确认你的电脑和机器人在同一个 WiFi 网络
5. 检查防火墙设置，确保端口没有被屏蔽

### 动作方向相反

**问题**: 说后退实际前进

**解决方法**: 检查电机接线，或者在代码中修改速度方向符号即可。

### 机械臂抖动严重

**问题**: 设置角度后机械臂抖动

**解决方法**:
1. 检查供电电压是否足够（建议 12V 以上电源）
2. 检查舵机扭力是否足够
3. 减小单次动作的角度变化范围，增加延时

## 技能开发

你可以基于现有的 HomeBot 技能，进一步开发更高级的功能：

- **视觉引导抓取**: 结合摄像头和物体识别，自动抓取物品
- **语音控制**: 接入语音识别，直接用语音说话控制
- **远程访问**: 结合内网穿透，实现外网远程控制机器人
- **SLAM 导航**: 结合激光雷达，实现自主导航

## 相关链接

- [Picoclaw 官方仓库](https://github.com/sipeed/picoclaw)
- [HomeBot 项目主页](../README.md)
- [Picoclaw 技能目录](../skills/) - 本地技能文件位置
  - [homebot-controller](../skills/homebot-controller/) - 底盘控制技能
  - [homebot-arm-controller](../skills/homebot-arm-controller/) - 机械臂控制技能
  - [what-does-robot-see](../skills/what-does-robot-see/) - 视觉查询技能
- [机械臂控制开发说明](../software/arm_service/README.md)
- [底盘控制开发说明](../software/chassis_service/README.md)

## 许可证

和 Picoclaw 一样，本文档遵循 MIT 许可证，自由开源。

---

*📌 「小龙虾虽小，五脏俱全，智能控制就是这么简单」*
