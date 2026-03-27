# 用 Picoclaw (小龙虾) 控制 HomeBot 机器人

> "Every bit helps, every bit matters." - Picoclaw

## 概述

[Picoclaw](https://github.com/sipeed/picoclaw) 是一个超轻量级的开源 AI 助手，通过扩展技能系统，可以轻松实现对 HomeBot 机器人的局域网智能控制。本文档介绍如何配置和使用 Picoclaw 控制 HomeBot 底盘和机械臂。

## 前置条件

1. 已经成功部署并运行 HomeBot 底盘和机械臂服务
2. HomeBot 机器人和 Picoclaw 运行在同一个局域网中
3. 已知 HomeBot 机器人的 IP 地址（本文示例：`192.168.1.13`）
4. 如果 Picoclaw 和 HomeBot 运行在同一主机上，IP 地址可使用本机回环地址 `127.0.0.1`

## 技能安装

Picoclaw 通过技能系统扩展功能，HomeBot 提供了一个统一的控制技能：

| 技能名称 | 功能说明 |
|---------|---------|
| **homebot** | 集成底盘控制、机械臂控制、视觉查询、姿态动作于一体 |

技能文件位于本仓库的 [`skills/homebot/`](https://github.com/choco-robot/homebot/tree/main/skills/homebot) 目录下。

### 安装方式

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

### 1. 默认端口配置

- 底盘默认端口：`5556`
- 机械臂默认端口：`5557`
- 视频服务默认端口：`5560`

### 2. 配置机器人 IP

编辑 `skills/homebot/scripts/robot_config.py`：

```python
ROBOT_IP = "192.168.1.13"  # 修改为你的机器人IP
```

### 3. MCP 服务器配置（推荐）

HomeBot 技能内置 MCP 服务器，配置后 Picoclaw 启动时会自动加载，LLM 可直接调用机器人控制工具。

编辑 Picoclaw 的 `config.json` 配置文件（路径：`~/.picoclaw/config.json`）：

```json
{
  "tools": {
    "mcp": {
      "enabled": true,
      "servers": {
        "homebot": {
          "enabled": true,
          "command": "python",
          "args": [
            "{{skill_path}}/mcp_homebot_server.py"
          ]
        }
      }
    }
  }
}
```

**配置说明：**
- `{{skill_path}}` 是 Picoclaw 的魔法变量，会自动替换为技能实际安装路径
- `command` 可以是 `python`（需在 PATH 中）或 Python 解释器的完整路径
- `args` 是数组格式，包含 MCP 服务器脚本路径
- 环境变量（如 `HOMEBOT_IP`、`ARK_API_KEY`）需在启动 Picoclaw 前设置

### 4. 通信协议

HomeBot 和 Picoclaw 使用 **ZeroMQ REQ-REP** 协议进行局域网通信：

- Picoclaw 作为客户端发起请求
- HomeBot 服务端处理请求并返回执行结果
- 请求执行完成后立即返回，支持同步控制

## 使用方式

### 自然语言控制（MCP 方式）

配置 MCP 服务器后，你可以直接用自然语言控制机器人：

```
你: 让机器人前进 20 厘米
AI: 我会使用 chassis_forward 工具让机器人前进 20 厘米... ✅ 机器人前进 20 厘米完成

你: 查看机械臂当前位置
AI: 我来获取机械臂当前各关节的角度位置... 

你: 机器人看到了什么？
AI: 我来捕获机器人摄像头画面并分析... [调用 robot_what_does_robot_see]
```

支持的指令示例：
```
"让机器人前进 20 厘米"
"右转 90 度"
"把机械臂回原点"
"让机械臂跳个舞"
"让机器人挥挥手"
"机器人看到了什么？"
"设置肩关节为 30 度，肘关节为 45 度"
"紧急停止"
```

MCP 工具列表：

| 工具名称 | 功能描述 |
|---------|---------|
| `chassis_forward` | 机器人前进指定距离（厘米） |
| `chassis_backward` | 机器人后退指定距离（厘米） |
| `chassis_left` | 机器人左转指定角度（度数） |
| `chassis_right` | 机器人右转指定角度（度数） |
| `chassis_stop` | 紧急停止机器人底盘 |
| `arm_move_joint` | 移动机械臂指定关节到目标角度 |
| `arm_get_positions` | 获取机械臂所有关节当前位置 |
| `arm_stop` | 停止机械臂所有运动 |
| `robot_what_does_robot_see` | 捕获机器人画面并 AI 分析场景 |

验证 MCP 服务器加载：配置完成后，重启 Picoclaw，执行 `/tools` 查看工具列表。

### 命令行控制

如果不想使用 MCP，也可以直接运行脚本控制：

**底盘控制：**
```bash
python scripts/chassis_control.py forward 10   # 前进10厘米
python scripts/chassis_control.py backward 15  # 后退15厘米
python scripts/chassis_control.py left 45      # 左转45度
python scripts/chassis_control.py right 90     # 右转90度
python scripts/chassis_control.py stop         # 紧急停止
```

**机械臂控制：**
```bash
python scripts/arm_control.py home                           # 回原点
python scripts/arm_control.py joint base 0                   # 设置单个关节
python scripts/arm_control.py joints "base:0,shoulder:30,elbow:45"  # 设置多个关节
python scripts/arm_control.py gripper open                   # 打开夹爪
python scripts/arm_control.py gripper close                  # 关闭夹爪
python scripts/arm_control.py status                         # 查看状态
```

**姿态动作：**
```bash
python scripts/arm_gestures.py wave    # 挥挥手
python scripts/arm_gestures.py nod     # 点点头
python scripts/arm_gestures.py shake   # 摇摇头
python scripts/arm_gestures.py all     # 执行全部动作
```

**视觉查询：**
```bash
python scripts/what_does_robot_see_workflow.py              # 捕获并分析
python scripts/what_does_robot_see_workflow.py --no-analysis # 仅捕获
```

**机械臂舞蹈：**
```bash
python scripts/dance.py
```

## 高级用法：编写自动化脚本

你可以直接在 Picoclaw 中让 AI 帮你编写控制脚本，实现复杂的自动化任务。

### 示例：机械臂控制脚本

```python
import sys
sys.path.insert(0, 'skills/homebot/scripts')

from arm_control import HomeBotArmController
from robot_config import ROBOT_IP, ARM_PORT
import time

# 连接机械臂
arm = HomeBotArmController(robot_ip=ROBOT_IP, robot_port=ARM_PORT)

# 回原点
arm.move_home()
time.sleep(3)

# 设置单个关节
arm.set_joint_angle("shoulder", 30.0)
time.sleep(2)

# 同时设置多个关节
arm.set_joint_angles({
    "base": 0,
    "shoulder": 20,
    "elbow": 45,
    "wrist_flex": 0
})
time.sleep(2)

# 打开夹爪
arm.open_gripper()

# 最后回原点
arm.move_home()
arm.close()
```

### 示例：自动巡线任务组合

```python
import sys
sys.path.insert(0, 'skills/homebot/scripts')

from chassis_control import HomeBotChassisController
from robot_config import ROBOT_IP, CHASSIS_PORT
import time

robot = HomeBotChassisController(ip=ROBOT_IP, port=CHASSIS_PORT)

# 走一个正方形
for _ in range(4):
    robot.forward_cm(50)   # 前进50厘米
    time.sleep(2)
    robot.left_deg(90)     # 左转90度
    time.sleep(2)

robot.close()
```

## 故障排查

### 连接超时

**问题**: 发送命令后一直等待，返回连接超时

**解决方法**:
1. 检查机器人是否开机
2. 检查 IP 地址是否正确（使用 `ping 192.168.1.13` 测试连通性）
3. 检查 HomeBot 服务是否已经启动（底盘服务、机械臂服务）
4. 确认你的电脑和机器人在同一个 WiFi 网络
5. 检查防火墙设置，确保端口没有被屏蔽
6. 确认端口配置正确（底盘 5556，机械臂 5557，视频 5560）

### 动作方向相反

**问题**: 说后退实际前进

**解决方法**: 检查电机接线，或者在代码中修改速度方向符号。

### 机械臂抖动严重

**问题**: 设置角度后机械臂抖动

**解决方法**:
1. 检查供电电压是否足够（建议 12V 以上电源）
2. 检查舵机扭力是否足够
3. 减小单次动作的角度变化范围，增加延时
4. 降低运动速度参数

### 视觉分析失败

**问题**: 执行 `what_does_robot_see_workflow.py` 时视觉分析失败

**解决方法**:
1. 检查是否已安装 `volcenginesdkarkruntime`：
   ```bash
   pip install volcenginesdkarkruntime
   ```
2. 检查是否已配置 `ARK_API_KEY` 环境变量：
   ```bash
   # Windows
   echo %ARK_API_KEY%
   
   # Linux/Mac
   echo $ARK_API_KEY
   ```
3. 如果不需要 AI 分析，使用 `--no-analysis` 参数仅捕获图像

### MCP 服务器无法加载

**问题**: 在 Picoclaw 中执行 `/tools` 看不到 HomeBot 的工具列表

**解决方法**:
1. **检查配置文件路径**：确保编辑的是 Picoclaw 实际使用的 `config.json` 文件
   - Windows: `%USERPROFILE%\.picoclaw\config.json`
   - Linux/Mac: `~/.picoclaw\config.json`

2. **检查 JSON 格式**：确保格式正确，特别是引号和逗号：
   ```json
   "mcp": {
     "enabled": true,
     "servers": {
       "homebot": {
         "enabled": true,
         "command": "python",
         "args": [
           "{{skill_path}}/mcp_homebot_server.py"
         ]
       }
     }
   }
   ```

3. **检查 Python 路径**：确保 `command` 指向的 Python 可以访问技能目录
   - 如果 Python 不在 PATH 中，使用完整路径：
     ```json
     "command": "C:\\Users\\YourName\\.picoclaw\\venv\\Scripts\\python.exe"
     ```

4. **手动测试 MCP 服务器**：
   ```bash
   cd ~/.picoclaw/skills/homebot
   python mcp_homebot_server.py
   # 如果没有报错，说明脚本本身正常
   ```

5. **检查 Picoclaw 日志**：查看启动日志中是否有 MCP 服务器相关的错误信息

6. **重启 Picoclaw**：修改配置后需要完全重启 Picoclaw 才能生效

## 技能开发

你可以基于现有的 HomeBot 技能，进一步开发更高级的功能：

- **视觉引导抓取**: 结合摄像头和物体识别，自动抓取物品
- **语音控制**: 接入语音识别，直接用语音说话控制
- **远程访问**: 结合内网穿透，实现外网远程控制机器人
- **SLAM 导航**: 结合激光雷达，实现自主导航
- **自定义姿态动作**: 基于 `arm_gestures.py` 开发更多有趣的机械臂动作

## 相关链接

- [Picoclaw 官方仓库](https://github.com/sipeed/picoclaw)
- [Picoclaw MCP 配置文档](https://docs.picoclaw.io/docs/configuration/tools/#mcp-model-context-protocol) - 详细 MCP 服务器配置说明
- [HomeBot 项目主页](../README.md)
- [HomeBot 技能目录](../skills/homebot/) - 本地技能文件位置
  - [SKILL.md](../skills/homebot/SKILL.md) - 技能详细说明文档
  - [config.json](../skills/homebot/config.json) - Picoclaw 配置示例（参考 MCP 配置部分）
  - [scripts/chassis_control.py](../skills/homebot/scripts/chassis_control.py) - 底盘控制脚本
  - [scripts/arm_control.py](../skills/homebot/scripts/arm_control.py) - 机械臂控制脚本
  - [scripts/arm_gestures.py](../skills/homebot/scripts/arm_gestures.py) - 姿态动作脚本
  - [scripts/what_does_robot_see_workflow.py](../skills/homebot/scripts/what_does_robot_see_workflow.py) - 视觉查询脚本
  - [mcp_homebot_server.py](../skills/homebot/mcp_homebot_server.py) - MCP 服务器

## 许可证

和 Picoclaw 一样，本文档遵循 MIT 许可证，自由开源。

---

*📌 「小龙虾虽小，五脏俱全，智能控制就是这么简单」*
