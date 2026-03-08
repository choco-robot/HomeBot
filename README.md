# HomeBot

HomeBot 是一个面向家庭场景的轻量级机器人项目，包含硬件设计和软件实现。

<img width="400" height="500" alt="效果图" src="https://github.com/user-attachments/assets/972daf26-fcf7-42d6-bbcd-2da5bf5c6ad2" />


## 项目结构

```
homebot/
├── docs/                      # 文档
├── hardware/                  # 硬件设计文件（SolidWorks, STL）
│   └── structure/
├── software/                  # 软件代码
│   ├── src/                   # 源代码
│   │   ├── common/            # 公共工具、消息定义、配置类
│   │   ├── configs/           # 运行时配置
│   │   ├── applications/      # 应用层模块（遥控、语音、跟随等）
│   │   ├── services/          # 服务层（运动、视觉、语音等）
│   │   │   └── chassis_arbiter/   # 底盘控制仲裁服务
│   │   │       └── control_sources/  # 控制源客户端（web/voice/auto/emergency）
│   │   ├── hal/               # 硬件抽象层（摄像头、底盘、机械臂、音频）
│   │   │   └── chassis/
│   │   └── tests/             # 测试代码
│   ├── models/                # 机器学习模型
│   ├── tools/                 # 数据采集、训练脚本
│   └── run_tests.py           # 自动化测试脚本
├── requirements.txt           # Python 依赖
├── pyproject.toml             # 构建系统配置
├── setup.py                   # 包安装配置
└── README.md                  # 项目说明
```

## 特性

- 纯Python实现，轻松跨平台，笔记本、树莓派等设备均可运行
- 采用分层模块化架构、ZeroMQ 作为通信总线
- 支持多种应用：手机遥控、语音交互、模仿学习、人跟随
- 硬件抽象层可适配不同传感器与执行器
- Python 配置类和 JSON 配置模板方便部署
- 兼容lerobot框架下的模仿学习、VLA算法（待开发）

## 安装

```bash
# 在项目根目录
python -m pip install -e .
```

或手动安装依赖：

```bash
pip install -r requirements.txt
```

## 快速开始

todo.

## 配置

todo.

## 开发规范

todo.

## 扩展

可以在后续迭代中添加 MQTT 桥接、多机器人协同、SLAM 等特性。

---

*Generated on 2026-03-08 by HomeBot development team.*
