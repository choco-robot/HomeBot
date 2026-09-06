# LeRobot 生态接入

本文档介绍 HomeBot 接入 LeRobot 生态的技术方案：Robot 适配器的注册机制、数据采集流程，以及训练后模型的推理部署链路。

> 对应分支：`feature/lerobot`，代码位于 `software/src/applications/imitation_learning/`。

## 概述

接入 LeRobot 生态的目标是让 HomeBot 可以直接使用官方工具链完成模仿学习的完整闭环：

- **数据采集**：`lerobot.record` + `so101_leader` 主臂遥操作，数据集与官方生态（SO-101、LeKiwi 等）格式互通
- **训练**：使用官方 ACT / SmolVLA 等策略实现，无需改动
- **推理部署**：通过 HomeBot 自研的 `policy_runner` 服务级链路下发动作，保留急停与仲裁保护

## 模块组成

| 模块 | 说明 |
|------|------|
| `robot.py` | `HomeBotRobot`：LeRobot `Robot` 接口适配器，注册类型 `type="homebot"` |
| `chassis_adapter.py` | 底盘形态抽象：`omni3`（三轮全向）/ `diff2`（双轮差动）/ `none`（纯臂） |
| `joint_map.py` | HomeBot ↔ LeRobot 关节命名与夹爪单位（度 ↔ 0-100）映射 |
| `policy_runner.py` | 策略推理部署（服务级，经仲裁器下发） |
| `verify_calibration.py` | 校准后两套坐标系一致性验证 |
| `plugins/lerobot_robot_homebot/`（仓库 `plugins/` 下） | LeRobot 第三方插件封装包，让官方 CLI 能自动发现 HomeBot |

## Robot 适配器的注册机制

`HomeBotRobot` 能被 `--robot.type=homebot` 使用，依赖三层机制：

### 1. 类型注册（导入期副作用）

`robot.py` 中通过 draccus 的 `ChoiceRegistry` 装饰器注册配置类：

```python
@RobotConfig.register_subclass("homebot")
@dataclass(kw_only=True)
class HomeBotRobotConfig(RobotConfig):
    port: str = "COM23"
    chassis_type: str = "none"
    ...
```

注册后，CLI 解析 `--robot.type=homebot` 时 draccus 会把命令行参数反序列化为 `HomeBotRobotConfig` 实例，所有 dataclass 字段自动成为 `--robot.xxx` 参数。

**关键点：注册是 import 时的副作用**——模块必须先被导入，装饰器才会执行。

### 2. 实例化（命名约定 + 模块回退查找）

`lerobot.record` 内部调用 `make_robot_from_config()`，对非内置类型落到 `make_device_from_device_class()`：

1. 配置类名去掉 `Config` 后缀得到设备类名：`HomeBotRobotConfig` → `HomeBotRobot`
2. 按候选模块依次尝试 import 并取出该类：配置类的父包模块 → `父包.设备类名小写`（lerobot 主线还包含配置类自身模块）

因此除"类名严格对应（仅差 `Config` 后缀）"外，还必须遵守官方约定第 4 条：**在父包 `__init__.py` 中暴露设备类**。lerobot 0.5.x 的候选模块不包含配置类自身模块，若只在 `robot.py` 中定义会报 `Could not locate device class`——本项目已在 `applications/imitation_learning/__init__.py` 中导出 `HomeBotRobot` / `HomeBotRobotConfig` 满足该约定。

### 3. 自动发现（`lerobot_robot_` 前缀插件包）

LeRobot 不会扫描任意模块，只自动发现**包名以 `lerobot_robot_` / `lerobot_camera_` / `lerobot_teleoperator_` 等前缀开头**的已安装包（v0.4.0 起支持，v0.4.x 扫描 `sys.path` 上的模块名，新版本扫描已安装 distribution 的元数据名）。发现后自动 import，触发第 1 步的注册。

为此仓库提供了插件封装包 `plugins/lerobot_robot_homebot/`：

```
plugins/lerobot_robot_homebot/
├── pyproject.toml                  # name = "lerobot_robot_homebot"
└── lerobot_robot_homebot/
    └── __init__.py                 # from applications.imitation_learning.robot import ...
```

> 注意两点：包名必须保留下划线形式（`lerobot_robot_homebot` 而非 `lerobot-robot-homebot`），否则发行版元数据名不带 `lerobot_robot_` 前缀，无法通过发现检查；封装包不能放在仓库根目录——根目录下的同名外层目录会被 Python 当作命名空间包，遮蔽已安装的真实插件，导致静默注册失败。

## 环境准备

LeRobot 使用独立环境，不进项目主 venv（官方支持 Python 3.10-3.12，主 venv 是 3.13）：

```bash
# 创建独立环境
uv venv venv-lerobot --python 3.12
venv-lerobot\Scripts\python.exe -m pip install -r requirements-lerobot.txt

# 让 HomeBot 代码可导入
set PYTHONPATH=E:\develop\HomeBot\homebot\software\src   :: Windows
export PYTHONPATH=.../software/src                      # Linux/macOS

# 安装插件封装包（editable），完成后 lerobot CLI 自动发现 homebot 类型
venv-lerobot\Scripts\python.exe -m pip install -e ./plugins/lerobot_robot_homebot
```

验证注册是否生效：

```bash
cd software/src
python -m applications.imitation_learning info --chassis-type omni3
```

## 数据采集

```bash
lerobot.record \
    --robot.type=homebot \
    --robot.port=COM23 \
    --robot.chassis_type=omni3 \
    --robot.cameras="{cam_front: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
    --teleop.type=so101_leader --teleop.port=COMxx ...
```

### 校准约定（重要）

LeRobot 校准会把 homing offset 写入舵机寄存器。执行校准时**必须将机械臂摆放到 HomeBot 零位姿态**（各关节 0°，raw 2048），使 LeRobot 坐标系与 HomeBot 坐标系重合，否则 web 遥控、手柄、语音等现有功能的读数会整体偏移。

校准后运行一致性验证：

```bash
cd software/src
python -m applications.imitation_learning verify-calibration --port COM23
```

### 底盘形态

- `omni3`：三轮全向，轮舵机注册到 lerobot 电机总线（与机械臂共享串口），速度键沿用 LeKiwi 约定（`x.vel` / `y.vel` / `theta.vel`，前两者 m/s，后者 deg/s）
- `diff2`：双轮差动，懒加载 homebot-navi 分支的 `diff_driver`，该分支合并后即用；横向速度自动忽略并告警
- `none`：纯机械臂

## 推理部署

### 推荐链路：policy_runner（自研，服务级）

训练好的模型**不走** LeRobot 官方的 async inference（Policy Server / Robot Client），而是走 HomeBot 自研的服务级链路，复用现有服务通道：

```
vision_service PUB(5560) ──→ VisionSubscriber 取图
arm_service REP(5557)    ──→ query 读关节角度
        ↓
lerobot API 本地加载策略（PreTrainedConfig + pre/post processors）
policy.select_action() 推理
        ↓
arm_service(5557) 下发关节 / chassis_service(5556) 下发速度
——经仲裁器，source="auto"（priority 3），保留急停锁定与 1 秒超时保护
```

用法：

```bash
cd software/src
python -m applications.imitation_learning run-policy \
    --policy <训练输出目录或 HF hub id> \
    --robot-host 192.168.x.x \
    --enable-chassis \
    --task "抓起红色方块"          # VLA 模型需要语言指令
```

特点：

- **推理机与机器人可分离**：策略跑在 GPU 机器上，机器人端只跑原有服务，无需安装 lerobot/torch
- **安全机制完整**：急停、控制权仲裁、超时自动停止全部有效
- `--dry-run` 只组装观测并打印动作、不下发，用于验证链路
- 相机键名默认从策略配置自动推断，多相机模型用 `--camera-key` 指定

### 为什么不走官方 async inference

官方链路（`lerobot.async_inference`）的 Robot Client 直接包住 `Robot` 接口在连硬件的机器上运行，动作经 gRPC 从远端 Policy Server 推回。对 HomeBot 有两个硬伤：

- **绕过仲裁器**：Robot Client 直接写舵机总线，急停锁定、控制源仲裁、超时保护全部失效，手柄/web 抢不回控制权
- 机器人端（或直连硬件的机器）必须安装全套 lerobot + torch，部署变重

官方链路的优势是动作分块排队与网络延迟补偿（适合 SmolVLA 等大模型）。将来如有需要，可借鉴其思路在"远端策略 → 本地执行"之间加一层带仲裁器适配的 client，而不是直接使用官方 Robot Client。

## 注意事项与已知限制

- **已在 lerobot 0.5.1 完成端到端验证**（插件发现 → `--robot.type=homebot` CLI 解析 → `make_robot_from_config` 实例化 → observation/action features 输出），但**尚未连接真机运行**。首次实机运行 `run-policy` 建议先 `--dry-run` 确认观测组装与动作输出合理，再低速开启 `--enable-chassis`
- **底盘无速度回读**：policy_runner 用最近一次下发的速度作为底盘观测（`x.vel` / `y.vel` / `theta.vel`）
- policy_runner 需要 `pyzmq`（已列入 requirements-lerobot.txt）；`load_policy()` 基于 lerobot 主线 API，lerobot 版本升级可能需要微调
- 插件自动发现要求 `lerobot>=0.4.0`

## 后续计划

- `diff2` 底盘适配等待 homebot-navi 分支（SLAM 导航）合并
- 真机闭环验证：数采 → 训练 → `run-policy` 推理

---

*文档创建：2026-09-06*
