#!/usr/bin/env python3
"""
HomeBot 机械臂 WLAN 主从遥操作应用入口

使用方法:
    cd software/src
    python -m applications.arm_teleop

    # 启动即开启遥操作，并指定从端地址
    python -m applications.arm_teleop --enable --slave-addr tcp://192.168.1.100:5557

    # 启动并直接录制（退出或按 r 停止时保存）
    python -m applications.arm_teleop --enable --record trajectories/demo.json

    # 回放一次
    python -m applications.arm_teleop --playback trajectories/demo.json

    # 0.5 倍速循环 3 次
    python -m applications.arm_teleop --playback trajectories/demo.json --playback-speed 0.5 --loop 3

    # 无限循环
    python -m applications.arm_teleop --playback trajectories/demo.json --loop-forever
"""
import sys
import os
import argparse
import logging

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from configs import get_config
from common.logging import get_logger
from .app import ArmTeleopApp, build_master_arm_config
from .gui import main as gui_main

logger = get_logger(__name__)


def main():
    """主入口"""
    parser = argparse.ArgumentParser(
        description="HomeBot 机械臂 WLAN 主从遥操作",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
说明:
  - 主臂直接通过串口读取，从臂通过 ZeroMQ 连接远端 arm_service
  - 默认关闭遥操作，需使用 --enable 开启
  - 关闭遥操作后会发送一次慢速保持命令，让从臂停在当前位置
  - 运行时热键: [e] 开关遥操作  [r] 开始/停止录制  [p] 播放默认轨迹  [s] 停止回放  [q] 退出
  - 使用 --gui 启动图形界面
        """
    )

    parser.add_argument(
        "--slave-addr",
        type=str,
        default=None,
        help="从端 arm_service 地址，如 tcp://192.168.1.100:5557"
    )
    parser.add_argument(
        "--enable",
        action="store_true",
        help="启动时开启遥操作（默认关闭）"
    )
    parser.add_argument(
        "--torque-off",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="是否关闭主臂扭矩以方便手动拖动（默认开启）"
    )
    parser.add_argument(
        "--read-rate",
        type=float,
        default=None,
        help="主臂读取频率（Hz）"
    )
    parser.add_argument(
        "--send-rate",
        type=float,
        default=None,
        help="向从臂下发频率（Hz）"
    )
    parser.add_argument(
        "--deadband",
        type=float,
        default=None,
        help="角度死区（度），小于该值不发送"
    )
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=None,
        help="速度自适应比例因子（<=0 则使用固定 default_speed）"
    )
    parser.add_argument(
        "--record",
        type=str,
        default=None,
        help="启动时开始录制并保存到指定文件"
    )
    parser.add_argument(
        "--playback",
        type=str,
        default=None,
        help="启动后回放指定轨迹文件"
    )
    parser.add_argument(
        "--playback-speed",
        type=float,
        default=1.0,
        help="回放速度倍数（默认 1.0）"
    )
    parser.add_argument(
        "--loop",
        type=int,
        default=1,
        help="回放循环次数（默认 1）"
    )
    parser.add_argument(
        "--loop-forever",
        action="store_true",
        help="无限循环回放"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="显示详细日志"
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="启动 Tkinter 图形界面"
    )

    args = parser.parse_args()

    if args.gui:
        gui_main()
        return

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    config = get_config()
    teleop_cfg = config.arm_teleop

    # 用命令行参数覆盖配置
    if args.slave_addr is not None:
        teleop_cfg.slave_arm_addr = args.slave_addr
    if args.enable:
        teleop_cfg.enabled_by_default = True
    teleop_cfg.torque_off = args.torque_off
    if args.read_rate is not None:
        teleop_cfg.read_rate = args.read_rate
    if args.send_rate is not None:
        teleop_cfg.send_rate = args.send_rate
    if args.deadband is not None:
        teleop_cfg.deadband_deg = args.deadband
    if args.speed_scale is not None:
        teleop_cfg.speed_scale = args.speed_scale

    master_arm_cfg = build_master_arm_config(config.arm)
    app = ArmTeleopApp(teleop_cfg, master_arm_cfg)

    if not app.initialize():
        sys.exit(1)

    # 处理启动模式
    loop_count = 0 if args.loop_forever else args.loop
    if args.record and args.playback:
        logger.error("不能同时指定 --record 和 --playback")
        sys.exit(1)

    try:
        if args.record:
            app._record_file = args.record
            app._start_recording()
        elif args.playback:
            # 延迟到 run() 启动后再回放，避免播放线程因 _running=False 立即退出
            app._pending_playback = (args.playback, args.playback_speed, loop_count)

        app.run()
    except Exception as e:
        logger.exception(f"遥操作应用异常: {e}")
    finally:
        app.stop()


if __name__ == "__main__":
    main()
