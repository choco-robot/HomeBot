# -*- coding: utf-8 -*-
"""模仿学习应用入口

用法:
    python -m applications.imitation_learning info [--chassis-type omni3]
    python -m applications.imitation_learning run-policy --policy <路径> [--robot-host 192.168.x.x]
    python -m applications.imitation_learning verify-calibration [--port COM23]

数据采集使用 lerobot 官方 CLI（需安装 lerobot，见 requirements-lerobot.txt）:
    lerobot.record --robot.type=homebot --robot.port=COM23 --teleop.type=so101_leader ...
"""
import argparse
import sys


def main():
    parser = argparse.ArgumentParser(prog="imitation_learning", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_info = sub.add_parser("info", help="打印 robot 的 observation/action features（验证 lerobot 注册）")
    p_info.add_argument("--chassis-type", default="none", choices=["none", "omni3", "diff2"])
    p_info.add_argument("--port", default="COM23")

    p_run = sub.add_parser("run-policy", help="运行训练好的策略（ACT/SmolVLA 推理部署）")
    p_run.add_argument("--policy", required=True, help="策略路径或 HF hub id")
    p_run.add_argument("--robot-host", default="localhost", help="机器人地址")
    p_run.add_argument("--fps", type=float, default=30.0)
    p_run.add_argument("--task", default=None, help="语言指令（VLA 模型需要）")
    p_run.add_argument("--camera-key", default=None, help="图像键名，默认从策略配置推断")
    p_run.add_argument("--enable-chassis", action="store_true", help="启用底盘动作")
    p_run.add_argument("--device", default="cuda")
    p_run.add_argument("--dry-run", action="store_true", help="只打印动作，不下发")

    p_cal = sub.add_parser("verify-calibration", help="验证 LeRobot 与 HomeBot 坐标系读数一致")
    p_cal.add_argument("--port", default="COM23")
    p_cal.add_argument("--baudrate", type=int, default=1_000_000)
    p_cal.add_argument("--tolerance", type=float, default=2.0, help="允许偏差（度）")

    args = parser.parse_args()

    if args.command == "info":
        from .robot import HomeBotRobot, HomeBotRobotConfig, LEROBOT_AVAILABLE

        if not LEROBOT_AVAILABLE:
            print("lerobot 未安装。请先安装: pip install -r requirements-lerobot.txt",
                  file=sys.stderr)
            sys.exit(1)
        config = HomeBotRobotConfig(port=args.port, chassis_type=args.chassis_type)
        robot = HomeBotRobot(config)
        print("observation_features:")
        for k, v in robot.observation_features.items():
            print(f"  {k}: {v}")
        print("action_features:")
        for k, v in robot.action_features.items():
            print(f"  {k}: {v}")

    elif args.command == "run-policy":
        from .policy_runner import run_policy

        run_policy(
            policy_path=args.policy,
            robot_host=args.robot_host,
            fps=args.fps,
            task=args.task,
            camera_key=args.camera_key,
            enable_chassis=args.enable_chassis,
            device=args.device,
            dry_run=args.dry_run,
        )

    elif args.command == "verify-calibration":
        from .verify_calibration import verify_calibration

        ok = verify_calibration(args.port, args.baudrate, args.tolerance)
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
