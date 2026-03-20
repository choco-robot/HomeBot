#!/usr/bin/env python3
"""
HomeBot 游戏手柄控制应用启动入口

使用方法:
    cd software/src
    python -m applications.gamepad_control
    
    # 指定手柄索引
    python -m applications.gamepad_control --controller 0
    
    # 详细日志
    python -m applications.gamepad_control --verbose

控制映射:
    底盘控制 (左手):
        - 左摇杆 ↑↓    : 前进 / 后退
        - 左摇杆 ←→    : 左转 / 右转
        - LT (左扳机)  : 向左平移
        - RT (右扳机)  : 向右平移
    
    机械臂控制 (右手):
        - 右摇杆 ←→    : 基座左右转
        - 右摇杆 ↑↓    : 前伸 / 后缩
        - 十字键 ↑↓    : 上升 / 下降
        - 十字键 ←→    : 手腕旋转
        - Y键          : 手腕上翻
        - A键          : 手腕下翻
        - B键          : 手腕一键水平 (180°-shoulder-elbow)
        - RB键         : 夹爪打开
        - LB键         : 夹爪关闭
    
    系统控制:
        - Back键       : 紧急停止
        - Start键      : 复位
"""
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from .app import GamepadControlApp
from configs import get_config
from common.logging import get_logger

logger = get_logger(__name__)


def main():
    """主入口"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="HomeBot 游戏手柄控制应用",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
控制说明:
  底盘: 左摇杆(移动/旋转) + LT/RT(平移)
  机械臂: 右摇杆(基座/伸缩) + 十字键(升降/腕转) + Y/A/B(腕翻) + RB/LB(夹爪)
  系统: Back(急停) / Start(复位)
        """
    )
    
    parser.add_argument(
        "--controller", "-c",
        type=int,
        default=0,
        help="手柄索引 (0-3)，默认 0"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="显示详细日志"
    )
    
    args = parser.parse_args()
    
    # 设置日志级别
    if args.verbose:
        import logging
        logging.getLogger().setLevel(logging.DEBUG)
    
    # 创建并运行应用
    config = get_config()
    app = GamepadControlApp(config=config.gamepad, controller_index=args.controller)
    
    try:
        app.run()
    except KeyboardInterrupt:
        logger.info("用户中断")
    finally:
        app.stop()


if __name__ == "__main__":
    main()
