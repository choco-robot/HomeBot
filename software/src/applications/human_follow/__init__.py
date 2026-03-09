"""
人体跟随应用模块

使用YOLO26进行人体检测，实现自动跟随功能

使用方法:
    # 方式一：命令行运行
    cd software/src
    python -m applications.human_follow
    
    # 方式二：代码调用
    from applications.human_follow import HumanFollowApp
    
    app = HumanFollowApp()
    app.run(display=True)
"""

from .detector import HumanDetector, Detection
from .tracker import TargetTracker, Target, TargetStatus
from .controller import FollowController, VelocityCommand
from .follow import HumanFollowApp, FollowMode, FollowStatus, main

__all__ = [
    'HumanDetector',
    'Detection',
    'TargetTracker',
    'Target',
    'TargetStatus',
    'FollowController',
    'VelocityCommand',
    'HumanFollowApp',
    'FollowMode',
    'FollowStatus',
    'main',
]

__version__ = '1.0.0'
