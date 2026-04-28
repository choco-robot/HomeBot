"""
机械臂人脸跟踪应用

控制手腕相机（安装在机械臂末端）左右和上下转动，保持人脸在画面正中间。
只控制 base 和 wrist_flex 两个关节。

使用示例:
    python -m applications.face_tracking
    python -m applications.face_tracking --display
"""

from .tracker import FaceTrackerApp

__all__ = ['FaceTrackerApp']
