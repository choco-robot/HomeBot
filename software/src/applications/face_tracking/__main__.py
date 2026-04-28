"""
机械臂人脸跟踪应用入口

控制手腕相机左右和上下转动，保持人脸在画面正中间。
只控制 base 和 wrist_flex 两个关节。

启动方式:
    cd software/src
    ..\..\venv\Scripts\python.exe -m applications.face_tracking
    
    # 带调试窗口显示
    ..\..\venv\Scripts\python.exe -m applications.face_tracking --display
    
    # 调整灵敏度
    ..\..\venv\Scripts\python.exe -m applications.face_tracking --kp-base 0.1 --kp-wrist 0.05 --max-step 3.0
"""

from .tracker import main

if __name__ == "__main__":
    main()
