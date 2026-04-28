"""Camera HAL module - 相机硬件抽象层.

此模块提供相机相关的接口:
- CameraDriver: 底层相机驱动
- CameraPublisher: 独立图像发布器 (特殊场景使用)
- CameraSubscriber: 图像订阅客户端 (订阅 VisionService)

推荐使用:
- VisionService (services.vision_service.vision) 作为图像采集和发布中心
- VisionSubscriber (services.vision_service.vision) 作为图像订阅客户端
"""

from hal.camera.driver import CameraDriver
from hal.camera.webcamera_driver import WebCameraDriver

__all__ = [
    'CameraDriver',
    'WebCameraDriver',
    'CameraPublisher', 
    'CameraSubscriber',
    'VisionClient',
]
