"""Vision Service - 视觉服务模块.

提供图像采集、处理和发布功能.其他应用通过订阅此服务获取视频流.

使用示例:
    # 启动视觉服务 (采集并发布图像)
    from services.vision_service import VisionService
    service = VisionService()
    service.start()
    
    # 在另一个应用中订阅图像
    from services.vision_service import VisionSubscriber
    subscriber = VisionSubscriber()
    frame_id, frame = subscriber.read_frame()
"""

from services.vision_service.vision import VisionService, VisionSubscriber, WebCameraVisionService

__all__ = [
    'VisionService',
    'VisionSubscriber',
    'WebCameraVisionService',
]
