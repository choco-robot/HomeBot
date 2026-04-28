"""
Vision Service 人体检测测试工具

从 Vision Service 订阅图像流，使用 YOLO 检测人体并可视化显示。

Usage:
    cd software/src
    python -m tests.test_vision_detector
    python -m tests.test_vision_detector --addr tcp://localhost:5560
    python -m tests.test_vision_detector --model models/yolo26n.pt --conf 0.5
"""
import argparse
import time
import cv2
import numpy as np

from common.logging import get_logger
from services.vision_service import VisionSubscriber
from applications.human_follow.detector import HumanDetector
from configs.config import get_config

logger = get_logger(__name__)


def main():
    parser = argparse.ArgumentParser(description='Vision Service 人体检测测试')
    parser.add_argument('--addr', default=None, help='Vision Service 订阅地址 (默认: tcp://localhost:5560)')
    parser.add_argument('--model', default=None, help='YOLO 模型路径')
    parser.add_argument('--conf', type=float, default=None, help='检测置信度阈值')
    parser.add_argument('--device', default='cpu', help='推理设备 (cpu/cuda/mps)')
    parser.add_argument('--mode', default='person', choices=['person', 'face'],
                       help='检测模式: person=人体, face=人脸')
    parser.add_argument('--no-display', action='store_true', help='不显示窗口，只打印检测信息')
    args = parser.parse_args()
    
    # 获取配置
    config = get_config()
    
    # 订阅地址
    sub_addr = args.addr or config.human_follow.vision_sub_addr
    
    # 模型参数
    model_path = args.model
    conf_threshold = args.conf if args.conf is not None else config.human_follow.conf_threshold
    
    logger.info("=" * 50)
    logger.info(f"Vision Service {'人体' if args.mode == 'person' else '人脸'}检测测试工具")
    logger.info("=" * 50)
    logger.info(f"订阅地址: {sub_addr}")
    logger.info(f"模型路径: {model_path or '默认'}")
    logger.info(f"置信度阈值: {conf_threshold}")
    logger.info(f"推理设备: {args.device}")
    logger.info(f"检测模式: {args.mode}")
    
    # 初始化检测器
    detector = HumanDetector(
        model_path=model_path,
        conf_threshold=conf_threshold,
        device=args.device,
        detect_mode=args.mode,
    )
    
    if not detector.initialize():
        logger.error("检测器初始化失败，请检查模型文件")
        return
    
    logger.info(f"模型信息: {detector.get_model_info()}")
    
    # 初始化订阅者
    subscriber = VisionSubscriber(sub_addr)
    subscriber.start()
    logger.info("VisionSubscriber 已启动，等待图像...")
    
    # 统计信息
    frame_count = 0
    detect_count = 0
    fps_time = time.time()
    
    try:
        while True:
            # 读取最新帧
            frame_id, frame = subscriber.read_frame()
            
            if frame is None:
                time.sleep(0.01)
                continue
            
            frame_count += 1
            
            # 人体检测
            output, detections = detector.detect_and_draw(frame)
            detect_count += len(detections)
            
            # 计算 FPS
            now = time.time()
            elapsed = now - fps_time
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                logger.info(f"FPS: {fps:.1f}, 帧数: {frame_count}, 本帧检测到: {len(detections)} 人")
                frame_count = 0
                detect_count = 0
                fps_time = now
            
            # 显示窗口
            if not args.no_display:
                # 添加辅助信息
                h, w = output.shape[:2]
                info = f"Frame: {frame_id} | People: {len(detections)} | Press 'q' to quit"
                cv2.putText(output, info, (10, h - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                window_title = f"Vision Service - {args.mode.upper()} Detection"
                cv2.imshow(window_title, output)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    logger.info("用户按 'q' 退出")
                    break
    
    except KeyboardInterrupt:
        logger.info("用户中断 (Ctrl+C)")
    finally:
        subscriber.stop()
        detector.release()
        cv2.destroyAllWindows()
        logger.info("测试工具已停止")


if __name__ == "__main__":
    main()
