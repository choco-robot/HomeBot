"""
麻将检测实时测试脚本

用于测试训练好的麻将检测模型

使用方法:
    cd software
    python tools/training/test_detection.py --model models/mahjong_yolo.pt --source 0
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

import cv2
import numpy as np
from applications.mahjong_bot.detector import MahjongDetector, TILE_CLASSES
from common.logging import get_logger

logger = get_logger(__name__)


def test_camera(
    model_path: str,
    source: str = "0",
    conf_threshold: float = 0.5,
    inference_size: int = 640,
    device: str = "cpu",
    save_video: str = None
):
    """
    使用摄像头测试检测模型
    
    Args:
        model_path: 模型文件路径
        source: 视频源 (0=默认摄像头, 或视频文件路径)
        conf_threshold: 置信度阈值
        inference_size: 推理尺寸
        device: 计算设备
        save_video: 保存视频路径 (可选)
    """
    # 初始化检测器
    logger.info("初始化检测器...")
    detector = MahjongDetector(
        model_path=model_path,
        conf_threshold=conf_threshold,
        inference_size=inference_size,
        device=device
    )
    
    if not detector.initialize():
        logger.error("检测器初始化失败")
        return False
    
    # 打开视频源
    try:
        source_int = int(source)
        source = source_int
    except ValueError:
        pass
    
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        logger.error(f"无法打开视频源: {source}")
        return False
    
    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    # 获取实际分辨率
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    
    logger.info(f"视频源: {source}")
    logger.info(f"分辨率: {width}x{height}, FPS: {fps}")
    
    # 视频写入器
    writer = None
    if save_video:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(save_video, fourcc, fps, (width, height))
        logger.info(f"保存视频到: {save_video}")
    
    # 性能统计
    frame_count = 0
    total_inference_time = 0
    
    logger.info("开始检测，按 'q' 退出，按 's' 保存截图")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            logger.warning("无法读取帧")
            break
        
        # 检测
        import time
        start_time = time.time()
        
        output, detections = detector.detect_and_draw(frame)
        
        inference_time = (time.time() - start_time) * 1000  # ms
        total_inference_time += inference_time
        frame_count += 1
        
        # 显示 FPS
        avg_fps = 1000 / (total_inference_time / frame_count) if frame_count > 0 else 0
        cv2.putText(output, f"FPS: {avg_fps:.1f}", (width - 150, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 显示检测到的牌
        if detections:
            tiles_text = ", ".join([d.class_name for d in detections[:10]])
            if len(detections) > 10:
                tiles_text += f" ... (+{len(detections) - 10})"
            cv2.putText(output, f"Tiles: {tiles_text}", (10, height - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # 显示结果
        cv2.imshow("Mahjong Detection Test", output)
        
        # 保存视频
        if writer:
            writer.write(output)
        
        # 按键处理
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            screenshot_path = f"mahjong_screenshot_{frame_count:04d}.jpg"
            cv2.imwrite(screenshot_path, output)
            logger.info(f"截图保存: {screenshot_path}")
    
    # 清理
    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()
    detector.release()
    
    # 输出统计
    if frame_count > 0:
        avg_inference_time = total_inference_time / frame_count
        logger.info("=" * 50)
        logger.info("测试统计:")
        logger.info(f"  总帧数: {frame_count}")
        logger.info(f"  平均推理时间: {avg_inference_time:.2f} ms")
        logger.info(f"  平均 FPS: {1000/avg_inference_time:.1f}")
        logger.info("=" * 50)
    
    return True


def test_image(
    model_path: str,
    image_path: str,
    conf_threshold: float = 0.5,
    inference_size: int = 640,
    device: str = "cpu",
    output_path: str = None
):
    """
    测试单张图片
    
    Args:
        model_path: 模型文件路径
        image_path: 图片路径
        conf_threshold: 置信度阈值
        inference_size: 推理尺寸
        device: 计算设备
        output_path: 输出图片路径 (可选)
    """
    # 初始化检测器
    logger.info("初始化检测器...")
    detector = MahjongDetector(
        model_path=model_path,
        conf_threshold=conf_threshold,
        inference_size=inference_size,
        device=device
    )
    
    if not detector.initialize():
        logger.error("检测器初始化失败")
        return False
    
    # 读取图片
    frame = cv2.imread(image_path)
    if frame is None:
        logger.error(f"无法读取图片: {image_path}")
        return False
    
    logger.info(f"图片尺寸: {frame.shape[1]}x{frame.shape[0]}")
    
    # 检测
    import time
    start_time = time.time()
    
    output, detections = detector.detect_and_draw(frame)
    
    inference_time = (time.time() - start_time) * 1000  # ms
    
    # 输出结果
    logger.info("=" * 50)
    logger.info(f"检测到 {len(detections)} 张牌:")
    for tile in detections:
        logger.info(f"  {tile.class_name}: {tile.confidence:.2f} at ({tile.center[0]}, {tile.center[1]})")
    logger.info(f"推理时间: {inference_time:.2f} ms")
    logger.info("=" * 50)
    
    # 保存结果
    if output_path:
        cv2.imwrite(output_path, output)
        logger.info(f"结果保存到: {output_path}")
    
    # 显示结果
    cv2.imshow("Mahjong Detection", output)
    logger.info("按任意键关闭窗口")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    detector.release()
    return True


def benchmark_model(
    model_path: str,
    image_path: str = None,
    num_runs: int = 100,
    inference_size: int = 640,
    device: str = "cpu"
):
    """
    模型性能基准测试
    
    Args:
        model_path: 模型文件路径
        image_path: 测试图片路径 (默认使用随机图像)
        num_runs: 运行次数
        inference_size: 推理尺寸
        device: 计算设备
    """
    import time
    
    # 初始化检测器
    logger.info("初始化检测器...")
    detector = MahjongDetector(
        model_path=model_path,
        inference_size=inference_size,
        device=device
    )
    
    if not detector.initialize():
        logger.error("检测器初始化失败")
        return False
    
    # 准备测试图像
    if image_path:
        frame = cv2.imread(image_path)
        if frame is None:
            logger.error(f"无法读取图片: {image_path}")
            return False
    else:
        # 使用随机图像
        frame = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)
    
    logger.info(f"测试图像尺寸: {frame.shape[1]}x{frame.shape[0]}")
    logger.info(f"推理尺寸: {inference_size}x{inference_size}")
    logger.info(f"运行次数: {num_runs}")
    
    # 预热
    logger.info("预热中...")
    for _ in range(10):
        detector.detect(frame)
    
    # 正式测试
    logger.info("开始测试...")
    times = []
    
    for i in range(num_runs):
        start = time.time()
        detector.detect(frame)
        elapsed = (time.time() - start) * 1000
        times.append(elapsed)
    
    # 统计
    times = np.array(times)
    logger.info("=" * 50)
    logger.info("基准测试结果:")
    logger.info(f"  平均推理时间: {times.mean():.2f} ms ({1000/times.mean():.1f} FPS)")
    logger.info(f"  中位数: {np.median(times):.2f} ms")
    logger.info(f"  最小值: {times.min():.2f} ms")
    logger.info(f"  最大值: {times.max():.2f} ms")
    logger.info(f"  标准差: {times.std():.2f} ms")
    logger.info("=" * 50)
    
    detector.release()
    return True


def main():
    parser = argparse.ArgumentParser(
        description="麻将检测实时测试脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 测试摄像头
  python tools/training/test_detection.py --model models/mahjong_yolo.pt --source 0
  
  # 测试视频文件
  python tools/training/test_detection.py --model models/mahjong_yolo.pt --source video.mp4
  
  # 测试单张图片
  python tools/training/test_detection.py --model models/mahjong_yolo.pt --image test.jpg
  
  # 性能基准测试
  python tools/training/test_detection.py --model models/mahjong_yolo.pt --benchmark --runs 200
        """
    )
    
    parser.add_argument("--model", "-m", type=str, default="models/mahjong_yolo.pt",
                        help="模型文件路径 (默认: models/mahjong_yolo.pt)")
    parser.add_argument("--source", "-s", type=str, default="0",
                        help="视频源 (默认: 0，即默认摄像头)")
    parser.add_argument("--image", "-i", type=str, default=None,
                        help="测试图片路径")
    parser.add_argument("--conf", "-c", type=float, default=0.5,
                        help="置信度阈值 (默认: 0.5)")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="推理尺寸 (默认: 640)")
    parser.add_argument("--device", "-d", type=str, default="cpu",
                        help="计算设备 (默认: cpu，可选: cuda/mps)")
    parser.add_argument("--save", type=str, default=None,
                        help="保存视频路径 (仅摄像头/视频模式)")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="输出图片路径 (仅图片模式)")
    parser.add_argument("--benchmark", "-b", action="store_true",
                        help="性能基准测试模式")
    parser.add_argument("--runs", "-r", type=int, default=100,
                        help="基准测试运行次数 (默认: 100)")
    
    args = parser.parse_args()
    
    # 检查模型文件
    model_path = Path(args.model)
    if not model_path.exists():
        logger.error(f"模型文件不存在: {args.model}")
        logger.info("请训练模型或检查路径")
        sys.exit(1)
    
    # 基准测试模式
    if args.benchmark:
        success = benchmark_model(
            model_path=args.model,
            image_path=args.image,
            num_runs=args.runs,
            inference_size=args.imgsz,
            device=args.device
        )
        sys.exit(0 if success else 1)
    
    # 图片测试模式
    if args.image:
        success = test_image(
            model_path=args.model,
            image_path=args.image,
            conf_threshold=args.conf,
            inference_size=args.imgsz,
            device=args.device,
            output_path=args.output
        )
        sys.exit(0 if success else 1)
    
    # 摄像头/视频测试模式
    success = test_camera(
        model_path=args.model,
        source=args.source,
        conf_threshold=args.conf,
        inference_size=args.imgsz,
        device=args.device,
        save_video=args.save
    )
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
