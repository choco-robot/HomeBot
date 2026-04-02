"""
麻将牌河图像采集辅助脚本

用于从摄像头采集13张手牌排列成一行（牌河）的训练图像。
从 1920x1080 画面中裁剪出中间 1920x320 的长条形区域。

使用方法:
    cd software
    python tools/capture_helper.py
    
交互控制:
    SPACE   - 保存当前帧
    +/-     - 微调裁剪区域上下位置
    r       - 重置裁剪位置
    p       - 切换是否保存原始图
    c       - 切换是否保存裁剪图
    q/ESC   - 退出
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import cv2
import numpy as np
from common.logging import get_logger
from configs.config import get_config

logger = get_logger(__name__)


class MahjongCaptureHelper:
    """麻将牌河图像采集辅助工具"""
    
    def __init__(
        self,
        output_dir: str = "datasets/mahjong_captured",
        device_id: int = 0,
        resolution: tuple = (1920, 1080),
        crop_y: int = 380,
        crop_height: int = 320,
        prefix: str = "mahjong",
        save_original: bool = True,
        save_cropped: bool = True,
        display_scale: float = 0.5
    ):
        """
        初始化采集助手
        
        Args:
            output_dir: 输出目录
            device_id: 摄像头设备ID
            resolution: 摄像头分辨率 (width, height)
            crop_y: 裁剪起始Y坐标
            crop_height: 裁剪高度
            prefix: 文件名前缀
            save_original: 是否保存原始图
            save_cropped: 是否保存裁剪后的图
            display_scale: 预览缩放比例
        """
        self.output_dir = Path(output_dir)
        self.device_id = device_id
        self.resolution = resolution
        self.crop_y = crop_y
        self.crop_height = crop_height
        self.crop_width = resolution[0]  # 全宽裁剪
        self.prefix = prefix
        self.save_original = save_original
        self.save_cropped = save_cropped
        self.display_scale = display_scale
        
        # 保存计数
        self.save_count = 0
        
        # 默认裁剪位置（用于重置）
        self.default_crop_y = crop_y
        
        # 创建输出目录
        self._init_directories()
        
        # 摄像头
        self.cap = None
        
        logger.info("=" * 50)
        logger.info("麻将牌河图像采集助手")
        logger.info("=" * 50)
        logger.info(f"输出目录: {self.output_dir}")
        logger.info(f"摄像头: 设备{device_id}, 分辨率{resolution}")
        logger.info(f"裁剪区域: y={crop_y}, height={crop_height}")
        logger.info(f"保存设置: 原图={save_original}, 裁剪图={save_cropped}")
        
    def _init_directories(self):
        """初始化输出目录结构"""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        if self.save_original:
            (self.output_dir / "original").mkdir(exist_ok=True)
        if self.save_cropped:
            (self.output_dir / "cropped").mkdir(exist_ok=True)
            
    def init_camera(self) -> bool:
        """初始化摄像头"""
        logger.info(f"初始化摄像头设备 {self.device_id}...")
        
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            logger.error(f"无法打开摄像头设备 {self.device_id}")
            return False
        
        # 设置分辨率
        width, height = self.resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        # 读取一帧测试
        ret, frame = self.cap.read()
        if not ret:
            logger.error("无法读取摄像头画面")
            return False
        
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info(f"✓ 摄像头已就绪: {actual_width}x{actual_height}")
        
        return True
    
    def crop_river_region(self, frame: np.ndarray) -> np.ndarray:
        """
        裁剪牌河区域
        
        Args:
            frame: 原始图像 (1920x1080)
            
        Returns:
            裁剪后的图像 (1920x320)
        """
        y1 = self.crop_y
        y2 = y1 + self.crop_height
        return frame[y1:y2, 0:self.crop_width]
    
    def create_preview(self, frame: np.ndarray) -> np.ndarray:
        """
        创建预览图像
        
        布局：
        [ 原图预览（带裁剪框） ]
        [ 裁剪后的牌河区域     ]
        """
        h, w = frame.shape[:2]
        
        # 裁剪区域
        cropped = self.crop_river_region(frame)
        crop_h, crop_w = cropped.shape[:2]
        
        # 计算统一的显示宽度（以裁剪图宽度为基准，或原图缩放后宽度）
        # 目标：让两张图显示宽度一致
        target_width = int(w * self.display_scale)  # 原图缩放后的宽度
        
        # 缩放原图到目标宽度
        scale = target_width / w
        small = cv2.resize(frame, (target_width, int(h * scale)))
        small_h, small_w = small.shape[:2]
        
        # 缩放裁剪图到相同宽度
        crop_scale = target_width / crop_w
        cropped_resized = cv2.resize(cropped, (target_width, int(crop_h * crop_scale)))
        resized_h = cropped_resized.shape[0]
        
        # 在缩放后的原图上绘制裁剪区域框
        y1_scaled = int(self.crop_y * scale)
        y2_scaled = int((self.crop_y + self.crop_height) * scale)
        cv2.rectangle(small, (0, y1_scaled), (small_w, y2_scaled), (0, 255, 0), 2)
        cv2.line(small, (0, y1_scaled), (small_w, y1_scaled), (0, 255, 255), 1)
        cv2.line(small, (0, y2_scaled), (small_w, y2_scaled), (0, 255, 255), 1)
        
        # 创建组合预览图（上下布局）
        gap = 10  # 中间间隔
        info_panel_height = 120  # 信息面板高度
        preview_height = small_h + gap + resized_h + info_panel_height
        preview_width = target_width
        
        preview = np.zeros((preview_height, preview_width, 3), dtype=np.uint8)
        preview[:] = (40, 40, 40)  # 深灰背景
        
        # 放置原图预览（上半部分）
        preview[:small_h, :small_w] = small
        
        # 添加标签
        cv2.putText(preview, "Original (with crop region)", (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 放置裁剪后的图（中间）
        y_offset = small_h + gap
        preview[y_offset:y_offset+resized_h, :target_width] = cropped_resized
        
        # 添加标签
        cv2.putText(preview, f"Cropped River Region ({crop_w}x{crop_h})", 
                   (10, y_offset + 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        
        # 添加信息面板（底部）
        info_y = y_offset + resized_h + 30
        
        # 左侧信息
        info_texts_left = [
            f"Device: {self.device_id}",
            f"Resolution: {w}x{h}",
            f"Crop: y={self.crop_y}, h={self.crop_height}",
        ]
        for i, text in enumerate(info_texts_left):
            cv2.putText(preview, text, (10, info_y + i * 22),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        
        # 右侧信息
        info_texts_right = [
            f"Saved: {self.save_count}",
            f"Save Orig: {'ON' if self.save_original else 'OFF'}",
            f"Save Crop: {'ON' if self.save_cropped else 'OFF'}",
        ]
        for i, text in enumerate(info_texts_right):
            cv2.putText(preview, text, (preview_width // 2, info_y + i * 22),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        
        # 添加操作提示（最底部）
        hint_text = "SPACE:Save  +/-:AdjustY  r:Reset  p:ToggleOrig  c:ToggleCrop  q:Quit"
        cv2.putText(preview, hint_text, (10, preview_height - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 100), 1)
        
        return preview
    
    def save_frame(self, frame: np.ndarray):
        """保存当前帧"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        
        # 保存原始图
        if self.save_original:
            orig_path = self.output_dir / "original" / f"{self.prefix}_{timestamp}.jpg"
            cv2.imwrite(str(orig_path), frame)
        
        # 保存裁剪图
        if self.save_cropped:
            cropped = self.crop_river_region(frame)
            crop_path = self.output_dir / "cropped" / f"{self.prefix}_{timestamp}.jpg"
            cv2.imwrite(str(crop_path), cropped)
        
        self.save_count += 1
        logger.info(f"✓ 已保存 [{self.save_count}]: {timestamp}")
        
    def adjust_crop(self, delta: int):
        """微调裁剪位置"""
        new_y = self.crop_y + delta
        max_y = self.resolution[1] - self.crop_height
        
        # 限制范围
        new_y = max(0, min(new_y, max_y))
        self.crop_y = new_y
        
        logger.debug(f"裁剪位置调整到: y={self.crop_y}")
        
    def run(self):
        """运行采集主循环"""
        if not self.init_camera():
            return False
        
        logger.info("=" * 50)
        logger.info("采集开始！")
        logger.info("按键控制:")
        logger.info("  SPACE - 保存当前帧")
        logger.info("  +/-   - 微调裁剪区域上下")
        logger.info("  r     - 重置裁剪位置")
        logger.info("  p     - 切换是否保存原始图")
        logger.info("  c     - 切换是否保存裁剪图")
        logger.info("  q/ESC - 退出")
        logger.info("=" * 50)
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    logger.warning("读取摄像头失败")
                    time.sleep(0.1)
                    continue
                
                # 创建预览
                preview = self.create_preview(frame)
                
                # 显示
                cv2.imshow("Mahjong Capture Helper", preview)
                
                # 按键处理
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == 27:  # q 或 ESC
                    break
                elif key == ord(' '):  # SPACE
                    self.save_frame(frame)
                elif key == ord('+') or key == ord('='):  # +
                    self.adjust_crop(-10)
                elif key == ord('-') or key == ord('_'):  # -
                    self.adjust_crop(10)
                elif key == ord('r'):  # r - 重置
                    self.crop_y = self.default_crop_y
                    logger.info(f"裁剪位置重置为: y={self.crop_y}")
                elif key == ord('p'):  # p - 切换保存原始图
                    self.save_original = not self.save_original
                    logger.info(f"保存原始图: {'ON' if self.save_original else 'OFF'}")
                elif key == ord('c'):  # c - 切换保存裁剪图
                    self.save_cropped = not self.save_cropped
                    logger.info(f"保存裁剪图: {'ON' if self.save_cropped else 'OFF'}")
                    
        except KeyboardInterrupt:
            logger.info("用户中断")
        finally:
            self.cleanup()
            
        return True
    
    def cleanup(self):
        """清理资源"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        
        logger.info("=" * 50)
        logger.info(f"采集完成！共保存 {self.save_count} 张图片")
        logger.info(f"输出目录: {self.output_dir}")
        logger.info("=" * 50)


def main():
    parser = argparse.ArgumentParser(
        description="麻将牌河图像采集辅助脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基础使用
  python tools/capture_helper.py
  
  # 指定输出目录
  python tools/capture_helper.py --output datasets/my_mahjong/
  
  # 调整裁剪位置（如果默认380不合适）
  python tools/capture_helper.py --crop-y 400
  
  # 只保存裁剪图
  python tools/capture_helper.py --no-original
  
  # 使用不同摄像头
  python tools/capture_helper.py --device 1
        """
    )
    
    parser.add_argument("--output", "-o", type=str, default="datasets/mahjong_captured",
                        help="输出目录 (默认: datasets/mahjong_captured)")
    parser.add_argument("--device", "-d", type=int, default=0,
                        help="摄像头设备ID (默认: 0)")
    parser.add_argument("--resolution", type=str, default="1920x1080",
                        help="摄像头分辨率 (默认: 1920x1080)")
    parser.add_argument("--crop-y", type=int, default=380,
                        help="裁剪起始Y坐标 (默认: 380)")
    parser.add_argument("--crop-height", type=int, default=320,
                        help="裁剪高度 (默认: 320)")
    parser.add_argument("--prefix", type=str, default="mahjong",
                        help="文件名前缀 (默认: mahjong)")
    parser.add_argument("--no-original", action="store_true",
                        help="不保存原始图")
    parser.add_argument("--no-cropped", action="store_true",
                        help="不保存裁剪图")
    parser.add_argument("--display-scale", type=float, default=0.5,
                        help="预览缩放比例 (默认: 0.5)")
    
    args = parser.parse_args()
    
    # 解析分辨率
    try:
        width, height = map(int, args.resolution.split('x'))
    except ValueError:
        logger.error(f"无效的分辨率格式: {args.resolution}")
        sys.exit(1)
    
    # 创建采集助手
    helper = MahjongCaptureHelper(
        output_dir=args.output,
        device_id=args.device,
        resolution=(width, height),
        crop_y=args.crop_y,
        crop_height=args.crop_height,
        prefix=args.prefix,
        save_original=not args.no_original,
        save_cropped=not args.no_cropped,
        display_scale=args.display_scale
    )
    
    # 运行
    success = helper.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
