"""WakeupASR 服务 - PUB模式

持续监听麦克风，检测唤醒词后自动进行ASR识别
识别结果通过 ZeroMQ PUB 发布

发布的事件:
- speech_detected: 唤醒并识别成功
  {
      "event": "speech_detected",
      "keyword": "你好小白",
      "asr_text": "向前走一米",
      "timestamp": 1710662847.123
  }
- heartbeat: 心跳（可选）
"""
import zmq
import time
import signal
import sys
from pathlib import Path
from typing import Optional

from common.logging import get_logger
from common.zmq_helper import create_socket
from services.speech_service.voice_engine import VoiceEngine

logger = get_logger(__name__)


class WakeupASRService:
    """唤醒+ASR 服务"""
    
    def __init__(self, pub_addr: str = "tcp://*:5571"):
        """初始化服务
        
        Args:
            pub_addr: PUB socket 绑定地址
        """
        self.voice = VoiceEngine()
        
        # ZeroMQ 上下文和 socket
        self.context = zmq.Context()
        self.pub_socket = create_socket(
            zmq.PUB,
            bind=True,
            address=pub_addr,
            context=self.context
        )
        
        # 等待 SUB 连接（PUB 的慢启动问题）
        logger.info("等待 PUB socket 准备...")
        time.sleep(0.5)
        
        self.is_running = False
        self.session_count = 0
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """处理系统信号"""
        logger.info(f"收到信号 {signum}，正在停止服务...")
        self.stop()
        sys.exit(0)
    
    def run(self):
        """主循环 - 持续监听并发布识别结果"""
        logger.info("=" * 50)
        logger.info("WakeupASR 服务启动")
        logger.info(f"发布地址: {self.pub_socket.LAST_ENDPOINT.decode() if hasattr(self.pub_socket, 'LAST_ENDPOINT') else 'tcp://*:5571'}")
        logger.info(f"唤醒词: {self.voice.wakeup_keyword}")
        logger.info("=" * 50)
        
        self.is_running = True
        
        while self.is_running:
            try:
                # 阶段1: 等待唤醒
                logger.info("等待唤醒...")
                if not self._wait_for_wakeup():
                    continue
                # 播放提示音
                self._play_wakeup_sound()
                
                # 阶段2: ASR 识别
                logger.info("唤醒成功，开始ASR识别...")
                asr_text = self._recognize_speech()
                
                if asr_text:
                    # 发布识别结果
                    self._publish_speech_event(asr_text)
                else:
                    logger.warning("ASR未识别到有效内容")
                    
            except Exception as e:
                logger.error(f"主循环异常: {e}", exc_info=True)
                time.sleep(1)
    
    def _wait_for_wakeup(self) -> bool:
        """等待唤醒词检测
        
        Returns:
            bool: 是否检测到唤醒词
        """
        try:
            while self.is_running:
                if self.voice.wakeup():
                    return True
                # 降低CPU占用
                time.sleep(0.01)
            return False
        except Exception as e:
            logger.error(f"唤醒检测异常: {e}")
            return False
    
    def _recognize_speech(self) -> Optional[str]:
        """进行语音识别
        
        Returns:
            str: 识别结果文本，失败返回 None
        """
        try:            
            # ASR 识别
            text = self.voice.recognize()
            return text if text else None
            
        except Exception as e:
            logger.error(f"ASR识别异常: {e}")
            return None
    
    def _play_wakeup_sound(self):
        """播放唤醒提示音"""
        try:
            # 尝试播放提示音，失败不影响主流程
            base_path = Path(__file__).parent.parent.parent.parent
            pcm_file = base_path / "cache" / "wozai.pcm"
            if pcm_file.exists():
                self.voice.play_pcm_file('wozai')
        except Exception as e:
            logger.debug(f"播放提示音失败: {e}")
    
    def _publish_speech_event(self, asr_text: str):
        """发布语音检测事件
        
        Args:
            asr_text: ASR识别结果
        """
        self.session_count += 1
        
        event = {
            "event": "speech_detected",
            "type": "wakeup_asr",
            "session_id": f"session_{self.session_count}",
            "keyword": self.voice.wakeup_keyword,
            "asr_text": asr_text,
            "timestamp": time.time()
        }
        
        try:
            self.pub_socket.send_json(event)
            logger.info(f"已发布识别结果: {asr_text}")
        except Exception as e:
            logger.error(f"发布事件失败: {e}")
    
    def stop(self):
        """停止服务"""
        logger.info("正在停止 WakeupASR 服务...")
        self.is_running = False
        
        # 释放资源
        try:
            if self.voice:
                self.voice.release()
            
            if self.pub_socket:
                self.pub_socket.close()
            
            if self.context:
                self.context.term()
                
            logger.info("WakeupASR 服务已停止")
        except Exception as e:
            logger.error(f"停止服务时出错: {e}")


def main():
    """服务入口"""
    from configs.config import get_config
    
    config = get_config()
    pub_addr = config.zmq.wakeup_pub_addr
    
    service = WakeupASRService(pub_addr=pub_addr)
    
    try:
        service.run()
    except KeyboardInterrupt:
        logger.info("收到键盘中断")
    finally:
        service.stop()


if __name__ == "__main__":
    main()
