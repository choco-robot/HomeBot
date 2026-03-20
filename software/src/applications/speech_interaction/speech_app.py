"""语音交互应用 - SUB模式

订阅 WakeupASR 服务的识别结果，进行对话管理并播放TTS

使用方式:
    python -m applications.speech_interaction
"""
import asyncio
import signal
import sys
import time
from typing import Optional

import zmq
from common.logging import get_logger
from common.zmq_helper import create_socket
from configs.config import get_config
from applications.speech_interaction.dialogue_manager import DialogueManager
from services.speech_service.voice_engine import VoiceEngine

logger = get_logger(__name__)


class SpeechInteractionApp:
    """语音交互应用"""
    
    def __init__(self):
        """初始化应用"""
        config = get_config()
        
        # ZeroMQ 上下文
        self.context = zmq.Context()
        
        # SUB socket：订阅 WakeupASR 服务
        self.sub_socket = create_socket(
            zmq.SUB,
            bind=False,
            address=config.zmq.wakeup_pub_addr.replace("*", "localhost"),
            context=self.context
        )
        # 订阅所有消息
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        # 设置接收超时（用于优雅退出）
        self.sub_socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms
        
        # 对话管理器
        self.dialogue_manager = DialogueManager()
        
        # TTS 客户端（本地直接调用）
        self.tts_engine = VoiceEngine()
        
        self.is_running = False
        self.processed_sessions = set()  # 防止重复处理
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """处理系统信号"""
        logger.info(f"收到信号 {signum}，正在停止应用...")
        self.stop()
        sys.exit(0)
    
    async def run(self):
        """主循环 - 订阅并处理语音事件"""
        logger.info("=" * 50)
        logger.info("语音交互应用启动")
        logger.info(f"订阅地址: {self.sub_socket.get_string(zmq.LAST_ENDPOINT)}")
        logger.info("等待语音输入...")
        logger.info("=" * 50)
        
        self.is_running = True
        
        while self.is_running:
            try:
                # 非阻塞接收事件
                try:
                    event = self.sub_socket.recv_json()
                except zmq.Again:
                    # 超时，继续循环
                    await asyncio.sleep(0.01)
                    continue
                
                # 处理事件
                await self._handle_event(event)
                
            except Exception as e:
                logger.error(f"主循环异常: {e}", exc_info=True)
                await asyncio.sleep(1)
    
    async def _handle_event(self, event: dict):
        """处理语音事件
        
        Args:
            event: 事件字典
        """
        event_type = event.get("event")
        
        if event_type == "speech_detected":
            await self._handle_speech_event(event)
        elif event_type == "heartbeat":
            logger.debug(f"收到心跳: {event}")
        else:
            logger.warning(f"未知事件类型: {event_type}")
    
    async def _handle_speech_event(self, event: dict):
        """处理语音识别事件
        
        Args:
            event: speech_detected 事件
        """
        session_id = event.get("session_id", "")
        asr_text = event.get("asr_text", "")
        keyword = event.get("keyword", "")
        
        logger.info("-" * 50)
        logger.info(f"检测到唤醒词: {keyword}")
        logger.info(f"ASR识别: {asr_text}")
        
        # 防重复处理
        if session_id in self.processed_sessions:
            logger.debug(f"会话 {session_id} 已处理，跳过")
            return
        self.processed_sessions.add(session_id)
        
        # 清理旧会话记录
        if len(self.processed_sessions) > 100:
            self.processed_sessions = set(list(self.processed_sessions)[-50:])
        
        # 对话处理
        try:
            async for reply, context in self.dialogue_manager.process_query(asr_text):
                if reply:
                    logger.info(f"AI回复: {reply}")
                    
                    # TTS 播放
                    await self._speak(reply)
        except Exception as e:
            logger.error(f"对话处理失败: {e}")
            await self._speak("抱歉，处理出错了，请再试一次")
        
        logger.info("-" * 50)
        logger.info("等待下一次唤醒...")
    
    async def _speak(self, text: str):
        """语音合成并播放
        
        Args:
            text: 要合成的文本
        """
        try:
            await self.tts_engine.synthesize(text)
        except Exception as e:
            logger.error(f"TTS失败: {e}")
    
    def stop(self):
        """停止应用"""
        logger.info("正在停止语音交互应用...")
        self.is_running = False
        
        try:
            # 释放TTS资源
            if self.tts_engine:
                self.tts_engine.release()
            
            # 关闭 socket
            if self.sub_socket:
                self.sub_socket.close()
            
            # 终止上下文
            if self.context:
                self.context.term()
            
            logger.info("语音交互应用已停止")
        except Exception as e:
            logger.error(f"停止应用时出错: {e}")


async def main():
    """应用入口"""
    app = SpeechInteractionApp()
    
    try:
        await app.run()
    except KeyboardInterrupt:
        logger.info("收到键盘中断")
    finally:
        app.stop()


if __name__ == "__main__":
    asyncio.run(main())
