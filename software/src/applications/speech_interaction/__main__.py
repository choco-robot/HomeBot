"""语音交互应用入口

使用方法:
    cd software/src
    python -m applications.speech_interaction
"""
import asyncio
from applications.speech_interaction.speech_app import main

if __name__ == "__main__":
    asyncio.run(main())
