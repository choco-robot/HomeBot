"""
HomeBot 麻将机器人启动入口

使用方法:
    cd software/src
    python -m applications.mahjong_bot
    
    # 或指定参数
    python -m applications.mahjong_bot --host 0.0.0.0 --port 5100

访问地址:
    http://<机器人IP>:5100/mahjong
"""
from applications.mahjong_bot.web_server import main

if __name__ == '__main__':
    main()
