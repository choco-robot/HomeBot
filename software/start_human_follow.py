#!/usr/bin/env python3
"""
启动人体跟随应用

使用方法:
    cd software
    python start_human_follow.py
    
    # 显示调试窗口
    python start_human_follow.py --display
    
    # 指定配置文件
    python start_human_follow.py --config /path/to/config.yaml
"""
import sys
import os

# 添加src到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from applications.human_follow import main

if __name__ == "__main__":
    main()
