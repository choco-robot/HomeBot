# -*- coding: utf-8 -*-
"""DepthService 启动入口

Usage:
    python -m navigation.services
    python -m navigation.services --vision tcp://127.0.0.1:5560
"""
from .depth_service import main

if __name__ == "__main__":
    main()
