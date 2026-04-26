# -*- coding: utf-8 -*-
"""全局导航应用启动入口

Usage:
    python -m navigation.applications.navigation
    python -m navigation.applications.navigation --goal-x 2.0 --goal-y 1.5
    python -m navigation.applications.navigation --inflation 0.25 --lookahead 0.5
"""
from .app import main

if __name__ == "__main__":
    main()
