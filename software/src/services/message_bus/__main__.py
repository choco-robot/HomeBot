# -*- coding: utf-8 -*-
"""消息总线服务入口

运行方式:
    cd software/src
    python -m services.message_bus
    python -m services.message_bus --xsub-addr tcp://*:5590 --xpub-addr tcp://*:5591
"""
import argparse

from configs import get_config
from common.logging import get_logger
from services.message_bus.service import run_broker

logger = get_logger(__name__)


def main():
    config = get_config()

    parser = argparse.ArgumentParser(description="HomeBot 通用消息总线服务 (XPUB-XSUB broker)")
    parser.add_argument("--xsub-addr", default=config.zmq.bus_xsub_addr,
                        help="XSUB 绑定地址（发布者 connect 到此），默认 %(default)s")
    parser.add_argument("--xpub-addr", default=config.zmq.bus_xpub_addr,
                        help="XPUB 绑定地址（订阅者 connect 到此），默认 %(default)s")
    args = parser.parse_args()

    logger.info("Starting message bus broker...")
    run_broker(args.xsub_addr, args.xpub_addr)


if __name__ == "__main__":
    main()
