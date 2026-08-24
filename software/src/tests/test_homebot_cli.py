# -*- coding: utf-8 -*-
"""HomeBot CLI 测试

用 click.testing.CliRunner 做离线测试：
1. 各命令帮助/解析正常
2. status / doctor 输出不崩
3. topic pub → 线程内 broker → topic echo 往返

运行方式:
    cd software/src
    python -m tests.test_homebot_cli
"""
import json
import socket
import threading
import time

import zmq
from click.testing import CliRunner

from homebot_cli.main import cli


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _start_broker():
    """线程内启动 XPUB-XSUB broker，返回 (xsub_addr, xpub_addr, cleanup)"""
    xsub_addr = f"tcp://127.0.0.1:{_free_port()}"
    xpub_addr = f"tcp://127.0.0.1:{_free_port()}"

    ctx = zmq.Context()
    xsub = ctx.socket(zmq.XSUB)
    xpub = ctx.socket(zmq.XPUB)
    xsub.bind(xsub_addr)
    xpub.bind(xpub_addr)

    def _run_proxy():
        try:
            zmq.proxy(xsub, xpub)
        except Exception:
            pass  # 测试结束时 socket/context 被回收，proxy 退出属正常

    threading.Thread(target=_run_proxy, daemon=True).start()

    def cleanup():
        xsub.close()
        xpub.close()
        ctx.term()

    return xsub_addr, xpub_addr, cleanup


def test_help_and_parse():
    """主命令和子命令的 --help 都能正常输出"""
    runner = CliRunner()
    for args in ([], ["--help"], ["start", "--help"], ["stop", "--help"],
                 ["status", "--help"], ["topic", "--help"], ["topic", "echo", "--help"],
                 ["topic", "pub", "--help"], ["move", "--help"], ["doctor", "--help"],
                 ["completion", "--help"]):
        result = runner.invoke(cli, args)
        assert result.exit_code == 0, f"{args} 失败: {result.output}"
    print("[OK] 命令解析与帮助输出正常")


def test_unknown_service():
    """非法服务名应报错"""
    runner = CliRunner()
    result = runner.invoke(cli, ["start", "nonexistent"])
    assert result.exit_code != 0, "非法服务名应返回非零退出码"
    assert "nonexistent" in result.output
    print("[OK] 非法服务名正确报错")


def test_status_and_doctor():
    """status / doctor 在无服务运行环境下也能正常输出"""
    runner = CliRunner()
    result = runner.invoke(cli, ["status"])
    assert result.exit_code == 0, result.output
    for name in ("bus", "motion", "vision", "web"):
        assert name in result.output, f"status 输出缺少服务 {name}"

    result = runner.invoke(cli, ["doctor"])
    assert result.exit_code == 0, result.output
    assert "Python" in result.output
    print("[OK] status / doctor 输出正常")


def test_topic_pub_bad_json():
    """topic pub 收到非法 JSON 应报错"""
    runner = CliRunner()
    result = runner.invoke(cli, ["topic", "pub", "user.x", "not-json"])
    assert result.exit_code != 0, "非法 JSON 应返回非零退出码"
    print("[OK] topic pub 非法 JSON 正确报错")


def test_topic_pub_echo_roundtrip():
    """BusPublisher 发布 → topic echo 命令订阅，经 broker 往返

    注意: 发布端直接用 BusPublisher，不用 CliRunner —— CliRunner 通过替换
    sys.stdout 捕获输出，两个并发 invoke 会互相污染捕获缓冲。
    """
    from common.bus import BusPublisher

    xsub_addr, xpub_addr, cleanup = _start_broker()
    runner = CliRunner()

    try:
        # 后台线程延迟发布一条消息（等待 echo 的订阅关系传播到 broker）
        def publish_later():
            time.sleep(1.0)
            pub = BusPublisher(addr=xsub_addr)
            time.sleep(0.3)
            pub.publish("user.cli_test", {"value": 42}, timestamp=time.time())
            time.sleep(0.3)
            pub.close()

        pub_thread = threading.Thread(target=publish_later, daemon=True)
        pub_thread.start()

        result = runner.invoke(cli, [
            "topic", "echo", "user.cli_test", "--count", "1",
            "--addr", xpub_addr,
        ])
        assert result.exit_code == 0, result.output
        # 从输出中提取 JSON 消息行
        msg_lines = [l for l in result.output.splitlines() if l.startswith("{")]
        assert len(msg_lines) == 1, f"应收到 1 条消息，输出: {result.output}"
        payload = json.loads(msg_lines[0])
        assert payload["type"] == "user.cli_test", payload
        assert payload["data"]["value"] == 42, payload

        pub_thread.join(timeout=5)
        print("[OK] topic echo 订阅接收正常")
    finally:
        cleanup()


def main():
    print("=" * 60)
    print("HomeBot CLI 测试")
    print("=" * 60)

    test_help_and_parse()
    test_unknown_service()
    test_status_and_doctor()
    test_topic_pub_bad_json()
    test_topic_pub_echo_roundtrip()

    print("=" * 60)
    print("全部测试通过")
    print("=" * 60)


if __name__ == "__main__":
    main()
