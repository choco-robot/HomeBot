# -*- coding: utf-8 -*-
"""共享串口总线并发压力测试（真机）

场景：motion_service 以 both 模式运行（底盘+机械臂共用一条舵机总线）时，
- 线程A：以 20Hz 向底盘服务发送速度指令（原地自转，结束后自动停止）
- 线程B：以 10Hz 向机械臂服务发送状态查询（走 sync_read 批量读路径）
- 线程C：订阅电池状态 PUB（5555）

统计各链路的往返时延、超时/失败次数，用于验证总线互斥与批量读改造。

用法：
    python tools/test_shared_bus_stress.py [--duration 6] [--spin 0.3]
"""
import argparse
import json
import statistics
import threading
import time

import zmq


def chassis_worker(duration: float, spin: float, addr: str, results: dict) -> None:
    """20Hz 底盘速度写，结束时发送零速停止"""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, 2000)
    sock.setsockopt(zmq.SNDTIMEO, 2000)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(addr)

    rtts = []
    fails = 0
    end = time.time() + duration
    while time.time() < end:
        cmd = {"source": "stress_test", "vx": 0.0, "vy": 0.0, "vz": spin, "priority": 1}
        t0 = time.perf_counter()
        try:
            sock.send_json(cmd)
            resp = sock.recv_json()
            rtts.append((time.perf_counter() - t0) * 1000)
            if not resp.get("success"):
                fails += 1
        except zmq.Again:
            fails += 1
        elapsed = time.perf_counter() - t0
        if elapsed < 0.05:
            time.sleep(0.05 - elapsed)

    # 发送零速停止
    try:
        sock.send_json({"source": "stress_test", "vx": 0.0, "vy": 0.0, "vz": 0.0, "priority": 1})
        sock.recv_json()
    except zmq.Again:
        fails += 1

    results["chassis"] = {"count": len(rtts), "fails": fails, "rtts": rtts}
    sock.close()
    ctx.term()


def arm_worker(duration: float, addr: str, results: dict) -> None:
    """10Hz 机械臂状态查询（sync_read 批量读路径）"""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, 2000)
    sock.setsockopt(zmq.SNDTIMEO, 2000)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(addr)

    rtts = []
    fails = 0
    none_joints = 0
    end = time.time() + duration
    while time.time() < end:
        t0 = time.perf_counter()
        try:
            sock.send_json({"query": True, "source": "stress_test", "priority": 1})
            resp = sock.recv_json()
            rtts.append((time.perf_counter() - t0) * 1000)
            if not resp.get("success"):
                fails += 1
            states = resp.get("joint_states") or {}
            none_joints += sum(1 for v in states.values() if v is None)
            results.setdefault("last_states", states)
        except zmq.Again:
            fails += 1
        elapsed = time.perf_counter() - t0
        if elapsed < 0.1:
            time.sleep(0.1 - elapsed)

    results["arm"] = {"count": len(rtts), "fails": fails, "none_joints": none_joints, "rtts": rtts}
    sock.close()
    ctx.term()


def battery_listener(duration: float, addr: str, results: dict) -> None:
    """订阅电池状态 PUB"""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVTIMEO, 1000)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.connect(addr)

    msgs = []
    end = time.time() + duration
    while time.time() < end:
        try:
            data = sock.recv_json()
            msgs.append(data)
        except zmq.Again:
            continue
        except Exception:
            # 可能是 multipart，尝试原始接收
            try:
                raw = sock.recv()
                msgs.append(raw.decode(errors="replace"))
            except Exception:
                continue

    results["battery"] = msgs
    sock.close()
    ctx.term()


def report(name: str, r: dict) -> None:
    rtts = r["rtts"]
    if rtts:
        print(f"[{name}] 请求 {r['count']} 次, 失败 {r['fails']} 次, "
              f"RTT avg={statistics.mean(rtts):.1f}ms "
              f"max={max(rtts):.1f}ms min={min(rtts):.1f}ms")
    else:
        print(f"[{name}] 无成功请求, 失败 {r['fails']} 次")


def main() -> None:
    parser = argparse.ArgumentParser(description="共享总线并发压力测试")
    parser.add_argument("--duration", type=float, default=6.0, help="压测时长（秒）")
    parser.add_argument("--spin", type=float, default=0.3, help="底盘自转角速度 (rad/s)")
    parser.add_argument("--chassis-addr", default="tcp://localhost:5556")
    parser.add_argument("--arm-addr", default="tcp://localhost:5557")
    parser.add_argument("--battery-addr", default="tcp://localhost:5555")
    args = parser.parse_args()

    results: dict = {}
    threads = [
        threading.Thread(target=chassis_worker, args=(args.duration, args.spin, args.chassis_addr, results)),
        threading.Thread(target=arm_worker, args=(args.duration, args.arm_addr, results)),
        threading.Thread(target=battery_listener, args=(args.duration + 2, args.battery_addr, results)),
    ]
    print(f"开始压测：底盘 {args.spin} rad/s 自转 + 机械臂 10Hz 查询，持续 {args.duration}s")
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    print("=" * 60)
    if "chassis" in results:
        report("底盘", results["chassis"])
    if "arm" in results:
        report("机械臂", results["arm"])
        print(f"[机械臂] 查询返回 None 的关节读数累计: {results['arm']['none_joints']}")
        print(f"[机械臂] 最后一次关节状态: {json.dumps(results.get('last_states', {}))}")
    bat = results.get("battery", [])
    print(f"[电池] 收到 {len(bat)} 条消息: {bat[:3]}")

    ok = (results.get("chassis", {}).get("fails", 1) == 0
          and results.get("arm", {}).get("fails", 1) == 0
          and results.get("arm", {}).get("none_joints", 1) == 0)
    print("=" * 60)
    print("结论:", "PASS - 并发读写无失败" if ok else "FAIL - 存在失败，请检查日志")


if __name__ == "__main__":
    main()
