# -*- coding: utf-8 -*-
"""视觉链路真机性能对比测试

同一进程内依次运行两组配置，对比优化前后的关键指标：
- A 组（优化后）：config 默认 = 采集 1920x1080，发布 640x480，JPEG q70
- B 组（基线）  ：发布 = 采集分辨率 1920x1080，JPEG q95（模拟改造前行为）

测量项：
1. 发布端编码耗时（imencode）与单帧字节数
2. 订阅端实测帧率、丢帧（frame_id 跳号）
3. VisionSubscriber 懒解码：同 frame_id 重复 read_frame 是否复用同一对象
4. 订阅端解码耗时

用法（在 software/ 或项目根目录）：
    venv/Scripts/python.exe software/tools/test_vision_perf.py [--device 0] [--duration 4]
"""
import argparse
import os
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

import cv2
import numpy as np
import zmq


def measure_phase(name: str, device: int, pub_w: int, pub_h: int, quality: int,
                  port: int, duration: float) -> dict:
    """运行一组配置并测量指标"""
    from configs import get_config
    from services.vision_service.vision import VisionService, VisionSubscriber

    cfg = get_config()
    cfg.camera.device_id = device
    cfg.camera.publish_width = pub_w
    cfg.camera.publish_height = pub_h
    cfg.camera.jpeg_quality = quality
    cfg.zmq.vision_pub_addr = f"tcp://*:{port}"

    service = VisionService(config=cfg)
    t = threading.Thread(target=service.start, daemon=True)
    t.start()

    # ---- 1. 原始 SUB 统计：帧率 / 字节数 / 丢帧 ----
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 500)
    sub.connect(f"tcp://localhost:{port}")

    # 摄像头初始化较慢（DSHOW 1080p 约 4s），先等到第一帧再开始计时
    wait_end = time.time() + 15
    got_first = False
    while time.time() < wait_end:
        try:
            sub.recv_multipart()
            got_first = True
            break
        except zmq.Again:
            continue
    if not got_first:
        print(f"[{name}] 等待首帧超时")
        sub.close()
        ctx.term()
        service.stop()
        t.join(timeout=3.0)
        return {"name": name, "fps": 0, "avg_kb": 0, "id_gaps": -1,
                "dims": "N/A", "decode_ms": 0, "lazy_ok": False}

    frame_count = 0
    byte_sizes = []
    last_id = -1
    id_gaps = 0
    sample_jpegs = []
    t_end = time.time() + duration
    while time.time() < t_end:
        try:
            parts = sub.recv_multipart()
        except zmq.Again:
            continue  # 测量窗口内超时不停，继续等到时间结束
        if len(parts) != 2:
            continue
        fid = int(parts[0].decode())
        if last_id >= 0 and fid > last_id + 1:
            id_gaps += fid - last_id - 1
        last_id = fid
        frame_count += 1
        byte_sizes.append(len(parts[1]))
        if len(sample_jpegs) < 10:
            sample_jpegs.append(bytes(parts[1]))
    sub.close()
    ctx.term()

    fps = frame_count / duration if duration > 0 else 0
    avg_kb = statistics.mean(byte_sizes) / 1024 if byte_sizes else 0

    # ---- 2. VisionSubscriber：懒解码 + 解码耗时 + 帧尺寸 ----
    vs = VisionSubscriber(sub_addr=f"tcp://localhost:{port}")
    vs.start()

    # 等到首帧可用（后台接收线程需要时间）
    f1 = None
    fid1 = None
    wait_end = time.time() + 10
    while time.time() < wait_end:
        fid1, f1 = vs.read_frame()
        if f1 is not None:
            break
        time.sleep(0.1)

    fid2, f2 = vs.read_frame()  # 立即再读一次
    lazy_ok = (fid1 == fid2) and (f1 is f2) and (f1 is not None)
    dims = f"{f1.shape[1]}x{f1.shape[0]}" if f1 is not None else "N/A"

    decode_ms = []
    for buf in sample_jpegs:
        t0 = time.perf_counter()
        cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
        decode_ms.append((time.perf_counter() - t0) * 1000)
    vs.stop()

    service.stop()
    t.join(timeout=3.0)

    return {
        "name": name,
        "fps": fps,
        "avg_kb": avg_kb,
        "id_gaps": id_gaps,
        "dims": dims,
        "decode_ms": statistics.mean(decode_ms) if decode_ms else 0,
        "lazy_ok": lazy_ok,
    }


def encode_benchmark(device: int) -> None:
    """采集一帧 1080p 原图，对比两种编码参数的 CPU 耗时与体积"""
    cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    ok, frame = cap.read()
    cap.release()
    if not ok:
        print("[编码基准] 采集失败，跳过")
        return

    small = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)

    def bench(img, q, n=20):
        cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])  # 预热
        t0 = time.perf_counter()
        for _ in range(n):
            _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
        return (time.perf_counter() - t0) / n * 1000, len(buf) / 1024

    ms_a, kb_a = bench(small, 70)
    ms_b, kb_b = bench(frame, 95)
    print(f"[编码基准] 640x480 q70 : {ms_a:.1f} ms/帧, {kb_a:.0f} KB/帧")
    print(f"[编码基准] 1920x1080 q95: {ms_b:.1f} ms/帧, {kb_b:.0f} KB/帧")


def main() -> None:
    parser = argparse.ArgumentParser(description="视觉链路性能对比测试")
    parser.add_argument("--device", type=int, default=0, help="摄像头设备号")
    parser.add_argument("--duration", type=float, default=4.0, help="每组测量时长（秒）")
    args = parser.parse_args()

    encode_benchmark(args.device)
    print("=" * 60)

    results = [
        measure_phase("A 优化后 (640x480 q70)", args.device, 640, 480, 70, 5560, args.duration),
        measure_phase("B 基线   (1920x1080 q95)", args.device, 0, 0, 95, 5562, args.duration),
    ]

    print("=" * 60)
    for r in results:
        print(f"[{r['name']}]")
        print(f"  实测帧率: {r['fps']:.1f} fps | 平均帧大小: {r['avg_kb']:.0f} KB | "
              f"解码尺寸: {r['dims']} | frame_id 跳号: {r['id_gaps']}")
        print(f"  订阅端解码: {r['decode_ms']:.1f} ms/帧 | 懒解码复用: {'OK' if r['lazy_ok'] else 'FAIL'}")

    a, b = results
    if b["avg_kb"] > 0:
        print("=" * 60)
        print(f"带宽下降: {b['avg_kb'] / max(a['avg_kb'], 0.01):.1f}x | "
              f"解码耗时下降: {b['decode_ms'] / max(a['decode_ms'], 0.01):.1f}x")
    lazy_all = all(r["lazy_ok"] for r in results)
    print("结论:", "PASS" if lazy_all and a["fps"] > 5 else "FAIL")


if __name__ == "__main__":
    main()
