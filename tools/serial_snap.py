#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""串口日志快照：自动识别日志口（排除蓝牙口，嗅探 [TB]/[BOOT] 特征），
抓取指定时长的带时间戳输出。给无人值守调试闭环用。

用法: python tools/serial_snap.py [--port COM14] [--duration 5] [--baud 115200]
"""
import argparse
import sys
import time

import serial
from serial.tools import list_ports


def candidate_ports():
    """所有非蓝牙串口"""
    out = []
    for p in list_ports.comports():
        d = (p.description or "")
        if ("蓝牙" in d) or ("Bluetooth" in d):
            continue
        out.append(p.device)
    return out


def sniff(port, baud, dur):
    """开一段串口收 dur 秒，返回收到的字节串（打不开返回 None）"""
    try:
        s = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"[snap] {port} 打不开: {e}", file=sys.stderr)
        return None
    t0 = time.time()
    buf = b""
    while time.time() - t0 < dur:
        buf += s.read(512)
    s.close()
    return buf


def auto_detect(baud):
    for port in candidate_ports():
        buf = sniff(port, baud, 1.5)
        if buf and (b"[TB]" in buf or b"[BOOT]" in buf or b"RM_Begin" in buf):
            print(f"[snap] 自动识别日志口 = {port}", file=sys.stderr)
            return port
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="串口号；缺省自动嗅探")
    ap.add_argument("--duration", type=float, default=5.0, help="抓取秒数")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    port = args.port or auto_detect(args.baud)
    if not port:
        print("[snap] 未识别到日志口（板子在跑吗？特征串没出现）", file=sys.stderr)
        return 1

    s = serial.Serial(port, args.baud, timeout=0.1)
    t0 = time.monotonic()
    tail = b""
    while time.monotonic() - t0 < args.duration:
        chunk = s.read(512)
        if not chunk:
            continue
        tail += chunk
        while b"\n" in tail:
            line, tail = tail.split(b"\n", 1)
            print(f"[{time.monotonic()-t0:7.3f}] {line.decode(errors='replace').rstrip()}")
    if tail:
        print(f"[{time.monotonic()-t0:7.3f}] {tail.decode(errors='replace').rstrip()}")
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
