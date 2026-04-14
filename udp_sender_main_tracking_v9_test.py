#!/usr/bin/env python3
"""
UDP sender for main_tracking_v9.py.

This script sends test packets to the Windows receiver (default: 127.0.0.1:8888)
using payload shapes that parse_udp_objects() already supports.
"""

import argparse
import json
import math
import random
import socket
import time
from typing import Dict, List, Tuple


IMG_W = 3840
IMG_H = 2160


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def make_box(cx: float, cy: float, w: float, h: float) -> List[float]:
    x1 = clamp(cx - w / 2.0, 0.0, IMG_W - 1.0)
    y1 = clamp(cy - h / 2.0, 0.0, IMG_H - 1.0)
    x2 = clamp(cx + w / 2.0, 0.0, IMG_W - 1.0)
    y2 = clamp(cy + h / 2.0, 0.0, IMG_H - 1.0)
    return [x1, y1, x2, y2]


def moving_targets(seq: int, t: float) -> Tuple[List[float], List[float], float, float]:
    # Target A
    cx1 = 1920 + 700 * math.sin(t * 0.7)
    cy1 = 1080 + 300 * math.sin(t * 1.1)
    w1 = 120 + 20 * math.sin(t * 0.9)
    h1 = 90 + 15 * math.cos(t * 0.6)
    d1 = 250.0 + 40.0 * math.sin(t * 0.8)

    # Target B (for multi-target scheduling)
    cx2 = 1200 + 800 * math.sin(t * 0.5 + 1.1)
    cy2 = 900 + 450 * math.cos(t * 0.65)
    w2 = 100 + 20 * math.cos(t * 0.4)
    h2 = 80 + 10 * math.sin(t * 0.5)
    d2 = 380.0 + 30.0 * math.cos(t * 0.9)

    box1 = make_box(cx1, cy1, w1, h1)
    box2 = make_box(cx2, cy2, w2, h2)

    # Add tiny jitter to emulate detector noise.
    box1 = [v + random.uniform(-1.0, 1.0) for v in box1]
    box2 = [v + random.uniform(-1.0, 1.0) for v in box2]
    return box1, box2, max(1.0, d1), max(1.0, d2)


def build_packet(board: str, cam: int, seq: int, t: float, payload_mode: str, target_count: int) -> Dict:
    box1, box2, d1, d2 = moving_targets(seq, t)
    if payload_mode == "stable":
        if target_count <= 1:
            objs = [box1[0], box1[1], box1[2], box1[3], d1]
        else:
            objs = [
                [box1[0], box1[1], box1[2], box1[3], d1],
                [box2[0], box2[1], box2[2], box2[3], d2],
            ]
    else:
        fmt = seq % 4

        if fmt == 0:
            objs = [box1[0], box1[1], box1[2], box1[3], d1]
        elif fmt == 1:
            objs = [{"box": box1, "distance_m": d1}]
            if target_count > 1:
                objs.append({"box": box2, "distance": d2})
        elif fmt == 2:
            boxes = [box1]
            distances = [d1]
            if target_count > 1:
                boxes.append(box2)
                distances.append(d2)
            objs = {"boxes": boxes, "distances": distances}
        else:
            x1, y1, x2, y2 = box1
            objs = [{"x": x1, "y": y1, "w": (x2 - x1), "h": (y2 - y1), "distance": d1}]
            if target_count > 1:
                x1b, y1b, x2b, y2b = box2
                objs.append({"x": x1b, "y": y1b, "w": (x2b - x1b), "h": (y2b - y1b), "distance": d2})

    return {
        "board": board,
        "cam": cam,
        "objs": objs,
        "seq": seq,
        "ts_sender": time.time(),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="UDP sender for main_tracking_v9.py")
    parser.add_argument("--ip", default="127.0.0.1", help="Receiver IP")
    parser.add_argument("--port", type=int, default=8888, help="Receiver UDP port")
    parser.add_argument("--board", default="BOARD_3", help="Board id string")
    parser.add_argument("--cam", type=int, default=2, help="Camera index")
    parser.add_argument("--fps", type=float, default=20.0, help="Send rate")
    parser.add_argument("--duration", type=float, default=30.0, help="Seconds to run")
    parser.add_argument(
        "--payload-mode",
        choices=("stable", "mixed"),
        default="stable",
        help="stable: steady format for integration; mixed: cycle all supported payload shapes",
    )
    parser.add_argument(
        "--target-count",
        type=int,
        choices=(1, 2),
        default=1,
        help="Number of simulated targets",
    )
    parser.add_argument(
        "--bad-json-every",
        type=int,
        default=0,
        help="Send an invalid JSON packet every N frames (0 disables)",
    )
    parser.add_argument(
        "--bad-utf8-every",
        type=int,
        default=0,
        help="Send invalid UTF-8 bytes every N frames (0 disables)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print packets without sending",
    )
    args = parser.parse_args()

    if args.fps <= 0:
        raise ValueError("fps must be > 0")
    if args.duration <= 0:
        raise ValueError("duration must be > 0")

    interval = 1.0 / args.fps
    total = int(args.duration * args.fps)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(
        f"[Sender] target={args.ip}:{args.port}, board={args.board}, cam={args.cam}, "
        f"fps={args.fps}, duration={args.duration}s, frames={total}, "
        f"payload_mode={args.payload_mode}, target_count={args.target_count}"
    )

    t0 = time.time()
    sent = 0
    for seq in range(total):
        now = time.time()
        t = now - t0

        if args.bad_utf8_every > 0 and seq > 0 and (seq % args.bad_utf8_every == 0):
            raw = b"\xff\xfe\xfd\x00bad_utf8"
            kind = "bad_utf8"
        elif args.bad_json_every > 0 and seq > 0 and (seq % args.bad_json_every == 0):
            raw = b"{\"board\":\"BROKEN\", \"objs\": [1,2,3,4]"  # missing closing brace
            kind = "bad_json"
        else:
            pkt = build_packet(args.board, args.cam, seq, t, args.payload_mode, args.target_count)
            raw = json.dumps(pkt, ensure_ascii=False).encode("utf-8")
            kind = "normal"

        if args.dry_run:
            if kind == "normal":
                print(raw.decode("utf-8"))
            else:
                print(f"<{kind}> {raw!r}")
        else:
            sock.sendto(raw, (args.ip, args.port))

        sent += 1
        if sent % max(1, int(args.fps)) == 0:
            print(f"[Sender] sent={sent}/{total}, kind={kind}")

        next_deadline = t0 + (seq + 1) * interval
        sleep_time = next_deadline - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)

    sock.close()
    print(f"[Sender] done, total_sent={sent}")


if __name__ == "__main__":
    main()
