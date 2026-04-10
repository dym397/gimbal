#!/usr/bin/env python3
"""
UDP sender for main_tracking_v9.py tracking evaluation.

Focused on two validation scenarios:
1. static: target stays at a fixed image position
2. linear: target moves at constant pixel velocity

Examples
--------
Static target at image center for 8 seconds:
    python udp_sender_tracking_scenarios.py --mode static --ip 192.168.2.88 --duration 8

Static target with an offset:
    python udp_sender_tracking_scenarios.py --mode static --cx 2520 --cy 1080 --duration 8

Linear motion, constant speed to the right:
    python udp_sender_tracking_scenarios.py --mode linear --cx 1400 --cy 1080 --vx 120 --duration 10

Linear motion, constant speed to the left and slightly upward:
    python udp_sender_tracking_scenarios.py --mode linear --cx 2600 --cy 1200 --vx -90 --vy -25 --duration 12
"""

import argparse
import json
import random
import socket
import time
from typing import Dict, List, Tuple


IMG_W = 3840.0
IMG_H = 2160.0

PRESET_POINTS = {
    "center": (1920.0, 1080.0),
    "left": (1320.0, 1080.0),
    "right": (2520.0, 1080.0),
    "up": (1920.0, 760.0),
    "down": (1920.0, 1400.0),
    "left_up": (1320.0, 760.0),
    "right_up": (2520.0, 760.0),
    "left_down": (1320.0, 1400.0),
    "right_down": (2520.0, 1400.0),
}


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def round2(v: float) -> float:
    return round(float(v), 2)


def make_box(cx: float, cy: float, w: float, h: float) -> List[float]:
    x1 = clamp(cx - w / 2.0, 0.0, IMG_W - 1.0)
    y1 = clamp(cy - h / 2.0, 0.0, IMG_H - 1.0)
    x2 = clamp(cx + w / 2.0, 0.0, IMG_W - 1.0)
    y2 = clamp(cy + h / 2.0, 0.0, IMG_H - 1.0)
    return [round2(x1), round2(y1), round2(x2), round2(y2)]


def resolve_center(args: argparse.Namespace) -> Tuple[float, float]:
    if args.cx is not None and args.cy is not None:
        return float(args.cx), float(args.cy)
    if args.cx is not None or args.cy is not None:
        raise ValueError("cx and cy must be provided together")
    return PRESET_POINTS[args.preset]


def validate_path(cx0: float, cy0: float, args: argparse.Namespace) -> None:
    half_w = args.box_w / 2.0
    half_h = args.box_h / 2.0

    points = [(cx0, cy0)]
    if args.mode == "linear":
        end_t = max(args.duration - (1.0 / args.fps), 0.0)
        points.append((cx0 + args.vx * end_t, cy0 + args.vy * end_t))

    for idx, (cx, cy) in enumerate(points):
        if not (half_w <= cx <= (IMG_W - 1.0 - half_w)):
            stage = "start" if idx == 0 else "end"
            raise ValueError(
                f"{stage} center x={cx:.2f} makes box leave image; "
                f"valid range is [{half_w:.2f}, {IMG_W - 1.0 - half_w:.2f}]"
            )
        if not (half_h <= cy <= (IMG_H - 1.0 - half_h)):
            stage = "start" if idx == 0 else "end"
            raise ValueError(
                f"{stage} center y={cy:.2f} makes box leave image; "
                f"valid range is [{half_h:.2f}, {IMG_H - 1.0 - half_h:.2f}]"
            )


def build_packet(
    board: str,
    cam: int,
    seq: int,
    box: List[float],
    distance_m: float,
    mode: str,
    cx: float,
    cy: float,
    elapsed: float,
) -> Dict:
    return {
        "board": board,
        "cam": cam,
        "objs": [box[0], box[1], box[2], box[3], distance_m],
        "seq": seq,
        "mode": mode,
        "center": [round2(cx), round2(cy)],
        "ts_sender": time.time(),
        "elapsed_sender": round2(elapsed),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="UDP sender for main_tracking_v9.py scenario tests")
    parser.add_argument("--mode", choices=("static", "linear"), required=True, help="Tracking scenario")
    parser.add_argument("--ip", default="127.0.0.1", help="Receiver IP")
    parser.add_argument("--port", type=int, default=8888, help="Receiver UDP port")
    parser.add_argument("--board", default="BOARD_3", help="Board id string")
    parser.add_argument("--cam", type=int, default=2, help="Camera index")
    parser.add_argument("--fps", type=float, default=15.0, help="Send rate in Hz")
    parser.add_argument("--duration", type=float, default=8.0, help="Scenario duration in seconds")
    parser.add_argument(
        "--preset",
        choices=tuple(PRESET_POINTS.keys()),
        default="center",
        help="Start position preset when cx/cy are not given",
    )
    parser.add_argument("--cx", type=float, help="Initial box center x in pixels")
    parser.add_argument("--cy", type=float, help="Initial box center y in pixels")
    parser.add_argument("--vx", type=float, default=80.0, help="Linear mode x velocity in px/s")
    parser.add_argument("--vy", type=float, default=0.0, help="Linear mode y velocity in px/s")
    parser.add_argument("--box-w", type=float, default=140.0, help="Bounding box width in pixels")
    parser.add_argument("--box-h", type=float, default=100.0, help="Bounding box height in pixels")
    parser.add_argument("--distance-m", type=float, default=320.0, help="Mono distance sent in packet")
    parser.add_argument(
        "--jitter",
        type=float,
        default=0.0,
        help="Uniform center jitter amplitude in pixels; 0 disables detector noise",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print packets without sending")
    args = parser.parse_args()

    if args.fps <= 0:
        raise ValueError("fps must be > 0")
    if args.duration <= 0:
        raise ValueError("duration must be > 0")
    if args.box_w <= 0 or args.box_h <= 0:
        raise ValueError("box-w and box-h must be > 0")
    if args.distance_m <= 0:
        raise ValueError("distance-m must be > 0")
    if args.jitter < 0:
        raise ValueError("jitter must be >= 0")

    cx0, cy0 = resolve_center(args)
    validate_path(cx0, cy0, args)

    total_frames = max(1, int(round(args.duration * args.fps)))
    interval = 1.0 / args.fps
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(
        f"[Sender] target={args.ip}:{args.port}, mode={args.mode}, board={args.board}, cam={args.cam}, "
        f"fps={args.fps}, duration={args.duration}s, frames={total_frames}"
    )
    print(
        f"[Sender] start_center=({cx0:.2f}, {cy0:.2f}), box=({args.box_w:.1f}x{args.box_h:.1f}), "
        f"distance={args.distance_m:.1f}m, jitter={args.jitter:.1f}px"
    )
    if args.mode == "linear":
        end_t = max(args.duration - interval, 0.0)
        end_cx = cx0 + args.vx * end_t
        end_cy = cy0 + args.vy * end_t
        print(
            f"[Sender] velocity=({args.vx:.2f}, {args.vy:.2f}) px/s, "
            f"end_center=({end_cx:.2f}, {end_cy:.2f})"
        )

    t0 = time.time()
    sent = 0

    try:
        for seq in range(total_frames):
            elapsed = seq * interval
            cx = cx0
            cy = cy0

            if args.mode == "linear":
                cx = cx0 + args.vx * elapsed
                cy = cy0 + args.vy * elapsed

            if args.jitter > 0:
                cx += random.uniform(-args.jitter, args.jitter)
                cy += random.uniform(-args.jitter, args.jitter)

            box = make_box(cx, cy, args.box_w, args.box_h)
            packet = build_packet(
                board=args.board,
                cam=args.cam,
                seq=seq,
                box=box,
                distance_m=args.distance_m,
                mode=args.mode,
                cx=cx,
                cy=cy,
                elapsed=elapsed,
            )
            raw = json.dumps(packet, ensure_ascii=False).encode("utf-8")

            if args.dry_run:
                print(raw.decode("utf-8"))
            else:
                sock.sendto(raw, (args.ip, args.port))

            sent += 1
            if sent == 1 or sent % max(1, int(round(args.fps))) == 0 or sent == total_frames:
                print(
                    f"[Sender] sent={sent}/{total_frames}, elapsed={elapsed:.2f}s, "
                    f"center=({cx:.2f}, {cy:.2f}), box={box}"
                )

            next_deadline = t0 + (seq + 1) * interval
            sleep_time = next_deadline - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
    finally:
        sock.close()

    print(f"[Sender] done, total_sent={sent}")


if __name__ == "__main__":
    main()
