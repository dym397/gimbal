#!/usr/bin/env python3
"""
Simple UDP sender for main_tracking_v9.py.

Send a few fixed targets in sequence so the Windows side can verify
basic gimbal response with real hardware.
"""

import argparse
import json
import socket
import time
from typing import Dict, List, Tuple


IMG_W = 3840.0
IMG_H = 2160.0


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def make_box(cx: float, cy: float, w: float = 140.0, h: float = 100.0) -> List[float]:
    x1 = clamp(cx - w / 2.0, 0.0, IMG_W - 1.0)
    y1 = clamp(cy - h / 2.0, 0.0, IMG_H - 1.0)
    x2 = clamp(cx + w / 2.0, 0.0, IMG_W - 1.0)
    y2 = clamp(cy + h / 2.0, 0.0, IMG_H - 1.0)
    return [round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)]


def build_presets() -> List[Tuple[str, List[float], float]]:
    return [
        ("center", make_box(1920.0, 1080.0), 320.0),
        ("right", make_box(2520.0, 1080.0), 320.0),
        ("left", make_box(1320.0, 1080.0), 320.0),
        ("up", make_box(1920.0, 760.0), 320.0),
        ("down", make_box(1920.0, 1400.0), 320.0),
    ]


def build_packet(board: str, cam: int, seq: int, box: List[float], distance_m: float, label: str) -> Dict:
    return {
        "board": board,
        "cam": cam,
        "objs": [box[0], box[1], box[2], box[3], distance_m],
        "seq": seq,
        "label": label,
        "ts_sender": time.time(),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Simple UDP sender for main_tracking_v9.py")
    parser.add_argument("--ip", default="127.0.0.1", help="Receiver IP, e.g. Windows host IP")
    parser.add_argument("--port", type=int, default=8888, help="Receiver UDP port")
    parser.add_argument("--board", default="BOARD_3", help="Board id string")
    parser.add_argument("--cam", type=int, default=2, help="Camera index")
    parser.add_argument("--fps", type=float, default=10.0, help="Send rate for each target hold")
    parser.add_argument("--hold", type=float, default=2.0, help="Seconds to hold each target")
    parser.add_argument("--repeat", type=int, default=1, help="How many times to replay the full preset sequence")
    parser.add_argument("--dry-run", action="store_true", help="Print packets without sending")
    args = parser.parse_args()

    if args.fps <= 0:
        raise ValueError("fps must be > 0")
    if args.hold <= 0:
        raise ValueError("hold must be > 0")
    if args.repeat <= 0:
        raise ValueError("repeat must be > 0")

    presets = build_presets()
    frames_per_target = max(1, int(round(args.fps * args.hold)))
    interval = 1.0 / args.fps
    total_frames = frames_per_target * len(presets) * args.repeat
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(
        f"[Sender] target={args.ip}:{args.port}, board={args.board}, cam={args.cam}, "
        f"fps={args.fps}, hold={args.hold}s, repeat={args.repeat}, total_frames={total_frames}"
    )
    print("[Sender] preset order: " + ", ".join(name for name, _, _ in presets))

    seq = 0
    for round_idx in range(args.repeat):
        for label, box, distance_m in presets:
            print(f"[Sender] round={round_idx + 1}/{args.repeat}, target={label}, box={box}, dist={distance_m}m")
            for _ in range(frames_per_target):
                pkt = build_packet(args.board, args.cam, seq, box, distance_m, label)
                raw = json.dumps(pkt, ensure_ascii=False).encode("utf-8")
                if args.dry_run:
                    print(raw.decode("utf-8"))
                else:
                    sock.sendto(raw, (args.ip, args.port))
                seq += 1
                time.sleep(interval)

    sock.close()
    print(f"[Sender] done, total_sent={seq}")


if __name__ == "__main__":
    main()
