"""Gimbal-camera visual lock independent of upstream UDP cadence."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


def should_run_vision_only_tick(
    *,
    has_udp_packets: bool,
    master_track_id,
    vision_frame_ts: float,
    last_control_frame_ts: float,
) -> bool:
    """Return True when fresh gimbal YOLO must drive control without UDP."""
    return (
        not bool(has_udp_packets)
        and master_track_id is not None
        and float(vision_frame_ts or 0.0)
        > float(last_control_frame_ts or 0.0) + 1e-9
    )


@dataclass(frozen=True)
class GimbalVisualLockConfig:
    image_width: float = 2560.0
    image_height: float = 1440.0
    aim_x_px: float = 1280.0
    aim_y_px: float = 720.0
    min_x_ratio: float = 0.05
    max_x_ratio: float = 0.95
    min_y_ratio: float = 0.05
    max_y_ratio: float = 0.95
    lost_timeout_s: float = 1.0
    outside_confirm_frames: int = 3


class GimbalVisualTargetLock:
    """Keep one YOLO simple ID locked until it is lost or leaves the area."""

    def __init__(self, config: GimbalVisualLockConfig):
        if config.image_width <= 0.0 or config.image_height <= 0.0:
            raise ValueError("image dimensions must be positive")
        if not (
            0.0 <= config.min_x_ratio < config.max_x_ratio <= 1.0
            and 0.0 <= config.min_y_ratio < config.max_y_ratio <= 1.0
        ):
            raise ValueError("visual lock ratios must form valid image ranges")
        if config.lost_timeout_s < 0.0:
            raise ValueError("lost_timeout_s must not be negative")
        if config.outside_confirm_frames < 1:
            raise ValueError("outside_confirm_frames must be at least 1")
        self.config = config
        self.sort_track_id: Optional[int] = None
        self.simple_id: Optional[int] = None
        self.started_ts = 0.0
        self.last_seen_ts = 0.0
        self.last_frame_ts = 0.0
        self.outside_frames = 0

    @property
    def active(self) -> bool:
        return self.sort_track_id is not None

    def holds_sort_track(self, track_id) -> bool:
        return (
            self.active
            and track_id is not None
            and int(track_id) == self.sort_track_id
        )

    def start(self, sort_track_id: int, now_ts: float) -> None:
        self.sort_track_id = int(sort_track_id)
        self.simple_id = None
        self.started_ts = float(now_ts)
        self.last_seen_ts = 0.0
        self.last_frame_ts = 0.0
        self.outside_frames = 0

    def release(self) -> None:
        self.sort_track_id = None
        self.simple_id = None
        self.started_ts = 0.0
        self.last_seen_ts = 0.0
        self.last_frame_ts = 0.0
        self.outside_frames = 0

    @staticmethod
    def _center(measurement: dict) -> Optional[tuple[float, float]]:
        center = measurement.get("center")
        if center is None or len(center) != 2:
            return None
        try:
            x, y = float(center[0]), float(center[1])
        except (TypeError, ValueError):
            return None
        return (x, y) if math.isfinite(x) and math.isfinite(y) else None

    def _inside_area(self, measurement: dict) -> bool:
        center = self._center(measurement)
        if center is None:
            return False
        x, y = center
        return (
            self.config.min_x_ratio * self.config.image_width
            <= x
            <= self.config.max_x_ratio * self.config.image_width
            and self.config.min_y_ratio * self.config.image_height
            <= y
            <= self.config.max_y_ratio * self.config.image_height
        )

    def _nearest_inside(self, measurements: list[dict]) -> Optional[dict]:
        candidates = []
        for measurement in measurements:
            center = self._center(measurement)
            simple_id = measurement.get("simple_id")
            if center is None or simple_id is None or not self._inside_area(measurement):
                continue
            distance = math.hypot(
                center[0] - self.config.aim_x_px,
                center[1] - self.config.aim_y_px,
            )
            candidates.append((distance, -float(measurement.get("confidence", 0.0)), measurement))
        return min(candidates, key=lambda item: (item[0], item[1]))[2] if candidates else None

    def update(
        self,
        *,
        frame_ts: float,
        measurements: list[dict],
        now_ts: float,
    ) -> dict:
        """Consume one new YOLO frame and return the retained measurement."""
        if not self.active:
            return {"state": "IDLE", "measurement": None, "release": False}
        frame_ts = float(frame_ts or 0.0)
        now_ts = float(now_ts)
        if frame_ts <= 0.0 or frame_ts <= self.last_frame_ts + 1e-9:
            return {
                "state": "NO_NEW_FRAME",
                "measurement": None,
                "release": False,
            }
        self.last_frame_ts = frame_ts
        measurements = list(measurements or [])

        selected = None
        if self.simple_id is None:
            selected = self._nearest_inside(measurements)
            if selected is not None:
                self.simple_id = int(selected["simple_id"])
        else:
            selected = next(
                (
                    measurement
                    for measurement in measurements
                    if measurement.get("simple_id") is not None
                    and int(measurement["simple_id"]) == self.simple_id
                ),
                None,
            )

        if selected is not None and self._inside_area(selected):
            self.last_seen_ts = frame_ts
            self.outside_frames = 0
            return {
                "state": "LOCKED",
                "measurement": selected,
                "release": False,
                "simple_id": self.simple_id,
            }

        if selected is not None:
            self.outside_frames += 1
            release = self.outside_frames >= self.config.outside_confirm_frames
            return {
                "state": "OUTSIDE_RELEASE" if release else "OUTSIDE_CONFIRMING",
                "measurement": None,
                "release": release,
                "reason": "gimbal_bbox_outside_lock_area",
                "simple_id": self.simple_id,
                "outside_frames": self.outside_frames,
            }

        reference_ts = self.last_seen_ts if self.last_seen_ts > 0.0 else self.started_ts
        lost_age = max(0.0, now_ts - reference_ts)
        release = lost_age >= self.config.lost_timeout_s
        return {
            "state": "MISSING_RELEASE" if release else "MISSING_HOLD",
            "measurement": None,
            "release": release,
            "reason": "gimbal_yolo_target_missing",
            "simple_id": self.simple_id,
            "lost_age_s": lost_age,
        }
