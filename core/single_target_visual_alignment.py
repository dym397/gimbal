"""Conservative single-target YOLO-to-gimbal visual alignment gate.

This module contains no camera, tracker, or serial I/O.  In the single-target
branch the sole YOLO box in the gimbal image is treated as the current master;
this module only decides when its centers are stable enough to justify one
gimbal correction.  The caller remains the sole owner of the command queue.
"""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class SingleTargetAlignmentConfig:
    image_width: float = 2560.0
    image_height: float = 1440.0
    fov_x_deg: float = 17.5
    fov_y_deg: float = 9.9
    stable_frames: int = 6
    max_center_std_x_px: float = 12.0
    max_center_std_y_px: float = 12.0
    max_center_speed_x_px_s: float = 15.0
    max_center_speed_y_px_s: float = 15.0
    trigger_x_px: float = 50.0
    trigger_y_px: float = 50.0
    centered_x_px: float = 30.0
    centered_y_px: float = 30.0
    max_frame_gap_s: float = 1.0
    max_step_az_deg: float = 3.0
    max_step_el_deg: float = 2.0
    max_corrections_per_lock: int = 3


class SingleTargetVisualAlignment:
    """Accumulate stable single-target centers and request one correction."""

    def __init__(self, config: SingleTargetAlignmentConfig):
        if config.stable_frames < 2:
            raise ValueError("stable_frames must be at least 2")
        if config.max_corrections_per_lock < 1:
            raise ValueError("max_corrections_per_lock must be at least 1")
        if config.image_width <= 0.0 or config.image_height <= 0.0:
            raise ValueError("image dimensions must be positive")
        if (
            config.centered_x_px < 0.0
            or config.centered_y_px < 0.0
            or config.trigger_x_px < config.centered_x_px
            or config.trigger_y_px < config.centered_y_px
        ):
            raise ValueError(
                "trigger thresholds must be no smaller than centered thresholds"
            )
        self.config = config
        self._samples = deque(maxlen=config.stable_frames)
        self._identity = None
        self._last_frame_ts = 0.0
        self._waiting_after_command = False
        self._command_stationary_ts = 0.0
        self._correction_identity = None
        self._correction_count = 0
        self._laser_ready = False
        self._laser_ready_since_ts = 0.0

    def _clear_laser_ready(self) -> None:
        self._laser_ready = False
        self._laser_ready_since_ts = 0.0

    def reset_samples(self) -> None:
        self._samples.clear()
        self._identity = None
        self._last_frame_ts = 0.0

    def mark_command_sent(self, stationary_ts: float) -> None:
        self._waiting_after_command = True
        self._command_stationary_ts = float(stationary_ts)
        self._correction_count += 1
        self._clear_laser_ready()
        self.reset_samples()

    @staticmethod
    def _finite_pair(center) -> Optional[tuple[float, float]]:
        if center is None or len(center) != 2:
            return None
        try:
            x = float(center[0])
            y = float(center[1])
        except (TypeError, ValueError):
            return None
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        return x, y

    @staticmethod
    def _linear_speed(samples, axis: int) -> float:
        times = [sample[0] for sample in samples]
        values = [sample[axis] for sample in samples]
        mean_t = statistics.fmean(times)
        mean_v = statistics.fmean(values)
        denominator = sum((value - mean_t) ** 2 for value in times)
        if denominator <= 1e-9:
            return math.inf
        return sum(
            (sample_t - mean_t) * (sample_v - mean_v)
            for sample_t, sample_v in zip(times, values)
        ) / denominator

    def _result(self, state: str, **values) -> dict:
        result = {
            "state": state,
            "processed_new_frame": False,
            "command_requested": False,
            "stable": False,
            "sample_count": len(self._samples),
            "control_x": math.nan,
            "control_y": math.nan,
            "dx_px": math.nan,
            "dy_px": math.nan,
            "std_x_px": math.nan,
            "std_y_px": math.nan,
            "speed_x_px_s": math.nan,
            "speed_y_px_s": math.nan,
            "delta_az_deg": 0.0,
            "delta_el_deg": 0.0,
            "correction_count": self._correction_count,
            "laser_ready": self._laser_ready,
            "laser_ready_since_ts": self._laser_ready_since_ts,
        }
        result.update(values)
        return result

    def update(
        self,
        *,
        frame_ts: float,
        master_track_id,
        measurement: Optional[dict],
        detection_count: int,
        gimbal_stationary: bool,
        stationary_ts: float,
    ) -> dict:
        if not gimbal_stationary:
            self._clear_laser_ready()
            self.reset_samples()
            return self._result("GIMBAL_MOVING")

        if self._waiting_after_command:
            if float(stationary_ts) <= self._command_stationary_ts + 1e-6:
                return self._result("WAIT_NEW_STATIONARY_EVENT")
            self._waiting_after_command = False
            self.reset_samples()

        if master_track_id is None:
            self._clear_laser_ready()
            self.reset_samples()
            return self._result("NO_LOCKED_TARGET")
        if int(detection_count) != 1:
            self._clear_laser_ready()
            self.reset_samples()
            return self._result("REQUIRE_EXACTLY_ONE_DETECTION")
        if not measurement:
            self._clear_laser_ready()
            self.reset_samples()
            return self._result("NO_SINGLE_YOLO_TARGET")

        center = self._finite_pair(measurement.get("center"))
        simple_id = measurement.get("simple_id")
        if center is None or simple_id is None:
            self._clear_laser_ready()
            self.reset_samples()
            return self._result("INVALID_YOLO_DETECTION")

        frame_ts = float(frame_ts)
        if not math.isfinite(frame_ts) or frame_ts <= 0.0:
            return self._result("INVALID_FRAME_TIMESTAMP")
        if frame_ts <= self._last_frame_ts + 1e-9:
            return self._result("DUPLICATE_FRAME")

        identity = (int(master_track_id), int(simple_id))
        if identity != self._correction_identity:
            self._correction_identity = identity
            self._correction_count = 0
            self._clear_laser_ready()
        if identity != self._identity:
            self.reset_samples()
            self._identity = identity
        elif (
            self._last_frame_ts > 0.0
            and frame_ts - self._last_frame_ts > self.config.max_frame_gap_s
        ):
            self._samples.clear()

        self._last_frame_ts = frame_ts
        self._samples.append((frame_ts, center[0], center[1]))
        if len(self._samples) < self.config.stable_frames:
            self._clear_laser_ready()
            return self._result(
                "ACCUMULATING",
                processed_new_frame=True,
                sample_count=len(self._samples),
            )

        samples = list(self._samples)
        x_values = [sample[1] for sample in samples]
        y_values = [sample[2] for sample in samples]
        std_x = float(statistics.pstdev(x_values))
        std_y = float(statistics.pstdev(y_values))
        speed_x = float(self._linear_speed(samples, 1))
        speed_y = float(self._linear_speed(samples, 2))
        # The six-frame window only proves that the target is stationary.
        # Once confirmed, use the newest (sixth) bbox center immediately.
        control_x, control_y = center
        dx = control_x - self.config.image_width / 2.0
        dy = control_y - self.config.image_height / 2.0
        stable = (
            std_x <= self.config.max_center_std_x_px
            and std_y <= self.config.max_center_std_y_px
            and abs(speed_x) <= self.config.max_center_speed_x_px_s
            and abs(speed_y) <= self.config.max_center_speed_y_px_s
        )
        common = {
            "processed_new_frame": True,
            "stable": stable,
            "sample_count": len(samples),
            "control_x": control_x,
            "control_y": control_y,
            "dx_px": dx,
            "dy_px": dy,
            "std_x_px": std_x,
            "std_y_px": std_y,
            "speed_x_px_s": speed_x,
            "speed_y_px_s": speed_y,
        }
        if not stable:
            self._clear_laser_ready()
            return self._result("TARGET_NOT_STABLE", **common)

        centered_x = abs(dx) <= self.config.centered_x_px
        centered_y = abs(dy) <= self.config.centered_y_px
        if centered_x and centered_y:
            self._correction_count = 0
            if not self._laser_ready:
                self._laser_ready = True
                self._laser_ready_since_ts = frame_ts
            return self._result("CENTERED_IN_DEADZONE", **common)

        outside_x = abs(dx) > self.config.trigger_x_px
        outside_y = abs(dy) > self.config.trigger_y_px
        if not outside_x and not outside_y:
            self._clear_laser_ready()
            return self._result("CENTER_HYSTERESIS_HOLD", **common)

        if self._correction_count >= self.config.max_corrections_per_lock:
            self._clear_laser_ready()
            return self._result("CORRECTION_LIMIT_REACHED", **common)

        delta_az = (
            dx * self.config.fov_x_deg / self.config.image_width
            if outside_x else 0.0
        )
        delta_el = (
            -dy * self.config.fov_y_deg / self.config.image_height
            if outside_y else 0.0
        )
        delta_az = max(
            -self.config.max_step_az_deg,
            min(self.config.max_step_az_deg, delta_az),
        )
        delta_el = max(
            -self.config.max_step_el_deg,
            min(self.config.max_step_el_deg, delta_el),
        )
        self._clear_laser_ready()
        return self._result(
            "ALIGNMENT_READY",
            **common,
            command_requested=True,
            delta_az_deg=delta_az,
            delta_el_deg=delta_el,
        )


def quantize_angle_0p1(value: float) -> float:
    """Round a finite angle to one decimal using symmetric half-up rules."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("angle must be finite")
    magnitude = math.floor(abs(value) * 10.0 + 0.5) / 10.0
    return math.copysign(magnitude, value)
