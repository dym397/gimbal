"""Single-target YOLO-to-gimbal visual alignment gate.

This module contains no camera, tracker, or serial I/O.  In the single-target
branch the sole YOLO box in the gimbal image is treated as the current master.
Production uses one fresh bbox directly; the configurable sample window remains
available for offline diagnostics. The caller owns the command queue.
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
    aim_offset_x_px: float = 0.0
    aim_offset_y_px: float = 0.0
    fov_x_deg: float = 17.5
    fov_y_deg: float = 9.9
    stable_frames: int = 1
    max_center_std_x_px: float = 10.0
    max_center_std_y_px: float = 10.0
    max_center_speed_x_px_s: float = 15.0
    max_center_speed_y_px_s: float = 15.0
    trigger_x_px: float = 10.0
    trigger_y_px: float = 10.0
    centered_x_px: float = 10.0
    centered_y_px: float = 10.0
    max_latest_to_median_px: float = 10.0
    max_center_jump_px: float = 0.0
    min_bbox_iou: float = 0.0
    recovery_confirm_frames: int = 1
    # <= 0 disables bbox age rejection in the direct-control production path.
    max_frame_age_s: float = 0.0
    laser_ready_radius_px: float = 20.0
    min_fine_step_deg: float = 0.10
    # <= 0 disables scan mode and directly converts bbox error to angle.
    fine_scan_max_error_px: float = 0.0
    # <= 0 disables time-gap resets. Target/gimbal/identity gates still reset.
    max_frame_gap_s: float = 0.0
    max_step_az_deg: float = 3.0
    max_step_el_deg: float = 2.0
    # <= 0 disables the correction-count limit so bbox motion keeps driving.
    max_corrections_per_lock: int = 0


class SingleTargetVisualAlignment:
    """Turn fresh single-target bbox centers into gimbal corrections."""

    def __init__(self, config: SingleTargetAlignmentConfig):
        if config.stable_frames < 1:
            raise ValueError("stable_frames must be at least 1")
        if config.max_corrections_per_lock < 0:
            raise ValueError("max_corrections_per_lock must not be negative")
        if config.recovery_confirm_frames < 1:
            raise ValueError("recovery_confirm_frames must be at least 1")
        if config.image_width <= 0.0 or config.image_height <= 0.0:
            raise ValueError("image dimensions must be positive")
        if config.laser_ready_radius_px < 0.0:
            raise ValueError("laser_ready_radius_px must not be negative")
        aim_x = config.image_width / 2.0 + config.aim_offset_x_px
        aim_y = config.image_height / 2.0 + config.aim_offset_y_px
        if not (0.0 <= aim_x < config.image_width and 0.0 <= aim_y < config.image_height):
            raise ValueError("laser aim point must remain inside the image")
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
        self.aim_x_px = aim_x
        self.aim_y_px = aim_y
        self._samples = deque(maxlen=config.stable_frames)
        self._identity = None
        self._last_frame_ts = 0.0
        self._last_input_frame_ts = 0.0
        self._recovery_required = False
        self._fresh_after_dropout = 0
        self._post_command_recheck = False
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
        self._last_input_frame_ts = 0.0
        self._recovery_required = False
        self._fresh_after_dropout = 0

    def release_target(self) -> None:
        """Clear every latch when the gimbal-camera target is released."""
        self.reset_samples()
        self._post_command_recheck = False
        self._waiting_after_command = False
        self._command_stationary_ts = 0.0
        self._correction_identity = None
        self._correction_count = 0
        self._clear_laser_ready()

    def mark_command_sent(self, stationary_ts: float) -> None:
        self._waiting_after_command = True
        self._command_stationary_ts = float(stationary_ts)
        self._correction_count += 1
        self._clear_laser_ready()
        self.reset_samples()
        self._post_command_recheck = True

    def build_no_return_scan_decision(
        self,
        current_decision: dict,
        *,
        scan_offset_az_deg: float,
        scan_offset_el_deg: float,
    ) -> Optional[dict]:
        """Build one 0.1-degree scan command around the latest bbox center.

        The proportional term first points back at the latest bbox center;
        the supplied absolute scan offset then selects one point around that
        center.  This prevents scan error from accumulating across steps.
        """
        if (
            not current_decision.get("processed_new_frame", False)
            or not current_decision.get("laser_ready", False)
        ):
            return None
        try:
            dx = float(current_decision["dx_px"])
            dy = float(current_decision["dy_px"])
            offset_az = float(scan_offset_az_deg)
            offset_el = float(scan_offset_el_deg)
        except (KeyError, TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in (dx, dy, offset_az, offset_el)):
            return None

        delta_az = dx * self.config.fov_x_deg / self.config.image_width + offset_az
        delta_el = -dy * self.config.fov_y_deg / self.config.image_height + offset_el
        delta_az = max(
            -self.config.max_step_az_deg,
            min(self.config.max_step_az_deg, delta_az),
        )
        delta_el = max(
            -self.config.max_step_el_deg,
            min(self.config.max_step_el_deg, delta_el),
        )
        result = dict(current_decision)
        result.update({
            "state": "LASER_NO_RETURN_SCAN_READY",
            "command_requested": True,
            "control_mode": "laser_no_return_bbox_center_scan",
            "delta_az_deg": delta_az,
            "delta_el_deg": delta_el,
            "scan_offset_az_deg": offset_az,
            "scan_offset_el_deg": offset_el,
            "laser_ready": False,
            "laser_ready_since_ts": 0.0,
        })
        return result

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
    def _finite_bbox(box) -> Optional[tuple[float, float, float, float]]:
        if box is None or len(box) != 4:
            return None
        try:
            values = tuple(float(value) for value in box)
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        if values[2] <= values[0] or values[3] <= values[1]:
            return None
        return values

    @staticmethod
    def _bbox_iou(
        left: tuple[float, float, float, float],
        right: tuple[float, float, float, float],
    ) -> float:
        x1 = max(left[0], right[0])
        y1 = max(left[1], right[1])
        x2 = min(left[2], right[2])
        y2 = min(left[3], right[3])
        intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        left_area = (left[2] - left[0]) * (left[3] - left[1])
        right_area = (right[2] - right[0]) * (right[3] - right[1])
        union = left_area + right_area - intersection
        return intersection / union if union > 1e-9 else 0.0

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
            "aim_x_px": self.aim_x_px,
            "aim_y_px": self.aim_y_px,
            "dx_px": math.nan,
            "dy_px": math.nan,
            "std_x_px": math.nan,
            "std_y_px": math.nan,
            "speed_x_px_s": math.nan,
            "speed_y_px_s": math.nan,
            "latest_to_median_px": math.nan,
            "frame_age_s": math.nan,
            "fresh_after_dropout": self._fresh_after_dropout,
            "outlier_center_jump_px": math.nan,
            "outlier_bbox_iou": math.nan,
            "control_mode": "none",
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
        decision_ts: Optional[float] = None,
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
            self._post_command_recheck = False
            return self._result("NO_LOCKED_TARGET")

        frame_ts = float(frame_ts)
        if not math.isfinite(frame_ts) or frame_ts <= 0.0:
            return self._result("INVALID_FRAME_TIMESTAMP")
        if frame_ts <= self._last_input_frame_ts + 1e-9:
            return self._result("DUPLICATE_FRAME")

        master_track_id = int(master_track_id)
        if self._identity is not None and self._identity[0] != master_track_id:
            self.reset_samples()
            self._post_command_recheck = False

        if int(detection_count) > 1:
            self._clear_laser_ready()
            self.reset_samples()
            self._post_command_recheck = False
            self._last_input_frame_ts = frame_ts
            return self._result(
                "REQUIRE_EXACTLY_ONE_DETECTION",
                processed_new_frame=True,
            )
        if int(detection_count) == 0 or not measurement:
            self._clear_laser_ready()
            self._recovery_required = True
            self._fresh_after_dropout = 0
            self._last_input_frame_ts = frame_ts
            return self._result(
                "DETECTION_MISSED_HOLD",
                processed_new_frame=True,
            )

        center = self._finite_pair(measurement.get("center"))
        bbox = self._finite_bbox(measurement.get("bbox"))
        simple_id = measurement.get("simple_id")
        if center is None or simple_id is None:
            self._clear_laser_ready()
            self.reset_samples()
            self._post_command_recheck = False
            self._last_input_frame_ts = frame_ts
            return self._result(
                "INVALID_YOLO_DETECTION",
                processed_new_frame=True,
            )

        identity = (master_track_id, int(simple_id))
        same_correction_identity = identity == self._correction_identity
        if not same_correction_identity:
            self._correction_identity = identity
            self._correction_count = 0
            self._clear_laser_ready()
        if identity != self._identity:
            preserve_post_command = (
                self._post_command_recheck
                and same_correction_identity
            )
            self.reset_samples()
            self._identity = identity
            self._post_command_recheck = preserve_post_command
        elif (
            self.config.max_frame_gap_s > 0.0
            and self._last_frame_ts > 0.0
            and frame_ts - self._last_frame_ts > self.config.max_frame_gap_s
        ):
            self._samples.clear()

        self._last_input_frame_ts = frame_ts
        reported_missing_frames = int(
            measurement.get("bbox_jitter_previous_missing_frames", 0) or 0
        )
        if reported_missing_frames > 0 and not self._recovery_required:
            self._recovery_required = True
            self._fresh_after_dropout = 0

        center_jump = math.nan
        bbox_iou = math.nan
        if self._samples:
            previous = self._samples[-1]
            center_jump = math.hypot(
                center[0] - previous[1], center[1] - previous[2]
            )
            if bbox is not None and previous[3] is not None:
                bbox_iou = self._bbox_iou(previous[3], bbox)
            jump_rejected = (
                self.config.max_center_jump_px > 0.0
                and center_jump > self.config.max_center_jump_px
            )
            iou_rejected = (
                self.config.min_bbox_iou > 0.0
                and math.isfinite(bbox_iou)
                and bbox_iou < self.config.min_bbox_iou
            )
            if jump_rejected or iou_rejected:
                self._samples.clear()
                self._samples.append((frame_ts, center[0], center[1], bbox))
                self._last_frame_ts = frame_ts
                self._recovery_required = True
                self._fresh_after_dropout = 1
                self._clear_laser_ready()
                return self._result(
                    "OUTLIER_REACQUIRE",
                    processed_new_frame=True,
                    sample_count=1,
                    outlier_center_jump_px=center_jump,
                    outlier_bbox_iou=bbox_iou,
                )

        self._last_frame_ts = frame_ts
        self._samples.append((frame_ts, center[0], center[1], bbox))
        if self._recovery_required:
            self._fresh_after_dropout += 1
            recovery_frames = (
                1 if self._post_command_recheck
                else self.config.recovery_confirm_frames
            )
            if self._fresh_after_dropout >= recovery_frames:
                self._recovery_required = False

        recovering = self._recovery_required
        required_samples = 1 if self._post_command_recheck else self.config.stable_frames
        if len(self._samples) < required_samples:
            self._clear_laser_ready()
            return self._result(
                "RECOVERING_AFTER_DROPOUT" if recovering else "ACCUMULATING",
                processed_new_frame=True,
                sample_count=len(self._samples),
            )

        if recovering:
            self._clear_laser_ready()
            return self._result(
                "RECOVERING_AFTER_DROPOUT",
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
        # Median control rejects the normal 3-6 px YOLO center jitter observed
        # in field logs while still using the latest valid four detections.
        control_x = float(statistics.median(x_values))
        control_y = float(statistics.median(y_values))
        latest_to_median = math.hypot(
            center[0] - control_x, center[1] - control_y
        )
        dx = control_x - self.aim_x_px
        dy = control_y - self.aim_y_px
        stable = (
            std_x <= self.config.max_center_std_x_px
            and std_y <= self.config.max_center_std_y_px
            and latest_to_median <= self.config.max_latest_to_median_px
        )
        decision_ts = frame_ts if decision_ts is None else float(decision_ts)
        frame_age = max(0.0, decision_ts - frame_ts)
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
            "latest_to_median_px": latest_to_median,
            "frame_age_s": frame_age,
            "fresh_after_dropout": self._fresh_after_dropout,
        }
        if not stable:
            self._clear_laser_ready()
            return self._result("TARGET_NOT_STABLE", **common)

        if (
            self.config.max_frame_age_s > 0.0
            and frame_age > self.config.max_frame_age_s
        ):
            self._clear_laser_ready()
            if self._post_command_recheck:
                self._samples.clear()
            return self._result("STALE_FRAME_HOLD", **common)

        laser_aim_error_px = math.hypot(dx, dy)
        if laser_aim_error_px <= self.config.laser_ready_radius_px:
            self._correction_count = 0
            self._post_command_recheck = False
            if not self._laser_ready:
                self._laser_ready = True
                self._laser_ready_since_ts = frame_ts
            return self._result("CENTERED_IN_DEADZONE", **common)

        outside_x = abs(dx) > self.config.trigger_x_px
        outside_y = abs(dy) > self.config.trigger_y_px
        if not outside_x and not outside_y:
            self._clear_laser_ready()
            return self._result("CENTER_HYSTERESIS_HOLD", **common)

        if (
            self.config.max_corrections_per_lock > 0
            and self._correction_count >= self.config.max_corrections_per_lock
        ):
            self._clear_laser_ready()
            self._post_command_recheck = False
            return self._result("CORRECTION_LIMIT_REACHED", **common)

        fine_scan_x = (
            self.config.fine_scan_max_error_px > 0.0
            and outside_x
            and abs(dx) <= self.config.fine_scan_max_error_px
        )
        fine_scan_y = (
            self.config.fine_scan_max_error_px > 0.0
            and outside_y
            and abs(dy) <= self.config.fine_scan_max_error_px
        )
        delta_az = (
            math.copysign(self.config.min_fine_step_deg, dx)
            if fine_scan_x else (
                dx * self.config.fov_x_deg / self.config.image_width
                if outside_x else 0.0
            )
        )
        delta_el = (
            math.copysign(self.config.min_fine_step_deg, -dy)
            if fine_scan_y else (
                -dy * self.config.fov_y_deg / self.config.image_height
                if outside_y else 0.0
            )
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
            control_mode=(
                "direct_bbox"
                if self.config.fine_scan_max_error_px <= 0.0
                else (
                    "fine_0p1_scan"
                    if (fine_scan_x or not outside_x)
                    and (fine_scan_y or not outside_y)
                    else (
                        "hybrid_scan_coarse"
                        if fine_scan_x or fine_scan_y
                        else "proportional_coarse"
                    )
                )
            ),
        )


def quantize_angle_0p1(value: float) -> float:
    """Round a finite angle to one decimal using symmetric half-up rules."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("angle must be finite")
    magnitude = math.floor(abs(value) * 10.0 + 0.5) / 10.0
    return math.copysign(magnitude, value)
