from pathlib import Path
import sys


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from single_target_visual_alignment import (  # noqa: E402
    SingleTargetAlignmentConfig,
    SingleTargetVisualAlignment,
    quantize_angle_0p1,
)


def _controller(**overrides):
    values = {
        "stable_frames": 6,
        "max_center_std_x_px": 12.0,
        "max_center_std_y_px": 12.0,
        "max_center_speed_x_px_s": 15.0,
        "max_center_speed_y_px_s": 15.0,
        "trigger_x_px": 50.0,
        "trigger_y_px": 50.0,
        "centered_x_px": 30.0,
        "centered_y_px": 30.0,
    }
    values.update(overrides)
    return SingleTargetVisualAlignment(
        SingleTargetAlignmentConfig(**values)
    )


def _measurement(x, y, simple_id=7):
    return {"center": [x, y], "simple_id": simple_id}


def _feed(controller, centers, stationary_ts=1.0):
    result = None
    for index, center in enumerate(centers):
        result = controller.update(
            frame_ts=1.0 + index * 0.2,
            master_track_id=11,
            measurement=_measurement(*center),
            detection_count=1,
            gimbal_stationary=True,
            stationary_ts=stationary_ts,
        )
    return result


def _feed_stable(controller, center, stationary_ts=1.0):
    return _feed(controller, [center] * 6, stationary_ts=stationary_ts)


def test_stable_target_outside_deadzone_requests_one_correctly_signed_step():
    controller = _controller()
    result = _feed_stable(controller, (1480.0, 820.0))

    assert result["state"] == "ALIGNMENT_READY"
    assert result["command_requested"] is True
    assert abs(result["delta_az_deg"] - 200.0 * 17.5 / 2560.0) < 1e-9
    assert abs(result["delta_el_deg"] + 100.0 * 9.9 / 1440.0) < 1e-9


def test_stable_target_inside_deadzone_does_not_move_gimbal():
    controller = _controller()
    result = _feed_stable(controller, (1298.0, 696.0))

    assert result["state"] == "CENTERED_IN_DEADZONE"
    assert result["stable"] is True
    assert result["command_requested"] is False
    assert result["laser_ready"] is True
    assert result["laser_ready_since_ts"] == 2.0


def test_laser_ready_stays_latched_for_duplicate_main_loop_reads():
    controller = _controller()
    centered = _feed_stable(controller, (1280.0, 720.0))
    assert centered["laser_ready"] is True

    duplicate = controller.update(
        frame_ts=2.0,
        master_track_id=11,
        measurement=_measurement(1280.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert duplicate["state"] == "DUPLICATE_FRAME"
    assert duplicate["laser_ready"] is True
    assert duplicate["laser_ready_since_ts"] == 2.0


def test_laser_ready_clears_when_target_or_gimbal_is_not_stable():
    controller = _controller()
    assert _feed_stable(controller, (1280.0, 720.0))["laser_ready"] is True

    moving = controller.update(
        frame_ts=2.2,
        master_track_id=11,
        measurement=None,
        detection_count=0,
        gimbal_stationary=False,
        stationary_ts=1.0,
    )

    assert moving["laser_ready"] is False


def test_continuously_moving_target_never_becomes_alignment_ready():
    controller = _controller()
    centers = [(1400.0 + 20.0 * index, 720.0) for index in range(12)]
    result = _feed(controller, centers)

    assert result["state"] == "TARGET_NOT_STABLE"
    assert result["command_requested"] is False
    assert abs(result["speed_x_px_s"]) > 15.0


def test_more_than_one_yolo_detection_resets_stability_window():
    controller = _controller()
    _feed(controller, [(1480.0, 720.0)] * 5)
    result = controller.update(
        frame_ts=2.2,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=2,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert result["state"] == "REQUIRE_EXACTLY_ONE_DETECTION"
    assert result["sample_count"] == 0


def test_command_latch_requires_a_new_stationary_event_before_rearming():
    controller = _controller()
    ready = _feed_stable(
        controller, (1480.0, 720.0), stationary_ts=1.0
    )
    assert ready["command_requested"] is True
    controller.mark_command_sent(stationary_ts=1.0)

    waiting = controller.update(
        frame_ts=3.0,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    assert waiting["state"] == "WAIT_NEW_STATIONARY_EVENT"

    moving = controller.update(
        frame_ts=3.2,
        master_track_id=11,
        measurement=None,
        detection_count=0,
        gimbal_stationary=False,
        stationary_ts=1.0,
    )
    assert moving["state"] == "GIMBAL_MOVING"

    rearmed = controller.update(
        frame_ts=4.0,
        master_track_id=11,
        measurement=_measurement(1300.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=3.8,
    )
    assert rearmed["state"] == "ACCUMULATING"
    assert rearmed["sample_count"] == 1


def test_axis_inside_deadzone_is_not_adjusted():
    controller = _controller()
    result = _feed_stable(controller, (1480.0, 735.0))

    assert result["command_requested"] is True
    assert result["delta_az_deg"] > 0.0
    assert result["delta_el_deg"] == 0.0


def test_center_hysteresis_band_holds_without_command():
    controller = _controller()
    result = _feed_stable(controller, (1320.0, 720.0))

    assert result["state"] == "CENTER_HYSTERESIS_HOLD"
    assert result["stable"] is True
    assert result["command_requested"] is False


def test_correction_limit_prevents_endless_retries_for_one_lock():
    controller = _controller(max_corrections_per_lock=1)
    ready = _feed_stable(
        controller, (1480.0, 720.0), stationary_ts=1.0
    )
    controller.mark_command_sent(stationary_ts=1.0)
    controller.update(
        frame_ts=3.0,
        master_track_id=11,
        measurement=None,
        detection_count=0,
        gimbal_stationary=False,
        stationary_ts=1.0,
    )
    assert ready["command_requested"] is True

    result = None
    for index in range(6):
        result = controller.update(
            frame_ts=4.0 + index * 0.2,
            master_track_id=11,
            measurement=_measurement(1480.0, 720.0),
            detection_count=1,
            gimbal_stationary=True,
            stationary_ts=3.8,
        )

    assert result["state"] == "CORRECTION_LIMIT_REACHED"
    assert result["command_requested"] is False


def test_sixth_stable_frame_can_request_alignment_immediately():
    controller = _controller()
    result = _feed(controller, [(1480.0, 720.0)] * 6)

    assert result["state"] == "ALIGNMENT_READY"
    assert result["stable"] is True
    assert result["sample_count"] == 6
    assert result["command_requested"] is True


def test_control_coordinate_uses_current_sixth_frame_after_confirmation():
    controller = _controller()
    _feed(controller, [(1480.0, 720.0)] * 5)
    result = controller.update(
        frame_ts=2.0,
        master_track_id=11,
        measurement=_measurement(1485.0, 722.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert result["state"] == "ALIGNMENT_READY"
    assert result["control_x"] == 1485.0
    assert result["control_y"] == 722.0


def test_angle_quantization_keeps_exactly_one_decimal_resolution():
    assert quantize_angle_0p1(77.866) == 77.9
    assert quantize_angle_0p1(-0.8975) == -0.9
    assert quantize_angle_0p1(10.04) == 10.0
    assert quantize_angle_0p1(10.05) == 10.1
