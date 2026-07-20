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
        "stable_frames": 4,
        "max_center_std_x_px": 10.0,
        "max_center_std_y_px": 10.0,
        "max_center_speed_x_px_s": 15.0,
        "max_center_speed_y_px_s": 15.0,
        "trigger_x_px": 10.0,
        "trigger_y_px": 10.0,
        "centered_x_px": 10.0,
        "centered_y_px": 10.0,
        "max_frame_gap_s": 0.0,
        "max_latest_to_median_px": 10.0,
        "max_center_jump_px": 30.0,
        "min_bbox_iou": 0.30,
        "recovery_confirm_frames": 2,
        "max_frame_age_s": 0.80,
        "min_fine_step_deg": 0.10,
        "fine_scan_max_error_px": 50.0,
    }
    values.update(overrides)
    return SingleTargetVisualAlignment(
        SingleTargetAlignmentConfig(**values)
    )


def _measurement(
    x, y, simple_id=7, width=100.0, height=80.0,
    previous_missing_frames=0,
):
    return {
        "center": [x, y],
        "bbox": [
            x - width / 2.0,
            y - height / 2.0,
            x + width / 2.0,
            y + height / 2.0,
        ],
        "simple_id": simple_id,
        "bbox_jitter_previous_missing_frames": previous_missing_frames,
    }


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
    return _feed(controller, [center] * 4, stationary_ts=stationary_ts)


def test_stable_target_outside_deadzone_requests_one_correctly_signed_step():
    controller = _controller()
    result = _feed_stable(controller, (1480.0, 820.0))

    assert result["state"] == "ALIGNMENT_READY"
    assert result["command_requested"] is True
    assert abs(result["delta_az_deg"] - 200.0 * 17.5 / 2560.0) < 1e-9
    assert abs(result["delta_el_deg"] + 100.0 * 9.9 / 1440.0) < 1e-9


def test_stable_target_inside_deadzone_does_not_move_gimbal():
    controller = _controller()
    result = _feed_stable(controller, (1290.0, 710.0))

    assert result["state"] == "CENTERED_IN_DEADZONE"
    assert result["stable"] is True
    assert result["command_requested"] is False
    assert result["laser_ready"] is True
    assert result["laser_ready_since_ts"] == 1.6


def test_laser_ready_stays_latched_for_duplicate_main_loop_reads():
    controller = _controller()
    centered = _feed_stable(controller, (1280.0, 720.0))
    assert centered["laser_ready"] is True

    duplicate = controller.update(
        frame_ts=1.6,
        master_track_id=11,
        measurement=_measurement(1280.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert duplicate["state"] == "DUPLICATE_FRAME"
    assert duplicate["laser_ready"] is True
    assert duplicate["laser_ready_since_ts"] == 1.6


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


def test_disabled_frame_gap_limit_keeps_samples_across_long_dropout():
    controller = _controller()
    first = controller.update(
        frame_ts=1.0,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    after_long_gap = controller.update(
        frame_ts=11.0,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    after_another_long_gap = controller.update(
        frame_ts=31.0,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert first["sample_count"] == 1
    assert after_long_gap["sample_count"] == 2
    assert after_another_long_gap["sample_count"] == 3


def test_positive_frame_gap_limit_can_still_enable_strict_reset():
    controller = _controller(max_frame_gap_s=2.0)
    controller.update(
        frame_ts=1.0,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    result = controller.update(
        frame_ts=3.1,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert result["state"] == "ACCUMULATING"
    assert result["sample_count"] == 1


def test_missed_detection_holds_samples_and_requires_two_fresh_returns():
    controller = _controller()
    _feed(controller, [(1480.0, 720.0)] * 2)
    missed = controller.update(
        frame_ts=1.4,
        master_track_id=11,
        measurement=None,
        detection_count=0,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    first_return = controller.update(
        frame_ts=1.6,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    second_return = controller.update(
        frame_ts=1.8,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert missed["state"] == "DETECTION_MISSED_HOLD"
    assert missed["sample_count"] == 2
    assert first_return["state"] == "RECOVERING_AFTER_DROPOUT"
    assert first_return["sample_count"] == 3
    assert second_return["state"] == "ALIGNMENT_READY"
    assert second_return["fresh_after_dropout"] == 2


def test_detector_reported_misses_require_two_fresh_returns():
    controller = _controller()
    _feed(controller, [(1480.0, 720.0)] * 3)
    first_return = controller.update(
        frame_ts=1.6,
        master_track_id=11,
        measurement=_measurement(
            1480.0, 720.0, previous_missing_frames=3
        ),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    second_return = controller.update(
        frame_ts=1.8,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert first_return["state"] == "RECOVERING_AFTER_DROPOUT"
    assert second_return["state"] == "ALIGNMENT_READY"


def test_large_center_jump_reacquires_instead_of_commanding():
    controller = _controller()
    _feed(controller, [(1280.0, 720.0)] * 3)
    result = controller.update(
        frame_ts=1.6,
        master_track_id=11,
        measurement=_measurement(1400.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert result["state"] == "OUTLIER_REACQUIRE"
    assert result["sample_count"] == 1
    assert result["outlier_center_jump_px"] == 120.0
    assert result["command_requested"] is False


def test_low_bbox_iou_reacquires_even_without_center_jump():
    controller = _controller()
    controller.update(
        frame_ts=1.0,
        master_track_id=11,
        measurement=_measurement(1280.0, 720.0, width=100.0, height=80.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    result = controller.update(
        frame_ts=1.2,
        master_track_id=11,
        measurement=_measurement(1280.0, 720.0, width=20.0, height=16.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert result["state"] == "OUTLIER_REACQUIRE"
    assert result["outlier_bbox_iou"] < 0.30


def test_stale_latest_frame_never_requests_gimbal_command():
    controller = _controller()
    _feed(controller, [(1480.0, 720.0)] * 3)
    result = controller.update(
        frame_ts=1.6,
        decision_ts=2.5,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert result["state"] == "STALE_FRAME_HOLD"
    assert result["frame_age_s"] > 0.80
    assert result["command_requested"] is False


def test_near_center_alignment_uses_exact_point_one_degree_scan():
    controller = _controller()
    result = _feed_stable(controller, (1310.0, 700.0))

    assert result["state"] == "ALIGNMENT_READY"
    assert result["control_mode"] == "fine_0p1_scan"
    assert result["delta_az_deg"] == 0.1
    assert result["delta_el_deg"] == 0.1


def test_each_scan_step_uses_first_bbox_after_new_stationary_event():
    controller = _controller(laser_ready_radius_px=0.0)
    first = _feed_stable(controller, (1310.0, 720.0))
    assert first["state"] == "ALIGNMENT_READY"
    controller.mark_command_sent(stationary_ts=1.0)

    controller.update(
        frame_ts=2.0,
        master_track_id=11,
        measurement=None,
        detection_count=0,
        gimbal_stationary=False,
        stationary_ts=1.0,
    )
    next_step = controller.update(
        frame_ts=2.2,
        master_track_id=11,
        measurement=_measurement(1295.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=2.1,
    )

    assert next_step["sample_count"] == 1
    assert next_step["state"] == "ALIGNMENT_READY"
    assert next_step["control_mode"] == "fine_0p1_scan"
    assert next_step["delta_az_deg"] == 0.1


def test_target_identity_change_after_command_returns_to_four_frame_gate():
    controller = _controller()
    first = _feed_stable(controller, (1310.0, 720.0))
    assert first["command_requested"] is True
    controller.mark_command_sent(stationary_ts=1.0)
    controller.update(
        frame_ts=2.0,
        master_track_id=11,
        measurement=None,
        detection_count=0,
        gimbal_stationary=False,
        stationary_ts=1.0,
    )
    switched = controller.update(
        frame_ts=2.2,
        master_track_id=11,
        measurement=_measurement(1310.0, 720.0, simple_id=8),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=2.1,
    )

    assert switched["state"] == "ACCUMULATING"
    assert switched["sample_count"] == 1
    assert switched["command_requested"] is False


def test_point_one_degree_closed_loop_scan_reaches_center_in_three_steps():
    controller = _controller(max_corrections_per_lock=3)
    decision = _feed_stable(controller, (1324.0, 720.0))
    assert decision["delta_az_deg"] == 0.1

    for index, center_x in enumerate((1309.4, 1294.8, 1280.2), start=1):
        stationary_before = float(index)
        controller.mark_command_sent(stationary_ts=stationary_before)
        controller.update(
            frame_ts=2.0 + index,
            master_track_id=11,
            measurement=None,
            detection_count=0,
            gimbal_stationary=False,
            stationary_ts=stationary_before,
        )
        decision = controller.update(
            frame_ts=2.1 + index,
            master_track_id=11,
            measurement=_measurement(center_x, 720.0),
            detection_count=1,
            gimbal_stationary=True,
            stationary_ts=stationary_before + 0.5,
        )

    assert decision["state"] == "CENTERED_IN_DEADZONE"
    assert decision["sample_count"] == 1
    assert decision["laser_ready"] is True


def test_command_latch_requires_a_new_stationary_event_before_rearming():
    controller = _controller(laser_ready_radius_px=0.0)
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
    assert rearmed["state"] == "ALIGNMENT_READY"
    assert rearmed["sample_count"] == 1
    assert rearmed["control_mode"] == "fine_0p1_scan"
    assert rearmed["delta_az_deg"] == 0.1


def test_visual_lock_release_clears_command_wait_latch():
    controller = SingleTargetVisualAlignment(SingleTargetAlignmentConfig())
    ready = controller.update(
        frame_ts=1.0,
        master_track_id=11,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    assert ready["command_requested"] is True
    controller.mark_command_sent(stationary_ts=1.0)
    controller.release_target()
    reacquired = controller.update(
        frame_ts=1.1,
        master_track_id=12,
        measurement=_measurement(1480.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    assert reacquired["state"] == "ALIGNMENT_READY"
    assert reacquired["command_requested"] is True


def test_axis_inside_deadzone_is_not_adjusted():
    controller = _controller()
    result = _feed_stable(controller, (1480.0, 728.0))

    assert result["command_requested"] is True
    assert result["delta_az_deg"] > 0.0
    assert result["delta_el_deg"] == 0.0


def test_center_boundary_is_accepted_without_command():
    controller = _controller()
    result = _feed_stable(controller, (1290.0, 720.0))

    assert result["state"] == "CENTERED_IN_DEADZONE"
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


def test_fourth_stable_frame_can_request_alignment_immediately():
    controller = _controller()
    result = _feed(controller, [(1480.0, 720.0)] * 4)

    assert result["state"] == "ALIGNMENT_READY"
    assert result["stable"] is True
    assert result["sample_count"] == 4
    assert result["command_requested"] is True


def test_control_coordinate_uses_four_frame_median_after_confirmation():
    controller = _controller()
    _feed(controller, [(1480.0, 720.0)] * 3)
    result = controller.update(
        frame_ts=1.6,
        master_track_id=11,
        measurement=_measurement(1485.0, 722.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert result["state"] == "ALIGNMENT_READY"
    assert result["control_x"] == 1480.0
    assert result["control_y"] == 720.0
    assert abs(result["latest_to_median_px"] - 29.0 ** 0.5) < 1e-9


def test_angle_quantization_keeps_exactly_one_decimal_resolution():
    assert quantize_angle_0p1(77.866) == 77.9
    assert quantize_angle_0p1(-0.8975) == -0.9
    assert quantize_angle_0p1(10.04) == 10.0
    assert quantize_angle_0p1(10.05) == 10.1


def test_production_default_has_no_closed_loop_correction_limit():
    config = SingleTargetAlignmentConfig()
    assert config.max_corrections_per_lock == 0
    assert config.stable_frames == 1
    assert config.recovery_confirm_frames == 1
    assert config.max_center_jump_px == 0.0
    assert config.min_bbox_iou == 0.0
    assert config.fine_scan_max_error_px == 0.0


def test_production_default_uses_first_bbox_without_stability_gate():
    controller = SingleTargetVisualAlignment(SingleTargetAlignmentConfig())
    result = controller.update(
        frame_ts=1.0,
        decision_ts=1.1,
        master_track_id=11,
        measurement=_measurement(1310.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert result["sample_count"] == 1
    assert result["state"] == "ALIGNMENT_READY"
    assert result["control_mode"] == "direct_bbox"
    assert abs(result["delta_az_deg"] - 30.0 * 17.5 / 2560.0) < 1e-9


def test_production_keeps_following_bbox_beyond_six_corrections():
    controller = SingleTargetVisualAlignment(SingleTargetAlignmentConfig())
    frame_ts = 1.0
    stationary_ts = 1.0
    for _ in range(8):
        result = controller.update(
            frame_ts=frame_ts,
            master_track_id=11,
            measurement=_measurement(1480.0, 720.0),
            detection_count=1,
            gimbal_stationary=True,
            stationary_ts=stationary_ts,
        )
        assert result["state"] == "ALIGNMENT_READY"
        assert result["command_requested"] is True
        controller.mark_command_sent(stationary_ts=stationary_ts)
        frame_ts += 0.1
        controller.update(
            frame_ts=frame_ts,
            master_track_id=11,
            measurement=None,
            detection_count=0,
            gimbal_stationary=False,
            stationary_ts=stationary_ts,
        )
        frame_ts += 0.1
        stationary_ts += 1.0


def test_production_default_first_bbox_inside_center_enables_laser():
    controller = SingleTargetVisualAlignment(SingleTargetAlignmentConfig())
    result = controller.update(
        frame_ts=1.0,
        master_track_id=11,
        measurement=_measurement(1280.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert result["state"] == "CENTERED_IN_DEADZONE"
    assert result["laser_ready"] is True


def test_calibrated_laser_aim_point_is_alignment_center():
    controller = _controller(aim_offset_x_px=10.0, aim_offset_y_px=4.0)

    centered = _feed_stable(controller, (1290.0, 724.0))

    assert centered["state"] == "CENTERED_IN_DEADZONE"
    assert centered["dx_px"] == 0.0
    assert centered["dy_px"] == 0.0
    assert centered["aim_x_px"] == 1290.0
    assert centered["aim_y_px"] == 724.0
    assert centered["laser_ready"] is True


def test_calibrated_laser_aim_offset_changes_angle_error_reference():
    controller = _controller(
        aim_offset_x_px=10.0,
        aim_offset_y_px=4.0,
        centered_x_px=0.0,
        centered_y_px=0.0,
        trigger_x_px=0.0,
        trigger_y_px=0.0,
        laser_ready_radius_px=0.0,
    )

    result = _feed_stable(controller, (1280.0, 720.0))

    assert result["state"] == "ALIGNMENT_READY"
    assert result["dx_px"] == -10.0
    assert result["dy_px"] == -4.0
    assert result["delta_az_deg"] < 0.0
    assert result["delta_el_deg"] > 0.0


def test_production_default_disables_bbox_age_rejection():
    controller = SingleTargetVisualAlignment(SingleTargetAlignmentConfig())
    result = controller.update(
        frame_ts=1.0,
        decision_ts=10.0,
        master_track_id=11,
        measurement=_measurement(1320.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert result["frame_age_s"] == 9.0
    assert result["state"] == "ALIGNMENT_READY"
    assert result["command_requested"] is True


def test_laser_ready_radius_uses_twenty_pixel_euclidean_distance():
    controller = SingleTargetVisualAlignment(SingleTargetAlignmentConfig())
    boundary = controller.update(
        frame_ts=1.0,
        master_track_id=11,
        measurement=_measurement(1300.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    outside = controller.update(
        frame_ts=1.1,
        master_track_id=11,
        measurement=_measurement(1300.1, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )

    assert boundary["state"] == "CENTERED_IN_DEADZONE"
    assert boundary["laser_ready"] is True
    assert outside["state"] == "ALIGNMENT_READY"
    assert outside["laser_ready"] is False


def test_no_return_scan_is_absolute_point_one_degree_around_latest_bbox():
    controller = SingleTargetVisualAlignment(SingleTargetAlignmentConfig())
    centered = controller.update(
        frame_ts=1.0,
        master_track_id=11,
        measurement=_measurement(1290.0, 720.0),
        detection_count=1,
        gimbal_stationary=True,
        stationary_ts=1.0,
    )
    scan = controller.build_no_return_scan_decision(
        centered,
        scan_offset_az_deg=0.1,
        scan_offset_el_deg=0.0,
    )

    assert scan is not None
    assert scan["state"] == "LASER_NO_RETURN_SCAN_READY"
    assert scan["control_mode"] == "laser_no_return_bbox_center_scan"
    assert scan["command_requested"] is True
    assert scan["laser_ready"] is False
    assert abs(scan["delta_az_deg"] - (10.0 * 17.5 / 2560.0 + 0.1)) < 1e-9
    assert scan["delta_el_deg"] == 0.0
