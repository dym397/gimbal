from pathlib import Path
import sys


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from gimbal_visual_target_lock import (  # noqa: E402
    GimbalVisualLockConfig,
    GimbalVisualTargetLock,
    should_run_vision_only_tick,
)
from single_target_visual_alignment import (  # noqa: E402
    SingleTargetAlignmentConfig,
    SingleTargetVisualAlignment,
)


def _measurement(simple_id, x, y, confidence=0.9):
    return {
        "simple_id": simple_id,
        "center": [x, y],
        "bbox": [x - 50.0, y - 40.0, x + 50.0, y + 40.0],
        "confidence": confidence,
    }


def _lock(**overrides):
    values = {"lost_timeout_s": 1.0, "outside_confirm_frames": 3}
    values.update(overrides)
    return GimbalVisualTargetLock(GimbalVisualLockConfig(**values))


def test_acquires_detection_nearest_aim_and_holds_sort_track():
    lock = _lock()
    lock.start(17, 10.0)
    result = lock.update(
        frame_ts=10.1,
        now_ts=10.1,
        measurements=[
            _measurement(3, 400.0, 400.0),
            _measurement(7, 1290.0, 730.0),
        ],
    )
    assert lock.holds_sort_track(17)
    assert lock.simple_id == 7
    assert result["measurement"]["simple_id"] == 7


def test_multiple_detections_continue_original_simple_id():
    lock = _lock()
    lock.start(17, 10.0)
    lock.update(
        frame_ts=10.1,
        now_ts=10.1,
        measurements=[_measurement(7, 1290.0, 730.0)],
    )
    result = lock.update(
        frame_ts=10.2,
        now_ts=10.2,
        measurements=[
            _measurement(8, 1280.0, 720.0),
            _measurement(7, 1400.0, 760.0),
        ],
    )
    assert result["state"] == "LOCKED"
    assert result["measurement"]["simple_id"] == 7


def test_udp_independent_lock_survives_missing_sort_updates():
    lock = _lock()
    lock.start(17, 10.0)
    for index, x in enumerate((1300.0, 1350.0, 1400.0), start=1):
        result = lock.update(
            frame_ts=10.0 + index * 0.2,
            now_ts=10.0 + index * 0.2,
            measurements=[_measurement(7, x, 730.0)],
        )
        assert result["state"] == "LOCKED"
        assert lock.holds_sort_track(17)


def test_fresh_yolo_frame_runs_control_tick_without_udp():
    assert should_run_vision_only_tick(
        has_udp_packets=False,
        master_track_id=17,
        vision_frame_ts=10.2,
        last_control_frame_ts=10.1,
    )
    assert not should_run_vision_only_tick(
        has_udp_packets=False,
        master_track_id=17,
        vision_frame_ts=10.1,
        last_control_frame_ts=10.1,
    )


def test_no_udp_bbox_frames_continue_closed_loop_alignment():
    lock = _lock()
    alignment = SingleTargetVisualAlignment(SingleTargetAlignmentConfig())
    lock.start(17, 10.0)
    last_control_ts = 0.0
    stationary_ts = 1.0
    for index, x in enumerate((1480.0, 1400.0, 1320.0), start=1):
        frame_ts = 10.0 + index * 0.2
        assert should_run_vision_only_tick(
            has_udp_packets=False,
            master_track_id=17,
            vision_frame_ts=frame_ts,
            last_control_frame_ts=last_control_ts,
        )
        selected = lock.update(
            frame_ts=frame_ts,
            now_ts=frame_ts,
            measurements=[_measurement(7, x, 720.0)],
        )["measurement"]
        decision = alignment.update(
            frame_ts=frame_ts,
            master_track_id=17,
            measurement=selected,
            detection_count=1,
            gimbal_stationary=True,
            stationary_ts=stationary_ts,
        )
        assert decision["state"] == "ALIGNMENT_READY"
        assert decision["command_requested"] is True
        alignment.mark_command_sent(stationary_ts)
        alignment.update(
            frame_ts=frame_ts + 0.05,
            master_track_id=17,
            measurement=None,
            detection_count=0,
            gimbal_stationary=False,
            stationary_ts=stationary_ts,
        )
        stationary_ts += 1.0
        last_control_ts = frame_ts
    assert not should_run_vision_only_tick(
        has_udp_packets=False,
        master_track_id=None,
        vision_frame_ts=10.2,
        last_control_frame_ts=10.1,
    )


def test_missing_target_releases_only_after_timeout():
    lock = _lock(lost_timeout_s=1.0)
    lock.start(17, 10.0)
    lock.update(
        frame_ts=10.1,
        now_ts=10.1,
        measurements=[_measurement(7, 1290.0, 730.0)],
    )
    held = lock.update(frame_ts=10.5, now_ts=10.5, measurements=[])
    released = lock.update(frame_ts=11.2, now_ts=11.2, measurements=[])
    assert held["state"] == "MISSING_HOLD"
    assert held["release"] is False
    assert released["state"] == "MISSING_RELEASE"
    assert released["release"] is True


def test_visual_control_starts_after_detection_and_survives_missing_grace():
    lock = _lock(lost_timeout_s=5.0)
    lock.start(17, 10.0)
    assert lock.holds_sort_track(17)
    assert not lock.holds_visual_control(17)

    acquired = lock.update(
        frame_ts=10.1,
        now_ts=10.1,
        measurements=[_measurement(7, 1290.0, 730.0)],
    )
    assert acquired["state"] == "LOCKED"
    assert lock.holds_visual_control(17)

    missing = lock.update(frame_ts=14.9, now_ts=14.9, measurements=[])
    assert missing["state"] == "MISSING_HOLD"
    assert missing["release"] is False
    assert lock.holds_visual_control(17)

    released = lock.update(frame_ts=15.2, now_ts=15.2, measurements=[])
    assert released["state"] == "MISSING_RELEASE"
    assert released["release"] is True


def test_outside_area_requires_three_new_frames_before_release():
    lock = _lock(outside_confirm_frames=3)
    lock.start(17, 10.0)
    lock.update(
        frame_ts=10.1,
        now_ts=10.1,
        measurements=[_measurement(7, 1290.0, 730.0)],
    )
    states = []
    for index in range(3):
        result = lock.update(
            frame_ts=10.2 + index * 0.1,
            now_ts=10.2 + index * 0.1,
            measurements=[_measurement(7, 40.0, 730.0)],
        )
        states.append(result["state"])
    assert states == [
        "OUTSIDE_CONFIRMING",
        "OUTSIDE_CONFIRMING",
        "OUTSIDE_RELEASE",
    ]
