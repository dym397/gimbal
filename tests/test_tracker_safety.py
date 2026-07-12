import importlib.util
import sys
import types
from pathlib import Path


# The repository root contains a compatibility launcher that intentionally
# executes the hardware program when imported. Load the real core module by
# absolute path so tests can never start serial/network threads.
_TEST_PATH = Path(__file__).resolve()
_CORE_PATH_CANDIDATES = [
    _TEST_PATH.parents[1] / "core" / "main_tracking_v9.py",
    _TEST_PATH.with_name("main_tracking_v9.py"),
]
_CORE_PATH = next(path for path in _CORE_PATH_CANDIDATES if path.exists())
sys.path.insert(0, str(_CORE_PATH.parent))
_previous_vision_module = sys.modules.get("gimbal_vision_ranging")
_vision_stub = types.ModuleType("gimbal_vision_ranging")
_vision_stub.GimbalVisionRangingService = None
sys.modules["gimbal_vision_ranging"] = _vision_stub
try:
    _spec = importlib.util.spec_from_file_location("tracking_core_under_test", _CORE_PATH)
    tracking = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(tracking)
finally:
    if _previous_vision_module is None:
        sys.modules.pop("gimbal_vision_ranging", None)
    else:
        sys.modules["gimbal_vision_ranging"] = _previous_vision_module


def _measurement(az, board="BOARD_3", cam=2, logic_id=13, source_ts=None):
    return {
        "az": float(az),
        "el": 20.0,
        "mono_dist": None,
        "board": board,
        "cam": cam,
        "logic_id": logic_id,
        "source_ts": source_ts,
        "source_boards": [board],
        "source_cams": [str(cam)],
        "source_logic_ids": [str(logic_id)],
    }


def _tracker_with_track(now_t=100.0):
    tracker = tracking.MultiTargetTracker(max_lost_seconds=5.0)
    tracker.update([_measurement(0.0)], dt=0.1, now_t=now_t)
    return tracker, tracker.tracks[0]


def test_track_remembers_its_latest_detection_source():
    tracker, track = _tracker_with_track()

    assert tracking.track_ui_source(track, "fallback", 9) == ("BOARD_3", 2)

    tracker.update(
        [_measurement(0.2, board="BOARD_3", cam=3, logic_id=14)],
        dt=0.1,
        now_t=100.1,
    )

    assert len(tracker.tracks) == 1
    assert tracking.track_ui_source(track, "fallback", 9) == ("BOARD_3", 3)
    assert track.last_source_logic_id == 14


def test_covariance_growth_cannot_accept_ten_degree_jump():
    tracker, original = _tracker_with_track()
    original.P[0, 0] = 10000.0
    original.P[1, 1] = 10000.0

    tracker.update([_measurement(10.0)], dt=0.2, now_t=100.2)

    assert len(tracker.tracks) == 2
    assert original.last_update_ts == 100.0


def test_long_lost_track_uses_strict_reacquire_gate():
    tracker, original = _tracker_with_track()
    original.P[0, 0] = 10000.0
    original.P[1, 1] = 10000.0

    tracker.update([_measurement(4.0)], dt=0.25, now_t=101.2)

    assert len(tracker.tracks) == 2
    assert original.last_update_ts == 100.0


def test_recent_track_can_still_match_inside_global_hard_cap():
    tracker, original = _tracker_with_track()
    original.P[0, 0] = 10000.0
    original.P[1, 1] = 10000.0

    tracker.update([_measurement(5.5)], dt=0.2, now_t=100.2)

    assert len(tracker.tracks) == 1
    assert original.last_update_ts == 100.2


def test_ui_freshness_is_stricter_than_internal_retention():
    _, track = _tracker_with_track()

    assert tracking.track_is_ui_fresh(track, 100.9)
    assert not tracking.track_is_ui_fresh(track, 101.1)
    assert track.lost_seconds(101.1) < tracking.TRACK_MAX_LOST_SECONDS


def test_fused_measurement_preserves_all_sources_and_primary_source():
    fused, groups = tracking.fuse_measurements_by_angle(
        [
            _measurement(
                10.0, board="BOARD_3", cam=2, logic_id=13, source_ts=100.0
            ),
            _measurement(
                10.2, board="BOARD_3", cam=3, logic_id=14, source_ts=100.1
            ),
        ],
        threshold_deg=0.5,
    )

    assert len(groups) == 1
    assert len(fused) == 1
    assert fused[0]["board"] == "BOARD_3"
    assert fused[0]["cam"] == 3
    assert fused[0]["source_cams"] == ["2", "3"]
    assert fused[0]["source_logic_ids"] == ["13", "14"]
