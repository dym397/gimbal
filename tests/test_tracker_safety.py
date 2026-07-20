import importlib.util
import math
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


def test_linux_rid_device_defaults_use_fixed_physical_usb_paths(monkeypatch):
    monkeypatch.setattr(tracking.os, "name", "posix")

    defaults = tracking._platform_serial_defaults()

    prefix = "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:"
    assert defaults["rid"] == f"{prefix}1.1:1.0-port0"
    assert defaults["gps"] == f"{prefix}1.2:1.0-port0"
    assert defaults["gimbal"] == f"{prefix}1.4:1.0-port0"


def _track_at_map_az(map_az):
    _, track = _tracker_with_track()
    track.state[0, 0] = (
        float(map_az) - tracking.DEVICE_HEADING_DEG
    ) % 360.0
    return track


def _rid_at_map_az(ui_id, map_az):
    return {
        "key": ("GB42590-2023", 1, f"RID-{ui_id}"),
        "ui_id": int(ui_id),
        "rid_id": f"RID-{ui_id}",
        "map_az": float(map_az),
        "distance_m": 100.0 + ui_id,
        "elevation_deg": 5.0 + ui_id,
    }


def test_stateless_ui_pair_selects_only_nearest_rid_when_sort_is_fewer():
    pairs = tracking.pair_ui_tracks_with_rid(
        [_track_at_map_az(100.0)],
        [_rid_at_map_az(1, 102.0), _rid_at_map_az(2, 220.0)],
    )

    assert len(pairs) == 1
    assert pairs[0]["rid"]["ui_id"] == 1
    assert pairs[0]["az_error_deg"] == 2.0


def test_stateless_ui_pair_emits_all_rid_when_counts_are_equal():
    pairs = tracking.pair_ui_tracks_with_rid(
        [_track_at_map_az(100.0), _track_at_map_az(200.0)],
        [_rid_at_map_az(1, 198.0), _rid_at_map_az(2, 101.0)],
    )

    assert len(pairs) == 2
    assert {item["rid"]["ui_id"] for item in pairs} == {1, 2}


def test_stateless_ui_pair_emits_all_rid_when_sort_is_more_numerous():
    pairs = tracking.pair_ui_tracks_with_rid(
        [
            _track_at_map_az(50.0),
            _track_at_map_az(150.0),
            _track_at_map_az(250.0),
        ],
        [_rid_at_map_az(1, 52.0), _rid_at_map_az(2, 248.0)],
    )

    assert len(pairs) == 2
    assert {item["rid"]["ui_id"] for item in pairs} == {1, 2}


def test_stateless_rid_ui_values_are_all_owned_by_rid():
    rid = _rid_at_map_az(7, 361.5)
    rid["distance_m"] = 432.1
    rid["elevation_deg"] = 12.25

    values = tracking.build_stateless_rid_ui_values(rid)

    assert values == {
        "target_id": 7,
        "azimuth": 1.5,
        "elevation": 12.25,
        "distance": 432.1,
    }


def test_stateless_rid_ui_values_mark_unavailable_elevation_as_nan():
    rid = _rid_at_map_az(8, 90.0)
    rid["elevation_deg"] = None

    values = tracking.build_stateless_rid_ui_values(rid)

    assert values["target_id"] == 8
    assert values["azimuth"] == 90.0
    assert values["distance"] == 108.0
    assert math.isnan(values["elevation"])


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
