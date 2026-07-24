import importlib.util
import math
import struct
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


def _rid_at_map_az(ui_id, map_az, measurement_seq=1, timestamp=1.0):
    return {
        "key": ("GB42590-2023", 1, f"RID-{ui_id}"),
        "ui_id": int(ui_id),
        "rid_id": f"RID-{ui_id}",
        "map_az": float(map_az),
        "distance_m": 100.0 + ui_id,
        "elevation_deg": 5.0 + ui_id,
        "age_s": 0.0,
        "update_seq": int(measurement_seq),
        "measurement_seq": int(measurement_seq),
        "rid_render_timestamp": float(timestamp),
    }


def test_ui_pair_waits_for_four_distinct_rid_measurements():
    associator = tracking.RIDFourPointAssociator()
    track = _track_at_map_az(103.1)
    pairs = []
    diagnostics = []
    for sequence in range(1, 5):
        pairs, diagnostics = tracking.pair_ui_tracks_with_rid(
            [track],
            [_rid_at_map_az(1, 100.0, sequence, float(sequence))],
            associator,
            now_ts=float(sequence),
        )
        if sequence < 4:
            assert pairs == []

    assert len(pairs) == 1
    assert pairs[0]["rid"]["ui_id"] == 1
    assert diagnostics[0]["reason"] == "four_point_gate_pass_selected"


def test_ui_pair_does_not_force_a_wrong_bias_match():
    associator = tracking.RIDFourPointAssociator()
    track = _track_at_map_az(120.0)
    pairs = []
    for sequence in range(1, 5):
        pairs, diagnostics = tracking.pair_ui_tracks_with_rid(
            [track],
            [_rid_at_map_az(1, 100.0, sequence, float(sequence))],
            associator,
            now_ts=float(sequence),
        )

    assert pairs == []
    assert diagnostics[0]["reason"] == "four_point_bias_error_exceeds_gate"


def test_unmatched_ui_sort_track_is_kept_without_rid():
    first = _track_at_map_az(103.1)
    second = _track_at_map_az(203.1)
    second.id = first.id + 1
    matched = [{
        "sort_track": first,
        "rid": _rid_at_map_az(1, 100.0, 4, 4.0),
        "az_error_deg": 3.1,
    }]

    completed = tracking.complete_ui_fusion_pairs(
        [first, second], matched
    )

    assert [item["sort_track"] for item in completed] == [first, second]
    assert completed[0]["rid"]["rid_id"] == "RID-1"
    assert completed[1]["rid"] is None
    assert math.isnan(completed[1]["az_error_deg"])


def test_rid_matched_ui_values_use_only_rid_distance():
    track = _track_at_map_az(123.5)
    track.state[1, 0] = -4.25
    rid = _rid_at_map_az(7, 361.5)
    rid["distance_m"] = 432.1
    rid["elevation_deg"] = 12.25

    values = tracking.build_rid_matched_ui_values(track, rid, sort_ui_id=29)

    assert values == {
        "target_id": 29,
        "azimuth": 123.5,
        "elevation": -4.25,
        "distance": 432.1,
    }


def test_ui_threat_score_uses_requested_distance_boundaries():
    assert tracking.ui_threat_score_from_distance(99.999) == 100.0
    assert tracking.ui_threat_score_from_distance(100.0) == 50.0
    assert tracking.ui_threat_score_from_distance(300.0) == 50.0
    assert tracking.ui_threat_score_from_distance(300.001) == 0.0
    assert math.isnan(tracking.ui_threat_score_from_distance(float("nan")))


def test_ui_status_packet_contains_nan_distance_and_nan_threat():
    class CaptureSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, packet, address):
            self.sent.append((packet, address))

    sender = tracking.UISender("127.0.0.1", 9999)
    sender.sock.close()
    sender.sock = CaptureSocket()

    sender.send_status(
        "BOARD_3",
        2,
        7,
        azimuth=123.0,
        elevation=4.0,
        distance=float("nan"),
        threat_score=tracking.ui_threat_score_from_distance(float("nan")),
    )

    packet, address = sender.sock.sent[0]
    unpacked = struct.unpack("!BB8sIffff", packet)
    assert address == ("127.0.0.1", 9999)
    assert unpacked[0] == 0x02
    assert unpacked[3] == 7
    assert math.isnan(unpacked[6])
    assert math.isnan(unpacked[7])


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
