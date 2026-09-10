import csv
import importlib.util
import math
import struct
import sys
import tempfile
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


def test_ui_track_is_exposed_only_after_thirteen_hits():
    tracker, track = _tracker_with_track()

    for hit_number in range(2, 13):
        now_t = 100.0 + (hit_number - 1) * 0.1
        tracker.update([_measurement(0.0)], dt=0.1, now_t=now_t)
        if track.hit_streak >= tracking.UI_TRACK_CONFIRM_HITS:
            track.ui_confirmed = True

    assert track.hit_streak == 12
    assert tracking.select_ui_tracks_for_display([track], 101.1) == []

    tracker.update([_measurement(0.0)], dt=0.1, now_t=101.2)
    if track.hit_streak >= tracking.UI_TRACK_CONFIRM_HITS:
        track.ui_confirmed = True

    assert track.hit_streak == 13
    assert tracking.select_ui_tracks_for_display([track], 101.2) == [track]


def test_ui_heading_packet_updates_map_heading_at_runtime():
    assert hasattr(tracking, "decode_device_heading_packet")
    assert hasattr(tracking, "set_device_heading_deg")
    assert hasattr(tracking, "get_device_heading_deg")

    packet = struct.pack("!Bf", 0x04, 271.25)
    decoded_heading = tracking.decode_device_heading_packet(packet)
    original_heading = tracking.get_device_heading_deg()
    try:
        old_heading, new_heading = tracking.set_device_heading_deg(
            decoded_heading
        )
        assert old_heading == original_heading
        assert new_heading == 271.25
        assert tracking.relative_to_map_azimuth(10.0) == 281.25
    finally:
        tracking.set_device_heading_deg(original_heading)


def test_ui_heading_packet_rejects_wrong_type_length_and_value():
    assert hasattr(tracking, "decode_device_heading_packet")

    invalid_packets = (
        struct.pack("!Bf", 0x02, 180.0),
        b"\x04\x00\x00\x00",
        struct.pack("!Bf", 0x04, 360.0),
        struct.pack("!Bf", 0x04, float("nan")),
    )
    for packet in invalid_packets:
        try:
            tracking.decode_device_heading_packet(packet)
        except ValueError:
            continue
        raise AssertionError(f"invalid packet accepted: {packet!r}")


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


def test_long_lost_track_can_reacquire_just_inside_four_degree_cap():
    tracker, original = _tracker_with_track()
    original.P[0, 0] = 10000.0
    original.P[1, 1] = 10000.0

    tracker.update([_measurement(3.99)], dt=0.25, now_t=101.2)

    assert tracking.TRACK_REACQUIRE_MAX_DEG == 4.0
    assert len(tracker.tracks) == 1
    assert original.last_update_ts == 101.2


def test_recent_track_can_still_match_inside_global_hard_cap():
    tracker, original = _tracker_with_track()
    original.P[0, 0] = 10000.0
    original.P[1, 1] = 10000.0

    tracker.update([_measurement(5.5)], dt=0.2, now_t=100.2)

    assert len(tracker.tracks) == 1
    assert original.last_update_ts == 100.2


def test_ui_freshness_is_stricter_than_internal_retention():
    _, track = _tracker_with_track()

    assert tracking.track_is_ui_fresh(track, 102.9)
    assert not tracking.track_is_ui_fresh(track, 103.1)
    assert track.lost_seconds(103.1) < tracking.TRACK_MAX_LOST_SECONDS


def test_ui_dropout_tolerance_outlives_control_lock_only():
    _, track = _tracker_with_track()
    track.confirmed = True
    track.ui_confirmed = True
    dropout_time = 101.7

    assert track.lost_seconds(dropout_time) > tracking.MAX_LOCK_LOST_SECONDS
    assert tracking.track_is_ui_fresh(track, dropout_time)
    assert tracking.select_ui_tracks_for_display(
        [track], dropout_time
    ) == [track]


def test_default_internal_retention_expires_at_twelve_seconds():
    tracker = tracking.MultiTargetTracker(
        max_lost_seconds=tracking.TRACK_MAX_LOST_SECONDS
    )
    tracker.update([_measurement(0.0)], dt=0.1, now_t=100.0)
    original = tracker.tracks[0]

    tracker.update([], dt=0.25, now_t=111.999)
    assert original in tracker.tracks

    tracker.update([], dt=0.25, now_t=112.0)
    assert original not in tracker.tracks


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


def test_ui_pair_exposes_only_the_initial_special_binding_as_changed():
    """Catch loss of the transition flag needed for RID filter resets."""
    associator = tracking.RIDFourPointAssociator()
    track = _track_at_map_az(100.0)
    special = _rid_at_map_az(1, 100.0, 1, 1.0)
    special["key"] = (
        "GB42590-2023", 1, "1581F6W8W255D0020XDB"
    )
    special["rid_id"] = "1581F6W8W255D0020XDB"

    first_pairs, _ = tracking.pair_ui_tracks_with_rid(
        [track], [special], associator, now_ts=1.0
    )
    second_pairs, _ = tracking.pair_ui_tracks_with_rid(
        [track], [special], associator, now_ts=1.1
    )

    assert first_pairs[0]["binding_changed"] is True
    assert second_pairs[0]["binding_changed"] is False


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


def test_ui_status_packet_is_not_sent_without_valid_distance():
    class CaptureSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, packet, address):
            self.sent.append((packet, address))

    sender = tracking.UISender("127.0.0.1", 9999)
    sender.sock.close()
    sender.sock = CaptureSocket()

    for distance in (None, "invalid", float("nan"), float("inf"), 0.0, -1.0):
        sent = sender.send_status(
            "BOARD_3",
            2,
            7,
            azimuth=123.0,
            elevation=4.0,
            distance=distance,
            threat_score=float("nan"),
        )
        assert sent is False
    assert sender.sock.sent == []


def test_ui_status_packet_is_sent_with_valid_distance():
    class CaptureSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, packet, address):
            self.sent.append((packet, address))

    sender = tracking.UISender("127.0.0.1", 9999)
    sender.sock.close()
    sender.sock = CaptureSocket()

    sent = sender.send_status(
        "BOARD_3",
        2,
        7,
        azimuth=123.0,
        elevation=4.0,
        distance=125.0,
        threat_score=50.0,
        replaced_target_id=23,
    )

    assert sent is True
    packet, address = sender.sock.sent[0]
    assert len(packet) == 34
    unpacked = struct.unpack("!BB8sIffffI", packet)
    assert address == ("127.0.0.1", 9999)
    assert unpacked[0] == 0x02
    assert unpacked[3] == 7
    assert unpacked[6] == 125.0
    assert unpacked[7] == 50.0
    assert unpacked[8] == 23


def test_ui_status_packet_defaults_replaced_target_id_to_zero():
    class CaptureSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, packet, address):
            self.sent.append((packet, address))

    sender = tracking.UISender("127.0.0.1", 9999)
    sender.sock.close()
    sender.sock = CaptureSocket()

    assert sender.send_status(
        "BOARD_3",
        2,
        7,
        azimuth=123.0,
        elevation=4.0,
        distance=125.0,
        threat_score=50.0,
    )
    packet, _ = sender.sock.sent[0]

    assert struct.unpack("!BB8sIffffI", packet)[8] == 0


def test_ui_receiver_parses_replaced_target_id():
    from tools_py import udp_ui_receiver

    packet = struct.pack(
        "!BB8sIffffI",
        0x02,
        2,
        b"BOARD_3",
        7,
        123.0,
        4.0,
        125.0,
        50.0,
        23,
    )

    parsed = udp_ui_receiver.parse_status_packet(packet)

    assert "target_id=7" in parsed
    assert "threat=50.000" in parsed
    assert "replaced_target_id=23" in parsed


def test_rid_ui_replacement_tracker_keeps_same_id_across_binding_gap():
    tracker = tracking.RIDUIReplacementTracker(repeat_count=3)
    rid_key = ("GB42590-2023", 1, "RID-A")

    assert tracker.observe_bindings({rid_key: 7}) == []
    assert tracker.peek_replacement(rid_key) == 0
    assert tracker.observe_bindings({}) == []
    assert tracker.observe_bindings({rid_key: 7}) == []
    assert tracker.peek_replacement(rid_key) == 0
    assert not tracker.is_superseded(7)


def test_rid_ui_replacement_tracker_repeats_old_id_three_times():
    tracker = tracking.RIDUIReplacementTracker(repeat_count=3)
    rid_key = ("GB42590-2023", 1, "RID-A")
    tracker.observe_bindings({rid_key: 7})

    assert tracker.observe_bindings({rid_key: 12}) == [{
        "rid_key": rid_key,
        "old_ui_id": 7,
        "new_ui_id": 12,
    }]
    assert tracker.is_superseded(7)
    assert tracker.peek_replacement(rid_key) == 7
    assert tracker.mark_sent(rid_key, 7) == 2
    assert tracker.mark_sent(rid_key, 7) == 1
    assert tracker.mark_sent(rid_key, 7) == 0
    assert tracker.peek_replacement(rid_key) == 0
    assert tracker.is_superseded(7)


def test_rid_ui_replacement_tracker_isolates_rids_and_queues_switches_fifo():
    tracker = tracking.RIDUIReplacementTracker(repeat_count=3)
    rid_a = ("GB42590-2023", 1, "RID-A")
    rid_b = ("GB42590-2023", 1, "RID-B")
    tracker.observe_bindings({rid_a: 7, rid_b: 20})
    tracker.observe_bindings({rid_a: 12, rid_b: 20})
    tracker.observe_bindings({rid_a: 18, rid_b: 25})

    assert tracker.peek_replacement(rid_a) == 7
    assert tracker.peek_replacement(rid_b) == 20
    for expected_remaining in (2, 1, 0):
        assert tracker.mark_sent(rid_a, 7) == expected_remaining
    assert tracker.peek_replacement(rid_a) == 12
    assert tracker.peek_replacement(rid_b) == 20


def test_current_rid_binding_cancels_deletion_and_supersession_for_ui_id():
    tracker = tracking.RIDUIReplacementTracker(repeat_count=3)
    rid_a = ("GB42590-2023", 1, "RID-A")
    rid_b = ("GB42590-2023", 1, "RID-B")
    tracker.observe_bindings({rid_a: 7, rid_b: 20})
    tracker.observe_bindings({rid_a: 12, rid_b: 20})
    assert tracker.peek_replacement(rid_a) == 7
    assert tracker.is_superseded(7)

    tracker.observe_bindings({rid_a: 12, rid_b: 7})

    assert tracker.peek_replacement(rid_a) == 0
    assert not tracker.is_superseded(7)
    assert tracker.peek_replacement(rid_b) == 20


def test_rid_ui_replacement_tracker_forget_removes_all_rid_state():
    tracker = tracking.RIDUIReplacementTracker(repeat_count=3)
    rid_key = ("GB42590-2023", 1, "RID-A")
    tracker.observe_bindings({rid_key: 7})
    tracker.observe_bindings({rid_key: 12})

    assert tracker.forget(rid_key) is True
    assert tracker.peek_replacement(rid_key) == 0
    assert not tracker.is_superseded(7)
    assert tracker.forget(rid_key) is False


def test_vision_single_target_binds_without_pixel_distance_gate():
    results, unmatched_tracks, unmatched_detections = (
        tracking.associate_vision_measurements_nearest(
            [{"track_id": 7, "center": (1280.0, 720.0)}],
            [{
                "center": (5000.0, 4000.0),
                "distance": 123.0,
                "distance_valid": True,
            }],
        )
    )

    assert set(results) == {7}
    assert results[7]["distance"] == 123.0
    assert results[7]["association_error_px"] > 3000.0
    assert (
        results[7]["association_policy"]
        == "hungarian_compensated_y_no_gate"
    )
    assert results[7]["association_cost_y_px"] == 3280.0
    assert unmatched_tracks == []
    assert unmatched_detections == []


def test_vision_multi_target_uses_global_one_to_one_y_assignment():
    diagnostics = []
    results, unmatched_tracks, unmatched_detections = (
        tracking.associate_vision_measurements_nearest(
            [
                {"track_id": 1, "center": (0.0, 0.0)},
                {"track_id": 2, "center": (100.0, 100.0)},
            ],
            [
                {"center": (10.0, 90.0), "label": "bottom"},
                {"center": (90.0, 10.0), "label": "top"},
            ],
            candidate_diagnostics=diagnostics,
        )
    )

    assert results[1]["label"] == "top"
    assert results[2]["label"] == "bottom"
    assert unmatched_tracks == []
    assert unmatched_detections == []
    assert len(diagnostics) == 4
    selected = {
        (item["track_id"], item["measurement_index"])
        for item in diagnostics
        if item["selected"]
    }
    assert selected == {(1, 1), (2, 0)}
    assert all(
        item["association_error_px"] >= 0.0 for item in diagnostics
    )
    assert all(
        item["association_cost_y_px"] >= 0.0 for item in diagnostics
    )


def test_vision_y_assignment_uses_zero_global_compensation():
    diagnostics = []
    results, unmatched_tracks, unmatched_detections = (
        tracking.associate_vision_measurements_nearest(
            [
                {
                    "track_id": 12,
                    "logic_id": 12,
                    "center": (1280.0, 700.0),
                },
                {
                    "track_id": 13,
                    "logic_id": 13,
                    "center": (1280.0, 800.0),
                },
            ],
            [
                {"center": (2400.0, 746.5), "label": "logic12"},
                {"center": (100.0, 1106.5), "label": "logic13"},
            ],
            candidate_diagnostics=diagnostics,
        )
    )

    assert results[12]["label"] == "logic12"
    assert results[13]["label"] == "logic13"
    assert results[12]["sort_y_compensation_px"] == 0.0
    assert results[13]["sort_y_compensation_px"] == 0.0
    assert results[12]["association_cost_y_px"] == 46.5
    assert results[13]["association_cost_y_px"] == 306.5
    assert tracking.gimbal_vision_y_compensation_px(99) == 0.0
    assert tracking.gimbal_vision_y_compensation_px(None) == 0.0
    assert unmatched_tracks == []
    assert unmatched_detections == []


def test_camera_theta_offsets_skip_manually_calibrated_layer3():
    center_x = tracking.IMG_W / 2.0
    center_y = tracking.IMG_H / 2.0

    manual_az, manual_el = tracking.calculate_angles(
        13, center_x, center_y, tracking.DEVICE_THETA[13]
    )
    untested_az, untested_el = tracking.calculate_angles(
        8, center_x, center_y, tracking.DEVICE_THETA[8]
    )

    assert manual_az == 359.0
    assert manual_el == 13.0
    assert abs(untested_az - 358.0759) < 1e-9
    assert abs(untested_el - 3.65) < 1e-9


def test_field_logger_writes_dedicated_replay_logs():
    with tempfile.TemporaryDirectory() as log_dir:
        logger = tracking.FieldLogger(log_dir)
        logger.write_vision_association({
            "timestamp": "1.0",
            "event": "CANDIDATE",
            "vision_frame_ts": "0.9",
            "sort_track_id": 7,
            "simple_id": 3,
            "association_error_px": "12.5",
            "selected": 1,
        })
        logger.write_distance_arbitration({
            "timestamp": "1.0",
            "track_id": 7,
            "selected_family": "vision",
            "selected_distance": "123.0",
            "selected_valid": 1,
        })
        previous_logger = tracking.FIELD_LOGGER
        tracking.FIELD_LOGGER = logger
        try:
            tracking.field_log_event({
                "timestamp": "1.0",
                "event": "GIMBAL_VISION_ASSOC",
            })
        finally:
            tracking.FIELD_LOGGER = previous_logger
        logger.close()

        vision_path = next(
            Path(log_dir).glob("vision_association_*.csv")
        )
        distance_path = next(
            Path(log_dir).glob("distance_arbitration_*.csv")
        )
        events_path = next(Path(log_dir).glob("events_*.csv"))
        with vision_path.open(encoding="utf-8", newline="") as stream:
            vision_rows = list(csv.DictReader(stream))
        with distance_path.open(encoding="utf-8", newline="") as stream:
            distance_rows = list(csv.DictReader(stream))
        with events_path.open(encoding="utf-8", newline="") as stream:
            event_rows = list(csv.DictReader(stream))

        assert vision_rows[0]["event"] == "CANDIDATE"
        assert vision_rows[0]["sort_track_id"] == "7"
        assert vision_rows[0]["simple_id"] == "3"
        assert vision_rows[0]["selected"] == "1"
        assert distance_rows[0]["selected_family"] == "vision"
        assert distance_rows[0]["selected_distance"] == "123.0"
        assert event_rows == []


def _fresh_rid_pair(track, distance=400.0, now_t=100.0, sequence=4):
    rid = _rid_at_map_az(1, 100.0, sequence, now_t)
    rid.update({
        "distance_m": float(distance),
        "last_receive_ts": float(now_t),
        "age_s": 0.0,
    })
    return {
        "sort_track": track,
        "rid": rid,
        "az_error_deg": 3.1,
    }


def _vision_candidate(distance=120.0, frame_ts=100.0):
    return {
        "distance": float(distance),
        "distance_valid": True,
        "distance_source": "mlp_warmup",
        "frame_ts": float(frame_ts),
        "safe": True,
    }


def test_final_distance_candidate_prefers_fresh_rid_over_vision():
    track = _track_at_map_az(103.1)

    candidate = tracking.choose_final_distance_candidate(
        track,
        _fresh_rid_pair(track, distance=400.0),
        _vision_candidate(distance=120.0),
        gimbal_target_id=track.id,
        curr_time=100.0,
    )

    assert candidate["valid"] is True
    assert candidate["source_family"] == "rid"
    assert candidate["distance"] == 400.0


def test_only_changed_special_binding_requests_distance_filter_reset():
    """Catch filter-reset behavior leaking into ordinary RID candidates."""
    track = _track_at_map_az(103.1)
    special_pair = _fresh_rid_pair(track, distance=400.0)
    special_pair.update({
        "binding_state": "forced_nearest",
        "binding_changed": True,
    })

    special_candidate = tracking.choose_final_distance_candidate(
        track,
        special_pair,
        _vision_candidate(distance=120.0),
        gimbal_target_id=track.id,
        curr_time=100.0,
    )
    normal_candidate = tracking.choose_final_distance_candidate(
        track,
        _fresh_rid_pair(track, distance=400.0),
        _vision_candidate(distance=120.0),
        gimbal_target_id=track.id,
        curr_time=100.0,
    )

    assert special_candidate["force_filter_reset"] is True
    assert normal_candidate["force_filter_reset"] is False


def test_final_distance_candidate_uses_vision_when_rid_is_stale():
    track = _track_at_map_az(103.1)
    rid_pair = _fresh_rid_pair(track, distance=400.0, now_t=93.9)

    candidate = tracking.choose_final_distance_candidate(
        track,
        rid_pair,
        _vision_candidate(distance=120.0, frame_ts=100.0),
        gimbal_target_id=track.id,
        curr_time=100.0,
    )

    assert candidate["valid"] is True
    assert candidate["source_family"] == "vision"
    assert candidate["distance"] == 120.0


def test_vision_candidate_only_applies_to_current_gimbal_target():
    track = _track_at_map_az(103.1)

    candidate = tracking.choose_final_distance_candidate(
        track,
        None,
        _vision_candidate(distance=120.0),
        gimbal_target_id=track.id + 1,
        curr_time=100.0,
    )

    assert candidate["valid"] is False
    assert math.isnan(candidate["distance"])


def test_distance_filter_resets_when_source_changes():
    track = _track_at_map_az(103.1)
    assert track.set_final_distance(100.0, 100.0, "mlp_warmup")
    track.dist_state[1, 0] = 12.0

    assert track.set_final_distance(400.0, 100.1, "rid_gps")

    assert track.dist_source == "rid_gps"
    assert track.dist_state[0, 0] == 400.0
    assert track.dist_state[1, 0] == 0.0
    assert track.dist_P[0, 0] == 10.0


def test_distance_filter_can_reset_when_special_rid_binding_changes():
    """Catch carrying one special RID's state into another RID binding."""
    track = _track_at_map_az(103.1)
    assert track.set_final_distance(100.0, 100.0, "rid_gps")
    track.dist_state[1, 0] = 12.0

    assert track.set_final_distance(
        400.0,
        100.1,
        "rid_gps",
        force_reset=True,
    )

    assert track.dist_source == "rid_gps"
    assert track.dist_state[0, 0] == 400.0
    assert track.dist_state[1, 0] == 0.0
    assert track.dist_P[0, 0] == 10.0


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
