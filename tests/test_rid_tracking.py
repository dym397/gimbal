import sys
import json
import math
from pathlib import Path


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
sys.path.insert(0, str(CORE_DIR))

from rid_tracking import (  # noqa: E402
    RIDSortAssociator,
    RIDStreamParser,
    RIDTrackManager,
    RIDTrajectoryRenderer,
    elevation_from_altitudes,
    elevation_from_relative_height,
    enrich_rid_tracks,
    horizontal_distance_and_bearing,
)


def _payload(rid_id="RID-A", lon=104.0, lat=30.0, timestamp=100):
    return {
        "MonitorInfo": {"Name": "XP-MRID-04", "Ch": 6},
        "UAVInfo": {
            "RID_Standard": "GB42590-2023",
            "ID": rid_id,
            "ID_Type": 1,
            "Type": 2,
            "Lon": lon,
            "Lat": lat,
            "AltGeo": 500,
            "Height": 50,
            "H_Speed": 0,
            "V_Speed": 0,
            "Sta": 2,
            "T_Stamp": timestamp,
        },
        "OperatorInfo": {"Lon": 104.1, "Lat": 30.1, "Height": 450},
    }


def _longitude_offset_m(east_m, latitude=30.0, base_longitude=104.0):
    return base_longitude + math.degrees(
        float(east_m) / (6_371_008.8 * math.cos(math.radians(latitude)))
    )


def _moving_payload(rid_id, east_m, timestamp, speed_mps, heading_deg):
    payload = _payload(
        rid_id,
        lon=_longitude_offset_m(east_m),
        lat=30.0,
        timestamp=timestamp,
    )
    payload["UAVInfo"]["H_Speed"] = speed_mps
    payload["UAVInfo"]["Trk"] = heading_deg
    return payload


def _rid_frame(value):
    payload = json.dumps(
        value,
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")
    length_value = len(payload) + 2
    return (
        b"\x08\x17\x30"
        + length_value.to_bytes(2, "little")
        + payload
        + b"\x3f\x55"
    )


def test_stream_parser_handles_binary_framing_split_and_sticky_packets():
    parser = RIDStreamParser()
    first = _rid_frame({"UAVInfo": {"ID": "A"}})
    second = _rid_frame({"UAVInfo": {"ID": "B"}})

    assert parser.feed(first[:1]) == []
    assert parser.feed(first[1:4]) == []
    assert parser.feed(first[4:13]) == []
    parsed = parser.feed(first[13:] + second)

    assert [item["UAVInfo"]["ID"] for item in parsed] == ["A", "B"]
    assert parser.valid_frames == 2
    assert parser.discarded_bytes == 0
    assert parser.header_errors == 0
    assert parser.length_errors == 0
    assert parser.tail_errors == 0
    assert parser.decode_errors == 0
    assert parser.last_feed_frames == [
        {
            "length_field": len(
                json.dumps(
                    {"UAVInfo": {"ID": "A"}}, separators=(",", ":")
                ).encode("utf-8")
            ) + 2,
            "payload_length": len(
                json.dumps(
                    {"UAVInfo": {"ID": "A"}}, separators=(",", ":")
                ).encode("utf-8")
            ),
            "frame_length": len(first),
        },
        {
            "length_field": len(
                json.dumps(
                    {"UAVInfo": {"ID": "B"}}, separators=(",", ":")
                ).encode("utf-8")
            ) + 2,
            "payload_length": len(
                json.dumps(
                    {"UAVInfo": {"ID": "B"}}, separators=(",", ":")
                ).encode("utf-8")
            ),
            "frame_length": len(second),
        },
    ]


def test_stream_parser_length_includes_two_length_bytes():
    parser = RIDStreamParser()
    payload = b'{"example":"1234567890123456789012345678"}'
    assert len(payload) == 42
    frame = b"\x08\x17\x30\x2c\x00" + payload + b"\x3f\x55"

    assert parser.feed(frame) == [{"example": "1234567890123456789012345678"}]
    assert parser.last_feed_frames == [{
        "length_field": 44,
        "payload_length": 42,
        "frame_length": 49,
    }]


def test_stream_parser_accepts_every_single_split_boundary():
    frame = _rid_frame({"UAVInfo": {"ID": "SPLIT"}})
    for split_at in range(len(frame) + 1):
        parser = RIDStreamParser()
        first_result = parser.feed(frame[:split_at])
        second_result = parser.feed(frame[split_at:])
        combined = first_result + second_result

        assert combined == [{"UAVInfo": {"ID": "SPLIT"}}]
        assert parser.valid_frames == 1
        assert parser.discarded_bytes == 0
        assert parser.header_errors == 0


def test_stream_parser_rejects_bad_length_and_resynchronizes():
    parser = RIDStreamParser()
    good = _rid_frame({"UAVInfo": {"ID": "GOOD"}})
    bad_length = b"\x08\x17\x30\x01\x00garbage"

    parsed = parser.feed(b"noise" + bad_length + good)

    assert parsed == [{"UAVInfo": {"ID": "GOOD"}}]
    assert parser.valid_frames == 1
    assert parser.header_errors >= 1
    assert parser.length_errors == 1
    assert parser.discarded_bytes > 0


def test_stream_parser_rejects_bad_tail_and_recovers_next_frame():
    parser = RIDStreamParser()
    bad = bytearray(_rid_frame({"UAVInfo": {"ID": "BAD"}}))
    bad[-2:] = b"XX"
    good = _rid_frame({"UAVInfo": {"ID": "GOOD"}})

    parsed = parser.feed(bytes(bad) + good)

    assert parsed == [{"UAVInfo": {"ID": "GOOD"}}]
    assert parser.valid_frames == 1
    assert parser.tail_errors == 1
    assert parser.discarded_bytes == len(bad)


def test_stream_parser_counts_utf8_and_json_errors_without_losing_sync():
    parser = RIDStreamParser()

    def framed_payload(payload):
        length_value = len(payload) + 2
        return (
            b"\x08\x17\x30"
            + length_value.to_bytes(2, "little")
            + payload
            + b"\x3f\x55"
        )

    data = (
        framed_payload(b"\xff")
        + framed_payload(b"{invalid}")
        + _rid_frame({"UAVInfo": {"ID": "GOOD"}})
    )
    parsed = parser.feed(data)

    assert parsed == [{"UAVInfo": {"ID": "GOOD"}}]
    assert parser.utf8_errors == 1
    assert parser.json_errors == 1
    assert parser.decode_errors == 2
    assert parser.valid_frames == 1


def test_rid_manager_uses_identity_key_and_stable_sequential_ui_ids():
    manager = RIDTrackManager(track_ttl_s=5.0)

    first = manager.update_payload(_payload("RID-A"), receive_ts=10.0)
    duplicate = manager.update_payload(_payload("RID-A"), receive_ts=10.2)
    second = manager.update_payload(_payload("RID-B"), receive_ts=10.3)

    assert first["track"]["ui_id"] == 1
    assert duplicate["track"]["ui_id"] == 1
    assert duplicate["duplicate"]
    assert duplicate["track"]["measurement_seq"] == first["track"]["measurement_seq"]
    assert duplicate["track"]["measurement_count"] == 1
    assert len(duplicate["track"]["measurement_history"]) == 1
    assert second["track"]["ui_id"] == 2
    assert len(manager.snapshot(now_ts=10.5)) == 2
    assert manager.snapshot(now_ts=16.0) == []


def test_rid_manager_deletes_after_five_minutes_and_assigns_a_new_ui_id():
    manager = RIDTrackManager(track_ttl_s=5.0, delete_after_s=300.0)

    first = manager.update_payload(_payload("RID-A"), receive_ts=10.0)
    assert first["track"]["ui_id"] == 1
    assert manager.snapshot(now_ts=309.999, include_stale=True)[0]["ui_id"] == 1

    # Exactly five minutes without any report permanently removes the track.
    assert manager.snapshot(now_ts=310.0, include_stale=True) == []
    returned = manager.update_payload(
        _payload("RID-A", timestamp=101),
        receive_ts=311.0,
    )
    assert returned["is_new"]
    assert returned["track"]["ui_id"] == 2

    # The receive path also prunes first when no periodic snapshot ran.
    direct_manager = RIDTrackManager(track_ttl_s=5.0, delete_after_s=300.0)
    direct_manager.update_payload(_payload("RID-B"), receive_ts=20.0)
    direct_return = direct_manager.update_payload(
        _payload("RID-B", timestamp=102),
        receive_ts=320.0,
    )
    assert direct_return["reason"] == "new_track_after_expiry"
    assert direct_return["track"]["ui_id"] == 2
    assert [item["ui_id"] for item in direct_return["expired_tracks"]] == [1]


def test_zero_zero_position_keeps_identity_but_never_creates_geometry():
    manager = RIDTrackManager(track_ttl_s=5.0)
    invalid_payload = _payload(
        "1581F6W8W255D0020XDB",
        lon=0,
        lat=0,
        timestamp=1546366335,
    )
    invalid_payload["UAVInfo"].update({
        "AltGeo": -1000,
        "Height": 0,
        "AltBaro": 579.5,
        "Trk": 361,
        "Sta": 1,
    })

    first = manager.update_payload(invalid_payload, receive_ts=10.0)
    duplicate = manager.update_payload(invalid_payload, receive_ts=10.2)

    assert first["accepted"]
    assert first["reason"] == "new_track_position_invalid_zero"
    assert first["track"]["position_valid"] is False
    assert first["track"]["measurement_seq"] == 0
    assert first["track"]["measurement_count"] == 0
    assert first["track"]["measurement_history"] == []
    assert duplicate["accepted"]
    assert duplicate["duplicate"]
    assert duplicate["reason"] == "duplicate_position_invalid_zero"
    assert duplicate["track"]["ui_id"] == first["track"]["ui_id"]
    assert duplicate["track"]["invalid_position_count"] == 2
    assert enrich_rid_tracks(
        manager.snapshot(now_ts=10.3), 30.0, 104.0
    ) == []

    valid_payload = _payload(
        "1581F6W8W255D0020XDB",
        lon=104.001,
        lat=30.0,
        timestamp=1546366336,
    )
    acquired = manager.update_payload(valid_payload, receive_ts=10.5)

    assert acquired["accepted"]
    assert acquired["reason"] == "position_acquired"
    assert acquired["track"]["position_valid"] is True
    assert acquired["track"]["ui_id"] == first["track"]["ui_id"]
    assert acquired["track"]["measurement_seq"] == 1
    assert acquired["track"]["measurement_count"] == 1
    assert len(acquired["track"]["measurement_history"]) == 1
    enriched = enrich_rid_tracks(
        manager.snapshot(now_ts=10.6), 30.0, 104.0
    )
    assert len(enriched) == 1
    assert 95.0 < enriched[0]["distance_m"] < 97.0


def test_only_the_combined_zero_zero_coordinate_is_the_invalid_sentinel():
    manager = RIDTrackManager()

    equator = manager.update_payload(
        _payload("RID-EQUATOR", lon=104.0, lat=0.0),
        receive_ts=10.0,
    )
    prime_meridian = manager.update_payload(
        _payload("RID-MERIDIAN", lon=0.0, lat=30.0),
        receive_ts=10.1,
    )

    assert equator["track"]["position_valid"] is True
    assert prime_meridian["track"]["position_valid"] is True


def test_rid_measurement_history_preserves_updates_between_main_snapshots():
    manager = RIDTrackManager(track_ttl_s=5.0)
    for seq in range(3):
        manager.update_payload(
            _payload("RID-A", lon=104.001 + seq * 0.0001, timestamp=100 + seq),
            receive_ts=10.0 + seq * 0.1,
        )

    enriched = enrich_rid_tracks(manager.snapshot(now_ts=10.3), 30.0, 104.0)
    assert len(enriched[0]["trajectory_history"]) == 3

    associator = RIDSortAssociator(history_seconds=5.0)
    associator.observe([], enriched, now_ts=10.3)
    assert len(associator.rid_histories[enriched[0]["key"]]) == 3


def test_rid_measurement_history_keeps_motion_and_altitude_inputs_for_renderer():
    manager = RIDTrackManager(track_ttl_s=5.0)
    payload = _moving_payload("RID-A", 10.0, 100, 8.5, 90.0)
    payload["UAVInfo"]["AltGeo"] = 512.5

    result = manager.update_payload(payload, receive_ts=10.0)
    sample = result["track"]["measurement_history"][0]

    assert sample["alt_geo"] == 512.5
    assert sample["horizontal_speed"] == 8.5
    assert sample["track_heading"] == 90.0


def test_rid_renderer_uses_endpoint_bounded_linear_interpolation():
    manager = RIDTrackManager(track_ttl_s=5.0)
    manager.update_payload(
        _moving_payload("RID-A", 0.0, 100, 10.0, 90.0),
        receive_ts=10.0,
    )
    manager.update_payload(
        _moving_payload("RID-A", 20.0, 101, 10.0, 90.0),
        receive_ts=12.0,
    )
    renderer = RIDTrajectoryRenderer(
        render_delay_s=0.8,
        max_prediction_s=0.5,
        display_tau_s=0.0,
    )

    rendered = renderer.render(manager.snapshot(now_ts=12.0), now_ts=12.0)
    enriched = enrich_rid_tracks(rendered, 30.0, 104.0)

    assert len(enriched) == 1
    assert enriched[0]["rid_render_mode"] == "interpolate"
    assert 11.8 < enriched[0]["distance_m"] < 12.2
    assert enriched[0]["distance_m"] < 20.0


def test_rid_renderer_caps_prediction_at_half_a_second_then_freezes():
    manager = RIDTrackManager(track_ttl_s=5.0)
    manager.update_payload(
        _moving_payload("RID-A", 0.0, 100, 10.0, 90.0),
        receive_ts=10.0,
    )
    manager.update_payload(
        _moving_payload("RID-A", 10.0, 101, 10.0, 90.0),
        receive_ts=11.0,
    )
    renderer = RIDTrajectoryRenderer(
        render_delay_s=0.8,
        max_prediction_s=0.5,
        display_tau_s=0.0,
    )
    renderer.render(manager.snapshot(now_ts=11.0), now_ts=11.0)

    rendered = renderer.render(manager.snapshot(now_ts=13.0), now_ts=13.0)
    enriched = enrich_rid_tracks(rendered, 30.0, 104.0)

    assert enriched[0]["rid_render_mode"] == "freeze"
    assert enriched[0]["rid_prediction_age_s"] == 0.5
    assert 14.8 < enriched[0]["distance_m"] < 15.2


def test_rid_renderer_resets_velocity_on_reversal_and_stays_between_endpoints():
    manager = RIDTrackManager(track_ttl_s=5.0)
    manager.update_payload(
        _moving_payload("RID-A", 0.0, 100, 20.0, 90.0),
        receive_ts=10.0,
    )
    manager.update_payload(
        _moving_payload("RID-A", 20.0, 101, 20.0, 90.0),
        receive_ts=11.0,
    )
    manager.update_payload(
        _moving_payload("RID-A", 10.0, 102, 10.0, 270.0),
        receive_ts=12.0,
    )
    renderer = RIDTrajectoryRenderer(
        render_delay_s=0.8,
        max_prediction_s=0.5,
        display_tau_s=0.0,
    )

    rendered = renderer.render(manager.snapshot(now_ts=12.5), now_ts=12.5)
    enriched = enrich_rid_tracks(rendered, 30.0, 104.0)

    assert enriched[0]["rid_filter_update_mode"] == "turn_reset"
    assert enriched[0]["rid_render_mode"] == "interpolate"
    assert 10.0 <= enriched[0]["distance_m"] <= 20.0


def test_rid_renderer_does_not_interpolate_across_a_long_reacquisition_gap():
    manager = RIDTrackManager(track_ttl_s=5.0, delete_after_s=300.0)
    manager.update_payload(
        _moving_payload("RID-A", 0.0, 100, 10.0, 90.0),
        receive_ts=10.0,
    )
    manager.update_payload(
        _moving_payload("RID-A", 100.0, 101, 0.0, 90.0),
        receive_ts=16.0,
    )
    renderer = RIDTrajectoryRenderer(
        render_delay_s=0.8,
        max_prediction_s=0.5,
        active_ttl_s=5.0,
        display_tau_s=0.0,
    )

    rendered = renderer.render(manager.snapshot(now_ts=16.1), now_ts=16.1)
    enriched = enrich_rid_tracks(rendered, 30.0, 104.0)

    assert enriched[0]["rid_filter_update_mode"] == "reacquired"
    assert enriched[0]["rid_render_mode"] == "hold_before_first"
    assert 99.8 < enriched[0]["distance_m"] < 100.2


def test_geodesy_returns_horizontal_distance_and_true_north_bearing():
    distance, bearing = horizontal_distance_and_bearing(
        30.0, 104.0, 30.0, 104.001
    )

    assert 95.0 < distance < 97.0
    assert 89.9 < bearing < 90.1


def test_rid_elevation_uses_relative_height_and_horizontal_distance():
    assert 44.9 < elevation_from_relative_height(100.0, 100.0) < 45.1
    assert elevation_from_relative_height(100.0, -1000.0) is None
    assert elevation_from_relative_height(100.0, None) is None


def test_rid_elevation_uses_altgeo_minus_station_ellipsoid_height():
    assert 44.9 < elevation_from_altitudes(100.0, 500.0, 400.0) < 45.1
    assert elevation_from_altitudes(100.0, -1000.0, 400.0) is None
    assert elevation_from_altitudes(100.0, 500.0, None) is None


def test_association_uses_azimuth_only_and_requires_distinct_updates():
    associator = RIDSortAssociator(
        max_az_error_deg=8.0,
        ambiguity_margin_deg=2.0,
        confirm_updates=3,
        hold_seconds=3.0,
        min_trajectory_points=1,
    )
    sort_tracks = [{
        "track_id": 7,
        "map_az": 101.0,
        "el": -25.0,
        "board": "BOARD_1",
        "cam": 4,
    }]
    rid = {
        "key": ("GB42590-2023", 1, "RID-A"),
        "rid_id": "RID-A",
        "ui_id": 1,
        "map_az": 100.0,
        "distance_m": 300.0,
        "age_s": 0.0,
        "update_seq": 1,
    }

    bindings, _ = associator.associate(sort_tracks, [rid], now_ts=10.0)
    assert bindings == {}
    # Re-running the main loop without a new RID update must not count as a
    # second confirmation, regardless of the deliberately unrelated elevation.
    bindings, _ = associator.associate(sort_tracks, [rid], now_ts=10.1)
    assert bindings == {}

    rid["update_seq"] = 2
    bindings, _ = associator.associate(sort_tracks, [rid], now_ts=10.2)
    assert bindings == {}
    rid["update_seq"] = 3
    bindings, diagnostics = associator.associate(sort_tracks, [rid], now_ts=10.3)

    assert bindings[7]["rid"]["rid_id"] == "RID-A"
    assert bindings[7]["state"] == "confirmed"
    assert diagnostics[0]["az_error_deg"] == 1.0


def test_association_rejects_close_azimuth_candidates_as_ambiguous():
    associator = RIDSortAssociator(
        max_az_error_deg=8.0,
        ambiguity_margin_deg=2.0,
        confirm_updates=1,
        min_trajectory_points=1,
    )
    sort_tracks = [{"track_id": 1, "map_az": 100.0}]
    rid_tracks = [
        {
            "key": ("GB42590-2023", 1, "RID-A"),
            "rid_id": "RID-A",
            "ui_id": 1,
            "map_az": 100.5,
            "distance_m": 100.0,
            "age_s": 0.0,
            "update_seq": 1,
        },
        {
            "key": ("GB42590-2023", 1, "RID-B"),
            "rid_id": "RID-B",
            "ui_id": 2,
            "map_az": 101.5,
            "distance_m": 200.0,
            "age_s": 0.0,
            "update_seq": 2,
        },
    ]

    bindings, diagnostics = associator.associate(
        sort_tracks, rid_tracks, now_ts=10.0
    )

    assert bindings == {}
    assert any(item["ambiguous"] for item in diagnostics)


def test_association_logs_pair_even_when_azimuth_exceeds_gate():
    associator = RIDSortAssociator(
        max_az_error_deg=8.0,
        ambiguity_margin_deg=2.0,
        confirm_updates=1,
        min_trajectory_points=1,
    )
    bindings, diagnostics = associator.associate(
        [{"track_id": 1, "map_az": 10.0}],
        [{
            "key": ("GB42590-2023", 1, "RID-A"),
            "rid_id": "RID-A",
            "ui_id": 1,
            "map_az": 30.0,
            "distance_m": 500.0,
            "age_s": 0.0,
            "update_seq": 1,
        }],
        now_ts=10.0,
    )

    assert bindings == {}
    assert diagnostics[0]["az_error_deg"] == 20.0
    assert diagnostics[0]["reason"] == "az_error_exceeds_gate"


def test_rid_only_history_waits_for_overlapping_sort_trajectory():
    associator = RIDSortAssociator(
        max_az_error_deg=8.0,
        ambiguity_margin_deg=1.0,
        confirm_updates=1,
        trajectory_points=10,
        min_trajectory_points=4,
        history_seconds=20.0,
    )
    rid = {
        "key": ("GB42590-2023", 1, "RID-A"),
        "rid_id": "RID-A",
        "ui_id": 1,
        "distance_m": 200.0,
        "age_s": 0.0,
        "update_seq": 0,
        "measurement_seq": 0,
    }

    # RID is visible for five updates before the detector creates a SORT track.
    for seq in range(1, 6):
        rid.update({
            "map_az": 100.0 + seq,
            "update_seq": seq,
            "measurement_seq": seq,
            "last_changed_ts": float(seq),
        })
        associator.observe([], [rid], now_ts=float(seq))

    rid.update({
        "map_az": 106.0,
        "update_seq": 6,
        "measurement_seq": 6,
        "last_changed_ts": 6.0,
    })
    bindings, diagnostics = associator.associate(
        [{"track_id": 7, "map_az": 106.5}], [rid], now_ts=6.0
    )
    assert bindings == {}
    assert diagnostics[0]["trajectory_samples"] == 1
    assert diagnostics[0]["reason"] == "trajectory_warmup_1/4"

    for seq in range(7, 10):
        rid.update({
            "map_az": 100.0 + seq,
            "update_seq": seq,
            "measurement_seq": seq,
            "last_changed_ts": float(seq),
        })
        bindings, diagnostics = associator.associate(
            [{"track_id": 7, "map_az": 100.5 + seq}],
            [rid],
            now_ts=float(seq),
        )

    assert diagnostics[0]["trajectory_samples"] == 4
    assert bindings[7]["rid"]["rid_id"] == "RID-A"


def test_trajectory_cost_prevents_last_point_nearest_neighbour_swap():
    associator = RIDSortAssociator(
        max_az_error_deg=15.0,
        max_curve_error_deg=15.0,
        ambiguity_margin_deg=0.0,
        confirm_updates=99,
        trajectory_points=10,
        min_trajectory_points=3,
        history_seconds=20.0,
    )
    rid_a = {
        "key": ("GB42590-2023", 1, "RID-A"),
        "rid_id": "RID-A",
        "ui_id": 1,
        "distance_m": 100.0,
        "age_s": 0.0,
    }
    rid_b = {
        "key": ("GB42590-2023", 1, "RID-B"),
        "rid_id": "RID-B",
        "ui_id": 2,
        "distance_m": 120.0,
        "age_s": 0.0,
    }
    curves = [
        (100.0, 112.0, 100.0, 112.0),
        (102.0, 111.5, 102.0, 111.5),
        (104.0, 111.0, 104.0, 111.0),
        (106.0, 110.8, 106.0, 110.8),
        (108.0, 110.5, 108.0, 110.5),
        # This last point alone favours S1->B and S2->A.
        (110.2, 109.8, 109.0, 110.3),
    ]

    diagnostics = []
    for seq, (sort_a, sort_b, map_a, map_b) in enumerate(curves, start=1):
        now_ts = 10.0 + seq
        rid_a.update({
            "map_az": map_a,
            "update_seq": seq * 2 - 1,
            "measurement_seq": seq * 2 - 1,
            "last_changed_ts": now_ts,
        })
        rid_b.update({
            "map_az": map_b,
            "update_seq": seq * 2,
            "measurement_seq": seq * 2,
            "last_changed_ts": now_ts,
        })
        _, diagnostics = associator.associate(
            [
                {"track_id": 1, "map_az": sort_a},
                {"track_id": 2, "map_az": sort_b},
            ],
            [rid_a, rid_b],
            now_ts=now_ts,
        )

    selected = {
        (item["sort_track_id"], item["rid_id"])
        for item in diagnostics
        if item["reason"].startswith("candidate_")
    }
    assert selected == {(1, "RID-A"), (2, "RID-B")}
    assert all(item["trajectory_samples"] == 6 for item in diagnostics)


def test_missing_sort_holds_rid_identity_against_immediate_reassignment():
    associator = RIDSortAssociator(
        max_az_error_deg=8.0,
        ambiguity_margin_deg=1.0,
        confirm_updates=1,
        min_trajectory_points=1,
        hold_seconds=3.0,
        history_seconds=10.0,
    )
    rid = {
        "key": ("GB42590-2023", 1, "RID-A"),
        "rid_id": "RID-A",
        "ui_id": 1,
        "map_az": 100.0,
        "distance_m": 200.0,
        "age_s": 0.0,
        "update_seq": 1,
        "measurement_seq": 1,
        "last_changed_ts": 10.0,
    }
    bindings, _ = associator.associate(
        [{"track_id": 1, "map_az": 100.5}], [rid], now_ts=10.0
    )
    assert bindings[1]["rid"]["rid_id"] == "RID-A"

    associator.associate([], [rid], now_ts=10.5)
    rid.update({
        "update_seq": 2,
        "measurement_seq": 2,
        "last_changed_ts": 11.0,
    })
    bindings, diagnostics = associator.associate(
        [{"track_id": 2, "map_az": 100.2}], [rid], now_ts=11.0
    )
    assert bindings == {}
    assert diagnostics[0]["reason"] == "reserved_by_confirmed_binding"


def test_enrich_rid_tracks_does_not_use_operator_coordinates():
    manager = RIDTrackManager()
    payload = _payload("RID-A", lon=104.001, lat=30.0)
    payload["UAVInfo"]["Height"] = 5
    manager.update_payload(payload, receive_ts=10.0)
    rid_tracks = manager.snapshot(now_ts=10.1)
    enriched = enrich_rid_tracks(
        rid_tracks,
        30.0,
        104.0,
        station_altitude=450.0,
    )

    assert 95.0 < enriched[0]["distance_m"] < 97.0
    assert 89.9 < enriched[0]["map_az"] < 90.1
    assert 27.0 < enriched[0]["elevation_deg"] < 28.0
    assert enriched[0]["vertical_delta_m"] == 50.0
