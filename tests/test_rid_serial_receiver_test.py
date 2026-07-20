import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CORE_DIR = PROJECT_ROOT / "core"
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from rid_tracking import RIDStreamParser  # noqa: E402
from tools_py import rid_serial_receiver_test as receiver  # noqa: E402


def _frame(payload):
    raw = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    length_value = len(raw) + 2
    return (
        b"\x08\x17\x30"
        + length_value.to_bytes(2, "little")
        + raw
        + b"\x3f\x55"
    )


def test_standalone_receiver_uses_production_parser_class():
    assert receiver.RIDStreamParser is RIDStreamParser


def test_standalone_summary_marks_zero_zero_position_invalid():
    summary = receiver._rid_summary({
        "UAVInfo": {
            "ID": "RID-ZERO",
            "Lon": 0,
            "Lat": 0,
        }
    })

    assert "rid_id=RID-ZERO" in summary
    assert "position_valid=False" in summary


def test_standalone_raw_diagnostic_matches_strict_parser_result():
    parser = receiver.RIDStreamParser()
    chunk = _frame({"UAVInfo": {"ID": "RID-TEST"}})
    before = receiver._counter_snapshot(parser)
    payloads = parser.feed(chunk)
    after = receiver._counter_snapshot(parser)
    delta = receiver._counter_delta(before, after)

    row = receiver._build_raw_log_row(
        123.5,
        chunk,
        parser,
        delta,
        list(parser.last_feed_frames),
        len(payloads),
    )

    assert payloads == [{"UAVInfo": {"ID": "RID-TEST"}}]
    assert row["parsed_payload_count"] == 1
    assert row["valid_frames_delta"] == 1
    assert row["decoded_frame_lengths"] == [len(chunk)]
    assert row["header_errors_delta"] == 0
    assert row["length_errors_delta"] == 0
    assert row["tail_errors_delta"] == 0
