from pathlib import Path
import sys


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

from GT06Z_gimbal import GT06ZGimbal  # noqa: E402


class _FakeSerial:
    is_open = True

    @staticmethod
    def reset_input_buffer():
        return None


def test_driver_quantizes_multi_decimal_targets_before_wire_and_state():
    gimbal = GT06ZGimbal("TEST")
    gimbal.ser = _FakeSerial()
    gimbal.min_interval = 0.0
    sent = []
    gimbal._send_frame = lambda cmd1, cmd2, value: sent.append(
        (cmd1, cmd2, value)
    )
    gimbal._read_specific_response = lambda **_kwargs: b"ack"

    status = gimbal.set_angles(
        elevation_deg=-0.8975,
        azimuth_deg=77.866,
        force=True,
    )

    assert status["el_sent"] is True
    assert status["az_sent"] is True
    assert sent == [
        (0x00, 0x4D, 9),
        (0x00, 0x4B, 779),
    ]
    assert gimbal.last_sent_el == -0.9
    assert gimbal.last_sent_az == 77.9
