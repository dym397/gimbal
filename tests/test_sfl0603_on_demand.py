import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

import main_tracking_v9 as runtime  # noqa: E402


if runtime.SFL0603ResponseTimeoutError is None:
    class _TestResponseTimeoutError(Exception):
        pass

    runtime.SFL0603ResponseTimeoutError = _TestResponseTimeoutError


class _FakeLaser:
    def __init__(self, distance_m=321.4, distances=None):
        self.distance_m = distance_m
        self.distances = list(distances or [])
        self.calls = []
        self.start_count = 0
        self.returned_distances = []

    def arm_ranging(self):
        self.calls.append("arm")

    def start_continuous(self, period_ms):
        self.calls.append(("start", period_ms))
        self.start_count += 1

    def read_measurement(self, timeout):
        self.calls.append(("read", timeout))
        time.sleep(0.005)
        distance_m = (
            self.distances.pop(0) if self.distances else self.distance_m
        )
        self.returned_distances.append(distance_m)
        return SimpleNamespace(
            valid=True,
            target_1_m=distance_m,
            flags=SimpleNamespace(raw=0x7C),
        )

    def stop_measurement(self):
        self.calls.append("stop")

    def disarm_ranging(self, stop=False):
        self.calls.append(("disarm", stop))

    def close(self):
        self.calls.append("close")


def _wait_until(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_no_alignment_request_keeps_sfl0603_in_standby():
    runtime.shared_state = runtime.SharedHardwareState()
    laser = _FakeLaser()
    stop_event = threading.Event()
    thread = threading.Thread(
        target=runtime.laser_reader_thread,
        args=(laser, stop_event),
        daemon=True,
    )
    thread.start()
    time.sleep(0.08)
    stop_event.set()
    thread.join(timeout=1.0)

    assert laser.start_count == 0
    assert "arm" not in laser.calls


def test_target_request_keeps_10hz_after_repeated_valid_results():
    runtime.shared_state = runtime.SharedHardwareState()
    with runtime.shared_state.lock:
        runtime.shared_state.is_stationary = True
        runtime.shared_state.laser_request_ts = 1234.5
        runtime.shared_state.laser_request_track_id = 17
        runtime.shared_state.laser_request_heartbeat_ts = time.time()

    laser = _FakeLaser(distance_m=456.7)
    stop_event = threading.Event()
    thread = threading.Thread(
        target=runtime.laser_reader_thread,
        args=(laser, stop_event),
        daemon=True,
    )
    thread.start()

    assert _wait_until(
        lambda: runtime.shared_state.raw_laser_dist is not None
    )
    assert _wait_until(lambda: len(laser.returned_distances) >= 3)
    assert "stop" not in laser.calls
    with runtime.shared_state.lock:
        runtime.shared_state.laser_request_ts = 0.0
        runtime.shared_state.laser_request_track_id = -1
        runtime.shared_state.laser_request_heartbeat_ts = 0.0
    assert _wait_until(lambda: "stop" in laser.calls)
    stop_event.set()
    thread.join(timeout=1.0)

    assert laser.start_count == 1
    assert ("start", 100) in laser.calls
    assert "stop" in laser.calls
    assert laser.calls.index("stop") < next(
        index
        for index, call in enumerate(laser.calls)
        if isinstance(call, tuple) and call[0] == "disarm"
    )
    with runtime.shared_state.lock:
        assert runtime.shared_state.laser_active is False
        assert runtime.shared_state.laser_status == "REQUEST_CANCELLED"
        assert runtime.shared_state.raw_laser_dist == 456.7
        assert runtime.shared_state.raw_laser_request_ts == 1234.5
        assert runtime.shared_state.raw_laser_track_id == 17


def test_all_valid_distances_keep_same_10hz_session_active():
    runtime.shared_state = runtime.SharedHardwareState()
    with runtime.shared_state.lock:
        runtime.shared_state.is_stationary = True
        runtime.shared_state.laser_request_ts = 1500.0
        runtime.shared_state.laser_request_track_id = 19
        runtime.shared_state.laser_request_heartbeat_ts = time.time()

    laser = _FakeLaser(distance_m=450.0, distances=[650.0, 450.0])
    stop_event = threading.Event()
    thread = threading.Thread(
        target=runtime.laser_reader_thread,
        args=(laser, stop_event),
        daemon=True,
    )
    thread.start()

    assert _wait_until(lambda: len(laser.returned_distances) >= 4)
    assert laser.returned_distances[:2] == [650.0, 450.0]
    assert "stop" not in laser.calls
    with runtime.shared_state.lock:
        assert runtime.shared_state.laser_active is True
        assert runtime.shared_state.laser_status == "RANGING_VALID_RETURN"
        assert runtime.shared_state.raw_laser_dist == 450.0
        runtime.shared_state.laser_request_ts = 0.0
        runtime.shared_state.laser_request_track_id = -1
        runtime.shared_state.laser_request_heartbeat_ts = 0.0
    assert _wait_until(lambda: "stop" in laser.calls)
    stop_event.set()
    thread.join(timeout=1.0)

    assert laser.start_count == 1
    with runtime.shared_state.lock:
        assert runtime.shared_state.laser_active is False
        assert runtime.shared_state.laser_status == "REQUEST_CANCELLED"
        assert runtime.shared_state.raw_laser_dist == 450.0


class _TimeoutLaser(_FakeLaser):
    def read_measurement(self, timeout):
        self.calls.append(("read_timeout", timeout))
        time.sleep(min(0.02, timeout))
        raise runtime.SFL0603ResponseTimeoutError("test timeout")


def test_gimbal_motion_does_not_stop_target_continuous_session():
    runtime.shared_state = runtime.SharedHardwareState()
    with runtime.shared_state.lock:
        runtime.shared_state.is_stationary = True
        runtime.shared_state.laser_request_ts = 2000.0
        runtime.shared_state.laser_request_track_id = 23
        runtime.shared_state.laser_request_heartbeat_ts = time.time()

    laser = _TimeoutLaser()
    stop_event = threading.Event()
    thread = threading.Thread(
        target=runtime.laser_reader_thread,
        args=(laser, stop_event),
        daemon=True,
    )
    thread.start()
    assert _wait_until(lambda: runtime.shared_state.laser_active)
    read_count_before_motion = len(laser.calls)

    with runtime.shared_state.lock:
        runtime.shared_state.is_stationary = False
    time.sleep(0.08)
    assert len(laser.calls) > read_count_before_motion
    assert "stop" not in laser.calls
    with runtime.shared_state.lock:
        runtime.shared_state.laser_request_ts = 0.0
        runtime.shared_state.laser_request_track_id = -1
        runtime.shared_state.laser_request_heartbeat_ts = 0.0
    assert _wait_until(lambda: "stop" in laser.calls)
    stop_event.set()
    thread.join(timeout=1.0)

    assert laser.start_count == 1
    assert runtime.shared_state.laser_active is False


def test_stale_alignment_heartbeat_stops_active_session():
    runtime.shared_state = runtime.SharedHardwareState()
    with runtime.shared_state.lock:
        runtime.shared_state.is_stationary = True
        runtime.shared_state.laser_request_ts = 2500.0
        runtime.shared_state.laser_request_track_id = 25
        runtime.shared_state.laser_request_heartbeat_ts = time.time()

    old_timeout = runtime.SFL0603_REQUEST_HEARTBEAT_TIMEOUT_SECONDS
    runtime.SFL0603_REQUEST_HEARTBEAT_TIMEOUT_SECONDS = 0.06
    laser = _TimeoutLaser()
    stop_event = threading.Event()
    thread = threading.Thread(
        target=runtime.laser_reader_thread,
        args=(laser, stop_event),
        daemon=True,
    )
    try:
        thread.start()
        assert _wait_until(lambda: runtime.shared_state.laser_active)
        assert _wait_until(lambda: "stop" in laser.calls)
    finally:
        stop_event.set()
        thread.join(timeout=1.0)
        runtime.SFL0603_REQUEST_HEARTBEAT_TIMEOUT_SECONDS = old_timeout

    assert laser.start_count == 1
    assert runtime.shared_state.laser_active is False
    assert runtime.shared_state.laser_status == "REQUEST_CANCELLED"


def test_no_valid_result_requests_scan_but_keeps_10hz_active():
    runtime.shared_state = runtime.SharedHardwareState()
    with runtime.shared_state.lock:
        runtime.shared_state.is_stationary = True
        runtime.shared_state.laser_request_ts = 3000.0
        runtime.shared_state.laser_request_track_id = 29
        runtime.shared_state.laser_request_heartbeat_ts = time.time()

    old_timeout = runtime.SFL0603_SESSION_TIMEOUT_SECONDS
    runtime.SFL0603_SESSION_TIMEOUT_SECONDS = 0.06
    laser = _TimeoutLaser()
    stop_event = threading.Event()
    thread = threading.Thread(
        target=runtime.laser_reader_thread,
        args=(laser, stop_event),
        daemon=True,
    )
    try:
        thread.start()
        assert _wait_until(
            lambda: runtime.shared_state.laser_no_return_ts > 0.0
        )
        assert runtime.shared_state.laser_active is True
        assert runtime.shared_state.laser_status == "RANGING_NO_VALID_RETURN"
        assert "stop" not in laser.calls
        with runtime.shared_state.lock:
            runtime.shared_state.laser_request_ts = 0.0
            runtime.shared_state.laser_request_track_id = -1
            runtime.shared_state.laser_request_heartbeat_ts = 0.0
        assert _wait_until(lambda: "stop" in laser.calls)
    finally:
        stop_event.set()
        thread.join(timeout=1.0)
        runtime.SFL0603_SESSION_TIMEOUT_SECONDS = old_timeout

    assert laser.start_count == 1
    assert runtime.shared_state.raw_laser_dist is None
    assert runtime.shared_state.laser_active is False
