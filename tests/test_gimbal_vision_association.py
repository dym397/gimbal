from pathlib import Path
import sys

import numpy as np


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
if str(CORE_DIR) not in sys.path:
    sys.path.insert(0, str(CORE_DIR))

import gimbal_vision_ranging as vision_module
from gimbal_vision_ranging import GimbalVisionRangingService


def _service_for_association():
    service = GimbalVisionRangingService.__new__(
        GimbalVisionRangingService
    )
    service.association_max_px = 260.0
    service.association_ambiguity_margin_px = 30.0
    service.simple_mode = False
    service.track_states = {}
    return service


def _detection(center_x, center_y, confidence=0.9):
    return {
        "center": np.array(
            [center_x, center_y], dtype=np.float32
        ),
        "box": np.array(
            [
                center_x - 10,
                center_y - 5,
                center_x + 10,
                center_y + 5,
            ],
            dtype=np.float32,
        ),
        "confidence": confidence,
        "class_id": 0,
        "sharpness": 100.0,
    }


def test_detection_only_init_never_constructs_distance_runtime(monkeypatch):
    class FakeCamera:
        def __init__(self, *args, **kwargs):
            pass

    class FakeDetector:
        def __init__(self, *args, **kwargs):
            self.backend = "rknn"

    class ForbiddenDistanceRuntime:
        def __init__(self):
            raise AssertionError("distance runtime must not be constructed")

    monkeypatch.setattr(vision_module, "_LatestFrameCamera", FakeCamera)
    monkeypatch.setattr(vision_module, "_CpuYoloDetector", FakeDetector)
    monkeypatch.setattr(
        vision_module, "_DistanceRuntime", ForbiddenDistanceRuntime
    )

    service = GimbalVisionRangingService(
        camera_source="unused",
        detection_only=True,
    )

    assert service.detection_only is True
    assert service.simple_mode is True
    assert service._spare_distance_runtime is None


def test_detection_only_simple_result_contains_yolo_but_no_distance():
    service = GimbalVisionRangingService.__new__(
        GimbalVisionRangingService
    )
    service.detection_only = True
    service._spare_distance_runtime = None
    service.simple_track_states = {}

    result = service._simple_measurement_result(
        simple_id=3,
        detection=_detection(1280.0, 720.0),
        association_error_px=0.0,
        frame_ts=10.0,
    )

    assert result["state"] == "DETECTED"
    assert result["simple_id"] == 3
    assert result["center"] == [1280.0, 720.0]
    assert result["distance_valid"] is False
    assert result["distance_source"] == "none"
    assert result["reason"] == "YOLO_DETECTION_ONLY"
    assert service.simple_track_states[3]["distance"] is None


def test_detection_only_frame_skips_hungarian_association():
    service = GimbalVisionRangingService.__new__(
        GimbalVisionRangingService
    )
    service.detection_only = True
    service.simple_mode = True
    service._spare_distance_runtime = None
    service.simple_track_states = {}
    service.simple_next_track_id = 1
    service.track_states = {}
    service.track_state_ttl_s = 2.0
    service.simple_association_max_px = 3000.0
    service._build_simple_roi_origins = lambda: [(0, 0)]
    service._detect_all = lambda frame, origins: (
        [_detection(1280.0, 720.0)],
        100.0,
        1,
    )
    service._associate_simple_detections = lambda *args, **kwargs: (
        (_ for _ in ()).throw(
            AssertionError("Hungarian association must not run")
        )
    )

    result = service._process_frame(
        np.zeros((1440, 2560, 3), dtype=np.uint8),
        frame_ts=20.0,
        master_track_id=11,
        predictions={},
    )

    assert result["detection_count"] == 1
    assert result["simple_measurements"][0]["simple_id"] == 1
    assert result["simple_measurements"][0]["state"] == "DETECTED"


def test_clear_targets_bind_to_expected_sort_ids():
    service = _service_for_association()
    predictions = {
        11: np.array([100.0, 100.0], dtype=np.float32),
        22: np.array([300.0, 100.0], dtype=np.float32),
    }
    detections = [
        _detection(105.0, 102.0),
        _detection(296.0, 99.0),
    ]

    matches, unmatched, ambiguous = service._associate(
        predictions, detections, frame_ts=1.0
    )

    assert set(matches) == {11, 22}
    assert np.allclose(matches[11][0]["center"], [105.0, 102.0])
    assert np.allclose(matches[22][0]["center"], [296.0, 99.0])
    assert unmatched == set()
    assert ambiguous == set()


def test_close_crossing_targets_are_rejected_instead_of_id_swapped():
    service = _service_for_association()
    predictions = {
        11: np.array([100.0, 100.0], dtype=np.float32),
        22: np.array([120.0, 100.0], dtype=np.float32),
    }
    detections = [
        _detection(109.0, 100.0),
        _detection(111.0, 100.0),
    ]

    matches, unmatched, ambiguous = service._associate(
        predictions, detections, frame_ts=1.0
    )

    assert matches == {}
    assert unmatched == {0, 1}
    assert ambiguous == {11, 22}


def test_detection_outside_gate_is_never_bound():
    service = _service_for_association()
    predictions = {
        11: np.array([100.0, 100.0], dtype=np.float32),
    }
    detections = [_detection(500.0, 100.0)]

    matches, unmatched, ambiguous = service._associate(
        predictions, detections, frame_ts=1.0
    )

    assert matches == {}
    assert unmatched == {0}
    assert ambiguous == set()


def test_one_yolo_pass_updates_independent_track_distance_states():
    class FakeDetector:
        backend = "torch"

        def detect_batch(self, crops):
            rows = [
                np.array(
                    [300.0, 290.0, 340.0, 310.0, 0.9, 0.0],
                    dtype=np.float32,
                ),
                np.array(
                    [500.0, 290.0, 540.0, 310.0, 0.8, 0.0],
                    dtype=np.float32,
                ),
            ]
            return [rows for _ in crops]

    class FakeDistance:
        def __init__(self, distance):
            self.distance = distance
            self.update_count = 0

        def update(self, _detection, _frame_ts):
            self.update_count += 1
            return {
                "valid": True,
                "distance": self.distance,
                "source": "test",
                "warmup_count": 25,
                "reason": "",
            }

        def mark_missing(self, _frame_ts):
            raise AssertionError("matched target was marked missing")

    service = _service_for_association()
    service.min_sharpness = -1.0
    service.unsafe_confirm_frames = 3
    service.track_state_ttl_s = 2.0
    service.detector = FakeDetector()
    distance_11 = FakeDistance(80.0)
    distance_22 = FakeDistance(120.0)
    service.track_states = {
        11: {
            "distance": distance_11,
            "last_center": None,
            "last_center_ts": 0.0,
            "center_velocity": np.zeros(2, dtype=np.float32),
            "missing_frames": 0,
            "unsafe_frames": 0,
            "last_context_ts": 1.0,
        },
        22: {
            "distance": distance_22,
            "last_center": None,
            "last_center_ts": 0.0,
            "center_velocity": np.zeros(2, dtype=np.float32),
            "missing_frames": 0,
            "unsafe_frames": 0,
            "last_context_ts": 1.0,
        },
    }
    predictions = {
        11: np.array([1000.0, 680.0], dtype=np.float32),
        22: np.array([1200.0, 680.0], dtype=np.float32),
    }
    frame = np.zeros((1440, 2560, 3), dtype=np.uint8)

    result = service._process_frame(
        frame,
        frame_ts=1.1,
        master_track_id=11,
        predictions=predictions,
    )

    assert result["roi_count"] == 1
    assert result["matched_count"] == 2
    assert result["track_results"][11]["distance"] == 80.0
    assert result["track_results"][22]["distance"] == 120.0
    assert distance_11.update_count == 1
    assert distance_22.update_count == 1
