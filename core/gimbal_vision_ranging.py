#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gimbal-camera YOLO detection and optional ranging service.

The service deliberately separates camera capture from inference:
- capture always drains the camera so movement-blurred frames do not remain queued;
- inference is allowed only after the gimbal reports settled;
- RK3588 uses bestall_2k.rknn on the native 2560x1440 gimbal frame;
- all visible SORT projections are associated to YOLO boxes one-to-one;
- every SORT track_id owns an independent motion-gate/MLP/GRU temporal state;
- bbox coordinates enter the distance model in the native 2560x1440 camera
  coordinate system.
"""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment

try:
    import cv2
except ImportError as exc:
    raise ImportError(
        "OpenCV is required by gimbal_vision_ranging. On RK3588 install "
        "it with: sudo apt-get install -y python3-opencv"
    ) from exc

try:
    import torch
except ImportError:
    torch = None

try:
    from rknnlite.api import RKNNLite
except ImportError:
    RKNNLite = None


PROJECT_ROOT = Path(__file__).resolve().parent
if (
    not (PROJECT_ROOT / "distance model").is_dir()
    and (PROJECT_ROOT.parent / "distance model").is_dir()
):
    PROJECT_ROOT = PROJECT_ROOT.parent
DISTANCE_ROOT = PROJECT_ROOT / "distance model"
DISTANCE_SRC = DISTANCE_ROOT / "src"

FRAME_W = 2560
FRAME_H = 1440
ROI_SIZE = 640
TARGET_WIDTH_M = 0.30
TARGET_HEIGHT_M = 0.15
FX = 11494.25
FY = 11494.25


def _find_yolov5_root() -> Path:
    configured = os.getenv("YOLOV5_ROOT", "").strip()
    candidates = [
        Path(configured) if configured else None,
        PROJECT_ROOT / "yolov5",
        PROJECT_ROOT / "third_party" / "yolov5-v7",
        DISTANCE_ROOT / "yolov5",
        PROJECT_ROOT / "scratch" / "yolov5-v7",
    ]
    for candidate in candidates:
        if candidate and (candidate / "models" / "common.py").is_file():
            return candidate.resolve()
    raise RuntimeError(
        "YOLOv5 runtime not found. Set YOLOV5_ROOT to a YOLOv5 v7-compatible "
        "repository containing models/common.py."
    )


def _enumerate_dshow_video_devices() -> list[str]:
    if os.name != "nt":
        return []
    try:
        result = subprocess.run(
            [
                "ffmpeg",
                "-hide_banner",
                "-list_devices",
                "true",
                "-f",
                "dshow",
                "-i",
                "dummy",
            ],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=20,
            check=False,
        )
    except FileNotFoundError:
        return []
    text = (result.stderr or "") + "\n" + (result.stdout or "")
    devices = []
    for line in text.splitlines():
        match = re.search(r'\]\s+"(.+)"\s+\(video\)', line)
        if match:
            devices.append(match.group(1))
    return devices


def _parse_camera_source(value: str):
    text = str(value).strip()
    if not text:
        raise ValueError("empty gimbal camera source")
    if os.name == "nt":
        devices = _enumerate_dshow_video_devices()
        matches = [index for index, name in enumerate(devices) if name == text]
        if len(matches) == 1:
            index = matches[0]
            print(
                f"[GimbalVision][Camera] DirectShow name {text!r} "
                f"resolved to index {index}"
            )
            return index, cv2.CAP_DSHOW, f"{text}(dshow-index:{index})"
        if len(matches) > 1:
            raise RuntimeError(
                f"multiple DirectShow cameras named {text!r}; "
                "configure an explicit numeric index"
            )
        if text.startswith("0") and len(text) > 1 and text.isdigit():
            available = ", ".join(
                f"{index}:{name}" for index, name in enumerate(devices)
            ) or "none"
            raise RuntimeError(
                f"gimbal camera device name {text!r} not found; "
                f"DirectShow devices: {available}"
            )
        if text.lstrip("+-").isdigit():
            return int(text), cv2.CAP_DSHOW, f"dshow-index:{int(text)}"
        return text, cv2.CAP_ANY, text
    if text.lstrip("+-").isdigit():
        return int(text), cv2.CAP_V4L2, f"v4l2-index:{int(text)}"
    if text.startswith("/dev/video") or text.startswith("/dev/v4l/"):
        return text, cv2.CAP_V4L2, text
    return text, cv2.CAP_ANY, text


def _load_mlp(path: Path) -> dict:
    data = np.load(path, allow_pickle=True)
    return {
        "x_mean": data["x_mean"],
        "x_std": np.where(data["x_std"] < 1e-2, 1.0, data["x_std"]),
        "y_mean": data["y_mean"],
        "y_std": data["y_std"],
        "w1": data["w1"],
        "b1": data["b1"],
        "w2": data["w2"],
        "b2": data["b2"],
        "w3": data["w3"],
        "b3": data["b3"],
        "feature_columns": [str(value) for value in data["feature_columns"]],
    }


def _predict_mlp(model: dict, features: dict) -> float:
    raw = np.array(
        [[features[name] for name in model["feature_columns"]]],
        dtype=np.float32,
    )
    normalized = (raw - model["x_mean"]) / model["x_std"]
    hidden1 = np.maximum(normalized @ model["w1"] + model["b1"], 0.0)
    hidden2 = np.maximum(hidden1 @ model["w2"] + model["b2"], 0.0)
    scaled = hidden2 @ model["w3"] + model["b3"]
    return float((scaled * model["y_std"] + model["y_mean"]).reshape(-1)[0])


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


class _NumpyGruDistanceModel:
    """Small NumPy GRU forward pass for RK3588 without PyTorch."""

    def __init__(self, path: Path):
        weights = np.load(path)
        self.weight_ih = weights["gru.weight_ih_l0"].astype(np.float32)
        self.weight_hh = weights["gru.weight_hh_l0"].astype(np.float32)
        self.bias_ih = weights["gru.bias_ih_l0"].astype(np.float32)
        self.bias_hh = weights["gru.bias_hh_l0"].astype(np.float32)
        self.fc_weight = weights["fc.weight"].astype(np.float32)
        self.fc_bias = weights["fc.bias"].astype(np.float32)
        self.hidden_size = int(self.weight_hh.shape[1])

    def predict(self, sequence: np.ndarray) -> float:
        h = np.zeros((self.hidden_size,), dtype=np.float32)
        for x in np.asarray(sequence, dtype=np.float32):
            gate_input = self.weight_ih @ x + self.bias_ih
            gate_hidden = self.weight_hh @ h + self.bias_hh
            i_r, i_z, i_n = np.split(gate_input, 3)
            h_r, h_z, h_n = np.split(gate_hidden, 3)
            reset = _sigmoid(i_r + h_r)
            update = _sigmoid(i_z + h_z)
            candidate = np.tanh(i_n + reset * h_n)
            h = (1.0 - update) * candidate + update * h
        return float((self.fc_weight @ h + self.fc_bias).reshape(-1)[0])


def _robust_physics_distance(
    width_px: float,
    height_px: float,
    disagreement_ratio: float,
) -> dict:
    result = {
        "valid": False,
        "reason": "",
        "width_m": math.nan,
        "height_m": math.nan,
        "area_m": math.nan,
        "fused_m": math.nan,
    }
    if width_px < 3.0 or height_px < 3.0:
        result["reason"] = "BBOX_TOO_SMALL"
        return result

    aspect = width_px / max(height_px, 1e-6)
    if aspect < 1.15 or aspect >= 8.0:
        result["reason"] = "ASPECT_RATIO_OUT_OF_RANGE"
        return result

    d_width = FX * TARGET_WIDTH_M / width_px
    d_height = FY * TARGET_HEIGHT_M / height_px
    d_area = math.sqrt(
        FX * FY * TARGET_WIDTH_M * TARGET_HEIGHT_M / (width_px * height_px)
    )
    candidates = [d_width, d_height, d_area]
    median = float(np.median(candidates))
    weights = [1.0, 1.0, 1.0]
    for index, value in enumerate(candidates):
        if abs(value - median) / max(1.0, median) > disagreement_ratio:
            weights[index] = 0.0
    if sum(weights) < 1.5:
        result.update(
            reason="PHYSICS_DISAGREEMENT",
            width_m=d_width,
            height_m=d_height,
            area_m=d_area,
            fused_m=sum(candidates) / len(candidates),
        )
        return result

    result.update(
        valid=True,
        width_m=d_width,
        height_m=d_height,
        area_m=d_area,
        fused_m=sum(
            weight * value for weight, value in zip(weights, candidates)
        ) / sum(weights),
    )
    return result


class _DistanceRuntime:
    """Port of the distance-model physics -> MLP -> 25-frame GRU chain."""

    _assets_lock = threading.Lock()
    _shared_assets = None

    def __init__(self):
        assets = self._get_shared_assets()
        self.config = assets["config"]
        self._motion_gate_cls = assets["motion_gate_cls"]
        self.mlp = assets["mlp"]
        self.feature_columns = assets["feature_columns"]
        self.sequence_length = assets["sequence_length"]
        self.means = assets["means"]
        self.stds = assets["stds"]
        self.gru = assets["gru"]
        self.gru_backend = assets["gru_backend"]
        self.lost_timeout = float(
            self.config.get("tracking", {}).get("lost_timeout_s", 1.0)
        )
        self.max_reject = int(
            self.config.get("tracking", {}).get(
                "max_consecutive_reject_frames", 5
            )
        )
        self.reset()

    @classmethod
    def _get_shared_assets(cls) -> dict:
        with cls._assets_lock:
            if cls._shared_assets is not None:
                return cls._shared_assets

            if str(DISTANCE_SRC) not in sys.path:
                sys.path.insert(0, str(DISTANCE_SRC))
            from uav_distance_pipeline.runtime_motion_gate import (
                RuntimeMotionGate,
            )

            config = yaml.safe_load(
                (
                    DISTANCE_ROOT
                    / "configs"
                    / "runtime_distance_stabilization.yaml"
                ).read_text(encoding="utf-8")
            )
            schema = json.loads(
                (DISTANCE_ROOT / "models" / "feature_schema.json")
                .read_text(encoding="utf-8")
            )
            feature_columns = schema["feature_columns"]
            normalization = json.loads(
                (DISTANCE_ROOT / "models" / "normalization.json")
                .read_text(encoding="utf-8")
            )
            means = np.array(
                [
                    normalization["mean"][name]
                    for name in feature_columns
                ],
                dtype=np.float32,
            )
            stds = np.array(
                [
                    normalization["std"][name]
                    for name in feature_columns
                ],
                dtype=np.float32,
            )
            cls._shared_assets = {
                "config": config,
                "motion_gate_cls": RuntimeMotionGate,
                "mlp": _load_mlp(
                    DISTANCE_ROOT
                    / "models"
                    / "distance_residual_mlp_numpy.npz"
                ),
                "feature_columns": feature_columns,
                "sequence_length": int(
                    schema.get("sequence_length", 25)
                ),
                "means": means,
                "stds": np.where(stds < 2e-2, 1.0, stds),
            }
            numpy_gru_path = (
                DISTANCE_ROOT
                / "models"
                / "distance_gru_static_25f_numpy.npz"
            )
            if numpy_gru_path.is_file():
                cls._shared_assets["gru"] = _NumpyGruDistanceModel(
                    numpy_gru_path
                )
                cls._shared_assets["gru_backend"] = "numpy_gru"
            elif torch is not None:
                cls._shared_assets["gru"] = torch.jit.load(
                    str(
                        DISTANCE_ROOT
                        / "models"
                        / "distance_gru_static_25f.torchscript.pt"
                    ),
                    map_location="cpu",
                ).eval()
                cls._shared_assets["gru_backend"] = "torchscript"
            else:
                raise RuntimeError(
                    "No distance GRU backend available. Provide "
                    "distance model/models/distance_gru_static_25f_numpy.npz "
                    "or install PyTorch."
                )
            return cls._shared_assets

    def reset(self) -> None:
        self.motion_gate = self._motion_gate_cls(self.config)
        self.features: deque[np.ndarray] = deque(maxlen=self.sequence_length)
        self.last_timestamp: float | None = None
        self.reject_count = 0

    def mark_missing(self, timestamp: float) -> None:
        if (
            self.last_timestamp is not None
            and timestamp - self.last_timestamp > self.lost_timeout
        ):
            self.reset()

    def update(self, detection: dict, timestamp: float) -> dict:
        if (
            self.last_timestamp is not None
            and timestamp - self.last_timestamp > self.lost_timeout
        ):
            self.reset()

        box = detection["box"]
        width_px = float(box[2] - box[0])
        height_px = float(box[3] - box[1])
        disagreement = float(
            self.config.get("measurement_quality", {}).get(
                "max_physics_disagreement_ratio", 0.5
            )
        )
        physics = _robust_physics_distance(
            width_px, height_px, disagreement
        )
        if not physics["valid"]:
            return {
                "valid": False,
                "reason": physics["reason"],
                "warmup_count": len(self.features),
            }

        gate_decision, kalman_distance, radial_velocity = (
            self.motion_gate.process(
                physics["fused_m"],
                timestamp,
                measurement_valid=True,
            )
        )
        if gate_decision == "REJECT":
            self.reject_count += 1
            if self.reject_count < self.max_reject:
                return {
                    "valid": False,
                    "reason": "MOTION_GATE_REJECT",
                    "warmup_count": len(self.features),
                }
            self.motion_gate.reset(physics["fused_m"])
            kalman_distance = physics["fused_m"]
            radial_velocity = 0.0
            self.reject_count = 0
        else:
            self.reject_count = 0

        center_u = float((box[0] + box[2]) / 2.0)
        center_v = float((box[1] + box[3]) / 2.0)
        frame_dt = (
            1.0 / 25.0
            if self.last_timestamp is None
            else max(1e-3, timestamp - self.last_timestamp)
        )
        features = {
            "bbox_width_px": width_px,
            "bbox_height_px": height_px,
            "bbox_area_px": width_px * height_px,
            "bbox_aspect_ratio": width_px / max(height_px, 1e-6),
            "bbox_center_u": center_u,
            "bbox_center_v": center_v,
            "bbox_center_u_norm": center_u / FRAME_W,
            "bbox_center_v_norm": center_v / FRAME_H,
            "detector_confidence": float(detection["confidence"]),
            "is_missing": 0.0,
            "is_interpolated": 0.0,
            "physics_width_range_m": physics["width_m"],
            "physics_height_range_m": physics["height_m"],
            "physics_area_range_m": physics["area_m"],
            "fused_range_m": physics["fused_m"],
            "frame_delta_t_s": frame_dt,
        }
        residual = _predict_mlp(self.mlp, features)
        mlp_distance = physics["fused_m"] + residual
        if not math.isfinite(mlp_distance) or mlp_distance <= 0.0:
            mlp_distance = physics["fused_m"]

        features["mlp_predicted_residual_m"] = residual
        features["raw_predicted_range_m"] = mlp_distance
        vector = np.array(
            [features[name] for name in self.feature_columns],
            dtype=np.float32,
        )
        self.features.append(vector)
        self.last_timestamp = timestamp

        stable_distance = mlp_distance
        source = "mlp_warmup"
        if len(self.features) == self.sequence_length:
            sequence = (
                np.array(self.features, dtype=np.float32) - self.means
            ) / self.stds
            if self.gru_backend == "numpy_gru":
                stable_distance = self.gru.predict(sequence)
            else:
                tensor = torch.from_numpy(sequence).unsqueeze(0)
                with torch.no_grad():
                    stable_distance = float(self.gru(tensor)[0].item())
            source = "gimbal_yolo_gru"

        if (
            not math.isfinite(stable_distance)
            or stable_distance <= 0.0
            or abs(stable_distance - kalman_distance) > 30.0
        ):
            stable_distance = float(kalman_distance)
            source = (
                "gimbal_yolo_kalman"
                if len(self.features) == self.sequence_length
                else "mlp_warmup"
            )

        return {
            # Physics + MLP already provide a usable first-frame estimate.
            # Keep accumulating the fixed 25-frame sequence so GRU can take
            # over once its native input window is complete.
            "valid": True,
            "reason": "",
            "distance": stable_distance,
            "source": source,
            "warmup_count": len(self.features),
            "physics_distance": physics["fused_m"],
            "mlp_distance": mlp_distance,
            "radial_velocity": radial_velocity,
        }


class _CpuYoloDetector:
    """YOLOv5 detector with RK3588-friendly backend selection.

    The RK3588 production path is the native 2K RKNN model. The legacy
    torch/opencv crop backends are retained as a development fallback.
    """

    def __init__(self, weights: Path, confidence: float):
        self.confidence = float(confidence)
        requested_backend = os.getenv(
            "GIMBAL_YOLO_BACKEND", "auto"
        ).strip().lower()
        if requested_backend not in ("auto", "rknn", "torch", "opencv"):
            raise ValueError(
                "GIMBAL_YOLO_BACKEND must be auto, rknn, torch, or opencv"
            )

        rknn_path = Path(
            os.getenv(
                "GIMBAL_YOLO_RKNN",
                str(DISTANCE_ROOT / "bestall_2k.rknn"),
            )
        )
        can_use_rknn = RKNNLite is not None and rknn_path.is_file()
        if requested_backend == "rknn" and not can_use_rknn:
            raise RuntimeError(
                f"GIMBAL_YOLO_BACKEND=rknn requires RKNNLite and model: {rknn_path}"
            )

        can_use_torch = torch is not None and Path(weights).suffix == ".pt"
        if requested_backend == "torch" and not can_use_torch:
            raise RuntimeError(
                "GIMBAL_YOLO_BACKEND=torch requires PyTorch and a .pt model"
            )
        if requested_backend in ("auto", "rknn") and can_use_rknn:
            self.backend = "rknn"
        elif requested_backend in ("auto", "torch") and can_use_torch:
            self.backend = "torch"
        else:
            self.backend = "opencv"

        if self.backend == "rknn":
            self._init_rknn(rknn_path)
        elif self.backend == "torch":
            self._init_torch(weights)
        else:
            self._init_opencv(weights)

    def _init_rknn(self, rknn_path: Path) -> None:
        self.rknn_path = Path(rknn_path)
        self.rknn = RKNNLite(verbose=False)
        ret = self.rknn.load_rknn(str(self.rknn_path))
        if ret != 0:
            raise RuntimeError(f"load_rknn failed ret={ret}: {self.rknn_path}")
        core_mask = getattr(
            RKNNLite,
            "NPU_CORE_0_1_2",
            getattr(RKNNLite, "NPU_CORE_AUTO", 0),
        )
        ret = self.rknn.init_runtime(core_mask=core_mask)
        if ret != 0:
            raise RuntimeError(f"init_runtime failed ret={ret}: {self.rknn_path}")
        print(
            f"[GimbalVision] YOLO loaded with RKNNLite native-2K: "
            f"weights={self.rknn_path}, input={FRAME_W}x{FRAME_H}, device=npu"
        )

    def _init_torch(self, weights: Path) -> None:
        yolo_root = _find_yolov5_root()
        if str(yolo_root) not in sys.path:
            sys.path.insert(0, str(yolo_root))
        from models.common import DetectMultiBackend
        from utils.general import non_max_suppression

        self._nms = non_max_suppression
        self.device = torch.device("cpu")

        # PyTorch >=2.6 changed torch.load(weights_only=True) by default.
        # YOLOv5 v7 checkpoints contain model classes and require the legacy
        # full-checkpoint loading behavior.
        original_torch_load = torch.load

        def compatible_torch_load(*args, **kwargs):
            kwargs.setdefault("weights_only", False)
            return original_torch_load(*args, **kwargs)

        torch.load = compatible_torch_load
        try:
            self.model = DetectMultiBackend(
                str(weights),
                device=self.device,
                fp16=False,
            )
        finally:
            torch.load = original_torch_load
        self.model.warmup(imgsz=(1, 3, ROI_SIZE, ROI_SIZE))
        print(
            f"[GimbalVision] YOLO loaded on CPU only: "
            f"weights={weights}, runtime={yolo_root}"
        )

    def _init_opencv(self, weights: Path) -> None:
        onnx_path = DISTANCE_ROOT / "best.onnx"
        configured = os.getenv("GIMBAL_YOLO_ONNX", "").strip()
        if configured:
            onnx_path = Path(configured)
        elif Path(weights).suffix == ".onnx":
            onnx_path = Path(weights)
        if not onnx_path.is_file():
            raise RuntimeError(
                f"OpenCV YOLO backend requires ONNX model: {onnx_path}"
            )
        self.net = cv2.dnn.readNetFromONNX(str(onnx_path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        print(
            f"[GimbalVision] YOLO loaded with OpenCV-DNN: "
            f"weights={onnx_path}, device=cpu"
        )

    def detect(self, crop: np.ndarray) -> list[np.ndarray]:
        batches = self.detect_batch([crop])
        return batches[0] if batches else []

    def detect_batch(
        self, crops: list[np.ndarray]
    ) -> list[list[np.ndarray]]:
        if self.backend == "rknn":
            return [[] for _ in crops]
        if self.backend == "opencv":
            return [self._detect_opencv(crop) for crop in crops]
        return self._detect_torch_batch(crops)

    @staticmethod
    def _xywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
        converted = np.empty_like(boxes, dtype=np.float32)
        converted[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
        converted[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
        converted[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.0
        converted[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.0
        return converted

    @staticmethod
    def _nms_numpy(
        boxes: np.ndarray,
        scores: np.ndarray,
        iou_threshold: float = 0.45,
    ) -> list[int]:
        order = scores.argsort()[::-1]
        keep: list[int] = []
        while order.size > 0:
            index = int(order[0])
            keep.append(index)
            if order.size == 1:
                break
            current = boxes[index]
            rest = boxes[order[1:]]
            x1 = np.maximum(current[0], rest[:, 0])
            y1 = np.maximum(current[1], rest[:, 1])
            x2 = np.minimum(current[2], rest[:, 2])
            y2 = np.minimum(current[3], rest[:, 3])
            intersection = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
            current_area = max(0.0, current[2] - current[0]) * max(
                0.0, current[3] - current[1]
            )
            rest_area = np.maximum(0.0, rest[:, 2] - rest[:, 0]) * np.maximum(
                0.0, rest[:, 3] - rest[:, 1]
            )
            iou = intersection / (current_area + rest_area - intersection + 1e-9)
            order = order[1:][iou <= iou_threshold]
        return keep

    def detect_frame(self, frame: np.ndarray) -> list[np.ndarray]:
        if self.backend != "rknn":
            raise RuntimeError("detect_frame is only available for RKNN backend")
        if frame.shape[:2] != (FRAME_H, FRAME_W):
            return []

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_tensor = np.expand_dims(image, 0).astype(np.uint8)
        outputs = self.rknn.inference(inputs=[input_tensor])
        if not outputs:
            return []
        output = np.squeeze(np.asarray(outputs[0]))
        if output.ndim != 2:
            output = output.reshape(-1, output.shape[-1])
        if output.ndim != 2 or output.shape[1] < 5:
            return []

        object_confidence = output[:, 4].astype(np.float32)
        if output.shape[1] > 5:
            class_scores = output[:, 5:].astype(np.float32)
            class_ids = np.argmax(class_scores, axis=1).astype(np.int32)
            scores = object_confidence * class_scores[
                np.arange(class_scores.shape[0]), class_ids
            ]
        else:
            class_ids = np.zeros((output.shape[0],), dtype=np.int32)
            scores = object_confidence

        mask = scores >= self.confidence
        if not np.any(mask):
            return []

        boxes = self._xywh_to_xyxy(output[mask, :4].astype(np.float32))
        scores = scores[mask]
        class_ids = class_ids[mask]
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, FRAME_W - 1)
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, FRAME_H - 1)

        # Native 2K YOLO has 226800 candidates. Keep the best candidates before
        # NMS so the main ranging thread cannot stall on low-confidence clutter.
        if len(scores) > 300:
            top_indices = np.argpartition(scores, -300)[-300:]
            boxes = boxes[top_indices]
            scores = scores[top_indices]
            class_ids = class_ids[top_indices]

        keep = self._nms_numpy(boxes, scores, 0.45)
        rows = []
        for index in keep[:20]:
            rows.append(
                np.array(
                    [
                        boxes[index, 0],
                        boxes[index, 1],
                        boxes[index, 2],
                        boxes[index, 3],
                        scores[index],
                        class_ids[index],
                    ],
                    dtype=np.float32,
                )
            )
        return rows

    def _detect_torch_batch(
        self, crops: list[np.ndarray]
    ) -> list[list[np.ndarray]]:
        if not crops:
            return []
        if any(
            crop.shape[:2] != (ROI_SIZE, ROI_SIZE)
            for crop in crops
        ):
            return [[] for _ in crops]
        images = np.stack(
            [
                np.ascontiguousarray(
                    crop.transpose((2, 0, 1))[::-1]
                )
                for crop in crops
            ]
        )
        tensor = torch.from_numpy(images).to(self.device).float() / 255.0
        with torch.no_grad():
            predictions = self.model(tensor)
        rows_by_image = self._nms(
            predictions,
            self.confidence,
            0.45,
            max_det=20,
        )
        return [
            [row.detach().cpu().numpy() for row in rows]
            for rows in rows_by_image
        ]

    def _detect_opencv(self, crop: np.ndarray) -> list[np.ndarray]:
        if crop.shape[:2] != (ROI_SIZE, ROI_SIZE):
            return []
        blob = cv2.dnn.blobFromImage(
            crop,
            scalefactor=1.0 / 255.0,
            size=(ROI_SIZE, ROI_SIZE),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        )
        self.net.setInput(blob)
        output = self.net.forward()
        output = np.asarray(output)
        if output.ndim == 3:
            output = output[0]
        if output.ndim != 2 or output.shape[1] < 5:
            return []

        boxes = []
        scores = []
        class_ids = []
        for row in output:
            object_confidence = float(row[4])
            if row.shape[0] > 5:
                class_scores = row[5:]
                class_id = int(np.argmax(class_scores))
                score = object_confidence * float(class_scores[class_id])
            else:
                class_id = 0
                score = object_confidence
            if score < self.confidence:
                continue
            center_x, center_y, width, height = [
                float(value) for value in row[:4]
            ]
            x1 = center_x - width / 2.0
            y1 = center_y - height / 2.0
            boxes.append([x1, y1, width, height])
            scores.append(score)
            class_ids.append(class_id)

        if not boxes:
            return []
        indices = cv2.dnn.NMSBoxes(
            boxes,
            scores,
            score_threshold=self.confidence,
            nms_threshold=0.45,
        )
        if len(indices) == 0:
            return []
        rows = []
        for index in np.array(indices).reshape(-1)[:20]:
            x1, y1, width, height = boxes[int(index)]
            rows.append(
                np.array(
                    [
                        x1,
                        y1,
                        x1 + width,
                        y1 + height,
                        scores[int(index)],
                        class_ids[int(index)],
                    ],
                    dtype=np.float32,
                )
            )
        return rows


class _LatestFrameCamera:
    def __init__(self, camera_ref, width: int, height: int):
        source, backend, label = _parse_camera_source(camera_ref)
        self.source = source
        self.backend = backend
        self.label = label
        self.width = int(width)
        self.height = int(height)
        self.lock = threading.Lock()
        self.frame = None
        self.frame_ts = 0.0
        self.frame_seq = 0
        self.stop_event = threading.Event()
        self.thread = None
        self.capture = None
        self.is_video_file = (
            isinstance(source, str) and Path(source).is_file()
        )
        try:
            self.video_start_frame = max(
                0, int(os.getenv("GIMBAL_VIDEO_START_FRAME", "0"))
            )
        except ValueError:
            self.video_start_frame = 0
        self.video_realtime = os.getenv(
            "GIMBAL_VIDEO_REALTIME", "1"
        ).strip().lower() not in ("0", "false", "no", "off")
        self.video_loop = os.getenv(
            "GIMBAL_VIDEO_LOOP", "0"
        ).strip().lower() not in ("0", "false", "no", "off")
        self.video_fps = 0.0

    def start(self) -> None:
        capture = cv2.VideoCapture(self.source, self.backend)
        if not capture.isOpened() and self.backend != cv2.CAP_ANY:
            capture.release()
            capture = cv2.VideoCapture(self.source)
        if not capture.isOpened():
            raise RuntimeError(
                f"cannot open gimbal camera source={self.label!r}"
            )
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.is_video_file:
            self.video_fps = float(capture.get(cv2.CAP_PROP_FPS))
            if not math.isfinite(self.video_fps) or self.video_fps <= 0.0:
                self.video_fps = 25.0
            capture.set(
                cv2.CAP_PROP_POS_FRAMES,
                float(self.video_start_frame),
            )
        self.capture = capture
        if self.is_video_file:
            print(
                f"[GimbalVision][Camera] opened video {self.label}, "
                f"start_frame={self.video_start_frame}, "
                f"fps={self.video_fps:.3f}, "
                f"realtime={self.video_realtime}, loop={self.video_loop}"
            )
        else:
            print(f"[GimbalVision][Camera] opened {self.label}")
        self.thread = threading.Thread(
            target=self._run,
            name="gimbal-camera-capture",
            daemon=True,
        )
        self.thread.start()

    def _run(self) -> None:
        failure_count = 0
        next_publish_t = time.monotonic()
        while not self.stop_event.is_set():
            ok, frame = self.capture.read()
            if not ok:
                if self.is_video_file and self.video_loop:
                    self.capture.set(
                        cv2.CAP_PROP_POS_FRAMES,
                        float(self.video_start_frame),
                    )
                    next_publish_t = time.monotonic()
                    continue
                failure_count += 1
                if failure_count % 50 == 1:
                    print(
                        f"[GimbalVision][Camera] frame read failed "
                        f"count={failure_count}"
                    )
                time.sleep(0.02)
                continue
            failure_count = 0
            if self.is_video_file and self.video_realtime:
                now_monotonic = time.monotonic()
                wait_s = next_publish_t - now_monotonic
                if wait_s > 0.0:
                    self.stop_event.wait(wait_s)
                    if self.stop_event.is_set():
                        break
                elif wait_s < -1.0:
                    next_publish_t = now_monotonic
                next_publish_t += 1.0 / self.video_fps
            timestamp = time.time()
            with self.lock:
                self.frame = frame
                self.frame_ts = timestamp
                self.frame_seq += 1

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None, 0.0, 0
            return self.frame.copy(), self.frame_ts, self.frame_seq

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.capture is not None:
            self.capture.release()


class GimbalVisionRangingService:
    """Gimbal-camera YOLO service with optional legacy distance inference.

    In simple mode, detections are buffered by temporary image-space IDs and
    kept independent of SORT.  ``detection_only=True`` is the single-laser
    production path: it never constructs or calls the physical/MLP/GRU distance
    runtime and returns YOLO bbox/center data only.
    """

    def __init__(
        self,
        camera_source,
        weights: Path | None = None,
        confidence: float = 0.30,
        settle_delay_s: float = 0.20,
        min_sharpness: float = 20.0,
        unsafe_confirm_frames: int = 3,
        association_max_px: float = 260.0,
        association_ambiguity_margin_px: float = 30.0,
        track_state_ttl_s: float = 2.0,
        detection_only: bool = False,
    ):
        self.camera = _LatestFrameCamera(
            camera_source,
            FRAME_W,
            FRAME_H,
        )
        self.detector = _CpuYoloDetector(
            weights or (DISTANCE_ROOT / "best.pt"),
            confidence,
        )
        self.detection_only = bool(detection_only)
        # The single-laser branch must not load physical/MLP/GRU assets.  Keep
        # the legacy runtime available only for callers that explicitly use
        # this service for monocular ranging.
        self._spare_distance_runtime = (
            None if self.detection_only else _DistanceRuntime()
        )
        self.settle_delay_s = float(settle_delay_s)
        self.min_sharpness = float(min_sharpness)
        self.unsafe_confirm_frames = max(1, int(unsafe_confirm_frames))
        self.association_max_px = max(20.0, float(association_max_px))
        self.association_ambiguity_margin_px = max(
            0.0, float(association_ambiguity_margin_px)
        )
        self.simple_mode = self.detection_only or os.getenv(
            "GIMBAL_VISION_SIMPLE_MODE", "1"
        ).strip().lower() not in ("0", "false", "no", "off")
        if self.detection_only:
            print(
                "[GimbalVision] YOLO detection-only mode enabled: "
                "physical/MLP/GRU distance models are disabled"
            )
        elif self.simple_mode:
            print(
                "[GimbalVision] simple ranging mode enabled: "
                "YOLO buffers are independent of SORT; ranging is associated afterwards"
            )
        self.simple_association_max_px = max(
            20.0,
            # Temporary single-target diagnostic default. Override with the
            # environment variable before multi-target operation.
            float(os.getenv("GIMBAL_VISION_SIMPLE_ASSOC_MAX_PX", "3000")),
        )
        # Reuse the frame already owned by this service. Do not open the V4L2
        # device in a separate preview process.
        self.preview_enabled = os.getenv(
            "GIMBAL_VISION_PREVIEW", "0"
        ).strip().lower() not in ("0", "false", "no", "off")
        self.preview_window_name = os.getenv(
            "GIMBAL_VISION_PREVIEW_WINDOW", "Gimbal YOLO preview"
        ).strip() or "Gimbal YOLO preview"
        self.preview_max_width = max(
            320, int(os.getenv("GIMBAL_VISION_PREVIEW_MAX_WIDTH", "1280"))
        )
        self.preview_initialized = False
        self.preview_error_reported = False
        self.track_state_ttl_s = max(0.5, float(track_state_ttl_s))
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None

        self.master_track_id = None
        self.gimbal_settled = False
        self.settled_ts = 0.0
        self.track_predictions: dict[int, np.ndarray] = {}
        self.track_states: dict[int, dict] = {}
        self.simple_track_states: dict[int, dict] = {}
        self.simple_next_track_id = 1
        self.last_processed_frame_seq = -1
        self.result = self._empty_result("IDLE")
        self.result["track_results"] = {}
        self.result["simple_measurements"] = []

    @staticmethod
    def _empty_result(state: str, track_id=None) -> dict:
        return {
            "state": state,
            "track_id": track_id,
            "bbox": None,
            "confidence": math.nan,
            "distance": math.nan,
            "distance_source": "none",
            "distance_valid": False,
            "warmup_count": 0,
            "frame_ts": 0.0,
            "sharpness": math.nan,
            "safe": False,
            "reposition_requested": False,
            "association_error_px": math.nan,
            "reason": "",
        }

    @staticmethod
    def _copy_track_result(result: dict) -> dict:
        copied = dict(result)
        if isinstance(copied.get("bbox"), np.ndarray):
            copied["bbox"] = copied["bbox"].copy()
        return copied

    def start(self) -> None:
        self.camera.start()
        if self.preview_enabled:
            print(
                f"[GimbalVision] local preview enabled: "
                f"window={self.preview_window_name!r}, "
                f"max_width={self.preview_max_width}"
            )
        self.thread = threading.Thread(
            target=self._run,
            name=(
                "gimbal-yolo-detection"
                if self.detection_only
                else "gimbal-vision-ranging"
            ),
            daemon=True,
        )
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.camera.stop()
        if self.preview_initialized:
            try:
                cv2.destroyWindow(self.preview_window_name)
                cv2.waitKey(1)
            except cv2.error:
                pass

    def _show_preview(self, frame, result: dict | None = None) -> None:
        """Show the raw gimbal camera with raw YOLO and ranging state."""
        if not self.preview_enabled or frame is None:
            return
        try:
            canvas = frame.copy()
            result = result or {}
            frame_h, frame_w = canvas.shape[:2]
            optical_center = (frame_w // 2, frame_h // 2)
            cross_half_size = max(18, int(round(min(frame_w, frame_h) * 0.025)))
            # Black outline keeps the optical-center marker visible over both
            # bright sky and dark targets; yellow matches the manual test UI.
            for thickness, color in ((7, (0, 0, 0)), (3, (0, 255, 255))):
                cv2.line(
                    canvas,
                    (optical_center[0] - cross_half_size, optical_center[1]),
                    (optical_center[0] + cross_half_size, optical_center[1]),
                    color,
                    thickness,
                    cv2.LINE_AA,
                )
                cv2.line(
                    canvas,
                    (optical_center[0], optical_center[1] - cross_half_size),
                    (optical_center[0], optical_center[1] + cross_half_size),
                    color,
                    thickness,
                    cv2.LINE_AA,
                )
                cv2.circle(
                    canvas,
                    optical_center,
                    8,
                    color,
                    thickness,
                    cv2.LINE_AA,
                )
            for detection in result.get("simple_detections", []):
                box = detection.get("bbox")
                if not isinstance(box, (list, tuple)) or len(box) != 4:
                    continue
                x1, y1, x2, y2 = (int(round(float(v))) for v in box)
                cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 220, 255), 2)
                cv2.putText(
                    canvas,
                    f"YOLO {float(detection.get('confidence', 0.0)):.2f}",
                    (x1, max(24, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (0, 220, 255), 2, cv2.LINE_AA,
                )
            for measurement in result.get("simple_measurements", []):
                box = measurement.get("bbox")
                if not isinstance(box, (list, tuple, np.ndarray)) or len(box) != 4:
                    continue
                x1, y1, x2, y2 = (int(round(float(v))) for v in box)
                valid = bool(measurement.get("distance_valid", False))
                color = (
                    (0, 220, 255)
                    if self.detection_only
                    else ((40, 220, 40) if valid else (0, 80, 255))
                )
                cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 3)
                if self.detection_only:
                    label = f"id={measurement.get('simple_id', '?')} YOLO"
                else:
                    warmup = int(measurement.get("warmup_count", 0))
                    label = f"id={measurement.get('simple_id', '?')} warmup={warmup}/25"
                if valid and not self.detection_only:
                    label += f" {float(measurement.get('distance', math.nan)):.1f}m"
                elif measurement.get("reason") and not self.detection_only:
                    label += f" {measurement['reason']}"
                cv2.putText(
                    canvas, label, (x1, min(FRAME_H - 12, y2 + 26)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2, cv2.LINE_AA,
                )
            state = str(result.get("state", "LIVE"))
            count = int(result.get("detection_count", 0))
            cv2.rectangle(canvas, (0, 0), (1450, 48), (0, 0, 0), -1)
            cv2.putText(
                canvas, f"Gimbal camera | state={state} | YOLO={count} | q: close preview",
                (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
                (255, 255, 255), 2, cv2.LINE_AA,
            )
            if not self.preview_initialized:
                cv2.namedWindow(self.preview_window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(
                    self.preview_window_name, self.preview_max_width,
                    int(self.preview_max_width * FRAME_H / FRAME_W),
                )
                self.preview_initialized = True
            scale = min(1.0, self.preview_max_width / float(canvas.shape[1]))
            if scale < 1.0:
                canvas = cv2.resize(canvas, None, fx=scale, fy=scale,
                                    interpolation=cv2.INTER_AREA)
            cv2.imshow(self.preview_window_name, canvas)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                self.preview_enabled = False
                cv2.destroyWindow(self.preview_window_name)
                self.preview_initialized = False
                print("[GimbalVision] local preview closed by user")
        except cv2.error as exc:
            if not self.preview_error_reported:
                print(f"[GimbalVision][Warn] local preview disabled: {exc}")
                self.preview_error_reported = True
            self.preview_enabled = False

    def _show_latest_preview(self, state: str) -> None:
        frame, _, _ = self.camera.latest()
        self._show_preview(frame, {"state": state})

    def update_context(
        self,
        master_track_id,
        gimbal_settled: bool,
        settled_ts: float,
        track_predictions=None,
        expected_center=None,
    ) -> None:
        """Update the current SORT-to-gimbal projection snapshot.

        track_predictions accepts dictionaries containing ``track_id`` and
        ``center``. ``expected_center`` remains as a compatibility path for a
        single master track.
        """
        normalized_master_id = (
            None if master_track_id is None else int(master_track_id)
        )
        normalized_predictions: dict[int, np.ndarray] = {}
        for item in track_predictions or []:
            try:
                track_id = int(item["track_id"])
                center = np.asarray(item["center"], dtype=np.float32)
            except (KeyError, TypeError, ValueError):
                continue
            if center.shape == (2,) and np.all(np.isfinite(center)):
                normalized_predictions[track_id] = center.copy()

        if (
            normalized_master_id is not None
            and normalized_master_id not in normalized_predictions
            and expected_center is not None
        ):
            center = np.asarray(expected_center, dtype=np.float32)
            if center.shape == (2,) and np.all(np.isfinite(center)):
                normalized_predictions[normalized_master_id] = center.copy()

        with self.lock:
            master_changed = normalized_master_id != self.master_track_id
            self.master_track_id = normalized_master_id
            self.gimbal_settled = bool(gimbal_settled)
            self.settled_ts = float(settled_ts)
            self.track_predictions = normalized_predictions
            if master_changed or not self.gimbal_settled:
                state = (
                    "IDLE"
                    if normalized_master_id is None
                    else "GIMBAL_MOVING"
                )
                self.result = self._empty_result(
                    state, normalized_master_id
                )
                self.result["reason"] = (
                    "" if normalized_master_id is None
                    else "GIMBAL_NOT_SETTLED"
                )
                self.result["track_results"] = {}
                self.result["simple_measurements"] = []
                self.simple_track_states.clear()
                self.simple_next_track_id = 1

    def get_result(self) -> dict:
        with self.lock:
            result = self._copy_track_result(self.result)
            result["track_results"] = {
                int(track_id): self._copy_track_result(track_result)
                for track_id, track_result in self.result.get(
                    "track_results", {}
                ).items()
            }
            result["simple_measurements"] = [
                self._copy_track_result(item)
                for item in self.result.get("simple_measurements", [])
            ]
            return result

    def _state_for(self, track_id: int, now_t: float) -> dict:
        state = self.track_states.get(track_id)
        if state is None:
            if self._spare_distance_runtime is not None:
                distance = self._spare_distance_runtime
                self._spare_distance_runtime = None
            else:
                distance = _DistanceRuntime()
            state = {
                "distance": distance,
                "last_center": None,
                "last_center_ts": 0.0,
                "center_velocity": np.zeros(2, dtype=np.float32),
                "missing_frames": 0,
                "unsafe_frames": 0,
                "last_context_ts": now_t,
            }
            self.track_states[track_id] = state
        state["last_context_ts"] = now_t
        return state

    def _prune_track_states(self, active_ids: set[int], now_t: float) -> None:
        for track_id, state in list(self.track_states.items()):
            if track_id in active_ids:
                continue
            if now_t - state["last_context_ts"] > self.track_state_ttl_s:
                del self.track_states[track_id]

    def _simple_state_for(self, simple_id: int, now_t: float) -> dict:
        state = self.simple_track_states.get(simple_id)
        if state is None:
            if self.detection_only:
                distance = None
            elif self._spare_distance_runtime is not None:
                distance = self._spare_distance_runtime
                self._spare_distance_runtime = None
            else:
                distance = _DistanceRuntime()
            state = {
                "distance": distance,
                "last_box": None,
                "last_center": None,
                "last_center_ts": 0.0,
                "center_velocity": np.zeros(2, dtype=np.float32),
                "class_id": None,
                "missing_frames": 0,
                "last_context_ts": now_t,
            }
            self.simple_track_states[simple_id] = state
        state["last_context_ts"] = now_t
        return state

    def _prune_simple_states(self, now_t: float) -> None:
        for simple_id, state in list(self.simple_track_states.items()):
            if now_t - state["last_context_ts"] > self.track_state_ttl_s:
                del self.simple_track_states[simple_id]

    def _associate_simple_detections(
        self,
        detections: list[dict],
        frame_ts: float,
    ) -> tuple[dict[int, tuple[int, float]], set[int]]:
        """Associate YOLO detections only to image-space ranging buffers."""
        candidate_ids = [
            simple_id
            for simple_id, state in self.simple_track_states.items()
            if state.get("last_center") is not None
        ]
        if not candidate_ids or not detections:
            return {}, set(range(len(detections)))

        invalid_cost = self.simple_association_max_px * 10.0
        cost_matrix = np.full(
            (len(candidate_ids), len(detections)),
            invalid_cost,
            dtype=np.float32,
        )
        center_distances = np.full_like(cost_matrix, np.inf)
        for state_index, simple_id in enumerate(candidate_ids):
            state = self.simple_track_states[simple_id]
            predicted_center = state["last_center"]
            dt = float(
                np.clip(frame_ts - state["last_center_ts"], 0.0, 0.5)
            )
            predicted_center = (
                predicted_center + state["center_velocity"] * dt
            )
            for detection_index, detection in enumerate(detections):
                if (
                    state.get("class_id") is not None
                    and int(detection["class_id"]) != int(state["class_id"])
                ):
                    continue
                center_distance = float(
                    np.linalg.norm(detection["center"] - predicted_center)
                )
                center_distances[state_index, detection_index] = center_distance
                if center_distance > self.simple_association_max_px:
                    continue
                previous_box = state.get("last_box")
                iou = (
                    0.0
                    if previous_box is None
                    else self._bbox_iou(previous_box, detection["box"])
                )
                cost_matrix[state_index, detection_index] = (
                    center_distance + (1.0 - iou) * 30.0
                )

        state_indices, detection_indices = linear_sum_assignment(cost_matrix)
        matches = {}
        unmatched_detection_indices = set(range(len(detections)))
        for state_index, detection_index in zip(
            state_indices, detection_indices
        ):
            center_distance = float(
                center_distances[state_index, detection_index]
            )
            if center_distance > self.simple_association_max_px:
                continue
            simple_id = candidate_ids[state_index]
            matches[simple_id] = (detection_index, center_distance)
            unmatched_detection_indices.discard(detection_index)
        return matches, unmatched_detection_indices

    def _simple_measurement_result(
        self,
        simple_id: int,
        detection: dict,
        association_error_px: float,
        frame_ts: float,
    ) -> dict:
        state = self._simple_state_for(simple_id, frame_ts)
        previous_box = state["last_box"]
        previous_center = state["last_center"]
        previous_center_ts = state["last_center_ts"]
        previous_missing_frames = int(state.get("missing_frames", 0))
        jitter_valid = False
        jitter_dt_s = math.nan
        jitter_dx_px = math.nan
        jitter_dy_px = math.nan
        jitter_center_px = math.nan
        jitter_center_norm = math.nan
        jitter_width_delta_px = math.nan
        jitter_height_delta_px = math.nan
        jitter_iou = math.nan
        if previous_center is not None and previous_center_ts > 0.0:
            dt = frame_ts - previous_center_ts
            if 1e-3 <= dt <= 1.0:
                measured_velocity = (
                    detection["center"] - previous_center
                ) / dt
                state["center_velocity"] = (
                    0.6 * state["center_velocity"]
                    + 0.4 * measured_velocity
                ).astype(np.float32)
                if previous_box is not None and previous_missing_frames == 0:
                    current_box = detection["box"]
                    jitter_dx_px = float(
                        detection["center"][0] - previous_center[0]
                    )
                    jitter_dy_px = float(
                        detection["center"][1] - previous_center[1]
                    )
                    jitter_center_px = math.hypot(
                        jitter_dx_px, jitter_dy_px
                    )
                    previous_width = float(previous_box[2] - previous_box[0])
                    previous_height = float(previous_box[3] - previous_box[1])
                    current_width = float(current_box[2] - current_box[0])
                    current_height = float(current_box[3] - current_box[1])
                    previous_diagonal = max(
                        1.0, math.hypot(previous_width, previous_height)
                    )
                    jitter_center_norm = jitter_center_px / previous_diagonal
                    jitter_width_delta_px = current_width - previous_width
                    jitter_height_delta_px = current_height - previous_height
                    jitter_iou = self._bbox_iou(previous_box, current_box)
                    jitter_dt_s = float(dt)
                    jitter_valid = True
        state["last_box"] = detection["box"].copy()
        state["last_center"] = detection["center"].copy()
        state["last_center_ts"] = frame_ts
        state["class_id"] = int(detection["class_id"])
        state["missing_frames"] = 0

        if self.detection_only:
            measurement = {
                "valid": False,
                "reason": "YOLO_DETECTION_ONLY",
                "warmup_count": 0,
            }
        else:
            measurement = state["distance"].update(detection, frame_ts)
        distance_valid = bool(measurement.get("valid", False))
        box = detection["box"]
        return {
            "state": (
                "DETECTED"
                if self.detection_only
                else ("MEASURING" if distance_valid else "WARMING")
            ),
            "track_id": simple_id,
            "simple_id": simple_id,
            "bbox": [float(value) for value in box],
            "center": [float(value) for value in detection["center"]],
            "confidence": float(detection["confidence"]),
            "class_id": int(detection["class_id"]),
            "distance": float(measurement.get("distance", math.nan)),
            "distance_source": measurement.get("source", "none"),
            "distance_valid": distance_valid,
            "warmup_count": int(measurement.get("warmup_count", 0)),
            "frame_ts": frame_ts,
            "sharpness": float(detection.get("sharpness", math.nan)),
            "safe": self._is_safe(box),
            "reposition_requested": False,
            "association_error_px": float(association_error_px),
            "bbox_jitter_valid": jitter_valid,
            "bbox_jitter_dt_s": jitter_dt_s,
            "bbox_jitter_dx_px": jitter_dx_px,
            "bbox_jitter_dy_px": jitter_dy_px,
            "bbox_jitter_center_px": jitter_center_px,
            "bbox_jitter_center_norm": jitter_center_norm,
            "bbox_jitter_width_delta_px": jitter_width_delta_px,
            "bbox_jitter_height_delta_px": jitter_height_delta_px,
            "bbox_jitter_iou": jitter_iou,
            "bbox_jitter_previous_missing_frames": previous_missing_frames,
            "reason": measurement.get("reason", ""),
        }

    @staticmethod
    def _roi_origin(center) -> tuple[int, int]:
        x = int(
            np.clip(
                round(float(center[0])) - ROI_SIZE // 2,
                0,
                FRAME_W - ROI_SIZE,
            )
        )
        y = int(
            np.clip(
                round(float(center[1])) - ROI_SIZE // 2,
                0,
                FRAME_H - ROI_SIZE,
            )
        )
        return x, y

    @staticmethod
    def _is_safe(box: np.ndarray) -> bool:
        x1, y1, x2, y2 = [float(value) for value in box]
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        center_safe = (
            0.20 * FRAME_W <= center_x <= 0.80 * FRAME_W
            and 0.20 * FRAME_H <= center_y <= 0.80 * FRAME_H
        )
        bbox_safe = (
            x1 >= 0.05 * FRAME_W
            and x2 <= 0.95 * FRAME_W
            and y1 >= 0.05 * FRAME_H
            and y2 <= 0.95 * FRAME_H
        )
        return center_safe and bbox_safe

    @staticmethod
    def _bbox_iou(box_a: np.ndarray, box_b: np.ndarray) -> float:
        inter_x1 = max(float(box_a[0]), float(box_b[0]))
        inter_y1 = max(float(box_a[1]), float(box_b[1]))
        inter_x2 = min(float(box_a[2]), float(box_b[2]))
        inter_y2 = min(float(box_a[3]), float(box_b[3]))
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        intersection = inter_w * inter_h
        area_a = max(0.0, float(box_a[2] - box_a[0])) * max(
            0.0, float(box_a[3] - box_a[1])
        )
        area_b = max(0.0, float(box_b[2] - box_b[0])) * max(
            0.0, float(box_b[3] - box_b[1])
        )
        union = area_a + area_b - intersection
        return 0.0 if union <= 0.0 else intersection / union

    def _visible_predictions(
        self, predictions: dict[int, np.ndarray]
    ) -> dict[int, np.ndarray]:
        margin = self.association_max_px
        return {
            track_id: center
            for track_id, center in predictions.items()
            if (
                -margin <= float(center[0]) <= FRAME_W + margin
                and -margin <= float(center[1]) <= FRAME_H + margin
            )
        }

    def _build_roi_origins(
        self,
        predictions: dict[int, np.ndarray],
        master_track_id,
    ) -> list[tuple[int, int]]:
        ordered = sorted(
            predictions.items(),
            key=lambda item: (
                0 if item[0] == master_track_id else 1,
                item[0],
            ),
        )
        origins: list[tuple[int, int]] = []
        coverage_margin = 96
        for _, center in ordered:
            covered = any(
                (
                    origin_x + coverage_margin
                    <= center[0]
                    <= origin_x + ROI_SIZE - coverage_margin
                    and origin_y + coverage_margin
                    <= center[1]
                    <= origin_y + ROI_SIZE - coverage_margin
                )
                for origin_x, origin_y in origins
            )
            if not covered:
                origin = self._roi_origin(center)
                if origin not in origins:
                    origins.append(origin)
        return origins

    def _build_simple_roi_origins(self) -> list[tuple[int, int]]:
        """Use a fixed coverage grid when simple ranging is decoupled from SORT."""
        if self.detector.backend == "rknn":
            return [(0, 0)]

        step = max(160, ROI_SIZE - 160)
        xs = list(range(0, FRAME_W - ROI_SIZE + 1, step))
        ys = list(range(0, FRAME_H - ROI_SIZE + 1, step))
        if xs[-1] != FRAME_W - ROI_SIZE:
            xs.append(FRAME_W - ROI_SIZE)
        if ys[-1] != FRAME_H - ROI_SIZE:
            ys.append(FRAME_H - ROI_SIZE)
        return [(x, y) for y in ys for x in xs]

    def _deduplicate_detections(self, detections: list[dict]) -> list[dict]:
        kept: list[dict] = []
        for detection in sorted(
            detections,
            key=lambda item: item["confidence"],
            reverse=True,
        ):
            center = detection["center"]
            duplicate = False
            for existing in kept:
                if (
                    self._bbox_iou(detection["box"], existing["box"]) >= 0.5
                    or np.linalg.norm(center - existing["center"]) <= 12.0
                ):
                    duplicate = True
                    break
            if not duplicate:
                kept.append(detection)
        return kept

    def _detect_all(
        self,
        frame: np.ndarray,
        roi_origins: list[tuple[int, int]],
    ) -> tuple[list[dict], float, int]:
        if self.detector.backend == "rknn":
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            detections = []
            for row in self.detector.detect_frame(frame):
                x1, y1, x2, y2, confidence, class_id = row.tolist()
                box = np.array([x1, y1, x2, y2], dtype=np.float32)
                center = np.array(
                    [
                        (box[0] + box[2]) / 2.0,
                        (box[1] + box[3]) / 2.0,
                    ],
                    dtype=np.float32,
                )
                detections.append(
                    {
                        "box": box,
                        "center": center,
                        "confidence": float(confidence),
                        "class_id": int(class_id),
                        "sharpness": sharpness,
                    }
                )
            return (
                self._deduplicate_detections(detections),
                sharpness,
                1,
            )

        detections = []
        sharpness_values = []
        sharp_rois = []
        for origin_x, origin_y in roi_origins:
            crop = frame[
                origin_y : origin_y + ROI_SIZE,
                origin_x : origin_x + ROI_SIZE,
            ]
            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
            sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            sharpness_values.append(sharpness)
            if sharpness < self.min_sharpness:
                continue
            sharp_rois.append(
                (origin_x, origin_y, sharpness, crop)
            )

        rows_by_roi = self.detector.detect_batch(
            [item[3] for item in sharp_rois]
        )
        for (
            origin_x,
            origin_y,
            sharpness,
            _,
        ), rows in zip(sharp_rois, rows_by_roi):
            for row in rows:
                x1, y1, x2, y2, confidence, class_id = row.tolist()
                # A target clipped by a dynamic ROI has an invalid bbox size
                # for monocular distance. It should be detected in the ROI
                # centered on its own SORT projection instead.
                if (
                    x1 <= 2.0
                    or y1 <= 2.0
                    or x2 >= ROI_SIZE - 2.0
                    or y2 >= ROI_SIZE - 2.0
                ):
                    continue
                box = np.array(
                    [
                        x1 + origin_x,
                        y1 + origin_y,
                        x2 + origin_x,
                        y2 + origin_y,
                    ],
                    dtype=np.float32,
                )
                center = np.array(
                    [
                        (box[0] + box[2]) / 2.0,
                        (box[1] + box[3]) / 2.0,
                    ],
                    dtype=np.float32,
                )
                detections.append(
                    {
                        "box": box,
                        "center": center,
                        "confidence": float(confidence),
                        "class_id": int(class_id),
                        "sharpness": sharpness,
                    }
                )
        max_sharpness = (
            max(sharpness_values) if sharpness_values else math.nan
        )
        return (
            self._deduplicate_detections(detections),
            max_sharpness,
            len(sharp_rois),
        )

    def _associate(
        self,
        predictions: dict[int, np.ndarray],
        detections: list[dict],
        frame_ts: float,
    ) -> tuple[
        dict[int, tuple[dict, float]],
        set[int],
        set[int],
    ]:
        if not predictions or not detections:
            return {}, set(range(len(detections))), set()

        track_ids = list(predictions)
        invalid_cost = self.association_max_px * 10.0
        cost_matrix = np.full(
            (len(track_ids), len(detections)),
            invalid_cost,
            dtype=np.float32,
        )
        geometric_distances = np.full_like(cost_matrix, np.inf)
        for track_index, track_id in enumerate(track_ids):
            expected = predictions[track_id]
            state = self.track_states.get(track_id)
            previous_center = (
                None if state is None else state.get("last_center")
            )
            previous_center_ts = (
                0.0 if state is None else state.get("last_center_ts", 0.0)
            )
            center_velocity = (
                None if state is None else state.get("center_velocity")
            )
            predicted_visual_center = previous_center
            if (
                previous_center is not None
                and center_velocity is not None
                and previous_center_ts > 0.0
            ):
                visual_dt = float(
                    np.clip(frame_ts - previous_center_ts, 0.0, 0.5)
                )
                predicted_visual_center = (
                    previous_center + center_velocity * visual_dt
                )
            for detection_index, detection in enumerate(detections):
                geometric_distance = float(
                    np.linalg.norm(detection["center"] - expected)
                )
                geometric_distances[
                    track_index, detection_index
                ] = geometric_distance
                if geometric_distance > self.association_max_px:
                    continue
                continuity_cost = 0.0
                if predicted_visual_center is not None:
                    continuity_cost = min(
                        float(
                            np.linalg.norm(
                                detection["center"]
                                - predicted_visual_center
                            )
                        ),
                        self.association_max_px,
                    )
                cost_matrix[track_index, detection_index] = (
                    geometric_distance
                    + 0.25 * continuity_cost
                )

        track_indices, detection_indices = linear_sum_assignment(cost_matrix)
        matches = {}
        ambiguous_track_ids = set()
        unmatched_detection_indices = set(range(len(detections)))
        for track_index, detection_index in zip(
            track_indices, detection_indices
        ):
            geometric_distance = float(
                geometric_distances[track_index, detection_index]
            )
            if geometric_distance > self.association_max_px:
                continue

            row = cost_matrix[track_index]
            column = cost_matrix[:, detection_index]
            row_valid = np.sort(row[row < invalid_cost])
            column_valid = np.sort(column[column < invalid_cost])
            row_best_index = int(np.argmin(row))
            column_best_index = int(np.argmin(column))
            row_margin = (
                math.inf
                if len(row_valid) < 2
                else float(row_valid[1] - row_valid[0])
            )
            column_margin = (
                math.inf
                if len(column_valid) < 2
                else float(column_valid[1] - column_valid[0])
            )
            is_mutual_nearest = (
                row_best_index == detection_index
                and column_best_index == track_index
            )
            is_well_separated = (
                row_margin >= self.association_ambiguity_margin_px
                and column_margin >= self.association_ambiguity_margin_px
            )
            if not is_mutual_nearest or not is_well_separated:
                ambiguous_track_ids.add(track_ids[track_index])
                continue

            track_id = track_ids[track_index]
            matches[track_id] = (
                detections[detection_index],
                geometric_distance,
            )
            unmatched_detection_indices.discard(detection_index)
        return (
            matches,
            unmatched_detection_indices,
            ambiguous_track_ids,
        )

    def _missing_result(
        self,
        track_id: int,
        frame_ts: float,
        sharpness: float,
        reason: str,
        state_name: str = "ACQUIRE",
    ) -> dict:
        state = self._state_for(track_id, frame_ts)
        state["missing_frames"] += 1
        state["distance"].mark_missing(frame_ts)
        return {
            **self._empty_result(state_name, track_id),
            "frame_ts": frame_ts,
            "sharpness": sharpness,
            "reason": reason,
        }

    def _matched_result(
        self,
        track_id: int,
        detection: dict,
        association_error_px: float,
        frame_ts: float,
    ) -> dict:
        state = self._state_for(track_id, frame_ts)
        box = detection["box"]
        previous_center = state["last_center"]
        previous_center_ts = state["last_center_ts"]
        if previous_center is not None and previous_center_ts > 0.0:
            center_dt = frame_ts - previous_center_ts
            if 1e-3 <= center_dt <= 1.0:
                measured_velocity = (
                    detection["center"] - previous_center
                ) / center_dt
                state["center_velocity"] = (
                    0.6 * state["center_velocity"]
                    + 0.4 * measured_velocity
                ).astype(np.float32)
        state["last_center"] = detection["center"].copy()
        state["last_center_ts"] = frame_ts
        state["missing_frames"] = 0
        measurement = state["distance"].update(detection, frame_ts)
        safe = self._is_safe(box)
        state["unsafe_frames"] = (
            0 if safe else state["unsafe_frames"] + 1
        )
        distance_valid = bool(measurement.get("valid", False))
        return {
            "state": "MEASURING" if distance_valid else "WARMING",
            "track_id": track_id,
            "bbox": box,
            "confidence": detection["confidence"],
            "distance": float(measurement.get("distance", math.nan)),
            "distance_source": measurement.get("source", "none"),
            "distance_valid": distance_valid,
            "warmup_count": int(measurement.get("warmup_count", 0)),
            "frame_ts": frame_ts,
            "sharpness": detection["sharpness"],
            "safe": safe,
            "reposition_requested": (
                state["unsafe_frames"] >= self.unsafe_confirm_frames
            ),
            "association_error_px": association_error_px,
            "reason": measurement.get("reason", ""),
        }

    def _process_frame(
        self,
        frame,
        frame_ts: float,
        master_track_id,
        predictions: dict[int, np.ndarray],
    ) -> dict:
        active_ids = set(predictions)
        if not self.simple_mode:
            self._prune_track_states(active_ids, frame_ts)
            for track_id in active_ids:
                self._state_for(track_id, frame_ts)

        if frame.shape[:2] != (FRAME_H, FRAME_W):
            reason = (
                f"CAMERA_RESOLUTION_{frame.shape[1]}x{frame.shape[0]}"
            )
            track_results = {
                track_id: self._missing_result(
                    track_id, frame_ts, math.nan, reason
                )
                for track_id in active_ids
            }
            master_result = track_results.get(
                master_track_id,
                {
                    **self._empty_result("ACQUIRE", master_track_id),
                    "frame_ts": frame_ts,
                    "reason": reason,
                },
            )
            return {
                **master_result,
                "track_results": track_results,
                "detection_count": 0,
                "matched_count": 0,
                "unmatched_detection_count": 0,
                "visible_track_count": 0,
                "active_track_count": len(active_ids),
                "roi_count": 0,
                "sharp_roi_count": 0,
                "matched_track_ids": [],
                "ambiguous_track_ids": [],
                "unmatched_track_ids": sorted(int(x) for x in active_ids),
                "unmatched_detections": [],
            }

        if self.simple_mode:
            simple_roi_origins = self._build_simple_roi_origins()
            detections, max_sharpness, sharp_roi_count = self._detect_all(
                frame, simple_roi_origins
            )
            self._prune_simple_states(frame_ts)
            simple_measurements = []
            matched_simple_ids = set()
            if self.detection_only:
                # single-laser does not need image-space Hungarian matching.
                # With exactly one detection its identity is simply slot 1;
                # multi-detection frames are rejected later by the alignment
                # gate, so their index IDs are diagnostic only.
                unmatched_detection_indices = set()
                for detection_index, detection in enumerate(detections):
                    simple_id = detection_index + 1
                    result = self._simple_measurement_result(
                        simple_id,
                        detection,
                        0.0,
                        frame_ts,
                    )
                    simple_measurements.append(result)
                    matched_simple_ids.add(simple_id)
            else:
                matches, unmatched_detection_indices = (
                    self._associate_simple_detections(detections, frame_ts)
                )
                for simple_id, (detection_index, assoc_err) in matches.items():
                    result = self._simple_measurement_result(
                        simple_id,
                        detections[detection_index],
                        assoc_err,
                        frame_ts,
                    )
                    simple_measurements.append(result)
                    matched_simple_ids.add(simple_id)

            for simple_id, state in self.simple_track_states.items():
                if simple_id in matched_simple_ids:
                    continue
                state["missing_frames"] += 1
                if state["distance"] is not None:
                    state["distance"].mark_missing(frame_ts)

            if not self.detection_only:
                for detection_index in sorted(unmatched_detection_indices):
                    simple_id = self.simple_next_track_id
                    self.simple_next_track_id += 1
                    result = self._simple_measurement_result(
                        simple_id,
                        detections[detection_index],
                        0.0,
                        frame_ts,
                    )
                    simple_measurements.append(result)
                    matched_simple_ids.add(simple_id)

            if simple_measurements:
                master_result = dict(simple_measurements[0])
            else:
                master_result = {
                    **self._empty_result(
                        "ACQUIRE" if master_track_id is not None else "IDLE",
                        master_track_id,
                    ),
                    "frame_ts": frame_ts,
                    "reason": (
                        "SIMPLE_MODE_NO_DETECTION"
                        if len(detections) == 0
                        else "SIMPLE_MODE_NO_MASTER"
                    ),
                }

            unmatched_detection_preview = []
            all_detections = []
            for detection in detections:
                box = detection["box"]
                center = detection["center"]
                all_detections.append({
                    "bbox": [
                        float(box[0]),
                        float(box[1]),
                        float(box[2]),
                        float(box[3]),
                    ],
                    "center": [float(center[0]), float(center[1])],
                    "confidence": float(detection.get("confidence", math.nan)),
                    "class_id": int(detection.get("class_id", -1)),
                    "sharpness": float(detection.get("sharpness", math.nan)),
                })

            return {
                **master_result,
                "track_results": {},
                "simple_measurements": simple_measurements,
                "detection_count": len(detections),
                "matched_count": len(simple_measurements),
                "unmatched_detection_count": 0,
                "visible_track_count": len(matched_simple_ids),
                "active_track_count": len(self.simple_track_states),
                "roi_count": len(simple_roi_origins),
                "sharp_roi_count": sharp_roi_count,
                "matched_track_ids": sorted(matched_simple_ids),
                "ambiguous_track_ids": [],
                "unmatched_track_ids": [
                    int(simple_id)
                    for simple_id in self.simple_track_states
                    if simple_id not in matched_simple_ids
                ],
                "unmatched_detections": unmatched_detection_preview,
                "simple_detections": all_detections,
                "simple_association_max_px": self.simple_association_max_px,
            }
        visible_predictions = self._visible_predictions(predictions)
        roi_origins = self._build_roi_origins(
            visible_predictions, master_track_id
        )
        detections, max_sharpness, sharp_roi_count = self._detect_all(
            frame, roi_origins
        )
        matches, unmatched_detection_indices, ambiguous_track_ids = self._associate(
            visible_predictions,
            detections,
            frame_ts,
        )
        unmatched_detection_preview = []
        for detection_index in sorted(
            unmatched_detection_indices,
            key=lambda idx: float(detections[idx].get("confidence", 0.0)),
            reverse=True,
        )[:5]:
            detection = detections[detection_index]
            box = detection["box"]
            center = detection["center"]
            unmatched_detection_preview.append({
                "bbox": [
                    float(box[0]),
                    float(box[1]),
                    float(box[2]),
                    float(box[3]),
                ],
                "center": [float(center[0]), float(center[1])],
                "confidence": float(detection.get("confidence", math.nan)),
                "class_id": int(detection.get("class_id", -1)),
            })

        track_results = {}
        for track_id in active_ids:
            if track_id not in visible_predictions:
                track_results[track_id] = self._missing_result(
                    track_id,
                    frame_ts,
                    max_sharpness,
                    "TRACK_OUT_OF_GIMBAL_FOV",
                )
            elif track_id in matches:
                detection, association_error_px = matches[track_id]
                track_results[track_id] = self._matched_result(
                    track_id,
                    detection,
                    association_error_px,
                    frame_ts,
                )
            else:
                blurred = bool(roi_origins) and sharp_roi_count == 0
                ambiguous = track_id in ambiguous_track_ids
                track_results[track_id] = self._missing_result(
                    track_id,
                    frame_ts,
                    max_sharpness,
                    (
                        "FRAME_BLURRED"
                        if blurred
                        else "ASSOCIATION_AMBIGUOUS"
                        if ambiguous
                        else "TARGET_MISSING"
                    ),
                    "SETTLING" if blurred else "ACQUIRE",
                )

        master_result = track_results.get(
            master_track_id,
            {
                **self._empty_result(
                    "IDLE" if master_track_id is None else "ACQUIRE",
                    master_track_id,
                ),
                "frame_ts": frame_ts,
                "reason": (
                    "" if master_track_id is None else "TARGET_MISSING"
                ),
            },
        )
        return {
            **master_result,
            "track_results": track_results,
            "detection_count": len(detections),
            "matched_count": len(matches),
            "unmatched_detection_count": len(unmatched_detection_indices),
            "visible_track_count": len(visible_predictions),
            "active_track_count": len(active_ids),
            "roi_count": len(roi_origins),
            "sharp_roi_count": sharp_roi_count,
            "matched_track_ids": sorted(int(x) for x in matches),
            "ambiguous_track_ids": sorted(int(x) for x in ambiguous_track_ids),
            "unmatched_track_ids": sorted(
                int(track_id)
                for track_id in visible_predictions
                if track_id not in matches
            ),
            "unmatched_detections": unmatched_detection_preview,
        }

    def _run(self) -> None:
        print(
            "[GimbalVision] YOLO detection thread started"
            if self.detection_only
            else "[GimbalVision] multi-target ranging thread started"
        )
        while not self.stop_event.is_set():
            with self.lock:
                master_track_id = self.master_track_id
                settled = self.gimbal_settled
                settled_ts = self.settled_ts
                predictions = {
                    track_id: center.copy()
                    for track_id, center in self.track_predictions.items()
                }

            if master_track_id is None:
                with self.lock:
                    self.result = self._empty_result("IDLE")
                    self.result["track_results"] = {}
                self._show_latest_preview("IDLE")
                time.sleep(0.02)
                continue

            if not settled:
                with self.lock:
                    self.result = {
                        **self._empty_result(
                            "GIMBAL_MOVING", master_track_id
                        ),
                        "reason": "GIMBAL_NOT_SETTLED",
                        "track_results": {},
                    }
                self._show_latest_preview("GIMBAL_MOVING")
                time.sleep(0.02)
                continue

            frame, frame_ts, frame_seq = self.camera.latest()
            if frame is None or frame_seq == self.last_processed_frame_seq:
                time.sleep(0.005)
                continue
            self.last_processed_frame_seq = frame_seq

            if frame_ts < settled_ts + self.settle_delay_s:
                with self.lock:
                    self.result = {
                        **self._empty_result(
                            "SETTLING", master_track_id
                        ),
                        "frame_ts": frame_ts,
                        "reason": "POST_SETTLE_DELAY",
                        "track_results": {},
                    }
                self._show_preview(frame, {"state": "SETTLING"})
                continue

            result = self._process_frame(
                frame,
                frame_ts,
                master_track_id,
                predictions,
            )
            self._show_preview(frame, result)
            with self.lock:
                if self.gimbal_settled:
                    current_master_id = self.master_track_id
                    track_results = result["track_results"]
                    if self.simple_mode:
                        master_result = result
                    else:
                        master_result = track_results.get(
                            current_master_id,
                            {
                                **self._empty_result(
                                    "ACQUIRE", current_master_id
                                ),
                                "frame_ts": frame_ts,
                                "reason": "TARGET_MISSING",
                            },
                        )
                    self.result = {
                        **master_result,
                        "track_results": track_results,
                        "simple_measurements": result.get(
                            "simple_measurements", []
                        ),
                        "detection_count": result["detection_count"],
                        "matched_count": result["matched_count"],
                        "unmatched_detection_count": result.get(
                            "unmatched_detection_count", 0
                        ),
                        "visible_track_count": result.get(
                            "visible_track_count", 0
                        ),
                        "active_track_count": result.get(
                            "active_track_count", 0
                        ),
                        "roi_count": result["roi_count"],
                        "sharp_roi_count": result.get("sharp_roi_count", 0),
                        "matched_track_ids": result.get(
                            "matched_track_ids", []
                        ),
                        "ambiguous_track_ids": result.get(
                            "ambiguous_track_ids", []
                        ),
                        "unmatched_track_ids": result.get(
                            "unmatched_track_ids", []
                        ),
                        "unmatched_detections": result.get(
                            "unmatched_detections", []
                        ),
                    }
