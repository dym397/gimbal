import random
import socket
import threading
import time
import math
import struct
import json
import queue
import os
import sys
import csv
import atexit
from collections import deque, Counter

# ==========================================
# 0. 驱动引入
# ==========================================
try:
    from gimbal_interface import GT06ZAdapter
except ImportError as e:
    print(f"[System] 驱动接口加载失败: {e}")
    exit(1) # 驱动加载失败直接退出，防止后续报错
try:
    from mock_gimbal import MockGimbalAdapter
except ImportError:
    MockGimbalAdapter = None
try:
    from sfl0603_driver import (
        SFL0603,
        ResponseTimeoutError as SFL0603ResponseTimeoutError,
    )
except ImportError:
    SFL0603 = None
    SFL0603ResponseTimeoutError = None
try:
    from gps import DEFAULT_LATITUDE, DEFAULT_LONGITUDE, read_gps_fix
except ImportError:
    DEFAULT_LATITUDE = None
    DEFAULT_LONGITUDE = None
    read_gps_fix = None
try:
    from gimbal_vision_ranging import GimbalVisionRangingService
except ImportError:
    GimbalVisionRangingService = None
try:
    from single_target_visual_alignment import (
        SingleTargetAlignmentConfig,
        SingleTargetVisualAlignment,
        quantize_angle_0p1,
    )
except ImportError:
    SingleTargetAlignmentConfig = None
    SingleTargetVisualAlignment = None
    quantize_angle_0p1 = None
try:
    from gimbal_visual_target_lock import (
        GimbalVisualLockConfig,
        GimbalVisualTargetLock,
        should_run_vision_only_tick,
    )
except ImportError:
    GimbalVisualLockConfig = None
    GimbalVisualTargetLock = None
    should_run_vision_only_tick = None
# ==========================================
# 配置
# ==========================================
# Keep one codepath and switch only the serial defaults by platform.
def _platform_serial_defaults():
    if os.name == "nt":
        return {
            "gimbal": "COM8",
            "laser": "COM12",
            "gps": "COM8",
        }
    return {
        "gimbal": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.2:1.0-port0",
        "laser": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.4:1.0-port0",
        "gps": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.1:1.0-port0",
    }


_SERIAL_PORT_DEFAULTS = _platform_serial_defaults()


def _serial_port_default(role):
    return _SERIAL_PORT_DEFAULTS[role]


def _serial_port(env_name, role):
    port_name = os.getenv(env_name, _serial_port_default(role))
    return port_name.strip() if isinstance(port_name, str) else port_name


def _is_windows_com_port(port_name):
    if not isinstance(port_name, str):
        return False
    normalized = port_name.strip().upper()
    if normalized.startswith("\\\\.\\"):
        normalized = normalized[4:]
    return normalized.startswith("COM") and normalized[3:].isdigit() and int(normalized[3:]) > 0


def _normalize_windows_com_port(port_name):
    normalized = port_name.strip().upper()
    if normalized.startswith("\\\\.\\"):
        normalized = normalized[4:]
    return normalized


def _is_linux_serial_port(port_name):
    if not isinstance(port_name, str):
        return False
    normalized = port_name.strip()
    return normalized.startswith((
        "/dev/ttyUSB",
        "/dev/ttyACM",
        "/dev/ttyS",
        "/dev/serial/by-id/",
        "/dev/serial/by-path/",
    ))


def _validate_serial_port(label, port_name):
    if not isinstance(port_name, str) or not port_name:
        print(f"[Config][Warn] {label} 未配置有效的串口设备名。")
        return

    if os.name == "nt":
        if not _is_windows_com_port(port_name):
            print(
                f"[Config][Warn] {label}={port_name} 不是 Windows 串口设备名。"
                "请改为 COM3、COM4 或 \\\\.\\COM10 这类格式。"
            )
            return
        try:
            from serial.tools import list_ports
        except Exception:
            return
        available_ports = {_normalize_windows_com_port(port.device) for port in list_ports.comports()}
        normalized_port = _normalize_windows_com_port(port_name)
        if available_ports and normalized_port not in available_ports:
            print(
                f"[Config][Warn] {label}={port_name} 未在当前 Windows 串口列表中发现。"
                f"当前可用端口: {', '.join(sorted(available_ports))}"
            )
        return

    if not _is_linux_serial_port(port_name):
        print(
            f"[Config][Warn] {label}={port_name} 不是 Linux 串口设备路径。"
            "请改为 /dev/ttyUSB*、/dev/ttyACM*、/dev/ttyS*、/dev/serial/by-id/* "
            "或 /dev/serial/by-path/*。"
        )
        return
    if not os.path.exists(port_name):
        print(
            f"[Config][Warn] {label}={port_name} 当前不存在。"
            "请确认设备节点、udev 映射或 USB 串口权限。"
        )


class _TeeStream:
    def __init__(self, *streams):
        self._streams = streams
        self._lock = threading.Lock()

    def write(self, data):
        with self._lock:
            for stream in self._streams:
                stream.write(data)
            for stream in self._streams:
                stream.flush()
        return len(data)

    def flush(self):
        with self._lock:
            for stream in self._streams:
                stream.flush()

    def isatty(self):
        for stream in self._streams:
            is_tty = getattr(stream, "isatty", None)
            if callable(is_tty) and is_tty():
                return True
        return False

    @property
    def encoding(self):
        return getattr(self._streams[0], "encoding", "utf-8")

    def fileno(self):
        fileno = getattr(self._streams[0], "fileno", None)
        if callable(fileno):
            return fileno()
        raise OSError("fileno not supported")


_LOG_MIRROR_INITIALIZED = False
_LOG_MIRROR_PATH = None


def _create_run_log_dir(base_dir):
    os.makedirs(base_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    run_dir = os.path.join(base_dir, timestamp)
    suffix = 1
    while True:
        try:
            os.makedirs(run_dir, exist_ok=False)
            return run_dir
        except FileExistsError:
            run_dir = os.path.join(base_dir, f"{timestamp}_{suffix:02d}")
            suffix += 1


def _setup_log_mirror(log_dir):
    global _LOG_MIRROR_INITIALIZED, _LOG_MIRROR_PATH
    if _LOG_MIRROR_INITIALIZED:
        return _LOG_MIRROR_PATH

    os.makedirs(log_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    log_prefix = os.getenv("LOG_PREFIX", "main_tracking_v9").strip() or "main_tracking_v9"
    safe_prefix = "".join(ch if (ch.isalnum() or ch in ("-", "_")) else "_" for ch in log_prefix)
    log_path = os.path.join(log_dir, f"{safe_prefix}_{timestamp}.log")
    log_file = open(log_path, "a", encoding="utf-8", newline="\n", buffering=1)
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    sys.stdout = _TeeStream(original_stdout, log_file)
    sys.stderr = _TeeStream(original_stderr, log_file)

    def _cleanup_log_mirror():
        try:
            sys.stdout = original_stdout
            sys.stderr = original_stderr
        except Exception:
            pass
        try:
            log_file.flush()
            log_file.close()
        except Exception:
            pass

    atexit.register(_cleanup_log_mirror)
    _LOG_MIRROR_INITIALIZED = True
    _LOG_MIRROR_PATH = log_path
    return log_path


def _env_flag(name, default=True):
    val = os.getenv(name)
    if val is None:
        return default
    return str(val).strip().lower() not in ("0", "false", "no", "off")


def _env_float(name, default):
    val = os.getenv(name)
    if val is None:
        return float(default)
    try:
        return float(str(val).strip())
    except (TypeError, ValueError):
        print(f"[Config][Warn] {name}={val} 不是有效浮点数，使用默认值 {default}.")
        return float(default)


def _env_int(name, default):
    val = os.getenv(name)
    if val is None:
        return int(default)
    try:
        return int(str(val).strip())
    except (TypeError, ValueError):
        print(f"[Config][Warn] {name}={val} 不是有效整数，使用默认值 {default}.")
        return int(default)


UI_IP = os.getenv("UI_IP", "192.168.0.200")
# UI_IP="172.28.3.80"
UI_PORT = int(os.getenv("UI_PORT", "9999"))
LOCAL_PORT = int(os.getenv("LOCAL_PORT", "8888"))
WINDOWS_GIMBAL_CAMERA_SOURCE = "000000008"
LINUX_GIMBAL_CAMERA_SOURCE = "/dev/v4l/by-path/platform-xhci-hcd.4.auto-usb-0:1.1:1.0-video-index0"
ENABLE_STRIKE_SEND = _env_flag("ENABLE_STRIKE_SEND", False)
STRIKE_IP = os.getenv("STRIKE_IP", "192.168.0.80")
STRIKE_PORT = int(os.getenv("STRIKE_PORT", "10123"))
STRIKE_SEND_HZ = _env_float("STRIKE_SEND_HZ", 10.0)
STRIKE_WINDOW_SECONDS = _env_float("STRIKE_WINDOW_SECONDS", 1.0)
STRIKE_LEAD_TIME = _env_float("STRIKE_LEAD_TIME", 0.3)
STRIKE_SETTLED_EVENT_TTL = _env_float("STRIKE_SETTLED_EVENT_TTL", 0.5)
TRACK_DISTANCE_TTL = _env_float("TRACK_DISTANCE_TTL", 3.0)
ENABLE_GIMBAL_VISION = _env_flag("ENABLE_GIMBAL_VISION", True)
# This branch has one authoritative range source: the coaxial SFL0603 laser.
# Fixed-camera packets and gimbal-camera YOLO provide direction/identity only.
SINGLE_LASER_MODE = True
GIMBAL_CAMERA_SOURCE = os.getenv(
    "GIMBAL_CAMERA_SOURCE",
    WINDOWS_GIMBAL_CAMERA_SOURCE if os.name == "nt" else LINUX_GIMBAL_CAMERA_SOURCE,
).strip()
GIMBAL_VISION_CONFIDENCE = _env_float("GIMBAL_VISION_CONFIDENCE", 0.30)
GIMBAL_VISION_SETTLE_DELAY = _env_float("GIMBAL_VISION_SETTLE_DELAY", 0.20)
GIMBAL_VISION_MIN_SHARPNESS = _env_float("GIMBAL_VISION_MIN_SHARPNESS", 20.0)
GIMBAL_VISION_RESULT_TTL = _env_float("GIMBAL_VISION_RESULT_TTL", 1.00)
GIMBAL_VISION_ASSOCIATION_MAX_PX = _env_float(
    "GIMBAL_VISION_ASSOCIATION_MAX_PX", 260.0
)
GIMBAL_VISION_AMBIGUITY_MARGIN_PX = _env_float(
    "GIMBAL_VISION_AMBIGUITY_MARGIN_PX", 30.0
)
GIMBAL_VISION_TRACK_STATE_TTL = _env_float(
    "GIMBAL_VISION_TRACK_STATE_TTL", 2.0
)
ENABLE_SINGLE_TARGET_YOLO_ALIGNMENT = _env_flag(
    "ENABLE_SINGLE_TARGET_YOLO_ALIGNMENT", True
)
GIMBAL_YOLO_ALIGN_STABLE_FRAMES = _env_int(
    "GIMBAL_YOLO_ALIGN_STABLE_FRAMES", 1
)
GIMBAL_YOLO_ALIGN_MAX_STD_X_PX = _env_float(
    "GIMBAL_YOLO_ALIGN_MAX_STD_X_PX", 10.0
)
GIMBAL_YOLO_ALIGN_MAX_STD_Y_PX = _env_float(
    "GIMBAL_YOLO_ALIGN_MAX_STD_Y_PX", 10.0
)
GIMBAL_YOLO_ALIGN_MAX_SPEED_X_PX_S = _env_float(
    "GIMBAL_YOLO_ALIGN_MAX_SPEED_X_PX_S", 15.0
)
GIMBAL_YOLO_ALIGN_MAX_SPEED_Y_PX_S = _env_float(
    "GIMBAL_YOLO_ALIGN_MAX_SPEED_Y_PX_S", 15.0
)
GIMBAL_YOLO_ALIGN_TRIGGER_X_PX = _env_float(
    "GIMBAL_YOLO_ALIGN_TRIGGER_X_PX", 10.0
)
GIMBAL_YOLO_ALIGN_TRIGGER_Y_PX = _env_float(
    "GIMBAL_YOLO_ALIGN_TRIGGER_Y_PX", 10.0
)
GIMBAL_YOLO_ALIGN_CENTERED_X_PX = _env_float(
    "GIMBAL_YOLO_ALIGN_CENTERED_X_PX", 10.0
)
GIMBAL_YOLO_ALIGN_CENTERED_Y_PX = _env_float(
    "GIMBAL_YOLO_ALIGN_CENTERED_Y_PX", 10.0
)
GIMBAL_YOLO_ALIGN_MAX_FRAME_GAP_S = _env_float(
    # Disabled by default: a YOLO dropout does not prove target motion.
    # Set a positive value only when strict detection continuity is required.
    "GIMBAL_YOLO_ALIGN_MAX_FRAME_GAP_S", 0.0
)
GIMBAL_YOLO_ALIGN_MAX_LATEST_TO_MEDIAN_PX = _env_float(
    "GIMBAL_YOLO_ALIGN_MAX_LATEST_TO_MEDIAN_PX", 10.0
)
GIMBAL_YOLO_ALIGN_MAX_CENTER_JUMP_PX = _env_float(
    "GIMBAL_YOLO_ALIGN_MAX_CENTER_JUMP_PX", 0.0
)
GIMBAL_YOLO_ALIGN_MIN_BBOX_IOU = _env_float(
    "GIMBAL_YOLO_ALIGN_MIN_BBOX_IOU", 0.0
)
GIMBAL_YOLO_ALIGN_RECOVERY_CONFIRM_FRAMES = _env_int(
    "GIMBAL_YOLO_ALIGN_RECOVERY_CONFIRM_FRAMES", 1
)
GIMBAL_YOLO_ALIGN_MAX_FRAME_AGE_S = _env_float(
    "GIMBAL_YOLO_ALIGN_MAX_FRAME_AGE_S", 0.0
)
GIMBAL_LASER_READY_RADIUS_PX = _env_float(
    "GIMBAL_LASER_READY_RADIUS_PX", 20.0
)
GIMBAL_YOLO_ALIGN_MIN_FINE_STEP_DEG = _env_float(
    "GIMBAL_YOLO_ALIGN_MIN_FINE_STEP_DEG", 0.10
)
GIMBAL_YOLO_ALIGN_FINE_SCAN_MAX_ERROR_PX = _env_float(
    "GIMBAL_YOLO_ALIGN_FINE_SCAN_MAX_ERROR_PX", 0.0
)
GIMBAL_YOLO_ALIGN_MAX_STEP_AZ_DEG = _env_float(
    "GIMBAL_YOLO_ALIGN_MAX_STEP_AZ_DEG", 3.0
)
GIMBAL_YOLO_ALIGN_MAX_STEP_EL_DEG = _env_float(
    "GIMBAL_YOLO_ALIGN_MAX_STEP_EL_DEG", 2.0
)
GIMBAL_YOLO_ALIGN_MAX_CORRECTIONS = _env_int(
    "GIMBAL_YOLO_ALIGN_MAX_CORRECTIONS", 0
)
GIMBAL_VISUAL_LOCK_LOST_SECONDS = _env_float(
    "GIMBAL_VISUAL_LOCK_LOST_SECONDS", 5.0
)
GIMBAL_VISUAL_LOCK_OUTSIDE_CONFIRM_FRAMES = _env_int(
    "GIMBAL_VISUAL_LOCK_OUTSIDE_CONFIRM_FRAMES", 3
)
GIMBAL_VISUAL_LOCK_MIN_X_RATIO = _env_float(
    "GIMBAL_VISUAL_LOCK_MIN_X_RATIO", 0.05
)
GIMBAL_VISUAL_LOCK_MAX_X_RATIO = _env_float(
    "GIMBAL_VISUAL_LOCK_MAX_X_RATIO", 0.95
)
GIMBAL_VISUAL_LOCK_MIN_Y_RATIO = _env_float(
    "GIMBAL_VISUAL_LOCK_MIN_Y_RATIO", 0.05
)
GIMBAL_VISUAL_LOCK_MAX_Y_RATIO = _env_float(
    "GIMBAL_VISUAL_LOCK_MAX_Y_RATIO", 0.95
)
# SFL0603 aim point on the 2560x1440 image: (1288, 728).
GIMBAL_LASER_AIM_OFFSET_X_PX = _env_float(
    "GIMBAL_LASER_AIM_OFFSET_X_PX", 8.0
)
GIMBAL_LASER_AIM_OFFSET_Y_PX = _env_float(
    "GIMBAL_LASER_AIM_OFFSET_Y_PX", 8.0
)
SFL0603_CONTINUOUS_PERIOD_MS = _env_int(
    "SFL0603_CONTINUOUS_PERIOD_MS", 100
)
SFL0603_SESSION_TIMEOUT_SECONDS = _env_float(
    "SFL0603_SESSION_TIMEOUT_SECONDS", 1.5
)
SFL0603_READ_TIMEOUT_SECONDS = _env_float(
    "SFL0603_READ_TIMEOUT_SECONDS", 0.15
)
SFL0603_STOP_RETRIES = _env_int("SFL0603_STOP_RETRIES", 3)
SFL0603_REQUEST_HEARTBEAT_TIMEOUT_SECONDS = _env_float(
    "SFL0603_REQUEST_HEARTBEAT_TIMEOUT_SECONDS", 1.0
)
# Four absolute 0.1-degree offsets around the latest bbox center.  Between
# scan points each axis changes by no more than 0.1 degree.
SFL0603_NO_RETURN_SCAN_OFFSETS_DEG = (
    (0.1, 0.0),
    (0.0, 0.1),
    (-0.1, 0.0),
    (0.0, -0.1),
)
GIMBAL_PORT = _serial_port("GIMBAL_PORT", "gimbal")
LASER_PORT = _serial_port("LASER_PORT", "laser")
GPS_PORT = _serial_port("GPS_PORT", "gps")
USE_MOCK_GIMBAL = _env_flag("USE_MOCK_GIMBAL", False)  # True: 使用 mock_gimbal.py; False: 使用真实 GT06Z
USE_MOCK_LASER = _env_flag("USE_MOCK_LASER", False)   # True: do not open real laser; distance fusion still uses mono only.
ENABLE_GPS = _env_flag("ENABLE_GPS", True)
GPS_BAUDRATE = 115200
GPS_FIX_TIMEOUT_SECONDS = 5
GPS_STATUS_INTERVAL = 5.0
GPS_UI_SEND_INTERVAL = 10.0
GPS_DEBUG_RAW = _env_flag("GPS_DEBUG_RAW", False)
DEVICE_HEADING_DEG = _env_float("DEVICE_HEADING_DEG", 180) % 360.0  # 设备自身0度方向的地图方位：北0/东90/南180
# GIMBAL_AZ_BASE = 57.4  # 云台水平基准角（UI绝对方位 0° 映射到控制角的基准）
# GIMBAL_INIT_EL = -0.4  # 启动时俯仰归位角，目标通常从该方向进入
#测试版本基准角度
GIMBAL_AZ_BASE = 59.3  # 云台水平基准角（UI绝对方位 0° 映射到控制角的基准）
GIMBAL_INIT_EL = -0.4# 启动时俯仰归位角，目标通常从该方向进入
GIMBAL_CMD_DEADBAND_AZ = 0.20
GIMBAL_CMD_DEADBAND_EL = 0.12
AZ_PREEMPT_DEG = 0.3     # 方位轴抢占阈值，单位：度
EL_PREEMPT_DEG = 0.8      # 俯仰轴抢占阈值，单位：度
GIMBAL_SETTLE_THRESHOLD = 0.2
GIMBAL_SETTLE_TIMEOUT = 2.5
GIMBAL_SETTLE_DWELL_SECONDS = _env_float("GIMBAL_SETTLE_DWELL_SECONDS", 0.20)
GIMBAL_THREAD_SLEEP = 0.02
GIMBAL_QUERY_AFTER_CMD_DELAY = _env_float("GIMBAL_QUERY_AFTER_CMD_DELAY", 0.80)
GIMBAL_COMMAND_RETRY_INTERVAL = _env_float("GIMBAL_COMMAND_RETRY_INTERVAL", 0.90)
GIMBAL_STATIONARY_DELTA_DEG = _env_float("GIMBAL_STATIONARY_DELTA_DEG", 0.11)
GIMBAL_STATIONARY_DWELL_SECONDS = _env_float(
    "GIMBAL_STATIONARY_DWELL_SECONDS", 0.35
)
GIMBAL_PROGRESS_LOG_INTERVAL = 0.10
LASER_LOG_INTERVAL = _env_float("LASER_LOG_INTERVAL", 1.0)
LASER_RESULT_TTL = _env_float("LASER_RESULT_TTL", 1.0)
MONO_DIST_TTL = 1.2
DEFAULT_TRACKING_DISTANCE_M = 450.0  # 仅用于内部参数冷启动（保持稳定）
DEFAULT_DISTANCE_MIN_M = 200.0
DEFAULT_DISTANCE_MAX_M = 470.0
HIT_STREAK_DECAY = 1
STABILITY_HIT_CAP = 20
STABILITY_WEIGHT = 0.8
STATS_PRINT_INTERVAL = 2.0
MASTER_SELECTION_LOG_TOPK = 5
NO_PACKET_TRACKER_UPDATE_INTERVAL = 1.0 / 5.0
LOG_TO_FILE = _env_flag("LOG_TO_FILE", True)
LOG_DIR = os.getenv("LOG_DIR", "logs")
DEBUG_TRACKER = _env_flag("DEBUG_TRACKER", False)
DEBUG_KALMAN_MATCH = _env_flag("DEBUG_KALMAN_MATCH", False)
PRINT_PHASE_LOGS = _env_flag("PRINT_PHASE_LOGS", False)
PRINT_GIMBAL_PROGRESS = _env_flag("PRINT_GIMBAL_PROGRESS", False)
PRINT_EVENT_LOGS = _env_flag("PRINT_EVENT_LOGS", False)
PRINT_STATS = _env_flag("PRINT_STATS", False)
PRINT_LIVE_STATUS = _env_flag("PRINT_LIVE_STATUS", True)
LIVE_STATUS_INTERVAL = float(os.getenv("LIVE_STATUS_INTERVAL", "1.0"))
FIELD_LOG = _env_flag("FIELD_LOG", True)
FIELD_LOG_DIR = os.getenv("FIELD_LOG_DIR", LOG_DIR)
MEAS_FUSION_THRESHOLD_DEG = _env_float("MEAS_FUSION_THRESHOLD_DEG", 0.5)
MEAS_FUSION_WINDOW_SECONDS = _env_float("MEAS_FUSION_WINDOW_SECONDS", 0.20)
PACKET_QUEUE_MAXLEN = _env_int("PACKET_QUEUE_MAXLEN", 256)
TRACK_MAX_LOST_SECONDS = _env_float("TRACK_MAX_LOST_SECONDS", 5.0)  # internal anti-occlusion retention time
MAX_LOCK_LOST_SECONDS = _env_float("MAX_LOCK_LOST_SECONDS", 1.6)  # external UI/gimbal/strike lock grace time
UI_MAX_LOST_SECONDS = _env_float("UI_MAX_LOST_SECONDS", 1.0)  # hide stale predictions before the control lock grace expires
TRACK_ASSOCIATION_MAX_DEG = _env_float("TRACK_ASSOCIATION_MAX_DEG", 6.0)  # global hard cap for covariance-expanded association
TRACK_REACQUIRE_STRICT_AFTER_SECONDS = _env_float("TRACK_REACQUIRE_STRICT_AFTER_SECONDS", 1.0)
TRACK_REACQUIRE_MAX_DEG = _env_float("TRACK_REACQUIRE_MAX_DEG", 3.0)  # stricter cap after a longer detection gap
ASSOCIATION_BLOCKED_COST = 1.0e6
MAX_LOCK_LOST_FRAMES = _env_int("MAX_LOCK_LOST_FRAMES", 8)  # legacy log-only frame counter threshold
TRACK_CONFIRM_HITS = _env_int("TRACK_CONFIRM_HITS", 3)  # internal SORT/KF confirmation threshold
UI_TRACK_CONFIRM_HITS = _env_int("UI_TRACK_CONFIRM_HITS", 5)  # extra gate before exposing a UI ID
STRIKE_TRACK_CONFIRM_HITS = _env_int("STRIKE_TRACK_CONFIRM_HITS", UI_TRACK_CONFIRM_HITS)  # strike target is never exposed earlier than UI
MASTER_SWITCH_SCORE_MARGIN = _env_float("MASTER_SWITCH_SCORE_MARGIN", 2.0)
MASTER_SWITCH_CONFIRM_SECONDS = _env_float("MASTER_SWITCH_CONFIRM_SECONDS", 0.8)
STRIKE_TARGET_SWITCH_SCORE_MARGIN = _env_float("STRIKE_TARGET_SWITCH_SCORE_MARGIN", 8.0)
STRIKE_TARGET_SWITCH_CONFIRM_SECONDS = _env_float("STRIKE_TARGET_SWITCH_CONFIRM_SECONDS", 0.6)
GIMBAL_SAFE_FOV_RATIO_X = _env_float("GIMBAL_SAFE_FOV_RATIO_X", 0.60)
GIMBAL_SAFE_FOV_RATIO_Y = _env_float("GIMBAL_SAFE_FOV_RATIO_Y", 0.60)

# Gimbal-camera YOLO and SORT projection remain in native 2K coordinates.
IMG_W = _env_float("DETECTION_IMG_W", 2560.0)
IMG_H = _env_float("DETECTION_IMG_H", 1440.0)
FOV_X = 17.5
FOV_Y = 9.9
DEG_PER_PIXEL_X = FOV_X / IMG_W
DEG_PER_PIXEL_Y = FOV_Y / IMG_H

# Detection-end UDP bboxes use an explicit operator switch. Night detections
# are direct 2560x1440 -> 640x480 resizes; daytime detections stay in 2K.
USE_NIGHT_DETECTION_COORDS = _env_flag(
    "USE_NIGHT_DETECTION_COORDS", True
)
UDP_DETECTION_W = 640.0 if USE_NIGHT_DETECTION_COORDS else IMG_W
UDP_DETECTION_H = 480.0 if USE_NIGHT_DETECTION_COORDS else IMG_H
UDP_DETECTION_COORD_MODE = (
    "night_640x480" if USE_NIGHT_DETECTION_COORDS else "day_2560x1440"
)

class FieldLogger:
    def __init__(self, log_dir):
        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self.lock = threading.Lock()
        self.disabled = False
        self.last_flush_t = time.monotonic()
        self.flush_interval = 1.0
        self.raw_f = open(os.path.join(log_dir, f"raw_udp_{timestamp}.jsonl"), "a", encoding="utf-8", newline="\n")
        self.measurements_f = open(os.path.join(log_dir, f"measurements_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.summary_f = open(os.path.join(log_dir, f"track_summary_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.events_f = open(os.path.join(log_dir, f"events_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.gimbal_f = open(os.path.join(log_dir, f"gimbal_{timestamp}.csv"), "a", encoding="utf-8", newline="")

        self.measurements_fields = [
            "timestamp", "seq", "mode", "board", "cam", "logic_id", "meas_idx",
            "raw_bbox_x1", "raw_bbox_y1", "raw_bbox_x2", "raw_bbox_y2",
            "raw_bbox_w", "raw_bbox_h",
            "clipped_bbox_x1", "clipped_bbox_y1", "clipped_bbox_x2", "clipped_bbox_y2",
            "clipped_bbox_w", "clipped_bbox_h",
            "bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2", "bbox_cx", "bbox_cy",
            "bbox_w", "bbox_h", "is_edge_bbox", "visible_ratio",
            "mono_dist", "meas_az", "meas_el",
        ]
        self.summary_fields = [
            "timestamp", "seq", "mode", "dt",
            "window_packet_count", "used_packet_count",
            "same_source_packet_drop_count",
            "raw_meas_count", "fused_meas_count", "fusion_groups",
            "meas_count", "track_count",
            "valid_count", "track_ids", "valid_ids", "master_id",
            "hit_streaks", "time_since_updates", "track_states",
            "lost_seconds",
            "cmd_az", "cmd_el", "gimbal_ui_az", "gimbal_ui_el",
        ]
        self.events_fields = [
            "timestamp", "seq", "mode", "event", "track_id", "meas_idx",
            "meas_az", "meas_el", "pred_az", "map_az", "pred_el", "cost",
            "pred_az_cv", "pred_el_cv", "map_az_cv",
            "pred_az_ca", "pred_el_ca", "map_az_ca",
            "dynamic_thresh", "uncertainty", "p_az", "p_el",
            "hit_streak", "time_since_update", "reason",
            "lost_seconds",
            "internal_track_id", "ui_id",
            "master_id", "is_master",
            "distance", "distance_source",
            "dist_uncertainty", "radial_velocity",
            "threat_score",
            "detection_count", "matched_count", "unmatched_detection_count",
            "visible_track_count", "active_track_count",
            "roi_count", "sharp_roi_count",
            "matched_track_ids", "ambiguous_track_ids", "unmatched_track_ids",
            "raw_bbox_x1", "raw_bbox_y1", "raw_bbox_x2", "raw_bbox_y2",
            "clipped_bbox_x1", "clipped_bbox_y1", "clipped_bbox_x2", "clipped_bbox_y2",
            "vision_frame_ts", "vision_age", "simple_id", "class_id", "confidence",
            "bbox_cx", "bbox_cy", "center_dx_px", "center_dy_px",
            "bbox_width_px", "bbox_height_px",
            "bbox_jitter_valid", "bbox_jitter_dt_s",
            "bbox_jitter_dx_px", "bbox_jitter_dy_px",
            "bbox_jitter_center_px", "bbox_jitter_center_norm",
            "bbox_jitter_width_delta_px", "bbox_jitter_height_delta_px",
            "bbox_jitter_iou", "bbox_jitter_previous_missing_frames",
            "center_dx_norm", "center_dy_norm",
            "center_offset_az_deg", "center_offset_el_deg",
            "alignment_state", "alignment_sample_count",
            "alignment_std_x_px", "alignment_std_y_px",
            "alignment_speed_x_px_s", "alignment_speed_y_px_s",
            "alignment_latest_to_median_px", "alignment_frame_age_s",
            "alignment_fresh_after_dropout", "alignment_control_mode",
            "alignment_outlier_center_jump_px", "alignment_outlier_bbox_iou",
            "alignment_delta_az_deg", "alignment_delta_el_deg",
            "is_edge_bbox", "visible_ratio",
        ]
        self.gimbal_fields = [
            "timestamp", "event", "cmd_id", "track_id", "cmd_az", "cmd_el",
            "gimbal_ui_az", "gimbal_ui_el", "gimbal_ctrl_az", "gimbal_ctrl_el",
            "target_ctrl_az", "target_ctrl_el", "err_az", "err_el",
            "is_settled", "is_stationary", "settle_time",
            "retry_axes", "driver_status", "laser_valid", "laser_dist",
            "laser_source", "laser_ts", "laser_age", "laser_interval",
            "reason",
        ]

        self.measurements_writer = csv.DictWriter(self.measurements_f, fieldnames=self.measurements_fields, extrasaction="ignore")
        self.summary_writer = csv.DictWriter(self.summary_f, fieldnames=self.summary_fields, extrasaction="ignore")
        self.events_writer = csv.DictWriter(self.events_f, fieldnames=self.events_fields, extrasaction="ignore")
        self.gimbal_writer = csv.DictWriter(self.gimbal_f, fieldnames=self.gimbal_fields, extrasaction="ignore")
        self.measurements_writer.writeheader()
        self.summary_writer.writeheader()
        self.events_writer.writeheader()
        self.gimbal_writer.writeheader()
        print(f"[FieldLog] enabled: {os.path.abspath(log_dir)}")

    def _handle_write_error(self, exc):
        if not self.disabled:
            self.disabled = True
            print(f"[FieldLog][Warn] 写入失败，已自动停用结构化日志: {exc}")

    def _write_csv(self, writer, fields, row):
        writer.writerow({field: row.get(field, "") for field in fields})

    def _flush_if_due_locked(self):
        now = time.monotonic()
        if (now - self.last_flush_t) < self.flush_interval:
            return
        self.raw_f.flush()
        self.measurements_f.flush()
        self.summary_f.flush()
        self.events_f.flush()
        self.gimbal_f.flush()
        self.last_flush_t = now

    def write_raw_udp(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self.raw_f.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_measurement(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(self.measurements_writer, self.measurements_fields, row)
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_summary(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(self.summary_writer, self.summary_fields, row)
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_event(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(self.events_writer, self.events_fields, row)
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_gimbal(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(self.gimbal_writer, self.gimbal_fields, row)
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def flush(self):
        if self.disabled:
            return
        try:
            with self.lock:
                self.raw_f.flush()
                self.measurements_f.flush()
                self.summary_f.flush()
                self.events_f.flush()
                self.gimbal_f.flush()
                self.last_flush_t = time.monotonic()
        except Exception as e:
            self._handle_write_error(e)

    def close(self):
        with self.lock:
            for f in (self.raw_f, self.measurements_f, self.summary_f, self.events_f, self.gimbal_f):
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass


FIELD_LOGGER = None


def field_log_event(row):
    if FIELD_LOGGER is not None:
        try:
            FIELD_LOGGER.write_event(row)
        except Exception:
            pass


def field_log_gimbal(row):
    if FIELD_LOGGER is not None:
        try:
            FIELD_LOGGER.write_gimbal(row)
        except Exception:
            pass
#最终版本theta
DEVICE_THETA = {
    1: {"theta_vertical": 0.0000, "theta_horizontal": 33.4501},  # Layer 1 cam1
    2: {"theta_vertical": 0.0000, "theta_horizontal": 17.7874},  # Layer 1 cam2
    3: {"theta_vertical": 0.0000, "theta_horizontal": 1.0000},   # Layer 1 cam3
    4: {"theta_vertical": 0.0000, "theta_horizontal": 344.8421}, # Layer 1 cam4
    5: {"theta_vertical": 0.0000, "theta_horizontal": 327.6701}, # Layer 1 cam5

    6: {"theta_vertical": 5.5, "theta_horizontal": 35.3086},     # Layer 2 cam1
    7: {"theta_vertical": 5.5, "theta_horizontal": 16.5593},     # Layer 2 cam2
    8: {"theta_vertical": 5.5, "theta_horizontal": 1.0759},      # Layer 2 cam3
    9: {"theta_vertical": 5.5, "theta_horizontal": 343.7710},    # Layer 2 cam4
    10: {"theta_vertical": 5.5, "theta_horizontal": 322.0000},   # Layer 2 cam5

    11: {"theta_vertical": 15.5000, "theta_horizontal": 32.9870},  # Layer 3 cam1
    12: {"theta_vertical": 15.5000, "theta_horizontal": 20.6921},  # Layer 3 cam2
    13: {"theta_vertical": 15.5000, "theta_horizontal": 1.9703},   # Layer 3 cam3
    14: {"theta_vertical": 15.5000, "theta_horizontal": 340.9173}, # Layer 3 cam4
    15: {"theta_vertical": 15.5000, "theta_horizontal": 323.7531}, # Layer 3 cam5

    16: {"theta_vertical": 25.5000, "theta_horizontal": 37.7386},  # Layer 4 cam1
    17: {"theta_vertical": 25.5000, "theta_horizontal": 19.9455},  # Layer 4 cam2
    18: {"theta_vertical": 25.5000, "theta_horizontal": 1.7703},   # Layer 4 cam3
    19: {"theta_vertical": 25.5000, "theta_horizontal": 342.8870}, # Layer 4 cam4
    20: {"theta_vertical": 25.5000, "theta_horizontal": 324.5302}, # Layer 4 cam5

    21: {"theta_vertical": 35.0000, "theta_horizontal": 38.5403},  # Layer 5 cam1
    22: {"theta_vertical": 35.0000, "theta_horizontal": 19.5903},  # Layer 5 cam2
    23: {"theta_vertical": 35.0000, "theta_horizontal": 1.7703},   # Layer 5 cam3
    24: {"theta_vertical": 35.0000, "theta_horizontal": 341.7103}, # Layer 5 cam4
    25: {"theta_vertical": 35.0000, "theta_horizontal": 322.7703}, # Layer 5 cam5

    26: {"theta_vertical": 44.5000, "theta_horizontal": 42.7703},  # Layer 6 cam1
    27: {"theta_vertical": 44.5000, "theta_horizontal": 22.7703},  # Layer 6 cam2
    28: {"theta_vertical": 44.5000, "theta_horizontal": 2.7703},   # Layer 6 cam3
    29: {"theta_vertical": 44.5000, "theta_horizontal": 342.7703}, # Layer 6 cam4
    30: {"theta_vertical": 44.5000, "theta_horizontal": 322.7703}, # Layer 6 cam5

    31: {"theta_vertical": 54.0000, "theta_horizontal": 46.2403},  # Layer 7 cam1
    32: {"theta_vertical": 54.0000, "theta_horizontal": 25.0403},  # Layer 7 cam2
    33: {"theta_vertical": 54.0000, "theta_horizontal": 3.8703},   # Layer 7 cam3
    34: {"theta_vertical": 54.0000, "theta_horizontal": 341.6003}, # Layer 7 cam4
    35: {"theta_vertical": 54.0000, "theta_horizontal": 320.4303}, # Layer 7 cam5

    36: {"theta_vertical": 63.5000, "theta_horizontal": 48.8703},  # Layer 8 cam1
    37: {"theta_vertical": 63.5000, "theta_horizontal": 26.3703},  # Layer 8 cam2
    38: {"theta_vertical": 63.5000, "theta_horizontal": 3.8703},   # Layer 8 cam3
    39: {"theta_vertical": 63.5000, "theta_horizontal": 341.3703}, # Layer 8 cam4
    40: {"theta_vertical": 63.5000, "theta_horizontal": 318.8703}, # Layer 8 cam5

    41: {"theta_vertical": 73.0000, "theta_horizontal": 62.6703},  # Layer 9 cam1
    42: {"theta_vertical": 73.0000, "theta_horizontal": 38.6703},  # Layer 9 cam2
    43: {"theta_vertical": 73.0000, "theta_horizontal": 14.6703},  # Layer 9 cam3
    44: {"theta_vertical": 73.0000, "theta_horizontal": 350.6703}, # Layer 9 cam4
    45: {"theta_vertical": 73.0000, "theta_horizontal": 326.6703}, # Layer 9 cam5
}
# ==========================================
#  摄像头物理位置配置 (不变)
# ==========================================
# DEVICE_THETA = {
#     1: {"theta_vertical": -0.5579, "theta_horizontal": 32.4501},  # Layer 1 cam1
#     2: {"theta_vertical": -0.6494, "theta_horizontal": 16.0865},  # Layer 1 cam2
#     3: {"theta_vertical": -0.4663, "theta_horizontal": 0.0633},  # Layer 1 cam3
#     4: {"theta_vertical": 0.0000, "theta_horizontal": 343.8421},  # Layer 1 cam4
#     5: {"theta_vertical": 0.0000, "theta_horizontal": 326.6701},  # Layer 1 cam5
#     6: {"theta_vertical": 9.6803, "theta_horizontal": 36.7904},  # Layer 2 cam1
#     7: {"theta_vertical": 9.7292, "theta_horizontal": 18.1429},  # Layer 2 cam2
#     8: {"theta_vertical": 9.3961, "theta_horizontal": 2.6352},  # Layer 2 cam3
#     9: {"theta_vertical": 9.3992, "theta_horizontal": 345.2944},  # Layer 2 cam4
#     10: {"theta_vertical": 9.4304, "theta_horizontal": 328.5462},  # Layer 2 cam5
#     11: {"theta_vertical": 19.6400, "theta_horizontal": 34.9611},  # Layer 3 cam1
#     12: {"theta_vertical": 20.1031, "theta_horizontal": 18.3246},  # Layer 3 cam2
#     13: {"theta_vertical": 17.1175, "theta_horizontal": 355.5312},  # Layer 3 cam3
#     14: {"theta_vertical": 18.5912, "theta_horizontal": 343.6730},  # Layer 3 cam4
#     15: {"theta_vertical": 18.3194, "theta_horizontal": 327.5031},  # Layer 3 cam5
#     16: {"theta_vertical": 28.2994, "theta_horizontal": 37.9953},  # Layer 4 cam1
#     17: {"theta_vertical": 28.2763, "theta_horizontal": 18.9455},  # Layer 4 cam2
#     18: {"theta_vertical": 28.2687, "theta_horizontal": 0.4566},  # Layer 4 cam3
#     19: {"theta_vertical": 28.6994, "theta_horizontal": 342.0422},  # Layer 4 cam4
#     20: {"theta_vertical": 28.6831, "theta_horizontal": 323.3621},  # Layer 4 cam5
#     21: {"theta_vertical": 39.4600, "theta_horizontal": 41.3170},  # Layer 5 cam1
#     22: {"theta_vertical": 39.0169, "theta_horizontal": 21.8764},  # Layer 5 cam2
#     23: {"theta_vertical": 38.4706, "theta_horizontal": 1.6039},  # Layer 5 cam3
#     24: {"theta_vertical": 39.3731, "theta_horizontal": 342.4729},  # Layer 5 cam4
#     25: {"theta_vertical": 38.6506, "theta_horizontal": 323.5217},  # Layer 5 cam5
#     26: {"theta_vertical": 47.5837, "theta_horizontal": 41.5262},  # Layer 6 cam1
#     27: {"theta_vertical": 48.3719, "theta_horizontal": 23.2072},  # Layer 6 cam2
#     28: {"theta_vertical": 47.9000, "theta_horizontal": 3.6703},  # Layer 6 cam3
#     29: {"theta_vertical": 47.4831, "theta_horizontal": 341.4219},  # Layer 6 cam4
#     30: {"theta_vertical": 47.4894, "theta_horizontal": 322.8021},  # Layer 6 cam5
#     31: {"theta_vertical": 57.3831, "theta_horizontal": 44.0619},  # Layer 7 cam1
#     32: {"theta_vertical": 57.6294, "theta_horizontal": 20.6004},  # Layer 7 cam2
#     33: {"theta_vertical": 56.9069, "theta_horizontal": 4.2049},  # Layer 7 cam3
#     34: {"theta_vertical": 57.2212, "theta_horizontal": 342.2857},  # Layer 7 cam4
#     35: {"theta_vertical": 57.3381, "theta_horizontal": 319.9652},  # Layer 7 cam5
#     36: {"theta_vertical": 66.9506, "theta_horizontal": 45.4904},  # Layer 8 cam1
#     37: {"theta_vertical": 66.2794, "theta_horizontal": 28.2662},  # Layer 8 cam2
#     38: {"theta_vertical": 66.9000, "theta_horizontal": 4.7703},  # Layer 8 cam3
#     39: {"theta_vertical": 67.2738, "theta_horizontal": 337.8605},  # Layer 8 cam4
#     40: {"theta_vertical": 66.9719, "theta_horizontal": 318.2523},  # Layer 8 cam5
#     41: {"theta_vertical": 76.4000, "theta_horizontal": 63.5703},  # Layer 9 cam1
#     42: {"theta_vertical": 76.4000, "theta_horizontal": 39.5703},  # Layer 9 cam2
#     43: {"theta_vertical": 76.4000, "theta_horizontal": 15.5703},  # Layer 9 cam3
#     44: {"theta_vertical": 76.4000, "theta_horizontal": 351.5703},  # Layer 9 cam4
#     45: {"theta_vertical": 76.4000, "theta_horizontal": 327.5703},  # Layer 9 cam5
# }

# ==========================================
# 硬件映射表 (不变)
# ========================================== 
HARDWARE_MAP = {
    ("BOARD_1", 0): 1, ("BOARD_1", 1): 2, ("BOARD_1", 2): 3, ("BOARD_1", 3): 4, ("BOARD_1", 4): 5,
    ("BOARD_2", 0): 6, ("BOARD_2", 1): 7, ("BOARD_2", 2): 8, ("BOARD_2", 3): 9, ("BOARD_2", 4): 10,  
    ("BOARD_3", 0): 11, ("BOARD_3", 1): 12, ("BOARD_3", 2): 13, ("BOARD_3", 3): 14, ("BOARD_3", 4): 15,
    ("BOARD_4", 0): 16, ("BOARD_4", 1): 17, ("BOARD_4", 2): 18, ("BOARD_4", 3): 19, ("BOARD_4", 4): 20,
    ("BOARD_5", 0): 21, ("BOARD_5", 1): 22, ("BOARD_5", 2): 23, ("BOARD_5", 3): 24, ("BOARD_5", 4): 25,
    ("BOARD_6", 0): 26, ("BOARD_6", 1): 27, ("BOARD_6", 2): 28, ("BOARD_6", 3): 29, ("BOARD_6", 4): 30,
    ("BOARD_7", 0): 31 , ("BOARD_7", 1): 32, ("BOARD_7", 2): 33, ("BOARD_7", 3): 34, ("BOARD_7", 4): 35,
    ("BOARD_8", 0): 36, ("BOARD_8", 1): 37, ("BOARD_8", 2): 38, ("BOARD_8", 3): 39, ("BOARD_8", 4): 40,
    ("BOARD_9", 0): 41, ("BOARD_9", 1): 42, ("BOARD_9", 2): 43, ("BOARD_9", 3): 44, ("BOARD_9", 4): 45,
}

# ==========================================
# 解析与计算函数
# ==========================================
def normalize_board_id(board_id):
    """Normalize detector board names: board1/BOARD1/board_1 -> BOARD_1."""
    s = str(board_id).strip()
    if not s:
        return s
    compact = s.replace("_", "").upper()
    if compact.startswith("BOARD") and compact[5:].isdigit():
        return f"BOARD_{int(compact[5:])}"
    return s.upper()


def get_camera_params(board_id, cam_idx):
    norm_board_id = normalize_board_id(board_id)
    key = (norm_board_id, int(cam_idx))
    if key not in HARDWARE_MAP:
        print(
            f"[Warning] Unknown hardware mapping: Board={board_id} "
            f"(normalized={norm_board_id}), Cam={cam_idx}"
        )
        return None, None
    logic_id = HARDWARE_MAP[key]
    if logic_id not in DEVICE_THETA:
        print(f"[Warning] 逻辑ID {logic_id} 没有配置偏差数据")
        return None, None
    cfg = DEVICE_THETA[logic_id]
    return logic_id, cfg

# ==========================================
# 网络发送类 (UI)
# ==========================================
class UISender:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lock = threading.Lock()
        self.MSG_STATUS = 0x02
        self.MSG_GPS = 0x03

    def send_status(self, board_str, camera_id, target_id, azimuth, elevation, distance, threat_score=float("nan")):
            try:
                if not isinstance(board_str, str):
                    board_str = str(board_str)
                board_bytes = board_str.encode('utf-8')

                packet = struct.pack(
                    '!BB8sIffff',
                    self.MSG_STATUS,
                    int(camera_id),
                    board_bytes,
                    int(target_id),
                    float(azimuth),
                    float(elevation),
                    float(distance),
                    float(threat_score)
                )
                with self.lock:
                    self.sock.sendto(packet, (self.ip, self.port))
            except Exception as e:
                print(f"[Sender] Error: {e}")

    def send_gps_location(self, latitude, longitude):
            try:
                packet = struct.pack(
                    '!Bff',
                    self.MSG_GPS,
                    float(latitude),
                    float(longitude),
                )
                with self.lock:
                    self.sock.sendto(packet, (self.ip, self.port))
            except Exception as e:
                print(f"[Sender][GPS] Error: {e}")

# ==========================================
# 网络发送类 (打击端主控板)
# ==========================================
class StrikeSender:
    FRAME_HEAD = b"\xAA\x55"
    FRAME_TAIL = b"\x55\xAA"
    FRAME_LENGTH = 0x0D
    MIN_ELEVATION_DEG = -35.0
    MAX_ELEVATION_DEG = 60.0

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lock = threading.Lock()

    @staticmethod
    def _xor_checksum(data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    @classmethod
    def build_packet(cls, target_id, distance_m, azimuth_deg, elevation_deg):
        target_id = int(target_id)
        if target_id < 0 or target_id > 0xFF:
            raise ValueError(f"strike target_id out of uint8 range: {target_id}")

        distance_raw = int(round(float(distance_m) * 10.0))
        if distance_raw <= 0 or distance_raw > 0xFFFF:
            raise ValueError(f"strike distance out of uint16/0.1m range: {distance_m}")

        az_raw = int(round((float(azimuth_deg) % 360.0) * 10.0))
        if az_raw >= 3600:
            az_raw = 0

        elevation_deg = float(elevation_deg)
        if elevation_deg < cls.MIN_ELEVATION_DEG or elevation_deg > cls.MAX_ELEVATION_DEG:
            raise ValueError(f"strike elevation out of range: {elevation_deg}")
        el_raw = int(round(elevation_deg * 10.0))

        body = struct.pack(
            "!2sBBHHh",
            cls.FRAME_HEAD,
            cls.FRAME_LENGTH,
            target_id,
            distance_raw,
            az_raw,
            el_raw,
        )
        return body + bytes([cls._xor_checksum(body)]) + cls.FRAME_TAIL

    def send_target(self, target_id, distance_m, azimuth_deg, elevation_deg):
        packet = self.build_packet(target_id, distance_m, azimuth_deg, elevation_deg)
        with self.lock:
            self.sock.sendto(packet, (self.ip, self.port))
        return packet

# ==========================================
# 3. 激光与网络
# ==========================================
class SharedHardwareState:
    def __init__(self):
        self.lock = threading.Lock()
        self.gimbal_az = 0.0  # UI坐标系方位角
        self.gimbal_el = 0.0
        self.gimbal_att_ts = 0.0
        self.active_cmd_id = -1
        self.active_track_id = -1
        self.settled_cmd_id = -1
        self.settled_track_id = -1
        self.settled_ts = 0.0
        self.is_settled = False
        # Physical camera stability is independent of reaching the commanded
        # angle.  Ranging uses this flag; strike safety still uses is_settled.
        self.is_stationary = False
        self.stationary_ts = 0.0
        self.raw_laser_dist = None
        self.raw_laser_ts = 0.0
        self.raw_laser_request_ts = 0.0
        self.raw_laser_track_id = -1
        # SFL0603 is normally in standby. A positive request timestamp denotes
        # one aligned-target session. It remains at 10 Hz while the alignment
        # heartbeat is present, independent of the returned distance value.
        self.laser_request_ts = 0.0
        self.laser_request_track_id = -1
        self.laser_request_heartbeat_ts = 0.0
        self.laser_active = False
        self.laser_status = "UNAVAILABLE"
        self.laser_last_error = ""
        self.laser_no_return_ts = 0.0

shared_state = SharedHardwareState()
gimbal_cmd_queue = queue.Queue(maxsize=1)
packet_queue = deque(maxlen=PACKET_QUEUE_MAXLEN)


def quantize_gimbal_command_angle(value):
    """Normalize every queued gimbal target to the protocol's 0.1 degree."""
    if quantize_angle_0p1 is not None:
        return quantize_angle_0p1(value)
    value = float(value)
    magnitude = math.floor(abs(value) * 10.0 + 0.5) / 10.0
    return math.copysign(magnitude, value)


def sample_default_distance():
    """兜底距离采样：用于无激光、无单目时的保底值。"""
    return random.uniform(DEFAULT_DISTANCE_MIN_M, DEFAULT_DISTANCE_MAX_M)

def rk3588_thread():
    print(f"[Net] Listening RK3588 on {LOCAL_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    try:
        sock.bind(("0.0.0.0", LOCAL_PORT))
    except OSError as e:
        print(f"[Net][Fatal] 端口 {LOCAL_PORT} 绑定失败: {e}")
        return

    err_decode = 0
    err_json = 0
    err_sock = 0
    err_other = 0

    while True:
        try:
            data, addr = sock.recvfrom(65535)
            recv_ts = time.time()
            pkg = json.loads(data.decode("utf-8"))
            if isinstance(pkg, dict) and "objs" in pkg:
                pkg["_recv_ts"] = recv_ts
                if FIELD_LOGGER is not None:
                    FIELD_LOGGER.write_raw_udp({
                        "recv_ts": recv_ts,
                        "addr": f"{addr[0]}:{addr[1]}",
                        "packet_len": len(data),
                        "seq": pkg.get("seq", ""),
                        "mode": pkg.get("mode", ""),
                        "board": pkg.get("board", ""),
                        "cam": pkg.get("cam", ""),
                        "raw_obj_count": len(pkg.get("objs", [])) if isinstance(pkg.get("objs", []), list) else "",
                        "raw_objs": pkg.get("objs", []),
                    })
                packet_queue.append(pkg)

        except socket.timeout:
            continue

        except UnicodeDecodeError as e:
            err_decode += 1
            if err_decode % 50 == 1:
                print(f"[Net][DecodeError] count={err_decode}, err={e}")
            continue

        except json.JSONDecodeError as e:
            err_json += 1
            if err_json % 50 == 1:
                print(f"[Net][JSONError] count={err_json}, err={e}")
            continue

        except OSError as e:
            err_sock += 1
            if err_sock % 10 == 1:
                print(f"[Net][SocketError] count={err_sock}, 网卡/底层异常: {e}")
            time.sleep(0.5)
            continue

        except Exception as e:
            err_other += 1
            print(f"[Net][Unexpected] count={err_other}, type={type(e).__name__}, err={e}")
            continue

def push_latest_gimbal_cmd(cmd):
    cmd = dict(cmd)
    cmd["az"] = quantize_gimbal_command_angle(cmd["az"])
    cmd["el"] = quantize_gimbal_command_angle(cmd["el"])
    while True:
        try:
            gimbal_cmd_queue.put_nowait(cmd)
            return True
        except queue.Full:
            try:
                gimbal_cmd_queue.get_nowait()
            except queue.Empty:
                return False

def drain_latest_gimbal_cmd():
    latest_cmd = None
    while True:
        try:
            latest_cmd = gimbal_cmd_queue.get_nowait()
        except queue.Empty:
            break
    return latest_cmd

def _parse_positive_float(value):
    try:
        f = float(value)
        if f > 0:
            return f
    except (TypeError, ValueError):
        pass
    return None

def laser_reader_thread(laser, stop_event):
    """Run SFL0603 at 10 Hz while the target request remains live."""
    print("[LaserThread] SFL0603 target-continuous reader started in standby")
    with shared_state.lock:
        shared_state.laser_status = "STANDBY"
        shared_state.laser_last_error = ""
    active_request_ts = 0.0
    active_track_id = -1
    active_started_mono = 0.0
    completed_request_ts = 0.0

    def stop_active(reason, final_status="STANDBY"):
        nonlocal active_request_ts, active_track_id, active_started_mono
        if active_request_ts <= 0.0:
            return True
        stopped = False
        stop_error = None
        for attempt in range(1, max(1, SFL0603_STOP_RETRIES) + 1):
            try:
                laser.stop_measurement()
                stopped = True
                stop_error = None
                break
            except Exception as exc:
                stop_error = exc
                print(
                    f"[Laser][Warn] SFL0603 stop attempt {attempt}/"
                    f"{max(1, SFL0603_STOP_RETRIES)} failed: {exc}"
                )
                time.sleep(0.03)
        try:
            laser.disarm_ranging(stop=False)
        except Exception as exc:
            print(f"[Laser][Warn] SFL0603 disarm failed: {exc}")
        with shared_state.lock:
            shared_state.laser_active = False
            shared_state.laser_status = (
                final_status if stopped else "STOP_FAILED"
            )
            shared_state.laser_last_error = (
                "" if stopped else str(stop_error or "unknown stop failure")
            )
            shared_state.laser_no_return_ts = 0.0
        now_t = time.time()
        field_log_gimbal({
            "timestamp": f"{now_t:.6f}",
            "event": "LASER_SESSION_STOP" if stopped else "LASER_SESSION_STOP_FAILED",
            "track_id": active_track_id,
            "laser_source": "sfl0603",
            "reason": (
                f"{reason},request_ts={active_request_ts:.6f}"
                if stopped else
                f"{reason},request_ts={active_request_ts:.6f},error={stop_error}"
            ),
        })
        active_request_ts = 0.0
        active_track_id = -1
        active_started_mono = 0.0
        if not stopped:
            # close() performs one final best-effort STOP before releasing the
            # port. Do not permit another emission session after stop failure.
            try:
                laser.close()
            except Exception as exc:
                print(f"[Laser][Fatal] SFL0603 close after stop failure: {exc}")
        return stopped

    try:
        while not stop_event.is_set():
            with shared_state.lock:
                requested_ts = float(shared_state.laser_request_ts or 0.0)
                requested_track_id = int(shared_state.laser_request_track_id)
                request_heartbeat_ts = float(
                    shared_state.laser_request_heartbeat_ts or 0.0
                )
            request_heartbeat_fresh = (
                request_heartbeat_ts > 0.0
                and 0.0
                <= time.time() - request_heartbeat_ts
                <= SFL0603_REQUEST_HEARTBEAT_TIMEOUT_SECONDS
            )
            if not request_heartbeat_fresh:
                requested_ts = 0.0
                requested_track_id = -1

            if (
                active_request_ts > 0.0
                and (
                    requested_ts <= 0.0
                    or abs(requested_ts - active_request_ts) > 1e-6
                    or requested_track_id != active_track_id
                )
            ):
                if not stop_active(
                    "request_cancelled_or_changed", "REQUEST_CANCELLED"
                ):
                    return
                continue

            if requested_ts <= 0.0:
                stop_event.wait(0.02)
                continue

            if (
                active_request_ts <= 0.0
                and abs(requested_ts - completed_request_ts) <= 1e-6
            ):
                stop_event.wait(0.02)
                continue

            if active_request_ts <= 0.0:
                try:
                    laser.arm_ranging()
                    laser.start_continuous(
                        period_ms=SFL0603_CONTINUOUS_PERIOD_MS
                    )
                except Exception as exc:
                    try:
                        laser.disarm_ranging(stop=True)
                    except Exception:
                        pass
                    completed_request_ts = requested_ts
                    print(f"[Laser][Error] SFL0603 session start failed: {exc}")
                    with shared_state.lock:
                        shared_state.laser_active = False
                        shared_state.laser_status = "START_ERROR"
                        shared_state.laser_last_error = str(exc)
                    field_log_gimbal({
                        "timestamp": f"{time.time():.6f}",
                        "event": "LASER_SESSION_START_FAILED",
                        "track_id": requested_track_id,
                        "laser_source": "sfl0603",
                        "reason": str(exc),
                    })
                    stop_event.wait(0.05)
                    continue
                active_request_ts = requested_ts
                active_track_id = requested_track_id
                active_started_mono = time.monotonic()
                with shared_state.lock:
                    shared_state.laser_active = True
                    shared_state.laser_status = "RANGING_10HZ"
                    shared_state.laser_last_error = ""
                    shared_state.raw_laser_dist = None
                    shared_state.raw_laser_ts = 0.0
                    shared_state.raw_laser_request_ts = 0.0
                    shared_state.raw_laser_track_id = -1
                    shared_state.laser_no_return_ts = 0.0
                field_log_gimbal({
                    "timestamp": f"{time.time():.6f}",
                    "event": "LASER_SESSION_START",
                    "track_id": active_track_id,
                    "laser_source": "sfl0603",
                    "reason": (
                        f"aligned_request_ts={active_request_ts:.6f},"
                        f"period_ms={SFL0603_CONTINUOUS_PERIOD_MS}"
                    ),
                })

            if (
                time.monotonic() - active_started_mono
                >= SFL0603_SESSION_TIMEOUT_SECONDS
            ):
                no_return_t = time.time()
                active_started_mono = time.monotonic()
                with shared_state.lock:
                    shared_state.laser_status = "RANGING_NO_VALID_RETURN"
                    shared_state.laser_no_return_ts = no_return_t
                field_log_gimbal({
                    "timestamp": f"{no_return_t:.6f}",
                    "event": "LASER_NO_VALID_RETURN_SCAN_REQUEST",
                    "track_id": active_track_id,
                    "laser_source": "sfl0603",
                    "reason": (
                        f"continuous_10hz_kept_active,"
                        f"request_ts={active_request_ts:.6f}"
                    ),
                })
                continue

            try:
                measurement = laser.read_measurement(
                    timeout=SFL0603_READ_TIMEOUT_SECONDS
                )
            except SFL0603ResponseTimeoutError:
                continue
            except Exception as exc:
                completed_request_ts = active_request_ts
                print(f"[Laser][Error] SFL0603 read failed: {exc}")
                if not stop_active(f"read_error={exc}", "READ_ERROR"):
                    return
                with shared_state.lock:
                    shared_state.laser_last_error = str(exc)
                continue

            if not measurement.valid:
                with shared_state.lock:
                    shared_state.laser_status = "INVALID_RETURN_CONTINUE"
                field_log_gimbal({
                    "timestamp": f"{time.time():.6f}",
                    "event": "LASER_MEASUREMENT_REJECTED",
                    "track_id": active_track_id,
                    "laser_source": "sfl0603",
                    "laser_valid": 0,
                    "reason": f"flags=0x{measurement.flags.raw:02X}",
                })
                continue

            result_request_ts = active_request_ts
            result_track_id = active_track_id
            distance_m = float(measurement.target_1_m)
            now_t = time.time()
            with shared_state.lock:
                shared_state.raw_laser_dist = distance_m
                shared_state.raw_laser_ts = now_t
                shared_state.raw_laser_request_ts = result_request_ts
                shared_state.raw_laser_track_id = result_track_id
                shared_state.laser_no_return_ts = 0.0
            # Any valid return proves that the link is producing data. Keep
            # the same continuous 10 Hz session alive while bbox alignment
            # continues, regardless of the distance value.
            active_started_mono = time.monotonic()
            with shared_state.lock:
                shared_state.laser_status = "RANGING_VALID_RETURN"
            field_log_gimbal({
                "timestamp": f"{now_t:.6f}",
                "event": "LASER_VALID_RESULT_CONTINUE",
                "track_id": result_track_id,
                "laser_valid": 1,
                "laser_dist": f"{distance_m:.6f}",
                "laser_source": "sfl0603",
                "laser_ts": f"{now_t:.6f}",
                "reason": (
                    f"aligned_request_ts={result_request_ts:.6f},stopped=0"
                ),
            })
            continue
    finally:
        if active_request_ts > 0.0:
            stop_active("reader_shutdown", "SHUTDOWN")


def gps_sender_thread(sender):
    if read_gps_fix is None:
        print("[GPS][Warn] gps.py import failed, GPS location packet disabled.")
        return
    if DEFAULT_LATITUDE is None or DEFAULT_LONGITUDE is None:
        print("[GPS][Warn] default GPS location missing, GPS location packet disabled.")
        return

    print(
        f"[GPS] Thread started, periodic UI updates enabled on "
        f"{GPS_PORT}@{GPS_BAUDRATE}, fix_timeout={GPS_FIX_TIMEOUT_SECONDS}s, "
        f"ui_interval={GPS_UI_SEND_INTERVAL}s"
    )

    last_sent_latitude = None
    last_sent_longitude = None
    last_sent_source = "default"

    while True:
        cycle_start = time.monotonic()
        #print("[GPS] Searching satellites and waiting for valid latitude/longitude...")
        longitude, latitude, source = read_gps_fix(
            port=GPS_PORT,
            baudrate=GPS_BAUDRATE,
            timeout_seconds=GPS_FIX_TIMEOUT_SECONDS,
            print_raw=GPS_DEBUG_RAW,
            print_status=True,
            status_interval=GPS_STATUS_INTERVAL,
        )

        send_source = source
        if longitude is not None and latitude is not None:
            last_sent_longitude = longitude
            last_sent_latitude = latitude
            last_sent_source = source
        elif last_sent_longitude is not None and last_sent_latitude is not None:
            longitude = last_sent_longitude
            latitude = last_sent_latitude
            send_source = f"cached:{last_sent_source}"
        else:
            longitude = DEFAULT_LONGITUDE
            latitude = DEFAULT_LATITUDE
            send_source = "default"

        sender.send_gps_location(latitude=latitude, longitude=longitude)
        # print(
        #     f"[GPS] Sent location to UI: "
        #     f"source={send_source}, latitude={latitude:.6f}, longitude={longitude:.6f}"
        # )

        sleep_time = GPS_UI_SEND_INTERVAL - (time.monotonic() - cycle_start)
        if sleep_time > 0:
            time.sleep(sleep_time)


def parse_udp_objects(raw_objs):
    """
    归一化 UDP 目标列表，输出:
        [{"box": [x1, y1, x2, y2], "mono_dist": None,
          "cam": int|None, "board": str|None}, ...]

    支持格式:
    1) [x1, y1, x2, y2]
    2) [x1, y1, x2, y2, ...]  # extra values are ignored
    3) {"box":[x1,y1,x2,y2]}
    4) {"boxes":[[...],[...]]} (多坐标批量)

    检测端距离字段已停用；本端只使用云台 YOLO+测距模型更新距离。
    """
    parsed = []

    def append_obj(box, mono_dist=None, cam=None, board=None):
        parsed.append({
            "box": [box[0], box[1], box[2], box[3]],
            "mono_dist": None,
            "cam": cam,
            "board": board,
        })

    def first_present(obj, keys):
        for key in keys:
            if key in obj and obj.get(key) not in (None, ""):
                return obj.get(key)
        return None

    # 兼容单目标扁平格式:
    # objs = [x1, y1, x2, y2] / [x1, y1, x2, y2, dist]
    if isinstance(raw_objs, (list, tuple)) and len(raw_objs) >= 4 and not isinstance(raw_objs[0], (list, tuple, dict)):
        return [{"box": [raw_objs[0], raw_objs[1], raw_objs[2], raw_objs[3]], "mono_dist": None, "cam": None, "board": None}]

    # 兼容单目标字典:
    # objs = {"box":[...], "distance":...}
    if isinstance(raw_objs, dict):
        raw_objs = [raw_objs]

    if not isinstance(raw_objs, list):
        return parsed

    for obj_item in raw_objs:
        if isinstance(obj_item, dict):
            obj_cam = first_present(obj_item, ("cam", "cam_id", "camera", "camera_id", "cameraId"))
            obj_board = first_present(obj_item, ("board", "board_id", "boardId"))

            # 批量 boxes: {"boxes":[...]}；若带 distances 也忽略。
            boxes = obj_item.get("boxes", None)
            if isinstance(boxes, list):
                for b in boxes:
                    if not isinstance(b, (list, tuple)) or len(b) < 4:
                        continue
                    append_obj([b[0], b[1], b[2], b[3]], None, obj_cam, obj_board)
                continue

            # 单目标 box
            box = obj_item.get("box", None)
            if isinstance(box, (list, tuple)):
                # 兼容 {"box":[[...],[...]], ...}
                if len(box) > 0 and isinstance(box[0], (list, tuple)):
                    for b in box:
                        if isinstance(b, (list, tuple)) and len(b) >= 4:
                            append_obj([b[0], b[1], b[2], b[3]], None, obj_cam, obj_board)
                elif len(box) >= 4:
                    append_obj([box[0], box[1], box[2], box[3]], None, obj_cam, obj_board)
                continue

            # 兼容坐标键值形式
            if all(k in obj_item for k in ("x1", "y1", "x2", "y2")):
                append_obj(
                    [obj_item["x1"], obj_item["y1"], obj_item["x2"], obj_item["y2"]],
                    None,
                    obj_cam,
                    obj_board,
                )
                continue
            if all(k in obj_item for k in ("x", "y", "w", "h")):
                x = float(obj_item["x"])
                y = float(obj_item["y"])
                w = float(obj_item["w"])
                h = float(obj_item["h"])
                append_obj([x, y, x + w, y + h], None, obj_cam, obj_board)
                continue

        elif isinstance(obj_item, (list, tuple)):
            # 单目标: [x1, y1, x2, y2, (optional)dist]
            if len(obj_item) >= 4 and not isinstance(obj_item[0], (list, tuple, dict)):
                append_obj([obj_item[0], obj_item[1], obj_item[2], obj_item[3]], None)
                continue

            # 批量: [[x1,y1,x2,y2], [..], ...]
            if len(obj_item) > 0 and isinstance(obj_item[0], (list, tuple)):
                for b in obj_item:
                    if isinstance(b, (list, tuple)) and len(b) >= 4:
                        append_obj([b[0], b[1], b[2], b[3]], None)
                continue

    return parsed


def sanitize_bbox(rect, image_w=IMG_W, image_h=IMG_H):
    try:
        x1, y1, x2, y2 = (float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3]))
    except (TypeError, ValueError, IndexError):
        return None, "non_numeric_bbox"

    raw_w = x2 - x1
    raw_h = y2 - y1
    if raw_w <= 0 or raw_h <= 0:
        return None, "invalid_raw_bbox"

    image_w = float(image_w)
    image_h = float(image_h)
    clipped_x1 = min(max(x1, 0.0), image_w)
    clipped_y1 = min(max(y1, 0.0), image_h)
    clipped_x2 = min(max(x2, 0.0), image_w)
    clipped_y2 = min(max(y2, 0.0), image_h)
    clipped_w = clipped_x2 - clipped_x1
    clipped_h = clipped_y2 - clipped_y1
    if clipped_w <= 0 or clipped_h <= 0:
        return None, "invalid_clipped_bbox"

    raw_area = raw_w * raw_h
    clipped_area = clipped_w * clipped_h
    is_edge_bbox = (
        x1 < 0.0 or y1 < 0.0 or x2 > image_w or y2 > image_h
    )
    return {
        "raw": [x1, y1, x2, y2],
        "clipped": [clipped_x1, clipped_y1, clipped_x2, clipped_y2],
        "raw_w": raw_w,
        "raw_h": raw_h,
        "clipped_w": clipped_w,
        "clipped_h": clipped_h,
        "visible_ratio": clipped_area / raw_area,
        "is_edge_bbox": is_edge_bbox,
    }, ""


def get_smoothed_track_distance(track, curr_time, ttl=TRACK_DISTANCE_TTL):
    if track is None or track.dist_state is None:
        return None, "none"
    age = float(curr_time) - float(track.last_dist_ts)
    if age < 0.0 or age > float(ttl):
        return None, "stale_distance_kf"
    return float(track.dist_state[0, 0]), f"{track.dist_source}_smooth"


def select_track_distance(track, master_id, curr_time):
    distance, source = get_smoothed_track_distance(track, curr_time)
    if distance is None:
        return float("nan"), source
    return distance, source


def select_strike_distance(track, curr_time, strike_window):
    return get_smoothed_track_distance(track, curr_time)



def clear_strike_window(strike_window):
    strike_window["track_id"] = None
    strike_window["distance"] = None
    strike_window["source"] = "none"
    strike_window["valid_until"] = 0.0
    strike_window["last_send_ts"] = 0.0
    strike_window["last_consumed_settled_cmd_id"] = -1
import numpy as np
from scipy.optimize import linear_sum_assignment

# ==========================================
# 新增模块 1：角度计算工具
# ==========================================
def angular_diff(target, source):
    """计算两个绝对角度之间的最短物理距离 (-180 到 180度)"""
    return (target - source + 180.0) % 360.0 - 180.0


def circular_mean_deg(values):
    if not values:
        return 0.0
    sin_sum = sum(math.sin(math.radians(v)) for v in values)
    cos_sum = sum(math.cos(math.radians(v)) for v in values)
    if abs(sin_sum) < 1e-12 and abs(cos_sum) < 1e-12:
        return float(values[0]) % 360.0
    return math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0


def angle_measurement_distance(a, b):
    return math.hypot(
        angular_diff(float(a["az"]), float(b["az"])),
        float(a["el"]) - float(b["el"]),
    )


def measurement_source_key(item):
    logic_id = item.get("logic_id")
    if logic_id not in (None, ""):
        try:
            return ("logic", int(logic_id))
        except (TypeError, ValueError):
            return ("logic", str(logic_id))
    return ("board_cam", str(item.get("board", "")), str(item.get("cam", "")))


def fusion_group_has_source(group_items, meas):
    source_key = measurement_source_key(meas)
    return any(measurement_source_key(item) == source_key for item in group_items)


def build_fused_measurement(items):
    az_values = [float(item["az"]) for item in items]
    el_values = [float(item["el"]) for item in items]
    mono_values = []
    for item in items:
        mono_dist = _parse_positive_float(item.get("mono_dist"))
        if mono_dist is not None:
            mono_values.append(mono_dist)
    primary_item = max(
        items,
        key=lambda item: float(item.get("source_ts", 0.0) or 0.0),
    )
    fused = {
        "az": circular_mean_deg(az_values),
        "el": sum(el_values) / len(el_values),
        "mono_dist": (sum(mono_values) / len(mono_values)) if mono_values else None,
        "source_ts": float(primary_item.get("source_ts", 0.0) or 0.0),
        "source_meas_indices": [int(item.get("raw_meas_idx", idx)) for idx, item in enumerate(items)],
        "source_boards": [str(item.get("board", "")) for item in items],
        "source_cams": [str(item.get("cam", "")) for item in items],
        "source_logic_ids": [str(item.get("logic_id", "")) for item in items],
        "is_edge_bbox": any(bool(item.get("is_edge_bbox", False)) for item in items),
    }
    visible_values = []
    for item in items:
        try:
            visible_values.append(float(item.get("visible_ratio")))
        except (TypeError, ValueError):
            pass
    if visible_values:
        fused["visible_ratio"] = min(visible_values)
        fused["source_visible_ratios"] = visible_values
    if items:
        fused["board"] = primary_item.get("board")
        fused["cam"] = primary_item.get("cam")
        fused["logic_id"] = primary_item.get("logic_id")
    return fused


def fuse_measurements_by_angle(measurements, threshold_deg=MEAS_FUSION_THRESHOLD_DEG):
    """
    同一帧内的跨相机重复观测先在角度空间融合，再进入 SORT/Kalman。
    这里仅做帧内去重，不跨帧维护状态，避免改动 tracker 主体逻辑。
    """
    if not measurements:
        return [], []
    if threshold_deg <= 0:
        fused = []
        groups = []
        for i, meas in enumerate(measurements):
            item = dict(meas)
            item["source_meas_indices"] = [int(meas.get("raw_meas_idx", i))]
            item["source_cams"] = [str(meas.get("cam", ""))]
            fused.append(item)
            groups.append([item])
        return fused, groups

    groups = []
    for meas in measurements:
        best_idx = None
        best_dist = None
        for idx, group in enumerate(groups):
            # 只允许不同摄像头 ID 的观测互相融合。同一画面中的
            # 多个目标即使角度很近，也必须保留为独立观测交给 SORT。
            if fusion_group_has_source(group["items"], meas):
                continue
            dist = angle_measurement_distance(meas, group["center"])
            if dist <= threshold_deg and (best_dist is None or dist < best_dist):
                best_idx = idx
                best_dist = dist

        if best_idx is None:
            groups.append({"items": [dict(meas)], "center": dict(meas)})
            continue

        group = groups[best_idx]
        group["items"].append(dict(meas))
        group["center"] = build_fused_measurement(group["items"])

    fused = [build_fused_measurement(group["items"]) for group in groups]
    return fused, [group["items"] for group in groups]


def format_fusion_groups(groups):
    parts = []
    for idx, group in enumerate(groups):
        raw_indices = ",".join(str(item.get("raw_meas_idx", "")) for item in group)
        sources = ",".join(
            f"{item.get('board', '')}/{item.get('cam', '')}/logic={item.get('logic_id', '')}"
            for item in group
        )
        parts.append(f"{idx}:n={len(group)},raw={raw_indices},src={sources}")
    return ";".join(parts)


def relative_to_map_azimuth(relative_az, device_heading_deg=DEVICE_HEADING_DEG):
    """将设备自身坐标系方位角转换为正北为0度的地图绝对方位角。"""
    return (float(relative_az) + float(device_heading_deg)) % 360.0


def track_ui_source(track, fallback_board, fallback_cam):
    """Return this track's latest detection source, with a legacy fallback."""
    board = getattr(track, "last_source_board", None)
    cam = getattr(track, "last_source_cam", None)
    return (
        fallback_board if board in (None, "") else board,
        fallback_cam if cam in (None, "") else cam,
    )


def track_is_ui_fresh(track, now_t):
    """Hide stale prediction-only tracks while retaining them internally."""
    return track.lost_seconds(now_t) <= UI_MAX_LOST_SECONDS


def get_turn_direction_label(delta_az, delta_el, deadband_az=0.35, deadband_el=0.25):
    """
    根据目标相对当前云台姿态的角差，给出转动方向标签。
    delta_az > 0: 向右转；delta_az < 0: 向左转
    delta_el > 0: 向上转；delta_el < 0: 向下转
    """
    if delta_az > deadband_az:
        horiz = "RIGHT"
    elif delta_az < -deadband_az:
        horiz = "LEFT"
    else:
        horiz = "CENTER"

    if delta_el > deadband_el:
        vert = "UP"
    elif delta_el < -deadband_el:
        vert = "DOWN"
    else:
        vert = "LEVEL"

    if horiz == "CENTER" and vert == "LEVEL":
        return "HOLD"
    if horiz == "CENTER":
        return vert
    if vert == "LEVEL":
        return horiz
    return f"{horiz}_{vert}"


def gimbal_control_thread(gimbal):
    """
    云台硬件专属线程：独占串口读写，支持可抢占执行。
    """
    print("[GimbalThread] 控制线程已启动")
    active_cmd = None#当前正在执行的指令
    target_az = 0.0
    target_el = 0.0
    cmd_start_t = 0.0
    last_progress_log_t = 0.0
    settle_candidate_since = None
    next_feedback_query_t = 0.0
    last_az_send_t = 0.0
    last_el_send_t = 0.0
    last_motion_az = None
    last_motion_el = None
    stationary_candidate_since = None

    def mark_motion_expected():
        nonlocal stationary_candidate_since
        stationary_candidate_since = None
        with shared_state.lock:
            shared_state.is_stationary = False

    def record_feedback_motion(curr_el, curr_az, feedback_t):
        """Update UI attitude and physical stationary state from encoder deltas."""
        nonlocal last_motion_az, last_motion_el, stationary_candidate_since
        curr_el = quantize_gimbal_command_angle(curr_el)
        curr_az = quantize_gimbal_command_angle(curr_az)
        curr_ui_az = quantize_gimbal_command_angle(
            (curr_az - GIMBAL_AZ_BASE) % 360.0
        )
        curr_ui_el = quantize_gimbal_command_angle(curr_el - GIMBAL_INIT_EL)
        was_stationary = False
        with shared_state.lock:
            was_stationary = shared_state.is_stationary

        if last_motion_az is None or last_motion_el is None:
            stationary_candidate_since = feedback_t
            is_stationary = False
        else:
            delta_az = abs(angular_diff(curr_az, last_motion_az))
            delta_el = abs(curr_el - last_motion_el)
            physically_moving = (
                delta_az > GIMBAL_STATIONARY_DELTA_DEG
                or delta_el > GIMBAL_STATIONARY_DELTA_DEG
            )
            if physically_moving:
                stationary_candidate_since = None
                is_stationary = False
            else:
                if stationary_candidate_since is None:
                    stationary_candidate_since = feedback_t
                is_stationary = (
                    feedback_t - stationary_candidate_since
                    >= GIMBAL_STATIONARY_DWELL_SECONDS
                )

        last_motion_az = curr_az
        last_motion_el = curr_el
        with shared_state.lock:
            shared_state.gimbal_el = curr_ui_el
            shared_state.gimbal_az = curr_ui_az
            shared_state.gimbal_att_ts = feedback_t
            shared_state.is_stationary = is_stationary
            if is_stationary and not was_stationary:
                shared_state.stationary_ts = feedback_t
        if is_stationary and not was_stationary:
            field_log_gimbal({
                "timestamp": f"{feedback_t:.6f}",
                "event": "GIMBAL_STATIONARY",
                "gimbal_ui_az": f"{curr_ui_az:.1f}",
                "gimbal_ui_el": f"{curr_ui_el:.1f}",
                "gimbal_ctrl_az": f"{curr_az:.1f}",
                "gimbal_ctrl_el": f"{curr_el:.1f}",
                "is_settled": 1 if shared_state.is_settled else 0,
                "is_stationary": 1,
            })
        return curr_ui_az, curr_ui_el, is_stationary

    while True:
        try:
            #1.当前没有指令在执行：(空闲态)
            if active_cmd is None:
                try:
                    #等待队列命令
                    active_cmd = gimbal_cmd_queue.get(timeout=0.1)
                
                except queue.Empty:
                    #若指令队列为空
                    real_att = gimbal.get_attitude()#读当前云台姿态并写入共享状态
                    if real_att:
                        curr_el, curr_az, _ = real_att
                        record_feedback_motion(
                            curr_el, curr_az, time.time()
                        )
                    continue
                #若成功取到指令，下发指令到云台
                target_az = quantize_gimbal_command_angle(active_cmd["az"])
                target_el = quantize_gimbal_command_angle(active_cmd["el"])
                active_cmd["az"] = target_az
                active_cmd["el"] = target_el
                cmd_start_t = time.time()
                last_progress_log_t = 0.0
                settle_candidate_since = None
                send_status = gimbal.set_attitude(
                    elevation=target_el,
                    azimuth=target_az,
                    force=bool(active_cmd.get("force", False)),
                )
                send_t = time.time()
                last_az_send_t = send_t
                last_el_send_t = send_t
                next_feedback_query_t = send_t + GIMBAL_QUERY_AFTER_CMD_DELAY
                mark_motion_expected()
                with shared_state.lock:
                    shared_state.active_cmd_id = int(active_cmd["cmd_id"])#设置当前执行指令的ID
                    shared_state.active_track_id = int(active_cmd.get("track_id", -1))
                    shared_state.is_settled = False#转动到位标志重置
                if PRINT_EVENT_LOGS:
                    print(
                        f"[GimbalCmd] cmd_id={int(active_cmd['cmd_id'])}, "
                        f"track_id={int(active_cmd.get('track_id', -1))}, "
                        f"target_ctrl=(Az={target_az:.1f}°, El={target_el:.1f}°)"
                    )
                field_log_gimbal({
                    "timestamp": f"{cmd_start_t:.6f}",
                    "event": "GIMBAL_CMD_SEND",
                    "cmd_id": int(active_cmd["cmd_id"]),
                    "track_id": int(active_cmd.get("track_id", -1)),
                    "target_ctrl_az": f"{target_az:.1f}",
                    "target_ctrl_el": f"{target_el:.1f}",
                })
            #2.若当前有指令在执行(执行态)
            now_t = time.time()
            #检查指令队列中是否有更新的指令，如果有则取出最新的一条（丢弃旧指令），准备进行抢占式执行判断
            newer_cmd = drain_latest_gimbal_cmd()
            if newer_cmd is not None:
                new_az = quantize_gimbal_command_angle(newer_cmd["az"])
                new_el = quantize_gimbal_command_angle(newer_cmd["el"])
                newer_cmd["az"] = new_az
                newer_cmd["el"] = new_el
                new_track_id = int(newer_cmd.get("track_id", -1))
                curr_track_id = int(active_cmd.get("track_id", -1))
                #计算新指令与当前指令的角度差
                d_az = quantize_gimbal_command_angle(
                    abs(angular_diff(new_az, target_az))
                )
                d_el = quantize_gimbal_command_angle(
                    abs(new_el - target_el)
                )
                d_total = math.hypot(d_az, d_el)

                # 目标切换时必须整条命令替换，避免“新方位 + 旧俯仰”混合指向。
                full_replace = (new_track_id != curr_track_id)
                force_update = bool(newer_cmd.get("force", False))
                update_az = (
                    full_replace or force_update or (d_az > AZ_PREEMPT_DEG)
                )
                update_el = (
                    full_replace or force_update or (d_el > EL_PREEMPT_DEG)
                )

                if update_az or update_el:
                    if update_az:
                        target_az = new_az
                    if update_el:
                        target_el = new_el
                    active_cmd = newer_cmd
                    cmd_start_t = now_t
                    last_progress_log_t = 0.0
                    settle_candidate_since = None
                    send_status = gimbal.set_attitude(
                        elevation=target_el,
                        azimuth=target_az,
                        force=force_update,
                    )
                    send_t = time.time()
                    if update_az:
                        last_az_send_t = send_t
                    if update_el:
                        last_el_send_t = send_t
                    next_feedback_query_t = send_t + GIMBAL_QUERY_AFTER_CMD_DELAY
                    mark_motion_expected()
                    with shared_state.lock:
                        shared_state.active_cmd_id = int(active_cmd["cmd_id"])
                        shared_state.active_track_id = new_track_id
                        shared_state.is_settled = False

                    update_mode = "track_switch" if full_replace else "axis_update"
                    updated_axes = []
                    if update_az:
                        updated_axes.append("Az")
                    if update_el:
                        updated_axes.append("El")
                    updated_axes_text = "+".join(updated_axes) if updated_axes else "none"
                    if PRINT_EVENT_LOGS:
                        print(
                            f"[GimbalCmd] preempt cmd_id={int(active_cmd['cmd_id'])}, "
                            f"track_id={new_track_id}, mode={update_mode}, axes={updated_axes_text}, "
                            f"target_ctrl=(Az={target_az:.1f}°, El={target_el:.1f}°), "
                            f"delta=(dAz={d_az:.1f}°, dEl={d_el:.1f}°, total={d_total:.1f}°)"
                        )
                    field_log_gimbal({
                        "timestamp": f"{now_t:.6f}",
                        "event": "GIMBAL_CMD_PREEMPT",
                        "cmd_id": int(active_cmd["cmd_id"]),
                        "track_id": new_track_id,
                        "target_ctrl_az": f"{target_az:.1f}",
                        "target_ctrl_el": f"{target_el:.1f}",
                        "err_az": f"{d_az:.1f}",
                        "err_el": f"{d_el:.1f}",
                    })

            if time.time() < next_feedback_query_t:
                time.sleep(GIMBAL_THREAD_SLEEP)
                continue

            real_att = gimbal.get_attitude()
            if real_att:
                curr_el, curr_az, _ = real_att
                curr_el = quantize_gimbal_command_angle(curr_el)
                curr_az = quantize_gimbal_command_angle(curr_az)
                curr_ui_az, curr_ui_el, gimbal_is_stationary = (
                    record_feedback_motion(curr_el, curr_az, now_t)
                )
                err_az = quantize_gimbal_command_angle(
                    abs(angular_diff(target_az, curr_az))
                )
                err_el = quantize_gimbal_command_angle(
                    abs(curr_el - target_el)
                )

                if (last_progress_log_t == 0.0) or ((now_t - last_progress_log_t) >= GIMBAL_PROGRESS_LOG_INTERVAL):
                    elapsed = now_t - cmd_start_t
                    if PRINT_GIMBAL_PROGRESS:
                        print(
                            f"[GimbalAtt] cmd_id={int(active_cmd['cmd_id'])}, "
                            f"elapsed={elapsed:.3f}s, "
                            f"actual_ctrl=(Az={curr_az:.1f}°, El={curr_el:.1f}°), "
                            f"actual_ui=(Az={curr_ui_az:.1f}°, El={curr_el:.1f}°), "
                            f"target_ctrl=(Az={target_az:.1f}°, El={target_el:.1f}°), "
                            f"err=(dAz={err_az:.1f}°, dEl={err_el:.1f}°)"
                        )
                    field_log_gimbal({
                        "timestamp": f"{now_t:.6f}",
                        "event": "GIMBAL_ATT",
                        "cmd_id": int(active_cmd["cmd_id"]),
                        "track_id": int(active_cmd.get("track_id", -1)),
                        "gimbal_ui_az": f"{curr_ui_az:.1f}",
                        "gimbal_ui_el": f"{curr_ui_el:.1f}",
                        "gimbal_ctrl_az": f"{curr_az:.1f}",
                        "gimbal_ctrl_el": f"{curr_el:.1f}",
                        "target_ctrl_az": f"{target_az:.1f}",
                        "target_ctrl_el": f"{target_el:.1f}",
                        "err_az": f"{err_az:.1f}",
                        "err_el": f"{err_el:.1f}",
                        "is_stationary": 1 if gimbal_is_stationary else 0,
                    })
                    last_progress_log_t = now_t

                retry_az = (
                    err_az >= GIMBAL_SETTLE_THRESHOLD
                    and (now_t - last_az_send_t) >= GIMBAL_COMMAND_RETRY_INTERVAL
                )
                retry_el = (
                    err_el >= GIMBAL_SETTLE_THRESHOLD
                    and (now_t - last_el_send_t) >= GIMBAL_COMMAND_RETRY_INTERVAL
                )
                if retry_az or retry_el:
                    retry_status = gimbal.set_attitude(
                        elevation=target_el if retry_el else None,
                        azimuth=target_az if retry_az else None,
                        force=True,
                    )
                    retry_t = time.time()
                    if retry_az:
                        last_az_send_t = retry_t
                    if retry_el:
                        last_el_send_t = retry_t
                    next_feedback_query_t = (
                        retry_t + GIMBAL_QUERY_AFTER_CMD_DELAY
                    )
                    mark_motion_expected()
                    retry_axes = "+".join(
                        axis for axis, enabled in (
                            ("Az", retry_az), ("El", retry_el)
                        ) if enabled
                    )
                    field_log_gimbal({
                        "timestamp": f"{retry_t:.6f}",
                        "event": "GIMBAL_CMD_RETRY",
                        "cmd_id": int(active_cmd["cmd_id"]),
                        "track_id": int(active_cmd.get("track_id", -1)),
                        "target_ctrl_az": f"{target_az:.1f}",
                        "target_ctrl_el": f"{target_el:.1f}",
                        "err_az": f"{err_az:.1f}",
                        "err_el": f"{err_el:.1f}",
                        "retry_axes": retry_axes,
                        "driver_status": str(retry_status),
                    })
                    time.sleep(GIMBAL_THREAD_SLEEP)
                    continue

                if err_az < GIMBAL_SETTLE_THRESHOLD and err_el < GIMBAL_SETTLE_THRESHOLD:
                    if settle_candidate_since is None:
                        settle_candidate_since = now_t
                        time.sleep(GIMBAL_THREAD_SLEEP)
                        continue
                    if (now_t - settle_candidate_since) < GIMBAL_SETTLE_DWELL_SECONDS:
                        time.sleep(GIMBAL_THREAD_SLEEP)
                        continue
                    settle_dt = now_t - cmd_start_t
                    with shared_state.lock:
                        shared_state.is_settled = True
                        shared_state.settled_cmd_id = shared_state.active_cmd_id
                        shared_state.settled_track_id = shared_state.active_track_id
                        shared_state.settled_ts = now_t
                        active_cmd_id = shared_state.active_cmd_id
                        active_track_id = shared_state.active_track_id

                    if PRINT_EVENT_LOGS:
                        print(
                            f"[GimbalSettled] cmd_id={active_cmd_id}, track_id={active_track_id}, "
                            f"settle_time={settle_dt:.3f}s, "
                            f"actual_ctrl=(Az={curr_az:.1f}°, El={curr_el:.1f}°), "
                            f"target_ctrl=(Az={target_az:.1f}°, El={target_el:.1f}°), "
                            f"err=(dAz={err_az:.1f}°, dEl={err_el:.1f}°)"
                        )
                    field_log_gimbal({
                        "timestamp": f"{now_t:.6f}",
                        "event": "GIMBAL_SETTLED",
                        "cmd_id": int(active_cmd_id),
                        "track_id": int(active_track_id),
                        "gimbal_ui_az": f"{curr_ui_az:.1f}",
                        "gimbal_ui_el": f"{curr_ui_el:.1f}",
                        "gimbal_ctrl_az": f"{curr_az:.1f}",
                        "gimbal_ctrl_el": f"{curr_el:.1f}",
                        "target_ctrl_az": f"{target_az:.1f}",
                        "target_ctrl_el": f"{target_el:.1f}",
                        "err_az": f"{err_az:.1f}",
                        "err_el": f"{err_el:.1f}",
                        "is_settled": 1,
                        "is_stationary": 1 if gimbal_is_stationary else 0,
                        "settle_time": f"{settle_dt:.6f}",
                    })

                    # Real SFL0603 sessions are controlled by the visual-alignment request gate.
                    active_cmd = None
                    settle_candidate_since = None
                    continue
                else:
                    settle_candidate_since = None

            if (now_t - cmd_start_t) >= GIMBAL_SETTLE_TIMEOUT:
                elapsed = now_t - cmd_start_t
                cmd_id = int(active_cmd["cmd_id"]) if active_cmd is not None else -1
                track_id = int(active_cmd.get("track_id", -1)) if active_cmd is not None else -1
                if real_att:
                    if PRINT_EVENT_LOGS:
                        print(
                            f"[GimbalTimeout] cmd_id={cmd_id}, track_id={track_id}, "
                            f"elapsed={elapsed:.3f}s, "
                            f"actual_ctrl=(Az={curr_az:.1f}°, El={curr_el:.1f}°), "
                            f"target_ctrl=(Az={target_az:.1f}°, El={target_el:.1f}°), "
                            f"err=(dAz={err_az:.1f}°, dEl={err_el:.1f}°)"
                        )
                else:
                    if PRINT_EVENT_LOGS:
                        print(
                            f"[GimbalTimeout] cmd_id={cmd_id}, track_id={track_id}, "
                            f"elapsed={elapsed:.3f}s, no attitude feedback"
                        )
                field_log_gimbal({
                    "timestamp": f"{now_t:.6f}",
                    "event": "GIMBAL_TIMEOUT",
                    "cmd_id": cmd_id,
                    "track_id": track_id,
                    "target_ctrl_az": f"{target_az:.1f}",
                    "target_ctrl_el": f"{target_el:.1f}",
                    "err_az": "" if not real_att else f"{err_az:.1f}",
                    "err_el": "" if not real_att else f"{err_el:.1f}",
                    "is_settled": 0,
                    "is_stationary": (
                        "" if not real_att
                        else (1 if gimbal_is_stationary else 0)
                    ),
                    "settle_time": f"{elapsed:.6f}",
                })
                with shared_state.lock:
                    shared_state.is_settled = False
                active_cmd = None
                settle_candidate_since = None
                continue

            time.sleep(GIMBAL_THREAD_SLEEP)

        except Exception as e:
            print(f"[GimbalThread][Unexpected] type={type(e).__name__}, err={e}")
            time.sleep(0.05)

def get_dynamic_tracking_params(distance_m):
    """
    根据目标距离生成动态参数（连续插值）。
    冷启动/无效测距时默认按 DEFAULT_TRACKING_DISTANCE_M 处理。
    """
    if (distance_m is None) or (distance_m <= 0):
        distance_m = DEFAULT_TRACKING_DISTANCE_M

    dist_nodes = np.array([50.0, 100.0, 200.0, 400.0, 500.0], dtype=float)
    d = float(np.clip(distance_m, dist_nodes[0], dist_nodes[-1]))

    max_res_az = float(np.interp(d, dist_nodes, [2.5, 1.8, 1.2, 0.9, 0.7]))
    max_vel_az = float(np.interp(d, dist_nodes, [30.0, 20.0, 12.0, 8.0, 5.0]))
    dist_thresh = float(np.interp(d, dist_nodes, [4.0, 3.0, 2.0, 1.3, 0.9]))

    return {
        "DIST_M": d,
        "MAX_RES_AZ": max_res_az,#单帧角度的最大修正
        "MAX_RES_EL": max_res_az * 0.5,
        "MAX_VEL_AZ": max_vel_az,#速度限幅
        "MAX_VEL_EL": max_vel_az * 0.5,
        "DIST_THRESH": dist_thresh,#判定“当前帧的检测点”与“上一帧的追踪轨迹”是否为同一个目标的最大角度欧氏距离。
        "MIN_DT": 0.001,  # 仅防异常极小值，不再把 15 FPS 的实际 dt 抬高
    }

class RangeSmoother:
    """激光测距平滑器：限速 + EMA，防止参数抖动。"""
    def __init__(self, init_d=50.0, alpha=0.2, max_rate_mps=30.0):
        self.d = float(init_d)
        self.alpha = float(alpha)
        self.max_rate_mps = float(max_rate_mps)

    def update(self, raw_d, dt):
        if (raw_d is None) or (raw_d <= 0):
            return self.d

        raw_d = float(np.clip(raw_d, 50.0, 500.0))
        dt_eff = max(float(dt), 0.03)
        max_step = self.max_rate_mps * dt_eff

        raw_d = max(self.d - max_step, min(self.d + max_step, raw_d))
        self.d = self.d + self.alpha * (raw_d - self.d)
        return self.d

def ui_to_ctrl_angles(ui_az, ui_el):
    """将绝对角度转为云台控制角度 (复用原先 calculate_angles 里的逻辑)"""
    rel_az = ui_az
    if rel_az > 180.0:
        rel_az -= 360.0
    ctrl_az = GIMBAL_AZ_BASE + rel_az
    ctrl_el = GIMBAL_INIT_EL + ui_el
    if ctrl_az < 0.0: ctrl_az = 0.0
    if ctrl_az > 350.0: ctrl_az = 350.0
    return ctrl_az, ctrl_el

# ==========================================
# 新增模块 2：单目标卡尔曼追踪器 (AngleTracker)
# ==========================================
class StandardKalmanTrack:
    _id_count = 0
    def __init__(self, ui_az, ui_el, init_ts=None):
        StandardKalmanTrack._id_count += 1
        self.id = StandardKalmanTrack._id_count
        init_ts = time.time() if init_ts is None else float(init_ts)
        
        # 1. 原始 4D CV 状态矩阵 (Active 主控制源)
        self.state = np.array([[ui_az], [ui_el], [0.0], [0.0]], dtype=float)
        self.P = np.diag([1.0, 1.0, 10.0, 10.0])
        self.q_pos = 0.05
        self.q_vel = 0.2
        self.r_az = 3.5**2
        self.r_el = 1.8**2
        
        # 2. 新增 6D CA 影子状态矩阵 (Shadow 验证源)
        self.shadow_state = np.array([[ui_az], [ui_el], [0.0], [0.0], [0.0], [0.0]], dtype=float)
        self.shadow_P = np.diag([1.0, 1.0, 10.0, 10.0, 5.0, 5.0])
        self.shadow_q_pos = 0.05
        self.shadow_q_vel = 0.2
        self.shadow_q_acc = 0.1
        
        # 3. 1D 距离卡尔曼状态 (仅用于平滑去噪，不做长时预测)
        self.dist_state = None  # 首次接收到距离时预测并初始化: [[d], [v_d]]
        self.dist_P = np.diag([10.0, 5.0])
        self.q_dist_pos = 0.1
        self.q_dist_vel = 0.5
        self.r_dist = 25.0**2  # 较大测量噪声，以实现强力平滑
        self.last_dist_ts = 0.0
        self.dist_source = "none"
        self.dist_uncertainty = 0.0
        
        self.hit_streak = 1        # 连续命中次数 (用于建轨确认)
        self.time_since_update = 0 # 连丢次数
        self.created_ts = init_ts
        self.last_update_ts = init_ts  # last successful detection association time
        self.confirmed = False     # internal tracker confirmation
        self.ui_confirmed = False  # allow assigning/sending external UI ID
        self.strike_confirmed = False  # allow entering strike threat ranking
        
        # 历史队列 (基于 Active 状态记录)
        self.history = deque(maxlen=30)
        self.history.append((self.state.copy(), self.P.copy()))
        
        self.max_vel_az = 40.0
        self.max_vel_el = 15.0
        self.max_acc_az = 20.0  # 影子 CA 方位角加速度限幅
        self.max_acc_el = 10.0  # 影子 CA 俯仰角加速度限幅
        self.min_dt = 0.001
        self.dist_thresh = 4.0
        self.last_mono_dist = None
        self.mono_ts = 0.0
        self.last_sent_dist = None
        # Detection provenance belongs to the track, not to the most recently
        # received frame. UI status uses these fields after multi-camera fusion.
        self.last_source_board = None
        self.last_source_cam = None
        self.last_source_logic_id = None
        self.last_source_boards = ()
        self.last_source_cams = ()
        self.last_source_logic_ids = ()
        self.last_source_ts = 0.0

    def lost_seconds(self, now_t=None):
        now_t = time.time() if now_t is None else float(now_t)
        return max(0.0, now_t - float(self.last_update_ts))

    def set_detection_source(self, measurement, ts):
        """Remember the latest measurement provenance for per-track UI output."""
        if not isinstance(measurement, dict):
            return
        board = measurement.get("board")
        cam = measurement.get("cam")
        logic_id = measurement.get("logic_id")
        if board not in (None, ""):
            self.last_source_board = str(board)
        if cam not in (None, ""):
            try:
                self.last_source_cam = int(cam)
            except (TypeError, ValueError):
                self.last_source_cam = cam
        if logic_id not in (None, ""):
            self.last_source_logic_id = logic_id
        self.last_source_boards = tuple(measurement.get("source_boards") or ())
        self.last_source_cams = tuple(measurement.get("source_cams") or ())
        self.last_source_logic_ids = tuple(measurement.get("source_logic_ids") or ())
        source_ts = measurement.get("source_ts")
        try:
            parsed_source_ts = float(source_ts)
            self.last_source_ts = parsed_source_ts if parsed_source_ts > 0.0 else float(ts)
        except (TypeError, ValueError):
            self.last_source_ts = float(ts)

    def set_mono_distance(self, dist, ts):
        d = _parse_positive_float(dist)
        if d is None:
            return
        self.last_mono_dist = d
        self.mono_ts = float(ts)
        self._update_distance_filter(d, ts)

    def _update_distance_filter(self, dist_val, ts):
        """1D 距离卡尔曼滤波，含异常门控与新鲜度管理"""
        curr_t = float(ts)
        
        # Reset stale mono distance state instead of carrying old range into a new target interval.
        if self.dist_state is not None and (curr_t - self.last_dist_ts) > TRACK_DISTANCE_TTL:
            self.dist_state = None
            
        if self.dist_state is None:
            self.dist_state = np.array([[dist_val], [0.0]], dtype=float)
            self.last_dist_ts = curr_t
            self.dist_source = "mono"
            self.dist_uncertainty = np.sqrt(self.dist_P[0, 0])
            return
            
        # 异常门控：突变值 > 50m 判定为噪点，拒绝更新状态
        expected_d = self.dist_state[0, 0]
        if abs(dist_val - expected_d) > 50.0:
            return
            
        dt = curr_t - self.last_dist_ts
        if dt < self.min_dt:
            dt = self.min_dt
        self.last_dist_ts = curr_t
        self.dist_source = "mono"
        
        # 1. Predict
        F_d = np.array([[1.0, dt],
                        [0.0, 1.0]], dtype=float)
        Q_d = np.diag([self.q_dist_pos, self.q_dist_vel])
        self.dist_state = np.dot(F_d, self.dist_state)
        self.dist_P = np.dot(np.dot(F_d, self.dist_P), F_d.T) + Q_d
        
        # 2. Update
        H_d = np.array([[1.0, 0.0]], dtype=float)
        R_d = np.array([[self.r_dist]], dtype=float)
        
        Z = np.array([[dist_val]], dtype=float)
        Y = Z - np.dot(H_d, self.dist_state)
        S = np.dot(np.dot(H_d, self.dist_P), H_d.T) + R_d
        
        try:
            K = np.dot(np.dot(self.dist_P, H_d.T), np.linalg.inv(S))
        except np.linalg.LinAlgError:
            K = np.zeros((2, 1), dtype=float)
            
        self.dist_state = self.dist_state + np.dot(K, Y)
        # 限制径向速度在安全范围 ±25 m/s 内
        self.dist_state[1, 0] = np.clip(self.dist_state[1, 0], -25.0, 25.0)
        
        I = np.eye(2)
        self.dist_P = np.dot((I - np.dot(K, H_d)), self.dist_P)
        self.dist_uncertainty = np.sqrt(self.dist_P[0, 0])

    def get_param_distance(self, curr_time):
        if self.dist_state is not None and (curr_time - self.last_dist_ts) <= TRACK_DISTANCE_TTL:
            return float(self.dist_state[0, 0])
        if self.last_mono_dist is not None and (curr_time - self.mono_ts) <= MONO_DIST_TTL:
            return self.last_mono_dist
        if self.last_mono_dist is not None:
            return self.last_mono_dist
        return None

    def set_dynamic_params(self, params):
        if not params:
            return
        
        max_res_az = float(params.get('MAX_RES_AZ', 3.5))
        max_res_el = float(params.get('MAX_RES_EL', 1.8))
        self.r_az = max_res_az**2
        self.r_el = max_res_el**2
        
        self.max_vel_az = float(params.get('MAX_VEL_AZ', self.max_vel_az))
        self.max_vel_el = float(params.get('MAX_VEL_EL', self.max_vel_el))
        self.min_dt = float(params.get('MIN_DT', self.min_dt))
        self.dist_thresh = float(params.get('DIST_THRESH', self.dist_thresh))

    def predict(self, dt):
        """Active CV 与 Shadow CA 平行角度预测"""
        if dt < self.min_dt:
            dt = self.min_dt
            
        # A. Active 4D CV Predict
        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1]
        ], dtype=float)
        Q = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel])
        self.state = np.dot(F, self.state)
        self.state[0, 0] = self.state[0, 0] % 360.0
        self.P = np.dot(np.dot(F, self.P), F.T) + Q
        
        # B. Shadow 6D CA Predict
        F_shadow = np.array([
            [1.0, 0.0,  dt, 0.0, 0.5*dt**2,       0.0],
            [0.0, 1.0, 0.0,  dt,       0.0, 0.5*dt**2],
            [0.0, 0.0, 1.0, 0.0,        dt,       0.0],
            [0.0, 0.0, 0.0, 1.0,       0.0,        dt],
            [0.0, 0.0, 0.0, 0.0,       1.0,       0.0],
            [0.0, 0.0, 0.0, 0.0,       0.0,       1.0]
        ], dtype=float)
        Q_shadow = np.diag([self.shadow_q_pos, self.shadow_q_pos, self.shadow_q_vel, self.shadow_q_vel, self.shadow_q_acc, self.shadow_q_acc])
        self.shadow_state = np.dot(F_shadow, self.shadow_state)
        self.shadow_state[0, 0] = self.shadow_state[0, 0] % 360.0
        self.shadow_P = np.dot(np.dot(F_shadow, self.shadow_P), F_shadow.T) + Q_shadow
        
        self.time_since_update += 1
        self.history.append((self.state.copy(), self.P.copy()))

    def update(self, meas_az, meas_el, dt, now_t=None):
        """Active CV 与 Shadow CA 平行角度更新"""
        now_t = time.time() if now_t is None else float(now_t)
        self.time_since_update = 0
        self.last_update_ts = now_t
        self.hit_streak += 1
        if self.hit_streak >= TRACK_CONFIRM_HITS:
            self.confirmed = True

        # A. Active 4D CV Update
        Z = np.array([[meas_az], [meas_el]], dtype=float)
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)
        R = np.diag([self.r_az, self.r_el])
        Y = Z - np.dot(H, self.state)
        Y[0, 0] = angular_diff(Z[0, 0], self.state[0, 0])
        S = np.dot(np.dot(H, self.P), H.T) + R
        try:
            K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        except np.linalg.LinAlgError:
            K = np.zeros((4, 2), dtype=float)
        self.state = self.state + np.dot(K, Y)
        self.state[0, 0] = self.state[0, 0] % 360.0
        self.state[2, 0] = np.clip(self.state[2, 0], -self.max_vel_az, self.max_vel_az)
        self.state[3, 0] = np.clip(self.state[3, 0], -self.max_vel_el, self.max_vel_el)
        I = np.eye(4)
        self.P = np.dot((I - np.dot(K, H)), self.P)

        # B. Shadow 6D CA Update
        H_shadow = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
        ], dtype=float)
        Y_shadow = Z - np.dot(H_shadow, self.shadow_state)
        Y_shadow[0, 0] = angular_diff(Z[0, 0], self.shadow_state[0, 0])
        S_shadow = np.dot(np.dot(H_shadow, self.shadow_P), H_shadow.T) + R
        try:
            K_shadow = np.dot(np.dot(self.shadow_P, H_shadow.T), np.linalg.inv(S_shadow))
        except np.linalg.LinAlgError:
            K_shadow = np.zeros((6, 2), dtype=float)
        self.shadow_state = self.shadow_state + np.dot(K_shadow, Y_shadow)
        self.shadow_state[0, 0] = self.shadow_state[0, 0] % 360.0
        self.shadow_state[2, 0] = np.clip(self.shadow_state[2, 0], -self.max_vel_az, self.max_vel_az)
        self.shadow_state[3, 0] = np.clip(self.shadow_state[3, 0], -self.max_vel_el, self.max_vel_el)
        self.shadow_state[4, 0] = np.clip(self.shadow_state[4, 0], -self.max_acc_az, self.max_acc_az)
        self.shadow_state[5, 0] = np.clip(self.shadow_state[5, 0], -self.max_acc_el, self.max_acc_el)
        I_shadow = np.eye(6)
        self.shadow_P = np.dot((I_shadow - np.dot(K_shadow, H_shadow)), self.shadow_P)

        if len(self.history) > 0:
            self.history[-1] = (self.state.copy(), self.P.copy())
        else:
            self.history.append((self.state.copy(), self.P.copy()))

    def get_future_position(self, dt_delay):
        """Active 4D CV 角度预测"""
        fut_az = (self.state[0, 0] + self.state[2, 0] * dt_delay) % 360.0
        fut_el = self.state[1, 0] + self.state[3, 0] * dt_delay
        return fut_az, fut_el

    def get_shadow_future_position_ca(self, dt_delay):
        """Shadow 6D CA 角度预测 (仅供日志对比，不驱动云台)"""
        fut_az = (self.shadow_state[0, 0] + self.shadow_state[2, 0] * dt_delay + 0.5 * self.shadow_state[4, 0] * (dt_delay**2)) % 360.0
        fut_el = self.shadow_state[1, 0] + self.shadow_state[3, 0] * dt_delay + 0.5 * self.shadow_state[5, 0] * (dt_delay**2)
        return fut_az, fut_el

    def predict_future_n_steps(self, n=10, dt=0.066):
        """多帧预测接口 (兼容原版)"""
        if dt < self.min_dt:
            dt = self.min_dt
            
        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1]
        ], dtype=float)
        Q = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel])
        
        temp_state = self.state.copy()
        temp_P = self.P.copy()
        
        future_states = []
        future_Ps = []
        
        for _ in range(n):
            temp_state = np.dot(F, temp_state)
            temp_state[0, 0] = temp_state[0, 0] % 360.0
            temp_P = np.dot(np.dot(F, temp_P), F.T) + Q
            future_states.append(temp_state.copy())
            future_Ps.append(temp_P.copy())
            
        return future_states, future_Ps

# ==========================================
# 新增模块 3：多目标调度大脑 (MultiTargetTracker)
# ==========================================
class MultiTargetTracker:
    def __init__(
        self,
        max_lost_frames=30,
        max_lost_seconds=None,
        base_distance_threshold=4.0,
        distance_threshold=None,
    ):
        self.tracks = []
        self.max_lost_frames = max_lost_frames
        self.max_lost_seconds = (
            float(max_lost_seconds)
            if max_lost_seconds is not None
            else float(max_lost_frames) * NO_PACKET_TRACKER_UPDATE_INTERVAL
        )
        # Backward compatibility: keep supporting old constructor arg `distance_threshold`.
        if distance_threshold is not None:
            self.base_distance_threshold = float(distance_threshold)
        else:
            self.base_distance_threshold = float(base_distance_threshold)

    def _debug_context_row(self, debug_context, event):
        debug_context = debug_context or {}
        return {
            "timestamp": f"{time.time():.6f}",
            "mode": debug_context.get("mode", ""),
            "frame_id": debug_context.get("frame_id", ""),
            "event": event,
            "meas_count": debug_context.get("meas_count", ""),
        }

    @staticmethod
    def _association_gate(track, now_t):
        """Return a bounded association gate before Hungarian assignment."""
        uncertainty = float(np.sqrt(max(0.0, track.P[0, 0] + track.P[1, 1])))
        covariance_gate = float(track.dist_thresh) + (uncertainty * 1.5)
        lost_seconds = track.lost_seconds(now_t)
        hard_cap = TRACK_ASSOCIATION_MAX_DEG
        gate_mode = "recent"
        if lost_seconds > TRACK_REACQUIRE_STRICT_AFTER_SECONDS:
            hard_cap = min(hard_cap, TRACK_REACQUIRE_MAX_DEG)
            gate_mode = "strict_reacquire"
        dynamic_thresh = max(0.0, min(covariance_gate, hard_cap))
        return dynamic_thresh, uncertainty, lost_seconds, gate_mode

    def _log_new_track(self, track, meas_idx, meas, debug_context):
        if DEBUG_KALMAN_MATCH:
            print(
                f"[NEW_TRACK] track={track.id}, meas={meas_idx}, "
                f"meas=(Az={meas['az']:.2f}, El={meas['el']:.2f}), "
                f"mono={meas['mono_dist']}, dist_thresh={track.dist_thresh:.2f}, "
                f"R=({track.r_az:.2f},{track.r_el:.2f}), "
                f"Q=({track.q_pos:.2f},{track.q_vel:.2f})"
            )
        row = self._debug_context_row(debug_context, "NEW_TRACK")
        field_log_event({
            "timestamp": row["timestamp"],
            "seq": row["frame_id"],
            "mode": row["mode"],
            "event": "NEW_TRACK",
            "track_id": int(track.id),
            "meas_idx": meas_idx,
            "meas_az": f"{meas['az']:.6f}",
            "meas_el": f"{meas['el']:.6f}",
            "p_az": f"{track.P[0, 0]:.6f}",
            "p_el": f"{track.P[1, 1]:.6f}",
            "hit_streak": int(track.hit_streak),
            "time_since_update": int(track.time_since_update),
            "lost_seconds": "0.000000",
        })

    def _prune_lost_tracks(self, now_t=None, debug_context=None):
        now_t = time.time() if now_t is None else float(now_t)
        kept_tracks = []
        for track in self.tracks:
            lost_s = track.lost_seconds(now_t)
            if lost_s < self.max_lost_seconds:
                kept_tracks.append(track)
                continue
            if DEBUG_KALMAN_MATCH:
                print(
                    f"[TRACK_DELETE] track={track.id}, "
                    f"lost={lost_s:.2f}s/{track.time_since_update} updates, "
                    f"max_lost={self.max_lost_seconds:.2f}s, "
                    f"state=(Az={track.state[0,0]:.2f}, El={track.state[1,0]:.2f}), "
                    f"hits={track.hit_streak}"
                )
            row = self._debug_context_row(debug_context, "TRACK_DELETE")
            field_log_event({
                "timestamp": row["timestamp"],
                "seq": row["frame_id"],
                "mode": row["mode"],
                "event": "TRACK_DELETE",
                "track_id": int(track.id),
                "pred_az": f"{track.state[0, 0]:.6f}",
                "pred_el": f"{track.state[1, 0]:.6f}",
                "p_az": f"{track.P[0, 0]:.6f}",
                "p_el": f"{track.P[1, 1]:.6f}",
                "hit_streak": int(track.hit_streak),
                "time_since_update": int(track.time_since_update),
                "lost_seconds": f"{lost_s:.6f}",
                "reason": f"lost_seconds>={self.max_lost_seconds:.3f}",
            })
        self.tracks = kept_tracks

    def update(self, measurements, dt, params=None, now_t=None, debug_context=None):
        """
        measurements: 当前帧所有检测目标，可为:
            1) [az, el]
            2) {"az": az, "el": el, "mono_dist": d}
        dt: 距离上一帧经过的时间(秒)
        """
        if now_t is None:
            now_t = time.time()

        if params and ("DIST_THRESH" in params):
            self.base_distance_threshold = float(params["DIST_THRESH"])

        normalized_measurements = []
        for meas in measurements:
            if isinstance(meas, dict):
                az = meas.get("az", None)
                el = meas.get("el", None)
                mono_dist = meas.get("mono_dist", None)
                source_metadata = {
                    key: meas.get(key)
                    for key in (
                        "board", "cam", "logic_id", "source_ts", "source_boards",
                        "source_cams", "source_logic_ids",
                    )
                }
            elif isinstance(meas, (list, tuple)) and len(meas) >= 2:
                az = meas[0]
                el = meas[1]
                mono_dist = meas[2] if len(meas) >= 3 else None
                source_metadata = {}
            else:
                continue

            try:
                az = float(az) % 360.0
                el = float(el)
            except (TypeError, ValueError):
                continue
            mono_dist = _parse_positive_float(mono_dist)
            normalized_measurement = {
                "az": az,
                "el": el,
                "mono_dist": mono_dist,
            }
            normalized_measurement.update(source_metadata)
            normalized_measurements.append(normalized_measurement)

        # 1. 预测所有已有 Track 的新位置
        for track in self.tracks:
            if params:
                track.set_dynamic_params(params)
            else:
                dist_for_track = track.get_param_distance(now_t)
                if dist_for_track is not None:
                    track.set_dynamic_params(get_dynamic_tracking_params(dist_for_track))
                else:
                    track.dist_thresh = self.base_distance_threshold
            track.predict(dt)
            
        # 如果当前帧没检测到东西，直接清理丢失目标并返回
        if len(normalized_measurements) == 0:
            self._prune_lost_tracks(now_t=now_t, debug_context=debug_context)
            return self.tracks

        if len(self.tracks) == 0:
            # 全是新目标,新建轨迹
            for m_idx, meas in enumerate(normalized_measurements):
                t = StandardKalmanTrack(
                    meas["az"],
                    meas["el"],
                    init_ts=now_t,
                )
                t.set_mono_distance(meas["mono_dist"], now_t)
                t.set_detection_source(meas, now_t)
                if params:
                    t.set_dynamic_params(params)
                else:
                    dist_for_track = t.get_param_distance(now_t)
                    if dist_for_track is not None:
                        t.set_dynamic_params(get_dynamic_tracking_params(dist_for_track))
                    else:
                        t.dist_thresh = self.base_distance_threshold
                self.tracks.append(t)
                self._log_new_track(t, m_idx, meas, debug_context)
            return self.tracks

        # 2. 计算代价dxfcs矩阵 (角度欧氏距离)
        cost_matrix = np.zeros((len(self.tracks), len(normalized_measurements)))
        for t, track in enumerate(self.tracks):
            for m, meas in enumerate(normalized_measurements):
                diff_az = angular_diff(meas["az"], track.state[0, 0])

                diff_el = meas["el"] - track.state[1, 0]
                distance = np.sqrt(diff_az**2 + diff_el**2)
                cost_matrix[t, m] = distance

        # 3. Gate impossible pairs before Hungarian assignment. A large finite
        # cost keeps scipy robust when a row has no feasible measurement; the
        # post-assignment check below still leaves blocked pairs unmatched.
        association_gates = [
            self._association_gate(track, now_t) for track in self.tracks
        ]
        gated_cost_matrix = cost_matrix.copy()
        for t_idx, gate_info in enumerate(association_gates):
            dynamic_thresh = gate_info[0]
            blocked = gated_cost_matrix[t_idx, :] >= dynamic_thresh
            gated_cost_matrix[t_idx, blocked] = ASSOCIATION_BLOCKED_COST
        track_indices, meas_indices = linear_sum_assignment(gated_cost_matrix)

        # 4. 更新匹配成功的 Track (加入协方差动态门限)
        unmatched_measurements = set(range(len(normalized_measurements)))
        matched_tracks = set()
        for t_idx, m_idx in zip(track_indices, meas_indices):
            track = self.tracks[t_idx]
            
            dynamic_thresh, uncertainty, lost_s_before_match, gate_mode = (
                association_gates[t_idx]
            )
            cost = cost_matrix[t_idx, m_idx]
            meas = normalized_measurements[m_idx]
            pred_az = float(track.state[0, 0])
            pred_el = float(track.state[1, 0])
            pair_is_feasible = gated_cost_matrix[t_idx, m_idx] < ASSOCIATION_BLOCKED_COST
            
            if pair_is_feasible:
                if DEBUG_KALMAN_MATCH:
                    print(
                        f"[MATCH_ACCEPT] track={track.id}, meas={m_idx}, "
                        f"meas=(Az={meas['az']:.2f}, El={meas['el']:.2f}), "
                        f"pred=(Az={pred_az:.2f}, El={pred_el:.2f}), "
                        f"cost={cost:.2f}, thresh={dynamic_thresh:.2f}, gate={gate_mode}, "
                        f"unc={uncertainty:.2f}, "
                        f"Ppos=({track.P[0,0]:.2f},{track.P[1,1]:.2f}), "
                        f"hits={track.hit_streak}, lost={track.time_since_update}, "
                        f"R=({track.r_az:.2f},{track.r_el:.2f}), "
                        f"Q=({track.q_pos:.2f},{track.q_vel:.2f})"
                    )
                row = self._debug_context_row(debug_context, "MATCH_ACCEPT")
                field_log_event({
                    "timestamp": row["timestamp"],
                    "seq": row["frame_id"],
                    "mode": row["mode"],
                    "event": "MATCH_ACCEPT",
                    "track_id": int(track.id),
                    "meas_idx": m_idx,
                    "meas_az": f"{meas['az']:.6f}",
                    "meas_el": f"{meas['el']:.6f}",
                    "pred_az": f"{pred_az:.6f}",
                    "pred_el": f"{pred_el:.6f}",
                    "cost": f"{cost:.6f}",
                    "dynamic_thresh": f"{dynamic_thresh:.6f}",
                    "uncertainty": f"{uncertainty:.6f}",
                    "p_az": f"{track.P[0, 0]:.6f}",
                    "p_el": f"{track.P[1, 1]:.6f}",
                    "hit_streak": int(track.hit_streak),
                    "time_since_update": int(track.time_since_update),
                    "lost_seconds": f"{lost_s_before_match:.6f}",
                })
                track.update(meas["az"], meas["el"], dt, now_t=now_t)
                track.set_mono_distance(meas["mono_dist"], now_t)
                track.set_detection_source(meas, now_t)
                unmatched_measurements.discard(m_idx)
                matched_tracks.add(t_idx)
            else:
                if DEBUG_KALMAN_MATCH:
                    print(
                        f"[MATCH_REJECT] track={track.id}, meas={m_idx}, "
                        f"meas=(Az={meas['az']:.2f}, El={meas['el']:.2f}), "
                        f"pred=(Az={pred_az:.2f}, El={pred_el:.2f}), "
                        f"cost={cost:.2f}, thresh={dynamic_thresh:.2f}, gate={gate_mode}, "
                        f"unc={uncertainty:.2f}, "
                        f"Ppos=({track.P[0,0]:.2f},{track.P[1,1]:.2f}), "
                        f"dist_thresh={track.dist_thresh:.2f}, "
                        f"hits={track.hit_streak}, lost={track.time_since_update}, "
                        f"R=({track.r_az:.2f},{track.r_el:.2f}), "
                        f"Q=({track.q_pos:.2f},{track.q_vel:.2f})"
                    )
                row = self._debug_context_row(debug_context, "MATCH_REJECT")
                field_log_event({
                    "timestamp": row["timestamp"],
                    "seq": row["frame_id"],
                    "mode": row["mode"],
                    "event": "MATCH_REJECT",
                    "track_id": int(track.id),
                    "meas_idx": m_idx,
                    "meas_az": f"{meas['az']:.6f}",
                    "meas_el": f"{meas['el']:.6f}",
                    "pred_az": f"{pred_az:.6f}",
                    "pred_el": f"{pred_el:.6f}",
                    "cost": f"{cost:.6f}",
                    "dynamic_thresh": f"{dynamic_thresh:.6f}",
                    "uncertainty": f"{uncertainty:.6f}",
                    "p_az": f"{track.P[0, 0]:.6f}",
                    "p_el": f"{track.P[1, 1]:.6f}",
                    "hit_streak": int(track.hit_streak),
                    "time_since_update": int(track.time_since_update),
                    "lost_seconds": f"{lost_s_before_match:.6f}",
                    "reason": f"pre_hungarian_gate:{gate_mode}",
                })

        # 未匹配轨迹衰减稳定帧，避免历史累计导致“永久霸榜”
        for t_idx, track in enumerate(self.tracks):
            if t_idx not in matched_tracks:
                track.hit_streak = max(0, track.hit_streak - HIT_STREAK_DECAY)

        # 5. 为没匹配上的坐标创建新 Track
        for m_idx in unmatched_measurements:
            meas = normalized_measurements[m_idx]
            t = StandardKalmanTrack(meas["az"], meas["el"], init_ts=now_t)
            t.set_mono_distance(meas["mono_dist"], now_t)
            t.set_detection_source(meas, now_t)
            if params:
                t.set_dynamic_params(params)
            else:
                dist_for_track = t.get_param_distance(now_t)
                if dist_for_track is not None:
                    t.set_dynamic_params(get_dynamic_tracking_params(dist_for_track))
                else:
                    t.dist_thresh = self.base_distance_threshold
            self.tracks.append(t)
            self._log_new_track(t, m_idx, meas, debug_context)

        # 6. 删除丢失太久的 Track
        self._prune_lost_tracks(now_t=now_t, debug_context=debug_context)

        return self.tracks

# ==========================================
# 4. 核心解算 V8 (修改版：基准水平90度)
# ==========================================
def calculate_angles(
    cam_key,
    cx,
    cy,
    cfg=None,
    image_w=IMG_W,
    image_h=IMG_H,
):
    base_az = cfg["theta_horizontal"] 
    base_el = cfg["theta_vertical"]   
    image_w = float(image_w)
    image_h = float(image_h)
    
    # 1. 计算目标在图像中的像素偏移
    diff_x = cx - (image_w / 2.0)
    diff_y = cy - (image_h / 2.0)
    
    # 2. 像素转换成角度偏移
    offset_az = diff_x * FOV_X / image_w
    offset_el = -diff_y * FOV_Y / image_h

    # 3. 计算系统绝对角度 (UI显示用, 保持0~360的罗盘习惯)
    ui_az = (base_az + offset_az) % 360.0
    ui_el = base_el + offset_el

    return ui_az, ui_el


def evaluate_track_threat(track, curr_gimbal_az, curr_gimbal_el, master_id=None):
    """保留现有调度权重，仅把评分拆出来便于审计与日志打印。"""
    v_mag = math.hypot(track.state[2, 0], track.state[3, 0])
    speed_score = v_mag * 2.0

    dist_az = angular_diff(track.state[0, 0], curr_gimbal_az)
    dist_el = track.state[1, 0] - curr_gimbal_el
    dist = math.hypot(dist_az, dist_el)
    distance_score = -dist * 1.0

    stability_level = min(track.hit_streak, STABILITY_HIT_CAP)
    stability_score = math.log1p(stability_level) * STABILITY_WEIGHT

    threat_score = speed_score + distance_score + stability_score
    inertia_applied = (track.id == master_id)
    if inertia_applied:
        threat_score *= 1.15

    return {
        "track": track,
        "track_id": int(track.id),
        "speed_deg_s": v_mag,
        "distance_deg": dist,
        "hit_streak": int(track.hit_streak),
        "speed_score": speed_score,
        "distance_score": distance_score,
        "stability_score": stability_score,
        "inertia_applied": inertia_applied,
        "threat_score": threat_score,
    }


def choose_master_track(valid_tracks, curr_gimbal_az, curr_gimbal_el, master_id=None):
    ranked_candidates = [
        evaluate_track_threat(t, curr_gimbal_az, curr_gimbal_el, master_id=master_id)
        for t in valid_tracks
    ]
    ranked_candidates.sort(key=lambda item: item["threat_score"], reverse=True)
    best_track = ranked_candidates[0]["track"] if ranked_candidates else None
    return best_track, ranked_candidates


def format_selection_candidates(ranked_candidates, topk=MASTER_SELECTION_LOG_TOPK):
    if not ranked_candidates:
        return "none"

    items = []
    for item in ranked_candidates[:topk]:
        inertia_text = ", hold=1.15x" if item["inertia_applied"] else ""
        items.append(
            f"id={item['track_id']}, score={item['threat_score']:.2f}, "
            f"speed={item['speed_deg_s']:.2f}deg/s, dist={item['distance_deg']:.2f}deg, "
            f"hits={item['hit_streak']}, stab={item['stability_score']:.2f}{inertia_text}"
        )
    return " | ".join(items)


def evaluate_strike_threat(track, curr_time, vision_track_result=None, strike_target_id=None):
    """Score targets for strike guidance.

    Unlike master selection, strike selection requires a fresh gimbal-camera
    distance and a safe bbox. The score then favors nearer, closing, stable
    tracks while adding inertia to avoid target ping-pong.
    """
    if (
        track is None
        or not track.confirmed
        or track.lost_seconds(curr_time) > MAX_LOCK_LOST_SECONDS
    ):
        return None

    vision_track_result = vision_track_result or {}
    frame_ts = float(vision_track_result.get("frame_ts", 0.0) or 0.0)
    vision_age = float(curr_time) - frame_ts
    if not (
        vision_track_result.get("distance_valid")
        and vision_track_result.get("safe")
        and 0.0 <= vision_age <= GIMBAL_VISION_RESULT_TTL
    ):
        return None

    distance_m, distance_source = get_smoothed_track_distance(track, curr_time)
    if distance_m is None:
        return None

    radial_velocity = (
        float(track.dist_state[1, 0])
        if track.dist_state is not None
        else 0.0
    )
    closing_speed_mps = max(0.0, -radial_velocity)
    stability_level = min(track.hit_streak, STABILITY_HIT_CAP)
    stability_score = math.log1p(stability_level) * 3.0
    distance_score = max(0.0, 600.0 - float(distance_m)) * 0.10
    closing_score = closing_speed_mps * 2.0
    freshness_score = max(0.0, GIMBAL_VISION_RESULT_TTL - vision_age) * 3.0

    threat_score = (
        distance_score
        + closing_score
        + stability_score
        + freshness_score
    )
    raw_threat_score = threat_score

    inertia_applied = (track.id == strike_target_id)
    if inertia_applied:
        threat_score *= 1.20

    return {
        "track": track,
        "track_id": int(track.id),
        "distance_m": float(distance_m),
        "distance_source": distance_source,
        "radial_velocity_mps": radial_velocity,
        "closing_speed_mps": closing_speed_mps,
        "hit_streak": int(track.hit_streak),
        "vision_age": vision_age,
        "distance_score": distance_score,
        "closing_score": closing_score,
        "stability_score": stability_score,
        "freshness_score": freshness_score,
        "inertia_applied": inertia_applied,
        "raw_threat_score": raw_threat_score,
        "threat_score": threat_score,
    }


def choose_strike_target(valid_tracks, curr_time, track_results, strike_target_id=None):
    ranked_candidates = []
    for track in valid_tracks:
        item = evaluate_strike_threat(
            track,
            curr_time,
            vision_track_result=track_results.get(int(track.id), {}),
            strike_target_id=strike_target_id,
        )
        if item is not None:
            ranked_candidates.append(item)
    ranked_candidates.sort(key=lambda item: item["threat_score"], reverse=True)
    best_track = ranked_candidates[0]["track"] if ranked_candidates else None
    return best_track, ranked_candidates


def format_strike_candidates(ranked_candidates, topk=MASTER_SELECTION_LOG_TOPK):
    if not ranked_candidates:
        return "none"
    items = []
    for item in ranked_candidates[:topk]:
        inertia_text = ", hold=1.20x" if item["inertia_applied"] else ""
        items.append(
            f"id={item['track_id']}, score={item['threat_score']:.2f}, "
            f"dist={item['distance_m']:.1f}m, close={item['closing_speed_mps']:.1f}m/s, "
            f"hits={item['hit_streak']}, age={item['vision_age']:.2f}s{inertia_text}"
        )
    return " | ".join(items)
# ==========================================
# 6. 主逻辑 V9 (多目标预测与云台调度)
# ==========================================
def main():
    global FIELD_LOGGER
    run_log_dir = None
    if LOG_TO_FILE or FIELD_LOG:
        try:
            run_log_dir = _create_run_log_dir(LOG_DIR)
        except Exception as e:
            run_log_dir = LOG_DIR
            print(f"[Log][Warn] 运行日志目录创建失败，回退到 {LOG_DIR}: {e}")

    if LOG_TO_FILE:
        try:
            log_path = _setup_log_mirror(run_log_dir or LOG_DIR)
            if log_path:
                print(f"[Log] stdout/stderr -> {log_path}")
        except Exception as e:
            print(f"[Log][Warn] 日志文件初始化失败: {e}")

    if FIELD_LOG:
        try:
            FIELD_LOGGER = FieldLogger(run_log_dir or FIELD_LOG_DIR)
        except Exception as e:
            FIELD_LOGGER = None
            print(f"[FieldLog][Warn] 初始化失败: {e}")

    sender = UISender(UI_IP, UI_PORT)
    strike_sender = StrikeSender(STRIKE_IP, STRIKE_PORT) if ENABLE_STRIKE_SEND else None
    print(f"[Config] DEVICE_HEADING_DEG={DEVICE_HEADING_DEG:.2f} (map north=0, east=90, south=180)")
    print(
        "[Config] Detection UDP coordinates: "
        f"mode={UDP_DETECTION_COORD_MODE}, "
        f"size={UDP_DETECTION_W:.0f}x{UDP_DETECTION_H:.0f}, "
        f"switch=USE_NIGHT_DETECTION_COORDS={USE_NIGHT_DETECTION_COORDS}"
    )
    print(
        "[Config] Track association safety: "
        f"hard_cap={TRACK_ASSOCIATION_MAX_DEG:.2f}°, "
        f"strict_after={TRACK_REACQUIRE_STRICT_AFTER_SECONDS:.2f}s, "
        f"strict_cap={TRACK_REACQUIRE_MAX_DEG:.2f}°, "
        f"internal_keep={TRACK_MAX_LOST_SECONDS:.2f}s, "
        f"ui_fresh={UI_MAX_LOST_SECONDS:.2f}s"
    )
    if ENABLE_STRIKE_SEND:
        print(
            f"[Strike] Enabled target UDP sender: {STRIKE_IP}:{STRIKE_PORT}, "
            f"hz={STRIKE_SEND_HZ:.1f}, window={STRIKE_WINDOW_SECONDS:.2f}s, "
            f"lead={STRIKE_LEAD_TIME:.2f}s, settled_ttl={STRIKE_SETTLED_EVENT_TTL:.2f}s"
        )
    if not USE_MOCK_GIMBAL:
        _validate_serial_port("GIMBAL_PORT", GIMBAL_PORT)
    if not USE_MOCK_LASER:
        _validate_serial_port("LASER_PORT", LASER_PORT)
    if ENABLE_GPS:
        _validate_serial_port("GPS_PORT", GPS_PORT)

    if ENABLE_GPS:
        threading.Thread(target=gps_sender_thread, args=(sender,), daemon=True).start()
    
    if USE_MOCK_GIMBAL:
        if MockGimbalAdapter is None:
            print("[Error] USE_MOCK_GIMBAL=True, but mock_gimbal.py import failed.")
            return
        print("[Init] Connecting to Mock Gimbal at MOCK_PORT...")
        gimbal = MockGimbalAdapter(port="MOCK_PORT", az_base=GIMBAL_AZ_BASE)
    else:
        print(f"[Init] Connecting to Gimbal at {GIMBAL_PORT}...")
        gimbal = GT06ZAdapter(port=GIMBAL_PORT)
    if not gimbal.connect():
        print("[Error] Failed to connect gimbal.")
        return
    
    if not gimbal.wait_ready():
        print("[Warning] Gimbal not ready instantly, wait...")

    laser = None
    laser_stop_event = None
    laser_thread = None
    if USE_MOCK_LASER:
        print("[Init] Real laser logging disabled by USE_MOCK_LASER=True; distance source remains mono")
    else:
        if SFL0603 is None:
            print(
                "[Laser][Warn] sfl0603_driver.py import failed; "
                "SFL0603 ranging disabled."
            )
        else:
            try:
                # Constructor sends STOP and requires its acknowledgement.
                # Ranging remains disarmed until visual alignment requests one
                # short 10 Hz session.
                laser = SFL0603(
                    LASER_PORT,
                    stop_on_open=True,
                    allow_ranging=False,
                )
                laser_stop_event = threading.Event()
                laser_thread = threading.Thread(
                    target=laser_reader_thread,
                    args=(laser, laser_stop_event),
                    daemon=True,
                )
                laser_thread.start()
                print(
                    f"[Init] SFL0603 ready in standby on {LASER_PORT}; "
                    f"target-continuous period={SFL0603_CONTINUOUS_PERIOD_MS}ms, "
                    f"no_return_scan_interval={SFL0603_SESSION_TIMEOUT_SECONDS:.2f}s"
                )
            except Exception as e:
                print(f"[Laser][Warn] SFL0603 init failed on {LASER_PORT}: {e}")
                laser = None

    def get_laser_preview_status():
        """Return a lock-consistent snapshot for the camera overlay."""
        with shared_state.lock:
            distance = shared_state.raw_laser_dist
            return {
                "available": laser is not None,
                "active": bool(shared_state.laser_active),
                "status": str(shared_state.laser_status),
                "distance_m": (
                    None if distance is None else float(distance)
                ),
                "timestamp": float(shared_state.raw_laser_ts or 0.0),
                "error": str(shared_state.laser_last_error or ""),
                "period_ms": SFL0603_CONTINUOUS_PERIOD_MS,
                "aim_radius_px": GIMBAL_LASER_READY_RADIUS_PX,
                "no_return_timestamp": float(
                    shared_state.laser_no_return_ts or 0.0
                ),
            }

    # start background threads for network and gimbal control
    push_latest_gimbal_cmd({
        "cmd_id": 0,
        "track_id": -1,
        "az": GIMBAL_AZ_BASE,
        "el": GIMBAL_INIT_EL,
        "ts": time.time(),
    })
    print(
        f"[Init] Queue gimbal initial posture: "
        f"Az={GIMBAL_AZ_BASE:.1f}°, El={GIMBAL_INIT_EL:.1f}°"
    )
    threading.Thread(target=gimbal_control_thread, args=(gimbal,), daemon=True).start()
    threading.Thread(target=rk3588_thread, daemon=True).start()

    vision_service = None
    if ENABLE_GIMBAL_VISION:
        if GimbalVisionRangingService is None:
            print("[GimbalVision][Warn] module import failed; visual distance disabled")
        else:
            try:
                vision_service = GimbalVisionRangingService(
                    camera_source=GIMBAL_CAMERA_SOURCE,
                    confidence=GIMBAL_VISION_CONFIDENCE,
                    settle_delay_s=GIMBAL_VISION_SETTLE_DELAY,
                    min_sharpness=GIMBAL_VISION_MIN_SHARPNESS,
                    association_max_px=GIMBAL_VISION_ASSOCIATION_MAX_PX,
                    association_ambiguity_margin_px=(
                        GIMBAL_VISION_AMBIGUITY_MARGIN_PX
                    ),
                    track_state_ttl_s=GIMBAL_VISION_TRACK_STATE_TTL,
                    # single-laser uses this camera only for YOLO guidance.
                    # Do not load or run physics/MLP/GRU distance inference.
                    detection_only=SINGLE_LASER_MODE,
                    aim_offset_x_px=GIMBAL_LASER_AIM_OFFSET_X_PX,
                    aim_offset_y_px=GIMBAL_LASER_AIM_OFFSET_Y_PX,
                    laser_status_provider=get_laser_preview_status,
                )
                vision_service.start()
                print(
                    f"[GimbalVision] enabled camera={GIMBAL_CAMERA_SOURCE!r}, "
                    "YOLO=bestall_2k.rknn native 2560x1440 on RKNN/NPU, "
                    "mode=YOLO_detection_only, distance=SFL0603_on_demand"
                )
            except Exception as e:
                vision_service = None
                print(f"[GimbalVision][Warn] initialization failed: {e}")
    else:
        print("[GimbalVision] disabled by ENABLE_GIMBAL_VISION=False")

    visual_alignment = None
    visual_target_lock = None
    if ENABLE_SINGLE_TARGET_YOLO_ALIGNMENT and vision_service is not None:
        if (
            SingleTargetAlignmentConfig is None
            or SingleTargetVisualAlignment is None
        ):
            print(
                "[GimbalAlign][Warn] alignment module import failed; "
                "single-target YOLO alignment disabled"
            )
        else:
            visual_alignment = SingleTargetVisualAlignment(
                SingleTargetAlignmentConfig(
                    image_width=IMG_W,
                    image_height=IMG_H,
                    aim_offset_x_px=GIMBAL_LASER_AIM_OFFSET_X_PX,
                    aim_offset_y_px=GIMBAL_LASER_AIM_OFFSET_Y_PX,
                    fov_x_deg=FOV_X,
                    fov_y_deg=FOV_Y,
                    stable_frames=GIMBAL_YOLO_ALIGN_STABLE_FRAMES,
                    max_center_std_x_px=GIMBAL_YOLO_ALIGN_MAX_STD_X_PX,
                    max_center_std_y_px=GIMBAL_YOLO_ALIGN_MAX_STD_Y_PX,
                    max_center_speed_x_px_s=(
                        GIMBAL_YOLO_ALIGN_MAX_SPEED_X_PX_S
                    ),
                    max_center_speed_y_px_s=(
                        GIMBAL_YOLO_ALIGN_MAX_SPEED_Y_PX_S
                    ),
                    trigger_x_px=GIMBAL_YOLO_ALIGN_TRIGGER_X_PX,
                    trigger_y_px=GIMBAL_YOLO_ALIGN_TRIGGER_Y_PX,
                    centered_x_px=GIMBAL_YOLO_ALIGN_CENTERED_X_PX,
                    centered_y_px=GIMBAL_YOLO_ALIGN_CENTERED_Y_PX,
                    max_frame_gap_s=GIMBAL_YOLO_ALIGN_MAX_FRAME_GAP_S,
                    max_latest_to_median_px=(
                        GIMBAL_YOLO_ALIGN_MAX_LATEST_TO_MEDIAN_PX
                    ),
                    max_center_jump_px=GIMBAL_YOLO_ALIGN_MAX_CENTER_JUMP_PX,
                    min_bbox_iou=GIMBAL_YOLO_ALIGN_MIN_BBOX_IOU,
                    recovery_confirm_frames=(
                        GIMBAL_YOLO_ALIGN_RECOVERY_CONFIRM_FRAMES
                    ),
                    max_frame_age_s=GIMBAL_YOLO_ALIGN_MAX_FRAME_AGE_S,
                    laser_ready_radius_px=GIMBAL_LASER_READY_RADIUS_PX,
                    min_fine_step_deg=GIMBAL_YOLO_ALIGN_MIN_FINE_STEP_DEG,
                    fine_scan_max_error_px=(
                        GIMBAL_YOLO_ALIGN_FINE_SCAN_MAX_ERROR_PX
                    ),
                    max_step_az_deg=GIMBAL_YOLO_ALIGN_MAX_STEP_AZ_DEG,
                    max_step_el_deg=GIMBAL_YOLO_ALIGN_MAX_STEP_EL_DEG,
                    max_corrections_per_lock=(
                        GIMBAL_YOLO_ALIGN_MAX_CORRECTIONS
                    ),
                )
            )
            if (
                GimbalVisualLockConfig is not None
                and GimbalVisualTargetLock is not None
            ):
                visual_target_lock = GimbalVisualTargetLock(
                    GimbalVisualLockConfig(
                        image_width=IMG_W,
                        image_height=IMG_H,
                        aim_x_px=(
                            IMG_W / 2.0 + GIMBAL_LASER_AIM_OFFSET_X_PX
                        ),
                        aim_y_px=(
                            IMG_H / 2.0 + GIMBAL_LASER_AIM_OFFSET_Y_PX
                        ),
                        min_x_ratio=GIMBAL_VISUAL_LOCK_MIN_X_RATIO,
                        max_x_ratio=GIMBAL_VISUAL_LOCK_MAX_X_RATIO,
                        min_y_ratio=GIMBAL_VISUAL_LOCK_MIN_Y_RATIO,
                        max_y_ratio=GIMBAL_VISUAL_LOCK_MAX_Y_RATIO,
                        lost_timeout_s=GIMBAL_VISUAL_LOCK_LOST_SECONDS,
                        outside_confirm_frames=(
                            GIMBAL_VISUAL_LOCK_OUTSIDE_CONFIRM_FRAMES
                        ),
                    )
                )
            print(
                "[GimbalAlign] enabled direct single-bbox alignment: "
                f"bbox_samples={GIMBAL_YOLO_ALIGN_STABLE_FRAMES}, "
                f"std<=({GIMBAL_YOLO_ALIGN_MAX_STD_X_PX:.1f},"
                f"{GIMBAL_YOLO_ALIGN_MAX_STD_Y_PX:.1f})px, "
                f"speed<=({GIMBAL_YOLO_ALIGN_MAX_SPEED_X_PX_S:.1f},"
                f"{GIMBAL_YOLO_ALIGN_MAX_SPEED_Y_PX_S:.1f})px/s, "
                f"trigger=({GIMBAL_YOLO_ALIGN_TRIGGER_X_PX:.1f},"
                f"{GIMBAL_YOLO_ALIGN_TRIGGER_Y_PX:.1f})px, "
                f"centered=({GIMBAL_YOLO_ALIGN_CENTERED_X_PX:.1f},"
                f"{GIMBAL_YOLO_ALIGN_CENTERED_Y_PX:.1f})px, "
                f"frame_gap={'disabled' if GIMBAL_YOLO_ALIGN_MAX_FRAME_GAP_S <= 0.0 else f'{GIMBAL_YOLO_ALIGN_MAX_FRAME_GAP_S:.1f}s'}, "
                f"recover_frames={GIMBAL_YOLO_ALIGN_RECOVERY_CONFIRM_FRAMES}, "
                "bbox_age_check=disabled, "
                f"laser_ready_radius={GIMBAL_LASER_READY_RADIUS_PX:.1f}px, "
                "control=direct_bbox_to_angle_0.1deg, "
                f"laser_aim=({IMG_W / 2.0 + GIMBAL_LASER_AIM_OFFSET_X_PX:.1f},"
                f"{IMG_H / 2.0 + GIMBAL_LASER_AIM_OFFSET_Y_PX:.1f})px, "
                f"max_corrections={'disabled' if GIMBAL_YOLO_ALIGN_MAX_CORRECTIONS <= 0 else GIMBAL_YOLO_ALIGN_MAX_CORRECTIONS}"
            )
            if visual_target_lock is not None:
                print(
                    "[GimbalVisualLock] UDP-independent lock enabled: "
                    f"area=x[{GIMBAL_VISUAL_LOCK_MIN_X_RATIO:.2f},"
                    f"{GIMBAL_VISUAL_LOCK_MAX_X_RATIO:.2f}],"
                    f"y[{GIMBAL_VISUAL_LOCK_MIN_Y_RATIO:.2f},"
                    f"{GIMBAL_VISUAL_LOCK_MAX_Y_RATIO:.2f}], "
                    f"lost_timeout={GIMBAL_VISUAL_LOCK_LOST_SECONDS:.2f}s, "
                    "outside_confirm_frames="
                    f"{GIMBAL_VISUAL_LOCK_OUTSIDE_CONFIRM_FRAMES}"
                )
    elif ENABLE_SINGLE_TARGET_YOLO_ALIGNMENT:
        print(
            "[GimbalAlign] requested but gimbal vision is unavailable; disabled"
        )
    else:
        print(
            "[GimbalAlign] disabled by "
            "ENABLE_SINGLE_TARGET_YOLO_ALIGNMENT=False"
        )

    distance_mode = "SFL0603 target-continuous -> requested master_id"
    print(
        f"[Init] UI/Strike distance source: {distance_mode} "
        f"(ttl={TRACK_DISTANCE_TTL:.1f}s)"
    )

    
    print("=== System V9.0 (Predictive Tracking & Scheduling) Running ===")

    # 初始化追踪大脑
    tracker = MultiTargetTracker(
        max_lost_frames=50,
        max_lost_seconds=TRACK_MAX_LOST_SECONDS,
        distance_threshold=1.2,
    )
    
    # 状态机与调度变量
    master_id = None
    master_epoch_ts = 0.0
    strike_target_id = None
    strike_challenger_id = None
    strike_challenger_since = 0.0
    PREDICT_DELAY = 0.3     # 系统与物理响应总延迟 (打提前量)
    CONFIRM_HITS = TRACK_CONFIRM_HITS  # internal track gate; UI/strike use stricter external gates
    MAX_DT = 0.25            # Clamp dt to avoid model divergence
    global_cmd_id = 0
    last_sent_ctrl_az = None
    last_sent_ctrl_el = None
    challenger_id = None
    challenger_since = 0.0
    angle_unsafe_frames = 0
    last_applied_vision_ts = {}
    last_logged_vision_frame_ts = 0.0
    last_bound_laser_ts = 0.0
    laser_continuous_track_id = None
    laser_continuous_request_ts = 0.0
    laser_scan_track_id = None
    laser_scan_index = 0
    last_scanned_no_return_ts = 0.0
    last_control_vision_frame_ts = 0.0
    strike_window = {
        "track_id": None,
        "distance": None,
        "source": "none",
        "valid_until": 0.0,
        "last_send_ts": 0.0,
        "last_consumed_settled_cmd_id": -1,
    }
    
    last_time = time.time()
    # 统计日志：接收坐标与UI发送ID
    recv_obj_total = 0
    recv_unique_boxes = set()  # {(x1,y1,x2,y2), ...}
    ui_send_total = 0
    ui_send_counter = Counter()  # {ui_id: send_count}
    next_ui_id = 1
    track_to_ui_id = {}  # Allocate a stable UI ID when any valid track is first sent.
    stats_last_print = last_time
    live_last_print = last_time
    live_packet_count = 0
    live_obj_count = 0
    live_last_packet_t = 0.0
    live_last_meas_count = 0
    live_last_track_count = 0
    live_last_valid_count = 0
    fusion_packet_buffer = []
    fusion_window_start_t = 0.0

    def get_or_assign_ui_id(track):
        nonlocal next_ui_id, track_to_ui_id
        internal_id = int(track.id)
        if internal_id not in track_to_ui_id:
            track_to_ui_id[internal_id] = next_ui_id
            next_ui_id += 1
        return track_to_ui_id[internal_id]

    def maybe_print_live_status(now_t, meas_count=0, active_tracks=None, valid_tracks=None):
        nonlocal live_last_print, live_packet_count, live_obj_count
        nonlocal live_last_meas_count, live_last_track_count, live_last_valid_count
        if meas_count is not None:
            live_last_meas_count = meas_count
        if active_tracks is not None:
            live_last_track_count = len(active_tracks)
        if valid_tracks is not None:
            live_last_valid_count = len(valid_tracks)
        if (not PRINT_LIVE_STATUS) or ((now_t - live_last_print) < LIVE_STATUS_INTERVAL):
            return
        interval = max(now_t - live_last_print, 1e-6)
        udp_rate = live_packet_count / interval
        obj_rate = live_obj_count / interval
        last_udp_age = "" if live_last_packet_t <= 0 else f"{now_t - live_last_packet_t:.2f}s"
        print(
            f"[LIVE] udp={udp_rate:.1f}/s, objs={obj_rate:.1f}/s, "
            f"last_udp={last_udp_age}, meas={live_last_meas_count}, "
            f"tracks={live_last_track_count}, valid={live_last_valid_count}, master={master_id}"
        )
        live_last_print = now_t
        live_packet_count = 0
        live_obj_count = 0

    def summarize_window_field(pkgs, field):
        values = []
        for item in pkgs:
            value = item.get(field, "")
            if value in (None, ""):
                continue
            value = str(value)
            if value not in values:
                values.append(value)
        return ";".join(values)

    def packet_source_key(pkg):
        board = str(pkg.get("board", "Unknown"))
        try:
            cam = int(pkg.get("cam", 0))
        except (TypeError, ValueError):
            cam = str(pkg.get("cam", ""))
        return board, cam

    def keep_latest_packet_per_source(pkgs):
        """同一融合窗口内，每个物理摄像头只保留最新 UDP 包。

        这样融合窗口可以覆盖跨摄像头异步上报，同时避免同一摄像头
        连续两帧进入同一个 tracker.update()，造成重复观测/重复建轨。
        同一个最新包里的多个 objs 会全部保留，不影响同画面多目标。
        """
        latest_by_source = {}
        for pkg in pkgs:
            latest_by_source[packet_source_key(pkg)] = pkg
        latest_pkgs = sorted(
            latest_by_source.values(),
            key=lambda item: float(item.get("_recv_ts", 0.0) or 0.0),
        )
        return latest_pkgs, max(0, len(pkgs) - len(latest_pkgs))

    while True:
        try:
            curr_time = time.time()
            vision_only_tick = False

            # --- 1. 获取 UDP 数据：短时间窗内的分摄像头包合成一个逻辑帧 ---
            while packet_queue:
                pkg = packet_queue.popleft()
                if not fusion_packet_buffer:
                    fusion_window_start_t = float(pkg.get("_recv_ts", curr_time) or curr_time)
                fusion_packet_buffer.append(pkg)

                raw_objs = pkg.get("objs", [])
                live_packet_count += 1
                live_obj_count += len(raw_objs) if isinstance(raw_objs, list) else 0
                live_last_packet_t = curr_time

            if not fusion_packet_buffer:
                pending_vision_frame_ts = 0.0
                if vision_service is not None and master_id is not None:
                    with shared_state.lock:
                        no_udp_gimbal_stationary = bool(
                            shared_state.is_stationary
                        )
                        no_udp_stationary_ts = float(
                            shared_state.stationary_ts or 0.0
                        )
                    # A previous visual command sets the camera worker to
                    # GIMBAL_MOVING. Refresh encoder stationarity here so the
                    # worker resumes YOLO after settling even without UDP.
                    vision_service.update_context(
                        master_track_id=master_id,
                        gimbal_settled=no_udp_gimbal_stationary,
                        settled_ts=no_udp_stationary_ts,
                        track_predictions=[],
                    )
                    pending_vision_result = vision_service.get_result()
                    pending_vision_frame_ts = float(
                        pending_vision_result.get("frame_ts", 0.0) or 0.0
                    )
                vision_only_tick = bool(
                    should_run_vision_only_tick is not None
                    and should_run_vision_only_tick(
                        has_udp_packets=False,
                        master_track_id=master_id,
                        vision_frame_ts=pending_vision_frame_ts,
                        last_control_frame_ts=last_control_vision_frame_ts,
                    )
                )

            if not fusion_packet_buffer and not vision_only_tick:
                if tracker.tracks and (curr_time - last_time) >= NO_PACKET_TRACKER_UPDATE_INTERVAL:
                    dt = curr_time - last_time
                    if dt > MAX_DT:
                        dt = MAX_DT
                    tracker.update(
                        [],
                        dt,
                        now_t=curr_time,
                        debug_context={
                            "mode": "no_packet",
                            "frame_id": "",
                            "meas_count": 0,
                        },
                    )
                    last_time = curr_time
                if laser is not None and master_id is not None:
                    with shared_state.lock:
                        if laser_continuous_track_id != int(master_id):
                            laser_continuous_track_id = int(master_id)
                            laser_continuous_request_ts = curr_time
                        shared_state.laser_request_ts = (
                            laser_continuous_request_ts
                        )
                        shared_state.laser_request_track_id = int(master_id)
                        shared_state.laser_request_heartbeat_ts = curr_time
                maybe_print_live_status(curr_time, meas_count=None)
                time.sleep(0.005) # 稍微让出 CPU
                continue

            if (
                not vision_only_tick
                and (curr_time - fusion_window_start_t)
                < MEAS_FUSION_WINDOW_SECONDS
            ):
                maybe_print_live_status(curr_time, meas_count=None)
                time.sleep(0.005)
                continue

            if vision_only_tick:
                window_pkgs = [{
                    "board": "gimbal_camera",
                    "cam": 0,
                    "mode": "vision_only",
                    "seq": "",
                    "objs": [],
                    "_recv_ts": curr_time,
                }]
                raw_window_packet_count = 0
                same_source_packet_drop_count = 0
                used_window_packet_count = 0
            else:
                window_pkgs = fusion_packet_buffer
                fusion_packet_buffer = []
                fusion_window_start_t = 0.0
                raw_window_packet_count = len(window_pkgs)
                window_pkgs, same_source_packet_drop_count = (
                    keep_latest_packet_per_source(window_pkgs)
                )
                used_window_packet_count = len(window_pkgs)

            #计算两次逻辑观测帧的间隔时间
            dt = curr_time - last_time
            if dt <= 0:
                dt = 1.0 / 10.0
            #有可能两包数据间隔很久，为了防止追踪器模型发散，限制最大 dt
            if dt > MAX_DT:
                dt = MAX_DT
            last_time = curr_time

            frame_ref_pkg = window_pkgs[-1]
            board_str = frame_ref_pkg.get("board", "Unknown")
            try:
                cam_idx = int(frame_ref_pkg.get("cam", 0))
            except (TypeError, ValueError):
                cam_idx = 0
            sender_mode = summarize_window_field(window_pkgs, "mode")
            sender_seq = summarize_window_field(window_pkgs, "seq")

            # --- 2. 坐标解析为绝对角度 ---
            raw_measurements = []

            with shared_state.lock:
                shared_gimbal_az = shared_state.gimbal_az
                shared_gimbal_el = shared_state.gimbal_el
                active_gimbal_track_id = shared_state.active_track_id
                settled_cmd_id = shared_state.settled_cmd_id
                settled_track_id = shared_state.settled_track_id
                settled_ts = shared_state.settled_ts
                gimbal_is_settled = shared_state.is_settled
                gimbal_is_stationary = shared_state.is_stationary
                gimbal_stationary_ts = shared_state.stationary_ts
                raw_laser_dist = shared_state.raw_laser_dist
                raw_laser_ts = shared_state.raw_laser_ts
                raw_laser_request_ts = shared_state.raw_laser_request_ts
                raw_laser_track_id = shared_state.raw_laser_track_id
                laser_request_ts_snapshot = shared_state.laser_request_ts
                laser_request_track_id_snapshot = (
                    shared_state.laser_request_track_id
                )
                laser_no_return_ts_snapshot = (
                    shared_state.laser_no_return_ts
                )

            for pkt in window_pkgs:
                pkt_board_str = pkt.get("board", "Unknown")
                try:
                    pkt_cam_idx = int(pkt.get("cam", 0))
                except (TypeError, ValueError):
                    pkt_cam_idx = 0
                pkt_mode = pkt.get("mode", "")
                pkt_seq = pkt.get("seq", "")
                parsed_objs = parse_udp_objects(pkt.get("objs", []))

                for obj_raw_idx, obj_item in enumerate(parsed_objs):
                    raw_rect = obj_item["box"]
                    mono_dist = obj_item["mono_dist"]
                    if SINGLE_LASER_MODE:
                        mono_dist = None
                    obj_board = obj_item.get("board") or pkt_board_str
                    obj_cam_raw = obj_item.get("cam")
                    try:
                        obj_cam_idx = int(obj_cam_raw if obj_cam_raw is not None else pkt_cam_idx)
                    except (TypeError, ValueError):
                        print(f"[Warning] 无效摄像头ID: board={obj_board}, cam={obj_cam_raw}")
                        continue
                    logic_id, cfg = get_camera_params(obj_board, obj_cam_idx)
                    if logic_id is None:
                        continue
                    recv_obj_total += 1
                    bbox_info, bbox_reject_reason = sanitize_bbox(
                        raw_rect,
                        image_w=UDP_DETECTION_W,
                        image_h=UDP_DETECTION_H,
                    )
                    if bbox_info is None:
                        try:
                            raw_log_values = [float(raw_rect[i]) for i in range(4)]
                        except (TypeError, ValueError, IndexError):
                            raw_log_values = ["", "", "", ""]
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": pkt_seq,
                            "mode": pkt_mode,
                            "event": "BBOX_REJECT",
                            "meas_idx": obj_raw_idx,
                            "reason": bbox_reject_reason,
                            "raw_bbox_x1": "" if raw_log_values[0] == "" else f"{raw_log_values[0]:.3f}",
                            "raw_bbox_y1": "" if raw_log_values[1] == "" else f"{raw_log_values[1]:.3f}",
                            "raw_bbox_x2": "" if raw_log_values[2] == "" else f"{raw_log_values[2]:.3f}",
                            "raw_bbox_y2": "" if raw_log_values[3] == "" else f"{raw_log_values[3]:.3f}",
                        })
                        continue
                    rect = bbox_info["clipped"]
                    raw_rect = bbox_info["raw"]
                    recv_unique_boxes.add((
                        int(round(raw_rect[0])),
                        int(round(raw_rect[1])),
                        int(round(raw_rect[2])),
                        int(round(raw_rect[3])),
                    ))
                    if bbox_info["is_edge_bbox"]:
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": pkt_seq,
                            "mode": pkt_mode,
                            "event": "BBOX_CLIPPED",
                            "meas_idx": obj_raw_idx,
                            "reason": "bbox_out_of_image_bounds",
                            "raw_bbox_x1": f"{raw_rect[0]:.3f}",
                            "raw_bbox_y1": f"{raw_rect[1]:.3f}",
                            "raw_bbox_x2": f"{raw_rect[2]:.3f}",
                            "raw_bbox_y2": f"{raw_rect[3]:.3f}",
                            "clipped_bbox_x1": f"{rect[0]:.3f}",
                            "clipped_bbox_y1": f"{rect[1]:.3f}",
                            "clipped_bbox_x2": f"{rect[2]:.3f}",
                            "clipped_bbox_y2": f"{rect[3]:.3f}",
                            "is_edge_bbox": 1,
                            "visible_ratio": f"{bbox_info['visible_ratio']:.6f}",
                        })

                    cx = (rect[0] + rect[2]) / 2.0
                    cy = (rect[1] + rect[3]) / 2.0

                    res = calculate_angles(
                        logic_id,
                        cx,
                        cy,
                        cfg,
                        image_w=UDP_DETECTION_W,
                        image_h=UDP_DETECTION_H,
                    )
                    if res:
                        ui_az, ui_el = res
                        d_az_to_target = angular_diff(ui_az, shared_gimbal_az)
                        d_el_to_target = ui_el - shared_gimbal_el
                        # turn_dir = get_turn_direction_label(d_az_to_target, d_el_to_target)
                        meas_idx = len(raw_measurements)
                        raw_measurements.append({
                            "az": ui_az,
                            "el": ui_el,
                            "mono_dist": mono_dist,
                            "board": obj_board,
                            "cam": obj_cam_idx,
                            "logic_id": logic_id,
                            "source_ts": float(pkt.get("_recv_ts", curr_time) or curr_time),
                            "raw_meas_idx": meas_idx,
                            "is_edge_bbox": bbox_info["is_edge_bbox"],
                            "visible_ratio": bbox_info["visible_ratio"],
                        })
                        if FIELD_LOGGER is not None:
                            FIELD_LOGGER.write_measurement({
                                "timestamp": f"{curr_time:.6f}",
                                "seq": pkt_seq,
                                "mode": pkt_mode,
                                "board": obj_board,
                                "cam": obj_cam_idx,
                                "logic_id": logic_id,
                                "meas_idx": meas_idx,
                                "raw_bbox_x1": f"{raw_rect[0]:.3f}",
                                "raw_bbox_y1": f"{raw_rect[1]:.3f}",
                                "raw_bbox_x2": f"{raw_rect[2]:.3f}",
                                "raw_bbox_y2": f"{raw_rect[3]:.3f}",
                                "raw_bbox_w": f"{bbox_info['raw_w']:.3f}",
                                "raw_bbox_h": f"{bbox_info['raw_h']:.3f}",
                                "clipped_bbox_x1": f"{rect[0]:.3f}",
                                "clipped_bbox_y1": f"{rect[1]:.3f}",
                                "clipped_bbox_x2": f"{rect[2]:.3f}",
                                "clipped_bbox_y2": f"{rect[3]:.3f}",
                                "clipped_bbox_w": f"{bbox_info['clipped_w']:.3f}",
                                "clipped_bbox_h": f"{bbox_info['clipped_h']:.3f}",
                                "bbox_x1": f"{rect[0]:.3f}",
                                "bbox_y1": f"{rect[1]:.3f}",
                                "bbox_x2": f"{rect[2]:.3f}",
                                "bbox_y2": f"{rect[3]:.3f}",
                                "bbox_cx": f"{cx:.3f}",
                                "bbox_cy": f"{cy:.3f}",
                                "bbox_w": f"{rect[2] - rect[0]:.3f}",
                                "bbox_h": f"{rect[3] - rect[1]:.3f}",
                                "is_edge_bbox": 1 if bbox_info["is_edge_bbox"] else 0,
                                "visible_ratio": f"{bbox_info['visible_ratio']:.6f}",
                                "mono_dist": "" if mono_dist is None else f"{mono_dist:.6f}",
                                "meas_az": f"{ui_az:.6f}",
                                "meas_el": f"{ui_el:.6f}",
                            })
                        if PRINT_PHASE_LOGS:
                            if mono_dist is not None:
                                print(
                                    f"\n[Phase 1: 视觉解析] 收到目标 cx={cx:.1f}, cy={cy:.1f}, mono={mono_dist:.1f}m"
                                    f" -> 解算绝对角度: Az={ui_az:.2f}°, El={ui_el:.2f}°"
                                )
                            else:
                                print(
                                    f"\n[Phase 1: 视觉解析] 收到目标 cx={cx:.1f}, cy={cy:.1f}"
                                    f" -> 解算绝对角度: Az={ui_az:.2f}°, El={ui_el:.2f}°"
                                )
                        # print(
                        #     f"[DirCheck] gimbal_ui=(Az={shared_gimbal_az:.2f}°, El={shared_gimbal_el:.2f}°) "
                        #     f"target_delta=(dAz={d_az_to_target:.2f}°, dEl={d_el_to_target:.2f}°) => turn={turn_dir}"
                        # )
            # --- 3. 喂给 Tracker 更新所有目标轨迹 ---
            current_measurements, fusion_groups = fuse_measurements_by_angle(
                raw_measurements,
                threshold_deg=MEAS_FUSION_THRESHOLD_DEG,
            )
            fusion_groups_text = format_fusion_groups(fusion_groups)
            debug_context = {
                "mode": sender_mode,
                "frame_id": sender_seq,
                "meas_count": len(current_measurements),
            }
            active_tracks = tracker.update(
                current_measurements,
                dt,
                now_t=curr_time,
                debug_context=debug_context,
            )

            # Internal valid tracks: used by master selection, gimbal scheduling and vision-ranging binding.
            # External UI/strike IDs are exposed only after stricter gates, so short false alarms do not consume public IDs.
            valid_tracks = [
                t for t in active_tracks
                if t.confirmed
                and t.lost_seconds(curr_time) <= MAX_LOCK_LOST_SECONDS
            ]
            # The laser result binding below is independent of the optional
            # gimbal-vision service, so keep this lookup in the common scope.
            track_by_id = {
                int(track.id): track for track in valid_tracks
            }
            for t in valid_tracks:
                if t.hit_streak >= UI_TRACK_CONFIRM_HITS:
                    t.ui_confirmed = True
                if t.hit_streak >= STRIKE_TRACK_CONFIRM_HITS:
                    t.strike_confirmed = True
            ui_tracks = [
                t for t in valid_tracks
                if getattr(t, "ui_confirmed", False)
                and track_is_ui_fresh(t, curr_time)
            ]
            strike_valid_tracks = [
                t for t in valid_tracks
                if getattr(t, "ui_confirmed", False)
                and getattr(t, "strike_confirmed", False)
            ]

            # --- 4. 状态机：调度决策 ---
            visual_master_held = bool(
                visual_target_lock is not None
                and visual_target_lock.holds_sort_track(master_id)
            )
            master_track = next(
                (t for t in valid_tracks if t.id == master_id),
                None,
            )
            if master_track is None and visual_master_held:
                master_track = next(
                    (t for t in active_tracks if t.id == master_id),
                    None,
                )
            prev_master_id = master_id
            master_lost = (
                prev_master_id is not None
                and master_track is None
                and not visual_master_held
            )

            if master_lost:
                if PRINT_EVENT_LOGS:
                    print(
                        f"[TargetLost] master_id={prev_master_id} 不再满足锁定条件: "
                        f"valid_ids={[int(t.id) for t in valid_tracks]}"
                    )
                field_log_event({
                    "timestamp": f"{curr_time:.6f}",
                    "seq": sender_seq,
                    "mode": sender_mode,
                    "event": "TargetLost",
                    "track_id": int(prev_master_id),
                    "reason": "not_in_valid_tracks",
                })
                clear_strike_window(strike_window)
                if visual_target_lock is not None:
                    visual_target_lock.release()
                if visual_alignment is not None:
                    visual_alignment.release_target()
                master_id = None
                prev_master_id = None

            curr_gimbal_az = shared_gimbal_az
            curr_gimbal_el = shared_gimbal_el
            if visual_master_held:
                best_track = master_track
                ranked_candidates = []
            else:
                best_track, ranked_candidates = choose_master_track(
                    valid_tracks,
                    curr_gimbal_az,
                    curr_gimbal_el,
                    master_id=master_id,
                )
            selection_reason = None

            if master_track is None and best_track is not None:
                selection_reason = "master_lost" if master_lost else "initial_acquire"
                master_track = best_track
            elif master_track is not None and best_track is not None and best_track.id != master_track.id:
                eval_by_id = {
                    item["track_id"]: item for item in ranked_candidates
                }
                current_eval = eval_by_id.get(int(master_track.id))
                best_eval = eval_by_id.get(int(best_track.id))
                score_margin = (
                    best_eval["threat_score"] - current_eval["threat_score"]
                    if current_eval is not None and best_eval is not None
                    else -math.inf
                )
                if score_margin >= MASTER_SWITCH_SCORE_MARGIN:
                    if challenger_id != best_track.id:
                        challenger_id = best_track.id
                        challenger_since = curr_time
                    elif (curr_time - challenger_since) >= MASTER_SWITCH_CONFIRM_SECONDS:
                        selection_reason = "confirmed_higher_threat"
                        master_track = best_track
                else:
                    challenger_id = None
                    challenger_since = 0.0
            else:
                challenger_id = None
                challenger_since = 0.0

            if master_track is not None and (
                master_id is None or master_track.id != master_id
            ):
                old_master_id = master_id
                master_id = master_track.id
                master_epoch_ts = curr_time
                if visual_target_lock is not None:
                    visual_target_lock.start(master_id, curr_time)
                challenger_id = None
                challenger_since = 0.0
                angle_unsafe_frames = 0
                clear_strike_window(strike_window)
                candidates_text = format_selection_candidates(ranked_candidates)
                event_name = "TargetAcquire" if old_master_id is None else "TargetSwitch"
                if PRINT_EVENT_LOGS:
                    print(
                        f"[{event_name}] reason={selection_reason}, "
                        f"from={old_master_id}, to={master_id}, "
                        f"candidates={candidates_text}"
                    )
                field_log_event({
                    "timestamp": f"{curr_time:.6f}",
                    "seq": sender_seq,
                    "mode": sender_mode,
                    "event": event_name,
                    "track_id": int(master_id),
                    "reason": selection_reason,
                })
            track_ids = [int(t.id) for t in active_tracks]
            valid_ids = [int(t.id) for t in valid_tracks]
            hit_values = [int(t.hit_streak) for t in active_tracks]
            lost_values = [int(t.time_since_update) for t in active_tracks]
            lost_seconds_values = [
                f"{t.lost_seconds(curr_time):.3f}" for t in active_tracks
            ]
            track_states = ";".join(
                f"{int(t.id)}:{t.state[0,0]:.4f},{t.state[1,0]:.4f},{t.state[2,0]:.4f},{t.state[3,0]:.4f}"
                for t in active_tracks
            )
            if FIELD_LOGGER is not None:
                FIELD_LOGGER.write_summary({
                    "timestamp": f"{curr_time:.6f}",
                    "seq": sender_seq,
                    "mode": sender_mode,
                    "dt": f"{dt:.6f}",
                    "window_packet_count": raw_window_packet_count,
                    "used_packet_count": used_window_packet_count,
                    "same_source_packet_drop_count": same_source_packet_drop_count,
                    "raw_meas_count": len(raw_measurements),
                    "fused_meas_count": len(current_measurements),
                    "fusion_groups": fusion_groups_text,
                    "meas_count": len(current_measurements),
                    "track_count": len(active_tracks),
                    "valid_count": len(valid_tracks),
                    "track_ids": ";".join(str(x) for x in track_ids),
                    "valid_ids": ";".join(str(x) for x in valid_ids),
                    "master_id": "" if master_id is None else int(master_id),
                    "hit_streaks": ";".join(str(x) for x in hit_values),
                    "time_since_updates": ";".join(str(x) for x in lost_values),
                    "lost_seconds": ";".join(lost_seconds_values),
                    "track_states": track_states,
                    "cmd_az": "" if last_sent_ctrl_az is None else f"{last_sent_ctrl_az:.1f}",
                    "cmd_el": "" if last_sent_ctrl_el is None else f"{last_sent_ctrl_el:.1f}",
                    "gimbal_ui_az": f"{shared_gimbal_az:.6f}",
                    "gimbal_ui_el": f"{shared_gimbal_el:.6f}",
                })
            if DEBUG_TRACKER:
                print(
                    f"[TRACK_SUMMARY] raw_meas={len(raw_measurements)}, "
                    f"fused_meas={len(current_measurements)}, "
                    f"pkts={used_window_packet_count}/{raw_window_packet_count}, "
                    f"same_src_drop={same_source_packet_drop_count}, "
                    f"tracks={len(active_tracks)}, "
                    f"valid={len(valid_tracks)}, "
                    f"ids={track_ids}, "
                    f"valid_ids={valid_ids}, "
                    f"master={master_id}, "
                    f"hits={hit_values}, "
                    f"lost={lost_values}, "
                    f"lost_s={lost_seconds_values}"
                )

            vision_result = {
                "state": "DISABLED",
                "track_id": None,
                "distance_valid": False,
                "distance": float("nan"),
                "frame_ts": 0.0,
                "reposition_requested": False,
                "bbox": None,
            }
            alignment_decision = {
                "state": "DISABLED",
                "processed_new_frame": False,
                "command_requested": False,
                "laser_ready": False,
            }
            simple_measurements = []
            aligned_measurement = None
            visual_lock_decision = {
                "state": "DISABLED",
                "measurement": None,
                "release": False,
            }
            if vision_service is not None:
                vision_service.update_context(
                    master_track_id=master_id,
                    # Distance inference needs a stable image, not necessarily
                    # exact convergence to the requested mechanical angle.
                    gimbal_settled=gimbal_is_stationary,
                    settled_ts=gimbal_stationary_ts,
                    # single-laser is intentionally single-target: YOLO does
                    # not need SORT projection candidates or Hungarian match.
                    track_predictions=[],
                )
                vision_result = vision_service.get_result()
                simple_measurements = (
                    vision_result.get("simple_measurements", []) or []
                )
                vision_frame_ts = float(
                    vision_result.get("frame_ts", 0.0) or 0.0
                )
                if vision_frame_ts > 0.0:
                    last_control_vision_frame_ts = max(
                        last_control_vision_frame_ts,
                        vision_frame_ts,
                    )
                if (
                    visual_target_lock is not None
                    and visual_target_lock.holds_sort_track(master_id)
                ):
                    visual_lock_decision = visual_target_lock.update(
                        frame_ts=vision_frame_ts,
                        measurements=simple_measurements,
                        now_ts=curr_time,
                    )
                    aligned_measurement = visual_lock_decision.get(
                        "measurement"
                    )
                    if visual_lock_decision.get("release", False):
                        released_master_id = master_id
                        release_reason = visual_lock_decision.get(
                            "reason", "gimbal_visual_lock_released"
                        )
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "GIMBAL_VISUAL_LOCK_RELEASE",
                            "track_id": (
                                "" if released_master_id is None
                                else int(released_master_id)
                            ),
                            "master_id": (
                                "" if released_master_id is None
                                else int(released_master_id)
                            ),
                            "simple_id": visual_lock_decision.get(
                                "simple_id", ""
                            ),
                            "reason": (
                                f"{release_reason},"
                                f"state={visual_lock_decision.get('state')}"
                            ),
                        })
                        visual_target_lock.release()
                        if visual_alignment is not None:
                            visual_alignment.release_target()
                        master_id = None
                        master_track = None
                        master_epoch_ts = 0.0
                        challenger_id = None
                        challenger_since = 0.0
                        angle_unsafe_frames = 0
                        clear_strike_window(strike_window)
                        with shared_state.lock:
                            shared_state.laser_request_ts = 0.0
                            shared_state.laser_request_track_id = -1
                            shared_state.laser_request_heartbeat_ts = 0.0
                        vision_service.update_context(
                            master_track_id=None,
                            gimbal_settled=gimbal_is_stationary,
                            settled_ts=gimbal_stationary_ts,
                            track_predictions=[],
                        )
                elif master_id is not None and len(simple_measurements) == 1:
                    # Compatibility fallback if the lock helper is unavailable.
                    aligned_measurement = simple_measurements[0]
                direct_master_results = {}
                if master_id is not None and aligned_measurement is not None:
                    direct_result = dict(aligned_measurement)
                    direct_result["track_id"] = int(master_id)
                    direct_result["association_error_px"] = 0.0
                    direct_result["binding_mode"] = "single_target_master"
                    direct_master_results[int(master_id)] = direct_result
                vision_result["track_results"] = direct_master_results
                vision_result["matched_track_ids"] = sorted(
                    direct_master_results
                )
                vision_result["matched_count"] = len(direct_master_results)
                vision_result["unmatched_track_ids"] = (
                    [int(master_id)]
                    if master_id is not None and not direct_master_results
                    else []
                )
                vision_result["unmatched_detection_count"] = (
                    0 if direct_master_results else len(simple_measurements)
                )
                vision_result["unmatched_detections"] = (
                    [] if direct_master_results else simple_measurements[:5]
                )
                track_results = vision_result.get("track_results", {})
                if vision_frame_ts > last_logged_vision_frame_ts:
                    last_logged_vision_frame_ts = vision_frame_ts
                    # Record every raw gimbal-camera YOLO target before SORT
                    # association. This remains available even when ranging is
                    # warming, invalid, or cannot be written back to a track.
                    for detection_index, detection_item in enumerate(
                        simple_measurements
                    ):
                        detection_bbox = detection_item.get("bbox")
                        detection_center = detection_item.get("center")
                        if (
                            detection_center is None
                            and detection_bbox is not None
                            and len(detection_bbox) == 4
                        ):
                            detection_center = (
                                (float(detection_bbox[0]) + float(detection_bbox[2])) / 2.0,
                                (float(detection_bbox[1]) + float(detection_bbox[3])) / 2.0,
                            )
                        center_valid = (
                            detection_center is not None
                            and len(detection_center) == 2
                            and all(math.isfinite(float(v)) for v in detection_center)
                        )
                        if center_valid:
                            bbox_cx = float(detection_center[0])
                            bbox_cy = float(detection_center[1])
                            center_dx_px = bbox_cx - (
                                IMG_W / 2.0 + GIMBAL_LASER_AIM_OFFSET_X_PX
                            )
                            center_dy_px = bbox_cy - (
                                IMG_H / 2.0 + GIMBAL_LASER_AIM_OFFSET_Y_PX
                            )
                            center_dx_norm = center_dx_px / (IMG_W / 2.0)
                            center_dy_norm = center_dy_px / (IMG_H / 2.0)
                            center_offset_az = center_dx_px * FOV_X / IMG_W
                            center_offset_el = -center_dy_px * FOV_Y / IMG_H
                        else:
                            bbox_cx = bbox_cy = math.nan
                            center_dx_px = center_dy_px = math.nan
                            center_dx_norm = center_dy_norm = math.nan
                            center_offset_az = center_offset_el = math.nan
                        bbox_valid = (
                            detection_bbox is not None
                            and len(detection_bbox) == 4
                            and all(
                                math.isfinite(float(v))
                                for v in detection_bbox
                            )
                        )
                        if bbox_valid:
                            bbox_width_px = float(
                                detection_bbox[2] - detection_bbox[0]
                            )
                            bbox_height_px = float(
                                detection_bbox[3] - detection_bbox[1]
                            )
                        else:
                            bbox_width_px = bbox_height_px = math.nan
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "GIMBAL_VISION_DETECTION",
                            "track_id": "" if master_id is None else int(master_id),
                            "master_id": "" if master_id is None else int(master_id),
                            "meas_idx": int(detection_index),
                            "vision_frame_ts": f"{vision_frame_ts:.6f}",
                            "vision_age": f"{curr_time - vision_frame_ts:.6f}",
                            "simple_id": detection_item.get("simple_id", ""),
                            "class_id": detection_item.get("class_id", ""),
                            "confidence": f"{float(detection_item.get('confidence', math.nan)):.6f}",
                            "raw_bbox_x1": (
                                "" if detection_bbox is None
                                else f"{float(detection_bbox[0]):.3f}"
                            ),
                            "raw_bbox_y1": (
                                "" if detection_bbox is None
                                else f"{float(detection_bbox[1]):.3f}"
                            ),
                            "raw_bbox_x2": (
                                "" if detection_bbox is None
                                else f"{float(detection_bbox[2]):.3f}"
                            ),
                            "raw_bbox_y2": (
                                "" if detection_bbox is None
                                else f"{float(detection_bbox[3]):.3f}"
                            ),
                            "bbox_cx": "" if not center_valid else f"{bbox_cx:.3f}",
                            "bbox_cy": "" if not center_valid else f"{bbox_cy:.3f}",
                            "center_dx_px": "" if not center_valid else f"{center_dx_px:.3f}",
                            "center_dy_px": "" if not center_valid else f"{center_dy_px:.3f}",
                            "center_dx_norm": "" if not center_valid else f"{center_dx_norm:.6f}",
                            "center_dy_norm": "" if not center_valid else f"{center_dy_norm:.6f}",
                            "center_offset_az_deg": "" if not center_valid else f"{center_offset_az:.6f}",
                            "center_offset_el_deg": "" if not center_valid else f"{center_offset_el:.6f}",
                            "reason": (
                                f"state={detection_item.get('state', '')},"
                                f"warmup={detection_item.get('warmup_count', '')},"
                                f"distance_valid={1 if detection_item.get('distance_valid') else 0},"
                                f"safe={1 if detection_item.get('safe') else 0}"
                            ),
                        })
                        jitter_valid = bool(
                            detection_item.get("bbox_jitter_valid", False)
                        )
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "GIMBAL_VISION_BBOX_JITTER",
                            "track_id": (
                                "" if master_id is None else int(master_id)
                            ),
                            "master_id": (
                                "" if master_id is None else int(master_id)
                            ),
                            "meas_idx": int(detection_index),
                            "vision_frame_ts": f"{vision_frame_ts:.6f}",
                            "vision_age": f"{curr_time - vision_frame_ts:.6f}",
                            "simple_id": detection_item.get("simple_id", ""),
                            "class_id": detection_item.get("class_id", ""),
                            "confidence": f"{float(detection_item.get('confidence', math.nan)):.6f}",
                            "raw_bbox_x1": (
                                "" if not bbox_valid
                                else f"{float(detection_bbox[0]):.3f}"
                            ),
                            "raw_bbox_y1": (
                                "" if not bbox_valid
                                else f"{float(detection_bbox[1]):.3f}"
                            ),
                            "raw_bbox_x2": (
                                "" if not bbox_valid
                                else f"{float(detection_bbox[2]):.3f}"
                            ),
                            "raw_bbox_y2": (
                                "" if not bbox_valid
                                else f"{float(detection_bbox[3]):.3f}"
                            ),
                            "bbox_cx": (
                                "" if not center_valid else f"{bbox_cx:.3f}"
                            ),
                            "bbox_cy": (
                                "" if not center_valid else f"{bbox_cy:.3f}"
                            ),
                            "bbox_width_px": (
                                "" if not bbox_valid
                                else f"{bbox_width_px:.3f}"
                            ),
                            "bbox_height_px": (
                                "" if not bbox_valid
                                else f"{bbox_height_px:.3f}"
                            ),
                            "bbox_jitter_valid": 1 if jitter_valid else 0,
                            "bbox_jitter_dt_s": (
                                f"{float(detection_item.get('bbox_jitter_dt_s')):.6f}"
                                if jitter_valid else ""
                            ),
                            "bbox_jitter_dx_px": (
                                f"{float(detection_item.get('bbox_jitter_dx_px')):.3f}"
                                if jitter_valid else ""
                            ),
                            "bbox_jitter_dy_px": (
                                f"{float(detection_item.get('bbox_jitter_dy_px')):.3f}"
                                if jitter_valid else ""
                            ),
                            "bbox_jitter_center_px": (
                                f"{float(detection_item.get('bbox_jitter_center_px')):.3f}"
                                if jitter_valid else ""
                            ),
                            "bbox_jitter_center_norm": (
                                f"{float(detection_item.get('bbox_jitter_center_norm')):.6f}"
                                if jitter_valid else ""
                            ),
                            "bbox_jitter_width_delta_px": (
                                f"{float(detection_item.get('bbox_jitter_width_delta_px')):.3f}"
                                if jitter_valid else ""
                            ),
                            "bbox_jitter_height_delta_px": (
                                f"{float(detection_item.get('bbox_jitter_height_delta_px')):.3f}"
                                if jitter_valid else ""
                            ),
                            "bbox_jitter_iou": (
                                f"{float(detection_item.get('bbox_jitter_iou')):.6f}"
                                if jitter_valid else ""
                            ),
                            "bbox_jitter_previous_missing_frames": int(
                                detection_item.get(
                                    "bbox_jitter_previous_missing_frames", 0
                                ) or 0
                            ),
                            "reason": (
                                "consecutive_same_simple_id"
                                if jitter_valid
                                else "no_consecutive_bbox_history"
                            ),
                        })
                    unmatched_detections = vision_result.get(
                        "unmatched_detections", []
                    ) or []
                    top_unmatched = (
                        unmatched_detections[0]
                        if unmatched_detections
                        else {}
                    )
                    top_unmatched_bbox = top_unmatched.get("bbox")
                    per_track_reasons = []
                    for result_track_id, result_item in sorted(
                        track_results.items(),
                        key=lambda item: int(item[0]),
                    ):
                        per_track_reasons.append(
                            f"{int(result_track_id)}:"
                            f"{result_item.get('state')}/"
                            f"{result_item.get('reason', '')}"
                        )
                    field_log_event({
                        "timestamp": f"{curr_time:.6f}",
                        "seq": sender_seq,
                        "mode": sender_mode,
                        "event": "GIMBAL_VISION_ASSOC",
                        "track_id": "" if master_id is None else int(master_id),
                        "master_id": "" if master_id is None else int(master_id),
                        "detection_count": int(
                            vision_result.get("detection_count", 0) or 0
                        ),
                        "matched_count": int(
                            vision_result.get("matched_count", 0) or 0
                        ),
                        "unmatched_detection_count": int(
                            vision_result.get(
                                "unmatched_detection_count", 0
                            ) or 0
                        ),
                        "visible_track_count": int(
                            vision_result.get("visible_track_count", 0) or 0
                        ),
                        "active_track_count": int(
                            vision_result.get("active_track_count", 0) or 0
                        ),
                        "roi_count": int(vision_result.get("roi_count", 0) or 0),
                        "sharp_roi_count": int(
                            vision_result.get("sharp_roi_count", 0) or 0
                        ),
                        "matched_track_ids": ";".join(
                            str(x)
                            for x in vision_result.get(
                                "matched_track_ids", []
                            )
                        ),
                        "ambiguous_track_ids": ";".join(
                            str(x)
                            for x in vision_result.get(
                                "ambiguous_track_ids", []
                            )
                        ),
                        "unmatched_track_ids": ";".join(
                            str(x)
                            for x in vision_result.get(
                                "unmatched_track_ids", []
                            )
                        ),
                        "raw_bbox_x1": (
                            "" if top_unmatched_bbox is None
                            else f"{float(top_unmatched_bbox[0]):.3f}"
                        ),
                        "raw_bbox_y1": (
                            "" if top_unmatched_bbox is None
                            else f"{float(top_unmatched_bbox[1]):.3f}"
                        ),
                        "raw_bbox_x2": (
                            "" if top_unmatched_bbox is None
                            else f"{float(top_unmatched_bbox[2]):.3f}"
                        ),
                        "raw_bbox_y2": (
                            "" if top_unmatched_bbox is None
                            else f"{float(top_unmatched_bbox[3]):.3f}"
                        ),
                        "reason": (
                            f"state={vision_result.get('state')},"
                            f"top_unmatched_conf="
                            f"{float(top_unmatched.get('confidence', math.nan)):.3f},"
                            f"track_reasons={'|'.join(per_track_reasons)}"
                        ),
                    })
                    for detection_index, detection_item in enumerate(unmatched_detections[:5]):
                        detection_bbox = detection_item.get("bbox")
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "GIMBAL_VISION_UNMATCHED_DETECTION",
                            "track_id": "" if master_id is None else int(master_id),
                            "master_id": "" if master_id is None else int(master_id),
                            "meas_idx": int(detection_index),
                            "raw_bbox_x1": (
                                "" if detection_bbox is None
                                else f"{float(detection_bbox[0]):.3f}"
                            ),
                            "raw_bbox_y1": (
                                "" if detection_bbox is None
                                else f"{float(detection_bbox[1]):.3f}"
                            ),
                            "raw_bbox_x2": (
                                "" if detection_bbox is None
                                else f"{float(detection_bbox[2]):.3f}"
                            ),
                            "raw_bbox_y2": (
                                "" if detection_bbox is None
                                else f"{float(detection_bbox[3]):.3f}"
                            ),
                            "reason": (
                                "ranging_buffer_not_associated_to_sort,"
                                "distance_accumulation_not_blocked,"
                                f"confidence={float(detection_item.get('confidence', math.nan)):.3f},"
                                f"class_id={int(detection_item.get('class_id', -1))}"
                            ),
                        })
                distance_track_results = (
                    {}
                    if getattr(vision_service, "detection_only", False)
                    else track_results
                )
                for track_id, track_result in distance_track_results.items():
                    track_id = int(track_id)
                    track = track_by_id.get(track_id)
                    result_ts = float(
                        track_result.get("frame_ts", 0.0)
                    )
                    vision_age = curr_time - result_ts
                    distance_valid = bool(track_result.get("distance_valid"))
                    result_is_fresh = 0.0 <= vision_age <= GIMBAL_VISION_RESULT_TTL
                    result_is_new = result_ts > last_applied_vision_ts.get(track_id, 0.0)
                    skip_reasons = []
                    if track is None:
                        skip_reasons.append("track_not_in_valid_tracks")
                    if not distance_valid:
                        skip_reasons.append("distance_invalid")
                    if not result_is_fresh:
                        skip_reasons.append("vision_result_stale")
                    if not result_is_new:
                        skip_reasons.append("already_applied_or_old")
                    if skip_reasons:
                        diag_bbox = track_result.get("bbox")
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "GIMBAL_VISION_DISTANCE_DIAG",
                            "track_id": track_id,
                            "master_id": (
                                "" if master_id is None else int(master_id)
                            ),
                            "is_master": 1 if track_id == master_id else 0,
                            "distance": (
                                ""
                                if not math.isfinite(float(track_result.get("distance", math.nan)))
                                else f"{float(track_result.get('distance')):.6f}"
                            ),
                            "distance_source": track_result.get("distance_source", "none"),
                            "cost": (
                                f"{float(track_result.get('association_error_px')):.6f}"
                                if math.isfinite(float(track_result.get(
                                    "association_error_px", math.nan
                                )))
                                else ""
                            ),
                            "raw_bbox_x1": (
                                "" if diag_bbox is None
                                else f"{float(diag_bbox[0]):.3f}"
                            ),
                            "raw_bbox_y1": (
                                "" if diag_bbox is None
                                else f"{float(diag_bbox[1]):.3f}"
                            ),
                            "raw_bbox_x2": (
                                "" if diag_bbox is None
                                else f"{float(diag_bbox[2]):.3f}"
                            ),
                            "raw_bbox_y2": (
                                "" if diag_bbox is None
                                else f"{float(diag_bbox[3]):.3f}"
                            ),
                            "reason": (
                                f"not_written_to_track:{'|'.join(skip_reasons)},"
                                f"state={track_result.get('state')},"
                                f"model_distance_valid={1 if distance_valid else 0},"
                                f"safe={1 if track_result.get('safe') else 0},"
                                f"warmup={track_result.get('warmup_count')},"
                                f"age={vision_age:.3f},"
                                f"confidence={float(track_result.get('confidence', math.nan)):.3f},"
                                f"model_reason={track_result.get('reason', '')}"
                            ),
                        })
                        continue
                    vision_distance = _parse_positive_float(
                        track_result.get("distance")
                    )
                    if vision_distance is not None:
                        track.set_mono_distance(
                            vision_distance,
                            result_ts,
                        )
                        track.dist_source = track_result.get(
                            "distance_source", "gimbal_yolo_gru"
                        )
                        last_applied_vision_ts[track_id] = result_ts
                        vision_bbox = track_result.get("bbox")
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "GIMBAL_VISION_DISTANCE",
                            "track_id": track_id,
                            "cost": (
                                f"{float(track_result.get('association_error_px')):.6f}"
                                if math.isfinite(float(track_result.get(
                                    "association_error_px", math.nan
                                )))
                                else ""
                            ),
                            "master_id": (
                                "" if master_id is None else int(master_id)
                            ),
                            "is_master": 1 if track_id == master_id else 0,
                            "distance": f"{vision_distance:.6f}",
                            "distance_source": track.dist_source,
                            "raw_bbox_x1": (
                                "" if vision_bbox is None
                                else f"{float(vision_bbox[0]):.3f}"
                            ),
                            "raw_bbox_y1": (
                                "" if vision_bbox is None
                                else f"{float(vision_bbox[1]):.3f}"
                            ),
                            "raw_bbox_x2": (
                                "" if vision_bbox is None
                                else f"{float(vision_bbox[2]):.3f}"
                            ),
                            "raw_bbox_y2": (
                                "" if vision_bbox is None
                                else f"{float(vision_bbox[3]):.3f}"
                            ),
                            "reason": (
                                f"state={track_result.get('state')},"
                                f"warmup={track_result.get('warmup_count')},"
                                f"confidence={float(track_result.get('confidence', math.nan)):.3f}"
                            ),
                        })
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "GIMBAL_VISION_DISTANCE_DIAG",
                            "track_id": track_id,
                            "master_id": (
                                "" if master_id is None else int(master_id)
                            ),
                            "is_master": 1 if track_id == master_id else 0,
                            "distance": f"{vision_distance:.6f}",
                            "distance_source": track.dist_source,
                            "cost": (
                                f"{float(track_result.get('association_error_px')):.6f}"
                                if math.isfinite(float(track_result.get(
                                    "association_error_px", math.nan
                                )))
                                else ""
                            ),
                            "raw_bbox_x1": (
                                "" if vision_bbox is None
                                else f"{float(vision_bbox[0]):.3f}"
                            ),
                            "raw_bbox_y1": (
                                "" if vision_bbox is None
                                else f"{float(vision_bbox[1]):.3f}"
                            ),
                            "raw_bbox_x2": (
                                "" if vision_bbox is None
                                else f"{float(vision_bbox[2]):.3f}"
                            ),
                            "raw_bbox_y2": (
                                "" if vision_bbox is None
                                else f"{float(vision_bbox[3]):.3f}"
                            ),
                            "reason": (
                                "written_to_track,"
                                f"state={track_result.get('state')},"
                                "model_distance_valid=1,"
                                f"safe={1 if track_result.get('safe') else 0},"
                                f"warmup={track_result.get('warmup_count')},"
                                f"age={vision_age:.3f},"
                                f"confidence={float(track_result.get('confidence', math.nan)):.3f}"
                            ),
                        })

                active_track_ids = set(track_by_id)
                for stale_track_id in list(last_applied_vision_ts):
                    if stale_track_id not in active_track_ids:
                        del last_applied_vision_ts[stale_track_id]

            if visual_alignment is not None:
                alignment_decision = visual_alignment.update(
                    frame_ts=float(
                        vision_result.get("frame_ts", 0.0) or 0.0
                    ),
                    master_track_id=master_id,
                    measurement=aligned_measurement,
                    # Other YOLO boxes do not break the retained simple-ID
                    # lock; alignment sees only the selected target.
                    detection_count=1 if aligned_measurement is not None else 0,
                    gimbal_stationary=gimbal_is_stationary,
                    stationary_ts=gimbal_stationary_ts,
                    decision_ts=curr_time,
                )

                # If a centered 10 Hz session produced no valid return, scan
                # four 0.1-degree points around the latest bbox center. Each
                # step is recomputed from the newest bbox, so bbox motion still
                # remains the primary gimbal-control input.
                if laser_scan_track_id != master_id:
                    laser_scan_track_id = master_id
                    laser_scan_index = 0
                    last_scanned_no_return_ts = 0.0
                no_return_event_ts = float(
                    laser_no_return_ts_snapshot or 0.0
                )
                scan_triggered = (
                    master_id is not None
                    and int(laser_request_track_id_snapshot) == int(master_id)
                    and float(laser_request_ts_snapshot or 0.0) > 0.0
                    and no_return_event_ts > last_scanned_no_return_ts + 1e-6
                )
                if scan_triggered:
                    scan_offset_az, scan_offset_el = (
                        SFL0603_NO_RETURN_SCAN_OFFSETS_DEG[laser_scan_index]
                    )
                    scan_decision = (
                        visual_alignment.build_no_return_scan_decision(
                            alignment_decision,
                            scan_offset_az_deg=scan_offset_az,
                            scan_offset_el_deg=scan_offset_el,
                        )
                    )
                    if scan_decision is not None:
                        alignment_decision = scan_decision
                        alignment_decision["scan_index"] = laser_scan_index
                        last_scanned_no_return_ts = no_return_event_ts
                        laser_scan_index = (
                            laser_scan_index + 1
                        ) % len(SFL0603_NO_RETURN_SCAN_OFFSETS_DEG)
                if alignment_decision.get("processed_new_frame"):
                    field_log_event({
                        "timestamp": f"{curr_time:.6f}",
                        "seq": sender_seq,
                        "mode": sender_mode,
                        "event": "GIMBAL_YOLO_ALIGNMENT",
                        "track_id": (
                            "" if master_id is None else int(master_id)
                        ),
                        "master_id": (
                            "" if master_id is None else int(master_id)
                        ),
                        "vision_frame_ts": f"{float(vision_result.get('frame_ts', 0.0) or 0.0):.6f}",
                        "detection_count": int(
                            vision_result.get("detection_count", 0) or 0
                        ),
                        "simple_id": (
                            "" if not aligned_measurement
                            else aligned_measurement.get("simple_id", "")
                        ),
                        "bbox_cx": (
                            "" if not math.isfinite(float(alignment_decision.get("control_x", math.nan)))
                            else f"{float(alignment_decision['control_x']):.3f}"
                        ),
                        "bbox_cy": (
                            "" if not math.isfinite(float(alignment_decision.get("control_y", math.nan)))
                            else f"{float(alignment_decision['control_y']):.3f}"
                        ),
                        "center_dx_px": (
                            "" if not math.isfinite(float(alignment_decision.get("dx_px", math.nan)))
                            else f"{float(alignment_decision['dx_px']):.3f}"
                        ),
                        "center_dy_px": (
                            "" if not math.isfinite(float(alignment_decision.get("dy_px", math.nan)))
                            else f"{float(alignment_decision['dy_px']):.3f}"
                        ),
                        "alignment_state": alignment_decision.get(
                            "state", ""
                        ),
                        "alignment_sample_count": int(
                            alignment_decision.get("sample_count", 0) or 0
                        ),
                        "alignment_std_x_px": (
                            "" if not math.isfinite(float(alignment_decision.get("std_x_px", math.nan)))
                            else f"{float(alignment_decision['std_x_px']):.3f}"
                        ),
                        "alignment_std_y_px": (
                            "" if not math.isfinite(float(alignment_decision.get("std_y_px", math.nan)))
                            else f"{float(alignment_decision['std_y_px']):.3f}"
                        ),
                        "alignment_speed_x_px_s": (
                            "" if not math.isfinite(float(alignment_decision.get("speed_x_px_s", math.nan)))
                            else f"{float(alignment_decision['speed_x_px_s']):.3f}"
                        ),
                        "alignment_speed_y_px_s": (
                            "" if not math.isfinite(float(alignment_decision.get("speed_y_px_s", math.nan)))
                            else f"{float(alignment_decision['speed_y_px_s']):.3f}"
                        ),
                        "alignment_latest_to_median_px": (
                            "" if not math.isfinite(float(alignment_decision.get("latest_to_median_px", math.nan)))
                            else f"{float(alignment_decision['latest_to_median_px']):.3f}"
                        ),
                        "alignment_frame_age_s": (
                            "" if not math.isfinite(float(alignment_decision.get("frame_age_s", math.nan)))
                            else f"{float(alignment_decision['frame_age_s']):.6f}"
                        ),
                        "alignment_fresh_after_dropout": int(
                            alignment_decision.get("fresh_after_dropout", 0) or 0
                        ),
                        "alignment_control_mode": alignment_decision.get(
                            "control_mode", "none"
                        ),
                        "alignment_outlier_center_jump_px": (
                            "" if not math.isfinite(float(alignment_decision.get("outlier_center_jump_px", math.nan)))
                            else f"{float(alignment_decision['outlier_center_jump_px']):.3f}"
                        ),
                        "alignment_outlier_bbox_iou": (
                            "" if not math.isfinite(float(alignment_decision.get("outlier_bbox_iou", math.nan)))
                            else f"{float(alignment_decision['outlier_bbox_iou']):.6f}"
                        ),
                        "alignment_delta_az_deg": f"{float(alignment_decision.get('delta_az_deg', 0.0)):.6f}",
                        "alignment_delta_el_deg": f"{float(alignment_decision.get('delta_el_deg', 0.0)):.6f}",
                        "reason": (
                            f"state={alignment_decision.get('state')},"
                            f"stable={1 if alignment_decision.get('stable') else 0},"
                            f"command={1 if alignment_decision.get('command_requested') else 0},"
                            f"mode={alignment_decision.get('control_mode', 'none')}"
                        ),
                    })

            # The SFL0603 request records the authoritative target track ID.
            # Once a valid serial result arrives, bind it directly to that
            # still-existing track; later master/alignment changes must not
            # discard a measurement that was already captured for the target.
            laser_distance = _parse_positive_float(raw_laser_dist)
            laser_age = curr_time - float(raw_laser_ts or 0.0)
            # Once a target track exists, keep SFL0603 continuously ranging at
            # 10 Hz. Alignment only decides normal bbox correction and whether
            # a no-return scan may move around the latest bbox center.
            laser_request_ready = (
                laser is not None
                and master_id is not None
                and visual_target_lock is not None
                and visual_target_lock.holds_sort_track(master_id)
            )
            with shared_state.lock:
                if laser_request_ready:
                    if laser_continuous_track_id != int(master_id):
                        laser_continuous_track_id = int(master_id)
                        laser_continuous_request_ts = curr_time
                    shared_state.laser_request_ts = laser_continuous_request_ts
                    shared_state.laser_request_track_id = int(master_id)
                    shared_state.laser_request_heartbeat_ts = curr_time
                else:
                    laser_continuous_track_id = None
                    laser_continuous_request_ts = 0.0
                    shared_state.laser_request_ts = 0.0
                    shared_state.laser_request_track_id = -1
                    shared_state.laser_request_heartbeat_ts = 0.0
            laser_target_track = track_by_id.get(int(raw_laser_track_id))
            laser_bind_ready = (
                laser_target_track is not None
                and laser_distance is not None
                and 0.0 <= laser_age <= LASER_RESULT_TTL
                and float(raw_laser_request_ts) > 0.0
                and float(raw_laser_ts) >= float(raw_laser_request_ts)
                and float(raw_laser_ts) > last_bound_laser_ts
            )
            if laser_bind_ready:
                if laser_target_track.dist_source != "sfl0603_laser":
                    # Do not mix a historical monocular/model estimate into
                    # the first authoritative laser update for this track.
                    laser_target_track.dist_state = None
                laser_target_track.set_mono_distance(
                    laser_distance,
                    float(raw_laser_ts),
                )
                laser_target_track.dist_source = "sfl0603_laser"
                last_bound_laser_ts = float(raw_laser_ts)
                if int(laser_target_track.id) == int(raw_laser_track_id):
                    laser_scan_track_id = int(laser_target_track.id)
                    laser_scan_index = 0
                    last_scanned_no_return_ts = 0.0
                field_log_event({
                    "timestamp": f"{curr_time:.6f}",
                    "seq": sender_seq,
                    "mode": sender_mode,
                    "event": "LASER_BOUND_TRACK",
                    "track_id": int(laser_target_track.id),
                    "master_id": "" if master_id is None else int(master_id),
                    "is_master": 1 if laser_target_track.id == master_id else 0,
                    "distance": f"{laser_distance:.6f}",
                    "distance_source": "sfl0603_laser",
                    "vision_frame_ts": f"{float(raw_laser_request_ts):.6f}",
                    "reason": (
                        "direct_sfl0603_result_to_requested_track,"
                        f"laser_ts={float(raw_laser_ts):.6f},"
                        f"laser_age={laser_age:.3f}"
                    ),
                })

            # Publish the laser-backed track result in the same loop so UI and
            # strike consumers can use it immediately.
            if (
                laser_target_track is not None
                and laser_target_track.dist_source == "sfl0603_laser"
                and 0.0
                <= curr_time - float(laser_target_track.last_dist_ts)
                <= TRACK_DISTANCE_TTL
            ):
                laser_track_result = dict(
                    vision_result.get("track_results", {}).get(
                        int(laser_target_track.id), {}
                    )
                )
                laser_track_result.update({
                    "track_id": int(laser_target_track.id),
                    "distance": float(laser_target_track.dist_state[0, 0]),
                    "distance_source": "sfl0603_laser",
                    "distance_valid": True,
                    "frame_ts": float(laser_target_track.last_dist_ts),
                    "safe": True,
                    "reason": "SFL0603_DIRECT_TRACK_BOUND",
                })
                vision_result.setdefault("track_results", {})[
                    int(laser_target_track.id)
                ] = laser_track_result

            track_results_for_strike = (
                vision_result.get("track_results", {})
                if isinstance(vision_result, dict)
                else {}
            )
            current_strike_track = next(
                (t for t in strike_valid_tracks if t.id == strike_target_id),
                None,
            )
            proposed_strike_track, ranked_strike_candidates = choose_strike_target(
                strike_valid_tracks,
                curr_time,
                track_results_for_strike,
                strike_target_id=strike_target_id,
            )
            strike_selection_reason = None
            if current_strike_track is None and proposed_strike_track is not None:
                strike_selection_reason = (
                    "strike_lost" if strike_target_id is not None else "initial_strike_acquire"
                )
                current_strike_track = proposed_strike_track
            elif (
                current_strike_track is not None
                and proposed_strike_track is not None
                and proposed_strike_track.id != current_strike_track.id
            ):
                strike_eval_by_id = {
                    item["track_id"]: item for item in ranked_strike_candidates
                }
                current_strike_eval = strike_eval_by_id.get(
                    int(current_strike_track.id)
                )
                proposed_strike_eval = strike_eval_by_id.get(
                    int(proposed_strike_track.id)
                )
                strike_score_margin = (
                    proposed_strike_eval["threat_score"]
                    - current_strike_eval["threat_score"]
                    if current_strike_eval is not None
                    and proposed_strike_eval is not None
                    else math.inf
                )
                if strike_score_margin >= STRIKE_TARGET_SWITCH_SCORE_MARGIN:
                    if strike_challenger_id != proposed_strike_track.id:
                        strike_challenger_id = proposed_strike_track.id
                        strike_challenger_since = curr_time
                    elif (
                        curr_time - strike_challenger_since
                    ) >= STRIKE_TARGET_SWITCH_CONFIRM_SECONDS:
                        strike_selection_reason = "confirmed_higher_strike_threat"
                        current_strike_track = proposed_strike_track
                else:
                    strike_challenger_id = None
                    strike_challenger_since = 0.0
            elif proposed_strike_track is None:
                current_strike_track = None
                strike_challenger_id = None
                strike_challenger_since = 0.0
            else:
                strike_challenger_id = None
                strike_challenger_since = 0.0

            if current_strike_track is None:
                if strike_target_id is not None:
                    field_log_event({
                        "timestamp": f"{curr_time:.6f}",
                        "seq": sender_seq,
                        "mode": sender_mode,
                        "event": "STRIKE_TARGET_LOST",
                        "track_id": int(strike_target_id),
                        "master_id": "" if master_id is None else int(master_id),
                        "reason": "no_fresh_safe_distance_candidate",
                    })
                strike_target_id = None
                clear_strike_window(strike_window)
            elif strike_target_id != current_strike_track.id:
                old_strike_target_id = strike_target_id
                strike_target_id = current_strike_track.id
                clear_strike_window(strike_window)
                strike_challenger_id = None
                strike_challenger_since = 0.0
                candidates_text = format_strike_candidates(ranked_strike_candidates)
                if PRINT_EVENT_LOGS:
                    print(
                        f"[StrikeTarget] reason={strike_selection_reason}, "
                        f"from={old_strike_target_id}, to={strike_target_id}, "
                        f"candidates={candidates_text}"
                    )
                field_log_event({
                    "timestamp": f"{curr_time:.6f}",
                    "seq": sender_seq,
                    "mode": sender_mode,
                    "event": "STRIKE_TARGET_SWITCH",
                    "track_id": int(strike_target_id),
                    "master_id": "" if master_id is None else int(master_id),
                    "reason": strike_selection_reason,
                })

            # --- 5. 状态机：物理执行与测距 (LOCKED) ---
            if master_id is not None:
                if master_track is not None:
                    fut_az, fut_el = master_track.get_future_position(
                        dt_delay=PREDICT_DELAY
                    )
                    ctrl_az, ctrl_el = ui_to_ctrl_angles(fut_az, fut_el)
                else:
                    # SORT may expire during an upstream UDP outage. The
                    # retained gimbal-camera bbox still supplies relative
                    # corrections around the current encoder attitude.
                    ctrl_az, ctrl_el = ui_to_ctrl_angles(
                        shared_gimbal_az,
                        shared_gimbal_el,
                    )

                master_vision_result = (
                    vision_result.get("track_results", {}).get(
                        int(master_id)
                    )
                    if master_id is not None
                    else None
                )
                vision_frame_age = curr_time - float(
                    (
                        master_vision_result.get("frame_ts", 0.0)
                        if master_vision_result
                        else vision_result.get("frame_ts", 0.0)
                    )
                    or 0.0
                )
                vision_bbox_fresh = (
                    master_vision_result is not None
                    and master_vision_result.get("bbox") is not None
                    and 0.0 <= vision_frame_age <= GIMBAL_VISION_RESULT_TTL
                )
                reposition_reason = "none"
                visual_alignment_ready = bool(
                    alignment_decision.get("command_requested", False)
                )
                if visual_alignment_ready:
                    alignment_target_ui_az = (
                        shared_gimbal_az
                        + float(alignment_decision["delta_az_deg"])
                    ) % 360.0
                    alignment_target_ui_el = (
                        shared_gimbal_el
                        + float(alignment_decision["delta_el_deg"])
                    )
                    ctrl_az, ctrl_el = ui_to_ctrl_angles(
                        alignment_target_ui_az,
                        alignment_target_ui_el,
                    )
                    need_reposition = True
                    angle_unsafe_frames = 0
                    reposition_reason = (
                        "laser_no_return_bbox_center_scan"
                        if alignment_decision.get("state")
                        == "LASER_NO_RETURN_SCAN_READY"
                        else "direct_single_yolo_bbox_alignment"
                    )
                elif vision_bbox_fresh:
                    need_reposition = bool(
                        master_vision_result.get(
                            "reposition_requested", False
                        )
                    )
                    angle_unsafe_frames = 0
                    if need_reposition:
                        reposition_reason = "vision_bbox_outside_safe_zone"
                elif (
                    visual_target_lock is not None
                    and visual_target_lock.holds_visual_control(master_id)
                ):
                    # A short YOLO dropout must not hand control straight back
                    # to SORT.  The fixed-camera angle can have a systematic
                    # offset from the centered gimbal-camera angle, which would
                    # otherwise pull the camera away and create a ping-pong
                    # loop.  Hold the current posture until the visual lock's
                    # continuous-missing timeout explicitly releases control.
                    need_reposition = False
                    angle_unsafe_frames = 0
                    reposition_reason = "visual_lock_missing_hold_sort_suppressed"
                elif master_track is not None:
                    delta_az = abs(
                        angular_diff(
                            master_track.state[0, 0],
                            shared_gimbal_az,
                        )
                    )
                    delta_el = abs(
                        master_track.state[1, 0] - shared_gimbal_el
                    )
                    safe_half_az = FOV_X * GIMBAL_SAFE_FOV_RATIO_X / 2.0
                    safe_half_el = FOV_Y * GIMBAL_SAFE_FOV_RATIO_Y / 2.0
                    angle_safe = (
                        delta_az <= safe_half_az
                        and delta_el <= safe_half_el
                    )
                    angle_unsafe_frames = (
                        0 if angle_safe else angle_unsafe_frames + 1
                    )
                    need_reposition = angle_unsafe_frames >= 3
                    if need_reposition:
                        reposition_reason = "global_track_outside_safe_fov"
                else:
                    need_reposition = False
                    angle_unsafe_frames = 0

                # The GT06Z command payload has 0.1-degree resolution.  Use
                # the same quantized values for deadband checks, logs and the
                # actual queue so software never waits for an unsendable angle.
                ctrl_az = quantize_gimbal_command_angle(ctrl_az)
                ctrl_el = quantize_gimbal_command_angle(ctrl_el)

                # The target may move freely inside the central safe FOV. Only
                # reposition after three unsafe frames, and never continuously
                # preempt while the camera is physically moving. A target
                # switch may still replace the previous target command.
                can_issue_command = (
                    gimbal_is_stationary
                    or active_gimbal_track_id != master_id
                )
                need_send = need_reposition and can_issue_command
                if (
                    not visual_alignment_ready
                    and
                    active_gimbal_track_id == master_id
                    and last_sent_ctrl_az is not None
                    and last_sent_ctrl_el is not None
                ):
                    d_az = abs(angular_diff(ctrl_az, last_sent_ctrl_az))
                    d_el = abs(ctrl_el - last_sent_ctrl_el)
                    if d_az < GIMBAL_CMD_DEADBAND_AZ and d_el < GIMBAL_CMD_DEADBAND_EL:
                        need_send = False

                if need_send:
                    global_cmd_id += 1
                    if vision_service is not None:
                        vision_service.update_context(
                            master_track_id=master_id,
                            gimbal_settled=False,
                            settled_ts=settled_ts,
                        )
                    push_latest_gimbal_cmd({
                        "cmd_id": global_cmd_id,
                        "track_id": int(master_id) if master_id is not None else -1,
                        "az": ctrl_az,
                        "el": ctrl_el,
                        "ts": curr_time,
                        "force": visual_alignment_ready,
                    })
                    if visual_alignment_ready and visual_alignment is not None:
                        visual_alignment.mark_command_sent(
                            gimbal_stationary_ts
                        )
                    last_sent_ctrl_az = ctrl_az
                    last_sent_ctrl_el = ctrl_el
                    field_log_gimbal({
                        "timestamp": f"{curr_time:.6f}",
                        "event": "GIMBAL_CMD",
                        "cmd_id": int(global_cmd_id),
                        "track_id": "" if master_id is None else int(master_id),
                        "cmd_az": f"{ctrl_az:.1f}",
                        "cmd_el": f"{ctrl_el:.1f}",
                        "gimbal_ui_az": f"{shared_gimbal_az:.6f}",
                        "gimbal_ui_el": f"{shared_gimbal_el:.6f}",
                        "target_ctrl_az": f"{ctrl_az:.1f}",
                        "target_ctrl_el": f"{ctrl_el:.1f}",
                        "reason": reposition_reason,
                    })
                    angle_unsafe_frames = 0

                vision_distance_age = curr_time - float(
                    vision_result.get("frame_ts", 0.0)
                )
                strike_track = next(
                    (t for t in valid_tracks if t.id == strike_target_id),
                    None,
                )
                strike_track_result = (
                    track_results_for_strike.get(int(strike_target_id), {})
                    if strike_target_id is not None
                    else {}
                )
                strike_result_age = curr_time - float(
                    strike_track_result.get("frame_ts", 0.0) or 0.0
                )
                strike_visual_ready = (
                    ENABLE_STRIKE_SEND
                    and gimbal_is_settled
                    and strike_track is not None
                    and strike_track_result.get("distance_valid")
                    and strike_track_result.get("safe")
                    and 0.0 <= strike_result_age <= GIMBAL_VISION_RESULT_TTL
                )
                if strike_visual_ready:
                    strike_dist, strike_dist_source = select_strike_distance(
                        strike_track,
                        curr_time,
                        strike_window,
                    )
                    if strike_dist is not None:
                        window_was_open = (
                            strike_window["track_id"] == strike_target_id
                            and curr_time < strike_window["valid_until"]
                        )
                        strike_window["track_id"] = strike_target_id
                        strike_window["distance"] = strike_dist
                        strike_window["source"] = strike_dist_source
                        strike_window["valid_until"] = curr_time + STRIKE_WINDOW_SECONDS
                        if PRINT_EVENT_LOGS and not window_was_open:
                            print(
                                f"[Strike] visual window open "
                                f"track_id={strike_target_id}, source={strike_dist_source}, "
                                f"dist={strike_dist:.2f}m, valid={STRIKE_WINDOW_SECONDS:.2f}s"
                            )
                        if not window_was_open:
                            field_log_event({
                                "timestamp": f"{curr_time:.6f}",
                                "seq": sender_seq,
                                "mode": sender_mode,
                                "event": "STRIKE_WINDOW_OPEN",
                                "track_id": int(strike_target_id),
                                "master_id": "" if master_id is None else int(master_id),
                                "is_master": 1 if strike_target_id == master_id else 0,
                                "distance": f"{strike_dist:.6f}",
                                "distance_source": strike_dist_source,
                                "reason": "fresh_highest_threat_gimbal_yolo_distance",
                            })

                strike_window_valid = (
                    strike_visual_ready
                    and strike_window["track_id"] == strike_target_id
                    and strike_track is not None
                    and strike_track.lost_seconds(curr_time)
                    <= MAX_LOCK_LOST_SECONDS
                    and curr_time < strike_window["valid_until"]
                )
                if strike_window_valid and strike_sender is not None:
                    strike_interval = 1.0 / max(STRIKE_SEND_HZ, 0.1)
                    if (curr_time - strike_window["last_send_ts"]) >= strike_interval:
                        try:
                            strike_ui_id = get_or_assign_ui_id(strike_track)
                            
                            # 1. 角度预测：主打击模型采用 6D CA (常加速度外推)，同时生成 CV (常速度) 预测用于影子比对与日志记录
                            strike_rel_az_ca, strike_el_ca = strike_track.get_shadow_future_position_ca(STRIKE_LEAD_TIME)
                            strike_rel_az_cv, strike_el_cv = strike_track.get_future_position(STRIKE_LEAD_TIME)
                            strike_map_az_ca = relative_to_map_azimuth(strike_rel_az_ca)
                            strike_map_az_cv = relative_to_map_azimuth(strike_rel_az_cv)
                            
                            strike_rel_az = strike_rel_az_ca
                            strike_el = strike_el_ca
                            strike_map_az = strike_map_az_ca
                            
                            # 2. 距离处理：使用 Distance-KF 滤波器平滑去噪，但不作远期速度预测外推，若超过 3s 未更新则安全退回静态基准
                            # Distance uses only the current track's fresh smoothed KF value.
                            strike_smooth_dist, dist_source = get_smoothed_track_distance(strike_track, curr_time)
                            if strike_smooth_dist is None:
                                raise ValueError(f"no fresh smoothed track distance, source={dist_source}")
                            
                            # 3. 网络包构建与下发
                            strike_packet = strike_sender.send_target(
                                target_id=strike_ui_id,
                                distance_m=strike_smooth_dist,
                                azimuth_deg=strike_map_az,
                                elevation_deg=strike_el,
                            )
                            strike_window["last_send_ts"] = curr_time
                            
                            # 4. 详细结构化日志记录，包含 CV 与 CA 双预测指标、距离不确定度及径向估计速度
                            field_log_event({
                                "timestamp": f"{curr_time:.6f}",
                                "seq": sender_seq,
                                "mode": sender_mode,
                                "event": "STRIKE_SEND",
                                "track_id": int(strike_track.id),
                                "pred_az": f"{strike_rel_az:.6f}",
                                "map_az": f"{strike_map_az:.6f}",
                                "pred_el": f"{strike_el:.6f}",
                                "pred_az_cv": f"{strike_rel_az_cv:.6f}",
                                "pred_el_cv": f"{strike_el_cv:.6f}",
                                "map_az_cv": f"{strike_map_az_cv:.6f}",
                                "pred_az_ca": f"{strike_rel_az_ca:.6f}",
                                "pred_el_ca": f"{strike_el_ca:.6f}",
                                "map_az_ca": f"{strike_map_az_ca:.6f}",
                                "master_id": "" if master_id is None else int(master_id),
                                "is_master": 1 if strike_track.id == master_id else 0,
                                "internal_track_id": int(strike_track.id),
                                "ui_id": int(strike_ui_id),
                                "distance": f"{strike_smooth_dist:.6f}",
                                "distance_source": dist_source,
                                "dist_uncertainty": f"{strike_track.dist_uncertainty:.6f}",
                                "radial_velocity": f"{float(strike_track.dist_state[1, 0]):.6f}" if strike_track.dist_state is not None else "0.000000",
                                "reason": f"settled_cmd_id={settled_cmd_id},packet={strike_packet.hex(' ')}",
                            })
                        except Exception as e:
                            if PRINT_EVENT_LOGS:
                                print(f"[Strike][Warn] send skipped: {e}")
                            field_log_event({
                                "timestamp": f"{curr_time:.6f}",
                                "seq": sender_seq,
                                "mode": sender_mode,
                                "event": "STRIKE_SEND_SKIP",
                                "track_id": int(strike_track.id) if strike_track is not None else -1,
                                "master_id": "" if master_id is None else int(master_id),
                                "internal_track_id": int(strike_track.id) if strike_track is not None else -1,
                                "distance_source": strike_window["source"],
                                "reason": str(e),
                            })


                ui_threat_by_track_id = {
                    int(item["track_id"]): item for item in ranked_strike_candidates
                }
                vision_frame_ts = float(vision_result.get("frame_ts", 0.0) or 0.0)
                vision_result_age = curr_time - vision_frame_ts
                vision_result_fresh = (
                    gimbal_is_stationary
                    and 0.0 <= vision_result_age <= GIMBAL_VISION_RESULT_TTL
                )
                # E. UI receives every valid global track. A fresh matched
                # result refreshes the track filter; brief vision misses keep
                # the last filtered distance until TRACK_DISTANCE_TTL expires.
                for t in ui_tracks:
                    ui_track_result = track_results_for_strike.get(
                        int(t.id), {}
                    )
                    ui_result_age = curr_time - float(
                        ui_track_result.get("frame_ts", 0.0) or 0.0
                    )
                    ui_distance_ready = (
                        vision_result_fresh
                        and ui_track_result.get("distance_valid")
                        and 0.0 <= ui_result_age <= GIMBAL_VISION_RESULT_TTL
                    )
                    if ui_distance_ready:
                        send_dist, dist_source = select_track_distance(
                            t, master_id, curr_time
                        )
                    else:
                        held_dist, held_source = select_track_distance(
                            t, master_id, curr_time
                        )
                        if math.isfinite(held_dist):
                            send_dist = held_dist
                            dist_source = f"{held_source}_held"
                        else:
                            send_dist = float("nan")
                            dist_source = (
                                "vision_no_track_result"
                                if not ui_track_result
                                else (
                                    f"vision_{ui_track_result.get('state', 'unavailable').lower()}"
                                )
                            )
                    if math.isfinite(send_dist):
                        t.last_sent_dist = send_dist
                    threat_item = ui_threat_by_track_id.get(int(t.id))
                    threat_score = (
                        float(threat_item.get("raw_threat_score", threat_item["threat_score"]))
                        if threat_item is not None
                        else float("nan")
                    )
                    map_az = relative_to_map_azimuth(t.state[0, 0])
                    ui_id = get_or_assign_ui_id(t)
                    source_board, source_cam = track_ui_source(
                        t, board_str, cam_idx
                    )
                    sender.send_status(
                        source_board, source_cam, ui_id,
                        azimuth=map_az,
                        elevation=t.state[1, 0], 
                        distance=send_dist,
                        threat_score=threat_score
                    )
                    field_log_event({
                        "timestamp": f"{curr_time:.6f}",
                        "seq": sender_seq,
                        "mode": sender_mode,
                        "event": "UI_STATUS_SEND",
                        "track_id": int(t.id),
                        "pred_az": f"{t.state[0, 0]:.6f}",
                        "map_az": f"{map_az:.6f}",
                        "pred_el": f"{t.state[1, 0]:.6f}",
                        "hit_streak": int(t.hit_streak),
                        "time_since_update": int(t.time_since_update),
                        "lost_seconds": f"{t.lost_seconds(curr_time):.6f}",
                        "reason": (
                            f"internal_id={int(t.id)},ui_id={int(ui_id)},"
                            f"source={source_board}/{source_cam}"
                        ),
                        "internal_track_id": int(t.id),
                        "ui_id": int(ui_id),
                        "master_id": "" if master_id is None else int(master_id),
                        "is_master": 1 if t.id == master_id else 0,
                        "distance": "" if not math.isfinite(send_dist) else f"{send_dist:.6f}",
                        "distance_source": dist_source,
                        "threat_score": "" if not math.isfinite(threat_score) else f"{threat_score:.6f}",
                    })
                    ui_send_total += 1
                    ui_send_counter[int(ui_id)] += 1
            else:
                clear_strike_window(strike_window)

            maybe_print_live_status(
                curr_time,
                meas_count=len(current_measurements),
                active_tracks=active_tracks,
                valid_tracks=valid_tracks,
            )

            if PRINT_STATS and (curr_time - stats_last_print) >= STATS_PRINT_INTERVAL:
                top_id_text = "none"
                if ui_send_counter:
                    top_id_text = ", ".join([f"{tid}:{cnt}" for tid, cnt in ui_send_counter.most_common(8)])
                print(
                    f"[Stats] recv_objs_total={recv_obj_total}, recv_unique_boxes={len(recv_unique_boxes)} | "
                    f"ui_send_total={ui_send_total}, ui_unique_ids={len(ui_send_counter)}, ui_id_counts={top_id_text}"
                )
                stats_last_print = curr_time

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Err: {e}")
            import traceback
            traceback.print_exc()

    if laser_stop_event is not None:
        laser_stop_event.set()
    if laser_thread is not None:
        laser_thread.join(timeout=5.0)
        if laser_thread.is_alive():
            print("[Laser][Warn] SFL0603 reader did not stop within 5s")
    if laser is not None:
        laser.close()
    if vision_service is not None:
        vision_service.stop()
    if 'gimbal' in locals():
        gimbal.close()
    if FIELD_LOGGER is not None:
        logger = FIELD_LOGGER
        FIELD_LOGGER = None
        logger.close()
    final_id_text = "none"
    if ui_send_counter:
        final_id_text = ", ".join([f"{tid}:{cnt}" for tid, cnt in ui_send_counter.most_common()])
    print(
        f"[Stats][Final] recv_objs_total={recv_obj_total}, recv_unique_boxes={len(recv_unique_boxes)} | "
        f"ui_send_total={ui_send_total}, ui_unique_ids={len(ui_send_counter)}, ui_id_counts={final_id_text}"
    )

if __name__ == "__main__":
    main()
