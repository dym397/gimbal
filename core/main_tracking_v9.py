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
    from sddm_laser import SDDMLaser
except ImportError:
    SDDMLaser = None
try:
    from gps import (
        DEFAULT_LATITUDE,
        DEFAULT_LONGITUDE,
        read_gps_fix,
        wgs84_to_gcj02,
    )
except ImportError:
    DEFAULT_LATITUDE = None
    DEFAULT_LONGITUDE = None
    read_gps_fix = None
    wgs84_to_gcj02 = None
try:
    from rid_tracking import (
        RIDFourPointAssociator,
        RIDStreamParser,
        RIDTrackManager,
        RIDTrajectoryRenderer,
        enrich_rid_tracks,
    )
except ImportError:
    RIDFourPointAssociator = None
    RIDStreamParser = None
    RIDTrackManager = None
    RIDTrajectoryRenderer = None
    enrich_rid_tracks = None
try:
    from gimbal_vision_ranging import GimbalVisionRangingService
except ImportError:
    GimbalVisionRangingService = None
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
            "rid": "",
        }
    return {
        "gimbal": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.2:1.0-port0",
        "laser": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.4:1.0-port0",
        "gps": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.1:1.0-port0",
        "rid": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.3:1.0-port0",
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
ENABLE_GIMBAL_VISION = _env_flag("ENABLE_GIMBAL_VISION", False)
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
GIMBAL_PORT = _serial_port("GIMBAL_PORT", "gimbal")
LASER_PORT = _serial_port("LASER_PORT", "laser")
GPS_PORT = _serial_port("GPS_PORT", "gps")
RID_PORT = _serial_port("RID_PORT", "rid")
USE_MOCK_GIMBAL = _env_flag("USE_MOCK_GIMBAL", False)  # True: 使用 mock_gimbal.py; False: 使用真实 GT06Z
USE_MOCK_LASER = _env_flag("USE_MOCK_LASER", True)   # RID branch default: do not open the legacy laser.
ENABLE_GPS = _env_flag("ENABLE_GPS", True)
ENABLE_RID = _env_flag("ENABLE_RID", bool(RID_PORT))
GPS_BAUDRATE = 115200
GPS_FIX_TIMEOUT_SECONDS = 5
GPS_STATUS_INTERVAL = 5.0
GPS_UI_SEND_INTERVAL = 10.0
GPS_DEBUG_RAW = _env_flag("GPS_DEBUG_RAW", False)
RID_BAUDRATE = _env_int("RID_BAUDRATE", 115200)
RID_SERIAL_TIMEOUT = _env_float("RID_SERIAL_TIMEOUT", 0.20)
RID_RECONNECT_SECONDS = _env_float("RID_RECONNECT_SECONDS", 2.0)
RID_TRACK_TTL_SECONDS = _env_float("RID_TRACK_TTL_SECONDS", 5.0)
RID_TRACK_DELETE_AFTER_SECONDS = _env_float(
    "RID_TRACK_DELETE_AFTER_SECONDS", 300.0
)
RID_UI_RENDER_DELAY_SECONDS = _env_float("RID_UI_RENDER_DELAY_SECONDS", 0.8)
RID_UI_MAX_PREDICTION_SECONDS = _env_float(
    "RID_UI_MAX_PREDICTION_SECONDS", 0.5
)
RID_UI_DISPLAY_TAU_SECONDS = _env_float("RID_UI_DISPLAY_TAU_SECONDS", 0.2)
RID_UI_FILTER_ALPHA = _env_float("RID_UI_FILTER_ALPHA", 0.85)
RID_UI_FILTER_BETA = _env_float("RID_UI_FILTER_BETA", 0.18)
RID_UI_TURN_RESET_DEG = _env_float("RID_UI_TURN_RESET_DEG", 90.0)
RID_UI_RENDER_HISTORY_POINTS = _env_int("RID_UI_RENDER_HISTORY_POINTS", 12)
RID_UI_MAX_SPEED_MPS = _env_float("RID_UI_MAX_SPEED_MPS", 40.0)
RID_UI_THREAT_HIGH_MAX_DISTANCE_M = 100.0
RID_UI_THREAT_MEDIUM_MAX_DISTANCE_M = 300.0
RID_UI_THREAT_HIGH_SCORE = 100.0
RID_UI_THREAT_MEDIUM_SCORE = 50.0
RID_UI_THREAT_LOW_SCORE = 0.0
RID_ASSOC_MAX_AZ_DEG = _env_float("RID_ASSOC_MAX_AZ_DEG", 8.0)
RID_ASSOC_AMBIGUITY_MARGIN_DEG = _env_float(
    "RID_ASSOC_AMBIGUITY_MARGIN_DEG", 2.0
)
RID_ASSOC_CONFIRM_UPDATES = _env_int("RID_ASSOC_CONFIRM_UPDATES", 3)
RID_ASSOC_TRAJECTORY_POINTS = _env_int("RID_ASSOC_TRAJECTORY_POINTS", 10)
RID_ASSOC_MIN_TRAJECTORY_POINTS = _env_int(
    "RID_ASSOC_MIN_TRAJECTORY_POINTS", 4
)
RID_ASSOC_HISTORY_SECONDS = _env_float("RID_ASSOC_HISTORY_SECONDS", 12.0)
RID_ASSOC_SYNC_TOLERANCE_SECONDS = _env_float(
    "RID_ASSOC_SYNC_TOLERANCE_SECONDS", 0.50
)
RID_ASSOC_CURRENT_WEIGHT = _env_float("RID_ASSOC_CURRENT_WEIGHT", 0.35)
RID_ASSOC_CURVE_WEIGHT = _env_float("RID_ASSOC_CURVE_WEIGHT", 0.40)
RID_ASSOC_TREND_WEIGHT = _env_float("RID_ASSOC_TREND_WEIGHT", 0.25)
RID_ASSOC_MAX_CURVE_ERROR_DEG = _env_float(
    "RID_ASSOC_MAX_CURVE_ERROR_DEG", 8.0
)
RID_ASSOC_HOLD_SECONDS = _env_float("RID_ASSOC_HOLD_SECONDS", 3.0)
RID_ASSOC_HOLD_MAX_AZ_DEG = _env_float("RID_ASSOC_HOLD_MAX_AZ_DEG", 12.0)
RID_ASSOC_LOG_INTERVAL = _env_float("RID_ASSOC_LOG_INTERVAL", 0.50)
RID_FOUR_POINT_BIAS_DEG = _env_float(
    "RID_FOUR_POINT_BIAS_DEG", 3.1198935
)
RID_FOUR_POINT_WINDOW = _env_int("RID_FOUR_POINT_WINDOW", 4)
RID_FOUR_POINT_MAX_BIAS_ERROR_DEG = _env_float(
    "RID_FOUR_POINT_MAX_BIAS_ERROR_DEG", 2.0
)
RID_FOUR_POINT_MAX_SHAPE_P95_DEG = _env_float(
    "RID_FOUR_POINT_MAX_SHAPE_P95_DEG", 2.5
)
RID_ALLOW_DEFAULT_STATION_POSITION = _env_flag(
    "RID_ALLOW_DEFAULT_STATION_POSITION", False
)
DEVICE_HEADING_DEG = _env_float("DEVICE_HEADING_DEG", 180) % 360.0  # 设备自身0度方向的地图方位：北0/东90/南180
# GIMBAL_AZ_BASE = 57.4  # 云台水平基准角（UI绝对方位 0° 映射到控制角的基准）
# GIMBAL_INIT_EL = -0.4  # 启动时俯仰归位角，目标通常从该方向进入
#测试版本基准角度
GIMBAL_AZ_BASE = 60.3  # 云台编码器基准：设备自身相对方位0°映射到该控制角
GIMBAL_INIT_EL = -0.4# 启动时俯仰归位角，目标通常从该方向进入
GIMBAL_CMD_DEADBAND_AZ = 0.20
GIMBAL_CMD_DEADBAND_EL = 0.12
AZ_PREEMPT_DEG = 0.3     # 方位轴抢占阈值，单位：度
EL_PREEMPT_DEG = 0.8      # 俯仰轴抢占阈值，单位：度
GIMBAL_SETTLE_THRESHOLD = 0.3
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
PRINT_LIVE_STATUS = _env_flag("PRINT_LIVE_STATUS", False)
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
TRACK_REACQUIRE_MAX_DEG = _env_float("TRACK_REACQUIRE_MAX_DEG", 3.8)  # stricter cap after a longer detection gap
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
        self.raw_rid_f = open(os.path.join(log_dir, f"raw_rid_{timestamp}.jsonl"), "a", encoding="utf-8", newline="\n")
        self.raw_rid_serial_f = open(
            os.path.join(log_dir, f"raw_rid_serial_{timestamp}.jsonl"),
            "a",
            encoding="utf-8",
            newline="\n",
        )
        self.measurements_f = open(os.path.join(log_dir, f"measurements_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.summary_f = open(os.path.join(log_dir, f"track_summary_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.events_f = open(os.path.join(log_dir, f"events_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.gimbal_f = open(os.path.join(log_dir, f"gimbal_{timestamp}.csv"), "a", encoding="utf-8", newline="")
        self.rid_association_f = open(os.path.join(log_dir, f"rid_association_{timestamp}.csv"), "a", encoding="utf-8", newline="")

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
            "valid_count", "track_ids", "valid_ids", "ui_ids", "master_id",
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
            "is_edge_bbox", "visible_ratio",
        ]
        self.gimbal_fields = [
            "timestamp", "event", "cmd_id", "track_id", "cmd_az", "cmd_el",
            "gimbal_ui_az", "gimbal_ui_el", "gimbal_ctrl_az", "gimbal_ctrl_el",
            "target_ctrl_az", "target_ctrl_el", "err_az", "err_el",
            "is_settled", "is_stationary", "settle_time",
            "retry_axes", "driver_status", "laser_valid", "laser_dist",
            "laser_source", "laser_ts", "laser_age", "laser_interval",
        ]
        self.rid_association_fields = [
            "timestamp", "event", "cycle", "master_id",
            "sort_count", "rid_count", "binding_count",
            "sort_track_id", "board", "cam", "logic_id",
            "sort_relative_az", "sort_map_az", "sort_el", "sort_lost_seconds",
            "rid_ui_id", "rid_id", "rid_id_type", "rid_standard",
            "rid_map_az", "rid_elevation_deg", "rid_height",
            "az_error_deg", "curve_error_deg",
            "shape_error_deg", "trend_error_deg", "curve_bias_deg",
            "association_cost_deg", "trajectory_samples", "trajectory_ready",
            "max_az_error_deg", "max_curve_error_deg",
            "selected", "ambiguous", "binding_state", "reason",
            "distance_m", "rid_age_s", "rid_update_seq",
            "rid_measurement_seq", "rid_timestamp",
            "rid_latitude", "rid_longitude", "rid_alt_geo",
            "rid_raw_latitude", "rid_raw_longitude", "rid_raw_alt_geo",
            "rid_render_mode", "rid_filter_update_mode",
            "rid_render_timestamp", "rid_render_delay_s",
            "rid_prediction_age_s",
            "station_latitude", "station_longitude", "station_source",
            "station_altitude_m", "vertical_delta_m",
            "station_age_s", "device_heading_deg",
        ]

        self.measurements_writer = csv.DictWriter(self.measurements_f, fieldnames=self.measurements_fields, extrasaction="ignore")
        self.summary_writer = csv.DictWriter(self.summary_f, fieldnames=self.summary_fields, extrasaction="ignore")
        self.events_writer = csv.DictWriter(self.events_f, fieldnames=self.events_fields, extrasaction="ignore")
        self.gimbal_writer = csv.DictWriter(self.gimbal_f, fieldnames=self.gimbal_fields, extrasaction="ignore")
        self.rid_association_writer = csv.DictWriter(
            self.rid_association_f,
            fieldnames=self.rid_association_fields,
            extrasaction="ignore",
        )
        self.measurements_writer.writeheader()
        self.summary_writer.writeheader()
        self.events_writer.writeheader()
        self.gimbal_writer.writeheader()
        self.rid_association_writer.writeheader()
        # The SORT/RID alignment sidecar discovers these files as soon as they
        # exist. Publish every header before announcing that field logging is
        # ready, otherwise the sidecar can briefly observe an empty CSV and
        # exit during service startup.
        for stream in (
            self.raw_f,
            self.raw_rid_f,
            self.raw_rid_serial_f,
            self.measurements_f,
            self.summary_f,
            self.events_f,
            self.gimbal_f,
            self.rid_association_f,
        ):
            stream.flush()
        self.last_flush_t = time.monotonic()
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
        self.raw_rid_f.flush()
        self.raw_rid_serial_f.flush()
        self.measurements_f.flush()
        self.summary_f.flush()
        self.events_f.flush()
        self.gimbal_f.flush()
        self.rid_association_f.flush()
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

    def write_raw_rid(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self.raw_rid_f.write(
                    json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n"
                )
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def write_raw_rid_serial(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self.raw_rid_serial_f.write(
                    json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n"
                )
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

    def write_rid_association(self, row):
        if self.disabled:
            return
        try:
            with self.lock:
                self._write_csv(
                    self.rid_association_writer,
                    self.rid_association_fields,
                    row,
                )
                self._flush_if_due_locked()
        except Exception as e:
            self._handle_write_error(e)

    def flush(self):
        if self.disabled:
            return
        try:
            with self.lock:
                self.raw_f.flush()
                self.raw_rid_f.flush()
                self.raw_rid_serial_f.flush()
                self.measurements_f.flush()
                self.summary_f.flush()
                self.events_f.flush()
                self.gimbal_f.flush()
                self.rid_association_f.flush()
                self.last_flush_t = time.monotonic()
        except Exception as e:
            self._handle_write_error(e)

    def close(self):
        with self.lock:
            for f in (
                self.raw_f,
                self.raw_rid_f,
                self.raw_rid_serial_f,
                self.measurements_f,
                self.summary_f,
                self.events_f,
                self.gimbal_f,
                self.rid_association_f,
            ):
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
# 最终版本 theta；水平角已整体执行 (原值 - 1°) % 360°，以第一层第三摄像头为0°基准。
DEVICE_THETA = {
    1: {"theta_vertical": 0.0000, "theta_horizontal": 32.4501},  # Layer 1 cam1
    2: {"theta_vertical": 0.0000, "theta_horizontal": 16.7874},  # Layer 1 cam2
    3: {"theta_vertical": 0.0000, "theta_horizontal": 0.0000},   # Layer 1 cam3
    4: {"theta_vertical": 0.0000, "theta_horizontal": 343.8421}, # Layer 1 cam4
    5: {"theta_vertical": 0.0000, "theta_horizontal": 326.6701}, # Layer 1 cam5

    6: {"theta_vertical": 5.5, "theta_horizontal": 34.3086},     # Layer 2 cam1
    7: {"theta_vertical": 5.5, "theta_horizontal": 15.5593},     # Layer 2 cam2
    8: {"theta_vertical": 5.5, "theta_horizontal": 0.0759},      # Layer 2 cam3
    9: {"theta_vertical": 5.5, "theta_horizontal": 342.7710},    # Layer 2 cam4
    10: {"theta_vertical": 5.5, "theta_horizontal": 321.0000},   # Layer 2 cam5

    11: {"theta_vertical": 15.5000, "theta_horizontal": 31.9870},  # Layer 3 cam1
    12: {"theta_vertical": 15.5000, "theta_horizontal": 19.6921},  # Layer 3 cam2
    13: {"theta_vertical": 15.5000, "theta_horizontal": 0.9703},   # Layer 3 cam3
    14: {"theta_vertical": 15.5000, "theta_horizontal": 339.9173}, # Layer 3 cam4
    15: {"theta_vertical": 15.5000, "theta_horizontal": 322.7531}, # Layer 3 cam5

    16: {"theta_vertical": 25.5000, "theta_horizontal": 36.7386},  # Layer 4 cam1
    17: {"theta_vertical": 25.5000, "theta_horizontal": 18.9455},  # Layer 4 cam2
    18: {"theta_vertical": 25.5000, "theta_horizontal": 0.7703},   # Layer 4 cam3
    19: {"theta_vertical": 25.5000, "theta_horizontal": 341.8870}, # Layer 4 cam4
    20: {"theta_vertical": 25.5000, "theta_horizontal": 323.5302}, # Layer 4 cam5

    21: {"theta_vertical": 35.0000, "theta_horizontal": 37.5403},  # Layer 5 cam1
    22: {"theta_vertical": 35.0000, "theta_horizontal": 18.5903},  # Layer 5 cam2
    23: {"theta_vertical": 35.0000, "theta_horizontal": 0.7703},   # Layer 5 cam3
    24: {"theta_vertical": 35.0000, "theta_horizontal": 340.7103}, # Layer 5 cam4
    25: {"theta_vertical": 35.0000, "theta_horizontal": 321.7703}, # Layer 5 cam5

    26: {"theta_vertical": 44.5000, "theta_horizontal": 41.7703},  # Layer 6 cam1
    27: {"theta_vertical": 44.5000, "theta_horizontal": 21.7703},  # Layer 6 cam2
    28: {"theta_vertical": 44.5000, "theta_horizontal": 1.7703},   # Layer 6 cam3
    29: {"theta_vertical": 44.5000, "theta_horizontal": 341.7703}, # Layer 6 cam4
    30: {"theta_vertical": 44.5000, "theta_horizontal": 321.7703}, # Layer 6 cam5

    31: {"theta_vertical": 54.0000, "theta_horizontal": 45.2403},  # Layer 7 cam1
    32: {"theta_vertical": 54.0000, "theta_horizontal": 24.0403},  # Layer 7 cam2
    33: {"theta_vertical": 54.0000, "theta_horizontal": 2.8703},   # Layer 7 cam3
    34: {"theta_vertical": 54.0000, "theta_horizontal": 340.6003}, # Layer 7 cam4
    35: {"theta_vertical": 54.0000, "theta_horizontal": 319.4303}, # Layer 7 cam5

    36: {"theta_vertical": 63.5000, "theta_horizontal": 47.8703},  # Layer 8 cam1
    37: {"theta_vertical": 63.5000, "theta_horizontal": 25.3703},  # Layer 8 cam2
    38: {"theta_vertical": 63.5000, "theta_horizontal": 2.8703},   # Layer 8 cam3
    39: {"theta_vertical": 63.5000, "theta_horizontal": 340.3703}, # Layer 8 cam4
    40: {"theta_vertical": 63.5000, "theta_horizontal": 317.8703}, # Layer 8 cam5

    41: {"theta_vertical": 73.0000, "theta_horizontal": 61.6703},  # Layer 9 cam1
    42: {"theta_vertical": 73.0000, "theta_horizontal": 37.6703},  # Layer 9 cam2
    43: {"theta_vertical": 73.0000, "theta_horizontal": 13.6703},  # Layer 9 cam3
    44: {"theta_vertical": 73.0000, "theta_horizontal": 349.6703}, # Layer 9 cam4
    45: {"theta_vertical": 73.0000, "theta_horizontal": 325.6703}, # Layer 9 cam5
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

shared_state = SharedHardwareState()
gimbal_cmd_queue = queue.Queue(maxsize=1)
packet_queue = deque(maxlen=PACKET_QUEUE_MAXLEN)


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
    print("[LaserThread] 激光读取线程已启动")
    try:
        laser.start_measurement(continuous=True)
    except Exception as e:
        print(f"[Laser][Fatal] 启动连续测量失败: {e}")
        return

    last_log_t = 0.0
    while not stop_event.is_set():
        dist = laser.read_distance()
        if dist is None:
            continue
        now_t = time.time()
        if (now_t - last_log_t) >= LASER_LOG_INTERVAL:
            field_log_gimbal({
                "timestamp": f"{now_t:.6f}",
                "event": "LASER_READ_ONLY",
                "laser_valid": 1,
                "laser_dist": f"{float(dist):.6f}",
                "laser_source": "sddm",
                "laser_ts": f"{now_t:.6f}",
            })
            last_log_t = now_t


class SharedPositionState:
    """Thread-safe WGS-84 station position and ellipsoid height for RID."""

    def __init__(
        self,
        longitude=None,
        latitude=None,
        altitude=None,
        source="unavailable",
    ):
        self.lock = threading.Lock()
        self.longitude = longitude
        self.latitude = latitude
        self.altitude = altitude
        self.source = source
        self.updated_ts = time.time() if longitude is not None and latitude is not None else 0.0

    def update(
        self,
        longitude,
        latitude,
        source,
        altitude=None,
        updated_ts=None,
    ):
        with self.lock:
            self.longitude = float(longitude)
            self.latitude = float(latitude)
            if altitude is not None:
                self.altitude = float(altitude)
            self.source = str(source)
            self.updated_ts = time.time() if updated_ts is None else float(updated_ts)

    def snapshot(self, now_ts=None):
        now_ts = time.time() if now_ts is None else float(now_ts)
        with self.lock:
            longitude = self.longitude
            latitude = self.latitude
            altitude = self.altitude
            source = self.source
            updated_ts = self.updated_ts
        return {
            "longitude": longitude,
            "latitude": latitude,
            "altitude": altitude,
            "source": source,
            "updated_ts": updated_ts,
            "age_s": (
                max(0.0, now_ts - updated_ts)
                if updated_ts > 0.0
                else math.inf
            ),
            "valid": longitude is not None and latitude is not None,
            "altitude_valid": altitude is not None and math.isfinite(altitude),
        }


def rid_reader_thread(track_manager, stop_event):
    if RIDStreamParser is None:
        print("[RID][Warn] rid_tracking.py import failed; RID reader disabled.")
        return
    try:
        import serial
    except ImportError as e:
        print(f"[RID][Warn] pyserial unavailable: {e}")
        return

    parser = RIDStreamParser()
    parser_counter_names = (
        "discarded_bytes",
        "decode_errors",
        "valid_frames",
        "header_errors",
        "length_errors",
        "tail_errors",
        "truncated_frames",
        "utf8_errors",
        "json_errors",
        "json_type_errors",
    )
    while not stop_event.is_set():
        try:
            with serial.Serial(
                RID_PORT,
                RID_BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=RID_SERIAL_TIMEOUT,
            ) as ser:
                print(f"[RID] Serial reader connected: {RID_PORT}@{RID_BAUDRATE}")
                field_log_event({
                    "timestamp": f"{time.time():.6f}",
                    "event": "RID_SERIAL_OPEN",
                    "reason": (
                        f"port={RID_PORT},baud={RID_BAUDRATE},format=8N1,"
                        "frame=081730+len_le_including_2_length_bytes+json+3f55"
                    ),
                })
                while not stop_event.is_set():
                    waiting = int(getattr(ser, "in_waiting", 0) or 0)
                    chunk = ser.read(max(1, min(waiting, 65536)))
                    if not chunk:
                        continue
                    receive_ts = time.time()
                    counters_before = {
                        name: int(getattr(parser, name))
                        for name in parser_counter_names
                    }
                    payloads = parser.feed(chunk)
                    counter_deltas = {
                        name: int(getattr(parser, name)) - counters_before[name]
                        for name in parser_counter_names
                    }
                    decoded_frames = list(parser.last_feed_frames)
                    if FIELD_LOGGER is not None:
                        FIELD_LOGGER.write_raw_rid_serial({
                            "receive_ts": receive_ts,
                            "byte_count": len(chunk),
                            "chunk_hex": chunk.hex(),
                            "parsed_payload_count": len(payloads),
                            "decoded_length_fields": [
                                item["length_field"] for item in decoded_frames
                            ],
                            "decoded_payload_lengths": [
                                item["payload_length"] for item in decoded_frames
                            ],
                            "decoded_frame_lengths": [
                                item["frame_length"] for item in decoded_frames
                            ],
                            "discarded_bytes_delta": counter_deltas["discarded_bytes"],
                            "discarded_bytes_total": parser.discarded_bytes,
                            "decode_errors_delta": counter_deltas["decode_errors"],
                            "decode_errors_total": parser.decode_errors,
                            "valid_frames_delta": counter_deltas["valid_frames"],
                            "valid_frames_total": parser.valid_frames,
                            "header_errors_delta": counter_deltas["header_errors"],
                            "header_errors_total": parser.header_errors,
                            "length_errors_delta": counter_deltas["length_errors"],
                            "length_errors_total": parser.length_errors,
                            "tail_errors_delta": counter_deltas["tail_errors"],
                            "tail_errors_total": parser.tail_errors,
                            "truncated_frames_delta": counter_deltas["truncated_frames"],
                            "truncated_frames_total": parser.truncated_frames,
                            "utf8_errors_delta": counter_deltas["utf8_errors"],
                            "utf8_errors_total": parser.utf8_errors,
                            "json_errors_delta": counter_deltas["json_errors"],
                            "json_errors_total": parser.json_errors,
                            "json_type_errors_delta": counter_deltas["json_type_errors"],
                            "json_type_errors_total": parser.json_type_errors,
                            "parser_buffer_bytes_after": len(parser.buffer),
                        })
                    parse_error_deltas = {
                        name: counter_deltas[name]
                        for name in (
                            "header_errors",
                            "length_errors",
                            "tail_errors",
                            "truncated_frames",
                            "utf8_errors",
                            "json_errors",
                            "json_type_errors",
                        )
                        if counter_deltas[name]
                    }
                    if parse_error_deltas:
                        field_log_event({
                            "timestamp": f"{receive_ts:.6f}",
                            "event": "RID_FRAME_PARSE_ERROR",
                            "reason": (
                                f"chunk_bytes={len(chunk)},"
                                f"errors={parse_error_deltas},"
                                f"resync_discarded_delta="
                                f"{counter_deltas['discarded_bytes']},"
                                f"buffer_after={len(parser.buffer)}"
                            ),
                        })
                    for payload in payloads:
                        if FIELD_LOGGER is not None:
                            FIELD_LOGGER.write_raw_rid({
                                "receive_ts": receive_ts,
                                "payload": payload,
                            })
                        result = track_manager.update_payload(
                            payload,
                            receive_ts=receive_ts,
                        )
                        track = result.get("track") or {}
                        if result.get("accepted"):
                            rid_event = (
                                "RID_TRACK_UPDATE"
                                if track.get("position_valid")
                                else "RID_POSITION_INVALID"
                            )
                        else:
                            rid_event = "RID_PAYLOAD_REJECT"
                        field_log_event({
                            "timestamp": f"{receive_ts:.6f}",
                            "event": rid_event,
                            "ui_id": track.get("ui_id", ""),
                            "distance_source": (
                                "rid_gps" if track.get("position_valid") else ""
                            ),
                            "reason": (
                                f"result={result.get('reason', '')},"
                                f"rid_id={track.get('rid_id', result.get('rid_id', ''))},"
                                f"position_valid={track.get('position_valid', '')},"
                                f"lat={track.get('latitude', '')},"
                                f"lon={track.get('longitude', '')},"
                                f"rid_ts={track.get('rid_timestamp', '')},"
                                f"update_seq={track.get('update_seq', '')},"
                                f"measurement_seq={track.get('measurement_seq', '')},"
                                f"expired_ui_ids="
                                f"{[item.get('ui_id') for item in result.get('expired_tracks', [])]},"
                                f"invalid_position_count="
                                f"{track.get('invalid_position_count', '')}"
                            ),
                        })
        except Exception as e:
            print(f"[RID][Warn] serial reader error on {RID_PORT}: {e}")
            field_log_event({
                "timestamp": f"{time.time():.6f}",
                "event": "RID_SERIAL_ERROR",
                "reason": str(e),
            })
            stop_event.wait(max(0.1, RID_RECONNECT_SECONDS))


def gps_sender_thread(sender, position_state=None):
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
    last_sent_altitude = None
    last_sent_source = "default"

    while True:
        cycle_start = time.monotonic()
        #print("[GPS] Searching satellites and waiting for valid latitude/longitude...")
        longitude, latitude, altitude, source = read_gps_fix(
            port=GPS_PORT,
            baudrate=GPS_BAUDRATE,
            timeout_seconds=GPS_FIX_TIMEOUT_SECONDS,
            print_raw=GPS_DEBUG_RAW,
            print_status=True,
            status_interval=GPS_STATUS_INTERVAL,
            coordinate_system="wgs84",
            include_altitude=True,
        )

        send_source = source
        if longitude is not None and latitude is not None:
            last_sent_longitude = longitude
            last_sent_latitude = latitude
            if altitude is not None:
                last_sent_altitude = altitude
            last_sent_source = source
        elif last_sent_longitude is not None and last_sent_latitude is not None:
            longitude = last_sent_longitude
            latitude = last_sent_latitude
            altitude = last_sent_altitude
            send_source = f"cached:{last_sent_source}"
        else:
            longitude = DEFAULT_LONGITUDE
            latitude = DEFAULT_LATITUDE
            altitude = None
            send_source = "default"

        if (
            position_state is not None
            and (
                send_source != "default"
                or RID_ALLOW_DEFAULT_STATION_POSITION
            )
        ):
            position_state.update(
                longitude=longitude,
                latitude=latitude,
                altitude=altitude,
                source=send_source,
            )
            field_log_event({
                "timestamp": f"{time.time():.6f}",
                "event": "GPS_STATION_FIX",
                "reason": (
                    f"source={send_source},lat={latitude:.8f},"
                    f"lon={longitude:.8f},ellipsoid_height_m={altitude}"
                ),
            })

        # Keep the historical UI coordinate behavior while RID calculations
        # use the unrounded WGS-84 fix stored above.
        ui_longitude = longitude
        ui_latitude = latitude
        if send_source != "default" and wgs84_to_gcj02 is not None:
            ui_longitude, ui_latitude = wgs84_to_gcj02(
                round(float(longitude), 4),
                round(float(latitude), 4),
            )
        sender.send_gps_location(
            latitude=ui_latitude,
            longitude=ui_longitude,
        )
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


def pair_ui_tracks_with_rid(
    ui_tracks,
    rid_tracks,
    four_point_associator,
    now_ts=None,
):
    """Apply four-point gates, then return one-to-one UI SORT/RID pairs."""
    if four_point_associator is None:
        return [], []
    sort_items = [
        {
            "track_id": int(track.id),
            "map_az": relative_to_map_azimuth(track.state[0, 0]),
        }
        for track in ui_tracks
    ]
    bindings, diagnostics = four_point_associator.associate(
        sort_items,
        rid_tracks,
        now_ts=now_ts,
    )
    tracks_by_id = {int(track.id): track for track in ui_tracks}
    pairs = []
    for sort_id, binding in bindings.items():
        track = tracks_by_id.get(int(sort_id))
        if track is None:
            continue
        pairs.append({
            "sort_track": track,
            "rid": binding["rid"],
            "az_error_deg": binding["current_error_deg"],
            "bias_error_deg": binding["bias_error_deg"],
            "shape_p95_deg": binding["shape_p95_deg"],
            "curve_bias_deg": binding["curve_bias_deg"],
            "association_cost_deg": binding["association_cost_deg"],
            "trajectory_samples": binding["trajectory_samples"],
        })
    return (
        sorted(pairs, key=lambda item: int(item["rid"]["ui_id"])),
        diagnostics,
    )


def complete_ui_fusion_pairs(ui_tracks, matched_pairs):
    """Keep every UI-eligible SORT track, with RID optional per track."""
    matched_by_sort_id = {
        int(item["sort_track"].id): item for item in matched_pairs
    }
    return [
        matched_by_sort_id.get(
            int(track.id),
            {
                "sort_track": track,
                "rid": None,
                "az_error_deg": math.nan,
            },
        )
        for track in ui_tracks
    ]


def build_rid_matched_ui_values(sort_track, rid_item, sort_ui_id):
    """Use SORT for UI identity/angles and RID only for distance."""
    return {
        "target_id": int(sort_ui_id),
        "azimuth": relative_to_map_azimuth(sort_track.state[0, 0]),
        "elevation": float(sort_track.state[1, 0]),
        "distance": float(rid_item["distance_m"]),
    }


def ui_threat_score_from_distance(distance_m):
    """Map a valid UI distance to the three RID threat tiers.

    This score is display-only. It does not participate in SORT selection,
    gimbal control, strike selection, or any hardware safety decision.
    """
    try:
        distance_m = float(distance_m)
    except (TypeError, ValueError):
        return float("nan")
    if not math.isfinite(distance_m) or distance_m < 0.0:
        return float("nan")
    if distance_m < RID_UI_THREAT_HIGH_MAX_DISTANCE_M:
        return RID_UI_THREAT_HIGH_SCORE
    if distance_m <= RID_UI_THREAT_MEDIUM_MAX_DISTANCE_M:
        return RID_UI_THREAT_MEDIUM_SCORE
    return RID_UI_THREAT_LOW_SCORE


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
        curr_ui_az = (curr_az - GIMBAL_AZ_BASE) % 360.0
        curr_ui_el = curr_el - GIMBAL_INIT_EL
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
                "gimbal_ui_az": f"{curr_ui_az:.6f}",
                "gimbal_ui_el": f"{curr_ui_el:.6f}",
                "gimbal_ctrl_az": f"{curr_az:.6f}",
                "gimbal_ctrl_el": f"{curr_el:.6f}",
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
                target_az = float(active_cmd["az"])#方位角
                target_el = float(active_cmd["el"])#俯仰角
                cmd_start_t = time.time()
                last_progress_log_t = 0.0
                settle_candidate_since = None
                send_status = gimbal.set_attitude(
                    elevation=target_el, azimuth=target_az
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
                        f"target_ctrl=(Az={target_az:.2f}°, El={target_el:.2f}°)"
                    )
                field_log_gimbal({
                    "timestamp": f"{cmd_start_t:.6f}",
                    "event": "GIMBAL_CMD_SEND",
                    "cmd_id": int(active_cmd["cmd_id"]),
                    "track_id": int(active_cmd.get("track_id", -1)),
                    "target_ctrl_az": f"{target_az:.6f}",
                    "target_ctrl_el": f"{target_el:.6f}",
                })
            #2.若当前有指令在执行(执行态)
            now_t = time.time()
            #检查指令队列中是否有更新的指令，如果有则取出最新的一条（丢弃旧指令），准备进行抢占式执行判断
            newer_cmd = drain_latest_gimbal_cmd()
            if newer_cmd is not None:
                new_az = float(newer_cmd["az"])#新指令的方位角
                new_el = float(newer_cmd["el"])#新指令的俯仰角
                new_track_id = int(newer_cmd.get("track_id", -1))
                curr_track_id = int(active_cmd.get("track_id", -1))
                #计算新指令与当前指令的角度差
                d_az = abs(angular_diff(new_az, target_az))
                d_el = abs(new_el - target_el)
                d_total = math.hypot(d_az, d_el)

                # 目标切换时必须整条命令替换，避免“新方位 + 旧俯仰”混合指向。
                full_replace = (new_track_id != curr_track_id)
                update_az = full_replace or (d_az > AZ_PREEMPT_DEG)
                update_el = full_replace or (d_el > EL_PREEMPT_DEG)

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
                        elevation=target_el, azimuth=target_az
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
                            f"target_ctrl=(Az={target_az:.2f}°, El={target_el:.2f}°), "
                            f"delta=(dAz={d_az:.2f}°, dEl={d_el:.2f}°, total={d_total:.2f}°)"
                        )
                    field_log_gimbal({
                        "timestamp": f"{now_t:.6f}",
                        "event": "GIMBAL_CMD_PREEMPT",
                        "cmd_id": int(active_cmd["cmd_id"]),
                        "track_id": new_track_id,
                        "target_ctrl_az": f"{target_az:.6f}",
                        "target_ctrl_el": f"{target_el:.6f}",
                        "err_az": f"{d_az:.6f}",
                        "err_el": f"{d_el:.6f}",
                    })

            if time.time() < next_feedback_query_t:
                time.sleep(GIMBAL_THREAD_SLEEP)
                continue

            real_att = gimbal.get_attitude()
            if real_att:
                curr_el, curr_az, _ = real_att
                curr_ui_az, curr_ui_el, gimbal_is_stationary = (
                    record_feedback_motion(curr_el, curr_az, now_t)
                )
                err_az = abs(angular_diff(target_az, curr_az))
                err_el = abs(curr_el - target_el)

                if (last_progress_log_t == 0.0) or ((now_t - last_progress_log_t) >= GIMBAL_PROGRESS_LOG_INTERVAL):
                    elapsed = now_t - cmd_start_t
                    if PRINT_GIMBAL_PROGRESS:
                        print(
                            f"[GimbalAtt] cmd_id={int(active_cmd['cmd_id'])}, "
                            f"elapsed={elapsed:.3f}s, "
                            f"actual_ctrl=(Az={curr_az:.2f}°, El={curr_el:.2f}°), "
                            f"actual_ui=(Az={curr_ui_az:.2f}°, El={curr_el:.2f}°), "
                            f"target_ctrl=(Az={target_az:.2f}°, El={target_el:.2f}°), "
                            f"err=(dAz={err_az:.2f}°, dEl={err_el:.2f}°)"
                        )
                    field_log_gimbal({
                        "timestamp": f"{now_t:.6f}",
                        "event": "GIMBAL_ATT",
                        "cmd_id": int(active_cmd["cmd_id"]),
                        "track_id": int(active_cmd.get("track_id", -1)),
                        "gimbal_ui_az": f"{curr_ui_az:.6f}",
                        "gimbal_ui_el": f"{curr_ui_el:.6f}",
                        "gimbal_ctrl_az": f"{curr_az:.6f}",
                        "gimbal_ctrl_el": f"{curr_el:.6f}",
                        "target_ctrl_az": f"{target_az:.6f}",
                        "target_ctrl_el": f"{target_el:.6f}",
                        "err_az": f"{err_az:.6f}",
                        "err_el": f"{err_el:.6f}",
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
                        "target_ctrl_az": f"{target_az:.6f}",
                        "target_ctrl_el": f"{target_el:.6f}",
                        "err_az": f"{err_az:.6f}",
                        "err_el": f"{err_el:.6f}",
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
                            f"actual_ctrl=(Az={curr_az:.2f}°, El={curr_el:.2f}°), "
                            f"target_ctrl=(Az={target_az:.2f}°, El={target_el:.2f}°), "
                            f"err=(dAz={err_az:.2f}°, dEl={err_el:.2f}°)"
                        )
                    field_log_gimbal({
                        "timestamp": f"{now_t:.6f}",
                        "event": "GIMBAL_SETTLED",
                        "cmd_id": int(active_cmd_id),
                        "track_id": int(active_track_id),
                        "gimbal_ui_az": f"{curr_ui_az:.6f}",
                        "gimbal_ui_el": f"{curr_ui_el:.6f}",
                        "gimbal_ctrl_az": f"{curr_az:.6f}",
                        "gimbal_ctrl_el": f"{curr_el:.6f}",
                        "target_ctrl_az": f"{target_az:.6f}",
                        "target_ctrl_el": f"{target_el:.6f}",
                        "err_az": f"{err_az:.6f}",
                        "err_el": f"{err_el:.6f}",
                        "is_settled": 1,
                        "is_stationary": 1 if gimbal_is_stationary else 0,
                        "settle_time": f"{settle_dt:.6f}",
                    })

                    # Laser readings are logged by laser_reader_thread only; distance fusion uses mono only.
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
                            f"actual_ctrl=(Az={curr_az:.2f}°, El={curr_el:.2f}°), "
                            f"target_ctrl=(Az={target_az:.2f}°, El={target_el:.2f}°), "
                            f"err=(dAz={err_az:.2f}°, dEl={err_el:.2f}°)"
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
                    "target_ctrl_az": f"{target_az:.6f}",
                    "target_ctrl_el": f"{target_el:.6f}",
                    "err_az": "" if not real_att else f"{err_az:.6f}",
                    "err_el": "" if not real_att else f"{err_el:.6f}",
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
    """将设备自身相对方位/俯仰转换为云台编码器控制角。"""
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

    def set_mono_distance(self, dist, ts, source="mono"):
        d = _parse_positive_float(dist)
        if d is None:
            return
        self.last_mono_dist = d
        self.mono_ts = float(ts)
        self._update_distance_filter(d, ts, source=source)

    def _update_distance_filter(self, dist_val, ts, source="mono"):
        """1D 距离卡尔曼滤波，含异常门控与新鲜度管理"""
        curr_t = float(ts)
        
        # Reset stale mono distance state instead of carrying old range into a new target interval.
        if self.dist_state is not None and (curr_t - self.last_dist_ts) > TRACK_DISTANCE_TTL:
            self.dist_state = None
            
        if self.dist_state is None:
            self.dist_state = np.array([[dist_val], [0.0]], dtype=float)
            self.last_dist_ts = curr_t
            self.dist_source = str(source)
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
        self.dist_source = str(source)
        
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

    # 3. 计算设备自身坐标系角度；这还不是真北地图方位角。
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
    station_position = SharedPositionState(
        longitude=(
            DEFAULT_LONGITUDE if RID_ALLOW_DEFAULT_STATION_POSITION else None
        ),
        latitude=(
            DEFAULT_LATITUDE if RID_ALLOW_DEFAULT_STATION_POSITION else None
        ),
        source=(
            "configured_default"
            if RID_ALLOW_DEFAULT_STATION_POSITION
            else "waiting_for_wgs84_fix"
        ),
    )
    rid_track_manager = None
    rid_trajectory_renderer = None
    rid_four_point_associator = None
    # The legacy weighted/held association remains disabled. The active
    # implementation is the four-point bias/shape gate initialized below.
    rid_associator = None
    rid_stop_event = None
    if ENABLE_RID:
        if not RID_PORT:
            print("[RID][Warn] ENABLE_RID=True but RID_PORT is empty; RID disabled.")
        elif RIDTrackManager is None:
            print("[RID][Warn] rid_tracking.py import failed; RID disabled.")
        else:
            rid_track_manager = RIDTrackManager(
                track_ttl_s=RID_TRACK_TTL_SECONDS,
                delete_after_s=RID_TRACK_DELETE_AFTER_SECONDS,
            )
            if RIDTrajectoryRenderer is not None:
                rid_trajectory_renderer = RIDTrajectoryRenderer(
                    render_delay_s=RID_UI_RENDER_DELAY_SECONDS,
                    max_prediction_s=RID_UI_MAX_PREDICTION_SECONDS,
                    active_ttl_s=RID_TRACK_TTL_SECONDS,
                    display_tau_s=RID_UI_DISPLAY_TAU_SECONDS,
                    alpha=RID_UI_FILTER_ALPHA,
                    beta=RID_UI_FILTER_BETA,
                    turn_reset_deg=RID_UI_TURN_RESET_DEG,
                    history_points=RID_UI_RENDER_HISTORY_POINTS,
                    max_speed_mps=RID_UI_MAX_SPEED_MPS,
                    delete_after_s=RID_TRACK_DELETE_AFTER_SECONDS,
                )
            if RIDFourPointAssociator is not None:
                rid_four_point_associator = RIDFourPointAssociator(
                    learned_bias_deg=RID_FOUR_POINT_BIAS_DEG,
                    window_points=RID_FOUR_POINT_WINDOW,
                    max_bias_error_deg=RID_FOUR_POINT_MAX_BIAS_ERROR_DEG,
                    max_shape_p95_deg=RID_FOUR_POINT_MAX_SHAPE_P95_DEG,
                    history_seconds=RID_ASSOC_HISTORY_SECONDS,
                    sync_tolerance_seconds=RID_ASSOC_SYNC_TOLERANCE_SECONDS,
                )
            rid_stop_event = threading.Event()
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
    print(
        "[Config] RID UI fusion: "
        f"enabled={1 if rid_track_manager is not None else 0}, "
        f"port={RID_PORT or 'unset'}, baud={RID_BAUDRATE}, "
        "association=four_point_bias_shape_gate, "
        "ui=sort_id/sort_az/sort_el/rid_distance+sort_camera, "
        f"bias={RID_FOUR_POINT_BIAS_DEG:.6f}deg, "
        f"window={RID_FOUR_POINT_WINDOW}, "
        f"max_ebias={RID_FOUR_POINT_MAX_BIAS_ERROR_DEG:.2f}deg, "
        f"max_eshape95={RID_FOUR_POINT_MAX_SHAPE_P95_DEG:.2f}deg, "
        f"sync={RID_ASSOC_SYNC_TOLERANCE_SECONDS:.2f}s, "
        f"render_delay={RID_UI_RENDER_DELAY_SECONDS:.2f}s, "
        f"max_prediction={RID_UI_MAX_PREDICTION_SECONDS:.2f}s, "
        f"display_tau={RID_UI_DISPLAY_TAU_SECONDS:.2f}s, "
        f"turn_reset={RID_UI_TURN_RESET_DEG:.1f}deg, "
        f"rid_ttl={RID_TRACK_TTL_SECONDS:.2f}s, "
        f"rid_delete_after={RID_TRACK_DELETE_AFTER_SECONDS:.2f}s"
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
    if rid_track_manager is not None:
        _validate_serial_port("RID_PORT", RID_PORT)

    if ENABLE_GPS:
        threading.Thread(
            target=gps_sender_thread,
            args=(sender, station_position),
            daemon=True,
        ).start()
    if rid_track_manager is not None:
        threading.Thread(
            target=rid_reader_thread,
            args=(rid_track_manager, rid_stop_event),
            daemon=True,
        ).start()
    
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
    if USE_MOCK_LASER:
        print(
            "[Init] Legacy laser disabled by USE_MOCK_LASER=True; "
            "RID distance path is independent"
        )
    else:
        if SDDMLaser is None:
            print("[Laser][Warn] sddm_laser.py import failed; distance source remains mono.")
        else:
            try:
                laser = SDDMLaser(LASER_PORT)
                laser_stop_event = threading.Event()
                threading.Thread(
                    target=laser_reader_thread,
                    args=(laser, laser_stop_event),
                    daemon=True,
                ).start()
                print(f"[Init] Real laser read-only logging enabled on {LASER_PORT}")
            except Exception as e:
                print(f"[Laser][Warn] Init failed on {LASER_PORT}: {e}")
                laser = None

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
        f"Az={GIMBAL_AZ_BASE:.2f}°, El={GIMBAL_INIT_EL:.2f}°"
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
                )
                vision_service.start()
                print(
                    f"[GimbalVision] enabled camera={GIMBAL_CAMERA_SOURCE!r}, "
                    "YOLO=bestall_2k.rknn native 2560x1440 on RKNN/NPU, "
                    "association=simple_center_yolo"
                )
            except Exception as e:
                vision_service = None
                print(f"[GimbalVision][Warn] initialization failed: {e}")
    else:
        print("[GimbalVision] disabled by ENABLE_GIMBAL_VISION=False")

    distance_mode = (
        "RID WGS-84 horizontal distance"
        if rid_track_manager is not None
        else (
            "gimbal camera YOLO/MLP/GRU"
            if vision_service is not None
            else "none"
        )
    )
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
    latest_rid_bindings = {}
    last_applied_rid_measurement = {}
    rid_assoc_last_log_ts = 0.0
    rid_assoc_cycle = 0
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
        rid_binding = latest_rid_bindings.get(internal_id)
        if rid_binding is not None:
            return int(rid_binding["rid"]["ui_id"])
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
            if rid_track_manager is not None:
                for expired_track in rid_track_manager.prune_expired(curr_time):
                    field_log_event({
                        "timestamp": f"{curr_time:.6f}",
                        "event": "RID_TRACK_EXPIRED_DELETE",
                        "ui_id": expired_track.get("ui_id", ""),
                        "distance_source": "",
                        "reason": (
                            f"rid_id={expired_track.get('rid_id', '')},"
                            f"key={expired_track.get('key_text', '')},"
                            f"silence_s={expired_track.get('expired_age_s', 0.0):.6f},"
                            f"delete_after_s={RID_TRACK_DELETE_AFTER_SECONDS:.6f},"
                            "action=permanent_delete;next_report_creates_new_ui_id"
                        ),
                    })
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
                # Legacy persistent-association diagnostics. This block is
                # intentionally unreachable because rid_associator is None.
                if rid_track_manager is not None and rid_associator is not None:
                    idle_station = station_position.snapshot(curr_time)
                    if idle_station["valid"]:
                        idle_rid_tracks = enrich_rid_tracks(
                            rid_track_manager.snapshot(now_ts=curr_time),
                            station_latitude=idle_station["latitude"],
                            station_longitude=idle_station["longitude"],
                            station_altitude=idle_station["altitude"],
                        )
                        idle_sort_assoc_items = [
                            {
                                "track_id": int(track.id),
                                "created_ts": float(track.created_ts),
                                "map_az": relative_to_map_azimuth(
                                    track.state[0, 0]
                                ),
                            }
                            for track in tracker.tracks
                            if track_is_rid_association_eligible(
                                track, curr_time
                            )
                        ]
                        rid_associator.observe(
                            idle_sort_assoc_items,
                            idle_rid_tracks,
                            now_ts=curr_time,
                        )
                        idle_has_valid_sort = bool(idle_sort_assoc_items)
                        if (
                            idle_rid_tracks
                            and not idle_has_valid_sort
                            and FIELD_LOGGER is not None
                            and (curr_time - rid_assoc_last_log_ts)
                            >= RID_ASSOC_LOG_INTERVAL
                        ):
                            rid_assoc_last_log_ts = curr_time
                            rid_assoc_cycle += 1
                            FIELD_LOGGER.write_rid_association({
                                "timestamp": f"{curr_time:.6f}",
                                "event": "RID_ONLY_WAITING_FOR_SORT",
                                "cycle": rid_assoc_cycle,
                                "master_id": "",
                                "sort_count": 0,
                                "rid_count": len(idle_rid_tracks),
                                "binding_count": 0,
                                "max_az_error_deg": f"{RID_ASSOC_MAX_AZ_DEG:.6f}",
                                "max_curve_error_deg": f"{RID_ASSOC_MAX_CURVE_ERROR_DEG:.6f}",
                                "reason": (
                                    "rid_history_kept_no_ui_eligible_sort_track"
                                ),
                                "station_latitude": f"{float(idle_station['latitude']):.8f}",
                                "station_longitude": f"{float(idle_station['longitude']):.8f}",
                                "station_source": idle_station["source"],
                                "station_age_s": f"{idle_station['age_s']:.6f}",
                                "device_heading_deg": f"{DEVICE_HEADING_DEG:.6f}",
                            })
                            for rid_item in idle_rid_tracks:
                                FIELD_LOGGER.write_rid_association({
                                    "timestamp": f"{curr_time:.6f}",
                                    "event": "RID_TRACK",
                                    "cycle": rid_assoc_cycle,
                                    "master_id": "",
                                    "sort_count": 0,
                                    "rid_count": len(idle_rid_tracks),
                                    "binding_count": 0,
                                    "rid_ui_id": rid_item["ui_id"],
                                    "rid_id": rid_item["rid_id"],
                                    "rid_id_type": rid_item["id_type"],
                                    "rid_standard": rid_item["standard"],
                                    "rid_map_az": f"{rid_item['map_az']:.6f}",
                                    "distance_m": f"{rid_item['distance_m']:.6f}",
                                    "rid_age_s": f"{rid_item['age_s']:.6f}",
                                    "rid_update_seq": rid_item["update_seq"],
                                    "rid_measurement_seq": rid_item.get(
                                        "measurement_seq", ""
                                    ),
                                    "rid_timestamp": rid_item.get(
                                        "rid_timestamp", ""
                                    ),
                                    "rid_latitude": f"{rid_item['latitude']:.8f}",
                                    "rid_longitude": f"{rid_item['longitude']:.8f}",
                                    "rid_alt_geo": (
                                        "" if rid_item.get("alt_geo") is None
                                        else f"{rid_item['alt_geo']:.3f}"
                                    ),
                                    "reason": "rid_only_no_sort_track",
                                    "station_latitude": f"{float(idle_station['latitude']):.8f}",
                                    "station_longitude": f"{float(idle_station['longitude']):.8f}",
                                    "station_source": idle_station["source"],
                                    "station_age_s": f"{idle_station['age_s']:.6f}",
                                    "device_heading_deg": f"{DEVICE_HEADING_DEG:.6f}",
                                })
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
                maybe_print_live_status(curr_time, meas_count=None)
                time.sleep(0.005) # 稍微让出 CPU
                continue

            if (curr_time - fusion_window_start_t) < MEAS_FUSION_WINDOW_SECONDS:
                maybe_print_live_status(curr_time, meas_count=None)
                time.sleep(0.005)
                continue

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

            # Legacy weighted/held association and distance-writeback block.
            # It remains unreachable because rid_associator is None; the
            # active four-point association is evaluated below.
            latest_rid_bindings = {}
            rid_diagnostics = []
            rid_tracks = []
            station_snapshot = station_position.snapshot(curr_time)
            sort_assoc_items = []
            if rid_track_manager is not None and rid_associator is not None:
                for track in ui_tracks:
                    source_board, source_cam = track_ui_source(
                        track, board_str, cam_idx
                    )
                    sort_assoc_items.append({
                        "track_id": int(track.id),
                        "created_ts": float(track.created_ts),
                        "relative_az": float(track.state[0, 0]),
                        "map_az": relative_to_map_azimuth(track.state[0, 0]),
                        "el": float(track.state[1, 0]),
                        "lost_seconds": float(track.lost_seconds(curr_time)),
                        "board": source_board,
                        "cam": source_cam,
                        "logic_id": getattr(track, "last_source_logic_id", ""),
                    })

                rid_tracks = rid_track_manager.snapshot(now_ts=curr_time)
                if station_snapshot["valid"]:
                    rid_tracks = enrich_rid_tracks(
                        rid_tracks,
                        station_latitude=station_snapshot["latitude"],
                        station_longitude=station_snapshot["longitude"],
                        station_altitude=station_snapshot["altitude"],
                    )
                    latest_rid_bindings, rid_diagnostics = rid_associator.associate(
                        sort_assoc_items,
                        rid_tracks,
                        now_ts=curr_time,
                    )

                    track_by_id = {int(track.id): track for track in valid_tracks}
                    for sort_id, binding in latest_rid_bindings.items():
                        rid_item = binding["rid"]
                        rid_measurement_seq = int(
                            rid_item.get("measurement_seq", rid_item["update_seq"])
                        )
                        if (
                            last_applied_rid_measurement.get(sort_id)
                            == rid_measurement_seq
                        ):
                            continue
                        track = track_by_id.get(int(sort_id))
                        if track is None:
                            continue
                        track.set_mono_distance(
                            rid_item["distance_m"],
                            curr_time,
                            source="rid_gps",
                        )
                        last_applied_rid_measurement[sort_id] = rid_measurement_seq

                if (
                    station_snapshot["valid"]
                    and
                    FIELD_LOGGER is not None
                    and (curr_time - rid_assoc_last_log_ts) >= RID_ASSOC_LOG_INTERVAL
                ):
                    rid_assoc_last_log_ts = curr_time
                    rid_assoc_cycle += 1
                    station_fields = {
                        "station_latitude": (
                            "" if station_snapshot["latitude"] is None
                            else f"{float(station_snapshot['latitude']):.8f}"
                        ),
                        "station_longitude": (
                            "" if station_snapshot["longitude"] is None
                            else f"{float(station_snapshot['longitude']):.8f}"
                        ),
                        "station_altitude_m": (
                            "" if station_snapshot["altitude"] is None
                            else f"{float(station_snapshot['altitude']):.3f}"
                        ),
                        "station_source": station_snapshot["source"],
                        "station_age_s": (
                            "" if not math.isfinite(station_snapshot["age_s"])
                            else f"{station_snapshot['age_s']:.6f}"
                        ),
                        "device_heading_deg": f"{DEVICE_HEADING_DEG:.6f}",
                    }
                    FIELD_LOGGER.write_rid_association({
                        "timestamp": f"{curr_time:.6f}",
                        "event": "RID_ASSOC_SUMMARY",
                        "cycle": rid_assoc_cycle,
                        "master_id": "" if master_id is None else int(master_id),
                        "sort_count": len(sort_assoc_items),
                        "rid_count": len(rid_tracks),
                        "binding_count": len(latest_rid_bindings),
                        "max_az_error_deg": f"{RID_ASSOC_MAX_AZ_DEG:.6f}",
                        "max_curve_error_deg": f"{RID_ASSOC_MAX_CURVE_ERROR_DEG:.6f}",
                        "reason": (
                            "no_ui_eligible_sort_tracks" if not sort_assoc_items
                            else ("no_rid_tracks" if not rid_tracks else "evaluated")
                        ),
                        **station_fields,
                    })
                    for sort_item in sort_assoc_items:
                        FIELD_LOGGER.write_rid_association({
                            "timestamp": f"{curr_time:.6f}",
                            "event": "SORT_TRACK",
                            "cycle": rid_assoc_cycle,
                            "master_id": "" if master_id is None else int(master_id),
                            "sort_track_id": sort_item["track_id"],
                            "board": sort_item["board"],
                            "cam": sort_item["cam"],
                            "logic_id": sort_item["logic_id"],
                            "sort_relative_az": f"{sort_item['relative_az']:.6f}",
                            "sort_map_az": f"{sort_item['map_az']:.6f}",
                            "sort_el": f"{sort_item['el']:.6f}",
                            "sort_lost_seconds": f"{sort_item['lost_seconds']:.6f}",
                            **station_fields,
                        })
                    for rid_item in rid_tracks:
                        FIELD_LOGGER.write_rid_association({
                            "timestamp": f"{curr_time:.6f}",
                            "event": "RID_TRACK",
                            "cycle": rid_assoc_cycle,
                            "master_id": "" if master_id is None else int(master_id),
                            "rid_ui_id": rid_item["ui_id"],
                            "rid_id": rid_item["rid_id"],
                            "rid_id_type": rid_item["id_type"],
                            "rid_standard": rid_item["standard"],
                            "rid_map_az": f"{rid_item['map_az']:.6f}",
                            "distance_m": f"{rid_item['distance_m']:.6f}",
                            "rid_age_s": f"{rid_item['age_s']:.6f}",
                            "rid_update_seq": rid_item["update_seq"],
                            "rid_measurement_seq": rid_item.get("measurement_seq", ""),
                            "rid_timestamp": rid_item.get("rid_timestamp", ""),
                            "rid_latitude": f"{rid_item['latitude']:.8f}",
                            "rid_longitude": f"{rid_item['longitude']:.8f}",
                            "rid_alt_geo": (
                                "" if rid_item.get("alt_geo") is None
                                else f"{rid_item['alt_geo']:.3f}"
                            ),
                            **station_fields,
                        })
                    sort_log_by_id = {
                        item["track_id"]: item for item in sort_assoc_items
                    }
                    rid_log_by_key = {item["key"]: item for item in rid_tracks}
                    for diagnostic in rid_diagnostics:
                        sort_item = sort_log_by_id[diagnostic["sort_track_id"]]
                        rid_item = rid_log_by_key[diagnostic["rid_key"]]
                        binding = latest_rid_bindings.get(
                            diagnostic["sort_track_id"]
                        )
                        binding_state = ""
                        if (
                            binding is not None
                            and binding["rid"]["key"] == diagnostic["rid_key"]
                        ):
                            binding_state = binding["state"]
                        FIELD_LOGGER.write_rid_association({
                            "timestamp": f"{curr_time:.6f}",
                            "event": "RID_SORT_PAIR",
                            "cycle": rid_assoc_cycle,
                            "master_id": "" if master_id is None else int(master_id),
                            "sort_track_id": diagnostic["sort_track_id"],
                            "board": sort_item["board"],
                            "cam": sort_item["cam"],
                            "logic_id": sort_item["logic_id"],
                            "sort_relative_az": f"{sort_item['relative_az']:.6f}",
                            "sort_map_az": f"{diagnostic['sort_map_az']:.6f}",
                            "sort_el": f"{sort_item['el']:.6f}",
                            "sort_lost_seconds": f"{sort_item['lost_seconds']:.6f}",
                            "rid_ui_id": diagnostic["rid_ui_id"],
                            "rid_id": diagnostic["rid_id"],
                            "rid_id_type": rid_item["id_type"],
                            "rid_standard": rid_item["standard"],
                            "rid_map_az": f"{diagnostic['rid_map_az']:.6f}",
                            "az_error_deg": f"{diagnostic['az_error_deg']:.6f}",
                            "curve_error_deg": f"{diagnostic['curve_error_deg']:.6f}",
                            "shape_error_deg": f"{diagnostic['shape_error_deg']:.6f}",
                            "trend_error_deg": f"{diagnostic['trend_error_deg']:.6f}",
                            "curve_bias_deg": f"{diagnostic['curve_bias_deg']:.6f}",
                            "association_cost_deg": f"{diagnostic['association_cost_deg']:.6f}",
                            "trajectory_samples": diagnostic["trajectory_samples"],
                            "trajectory_ready": 1 if diagnostic["trajectory_ready"] else 0,
                            "max_az_error_deg": f"{diagnostic['max_az_error_deg']:.6f}",
                            "max_curve_error_deg": f"{diagnostic['max_curve_error_deg']:.6f}",
                            "selected": 1 if diagnostic["selected"] else 0,
                            "ambiguous": 1 if diagnostic["ambiguous"] else 0,
                            "binding_state": binding_state,
                            "reason": diagnostic["reason"],
                            "distance_m": f"{diagnostic['distance_m']:.6f}",
                            "rid_age_s": f"{diagnostic['rid_age_s']:.6f}",
                            "rid_update_seq": diagnostic["rid_update_seq"],
                            "rid_measurement_seq": diagnostic["rid_measurement_seq"],
                            "rid_timestamp": rid_item.get("rid_timestamp", ""),
                            "rid_latitude": f"{rid_item['latitude']:.8f}",
                            "rid_longitude": f"{rid_item['longitude']:.8f}",
                            "rid_alt_geo": (
                                "" if rid_item.get("alt_geo") is None
                                else f"{rid_item['alt_geo']:.3f}"
                            ),
                            **station_fields,
                        })
                elif (
                    not station_snapshot["valid"]
                    and (curr_time - rid_assoc_last_log_ts) >= RID_ASSOC_LOG_INTERVAL
                ):
                    rid_assoc_last_log_ts = curr_time
                    rid_assoc_cycle += 1
                    if FIELD_LOGGER is not None:
                        FIELD_LOGGER.write_rid_association({
                            "timestamp": f"{curr_time:.6f}",
                            "event": "RID_ASSOC_SUMMARY",
                            "cycle": rid_assoc_cycle,
                            "master_id": "" if master_id is None else int(master_id),
                            "sort_count": len(sort_assoc_items),
                            "rid_count": len(rid_tracks),
                            "binding_count": 0,
                            "max_az_error_deg": f"{RID_ASSOC_MAX_AZ_DEG:.6f}",
                            "max_curve_error_deg": f"{RID_ASSOC_MAX_CURVE_ERROR_DEG:.6f}",
                            "reason": "station_wgs84_position_unavailable",
                            "station_source": station_snapshot["source"],
                            "device_heading_deg": f"{DEVICE_HEADING_DEG:.6f}",
                        })
                    field_log_event({
                        "timestamp": f"{curr_time:.6f}",
                        "seq": sender_seq,
                        "mode": sender_mode,
                        "event": "RID_ASSOC_SKIP",
                        "reason": "station_wgs84_position_unavailable",
                    })

            # --- 4. 状态机：调度决策 ---
            # Keep SORT and RID trajectories independent. Materialize current
            # RID geometry for four-point matching; a successful match lends
            # only its distance to the SORT-owned UI packet.
            rid_tracks = []
            if rid_track_manager is not None:
                if station_snapshot["valid"]:
                    raw_rid_tracks = rid_track_manager.snapshot(
                        now_ts=curr_time
                    )
                    if rid_trajectory_renderer is not None:
                        raw_rid_tracks = rid_trajectory_renderer.render(
                            raw_rid_tracks,
                            now_ts=curr_time,
                        )
                    rid_tracks = enrich_rid_tracks(
                        raw_rid_tracks,
                        station_latitude=station_snapshot["latitude"],
                        station_longitude=station_snapshot["longitude"],
                        station_altitude=station_snapshot["altitude"],
                    )
                elif (curr_time - rid_assoc_last_log_ts) >= RID_ASSOC_LOG_INTERVAL:
                    rid_assoc_last_log_ts = curr_time
                    field_log_event({
                        "timestamp": f"{curr_time:.6f}",
                        "seq": sender_seq,
                        "mode": sender_mode,
                        "event": "RID_UI_FUSION_SKIP",
                        "reason": "station_wgs84_position_unavailable",
                    })

            master_track = next((t for t in valid_tracks if t.id == master_id), None)
            prev_master_id = master_id
            master_lost = (prev_master_id is not None and master_track is None)

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
                master_id = None
                prev_master_id = None

            curr_gimbal_az = shared_gimbal_az
            curr_gimbal_el = shared_gimbal_el
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
            ui_ids = [int(t.id) for t in ui_tracks]
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
                    "ui_ids": ";".join(str(x) for x in ui_ids),
                    "master_id": "" if master_id is None else int(master_id),
                    "hit_streaks": ";".join(str(x) for x in hit_values),
                    "time_since_updates": ";".join(str(x) for x in lost_values),
                    "lost_seconds": ";".join(lost_seconds_values),
                    "track_states": track_states,
                    "cmd_az": "" if last_sent_ctrl_az is None else f"{last_sent_ctrl_az:.6f}",
                    "cmd_el": "" if last_sent_ctrl_el is None else f"{last_sent_ctrl_el:.6f}",
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
            if vision_service is not None:
                track_predictions = []
                for track in valid_tracks:
                    expected_delta_az = angular_diff(
                        track.state[0, 0],
                        shared_gimbal_az,
                    )
                    expected_delta_el = (
                        track.state[1, 0] - shared_gimbal_el
                    )
                    track_predictions.append({
                        "track_id": int(track.id),
                        "center": (
                            IMG_W / 2.0
                            + expected_delta_az / FOV_X * IMG_W,
                            IMG_H / 2.0
                            - expected_delta_el / FOV_Y * IMG_H,
                        ),
                    })
                vision_service.update_context(
                    master_track_id=master_id,
                    # Distance inference needs a stable image, not necessarily
                    # exact convergence to the requested mechanical angle.
                    gimbal_settled=gimbal_is_stationary,
                    settled_ts=gimbal_stationary_ts,
                    track_predictions=track_predictions,
                )
                vision_result = vision_service.get_result()
                simple_measurements = (
                    vision_result.get("simple_measurements", []) or []
                )
                if simple_measurements:
                    association_max_px = float(
                        vision_result.get(
                            "simple_association_max_px", 180.0
                        )
                        or 180.0
                    )
                    prediction_map = {
                        int(item["track_id"]): np.asarray(
                            item["center"], dtype=np.float32
                        )
                        for item in track_predictions
                        if "track_id" in item and "center" in item
                    }
                    post_assoc_results = {}
                    matched_measurement_indices = set()
                    if prediction_map:
                        predicted_track_ids = list(prediction_map)
                        invalid_cost = association_max_px * 10.0
                        cost_matrix = np.full(
                            (
                                len(predicted_track_ids),
                                len(simple_measurements),
                            ),
                            invalid_cost,
                            dtype=np.float32,
                        )
                        geometric_distances = np.full_like(
                            cost_matrix, np.inf
                        )
                        for track_index, track_id in enumerate(
                            predicted_track_ids
                        ):
                            expected_center = prediction_map[track_id]
                            for measurement_index, measurement in enumerate(
                                simple_measurements
                            ):
                                measured_center = np.asarray(
                                    measurement.get(
                                        "center", [math.nan, math.nan]
                                    ),
                                    dtype=np.float32,
                                )
                                if not np.all(np.isfinite(measured_center)):
                                    continue
                                distance_px = float(
                                    np.linalg.norm(
                                        measured_center - expected_center
                                    )
                                )
                                geometric_distances[
                                    track_index, measurement_index
                                ] = distance_px
                                if distance_px <= association_max_px:
                                    cost_matrix[
                                        track_index, measurement_index
                                    ] = distance_px

                        track_indices, measurement_indices = (
                            linear_sum_assignment(cost_matrix)
                        )
                        for track_index, measurement_index in zip(
                            track_indices, measurement_indices
                        ):
                            distance_px = float(
                                geometric_distances[
                                    track_index, measurement_index
                                ]
                            )
                            if distance_px > association_max_px:
                                continue
                            track_id = predicted_track_ids[track_index]
                            result_item = dict(
                                simple_measurements[measurement_index]
                            )
                            result_item["track_id"] = track_id
                            result_item[
                                "association_error_px"
                            ] = distance_px
                            post_assoc_results[track_id] = result_item
                            matched_measurement_indices.add(
                                measurement_index
                            )

                    unmatched_measurements = [
                        measurement
                        for measurement_index, measurement in enumerate(
                            simple_measurements
                        )
                        if measurement_index
                        not in matched_measurement_indices
                    ]
                    vision_result["track_results"] = post_assoc_results
                    vision_result["matched_track_ids"] = sorted(
                        post_assoc_results
                    )
                    vision_result["matched_count"] = len(
                        post_assoc_results
                    )
                    vision_result["unmatched_track_ids"] = sorted(
                        track_id
                        for track_id in prediction_map
                        if track_id not in post_assoc_results
                    )
                    vision_result[
                        "unmatched_detection_count"
                    ] = len(unmatched_measurements)
                    vision_result[
                        "unmatched_detections"
                    ] = unmatched_measurements[:5]
                track_results = vision_result.get("track_results", {})
                vision_frame_ts = float(vision_result.get("frame_ts", 0.0) or 0.0)
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
                            center_dx_px = bbox_cx - IMG_W / 2.0
                            center_dy_px = bbox_cy - IMG_H / 2.0
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
                track_by_id = {
                    int(track.id): track for track in valid_tracks
                }
                for track_id, track_result in track_results.items():
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
            if master_track is not None:
                fut_az, fut_el = master_track.get_future_position(dt_delay=PREDICT_DELAY)
                ctrl_az, ctrl_el = ui_to_ctrl_angles(fut_az, fut_el)

                vision_frame_age = curr_time - float(
                    vision_result.get("frame_ts", 0.0)
                )
                vision_bbox_fresh = (
                    vision_result.get("track_id") == master_id
                    and vision_result.get("bbox") is not None
                    and 0.0 <= vision_frame_age <= GIMBAL_VISION_RESULT_TTL
                )
                reposition_reason = "none"
                if vision_bbox_fresh:
                    need_reposition = bool(
                        vision_result.get("reposition_requested", False)
                    )
                    angle_unsafe_frames = 0
                    if need_reposition:
                        reposition_reason = "vision_bbox_outside_safe_zone"
                else:
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
                    })
                    last_sent_ctrl_az = ctrl_az
                    last_sent_ctrl_el = ctrl_el
                    field_log_gimbal({
                        "timestamp": f"{curr_time:.6f}",
                        "event": "GIMBAL_CMD",
                        "cmd_id": int(global_cmd_id),
                        "track_id": "" if master_id is None else int(master_id),
                        "cmd_az": f"{ctrl_az:.6f}",
                        "cmd_el": f"{ctrl_el:.6f}",
                        "gimbal_ui_az": f"{shared_gimbal_az:.6f}",
                        "gimbal_ui_el": f"{shared_gimbal_el:.6f}",
                        "target_ctrl_az": f"{ctrl_az:.6f}",
                        "target_ctrl_el": f"{ctrl_el:.6f}",
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


                vision_frame_ts = float(vision_result.get("frame_ts", 0.0) or 0.0)
                vision_result_age = curr_time - vision_frame_ts
                vision_result_fresh = (
                    gimbal_is_stationary
                    and 0.0 <= vision_result_age <= GIMBAL_VISION_RESULT_TTL
                )
                # E. Build identity candidates only from UI-eligible SORT
                # tracks. Four distinct RID updates must pass both learned
                # bias and trajectory-shape gates before one-to-one assignment.
                if rid_track_manager is not None:
                    (
                        matched_ui_fusion_pairs,
                        ui_fusion_diagnostics,
                    ) = pair_ui_tracks_with_rid(
                        ui_tracks,
                        rid_tracks,
                        rid_four_point_associator,
                        now_ts=curr_time,
                    )
                    ui_fusion_pairs = complete_ui_fusion_pairs(
                        ui_tracks, matched_ui_fusion_pairs
                    )
                else:
                    matched_ui_fusion_pairs = []
                    ui_fusion_diagnostics = []
                    ui_fusion_pairs = [
                        {
                            "sort_track": track,
                            "rid": None,
                            "az_error_deg": math.nan,
                        }
                        for track in ui_tracks
                    ]
                selected_rid_keys = {
                    item["rid"]["key"]
                    for item in matched_ui_fusion_pairs
                    if item["rid"] is not None
                }
                if rid_track_manager is not None:
                    for dropped_rid in rid_tracks:
                        if dropped_rid["key"] in selected_rid_keys:
                            continue
                        field_log_event({
                            "timestamp": f"{curr_time:.6f}",
                            "seq": sender_seq,
                            "mode": sender_mode,
                            "event": "RID_UI_FOUR_POINT_SKIP",
                            "ui_id": int(dropped_rid["ui_id"]),
                            "reason": (
                                f"sort_count={len(ui_tracks)},"
                                f"rid_count={len(rid_tracks)},"
                                "no_selected_four_point_binding"
                            ),
                        })

                if (
                    rid_track_manager is not None
                    and FIELD_LOGGER is not None
                    and (curr_time - rid_assoc_last_log_ts)
                    >= RID_ASSOC_LOG_INTERVAL
                ):
                    rid_assoc_last_log_ts = curr_time
                    rid_assoc_cycle += 1
                    station_fields = {
                        "station_latitude": (
                            "" if station_snapshot["latitude"] is None
                            else f"{float(station_snapshot['latitude']):.8f}"
                        ),
                        "station_longitude": (
                            "" if station_snapshot["longitude"] is None
                            else f"{float(station_snapshot['longitude']):.8f}"
                        ),
                        "station_altitude_m": (
                            "" if station_snapshot["altitude"] is None
                            else f"{float(station_snapshot['altitude']):.3f}"
                        ),
                        "station_source": station_snapshot["source"],
                        "station_age_s": (
                            "" if not math.isfinite(station_snapshot["age_s"])
                            else f"{station_snapshot['age_s']:.6f}"
                        ),
                        "device_heading_deg": f"{DEVICE_HEADING_DEG:.6f}",
                    }
                    FIELD_LOGGER.write_rid_association({
                        "timestamp": f"{curr_time:.6f}",
                        "event": "RID_UI_FUSION_SUMMARY",
                        "cycle": rid_assoc_cycle,
                        "master_id": "" if master_id is None else int(master_id),
                        "sort_count": len(ui_tracks),
                        "rid_count": len(rid_tracks),
                        "binding_count": len(matched_ui_fusion_pairs),
                        "max_az_error_deg": (
                            f"{RID_FOUR_POINT_MAX_BIAS_ERROR_DEG:.6f}"
                        ),
                        "max_curve_error_deg": (
                            f"{RID_FOUR_POINT_MAX_SHAPE_P95_DEG:.6f}"
                        ),
                        "reason": "four_point_bias_shape_gated_assignment",
                        **station_fields,
                    })
                    for sort_track in ui_tracks:
                        source_board, source_cam = track_ui_source(
                            sort_track, board_str, cam_idx
                        )
                        FIELD_LOGGER.write_rid_association({
                            "timestamp": f"{curr_time:.6f}",
                            "event": "SORT_TRACK",
                            "cycle": rid_assoc_cycle,
                            "sort_track_id": int(sort_track.id),
                            "board": source_board,
                            "cam": source_cam,
                            "logic_id": getattr(
                                sort_track, "last_source_logic_id", ""
                            ),
                            "sort_relative_az": f"{sort_track.state[0, 0]:.6f}",
                            "sort_map_az": f"{relative_to_map_azimuth(sort_track.state[0, 0]):.6f}",
                            "sort_el": f"{sort_track.state[1, 0]:.6f}",
                            "sort_lost_seconds": f"{sort_track.lost_seconds(curr_time):.6f}",
                            "reason": "ui_eligible_sort_input",
                            **station_fields,
                        })
                    for current_rid in rid_tracks:
                        FIELD_LOGGER.write_rid_association({
                            "timestamp": f"{curr_time:.6f}",
                            "event": "RID_TRACK",
                            "cycle": rid_assoc_cycle,
                            "rid_ui_id": current_rid["ui_id"],
                            "rid_id": current_rid["rid_id"],
                            "rid_id_type": current_rid["id_type"],
                            "rid_standard": current_rid["standard"],
                            "rid_map_az": f"{current_rid['map_az']:.6f}",
                            "rid_elevation_deg": (
                                "" if current_rid.get("elevation_deg") is None
                                else f"{current_rid['elevation_deg']:.6f}"
                            ),
                            "rid_height": (
                                "" if current_rid.get("height") is None
                                else f"{current_rid['height']:.3f}"
                            ),
                            "distance_m": f"{current_rid['distance_m']:.6f}",
                            "rid_age_s": f"{current_rid['age_s']:.6f}",
                            "rid_update_seq": current_rid["update_seq"],
                            "rid_measurement_seq": current_rid.get(
                                "measurement_seq", ""
                            ),
                            "rid_timestamp": current_rid.get("rid_timestamp", ""),
                            "rid_latitude": f"{current_rid['latitude']:.8f}",
                            "rid_longitude": f"{current_rid['longitude']:.8f}",
                            "rid_alt_geo": (
                                "" if current_rid.get("alt_geo") is None
                                else f"{current_rid['alt_geo']:.3f}"
                            ),
                            "rid_raw_latitude": (
                                "" if current_rid.get("rid_raw_latitude") is None
                                else f"{current_rid['rid_raw_latitude']:.8f}"
                            ),
                            "rid_raw_longitude": (
                                "" if current_rid.get("rid_raw_longitude") is None
                                else f"{current_rid['rid_raw_longitude']:.8f}"
                            ),
                            "rid_raw_alt_geo": (
                                "" if current_rid.get("rid_raw_alt_geo") is None
                                else f"{current_rid['rid_raw_alt_geo']:.3f}"
                            ),
                            "rid_render_mode": current_rid.get(
                                "rid_render_mode", "raw"
                            ),
                            "rid_filter_update_mode": current_rid.get(
                                "rid_filter_update_mode", ""
                            ),
                            "rid_render_timestamp": (
                                "" if current_rid.get("rid_render_timestamp") is None
                                else f"{current_rid['rid_render_timestamp']:.6f}"
                            ),
                            "rid_render_delay_s": (
                                "" if current_rid.get("rid_render_delay_s") is None
                                else f"{current_rid['rid_render_delay_s']:.6f}"
                            ),
                            "rid_prediction_age_s": (
                                "" if current_rid.get("rid_prediction_age_s") is None
                                else f"{current_rid['rid_prediction_age_s']:.6f}"
                            ),
                            "vertical_delta_m": (
                                "" if current_rid.get("vertical_delta_m") is None
                                else f"{current_rid['vertical_delta_m']:.3f}"
                            ),
                            "selected": (
                                1 if current_rid["key"] in selected_rid_keys else 0
                            ),
                            "reason": (
                                "selected_after_four_point_gate"
                                if current_rid["key"] in selected_rid_keys
                                else "not_selected_by_four_point_gate"
                            ),
                            **station_fields,
                        })
                    ui_track_by_id = {
                        int(track.id): track for track in ui_tracks
                    }
                    rid_by_key = {
                        current_rid["key"]: current_rid
                        for current_rid in rid_tracks
                    }
                    for diagnostic in ui_fusion_diagnostics:
                        diagnostic_track = ui_track_by_id.get(
                            int(diagnostic["sort_track_id"])
                        )
                        diagnostic_rid = rid_by_key.get(
                            diagnostic["rid_key"]
                        )
                        if diagnostic_track is None or diagnostic_rid is None:
                            continue
                        source_board, source_cam = track_ui_source(
                            diagnostic_track, board_str, cam_idx
                        )
                        FIELD_LOGGER.write_rid_association({
                            "timestamp": f"{curr_time:.6f}",
                            "event": "RID_UI_FOUR_POINT_CANDIDATE",
                            "cycle": rid_assoc_cycle,
                            "sort_track_id": int(diagnostic_track.id),
                            "board": source_board,
                            "cam": source_cam,
                            "logic_id": getattr(
                                diagnostic_track,
                                "last_source_logic_id",
                                "",
                            ),
                            "sort_relative_az": (
                                f"{diagnostic_track.state[0, 0]:.6f}"
                            ),
                            "sort_map_az": (
                                f"{diagnostic['sort_map_az']:.6f}"
                            ),
                            "sort_el": (
                                f"{diagnostic_track.state[1, 0]:.6f}"
                            ),
                            "rid_ui_id": diagnostic["rid_ui_id"],
                            "rid_id": diagnostic["rid_id"],
                            "rid_map_az": (
                                f"{diagnostic['rid_map_az']:.6f}"
                            ),
                            "az_error_deg": (
                                f"{diagnostic['az_error_deg']:.6f}"
                            ),
                            "curve_error_deg": (
                                f"{diagnostic['curve_error_deg']:.6f}"
                            ),
                            "shape_error_deg": (
                                f"{diagnostic['shape_error_deg']:.6f}"
                            ),
                            "trend_error_deg": "0.000000",
                            "curve_bias_deg": (
                                f"{diagnostic['curve_bias_deg']:.6f}"
                            ),
                            "association_cost_deg": (
                                f"{diagnostic['association_cost_deg']:.6f}"
                            ),
                            "trajectory_samples": diagnostic[
                                "trajectory_samples"
                            ],
                            "trajectory_ready": (
                                1 if diagnostic["trajectory_ready"] else 0
                            ),
                            "max_az_error_deg": (
                                f"{diagnostic['max_az_error_deg']:.6f}"
                            ),
                            "max_curve_error_deg": (
                                f"{diagnostic['max_curve_error_deg']:.6f}"
                            ),
                            "selected": 1 if diagnostic["selected"] else 0,
                            "ambiguous": 0,
                            "binding_state": (
                                "four_point"
                                if diagnostic["selected"] else ""
                            ),
                            "reason": diagnostic["reason"],
                            "distance_m": (
                                f"{diagnostic['distance_m']:.6f}"
                            ),
                            "rid_age_s": (
                                f"{diagnostic['rid_age_s']:.6f}"
                            ),
                            "rid_update_seq": diagnostic["rid_update_seq"],
                            "rid_measurement_seq": diagnostic[
                                "rid_measurement_seq"
                            ],
                            **station_fields,
                        })
                    for current_pair in matched_ui_fusion_pairs:
                        pair_track = current_pair["sort_track"]
                        pair_rid = current_pair["rid"]
                        if pair_rid is None:
                            continue
                        source_board, source_cam = track_ui_source(
                            pair_track, board_str, cam_idx
                        )
                        FIELD_LOGGER.write_rid_association({
                            "timestamp": f"{curr_time:.6f}",
                            "event": "RID_UI_FOUR_POINT_PAIR",
                            "cycle": rid_assoc_cycle,
                            "sort_track_id": int(pair_track.id),
                            "board": source_board,
                            "cam": source_cam,
                            "logic_id": getattr(
                                pair_track, "last_source_logic_id", ""
                            ),
                            "sort_relative_az": f"{pair_track.state[0, 0]:.6f}",
                            "sort_map_az": f"{relative_to_map_azimuth(pair_track.state[0, 0]):.6f}",
                            "sort_el": f"{pair_track.state[1, 0]:.6f}",
                            "rid_ui_id": pair_rid["ui_id"],
                            "rid_id": pair_rid["rid_id"],
                            "rid_map_az": f"{pair_rid['map_az']:.6f}",
                            "rid_elevation_deg": (
                                "" if pair_rid.get("elevation_deg") is None
                                else f"{pair_rid['elevation_deg']:.6f}"
                            ),
                            "rid_height": (
                                "" if pair_rid.get("height") is None
                                else f"{pair_rid['height']:.3f}"
                            ),
                            "rid_render_mode": pair_rid.get(
                                "rid_render_mode", "raw"
                            ),
                            "rid_filter_update_mode": pair_rid.get(
                                "rid_filter_update_mode", ""
                            ),
                            "rid_render_timestamp": (
                                "" if pair_rid.get("rid_render_timestamp") is None
                                else f"{pair_rid['rid_render_timestamp']:.6f}"
                            ),
                            "rid_render_delay_s": (
                                "" if pair_rid.get("rid_render_delay_s") is None
                                else f"{pair_rid['rid_render_delay_s']:.6f}"
                            ),
                            "rid_prediction_age_s": (
                                "" if pair_rid.get("rid_prediction_age_s") is None
                                else f"{pair_rid['rid_prediction_age_s']:.6f}"
                            ),
                            "az_error_deg": f"{current_pair['az_error_deg']:.6f}",
                            "curve_error_deg": (
                                f"{current_pair['bias_error_deg']:.6f}"
                            ),
                            "shape_error_deg": (
                                f"{current_pair['shape_p95_deg']:.6f}"
                            ),
                            "curve_bias_deg": (
                                f"{current_pair['curve_bias_deg']:.6f}"
                            ),
                            "association_cost_deg": (
                                f"{current_pair['association_cost_deg']:.6f}"
                            ),
                            "trajectory_samples": current_pair[
                                "trajectory_samples"
                            ],
                            "trajectory_ready": 1,
                            "max_az_error_deg": (
                                f"{RID_FOUR_POINT_MAX_BIAS_ERROR_DEG:.6f}"
                            ),
                            "max_curve_error_deg": (
                                f"{RID_FOUR_POINT_MAX_SHAPE_P95_DEG:.6f}"
                            ),
                            "distance_m": f"{pair_rid['distance_m']:.6f}",
                            "vertical_delta_m": (
                                "" if pair_rid.get("vertical_delta_m") is None
                                else f"{pair_rid['vertical_delta_m']:.3f}"
                            ),
                            "selected": 1,
                            "binding_state": "four_point",
                            "reason": (
                                "four_point_gate_pass_selected"
                                if pair_rid.get("elevation_deg") is not None
                                else "four_point_gate_pass_selected;"
                                "altitude_difference_unavailable"
                            ),
                            **station_fields,
                        })

                for ui_pair in ui_fusion_pairs:
                    t = ui_pair["sort_track"]
                    rid_item = ui_pair["rid"]
                    if rid_item is not None:
                        ui_id = get_or_assign_ui_id(t)
                        matched_ui_values = build_rid_matched_ui_values(
                            t, rid_item, ui_id
                        )
                        send_dist = matched_ui_values["distance"]
                        dist_source = "rid_gps"
                    elif rid_track_manager is not None:
                        matched_ui_values = None
                        send_dist = float("nan")
                        dist_source = "rid_unmatched"
                    else:
                        matched_ui_values = None
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
                    # UI threat is deliberately distance-only in RID mode:
                    # <=100m high, (100m, 300m] medium, >300m low.
                    threat_score = ui_threat_score_from_distance(send_dist)
                    sort_map_az = relative_to_map_azimuth(t.state[0, 0])
                    if matched_ui_values is not None:
                        map_az = matched_ui_values["azimuth"]
                        send_el = matched_ui_values["elevation"]
                        ui_id = matched_ui_values["target_id"]
                    else:
                        map_az = sort_map_az
                        send_el = float(t.state[1, 0])
                        ui_id = get_or_assign_ui_id(t)
                    ui_az_source = "sort_map"
                    source_board, source_cam = track_ui_source(
                        t, board_str, cam_idx
                    )
                    sender.send_status(
                        source_board, source_cam, ui_id,
                        azimuth=map_az,
                        elevation=send_el,
                        distance=send_dist,
                        threat_score=threat_score
                    )
                    rid_id_text = (
                        "" if rid_item is None else rid_item["rid_id"]
                    )
                    rid_binding_text = (
                        "" if rid_item is None else "four_point"
                    )
                    rid_error_text = (
                        "" if rid_item is None
                        else f"{float(ui_pair['az_error_deg']):.6f}"
                    )
                    rid_map_az_text = (
                        "" if rid_item is None
                        else f"{float(rid_item['map_az']):.6f}"
                    )
                    rid_render_text = (
                        "" if rid_item is None
                        else (
                            f"{rid_item.get('rid_render_mode', 'raw')}/"
                            f"{float(rid_item.get('rid_prediction_age_s', 0.0)):.3f}s"
                        )
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
                            f"source={source_board}/{source_cam},"
                            f"rid_id={rid_id_text},"
                            f"rid_binding={rid_binding_text},"
                            f"rid_az_error={rid_error_text},"
                            f"threat_source=distance_tier,"
                            f"ui_az_source={ui_az_source},"
                            f"ui_el_source=sort,"
                            f"ui_id_source=sort,"
                            f"ui_distance_source="
                            f"{'rid' if rid_item is not None else 'sort'},"
                            f"sort_map_az={sort_map_az:.6f},"
                            f"rid_map_az={rid_map_az_text},"
                            f"rid_render={rid_render_text}"
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

    if rid_stop_event is not None:
        rid_stop_event.set()
    if laser_stop_event is not None:
        laser_stop_event.set()
        time.sleep(0.05)
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
