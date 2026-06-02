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
    from gps import DEFAULT_LATITUDE, DEFAULT_LONGITUDE, read_gps_fix
except ImportError:
    DEFAULT_LATITUDE = None
    DEFAULT_LONGITUDE = None
    read_gps_fix = None
# ==========================================
# 配置
# ==========================================
# Keep one codepath and switch only the serial defaults by platform.
def _platform_serial_defaults():
    if os.name == "nt":
        return {
            "gimbal": "COM9",
            "laser": "COM10",
            "imu": "COM11",
            "gps": "COM8",
        }
    return {
        "gimbal": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.4.3:1.0-port0",
        "laser": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.1:1.0-port0",
        "imu": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.4.2:1.0-port0",
        "gps": "/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2:1.0-port0",
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


UI_IP = os.getenv("UI_IP", "192.168.0.200")
# UI_IP="172.28.3.80"
UI_PORT = int(os.getenv("UI_PORT", "9999"))
LOCAL_PORT = int(os.getenv("LOCAL_PORT", "8888"))
ENABLE_UDP_DISTANCE = _env_flag("ENABLE_UDP_DISTANCE", False)  # True: 启用UDP距离输入; False: 不启用UDP距离输入
UDP_DISTANCE_PORT = int(os.getenv("UDP_DISTANCE_PORT", "1234"))
UDP_DISTANCE_TTL = _env_float("UDP_DISTANCE_TTL", 5.0)
GIMBAL_PORT = _serial_port("GIMBAL_PORT", "gimbal")
LASER_PORT = _serial_port("LASER_PORT", "laser")
GPS_PORT = _serial_port("GPS_PORT", "gps")
USE_MOCK_GIMBAL = _env_flag("USE_MOCK_GIMBAL", False)  # True: 使用 mock_gimbal.py; False: 使用真实 GT06Z
USE_MOCK_LASER = _env_flag("USE_MOCK_LASER", False)   # True: 激光通道使用 mono 模拟值; False: 使用真实 SDDM 激光
UI_REAL_LASER_ONLY = _env_flag("UI_REAL_LASER_ONLY", True)  # True: UI距离只发送新鲜真实激光，不使用mono/hold兜底
ENABLE_GPS = _env_flag("ENABLE_GPS", True)
ENABLE_IMU = _env_flag("ENABLE_IMU", False)      # Manual switch: True to enable IMU read/print
IMU_PORT = _serial_port("IMU_PORT", "imu")
IMU_BAUDRATE = 9600
IMU_PRINT_INTERVAL = 0.2
GPS_BAUDRATE = 115200
GPS_FIX_TIMEOUT_SECONDS = 5
GPS_STATUS_INTERVAL = 5.0
GPS_UI_SEND_INTERVAL = 10.0
GPS_DEBUG_RAW = _env_flag("GPS_DEBUG_RAW", False)
DEVICE_HEADING_DEG = _env_float("DEVICE_HEADING_DEG", 180) % 360.0  # 设备自身0度方向的地图方位：北0/东90/南180
GIMBAL_AZ_BASE = 59.3  # 云台水平基准角（UI绝对方位 0° 映射到控制角的基准）
GIMBAL_INIT_EL = 0.0  # 启动时俯仰归位角，目标通常从该方向进入
GIMBAL_CMD_DEADBAND_AZ = 0.20
GIMBAL_CMD_DEADBAND_EL = 0.12
AZ_PREEMPT_DEG = 0.3     # 方位轴抢占阈值，单位：度
EL_PREEMPT_DEG = 0.8      # 俯仰轴抢占阈值，单位：度
GIMBAL_SETTLE_THRESHOLD = 0.3
GIMBAL_SETTLE_TIMEOUT = 2.5
GIMBAL_THREAD_SLEEP = 0.02
GIMBAL_PROGRESS_LOG_INTERVAL = 0.10
LASER_DIST_TTL = 2.0
LASER_UI_HOLD_TTL = 5.0
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

IMG_W = 3840.0
IMG_H = 2160.0
FOV_X = 17.5
FOV_Y = 9.9
DEG_PER_PIXEL_X = FOV_X / IMG_W
DEG_PER_PIXEL_Y = FOV_Y / IMG_H

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
            "timestamp", "seq", "mode", "board", "cam", "meas_idx",
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
            "raw_meas_count", "fused_meas_count", "fusion_groups",
            "meas_count", "track_count",
            "valid_count", "track_ids", "valid_ids", "master_id",
            "hit_streaks", "time_since_updates", "track_states",
            "cmd_az", "cmd_el", "gimbal_ui_az", "gimbal_ui_el",
        ]
        self.events_fields = [
            "timestamp", "seq", "mode", "event", "track_id", "meas_idx",
            "meas_az", "meas_el", "pred_az", "map_az", "pred_el", "cost",
            "dynamic_thresh", "uncertainty", "p_az", "p_el",
            "hit_streak", "time_since_update", "reason",
            "internal_track_id", "ui_id",
            "master_id", "is_master", "laser_track_id",
            "distance", "distance_source", "track_laser_dist",
            "track_laser_ts", "track_laser_age",
            "raw_bbox_x1", "raw_bbox_y1", "raw_bbox_x2", "raw_bbox_y2",
            "clipped_bbox_x1", "clipped_bbox_y1", "clipped_bbox_x2", "clipped_bbox_y2",
            "is_edge_bbox", "visible_ratio",
        ]
        self.gimbal_fields = [
            "timestamp", "event", "cmd_id", "track_id", "cmd_az", "cmd_el",
            "gimbal_ui_az", "gimbal_ui_el", "gimbal_ctrl_az", "gimbal_ctrl_el",
            "target_ctrl_az", "target_ctrl_el", "err_az", "err_el",
            "is_settled", "settle_time", "laser_valid", "laser_dist",
            "laser_source", "laser_ts", "laser_age", "laser_interval",
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

# ==========================================
#  摄像头物理位置配置 (不变)
# ==========================================
# DEVICE_THETA = {
#     1: {"theta_vertical": 0.0, "theta_horizontal": 32.727},
#     2: {"theta_vertical": 0.0, "theta_horizontal": 16.364},
#     3: {"theta_vertical": 0.0, "theta_horizontal": 0.0},
#     4: {"theta_vertical": 0.0, "theta_horizontal": 343.636},
#     5: {"theta_vertical": 0.0, "theta_horizontal": 327.273},
#     6: {"theta_vertical": 9.5, "theta_horizontal": 34.286},
#     7: {"theta_vertical": 9.5, "theta_horizontal": 17.143},
#     8: {"theta_vertical": 9.5, "theta_horizontal": 0.0},
#     9: {"theta_vertical": 9.5, "theta_horizontal": 342.857},
#     10: {"theta_vertical": 9.5, "theta_horizontal": 325.714},
#     11: {"theta_vertical": 19.0, "theta_horizontal": 35.186},
#     12: {"theta_vertical": 19.0, "theta_horizontal": 18.043},
#     13: {"theta_vertical": 19.0, "theta_horizontal": 0.9},
#     14: {"theta_vertical": 19.0, "theta_horizontal": 343.757},
#     15: {"theta_vertical": 19.0, "theta_horizontal": 326.614},
#     16: {"theta_vertical": 28.5, "theta_horizontal": 36.7},
#     17: {"theta_vertical": 28.5, "theta_horizontal": 18.7},
#     18: {"theta_vertical": 28.5, "theta_horizontal": 0.7},
#     19: {"theta_vertical": 28.5, "theta_horizontal": 342.7},
#     20: {"theta_vertical": 28.5, "theta_horizontal": 324.7},
#     21: {"theta_vertical": 38.0, "theta_horizontal": 38.595},
#     22: {"theta_vertical": 38.0, "theta_horizontal": 19.647},
#     23: {"theta_vertical": 38.0, "theta_horizontal": 0.7},
#     24: {"theta_vertical": 38.0, "theta_horizontal": 341.753},
#     25: {"theta_vertical": 38.0, "theta_horizontal": 322.805},
#     26: {"theta_vertical": 47.5, "theta_horizontal": 41.7},
#     27: {"theta_vertical": 47.5, "theta_horizontal": 21.7},
#     28: {"theta_vertical": 47.5, "theta_horizontal": 1.7},
#     29: {"theta_vertical": 47.5, "theta_horizontal": 341.7},
#     30: {"theta_vertical": 47.5, "theta_horizontal": 321.7},
#     31: {"theta_vertical": 57.0, "theta_horizontal": 44.953},
#     32: {"theta_vertical": 57.0, "theta_horizontal": 23.776},
#     33: {"theta_vertical": 57.0, "theta_horizontal": 2.6},
#     34: {"theta_vertical": 57.0, "theta_horizontal": 341.424},
#     35: {"theta_vertical": 57.0, "theta_horizontal": 320.247},
#     36: {"theta_vertical": 66.5, "theta_horizontal": 47.6},
#     37: {"theta_vertical": 66.5, "theta_horizontal": 25.1},
#     38: {"theta_vertical": 66.5, "theta_horizontal": 2.6},
#     39: {"theta_vertical": 66.5, "theta_horizontal": 340.1},
#     40: {"theta_vertical": 66.5, "theta_horizontal": 317.6},
#     41: {"theta_vertical": 76.0, "theta_horizontal": 61.4},
#     42: {"theta_vertical": 76.0, "theta_horizontal": 37.4},
#     43: {"theta_vertical": 76.0, "theta_horizontal": 13.4},
#     44: {"theta_vertical": 76.0, "theta_horizontal": 349.4},
#     45: {"theta_vertical": 76.0, "theta_horizontal": 325.4}
# }

#这是用棋盘格标定出来的
DEVICE_THETA = {
    2: {"theta_vertical": 0.0000, "theta_horizontal": 16.7874},  # Layer 1 cam2
    3: {"theta_vertical": 0.0000, "theta_horizontal": 0.0000},  # Layer 1 cam3
    4: {"theta_vertical": 0.0000, "theta_horizontal": 343.8421},  # Layer 1 cam4
    5: {"theta_vertical": 0.0000, "theta_horizontal": 326.6701},  # Layer 1 cam5
    6: {"theta_vertical": 9.5000, "theta_horizontal": 34.3086},  # Layer 2 cam1
    7: {"theta_vertical": 9.5000, "theta_horizontal": 15.5593},  # Layer 2 cam2
    8: {"theta_vertical": 9.5000, "theta_horizontal": 0.0759},  # Layer 2 cam3
    9: {"theta_vertical": 9.5000, "theta_horizontal": 342.7710},  # Layer 2 cam4
    10: {"theta_vertical": 9.5000, "theta_horizontal": 342.3703},  # Layer 2 cam5
    11: {"theta_vertical": 19.0000, "theta_horizontal": 31.9870},  # Layer 3 cam1
    12: {"theta_vertical": 19.0000, "theta_horizontal": 19.6921},  # Layer 3 cam2
    13: {"theta_vertical": 19.0000, "theta_horizontal": 0.9703},  # Layer 3 cam3
    14: {"theta_vertical": 19.0000, "theta_horizontal": 339.9173},  # Layer 3 cam4
    15: {"theta_vertical": 19.0000, "theta_horizontal": 322.7531},  # Layer 3 cam5
    16: {"theta_vertical": 28.5000, "theta_horizontal": 36.7386},  # Layer 4 cam1
    18: {"theta_vertical": 28.5000, "theta_horizontal": 0.7703},  # Layer 4 cam3
    19: {"theta_vertical": 28.5000, "theta_horizontal": 341.8870},  # Layer 4 cam4
    20: {"theta_vertical": 28.5000, "theta_horizontal": 323.5302},  # Layer 4 cam5
    21: {"theta_vertical": 38.0000, "theta_horizontal": 37.5403},  # Layer 5 cam1
    22: {"theta_vertical": 38.0000, "theta_horizontal": 18.5903},  # Layer 5 cam2
    23: {"theta_vertical": 38.0000, "theta_horizontal": 0.7703},  # Layer 5 cam3
    24: {"theta_vertical": 38.0000, "theta_horizontal": 340.7103},  # Layer 5 cam4
    25: {"theta_vertical": 38.0000, "theta_horizontal": 321.7703},  # Layer 5 cam5
    26: {"theta_vertical": 47.5000, "theta_horizontal": 41.7703},  # Layer 6 cam1
    27: {"theta_vertical": 47.5000, "theta_horizontal": 21.7703},  # Layer 6 cam2
    28: {"theta_vertical": 47.5000, "theta_horizontal": 1.7703},  # Layer 6 cam3
    29: {"theta_vertical": 47.5000, "theta_horizontal": 341.7703},  # Layer 6 cam4
    30: {"theta_vertical": 47.5000, "theta_horizontal": 321.7703},  # Layer 6 cam5
    31: {"theta_vertical": 57.0000, "theta_horizontal": 45.2403},  # Layer 7 cam1
    32: {"theta_vertical": 57.0000, "theta_horizontal": 24.0403},  # Layer 7 cam2
    33: {"theta_vertical": 57.0000, "theta_horizontal": 2.8703},  # Layer 7 cam3
    34: {"theta_vertical": 57.0000, "theta_horizontal": 340.6003},  # Layer 7 cam4
    35: {"theta_vertical": 57.0000, "theta_horizontal": 319.4303},  # Layer 7 cam5
    36: {"theta_vertical": 66.5000, "theta_horizontal": 47.8703},  # Layer 8 cam1
    37: {"theta_vertical": 66.5000, "theta_horizontal": 25.3703},  # Layer 8 cam2
    38: {"theta_vertical": 66.5000, "theta_horizontal": 2.8703},  # Layer 8 cam3
    39: {"theta_vertical": 66.5000, "theta_horizontal": 340.3703},  # Layer 8 cam4
    40: {"theta_vertical": 66.5000, "theta_horizontal": 317.8703},  # Layer 8 cam5
    41: {"theta_vertical": 76.0000, "theta_horizontal": 61.6703},  # Layer 9 cam1
    42: {"theta_vertical": 76.0000, "theta_horizontal": 37.6703},  # Layer 9 cam2
    43: {"theta_vertical": 76.0000, "theta_horizontal": 13.6703},  # Layer 9 cam3
    44: {"theta_vertical": 76.0000, "theta_horizontal": 349.6703},  # Layer 9 cam4
    45: {"theta_vertical": 76.0000, "theta_horizontal": 325.6703},  # Layer 9 cam5
}

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
def get_camera_params(board_id, cam_idx):
    key = (str(board_id), int(cam_idx))
    if key not in HARDWARE_MAP:
        print(f"[Warning] 未知的硬件组合: Board={board_id}, Cam={cam_idx}")
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

    def send_status(self, board_str, camera_id, target_id, azimuth, elevation, distance):
            try:
                if not isinstance(board_str, str):
                    board_str = str(board_str)
                board_bytes = board_str.encode('utf-8')

                packet = struct.pack(
                    '!BB8sIfff',
                    self.MSG_STATUS,
                    int(camera_id),
                    board_bytes,
                    int(target_id),
                    float(azimuth),
                    float(elevation),
                    float(distance)
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
# 3. 激光与网络
# ==========================================
class SharedHardwareState:
    def __init__(self):
        self.lock = threading.Lock()
        self.raw_laser_dist = None
        self.raw_laser_ts = 0.0
        self.valid_laser_dist = None
        self.latest_dist = None
        self.laser_ts = 0.0
        self.gimbal_az = 0.0  # UI坐标系方位角
        self.gimbal_el = 0.0
        self.gimbal_att_ts = 0.0
        self.active_cmd_id = -1
        self.active_track_id = -1
        self.settled_cmd_id = -1
        self.settled_track_id = -1
        self.laser_track_id = -1
        self.settled_ts = 0.0
        self.is_settled = False
        self.udp_distance = None
        self.udp_distance_ts = 0.0
        self.udp_distance_status = ""
        self.udp_distance_addr = ""

shared_state = SharedHardwareState()
gimbal_cmd_queue = queue.Queue(maxsize=1)
packet_queue = deque(maxlen=20)


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


def udp_distance_thread():
    print(f"[DistanceUDP] Listening JSON distance on 0.0.0.0:{UDP_DISTANCE_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    try:
        sock.bind(("0.0.0.0", UDP_DISTANCE_PORT))
    except OSError as e:
        print(f"[DistanceUDP][Fatal] 端口 {UDP_DISTANCE_PORT} 绑定失败: {e}")
        return

    err_decode = 0
    err_json = 0
    err_other = 0

    while True:
        try:
            data, addr = sock.recvfrom(4096)
            recv_ts = time.time()
            text = data.decode("utf-8")
            pkg = json.loads(text)
            if not isinstance(pkg, dict):
                err_json += 1
                if err_json % 50 == 1:
                    print(f"[DistanceUDP][JSONError] count={err_json}, JSON根节点不是对象: {pkg!r}")
                continue

            raw_distance = pkg.get("distance")
            distance = _parse_positive_float(raw_distance)
            status = pkg.get("status", "")
            height = pkg.get("height")
            timestamp = pkg.get("timestamp")

            with shared_state.lock:
                shared_state.udp_distance = distance
                shared_state.udp_distance_ts = recv_ts
                shared_state.udp_distance_status = "" if status is None else str(status)
                shared_state.udp_distance_addr = f"{addr[0]}:{addr[1]}"

            print(
                f"[DistanceUDP] from {addr[0]}:{addr[1]} "
                f"height={height}, distance={raw_distance}, parsed_distance={distance}, "
                f"status={status}, timestamp={timestamp}",
                flush=True,
            )

        except socket.timeout:
            continue

        except UnicodeDecodeError as e:
            err_decode += 1
            if err_decode % 50 == 1:
                print(f"[DistanceUDP][DecodeError] count={err_decode}, err={e}")
            continue

        except json.JSONDecodeError as e:
            err_json += 1
            if err_json % 50 == 1:
                print(f"[DistanceUDP][JSONError] count={err_json}, err={e}, raw={data!r}")
            continue

        except Exception as e:
            err_other += 1
            print(f"[DistanceUDP][Unexpected] count={err_other}, type={type(e).__name__}, err={e}")
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

def read_laser_distance():
    with shared_state.lock:
        dist = shared_state.raw_laser_dist
        dist_ts = shared_state.raw_laser_ts
    if (time.time() - dist_ts) > LASER_DIST_TTL:
        return None
    return _parse_positive_float(dist)


def read_udp_distance():
    if not ENABLE_UDP_DISTANCE:
        return None
    with shared_state.lock:
        dist = shared_state.udp_distance
        dist_ts = shared_state.udp_distance_ts
    if dist is None:
        return None
    if (time.time() - dist_ts) > UDP_DISTANCE_TTL:
        return None
    return dist


def update_mock_laser_distance(dist, ts):
    parsed = _parse_positive_float(dist)
    with shared_state.lock:
        shared_state.raw_laser_dist = parsed
        shared_state.raw_laser_ts = float(ts) if parsed is not None else 0.0


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

    while not stop_event.is_set():
        dist = laser.read_distance()
        if dist is None:
            continue
        now_t = time.time()
        with shared_state.lock:
            shared_state.raw_laser_dist = dist
            shared_state.raw_laser_ts = now_t


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
        [{"box": [x1, y1, x2, y2], "mono_dist": float|None,
          "cam": int|None, "board": str|None}, ...]

    支持格式:
    1) [x1, y1, x2, y2]
    2) [x1, y1, x2, y2, dist]
    3) {"box":[x1,y1,x2,y2], "distance":d}
    4) {"boxes":[[...],[...]], "distances":[...]} (多坐标批量)
    """
    parsed = []

    def append_obj(box, mono_dist=None, cam=None, board=None):
        parsed.append({
            "box": [box[0], box[1], box[2], box[3]],
            "mono_dist": mono_dist,
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
        mono_dist = _parse_positive_float(raw_objs[4]) if len(raw_objs) >= 5 else None
        return [{"box": [raw_objs[0], raw_objs[1], raw_objs[2], raw_objs[3]], "mono_dist": mono_dist, "cam": None, "board": None}]

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
            default_dist = _parse_positive_float(
                obj_item.get(
                    "distance_m",
                    obj_item.get("distance", obj_item.get("dist", obj_item.get("range_m", obj_item.get("range"))))
                )
            )

            # 批量 boxes: {"boxes":[...], "distances":[...]}
            boxes = obj_item.get("boxes", None)
            if isinstance(boxes, list):
                dist_list = obj_item.get("distances", None)
                for i, b in enumerate(boxes):
                    if not isinstance(b, (list, tuple)) or len(b) < 4:
                        continue
                    mono_dist = default_dist
                    if isinstance(dist_list, list) and i < len(dist_list):
                        mono_dist = _parse_positive_float(dist_list[i]) or mono_dist
                    append_obj([b[0], b[1], b[2], b[3]], mono_dist, obj_cam, obj_board)
                continue

            # 单目标 box
            box = obj_item.get("box", None)
            if isinstance(box, (list, tuple)):
                # 兼容 {"box":[[...],[...]], ...}
                if len(box) > 0 and isinstance(box[0], (list, tuple)):
                    for b in box:
                        if isinstance(b, (list, tuple)) and len(b) >= 4:
                            append_obj([b[0], b[1], b[2], b[3]], default_dist, obj_cam, obj_board)
                elif len(box) >= 4:
                    append_obj([box[0], box[1], box[2], box[3]], default_dist, obj_cam, obj_board)
                continue

            # 兼容坐标键值形式
            if all(k in obj_item for k in ("x1", "y1", "x2", "y2")):
                append_obj(
                    [obj_item["x1"], obj_item["y1"], obj_item["x2"], obj_item["y2"]],
                    default_dist,
                    obj_cam,
                    obj_board,
                )
                continue
            if all(k in obj_item for k in ("x", "y", "w", "h")):
                x = float(obj_item["x"])
                y = float(obj_item["y"])
                w = float(obj_item["w"])
                h = float(obj_item["h"])
                append_obj([x, y, x + w, y + h], default_dist, obj_cam, obj_board)
                continue

        elif isinstance(obj_item, (list, tuple)):
            # 单目标: [x1, y1, x2, y2, (optional)dist]
            if len(obj_item) >= 4 and not isinstance(obj_item[0], (list, tuple, dict)):
                mono_dist = _parse_positive_float(obj_item[4]) if len(obj_item) >= 5 else None
                append_obj([obj_item[0], obj_item[1], obj_item[2], obj_item[3]], mono_dist)
                continue

            # 批量: [[x1,y1,x2,y2], [..], ...]
            if len(obj_item) > 0 and isinstance(obj_item[0], (list, tuple)):
                for b in obj_item:
                    if isinstance(b, (list, tuple)) and len(b) >= 4:
                        mono_dist = _parse_positive_float(b[4]) if len(b) >= 5 else None
                        append_obj([b[0], b[1], b[2], b[3]], mono_dist)
                continue

    return parsed


def sanitize_bbox(rect):
    try:
        x1, y1, x2, y2 = (float(rect[0]), float(rect[1]), float(rect[2]), float(rect[3]))
    except (TypeError, ValueError, IndexError):
        return None, "non_numeric_bbox"

    raw_w = x2 - x1
    raw_h = y2 - y1
    if raw_w <= 0 or raw_h <= 0:
        return None, "invalid_raw_bbox"

    clipped_x1 = min(max(x1, 0.0), IMG_W)
    clipped_y1 = min(max(y1, 0.0), IMG_H)
    clipped_x2 = min(max(x2, 0.0), IMG_W)
    clipped_y2 = min(max(y2, 0.0), IMG_H)
    clipped_w = clipped_x2 - clipped_x1
    clipped_h = clipped_y2 - clipped_y1
    if clipped_w <= 0 or clipped_h <= 0:
        return None, "invalid_clipped_bbox"

    raw_area = raw_w * raw_h
    clipped_area = clipped_w * clipped_h
    is_edge_bbox = (x1 < 0.0 or y1 < 0.0 or x2 > IMG_W or y2 > IMG_H)
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


def select_track_distance(track, master_id, curr_time):
    udp_distance = read_udp_distance()
    if udp_distance is not None:
        return udp_distance, "udp_distance"

    fresh_laser = (
        track.last_laser_dist
        if (track.last_laser_dist is not None and (curr_time - track.laser_ts) <= LASER_DIST_TTL)
        else None
    )
    held_laser = (
        track.last_laser_dist
        if (track.last_laser_dist is not None and (curr_time - track.laser_ts) <= LASER_UI_HOLD_TTL)
        else None
    )
    fresh_mono = (
        track.last_mono_dist
        if (track.last_mono_dist is not None and (curr_time - track.mono_ts) <= MONO_DIST_TTL)
        else None
    )

    if fresh_laser is not None:
        return fresh_laser, "laser"
    if held_laser is not None:
        return held_laser, "laser_hold"
    if fresh_mono is not None:
        return fresh_mono, "mono"
    if track.last_mono_dist is not None:
        return track.last_mono_dist, "mono_history"
    if track.last_sent_dist is not None:
        return track.last_sent_dist, "mono_sent_history"
    return float("nan"), "none"
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


def build_fused_measurement(items):
    az_values = [float(item["az"]) for item in items]
    el_values = [float(item["el"]) for item in items]
    mono_values = []
    for item in items:
        mono_dist = _parse_positive_float(item.get("mono_dist"))
        if mono_dist is not None:
            mono_values.append(mono_dist)
    fused = {
        "az": circular_mean_deg(az_values),
        "el": sum(el_values) / len(el_values),
        "mono_dist": (sum(mono_values) / len(mono_values)) if mono_values else None,
        "source_meas_indices": [int(item.get("raw_meas_idx", idx)) for idx, item in enumerate(items)],
        "source_cams": [str(item.get("cam", "")) for item in items],
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
        fused["board"] = items[0].get("board")
        fused["cam"] = items[0].get("cam")
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
            f"{item.get('board', '')}/{item.get('cam', '')}"
            for item in group
        )
        parts.append(f"{idx}:n={len(group)},raw={raw_indices},src={sources}")
    return ";".join(parts)


def relative_to_map_azimuth(relative_az, device_heading_deg=DEVICE_HEADING_DEG):
    """将设备自身坐标系方位角转换为正北为0度的地图绝对方位角。"""
    return (float(relative_az) + float(device_heading_deg)) % 360.0


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
                        curr_ui_az = (curr_az - GIMBAL_AZ_BASE) % 360.0
                        with shared_state.lock:
                            shared_state.gimbal_el = curr_el
                            shared_state.gimbal_az = curr_ui_az
                            shared_state.gimbal_att_ts = time.time()
                    continue
                #若成功取到指令，下发指令到云台
                target_az = float(active_cmd["az"])#方位角
                target_el = float(active_cmd["el"])#俯仰角
                cmd_start_t = time.time()
                last_progress_log_t = 0.0
                gimbal.set_attitude(elevation=target_el, azimuth=target_az)
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
                    gimbal.set_attitude(elevation=target_el, azimuth=target_az)
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

            real_att = gimbal.get_attitude()
            if real_att:
                curr_el, curr_az, _ = real_att
                curr_ui_az = (curr_az - GIMBAL_AZ_BASE) % 360.0
                err_az = abs(angular_diff(target_az, curr_az))
                err_el = abs(curr_el - target_el)

                with shared_state.lock:
                    shared_state.gimbal_el = curr_el
                    shared_state.gimbal_az = curr_ui_az
                    shared_state.gimbal_att_ts = now_t

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
                        "gimbal_ui_el": f"{curr_el:.6f}",
                        "gimbal_ctrl_az": f"{curr_az:.6f}",
                        "gimbal_ctrl_el": f"{curr_el:.6f}",
                        "target_ctrl_az": f"{target_az:.6f}",
                        "target_ctrl_el": f"{target_el:.6f}",
                        "err_az": f"{err_az:.6f}",
                        "err_el": f"{err_el:.6f}",
                    })
                    last_progress_log_t = now_t

                if err_az < GIMBAL_SETTLE_THRESHOLD and err_el < GIMBAL_SETTLE_THRESHOLD:
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
                        "gimbal_ui_el": f"{curr_el:.6f}",
                        "gimbal_ctrl_az": f"{curr_az:.6f}",
                        "gimbal_ctrl_el": f"{curr_el:.6f}",
                        "target_ctrl_az": f"{target_az:.6f}",
                        "target_ctrl_el": f"{target_el:.6f}",
                        "err_az": f"{err_az:.6f}",
                        "err_el": f"{err_el:.6f}",
                        "is_settled": 1,
                        "settle_time": f"{settle_dt:.6f}",
                    })

                    laser_dist = read_laser_distance()
                    laser_source = "mock_mono" if USE_MOCK_LASER else "sddm"
                    if laser_dist is not None:
                        trigger_t = time.time()
                        with shared_state.lock:
                            prev_laser_ts = shared_state.laser_ts
                            raw_laser_ts = shared_state.raw_laser_ts
                            shared_state.valid_laser_dist = laser_dist
                            shared_state.latest_dist = laser_dist
                            shared_state.laser_ts = trigger_t
                            shared_state.laser_track_id = shared_state.active_track_id
                            active_cmd_id = shared_state.active_cmd_id
                            active_track_id = shared_state.active_track_id
                        dt_laser = trigger_t - prev_laser_ts if prev_laser_ts > 0 else None
                        field_log_gimbal({
                            "timestamp": f"{trigger_t:.6f}",
                            "event": "LASER_TRIGGER",
                            "cmd_id": int(active_cmd_id),
                            "track_id": int(active_track_id),
                            "gimbal_ui_az": f"{curr_ui_az:.6f}",
                            "gimbal_ui_el": f"{curr_el:.6f}",
                            "gimbal_ctrl_az": f"{curr_az:.6f}",
                            "gimbal_ctrl_el": f"{curr_el:.6f}",
                            "target_ctrl_az": f"{target_az:.6f}",
                            "target_ctrl_el": f"{target_el:.6f}",
                            "err_az": f"{err_az:.6f}",
                            "err_el": f"{err_el:.6f}",
                            "is_settled": 1,
                            "settle_time": f"{settle_dt:.6f}",
                            "laser_valid": 1,
                            "laser_dist": f"{laser_dist:.6f}",
                            "laser_source": laser_source,
                            "laser_ts": f"{raw_laser_ts:.6f}",
                            "laser_age": f"{trigger_t - raw_laser_ts:.6f}" if raw_laser_ts > 0 else "",
                            "laser_interval": "" if dt_laser is None else f"{dt_laser:.6f}",
                        })
                        if prev_laser_ts > 0:
                            if PRINT_EVENT_LOGS:
                                print(
                                    f"[Laser] Triggered cmd_id={active_cmd_id}, track_id={active_track_id}, "
                                    f"dist={laser_dist:.2f}m, interval={dt_laser:.3f}s"
                                )
                        else:
                            if PRINT_EVENT_LOGS:
                                print(
                                    f"[Laser] Triggered cmd_id={active_cmd_id}, track_id={active_track_id}, "
                                    f"dist={laser_dist:.2f}m, interval=first"
                                )
                    else:
                        trigger_t = time.time()
                        field_log_gimbal({
                            "timestamp": f"{trigger_t:.6f}",
                            "event": "LASER_TRIGGER",
                            "cmd_id": int(active_cmd_id),
                            "track_id": int(active_track_id),
                            "gimbal_ui_az": f"{curr_ui_az:.6f}",
                            "gimbal_ui_el": f"{curr_el:.6f}",
                            "gimbal_ctrl_az": f"{curr_az:.6f}",
                            "gimbal_ctrl_el": f"{curr_el:.6f}",
                            "target_ctrl_az": f"{target_az:.6f}",
                            "target_ctrl_el": f"{target_el:.6f}",
                            "err_az": f"{err_az:.6f}",
                            "err_el": f"{err_el:.6f}",
                            "is_settled": 1,
                            "settle_time": f"{settle_dt:.6f}",
                            "laser_valid": 0,
                            "laser_source": laser_source,
                        })
                        if PRINT_EVENT_LOGS:
                            print("[Laser] No valid laser distance, use mono distance")
                    active_cmd = None
                    continue

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
                    "settle_time": f"{elapsed:.6f}",
                })
                with shared_state.lock:
                    shared_state.is_settled = False
                active_cmd = None
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
    ctrl_el = ui_el
    if ctrl_az < 0.0: ctrl_az = 0.0
    if ctrl_az > 350.0: ctrl_az = 350.0
    return ctrl_az, ctrl_el

# ==========================================
# 新增模块 2：单目标卡尔曼追踪器 (AngleTracker)
# ==========================================
class StandardKalmanTrack:
    _id_count = 0
    def __init__(self, ui_az, ui_el):
        StandardKalmanTrack._id_count += 1
        self.id = StandardKalmanTrack._id_count
        
        # 状态矩阵: X = [[az], [el], [v_az], [v_el]]
        self.state = np.array([[ui_az], [ui_el], [0.0], [0.0]], dtype=float)
        
        # 协方差矩阵 P
        self.P = np.diag([1.0, 1.0, 10.0, 10.0])
        
        # 过程噪声 / 测量噪声默认值
        self.q_pos = 0.05
        self.q_vel = 0.2
        self.r_az = 3.5**2
        self.r_el = 1.8**2
        
        self.hit_streak = 1        # 连续命中次数 (用于建轨确认)
        self.time_since_update = 0 # 连丢次数
        
        # 历史队列
        self.history = deque(maxlen=30)
        self.history.append((self.state.copy(), self.P.copy()))
        
        self.max_vel_az = 40.0
        self.max_vel_el = 15.0
        self.min_dt = 0.001
        self.dist_thresh = 4.0
        self.last_mono_dist = None
        self.mono_ts = 0.0
        self.last_laser_dist = None
        self.laser_ts = 0.0
        self.last_sent_dist = None

    def set_mono_distance(self, dist, ts):
        d = _parse_positive_float(dist)
        if d is None:
            return
        self.last_mono_dist = d
        self.mono_ts = float(ts)

    def set_laser_distance(self, dist, ts):
        d = _parse_positive_float(dist)
        if d is None:
            return
        self.last_laser_dist = d
        self.laser_ts = float(ts)

    def get_param_distance(self, curr_time):
        if self.last_laser_dist is not None and (curr_time - self.laser_ts) <= LASER_DIST_TTL:
            return self.last_laser_dist
        if self.last_mono_dist is not None and (curr_time - self.mono_ts) <= MONO_DIST_TTL:
            return self.last_mono_dist
        if self.last_mono_dist is not None:
            return self.last_mono_dist
        if self.last_laser_dist is not None:
            return self.last_laser_dist
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
        """标准 Kalman Predict"""
        if dt < self.min_dt:
            dt = self.min_dt
            
        # 状态转移矩阵 F
        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1]
        ], dtype=float)
        
        # 过程噪声 Q
        Q = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel])
        
        # 预测状态和协方差
        self.state = np.dot(F, self.state)
        # 防止角度跳变，对 Azimuth 进行取模
        self.state[0, 0] = self.state[0, 0] % 360.0
        self.P = np.dot(np.dot(F, self.P), F.T) + Q
        
        self.time_since_update += 1#若之后有轨迹匹配上就会清零time_since_update,判断目标是否丢失。
        self.history.append((self.state.copy(), self.P.copy()))

    def update(self, meas_az, meas_el, dt):
        """标准 Kalman Update"""
        self.time_since_update = 0
        self.hit_streak += 1

        Z = np.array([[meas_az], [meas_el]], dtype=float)
        
        # 观测矩阵 H
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)
        
        # 观测测量噪声 R
        R = np.diag([self.r_az, self.r_el])
        
        # 计算残差 Y = Z - HX(测量位置-预测位置)
        Y = Z - np.dot(H, self.state)
        
        # 核心：处理 Azimuth 的残差，防止角度回环跳变
        Y[0, 0] = angular_diff(Z[0, 0], self.state[0, 0])
        
        # S = H * P * H^T + R,S = 预测位置的不确定性 + 测量位置的不确定性
        S = np.dot(np.dot(H, self.P), H.T) + R
        
        # 卡尔曼增益 K = P * H^T * S^-1,根据 P 和 R 决定这次信检测多少(对预测值的修正力度)
        try:
            K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        except np.linalg.LinAlgError:
            print(f"[Tracker] Kalman matrix inversion failed for track {self.id}")
            K = np.zeros((4, 2), dtype=float)

        # X = X + K * Y
        self.state = self.state + np.dot(K, Y)
        # 更新后再次对 Az 取模
        self.state[0, 0] = self.state[0, 0] % 360.0
        
        # 速度限幅保护,目的就是防止检测框跳变、错匹配、噪声导致速度估计瞬间爆掉
        self.state[2, 0] = np.clip(self.state[2, 0], -self.max_vel_az, self.max_vel_az)
        self.state[3, 0] = np.clip(self.state[3, 0], -self.max_vel_el, self.max_vel_el)
        
        # P = (I - K * H) * P
        I = np.eye(4)
        self.P = np.dot((I - np.dot(K, H)), self.P)
        
        # 更新历史
        if len(self.history) > 0:
            self.history[-1] = (self.state.copy(), self.P.copy())
        else:
            self.history.append((self.state.copy(), self.P.copy()))

    def get_future_position(self, dt_delay):
        """打提前量：获取未来预测角度"""
        fut_az = (self.state[0, 0] + self.state[2, 0] * dt_delay) % 360.0
        fut_el = self.state[1, 0] + self.state[3, 0] * dt_delay
        return fut_az, fut_el
        
    def predict_future_n_steps(self, n=10, dt=0.066):
        """多帧预测接口"""
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
        base_distance_threshold=4.0,
        distance_threshold=None,
    ):
        self.tracks = []
        self.max_lost_frames = max_lost_frames
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
        })

    def _prune_lost_tracks(self, debug_context=None):
        kept_tracks = []
        for track in self.tracks:
            if track.time_since_update < self.max_lost_frames:
                kept_tracks.append(track)
                continue
            if DEBUG_KALMAN_MATCH:
                print(
                    f"[TRACK_DELETE] track={track.id}, "
                    f"lost={track.time_since_update}, max_lost={self.max_lost_frames}, "
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
            elif isinstance(meas, (list, tuple)) and len(meas) >= 2:
                az = meas[0]
                el = meas[1]
                mono_dist = meas[2] if len(meas) >= 3 else None
            else:
                continue

            try:
                az = float(az) % 360.0
                el = float(el)
            except (TypeError, ValueError):
                continue
            mono_dist = _parse_positive_float(mono_dist)
            normalized_measurements.append({
                "az": az,
                "el": el,
                "mono_dist": mono_dist,
            })

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
            self._prune_lost_tracks(debug_context=debug_context)
            return self.tracks

        if len(self.tracks) == 0:
            # 全是新目标,新建轨迹
            for m_idx, meas in enumerate(normalized_measurements):
                t = StandardKalmanTrack(meas["az"], meas["el"])
                t.set_mono_distance(meas["mono_dist"], now_t)
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

        # 3. 匈牙利匹配
        track_indices, meas_indices = linear_sum_assignment(cost_matrix)

        # 4. 更新匹配成功的 Track (加入协方差动态门限)
        unmatched_measurements = set(range(len(normalized_measurements)))
        matched_tracks = set()
        for t_idx, m_idx in zip(track_indices, meas_indices):
            track = self.tracks[t_idx]
            
            # 使用协方差评估不确定性
            uncertainty = np.sqrt(track.P[0, 0] + track.P[1, 1])
            # 动态欧氏门限：基础残差 + 协方差不确定性 * 膨胀系数(1.5)
            dynamic_thresh = track.dist_thresh + (uncertainty * 1.5)
            cost = cost_matrix[t_idx, m_idx]
            meas = normalized_measurements[m_idx]
            pred_az = float(track.state[0, 0])
            pred_el = float(track.state[1, 0])
            
            if cost < dynamic_thresh:
                if DEBUG_KALMAN_MATCH:
                    print(
                        f"[MATCH_ACCEPT] track={track.id}, meas={m_idx}, "
                        f"meas=(Az={meas['az']:.2f}, El={meas['el']:.2f}), "
                        f"pred=(Az={pred_az:.2f}, El={pred_el:.2f}), "
                        f"cost={cost:.2f}, thresh={dynamic_thresh:.2f}, "
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
                })
                track.update(meas["az"], meas["el"], dt)
                track.set_mono_distance(meas["mono_dist"], now_t)
                unmatched_measurements.discard(m_idx)
                matched_tracks.add(t_idx)
            else:
                if DEBUG_KALMAN_MATCH:
                    print(
                        f"[MATCH_REJECT] track={track.id}, meas={m_idx}, "
                        f"meas=(Az={meas['az']:.2f}, El={meas['el']:.2f}), "
                        f"pred=(Az={pred_az:.2f}, El={pred_el:.2f}), "
                        f"cost={cost:.2f}, thresh={dynamic_thresh:.2f}, "
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
                })

        # 未匹配轨迹衰减稳定帧，避免历史累计导致“永久霸榜”
        for t_idx, track in enumerate(self.tracks):
            if t_idx not in matched_tracks:
                track.hit_streak = max(0, track.hit_streak - HIT_STREAK_DECAY)

        # 5. 为没匹配上的坐标创建新 Track
        for m_idx in unmatched_measurements:
            meas = normalized_measurements[m_idx]
            t = StandardKalmanTrack(meas["az"], meas["el"])
            t.set_mono_distance(meas["mono_dist"], now_t)
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
        self._prune_lost_tracks(debug_context=debug_context)

        return self.tracks

# ==========================================
# 4. 核心解算 V8 (修改版：基准水平90度)
# ==========================================
def calculate_angles(cam_key, cx, cy, cfg=None):
    base_az = cfg["theta_horizontal"] 
    base_el = cfg["theta_vertical"]   
    
    # 1. 计算目标在图像中的像素偏移
    diff_x = cx - (IMG_W / 2.0)
    diff_y = cy - (IMG_H / 2.0)
    
    # 2. 像素转换成角度偏移
    offset_az = diff_x * DEG_PER_PIXEL_X
    offset_el = -diff_y * DEG_PER_PIXEL_Y 

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
    print(f"[Config] DEVICE_HEADING_DEG={DEVICE_HEADING_DEG:.2f} (map north=0, east=90, south=180)")
    print(f"[Config] UI_REAL_LASER_ONLY={UI_REAL_LASER_ONLY}")
    if UI_REAL_LASER_ONLY and USE_MOCK_LASER:
        print("[Config][Warn] UI_REAL_LASER_ONLY=True but USE_MOCK_LASER=True; UI距离将发送NaN，不适合真实激光测试。")
    if not USE_MOCK_GIMBAL:
        _validate_serial_port("GIMBAL_PORT", GIMBAL_PORT)
    if not USE_MOCK_LASER:
        _validate_serial_port("LASER_PORT", LASER_PORT)
    if ENABLE_GPS:
        _validate_serial_port("GPS_PORT", GPS_PORT)
    if ENABLE_IMU:
        _validate_serial_port("IMU_PORT", IMU_PORT)

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
    if USE_MOCK_LASER:
        print("[Init] Laser source: mock mono distance")
    else:
        if SDDMLaser is None:
            print("[Laser][Warn] sddm_laser.py import failed, fallback to mono distance only.")
        else:
            try:
                laser = SDDMLaser(LASER_PORT)
                laser_stop_event = threading.Event()
                threading.Thread(
                    target=laser_reader_thread,
                    args=(laser, laser_stop_event),
                    daemon=True,
                ).start()
                print(f"[Init] Real laser enabled on {LASER_PORT}")
            except Exception as e:
                print(f"[Laser][Warn] Init failed on {LASER_PORT}: {e}")
                laser = None

    if ENABLE_UDP_DISTANCE:
        print(
            f"[Init] UI distance priority: UDP JSON distance on port {UDP_DISTANCE_PORT} "
            f"(ttl={UDP_DISTANCE_TTL:.1f}s) -> laser -> mono"
        )
    else:
        print("[Init] UI distance priority: laser -> mono; UDP distance disabled")

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
    if ENABLE_UDP_DISTANCE:
        threading.Thread(target=udp_distance_thread, daemon=True).start()

    imu = None
    last_imu_print = 0.0
    if ENABLE_IMU:
        try:
            from hwt905_driver import HWT905
            imu = HWT905(IMU_PORT, IMU_BAUDRATE)
            imu.open()
            print(f"[IMU] Enabled on {IMU_PORT}@{IMU_BAUDRATE}")
        except Exception as e:
            print(f"[IMU] Init failed: {e}")
            imu = None
    
    print("=== System V9.0 (Predictive Tracking & Scheduling) Running ===")

    # 初始化追踪大脑
    tracker = MultiTargetTracker(max_lost_frames=50, distance_threshold=1.2)
    
    # 状态机与调度变量
    master_id = None
    lock_timer = 0.0
    LOCK_DURATION = 1.5      # 锁定目标的最长驻留时间
    PREDICT_DELAY = 0.3     # 系统与物理响应总延迟 (打提前量)
    CONFIRM_HITS = 3         # 连续追踪多少帧才确认为合法目标
    MAX_DT = 0.25            # Clamp dt to avoid model divergence
    global_cmd_id = 0
    last_sent_ctrl_az = None
    last_sent_ctrl_el = None
    
    last_time = time.time()
    # 统计日志：接收坐标与UI发送ID
    recv_obj_total = 0
    recv_unique_boxes = set()  # {(x1,y1,x2,y2), ...}
    ui_send_total = 0
    ui_send_counter = Counter()  # {ui_id: send_count}
    next_ui_id = 1
    track_to_ui_id = {}  # 只记录已经进入 UI 发送路径的主目标，不给非主目标预分配 UI ID。
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

    while True:
        try:
            curr_time = time.time()
            if imu and (curr_time - last_imu_print) >= IMU_PRINT_INTERVAL:
                acc, gyro, angle = imu.get_all()
                roll, pitch, yaw = angle
                print(f"ANGLE: {roll:6.2f} {pitch:6.2f} {yaw:6.2f}")
                last_imu_print = curr_time

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
                valid_laser_dist = shared_state.valid_laser_dist
                laser_ts = shared_state.laser_ts
                laser_track_id = shared_state.laser_track_id

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
                    bbox_info, bbox_reject_reason = sanitize_bbox(raw_rect)
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

                    res = calculate_angles(logic_id, cx, cy, cfg)
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

            # 将最新激光结果绑定到对应轨迹，避免目标切换时距离串目标
            if (curr_time - laser_ts) <= LASER_DIST_TTL and laser_track_id >= 0:
                laser_track = next((t for t in active_tracks if t.id == laser_track_id), None)
                if laser_track is not None:
                    laser_track.set_laser_distance(valid_laser_dist, laser_ts)
            
            # 过滤出合法的、可以被锁定的目标 (连续追踪超过 CONFIRM_HITS 次的)且没有丢失的目标 (time_since_update == 0)
            valid_tracks = [
                t for t in active_tracks
                if t.hit_streak >= CONFIRM_HITS and t.time_since_update == 0 
            ]

            # --- 4. 状态机：调度决策 ---
            # 检查当前跟踪的目标是否已经丢失
            master_track = next((t for t in valid_tracks if t.id == master_id), None)
            selection_reason = None
            prev_master_id = master_id
            master_lost = (prev_master_id is not None and master_track is None)
            lock_expired = (master_track is not None and lock_timer <= 0)

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
            
            if master_track is None or lock_timer <= 0:
                # 状态 A：寻找/切换新目标 (SEARCHING)
                if master_lost:
                    selection_reason = "master_lost"
                elif lock_expired:
                    selection_reason = "lock_timer_expired"
                else:
                    selection_reason = "initial_acquire"

                curr_gimbal_az = shared_gimbal_az
                curr_gimbal_el = shared_gimbal_el
                best_track, ranked_candidates = choose_master_track(
                    valid_tracks,
                    curr_gimbal_az,
                    curr_gimbal_el,
                    master_id=master_id,
                )

                if best_track:
                    master_id = best_track.id
                    lock_timer = LOCK_DURATION
                    master_track = best_track
                    best_eval = ranked_candidates[0]
                    candidates_text = format_selection_candidates(ranked_candidates)
                    if prev_master_id is None:
                        if PRINT_EVENT_LOGS:
                            print(
                                f"[TargetAcquire] reason={selection_reason}, master_id={master_id}, "
                                f"score={best_eval['threat_score']:.2f}, candidates={candidates_text}"
                            )
                    elif prev_master_id != master_id:
                        if PRINT_EVENT_LOGS:
                            print(
                                f"[TargetSwitch] reason={selection_reason}, from={prev_master_id}, to={master_id}, "
                                f"score={best_eval['threat_score']:.2f}, candidates={candidates_text}"
                            )
                    else:
                        if PRINT_EVENT_LOGS:
                            print(
                                f"[TargetKeep] reason={selection_reason}, master_id={master_id}, "
                                f"score={best_eval['threat_score']:.2f}, candidates={candidates_text}"
                            )
                    field_log_event({
                        "timestamp": f"{curr_time:.6f}",
                        "seq": sender_seq,
                        "mode": sender_mode,
                        "event": (
                            "TargetAcquire" if prev_master_id is None
                            else "TargetSwitch" if prev_master_id != master_id
                            else "TargetKeep"
                        ),
                        "track_id": int(master_id),
                        "reason": selection_reason,
                    })
            track_ids = [int(t.id) for t in active_tracks]
            valid_ids = [int(t.id) for t in valid_tracks]
            hit_values = [int(t.hit_streak) for t in active_tracks]
            lost_values = [int(t.time_since_update) for t in active_tracks]
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
                    f"tracks={len(active_tracks)}, "
                    f"valid={len(valid_tracks)}, "
                    f"ids={track_ids}, "
                    f"valid_ids={valid_ids}, "
                    f"master={master_id}, "
                    f"hits={hit_values}, "
                    f"lost={lost_values}"
                )
            # --- 5. 状态机：物理执行与测距 (LOCKED) ---
            if master_track is not None:
                lock_timer -= dt

                if USE_MOCK_LASER:
                    mock_laser_dist = (
                        master_track.last_mono_dist
                        if (master_track.last_mono_dist is not None and (curr_time - master_track.mono_ts) <= MONO_DIST_TTL)
                        else None
                    )
                    update_mock_laser_distance(mock_laser_dist, curr_time)
                
                # A. 提取提前量预测角度
                fut_az, fut_el = master_track.get_future_position(dt_delay=PREDICT_DELAY)
                
                # B. 转换为云台控制角
                ctrl_az, ctrl_el = ui_to_ctrl_angles(fut_az, fut_el)
                if PRINT_PHASE_LOGS:
                    print(f"[Phase 3: 预测控制] 目标当前估算Az={master_track.state[0, 0]:.2f}°, 速度={master_track.state[2, 0]:.2f}°/s")
                    print(f"                   -> 打提前量({PREDICT_DELAY}s后)Az={fut_az:.2f}°, El={fut_el:.2f}° | 下发云台指令: Az={ctrl_az:.2f}°, El={ctrl_el:.2f}°")
                # C. 非阻塞下发：只推送最新控制指令给云台线程
                need_send = True
                if (last_sent_ctrl_az is not None) and (last_sent_ctrl_el is not None):
                    d_az = abs(angular_diff(ctrl_az, last_sent_ctrl_az))
                    d_el = abs(ctrl_el - last_sent_ctrl_el)
                    if d_az < GIMBAL_CMD_DEADBAND_AZ and d_el < GIMBAL_CMD_DEADBAND_EL:
                        need_send = False

                if need_send:
                    global_cmd_id += 1
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
                    })

                # E. 向 UI 发送数据包：只发送当前正在跟踪的主目标。
                # 追踪器内部仍保留多目标轨迹，供后续目标丢失或锁定超时时切换使用。
                for t in (master_track,):
                    send_dist, dist_source = select_track_distance(t, master_id, curr_time)
                    if math.isfinite(send_dist) and dist_source.startswith("mono"):
                        t.last_sent_dist = send_dist

                    map_az = relative_to_map_azimuth(t.state[0, 0])
                    ui_id = get_or_assign_ui_id(t)
                    sender.send_status(
                        board_str, cam_idx, ui_id,
                        azimuth=map_az,
                        elevation=t.state[1, 0], 
                        distance=send_dist + round(random.uniform(0.0, 1.0), 1) if math.isfinite(send_dist) and dist_source.startswith("mono") else send_dist
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
                        "reason": f"internal_id={int(t.id)},ui_id={int(ui_id)}",
                        "internal_track_id": int(t.id),
                        "ui_id": int(ui_id),
                        "master_id": "" if master_id is None else int(master_id),
                        "is_master": 1 if t.id == master_id else 0,
                        "laser_track_id": int(laser_track_id),
                        "distance": "" if not math.isfinite(send_dist) else f"{send_dist:.6f}",
                        "distance_source": dist_source,
                        "track_laser_dist": "" if t.last_laser_dist is None else f"{t.last_laser_dist:.6f}",
                        "track_laser_ts": "" if t.last_laser_dist is None else f"{t.laser_ts:.6f}",
                        "track_laser_age": "" if t.last_laser_dist is None else f"{curr_time - t.laser_ts:.6f}",
                    })
                    ui_send_total += 1
                    ui_send_counter[int(ui_id)] += 1
            elif USE_MOCK_LASER:
                update_mock_laser_distance(None, curr_time)

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

    if imu:
        imu.close()
    if laser_stop_event is not None:
        laser_stop_event.set()
        time.sleep(0.05)
    if laser is not None:
        laser.close()
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
