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
# ==========================================
# 配置
# ==========================================
def _serial_port_default(role):
    if os.name == "nt":
        defaults = {
            "gimbal": "COM3",
            "laser": "COM4",
            "imu": "COM5",
        }
    else:
        defaults = {
            "gimbal": "/dev/ttyUSB0",
            "laser": "/dev/ttyUSB1",
            "imu": "/dev/ttyUSB2",
        }
    return defaults[role]


def _warn_if_windows_port(label, port_name):
    if os.name == "nt":
        return
    if isinstance(port_name, str) and port_name.upper().startswith("COM"):
        print(
            f"[Config][Warn] {label}={port_name} 看起来是 Windows 串口名。"
            "Linux 下请改为 /dev/ttyUSB* 或 /dev/ttyS*。"
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


def _setup_log_mirror(log_dir):
    global _LOG_MIRROR_INITIALIZED, _LOG_MIRROR_PATH
    if _LOG_MIRROR_INITIALIZED:
        return _LOG_MIRROR_PATH

    os.makedirs(log_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    log_path = os.path.join(log_dir, f"main_tracking_v9_{timestamp}.log")
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


UI_IP = "192.168.2.200"
UI_PORT = 9999
LOCAL_PORT = 8888       
GIMBAL_PORT = os.getenv("GIMBAL_PORT", _serial_port_default("gimbal"))
LASER_PORT = os.getenv("LASER_PORT", _serial_port_default("laser"))
USE_MOCK_GIMBAL = True  # True: 使用 mock_gimbal.py; False: 使用真实 GT06Z
ENABLE_IMU = False      # Manual switch: True to enable IMU read/print
IMU_PORT = os.getenv("IMU_PORT", _serial_port_default("imu"))
IMU_BAUDRATE = 9600
IMU_PRINT_INTERVAL = 0.2
GIMBAL_AZ_BASE = 90.0  # 云台水平基准角（UI绝对方位 0° 映射到控制角的基准）
GIMBAL_CMD_DEADBAND_AZ = 0.20
GIMBAL_CMD_DEADBAND_EL = 0.12
GIMBAL_PREEMPT_DEG = 0.7  # 新指令与当前指令的最小抢占角度差，单位：度
GIMBAL_SETTLE_THRESHOLD = 0.8
GIMBAL_SETTLE_TIMEOUT = 1.0
GIMBAL_THREAD_SLEEP = 0.02
LASER_DIST_TTL = 2.0
MONO_DIST_TTL = 1.2
DEFAULT_TRACKING_DISTANCE_M = 450.0  # 仅用于内部参数冷启动（保持稳定）
DEFAULT_DISTANCE_MIN_M = 200.0
DEFAULT_DISTANCE_MAX_M = 470.0
HIT_STREAK_DECAY = 1
STABILITY_HIT_CAP = 20
STABILITY_WEIGHT = 0.8
STATS_PRINT_INTERVAL = 2.0
LOG_TO_FILE = _env_flag("LOG_TO_FILE", True)
LOG_DIR = os.getenv("LOG_DIR", "logs")

IMG_W = 3840.0
IMG_H = 2160.0
FOV_X = 17.5
FOV_Y = 9.9
DEG_PER_PIXEL_X = FOV_X / IMG_W
DEG_PER_PIXEL_Y = FOV_Y / IMG_H

# ==========================================
#  摄像头物理位置配置 (不变)
# ==========================================
DEVICE_THETA = {
    1: {"theta_vertical": 0.0, "theta_horizontal": 32.727},
    2: {"theta_vertical": 0.0, "theta_horizontal": 16.364},
    3: {"theta_vertical": 0.0, "theta_horizontal": 0.0},
    4: {"theta_vertical": 0.0, "theta_horizontal": 343.636},
    5: {"theta_vertical": 0.0, "theta_horizontal": 327.273},
    6: {"theta_vertical": 9.5, "theta_horizontal": 34.286},
    7: {"theta_vertical": 9.5, "theta_horizontal": 17.143},
    8: {"theta_vertical": 9.5, "theta_horizontal": 0.0},
    9: {"theta_vertical": 9.5, "theta_horizontal": 342.857},
    10: {"theta_vertical": 9.5, "theta_horizontal": 325.714},
    11: {"theta_vertical": 28.5, "theta_horizontal": 36.0},
    12: {"theta_vertical": 28.5, "theta_horizontal": 18.0},
    13: {"theta_vertical": 28.5, "theta_horizontal": 0.0},
    14: {"theta_vertical": 28.5, "theta_horizontal": 342.0},
    15: {"theta_vertical": 28.5, "theta_horizontal": 324.0},
    16: {"theta_vertical": 38.0, "theta_horizontal": 37.895},
    17: {"theta_vertical": 38.0, "theta_horizontal": 18.947},
    18: {"theta_vertical": 38.0, "theta_horizontal": 0.0},
    19: {"theta_vertical": 38.0, "theta_horizontal": 341.053},
    20: {"theta_vertical": 38.0, "theta_horizontal": 322.105},
    21: {"theta_vertical": 47.5, "theta_horizontal": 40.0},
    22: {"theta_vertical": 47.5, "theta_horizontal": 20.0},
    23: {"theta_vertical": 47.5, "theta_horizontal": 0.0},
    24: {"theta_vertical": 47.5, "theta_horizontal": 340.0},
    25: {"theta_vertical": 47.5, "theta_horizontal": 320.0},
    26: {"theta_vertical": 57.0, "theta_horizontal": 42.353},
    27: {"theta_vertical": 57.0, "theta_horizontal": 21.176},
    28: {"theta_vertical": 57.0, "theta_horizontal": 0.0},
    29: {"theta_vertical": 57.0, "theta_horizontal": 338.824},
    30: {"theta_vertical": 57.0, "theta_horizontal": 317.647},
    31: {"theta_vertical": 66.5, "theta_horizontal": 45.0},
    32: {"theta_vertical": 66.5, "theta_horizontal": 22.5},
    33: {"theta_vertical": 66.5, "theta_horizontal": 0.0},
    34: {"theta_vertical": 66.5, "theta_horizontal": 337.5},
    35: {"theta_vertical": 66.5, "theta_horizontal": 315.0},
    36: {"theta_vertical": 76.0, "theta_horizontal": 48.0},
    37: {"theta_vertical": 76.0, "theta_horizontal": 24.0},
    38: {"theta_vertical": 76.0, "theta_horizontal": 0.0},
    39: {"theta_vertical": 76.0, "theta_horizontal": 336.0},
    40: {"theta_vertical": 76.0, "theta_horizontal": 312.0}
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
        self.MSG_STATUS = 0x02 

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
                self.sock.sendto(packet, (self.ip, self.port))
            except Exception as e:
                print(f"[Sender] Error: {e}")

# ==========================================
# 3. 激光与网络
# ==========================================
class SharedHardwareState:
    def __init__(self):
        self.lock = threading.Lock()
        self.raw_laser_dist = None
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
            pkg = json.loads(data.decode("utf-8"))
            if isinstance(pkg, dict) and "objs" in pkg:
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

def read_laser_distance():
    with shared_state.lock:
        dist = shared_state.raw_laser_dist
    return _parse_positive_float(dist)


def _parse_positive_float(value):
    try:
        f = float(value)
        if f > 0:
            return f
    except (TypeError, ValueError):
        pass
    return None


def parse_udp_objects(raw_objs):
    """
    归一化 UDP 目标列表，输出:
        [{"box": [x1, y1, x2, y2], "mono_dist": float|None}, ...]

    支持格式:
    1) [x1, y1, x2, y2]
    2) [x1, y1, x2, y2, dist]
    3) {"box":[x1,y1,x2,y2], "distance":d}
    4) {"boxes":[[...],[...]], "distances":[...]} (多坐标批量)
    """
    parsed = []

    # 兼容单目标扁平格式:
    # objs = [x1, y1, x2, y2] / [x1, y1, x2, y2, dist]
    if isinstance(raw_objs, (list, tuple)) and len(raw_objs) >= 4 and not isinstance(raw_objs[0], (list, tuple, dict)):
        mono_dist = _parse_positive_float(raw_objs[4]) if len(raw_objs) >= 5 else None
        return [{"box": [raw_objs[0], raw_objs[1], raw_objs[2], raw_objs[3]], "mono_dist": mono_dist}]

    # 兼容单目标字典:
    # objs = {"box":[...], "distance":...}
    if isinstance(raw_objs, dict):
        raw_objs = [raw_objs]

    if not isinstance(raw_objs, list):
        return parsed

    for obj_item in raw_objs:
        if isinstance(obj_item, dict):
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
                    parsed.append({"box": [b[0], b[1], b[2], b[3]], "mono_dist": mono_dist})
                continue

            # 单目标 box
            box = obj_item.get("box", None)
            if isinstance(box, (list, tuple)):
                # 兼容 {"box":[[...],[...]], ...}
                if len(box) > 0 and isinstance(box[0], (list, tuple)):
                    for b in box:
                        if isinstance(b, (list, tuple)) and len(b) >= 4:
                            parsed.append({"box": [b[0], b[1], b[2], b[3]], "mono_dist": default_dist})
                elif len(box) >= 4:
                    parsed.append({"box": [box[0], box[1], box[2], box[3]], "mono_dist": default_dist})
                continue

            # 兼容坐标键值形式
            if all(k in obj_item for k in ("x1", "y1", "x2", "y2")):
                parsed.append({
                    "box": [obj_item["x1"], obj_item["y1"], obj_item["x2"], obj_item["y2"]],
                    "mono_dist": default_dist,
                })
                continue
            if all(k in obj_item for k in ("x", "y", "w", "h")):
                x = float(obj_item["x"])
                y = float(obj_item["y"])
                w = float(obj_item["w"])
                h = float(obj_item["h"])
                parsed.append({"box": [x, y, x + w, y + h], "mono_dist": default_dist})
                continue

        elif isinstance(obj_item, (list, tuple)):
            # 单目标: [x1, y1, x2, y2, (optional)dist]
            if len(obj_item) >= 4 and not isinstance(obj_item[0], (list, tuple, dict)):
                mono_dist = _parse_positive_float(obj_item[4]) if len(obj_item) >= 5 else None
                parsed.append({"box": [obj_item[0], obj_item[1], obj_item[2], obj_item[3]], "mono_dist": mono_dist})
                continue

            # 批量: [[x1,y1,x2,y2], [..], ...]
            if len(obj_item) > 0 and isinstance(obj_item[0], (list, tuple)):
                for b in obj_item:
                    if isinstance(b, (list, tuple)) and len(b) >= 4:
                        mono_dist = _parse_positive_float(b[4]) if len(b) >= 5 else None
                        parsed.append({"box": [b[0], b[1], b[2], b[3]], "mono_dist": mono_dist})
                continue

    return parsed


def select_track_distance(track, master_id, curr_time):
    is_master = (track.id == master_id)
    fresh_laser = (
        track.last_laser_dist
        if (track.last_laser_dist is not None and (curr_time - track.laser_ts) <= LASER_DIST_TTL)
        else None
    )
    fresh_mono = (
        track.last_mono_dist
        if (track.last_mono_dist is not None and (curr_time - track.mono_ts) <= MONO_DIST_TTL)
        else None
    )

    if is_master and fresh_laser is not None:
        return fresh_laser, "laser"
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
                gimbal.set_attitude(elevation=target_el, azimuth=target_az)
                with shared_state.lock:
                    shared_state.active_cmd_id = int(active_cmd["cmd_id"])#设置当前执行指令的ID
                    shared_state.active_track_id = int(active_cmd.get("track_id", -1))
                    shared_state.is_settled = False#转动到位标志重置
            #2.若当前有指令在执行(执行态)
            now_t = time.time()
            #检查指令队列中是否有更新的指令，如果有则取出最新的一条（丢弃旧指令），准备进行抢占式执行判断
            newer_cmd = drain_latest_gimbal_cmd()
            if newer_cmd is not None:
                new_az = float(newer_cmd["az"])#新指令的方位角
                new_el = float(newer_cmd["el"])#新指令的俯仰角  
                #计算新指令与当前指令的角度差
                d_az = abs(angular_diff(new_az, target_az))
                d_el = abs(new_el - target_el)
                d_total = math.hypot(d_az, d_el)
                #如果角度差>预设的抢占阈值，则立即执行新指令
                if d_total > GIMBAL_PREEMPT_DEG:
                    target_az = new_az
                    target_el = new_el
                    active_cmd = newer_cmd
                    cmd_start_t = now_t
                    gimbal.set_attitude(elevation=target_el, azimuth=target_az)
                    with shared_state.lock:
                        shared_state.active_cmd_id = int(active_cmd["cmd_id"])
                        shared_state.active_track_id = int(active_cmd.get("track_id", -1))
                        shared_state.is_settled = False
                # 如果角度差<=抢占阈值，视为同一位置，直接丢弃新指令
                else:
                    pass

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

                if err_az < GIMBAL_SETTLE_THRESHOLD and err_el < GIMBAL_SETTLE_THRESHOLD:
                    with shared_state.lock:
                        shared_state.is_settled = True
                        shared_state.settled_cmd_id = shared_state.active_cmd_id
                        shared_state.settled_track_id = shared_state.active_track_id
                        shared_state.settled_ts = now_t

                    laser_dist = read_laser_distance()
                    if laser_dist is not None:
                        trigger_t = time.time()
                        with shared_state.lock:
                            prev_laser_ts = shared_state.laser_ts
                            shared_state.valid_laser_dist = laser_dist
                            shared_state.latest_dist = laser_dist
                            shared_state.laser_ts = trigger_t
                            shared_state.laser_track_id = shared_state.active_track_id
                            active_cmd_id = shared_state.active_cmd_id
                            active_track_id = shared_state.active_track_id
                        if prev_laser_ts > 0:
                            dt_laser = trigger_t - prev_laser_ts
                            print(
                                f"[Laser] Triggered cmd_id={active_cmd_id}, track_id={active_track_id}, "
                                f"dist={laser_dist:.2f}m, interval={dt_laser:.3f}s"
                            )
                        else:
                            print(
                                f"[Laser] Triggered cmd_id={active_cmd_id}, track_id={active_track_id}, "
                                f"dist={laser_dist:.2f}m, interval=first"
                            )
                    else:
                        print("[Laser] No valid laser distance, use mono distance")
                    active_cmd = None
                    continue

            if (now_t - cmd_start_t) >= GIMBAL_SETTLE_TIMEOUT:
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
        "MIN_DT": 0.08,  # 10 FPS 场景下的保护下限
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
        self.max_vel_el = 20.0
        self.min_dt = 0.03
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
        
        self.time_since_update += 1
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
        
        # 计算残差 Y = Z - HX
        Y = Z - np.dot(H, self.state)
        
        # 核心：处理 Azimuth 的残差，防止角度回环跳变
        Y[0, 0] = angular_diff(Z[0, 0], self.state[0, 0])
        
        # S = H * P * H^T + R
        S = np.dot(np.dot(H, self.P), H.T) + R
        
        # 卡尔曼增益 K = P * H^T * S^-1
        try:
            K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        except np.linalg.LinAlgError:
            print(f"[Tracker] Kalman matrix inversion failed for track {self.id}")
            K = np.zeros((4, 2), dtype=float)

        # X = X + K * Y
        self.state = self.state + np.dot(K, Y)
        # 更新后再次对 Az 取模
        self.state[0, 0] = self.state[0, 0] % 360.0
        
        # 速度限幅保护
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
        max_lost_frames=15,
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

    def update(self, measurements, dt, params=None, now_t=None):
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
            self.tracks = [t for t in self.tracks if t.time_since_update < self.max_lost_frames]
            return self.tracks

        if len(self.tracks) == 0:
            # 全是新目标
            for meas in normalized_measurements:
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
            return self.tracks

        # 2. 计算代价矩阵 (角度欧氏距离)
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
            
            if cost_matrix[t_idx, m_idx] < dynamic_thresh:
                meas = normalized_measurements[m_idx]
                track.update(meas["az"], meas["el"], dt)
                track.set_mono_distance(meas["mono_dist"], now_t)
                unmatched_measurements.discard(m_idx)
                matched_tracks.add(t_idx)
            else:
                pass

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

        # 6. 删除丢失太久的 Track
        self.tracks = [t for t in self.tracks if t.time_since_update < self.max_lost_frames]

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
# ==========================================
# 6. 主逻辑 V9 (多目标预测与云台调度)
# ==========================================
def main():
    if LOG_TO_FILE:
        try:
            log_path = _setup_log_mirror(LOG_DIR)
            if log_path:
                print(f"[Log] stdout/stderr -> {log_path}")
        except Exception as e:
            print(f"[Log][Warn] 日志文件初始化失败: {e}")

    sender = UISender(UI_IP, UI_PORT)
    if not USE_MOCK_GIMBAL:
        _warn_if_windows_port("GIMBAL_PORT", GIMBAL_PORT)
    _warn_if_windows_port("LASER_PORT", LASER_PORT)
    if ENABLE_IMU:
        _warn_if_windows_port("IMU_PORT", IMU_PORT)
    
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

    # start background threads for network and gimbal control
    threading.Thread(target=rk3588_thread, daemon=True).start()
    threading.Thread(target=gimbal_control_thread, args=(gimbal,), daemon=True).start()

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
    tracker = MultiTargetTracker(max_lost_frames=12, distance_threshold=1.2)
    
    # 状态机与调度变量
    master_id = None
    lock_timer = 0.0
    LOCK_DURATION = 1.5      # 锁定目标的最长驻留时间
    PREDICT_DELAY = 0.25     # 系统与物理响应总延迟 (打提前量)
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
    ui_send_counter = Counter()  # {target_id: send_count}
    stats_last_print = last_time

    while True:
        try:
            curr_time = time.time()
            if imu and (curr_time - last_imu_print) >= IMU_PRINT_INTERVAL:
                acc, gyro, angle = imu.get_all()
                roll, pitch, yaw = angle
                print(f"ANGLE: {roll:6.2f} {pitch:6.2f} {yaw:6.2f}")
                last_imu_print = curr_time

            # --- 1. 获取 UDP 数据 (如果有) ---
            if not packet_queue:
                time.sleep(0.005) # 稍微让出 CPU
                continue
            #计算两包UDP数据的间隔时间
            dt = curr_time - last_time
            if dt <= 0:
                dt = 1.0 / 10.0
            #有可能两包数据间隔很久，为了防止追踪器模型发散，限制最大 dt
            if dt > MAX_DT:
                dt = MAX_DT
            last_time = curr_time

            while len(packet_queue) > 1: packet_queue.popleft()
            pkg = packet_queue.popleft()
            
            board_str = pkg.get("board", "Unknown") 
            cam_idx = int(pkg.get("cam", 0))
            raw_objs = pkg.get("objs", [])

            logic_id, cfg = get_camera_params(board_str, cam_idx)
            if logic_id is None: continue 

            # --- 2. 坐标解析为绝对角度 ---
            current_measurements = []
            parsed_objs = parse_udp_objects(raw_objs)

            with shared_state.lock:
                shared_gimbal_az = shared_state.gimbal_az
                shared_gimbal_el = shared_state.gimbal_el
                valid_laser_dist = shared_state.valid_laser_dist
                laser_ts = shared_state.laser_ts
                laser_track_id = shared_state.laser_track_id

            for obj_item in parsed_objs:
                rect = obj_item["box"]
                mono_dist = obj_item["mono_dist"]
                recv_obj_total += 1
                recv_unique_boxes.add((
                    int(round(rect[0])),
                    int(round(rect[1])),
                    int(round(rect[2])),
                    int(round(rect[3])),
                ))
                
                cx = (rect[0] + rect[2]) / 2.0
                cy = (rect[1] + rect[3]) / 2.0
                
                res = calculate_angles(logic_id, cx, cy, cfg)
                if res:
                    ui_az, ui_el = res
                    d_az_to_target = angular_diff(ui_az, shared_gimbal_az)
                    d_el_to_target = ui_el - shared_gimbal_el
                    # turn_dir = get_turn_direction_label(d_az_to_target, d_el_to_target)
                    current_measurements.append({
                        "az": ui_az,
                        "el": ui_el,
                        "mono_dist": mono_dist,
                    })
                    # === 新增：Phase 1 打印 ===
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
            active_tracks = tracker.update(current_measurements, dt, now_t=curr_time)

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
            
            if master_track is None or lock_timer <= 0:
                # 状态 A：寻找/切换新目标 (SEARCHING)
                best_track = None
                best_score = -float('inf')
                
                curr_gimbal_az = shared_gimbal_az
                curr_gimbal_el = shared_gimbal_el
                #基于威胁算法的多目标抓捕优先级评估：(这个地方需要给出全部排序，不能只挑一个最优的)
                if len(valid_tracks) > 0:
                    for t in valid_tracks:
                        # 1. 速度因子 (正权 2.0)：提取目标的合成角速度大小
                        v_mag = np.sqrt(t.state[2, 0]**2 + t.state[3, 0]**2)
                        speed_score = v_mag * 2.0
                        
                        # 2. 距离因子 (负权 1.0)：计算目标当前角度与云台物理角度的距离
                        dist_az = angular_diff(t.state[0, 0], curr_gimbal_az)
                        dist_el = t.state[1, 0] - curr_gimbal_el
                        dist = np.sqrt(dist_az**2 + dist_el**2)
                        distance_score = -dist * 1.0
                        
                        # 3. 稳定性因子（弱权且饱和）：避免 hit_streak 越积越大主导调度
                        stability_level = min(t.hit_streak, STABILITY_HIT_CAP)
                        stability_score = math.log1p(stability_level) * STABILITY_WEIGHT
                        
                        # 综合威胁度得分
                        threat_score = speed_score + distance_score + stability_score
                        
                        # 当前锁定目标 15% 惯性加成防抖动
                        if t.id == master_id:
                            # 给原得分基础上乘以 1.15
                            threat_score *= 1.15
                            
                        if threat_score > best_score:
                            best_score = threat_score
                            best_track = t
                            
                    if best_track:
                        master_id = best_track.id
                        lock_timer = LOCK_DURATION
                        master_track = best_track
                        print(f"[Phase 2: 目标调度] 基于威胁度(v={np.sqrt(master_track.state[2,0]**2+master_track.state[3,0]**2):.1f}°/s, dist={np.sqrt(angular_diff(master_track.state[0,0], curr_gimbal_az)**2 + (master_track.state[1,0]-curr_gimbal_el)**2):.1f}°) 成功锁定目标 ID: {master_id}. 准备交由云台跟踪!")
            # --- 5. 状态机：物理执行与测距 (LOCKED) ---
            if master_track is not None:
                lock_timer -= dt
                
                # A. 提取提前量预测角度
                fut_az, fut_el = master_track.get_future_position(dt_delay=PREDICT_DELAY)
                
                # B. 转换为云台控制角
                ctrl_az, ctrl_el = ui_to_ctrl_angles(fut_az, fut_el)
                # === 新增：Phase 3 打印 ===
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

                # E. 向 UI 发送数据包 (遍历所有合法的追踪档案)
                # 这样即使云台在打 ID 1，UI 上也能看到 ID 2, 3 的平滑轨迹
                for t in valid_tracks:
                    send_dist, dist_source = select_track_distance(t, master_id, curr_time)
                    if math.isfinite(send_dist) and dist_source.startswith("mono"):
                        t.last_sent_dist = send_dist
                    
                    sender.send_status(
                        board_str, cam_idx, t.id,  # 这里传入的是持续追踪的 ID，而不是一闪而过的数组下标
                        azimuth=t.state[0, 0],        # 发送卡尔曼平滑后的位置
                        elevation=t.state[1, 0], 
                        distance=send_dist
                    )
                    ui_send_total += 1
                    ui_send_counter[int(t.id)] += 1

            if (curr_time - stats_last_print) >= STATS_PRINT_INTERVAL:
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
    if 'gimbal' in locals():
        gimbal.close()
    final_id_text = "none"
    if ui_send_counter:
        final_id_text = ", ".join([f"{tid}:{cnt}" for tid, cnt in ui_send_counter.most_common()])
    print(
        f"[Stats][Final] recv_objs_total={recv_obj_total}, recv_unique_boxes={len(recv_unique_boxes)} | "
        f"ui_send_total={ui_send_total}, ui_unique_ids={len(ui_send_counter)}, ui_id_counts={final_id_text}"
    )

if __name__ == "__main__":
    main()
