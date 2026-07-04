import io
import sys
import socket
import struct
from functools import partial
import math
import json
import random
import numpy as np
import cv2
import folium
import time

from PyQt5.QtCore import QEvent, QThread, pyqtSignal, QDateTime, Qt, QMutex, QTimer
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import QMainWindow, QApplication, QMessageBox, QListWidgetItem, QSizePolicy, QVBoxLayout, \
    QTableWidgetItem, QPushButton, QHeaderView, QLineEdit, QLabel, QWidget
from PyQt5 import uic, QtWidgets
from PyQt5.QtGui import QPixmap, QPainter, QColor, QIcon, QImage, QFontMetrics

camera_map = {
    ('BOARD_1', 0): {"cam_id": 0, "map_id": "1-1"},
    ('BOARD_1', 1): {"cam_id": 1, "map_id": "1-2"},
    ('BOARD_1', 2): {"cam_id": 2, "map_id": "1-3"},
    ('BOARD_1', 3): {"cam_id": 3, "map_id": "1-4"},
    ('BOARD_1', 4): {"cam_id": 4, "map_id": "1-5"},
    ('BOARD_2', 0): {"cam_id": 5, "map_id": "2-1"},
    ('BOARD_2', 1): {"cam_id": 6, "map_id": "2-2"},
    ('BOARD_2', 2): {"cam_id": 7, "map_id": "2-3"},
    ('BOARD_2', 3): {"cam_id": 8, "map_id": "2-4"},
    ('BOARD_2', 4): {"cam_id": 9, "map_id": "2-5"},
    ('BOARD_3', 0): {"cam_id": 10, "map_id": "3-1"},
    ('BOARD_3', 1): {"cam_id": 11, "map_id": "3-2"},
    ('BOARD_3', 2): {"cam_id": 12, "map_id": "3-3"},
    ('BOARD_3', 3): {"cam_id": 13, "map_id": "3-4"},
    ('BOARD_3', 4): {"cam_id": 14, "map_id": "3-5"},
    ('BOARD_4', 0): {"cam_id": 15, "map_id": "4-1"},
    ('BOARD_4', 1): {"cam_id": 16, "map_id": "4-2"},
    ('BOARD_4', 2): {"cam_id": 17, "map_id": "4-3"},
    ('BOARD_4', 3): {"cam_id": 18, "map_id": "4-4"},
    ('BOARD_4', 4): {"cam_id": 19, "map_id": "4-5"},
    ('BOARD_5', 0): {"cam_id": 20, "map_id": "5-1"},
    ('BOARD_5', 1): {"cam_id": 21, "map_id": "5-2"},
    ('BOARD_5', 2): {"cam_id": 22, "map_id": "5-3"},
    ('BOARD_5', 3): {"cam_id": 23, "map_id": "5-4"},
    ('BOARD_5', 4): {"cam_id": 24, "map_id": "5-5"},
    ('BOARD_6', 0): {"cam_id": 25, "map_id": "6-1"},
    ('BOARD_6', 1): {"cam_id": 26, "map_id": "6-2"},
    ('BOARD_6', 2): {"cam_id": 27, "map_id": "6-3"},
    ('BOARD_6', 3): {"cam_id": 28, "map_id": "6-4"},
    ('BOARD_6', 4): {"cam_id": 29, "map_id": "6-5"},
    ('BOARD_7', 0): {"cam_id": 30, "map_id": "7-1"},
    ('BOARD_7', 1): {"cam_id": 31, "map_id": "7-2"},
    ('BOARD_7', 2): {"cam_id": 32, "map_id": "7-3"},
    ('BOARD_7', 3): {"cam_id": 33, "map_id": "7-4"},
    ('BOARD_7', 4): {"cam_id": 34, "map_id": "7-5"},
    ('BOARD_8', 0): {"cam_id": 35, "map_id": "8-1"},
    ('BOARD_8', 1): {"cam_id": 36, "map_id": "8-2"},
    ('BOARD_8', 2): {"cam_id": 37, "map_id": "8-3"},
    ('BOARD_8', 3): {"cam_id": 38, "map_id": "8-4"},
    ('BOARD_8', 4): {"cam_id": 39, "map_id": "8-5"},
    ('BOARD_9', 0): {"cam_id": 40, "map_id": "9-1"},
    ('BOARD_9', 1): {"cam_id": 41, "map_id": "9-2"},
    ('BOARD_9', 2): {"cam_id": 42, "map_id": "9-3"},
    ('BOARD_9', 3): {"cam_id": 43, "map_id": "9-4"},
    ('BOARD_9', 4): {"cam_id": 44, "map_id": "9-5"},
}


# -----------------------------------------------------------
# 界面表格进行排序
# -----------------------------------------------------------
class NumericTableWidgetItem(QTableWidgetItem):
    """
    自定义的 Item，重写比较函数 (__lt__)，
    使得表格排序时按数字大小排，而不是按字符串排。
    """

    def __lt__(self, other):
        # 尝试将文本转为 float 进行比较
        try:
            return float(self.text()) < float(other.text())
        except ValueError:
            # 如果转换失败（比如有文字），回退到默认字符串比较
            return super().__lt__(other)


# -----------------------------------------------------------
# 后台接收线程
# -----------------------------------------------------------
class UdpReceiverThread(QThread):
    sig_video_frame = pyqtSignal(np.ndarray)  # 发送图像数据
    sig_heartbeat = pyqtSignal(int)  # 发送心跳包 (只发送ID，表示该ID活着)
    sig_error = pyqtSignal(str)  # 发送错误信息
    sig_status = pyqtSignal(int, int, float, float, float)  # 发送态势数据
    sig_system_ready = pyqtSignal()  # 初始化完成信号
    sig_position = pyqtSignal(float, float)  # 发送(lat, lon)

    def __init__(self, port=9999, is_ready=False):
        super().__init__()
        self.port = port
        self.running = False
        self.current_watching_id = -1
        self.sock = None

        # 系统就绪标志位
        self.is_system_ready = is_ready

    def set_watching_id(self, cam_id):
        self.current_watching_id = cam_id

    def stop(self):
        self.running = False

    def run(self):
        self.running = True

        MSG_VIDEO = 0x01
        MSG_STATUS = 0x02
        MSG_POSITION = 0x03  # 经纬度包

        STANDARD_HEADER_SIZE = 10  # 1(MsgType) + 1(CamID) + 8(BoardID)
        POSITION_PACKET_SIZE = 9  # 1(MsgType) + 4(latitude) + 4(longitude)

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 10 * 1024 * 1024)
            self.sock.bind(('0.0.0.0', self.port))
            self.sock.settimeout(0.5)

            print(f"[System] 服务启动，监听端口: {self.port}")

            while self.running:
                try:
                    data, addr = self.sock.recvfrom(65535)

                    if len(data) < 1:
                        continue

                    # 先只取消息类型
                    msg_type = data[0]

                    # ==================================================
                    # 1. 经纬度包：1B MsgType + 4B lat + 4B lon
                    # ==================================================
                    if msg_type == MSG_POSITION:
                        if len(data) < POSITION_PACKET_SIZE:
                            continue

                        lat, lon = struct.unpack('!ff', data[1:9])

                        # 首次收到有效包，可视为系统已就绪
                        if not self.is_system_ready:
                            self.is_system_ready = True
                            self.sig_system_ready.emit()

                        self.sig_position.emit(lat, lon)
                        continue

                    # ==================================================
                    # 2. 视频/态势包：按统一头解析
                    # ==================================================
                    if len(data) < STANDARD_HEADER_SIZE:
                        continue

                    cam_num = data[1]

                    try:
                        board_id_bytes = data[2:10]
                        board_id_str = board_id_bytes.decode('utf-8', errors='ignore').strip('\x00')
                    except Exception:
                        continue

                    mapping_info = camera_map.get((board_id_str, cam_num))
                    if mapping_info is None:
                        continue

                    cam_id = mapping_info["cam_id"]

                    if not self.is_system_ready:
                        if msg_type == MSG_VIDEO or msg_type == MSG_STATUS:
                            self.is_system_ready = True
                            self.sig_system_ready.emit()

                    if not self.is_system_ready:
                        continue

                    self.sig_heartbeat.emit(cam_id)

                    # ---------- 视频包 ----------
                    if msg_type == MSG_VIDEO:
                        if self.current_watching_id != -1 and cam_id == self.current_watching_id:
                            jpeg_data = data[STANDARD_HEADER_SIZE:]

                            np_arr = np.frombuffer(jpeg_data, dtype=np.uint8)
                            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                            if frame is not None:
                                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                self.sig_video_frame.emit(frame)

                    # ---------- 态势包 ----------
                    elif msg_type == MSG_STATUS:

                        STATUS_DATA_LEN = 16
                        if len(data) >= STANDARD_HEADER_SIZE + STATUS_DATA_LEN:
                            target_id, azimuth, elevation, distance = struct.unpack(
                                '!Ifff',
                                data[STANDARD_HEADER_SIZE: STANDARD_HEADER_SIZE + STATUS_DATA_LEN]
                            )
                            self.sig_status.emit(cam_id, target_id, azimuth, elevation, distance)

                    else:
                        print(f"[Warning] Unknown msg type: {msg_type}")

                except socket.timeout:
                    continue

                except Exception as e:
                    print(f"[Loop Error] {e}")
                    continue

        except Exception as e:
            self.sig_error.emit(str(e))
        finally:
            if self.sock:
                self.sock.close()
            print("[System] 接收线程已结束")


def xor_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def format_strike_tilt(tilt_raw):
    if 0 <= tilt_raw <= 9000:
        return f"向下 {tilt_raw / 100.0:.2f}°"
    if 27000 <= tilt_raw <= 35999:
        return f"向上 {(36000 - tilt_raw) / 100.0:.2f}°"
    return f"异常 {tilt_raw / 100.0:.2f}°"


def parse_strike_feedback_packet(data):
    if len(data) < 11:
        raise ValueError("打击端反馈帧长度不足")

    packet = data[:11]
    if packet[0] != 0xAA:
        raise ValueError("打击端反馈帧头错误")
    if packet[1] != 0x0C:
        raise ValueError("打击端反馈帧长度字段错误")
    if packet[10] != 0xBB:
        raise ValueError("打击端反馈帧尾错误")
    if xor_checksum(packet[:9]) != packet[9]:
        raise ValueError("打击端反馈校验错误")

    pan_raw = struct.unpack("!H", packet[2:4])[0]
    tilt_raw = struct.unpack("!H", packet[4:6])[0]
    pan_speed = packet[6]
    net_speed_raw = struct.unpack("!H", packet[7:9])[0]

    return {
        "pan_deg": pan_raw / 100.0,
        "tilt_raw": tilt_raw,
        "tilt_text": format_strike_tilt(tilt_raw),
        "pan_speed": pan_speed,
        "net_speed_mps": net_speed_raw / 10.0,
        "timestamp": time.time(),
    }


class StrikeFeedbackThread(QThread):
    sig_feedback = pyqtSignal(dict)
    sig_error = pyqtSignal(str)

    def __init__(self, port=10123):
        super().__init__()
        self.port = port
        self.running = False
        self.sock = None

    def stop(self):
        self.running = False

    def run(self):
        self.running = True

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("0.0.0.0", self.port))
            self.sock.settimeout(0.5)

            print(f"[Strike] Feedback listener started on port {self.port}")

            while self.running:
                try:
                    data, addr = self.sock.recvfrom(1024)
                    feedback = parse_strike_feedback_packet(data)
                    self.sig_feedback.emit(feedback)
                except socket.timeout:
                    continue
                except ValueError as e:
                    print(f"[Strike Packet Error] {e}")
                    continue
                except Exception as e:
                    print(f"[Strike Loop Error] {e}")
                    continue

        except Exception as e:
            self.sig_error.emit(str(e))
        finally:
            if self.sock:
                self.sock.close()
            print("[Strike] Feedback listener stopped")


class UdpJsonDistanceThread(QThread):
    sig_distance = pyqtSignal(object)
    sig_debug = pyqtSignal(str)
    sig_error = pyqtSignal(str)

    def __init__(self, host="0.0.0.0", port=1234):
        super().__init__()
        self.host = host
        self.port = port
        self.running = False
        self.sock = None

    def stop(self):
        self.running = False

    def run(self):
        self.running = True

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(0.5)

            print(f"[UDPJsonDistance] Listening on {self.host}:{self.port}")

            while self.running:
                try:
                    data, addr = self.sock.recvfrom(4096)
                    try:
                        text = data.decode("utf-8")
                    except UnicodeDecodeError as e:
                        self.sig_debug.emit(f"1234 invalid UTF-8 from {addr[0]}:{addr[1]}: {e}")
                        continue

                    self.sig_debug.emit(f"1234 raw from {addr[0]}:{addr[1]}: {text}")

                    try:
                        packet = json.loads(text)
                    except json.JSONDecodeError as e:
                        self.sig_debug.emit(f"1234 invalid JSON: {e}")
                        continue

                    if not isinstance(packet, dict):
                        self.sig_debug.emit(f"1234 JSON is not an object: {packet!r}")
                        continue

                    distance_field = None
                    for candidate in ("distance_2", "distance", "current_distance", "currentDistance"):
                        if candidate in packet:
                            distance_field = candidate
                            break

                    if distance_field is None:
                        self.sig_debug.emit("1234 JSON missing distance field")
                        continue

                    distance = packet.get(distance_field)
                    if distance is not None:
                        try:
                            distance = float(distance)
                        except (TypeError, ValueError):
                            self.sig_debug.emit(f"1234 invalid {distance_field} value: {packet.get(distance_field)!r}")
                            continue
                        if math.isnan(distance):
                            self.sig_debug.emit(f"1234 {distance_field} is NaN, display as --")
                            distance = None
                        elif distance > 400 or distance < 30:
                            self.sig_debug.emit(f"1234 {distance_field} out of display range: {distance:.1f}, display as --")
                            distance = None
                        else:
                            self.sig_debug.emit(f"1234 {distance_field} raw={distance:.1f}")
                    else:
                        self.sig_debug.emit(f"1234 {distance_field} is null, display as --")

                    self.sig_debug.emit(f"1234 using {distance_field}={distance!r}")

                    self.sig_distance.emit(distance)
                except socket.timeout:
                    continue
                except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as e:
                    print(f"[UDPJsonDistance Packet Error] {e}")
                    continue
                except Exception as e:
                    print(f"[UDPJsonDistance Loop Error] {e}")
                    continue

        except Exception as e:
            self.sig_error.emit(str(e))
        finally:
            if self.sock:
                self.sock.close()
            print("[UDPJsonDistance] Listener stopped")


# -----------------------------------------------------------
# 坐标转换
# -----------------------------------------------------------
def wgs84_to_gcj02(lng, lat):
    """
    WGS84转GCJ02(火星坐标系)
    :param lng: WGS84坐标系的经度
    :param lat: WGS84坐标系的纬度
    :return: 转换后的(经度, 纬度)
    """

    # 判断是否在国内，不在国内不做偏移
    def out_of_china(lng, lat):
        return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

    if out_of_china(lng, lat):
        return lng, lat

    a = 6378245.0
    ee = 0.00669342162296594323
    pi = 3.1415926535897932384626

    def _transformlat(lng, lat):
        ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * pi) + 40.0 * math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * pi) + 320 * math.sin(lat * pi / 30.0)) * 2.0 / 3.0
        return ret

    def _transformlng(lng, lat):
        ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
        ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lng * pi) + 40.0 * math.sin(lng / 3.0 * pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lng / 12.0 * pi) + 300.0 * math.sin(lng / 30.0 * pi)) * 2.0 / 3.0
        return ret

    dlat = _transformlat(lng - 105.0, lat - 35.0)
    dlng = _transformlng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)

    return lng + dlng, lat + dlat


# -----------------------------------------------------------
# 登录界面
# -----------------------------------------------------------
class LoginWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # 加载登录界面的 UI
        self.ui = uic.loadUi('./login.ui', self)
        self.ui.setWindowIcon(QIcon("images/login.png"))

        # 去掉窗口边框（可选，让界面更像个弹窗）
        # self.setWindowFlags(Qt.FramelessWindowHint)
        # 设置窗口标题
        self.setWindowTitle("用户登录")

        # --- 设置密码框为密文模式 ---
        self.ui.lineEdit_password.setEchoMode(QLineEdit.Password)

        self.ui.label_3.setFixedHeight(25)

        self.ui.pushButton_login.setStyleSheet(
            "QPushButton{font: 75 18px '黑体';background-color: rgb(60,60,60);"
            "color:rgb(255,255,255);border-radius: 5px;padding: 5px;}"
            "QPushButton:hover{background-color: rgb(125,125,125);}")
        self.ui.pushButton_cancel.setStyleSheet(
            "QPushButton{font: 75 18px '黑体';background-color: rgb(60,60,60);"
            "color:rgb(255,255,255);border-radius: 5px;padding: 5px;}"
            "QPushButton:hover{background-color: rgb(125,125,125);}")

        # --- 绑定按钮事件 ---
        self.ui.pushButton_login.clicked.connect(self.on_login_click)
        self.ui.pushButton_cancel.clicked.connect(self.on_cancel_click)

    def on_login_click(self):
        """点击登录按钮的逻辑"""
        username = self.ui.lineEdit_user.text().strip()  # 获取用户名并去空格
        password = self.ui.lineEdit_password.text().strip()  # 获取密码

        # 1. 校验是否为空
        if not username or not password:
            QMessageBox.warning(self, "提示", "请输入用户名和密码！")
            return

        # 2. 校验账号密码
        if username == "admin" and password == "123":
            QMessageBox.information(self, "登录成功", "欢迎进入复眼式光学阵列系统！")
            # --- 核心逻辑：关闭登录窗，打开主窗口 ---
            self.main_window = UI_MainWindow()  # 实例化主窗口
            self.main_window.ui.show()  # 显示主窗口
            self.close()  # 关闭登录窗口

        else:
            QMessageBox.critical(self, "错误", "用户名或密码错误，请重试！")
            # 可以选择清空密码框
            self.ui.lineEdit_password.clear()

    def on_cancel_click(self):
        """点击取消按钮，关闭整个程序"""
        self.close()


# ------------------------------------------------
# 主界面
# ------------------------------------------------
class UI_MainWindow(QMainWindow):
    # 定义一些常量
    TOTAL_CAMERAS = 45
    TIMEOUT_SECONDS = 3.0  # 超过3秒没收到数据视为断开
    GPS_RELOAD_THRESHOLD_METERS = 10.0  # GPS位置变化超过10米才重载地图

    # 状态枚举
    STATUS_UNKNOWN = 0  # 灰色 (初始)
    STATUS_ONLINE = 1  # 绿色
    STATUS_OFFLINE = 2  # 红色

    # ------------------------------------------------
    # 初始化
    # ------------------------------------------------
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi('./main.ui', self)

        # 窗口居中
        self.fit_window_to_current_screen()
        self.ui.setWindowIcon(QIcon("images/main.png"))

        # ----------------------------------------
        # 变量初始化
        # ----------------------------------------
        self.video_thread = None
        self.feedback_thread = None
        self.json_distance_thread = None
        self.current_cam_id = 0
        self.target_data_pool = {}
        self.latest_strike_feedback = None
        self.use_json_distance_override = True
        self.latest_json_distance = None
        self.latest_json_distance_time = None
        self.target_distance_noise = {}

        # 地图初始化状态
        self.map_initialized = False  # 是否已收到真实经纬度
        self.center_lat = None
        self.center_lon = None

        # 默认地图中心：北京
        self.default_lat = 39.9042
        self.default_lon = 116.4074

        self.enable_radar_scan = False

        # 控制遮罩是否在下一次地图加载完成后自动隐藏
        self.hide_overlay_after_load = False

        # =========================================================
        # 方便后续界面显示时，把 0-44 转换成 "1-1", "9-5" 等
        # =========================================================
        self.id_to_map_name = {}
        for info in camera_map.values():
            c_id = info['cam_id']
            m_id = info['map_id']
            self.id_to_map_name[c_id] = m_id

        # 记录每个摄像头最后一次收到心跳的时间戳 (float time.time())
        self.last_heartbeat = {}
        # 记录每个摄像头当前的状态，用于防止日志重复打印
        self.camera_states = [self.STATUS_UNKNOWN] * self.TOTAL_CAMERAS

        # 定时器：用于定期检查在线状态
        self.status_check_timer = QTimer(self)
        self.status_check_timer.timeout.connect(self.check_all_cameras_status)

        # 只要程序不关闭，这个状态就一直保存
        self.system_initialized_once = False
        self.current_session_has_data = False

        # ----------------------------------------------------
        # 界面刷新定时器
        # ----------------------------------------------------
        self.ui_refresh_timer = QTimer(self)
        self.ui_refresh_timer.timeout.connect(self.refresh_status_table)
        self.ui_refresh_timer.start(500)  # 500ms 刷新一次

        # ----------------------------------------
        # UI 初始化
        # ----------------------------------------
        self.ui.start.setEnabled(True)
        self.ui.shutdown.setEnabled(False)

        # 初始化地图
        self.browser = QWebEngineView()
        self.browser = QWebEngineView()
        self.browser.installEventFilter(self)
        layout = QVBoxLayout()
        layout.addWidget(self.browser)
        layout.setContentsMargins(0, 0, 0, 0)
        self.ui.tab_map.setLayout(layout)

        self.browser.loadFinished.connect(self.on_map_load_finished)

        self.init_map_overlay()
        self.load_map(self.default_lat, self.default_lon, center_popup="默认中心")
        self.show_map_overlay("等待定位数据...")

        # 连接信号
        self.ui.start.clicked.connect(self.on_start_click)
        self.ui.shutdown.clicked.connect(self.on_shutdown_click)
        self.ui.view_state.clicked.connect(self.on_check_device_status_clicked)
        self.ui.listWidget.itemClicked.connect(self.on_camera_selected)

        self.init_css()
        self.setup_list_widget()

        # 初始化日志
        self.log_message("系统初始化完成，等待连接...")

    def fit_window_to_current_screen(self):
        design_w, design_h = 1400, 953
        screen = QtWidgets.QApplication.primaryScreen()
        if screen is None:
            self.ui.resize(design_w, design_h)
            return

        available = screen.availableGeometry()
        margin = 40
        max_w = max(640, available.width() - margin)
        max_h = max(480, available.height() - margin)
        scale = min(1.0, max_w / design_w, max_h / design_h)

        target_w = min(available.width(), max(640, int(design_w * scale)))
        target_h = min(available.height(), max(480, int(design_h * scale)))
        x = available.x() + max(0, (available.width() - target_w) // 2)
        y = available.y() + max(0, (available.height() - target_h) // 2)

        self.ui.setMinimumSize(640, 480)
        self.ui.setGeometry(x, y, target_w, target_h)

    def init_map_overlay(self):
        # 直接把提示框挂到地图控件上，而不是 tab_map
        self.map_loading_label = QLabel("地图加载中...", self.browser)
        self.map_loading_label.setAlignment(Qt.AlignCenter)
        self.map_loading_label.setWordWrap(False)
        self.map_loading_label.setStyleSheet("""
            QLabel {
                background-color: rgba(255, 255, 255, 230);
                color: rgb(30, 30, 30);
                font-size: 15px;
                font-weight: bold;
                border: 1px solid rgba(0, 0, 0, 45);
                border-radius: 10px;
            }
        """)
        self.map_loading_label.hide()

        self.update_map_overlay_position()

    def update_map_loading_label_size(self):
        if not hasattr(self, "map_loading_label"):
            return

        fm = QFontMetrics(self.map_loading_label.font())
        text = self.map_loading_label.text()

        text_w = fm.horizontalAdvance(text)
        text_h = fm.height()

        # 手动留白，避免中文被裁掉
        box_w = max(165, text_w + 36)
        box_h = max(38, text_h + 16)

        self.map_loading_label.resize(box_w, box_h)

    def update_map_overlay_position(self):
        if not hasattr(self, "map_loading_label"):
            return

        self.update_map_loading_label_size()

        label_w = self.map_loading_label.width()
        label_h = self.map_loading_label.height()

        browser_w = self.browser.width()
        browser_h = self.browser.height()

        # 真正贴着地图右下角
        margin_right = 16
        margin_bottom = 16

        x = browser_w - label_w - margin_right
        y = browser_h - label_h - margin_bottom

        self.map_loading_label.move(max(0, x), max(0, y))

    def show_map_overlay(self, text="地图加载中..."):
        if hasattr(self, "map_loading_label"):
            self.map_loading_label.setText(text)
            self.update_map_overlay_position()
            self.map_loading_label.raise_()
            self.map_loading_label.show()
            QTimer.singleShot(0, self.update_map_overlay_position)

    def hide_map_overlay(self):
        if hasattr(self, "map_loading_label"):
            self.map_loading_label.hide()

    def build_map_loading_placeholder_html(self):
        return """
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8">
            <style>
                html, body {
                    margin: 0;
                    width: 100%;
                    height: 100%;
                    overflow: hidden;
                    background: #edf1f5;
                }
            </style>
        </head>
        <body></body>
        </html>
        """

    def reset_map_to_loading_view(self, text="地图加载中..."):
        self.enable_radar_scan = False
        self.map_initialized = False
        self.center_lat = None
        self.center_lon = None
        self.hide_overlay_after_load = False
        self.load_map(self.default_lat, self.default_lon, center_popup="默认中心")
        self.show_map_overlay(text)

    def eventFilter(self, watched, event):
        if watched is self.browser and event.type() in (QEvent.Show, QEvent.Resize):
            QTimer.singleShot(0, self.update_map_overlay_position)

        return super().eventFilter(watched, event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_map_overlay_position()

    def on_map_load_finished(self, ok):
        if ok and self.hide_overlay_after_load:
            self.hide_map_overlay()
            self.hide_overlay_after_load = False

    # ------------------------------------------------
    # 加载地图
    # ------------------------------------------------
    def load_map(self, lat, lon, center_popup="态势中心"):
        # 保存中心经纬度，后续计算目标位置时需要用到
        self.center_lat = lat
        self.center_lon = lon

        # 1. 初始化地图
        initial_zoom = 15
        radar_max_radius = 1000
        m = folium.Map(location=[lat, lon], zoom_start=initial_zoom, max_zoom=18, control_scale=True, tiles=None, prefix=False)

        # 2. 添加图层
        folium.TileLayer(
            tiles='https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
            attr='高德卫星',
            name='卫星底图',
            overlay=False,
            subdomains='1234'
        ).add_to(m)
        folium.TileLayer(
            tiles='https://webst0{s}.is.autonavi.com/appmaptile?style=8&x={x}&y={y}&z={z}',
            attr='高德标注',
            name='路网标注',
            overlay=True,
            subdomains='1234',
            opacity=0.85
        ).add_to(m)
        # 中心点：改为蓝色填充 + 白色粗边框，尺寸稍大
        folium.CircleMarker(location=[lat, lon], radius=6, color='white', weight=3, fill=True, fill_color='blue',
                            fill_opacity=1.0, popup=center_popup).add_to(m)

        # 3. 绘制雷达圈
        radar_radii = [1000, 800, 600, 500, 400, 300, 200, 100]
        radar_color = '#FFD700'
        for r in radar_radii:
            is_outer = (r == radar_max_radius)
            folium.Circle(location=[lat, lon], radius=r, color=radar_color, weight=2 if is_outer else 1, fill=True,
                          fill_color=radar_color, fill_opacity=0.1 if is_outer else 0.0).add_to(m)
            folium.map.Marker([lat + (r / 111132.0), lon], icon=folium.DivIcon(icon_size=(100, 20), icon_anchor=(0, 0),
                                                                               html=f'<div style="font-size: 14px; font-weight: bold; color: {radar_color}; text-shadow: 1px 1px 2px black;">{r}m</div>')).add_to(
                m)

        # 4. 绘制十字准星
        offset_lat = radar_max_radius / 111132.0
        offset_lon = radar_max_radius / (111132.0 * math.cos(math.radians(lat)))
        folium.PolyLine(locations=[[lat - offset_lat, lon], [lat + offset_lat, lon]], color=radar_color, weight=1,
                        opacity=0.6).add_to(m)
        folium.PolyLine(locations=[[lat, lon - offset_lon], [lat, lon + offset_lon]], color=radar_color, weight=1,
                        opacity=0.6).add_to(m)

        # 5. 右上角：恢复初始视图按钮
        reset_view_js = f"""
        <script>
        function addResetViewButton(map) {{
            var ResetControl = L.Control.extend({{
                options: {{ position: 'topright' }},
                onAdd: function () {{
                    var container = L.DomUtil.create(
                        'div',
                        'leaflet-bar leaflet-control leaflet-control-custom'
                    );

                    container.innerHTML = '⟳';
                    container.title = '恢复初始视图';
                    container.style.backgroundColor = 'white';
                    container.style.width = '30px';
                    container.style.height = '30px';
                    container.style.lineHeight = '30px';
                    container.style.textAlign = 'center';
                    container.style.cursor = 'pointer';
                    container.style.fontSize = '18px';

                    container.onclick = function () {{
                        map.setView([{lat}, {lon}], {initial_zoom});
                    }};
                    return container;
                }}
            }});
            map.addControl(new ResetControl());
        }}

        setTimeout(function() {{
            var map = {m.get_name()};
            addResetViewButton(map);
        }}, 500);
        </script>
        """
        m.get_root().html.add_child(folium.Element(reset_view_js))
        m.get_root().html.add_child(folium.Element('<style>.leaflet-control-attribution {display:none;}</style>'))

        # ==============================================================================
        # 注入动态更新标记的 JavaScript 代码
        # ==============================================================================
        map_var_name = m.get_name()  # 获取 folium 自动生成的地图变量名 (如 map_12345)

        js_marker_updater = f"""
        <script>
        // 定义一个全局对象，用来存储当前地图上的所有目标标记
        // Key: "camID_targetID", Value: Leaflet Marker Object
        var dynamicMarkers = {{}}; 

        // Python 将会调用这个函数，传入 JSON 字符串
        function updateRadarMarkers(json_str) {{
            var targets = JSON.parse(json_str);
            var current_keys = new Set();
            var map_obj = {map_var_name}; // 获取地图对象引用

            // 1. 遍历收到的新数据，更新或创建标记
            targets.forEach(function(t) {{
                current_keys.add(t.key); // 记录当前存在的 key

                if (dynamicMarkers[t.key]) {{
                    // --- 情况 A: 标记已存在，移动位置 ---
                    dynamicMarkers[t.key].setLatLng([t.lat, t.lon]);

                    // 如果需要更新标签文字，可以用:
                    // dynamicMarkers[t.key].setTooltipContent(t.label);
                }} else {{
                    // --- 情况 B: 标记不存在，创建新标记 ---
                    // 这里创建一个红色实心小圆点
                    var newMarker = L.circleMarker([t.lat, t.lon], {{
                        radius: 8,           // 半径大小
                        color: 'white',      // 描边颜色
                        weight: 1,           // 描边宽度
                        fillColor: '#FF0000',// 填充红色
                        fillOpacity: 1.0     // 不透明
                    }}).addTo(map_obj);

                    // 存入字典，以便下次移动或删除
                    dynamicMarkers[t.key] = newMarker;
                }}
            }});

            // 2. 清理已消失的目标
            // 遍历字典里所有的旧 Key，如果它不在本次的新数据 current_keys 里，说明目标丢失
            for (var key in dynamicMarkers) {{
                if (!current_keys.has(key)) {{
                    map_obj.removeLayer(dynamicMarkers[key]); // 从地图移除
                    delete dynamicMarkers[key];               // 从字典移除
                }}
            }}
        }}
        </script>

        <style>
        /* 设置目标文字标签的样式 */
        .target-label {{
            background: rgba(0, 0, 0, 0.6);
            border: none;
            color: yellow;
            font-size: 11px;
            font-weight: bold;
            box-shadow: none;
        }}
        /* 去掉标签的小箭头 */
        .leaflet-tooltip-left:before, .leaflet-tooltip-right:before {{
            border: none !important;
        }}
        </style>
        """
        # 将这段 JS 和 CSS 添加到地图 HTML 中
        m.get_root().html.add_child(folium.Element(js_marker_updater))

        if self.enable_radar_scan:
            radar_scan_js = f"""
            <script>
            setTimeout(function() {{
                var map_obj = {map_var_name};
                if (!map_obj) {{
                    return;
                }}

                var centerLat = {lat};
                var centerLon = {lon};
                var sweepHead = 0;
                var sweepLayers = [];
                var sweepConfig = [

                    {{tail: 26, width: 80, radius: 500, opacity: 0.20, color: '#2EEB9A'}}
                ];

                function metersToLat(meters) {{
                    return meters / 111132.0;
                }}

                function metersToLon(meters) {{
                    var denom = 111132.0 * Math.cos(centerLat * Math.PI / 180);
                    return meters / Math.max(denom, 0.000001);
                }}

                function pointAt(angleDeg, radiusMeters) {{
                    var rad = angleDeg * Math.PI / 180.0;
                    var deltaLat = Math.cos(rad) * metersToLat(radiusMeters);
                    var deltaLon = Math.sin(rad) * metersToLon(radiusMeters);
                    return [centerLat + deltaLat, centerLon + deltaLon];
                }}

                function buildSector(headAngle, tailAngle, radiusMeters) {{
                    var points = [[centerLat, centerLon]];
                    var steps = 24;

                    for (var i = 0; i <= steps; i++) {{
                        var angle = tailAngle + (headAngle - tailAngle) * (i / steps);
                        points.push(pointAt(angle, radiusMeters));
                    }}

                    points.push([centerLat, centerLon]);
                    return points;
                }}

                sweepConfig.forEach(function(cfg) {{
                    var layer = L.polygon(buildSector(0, -cfg.width, cfg.radius), {{
                        stroke: false,
                        fill: true,
                        fillColor: cfg.color,
                        fillOpacity: cfg.opacity,
                        interactive: false
                    }}).addTo(map_obj);

                    sweepLayers.push(layer);
                }});

                function animateSweep() {{
                    sweepHead = (sweepHead + 1.2) % 360;

                    sweepLayers.forEach(function(layer, index) {{
                        var cfg = sweepConfig[index];
                        var headAngle = sweepHead - cfg.tail;
                        var tailAngle = headAngle - cfg.width;
                        layer.setLatLngs(buildSector(headAngle, tailAngle, cfg.radius));
                    }});

                    window.requestAnimationFrame(animateSweep);
                }}

                animateSweep();
            }}, 500);
            </script>
            """
            m.get_root().html.add_child(folium.Element(radar_scan_js))

        # 6. 保存并显示
        data = io.BytesIO()
        m.save(data, close_file=False)
        self.browser.setHtml(data.getvalue().decode())

    # ------------------------------------------------
    # 计算距离
    # ------------------------------------------------
    def calculate_gps_distance_meters(self, lat1, lon1, lat2, lon2):
        earth_radius = 6371000.0

        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad

        a = (math.sin(delta_lat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return earth_radius * c

    # ------------------------------------------------
    # 收到坐标后更新地图
    # ------------------------------------------------
    def on_udp_position_received(self, lat, lon):
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            self.log_message(f"收到非法坐标: lat={lat}, lon={lon}")
            return

        # 每次收到合法 GPS 都更新地图中心
        self.mark_current_session_active()

        if self.map_initialized and self.center_lat is not None and self.center_lon is not None:
            move_distance = self.calculate_gps_distance_meters(self.center_lat, self.center_lon, lat, lon)
            if move_distance < self.GPS_RELOAD_THRESHOLD_METERS:
                return

        self.show_map_overlay("地图加载中...")

        self.hide_overlay_after_load = True
        self.enable_radar_scan = True
        self.load_map(lat, lon, center_popup="当前位置")

        if not self.map_initialized:
            self.map_initialized = True
            self.log_message(f"地图初始化完成: 纬度 {lat:.6f}, 经度 {lon:.6f}")
        else:
            self.log_message(f"地图中心已更新: 纬度 {lat:.6f}, 经度 {lon:.6f}")

    # ------------------------------------------------
    # 日志辅助功能
    # ------------------------------------------------
    def log_message(self, msg):
        """向文本框追加日志"""
        if hasattr(self.ui, 'textEdit'):
            time_str = QDateTime.currentDateTime().toString("HH:mm:ss")
            self.ui.textEdit.append(f"[{time_str}] {msg}")
        else:
            print(f"[Log] {msg}")

    # ------------------------------------------------
    # 当前会话是否已经连接
    # ------------------------------------------------
    def mark_current_session_active(self):
        if self.current_session_has_data:
            return

        self.current_session_has_data = True
        self.ui.start.setText("服务运行中")
        self.ui.start.setStyleSheet("background-color: green; color: white;")
        self.log_message("本次连接已收到有效数据，服务运行中")

    # ------------------------------------------------
    # 按钮功能实现
    # ------------------------------------------------
    def on_start_click(self):
        if self.video_thread is not None and self.video_thread.isRunning():
            return

        # 1. 启动接收线程
        self.current_session_has_data = False

        self.video_thread = UdpReceiverThread(port=9999, is_ready=self.system_initialized_once)
        self.video_thread.sig_video_frame.connect(self.update_video_display)
        self.video_thread.sig_heartbeat.connect(self.on_heartbeat_received)  # 绑定心跳
        self.video_thread.sig_error.connect(self.on_thread_error)
        self.video_thread.sig_status.connect(self.on_udp_status_received)
        self.video_thread.sig_system_ready.connect(self.on_system_init_received)
        self.video_thread.sig_position.connect(self.on_udp_position_received)

        self.video_thread.set_watching_id(self.current_cam_id)
        self.video_thread.start()

        self.feedback_thread = StrikeFeedbackThread(port=10123)
        self.feedback_thread.sig_feedback.connect(self.on_strike_feedback_received)
        self.feedback_thread.sig_error.connect(self.on_strike_thread_error)
        self.feedback_thread.start()

        # 2. 启动状态检查定时器 (每1秒检查一次)
        self.json_distance_thread = UdpJsonDistanceThread(port=1234)
        self.json_distance_thread.sig_distance.connect(self.on_json_distance_received)
        self.json_distance_thread.sig_error.connect(self.on_json_distance_thread_error)
        self.json_distance_thread.start()

        self.status_check_timer.start(1000)

        # 3. UI 更新
        self.ui.start.setEnabled(False)
        self.ui.shutdown.setEnabled(True)

        # 根据是否已经初始化过，显示不同的状态
        if self.system_initialized_once:
            # 已经初始化过了，直接变绿，无需等待
            self.log_message("正在启动接收服务 (快速恢复)...")
            self.ui.start.setText("等待数据")
            self.ui.start.setStyleSheet("background-color: #d4a017; color: white;")
        else:
            # 第一次启动，需要等待 0x03
            self.log_message("正在启动接收服务 (等待初始化)...")
            self.ui.start.setText("等待初始化")
            self.ui.start.setStyleSheet("background-color: #d4a017; color: white;")  # 黄色

    # ------------------------------------------------
    # 处理初始化完成
    # ------------------------------------------------
    def on_system_init_received(self):

        self.system_initialized_once = True

        self.log_message("=========================")
        self.log_message("收到初始化指令 - 系统就绪")
        self.log_message("=========================")

        # 更新按钮样式为绿色
        self.mark_current_session_active()

    # ------------------------------------------------
    # 点击关闭连接
    # ------------------------------------------------
    def on_shutdown_click(self):
        """断开连接"""
        if self.video_thread is None and self.feedback_thread is None and self.json_distance_thread is None:
            return

        self.log_message("正在停止服务...")

        self.clear_status_table()

        # 1. 停止定时器（不再检查心跳）
        self.status_check_timer.stop()

        # 2. 【关键步骤】先断开信号连接
        # 这样即使线程还在处理最后一帧，也不会触发 update_video_display 了
        try:
            if self.video_thread is not None:
                self.video_thread.sig_video_frame.disconnect(self.update_video_display)
                self.video_thread.sig_heartbeat.disconnect(self.on_heartbeat_received)
        except Exception:
            pass  # 防止如果未连接时报错

        # 3. 停止线程
        if self.video_thread is not None:
            self.video_thread.stop()
            self.video_thread.wait()
            self.video_thread = None

        if self.feedback_thread is not None:
            self.feedback_thread.stop()
            self.feedback_thread.wait()
            self.feedback_thread = None

        if self.json_distance_thread is not None:
            self.json_distance_thread.stop()
            self.json_distance_thread.wait()
            self.json_distance_thread = None

        # 4. 【关键步骤】立即清理画面，并显示你要求的文字
        # 获取当前正在看的摄像头ID
        map_name = self.id_to_map_name.get(self.current_cam_id, str(self.current_cam_id))
        current_name = f"摄像头 #{map_name}"

        self.clear_video_screen(f"{current_name} 无信号")

        # 5. 重置数据和UI
        self.reset_all_camera_icons_to_gray()
        self.last_heartbeat.clear()
        self.camera_states = [self.STATUS_UNKNOWN] * self.TOTAL_CAMERAS
        self.target_data_pool.clear()
        self.latest_strike_feedback = None
        self.latest_json_distance = None
        self.latest_json_distance_time = None
        self.target_distance_noise.clear()
        self.current_session_has_data = False
        self.reset_map_to_loading_view()

        # 6. 恢复按钮状态
        self.ui.start.setEnabled(True)
        self.ui.start.setText("建立连接")
        self.ui.start.setStyleSheet(
            "QPushButton{font: 75 16px '黑体';background-color: rgb(60,60,60);"
            "color:rgb(255,255,255);border-radius: 5px;padding: 5px;}"
            "QPushButton:hover{background-color: rgb(125,125,125);}")
        self.ui.shutdown.setEnabled(False)

        self.ui.camera_title.setText(f"请选择需要打开的摄像头")
        self.clear_video_screen("等待接收数据...")
        self.log_message("服务已停止。")

    # ------------------------------------------------
    # 线程
    # ------------------------------------------------
    def on_thread_error(self, msg):
        self.log_message(f"错误: {msg}")
        QMessageBox.critical(self, "连接错误", msg)
        self.on_shutdown_click()

    def on_strike_thread_error(self, msg):
        self.log_message(f"打击端反馈接收错误: {msg}")

    def on_json_distance_thread_error(self, msg):
        self.log_message(f"JSON distance receive error: {msg}")

    def on_json_distance_received(self, distance):
        self.latest_json_distance = distance
        self.latest_json_distance_time = time.time()
        if self.use_json_distance_override:
            target_ids = {tracking['target_id'] for tracking in self.target_data_pool.values()}
            self.refresh_target_distance_offsets(target_ids)
            for tracking in self.target_data_pool.values():
                tracking['dist'] = self.get_tracking_distance(
                    tracking['target_id'],
                    tracking.get('raw_dist')
                )

    def refresh_target_distance_offsets(self, target_ids):
        target_ids = sorted(set(target_ids))
        if not target_ids:
            self.target_distance_noise.clear()
            return

        offsets = {}
        current_offset = 0.0
        for index, target_id in enumerate(target_ids):
            if index > 0:
                current_offset += random.uniform(5.0, 10.0)
            offsets[target_id] = current_offset

        center_offset = sum(offsets.values()) / len(offsets)
        offsets = {target_id: offset - center_offset for target_id, offset in offsets.items()}

        if self.latest_json_distance is not None:
            min_display = self.latest_json_distance + min(offsets.values())
            max_display = self.latest_json_distance + max(offsets.values())
            shift = 0.0
            if min_display < 30.0:
                shift = 30.0 - min_display
            elif max_display > 400.0:
                shift = 400.0 - max_display
            offsets = {target_id: offset + shift for target_id, offset in offsets.items()}

        self.target_distance_noise = offsets

    def get_tracking_distance(self, target_id, tracking_distance):
        if not self.use_json_distance_override:
            return tracking_distance

        if self.latest_json_distance is None:
            return tracking_distance

        if target_id not in self.target_distance_noise:
            target_ids = {tracking['target_id'] for tracking in self.target_data_pool.values()}
            target_ids.add(target_id)
            self.refresh_target_distance_offsets(target_ids)

        distance = round(self.latest_json_distance + self.target_distance_noise[target_id], 1)
        return min(400.0, max(30.0, distance))

    def on_strike_feedback_received(self, feedback):
        feedback["timestamp"] = time.time()
        self.latest_strike_feedback = feedback

    # ------------------------------------------------
    # 心跳与状态检查
    # ------------------------------------------------
    def on_heartbeat_received(self, cam_id):
        """线程收到数据包时调用，只更新时间戳"""
        if 0 <= cam_id < self.TOTAL_CAMERAS:
            self.mark_current_session_active()
            self.last_heartbeat[cam_id] = time.time()

    # ------------------------------------------------
    # 检查所有摄像头的状态
    # ------------------------------------------------
    def check_all_cameras_status(self):
        """
        每秒被定时器调用一次。
        遍历所有摄像头，对比当前时间与最后一次心跳时间。
        """
        now = time.time()
        online_count = 0
        offline_count = 0

        for i in range(self.TOTAL_CAMERAS):

            map_name = self.id_to_map_name.get(i, f"#{i + 1}")
            display_name = f"摄像头 #{map_name}"

            last_time = self.last_heartbeat.get(i, 0)

            # 判断是否在线 (当前时间 - 最后活跃时间 < 超时阈值)
            is_alive = (now - last_time) < self.TIMEOUT_SECONDS and last_time > 0

            # 获取列表项
            item = self.ui.listWidget.item(i)

            # --- 状态机逻辑 ---
            old_state = self.camera_states[i]

            if is_alive:
                online_count += 1
                new_state = self.STATUS_ONLINE
                # 如果状态发生了改变 (之前是未知或离线，现在上线了)
                if old_state != self.STATUS_ONLINE:
                    item.setIcon(self.create_status_icon(Qt.green))
                    # self.log_message(f"{display_name} 已连接")

                    # 如果当前观看的正是这个摄像头，恢复提示文字(画面会由视频流覆盖)
                    if i == self.current_cam_id:
                        self.ui.video_label.setText("")
            else:
                # 离线逻辑
                if last_time == 0:
                    # 从未收到过数据，保持灰色(Unknown)
                    new_state = self.STATUS_UNKNOWN
                    # 计入未连接
                    offline_count += 1
                else:
                    # 曾经收到过数据，现在超时了 -> 红色(Offline)
                    offline_count += 1
                    new_state = self.STATUS_OFFLINE
                    if old_state != self.STATUS_OFFLINE:
                        item.setIcon(self.create_status_icon(Qt.red))
                        self.log_message(f"{display_name} 信号中断 (超时)")

                        # 【需求点】如果当前正在观看的摄像头断开了，画面要处理
                        if i == self.current_cam_id:
                            self.clear_video_screen(f"{display_name} 信号丢失")

            self.camera_states[i] = new_state

        status_summary = f"总计: {self.TOTAL_CAMERAS} | 已连接: {online_count} | 未连接/断开: {offline_count}"
        if hasattr(self.ui, 'label_status'):
            self.ui.label_status.setText(status_summary)

    # ------------------------------------------------
    # 检查设备状态
    # ------------------------------------------------
    def on_check_device_status_clicked(self):
        online_names = []
        offline_names = []

        for i in range(self.TOTAL_CAMERAS):
            # 获取显示名称
            map_name = self.id_to_map_name.get(i, f"#{i + 1}")

            map_name = "#" + map_name

            state = self.camera_states[i]
            if state == self.STATUS_ONLINE:
                online_names.append(map_name)
            else:
                offline_names.append(map_name)

        online_count = len(online_names)
        offline_count = len(offline_names)

        # 拼接字符串
        online_str = ", ".join(online_names) if online_names else "无"
        offline_str = ", ".join(offline_names) if offline_names else "无"

        self.log_message("===== 当前设备状态 =====")
        self.log_message(f"在线摄像头：{online_count} 台 ({online_str})")
        self.log_message(f"未连接摄像头：{offline_count} 台 ({offline_str})")
        self.log_message("========================")

    # ------------------------------------------------
    # 视频与列表UI设计
    # ------------------------------------------------
    def setup_list_widget(self):
        self.ui.listWidget.clear()
        for i in range(self.TOTAL_CAMERAS):
            # get(i, str(i)) 是为了防止字典里缺漏，缺漏时默认显示数字
            display_name = self.id_to_map_name.get(i, f"#{i + 1}")

            item = QListWidgetItem(f"摄像头 #{display_name}")

            item.setIcon(self.create_status_icon(Qt.gray))

            item.setData(Qt.UserRole, i)

            self.ui.listWidget.addItem(item)

        if self.ui.listWidget.count() > 0:
            self.current_cam_id = 0
            self.ui.listWidget.setCurrentRow(0)

            # 获取第一个摄像头的显示名称
            first_name = self.id_to_map_name.get(0, "1-1")
            self.clear_video_screen("等待接收数据...")

    # ------------------------------------------------
    # 左边列表中点击单个摄像头显示画面
    # ------------------------------------------------
    def on_camera_selected(self, item):
        cam_id = item.data(Qt.UserRole)
        self.current_cam_id = cam_id

        # 获取显示名称
        map_name = self.id_to_map_name.get(cam_id, str(cam_id))
        self.ui.camera_title.setText(f"正在显示摄像头 #{map_name}")

        self.ui.tabWidget.setCurrentIndex(0)

        # ---------------------------------------------------------
        # 1. 如果未建立连接，只提示，不记录日志，不执行后续逻辑
        # ---------------------------------------------------------
        if self.video_thread is None or not self.video_thread.isRunning():
            # self.clear_video_screen(f"已选中 #{cam_id + 1} (请先建立连接)")
            return  # 【关键】直接返回，不再执行下面的日志记录

        # ---------------------------------------------------------
        # 2. 如果已连接，执行正常的切换逻辑
        # ---------------------------------------------------------

        current_display_name = f"摄像头 #{map_name}"  # 统一变量名方便下面用

        # 通知线程切换ID
        self.video_thread.set_watching_id(cam_id)

        # 检查该摄像头当前状态
        state = self.camera_states[cam_id]
        if state == self.STATUS_ONLINE:
            self.ui.video_label.setText("正在缓冲...")
            # 这里的画面会随后被视频流覆盖
        elif state == self.STATUS_OFFLINE:
            self.clear_video_screen(f"{current_display_name} 当前离线")
        else:
            self.clear_video_screen(f"{current_display_name} 无信号")

        # 只有在已连接的情况下，才记录这条日志
        self.log_message(f"切换监视至{current_display_name}")

    # ------------------------------------------------
    # 实时更新摄像头画面
    # ------------------------------------------------
    def update_video_display(self, frame_rgb):
        """收到视频帧，更新界面"""
        # 只有当前选中的摄像头在线时才更新，防止串台
        # (虽然线程里已经过滤了ID，但这里双重保险)
        h, w, ch = frame_rgb.shape
        bytes_per_line = ch * w
        qt_img = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)

        lbl_size = self.ui.video_label.size()
        if not lbl_size.isEmpty():
            pixmap = QPixmap.fromImage(qt_img)
            scaled_pixmap = pixmap.scaled(lbl_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.ui.video_label.setPixmap(scaled_pixmap)

    # ------------------------------------------------
    # 清除页面中的视频画面
    # ------------------------------------------------
    def clear_video_screen(self, text):
        """将视频区域置黑并显示文字"""
        self.ui.video_label.clear()
        self.ui.video_label.setText(text)
        self.ui.video_label.setStyleSheet(
            "QLabel { background-color: black; color: white; border: 1px solid #555; font-size: 18px; }")

    # ------------------------------------------------
    # 重置摄像头UI图标至灰色
    # ------------------------------------------------
    def reset_all_camera_icons_to_gray(self):
        for i in range(self.ui.listWidget.count()):
            item = self.ui.listWidget.item(i)
            item.setIcon(self.create_status_icon(Qt.gray))

    # ------------------------------------------------
    # 实时更新检测的表格数据
    # ------------------------------------------------
    def refresh_status_table(self):
        """
        定时任务：合并追踪端与打击端数据，按目标 ID 刷新表格。
        """
        table = self.ui.message
        now = time.time()

        # 1. 分别清理两条链路的过期数据。任一链路存在数据时，目标行都保留。
        tracking_keys_to_remove = []
        for key, info in self.target_data_pool.items():
            if now - info['timestamp'] > 5.0:
                tracking_keys_to_remove.append(key)
        for key in tracking_keys_to_remove:
            del self.target_data_pool[key]

        strike = self.latest_strike_feedback
        if strike is not None and now - strike['timestamp'] > 5.0:
            self.latest_strike_feedback = None
            strike = None

        # 2. 同一 target_id 如果被多个相机看到，表格展示距离最近的一条追踪端数据。
        tracking_by_target = {}
        for data in self.target_data_pool.values():
            target_id = data['target_id']
            old_data = tracking_by_target.get(target_id)
            if old_data is None:
                tracking_by_target[target_id] = data
                continue

            old_dist = old_data.get('dist')
            new_dist = data.get('dist')
            if old_dist is None or (isinstance(old_dist, float) and math.isnan(old_dist)):
                tracking_by_target[target_id] = data
            elif new_dist is not None and not (isinstance(new_dist, float) and math.isnan(new_dist)):
                if new_dist < old_dist:
                    tracking_by_target[target_id] = data

        target_ids = set(tracking_by_target.keys())
        for target_id in list(self.target_distance_noise.keys()):
            if target_id not in target_ids:
                del self.target_distance_noise[target_id]

        def sort_key(target_id):
            tracking = tracking_by_target.get(target_id)
            if tracking is None:
                return (1, target_id)

            dist = tracking.get('dist')
            if dist is None or (isinstance(dist, float) and math.isnan(dist)):
                return (0, float('inf'), target_id)
            return (0, dist, target_id)

        sorted_target_ids = sorted(target_ids, key=sort_key)
        table_target_ids = sorted_target_ids if sorted_target_ids else ([None] if strike is not None else [])

        # 3. 设置行数
        table.setRowCount(len(table_target_ids))

        # --- 准备地图数据列表 ---
        map_targets_list = []

        # 4. 填入数据
        for row, target_id in enumerate(table_target_ids):
            tracking = tracking_by_target.get(target_id) if target_id is not None else None

            for col in range(table.columnCount()):
                if table.cellWidget(row, col) is not None:
                    table.removeCellWidget(row, col)

            table.setItem(row, 0, NumericTableWidgetItem("--" if target_id is None else str(target_id)))

            if tracking is not None:
                current_cam_id = tracking['cam_id']
                map_name = self.id_to_map_name.get(current_cam_id, str(current_cam_id))
                table.setItem(row, 1, NumericTableWidgetItem(f"摄像头 #{map_name}"))
                table.setItem(row, 2, NumericTableWidgetItem(f"{tracking['az']:.1f}"))
                table.setItem(row, 3, NumericTableWidgetItem(f"{tracking['el']:.1f}"))

                dist = tracking.get('dist')
                if dist is None or (isinstance(dist, float) and math.isnan(dist)):
                    table.setItem(row, 4, NumericTableWidgetItem("--"))
                else:
                    table.setItem(row, 4, NumericTableWidgetItem(f"{dist:.1f}"))

                # 只有地图中心初始化完成后，才计算目标经纬度并更新地图标记。
                if self.map_initialized and dist is not None and not (isinstance(dist, float) and math.isnan(dist)):
                    t_lat, t_lon = self.calculate_target_lat_lon(tracking['az'], dist)
                    map_targets_list.append({
                        "key": str(target_id),
                        "lat": t_lat,
                        "lon": t_lon
                    })
            else:
                table.setItem(row, 1, NumericTableWidgetItem("--"))
                table.setItem(row, 2, NumericTableWidgetItem("--"))
                table.setItem(row, 3, NumericTableWidgetItem("--"))
                table.setItem(row, 4, NumericTableWidgetItem("--"))

            if strike is not None:
                table.setItem(row, 5, NumericTableWidgetItem(f"{strike['pan_deg']:.2f}"))
                table.setItem(row, 6, NumericTableWidgetItem(strike['tilt_text']))
                table.setItem(row, 7, NumericTableWidgetItem(str(strike['pan_speed'])))
                table.setItem(row, 8, NumericTableWidgetItem(f"{strike['net_speed_mps']:.1f}"))
            else:
                table.setItem(row, 5, NumericTableWidgetItem("--"))
                table.setItem(row, 6, NumericTableWidgetItem("--"))
                table.setItem(row, 7, NumericTableWidgetItem("--"))
                table.setItem(row, 8, NumericTableWidgetItem("--"))

            # 居中对齐
            for col in range(table.columnCount()):
                item = table.item(row, col)
                if item: item.setTextAlignment(Qt.AlignCenter)

        # 发送数据给地图 JS

        if self.browser and self.map_initialized:
            json_str = json.dumps(map_targets_list)
            self.browser.page().runJavaScript(f"updateRadarMarkers('{json_str}');")

    # ------------------------------------------------
    # 将接收到的udp目标信息保存
    # ------------------------------------------------
    def on_udp_status_received(self, cam_id, target_id, azimuth, elevation, distance):
        """
        接收 UDP 数据存入字典。
        注意：必须依赖 target_id 来区分同相机的不同目标。
        """
        key = (cam_id, target_id)
        raw_distance = distance
        distance = self.get_tracking_distance(target_id, raw_distance)

        self.target_data_pool[key] = {
            'cam_id': cam_id,
            'target_id': target_id,  # 虽然不显示，但存着备用
            'az': azimuth,
            'el': elevation,
            'raw_dist': raw_distance,
            'dist': distance,
            'timestamp': time.time()
        }

    # ------------------------------------------------
    # 清楚目标表格信息
    # ------------------------------------------------
    def clear_status_table(self):
        """完全清空表格"""
        table = self.ui.message
        table.clearContents()
        table.setRowCount(0)

    # ------------------------------------------------
    # 显示摄像头相应画面（按钮）
    # ------------------------------------------------
    def open_video_by_cam_id(self, cam_id):

        self.video_thread.set_watching_id(cam_id)

        self.ui.tabWidget.setCurrentIndex(0)

        map_name = self.id_to_map_name.get(cam_id, str(cam_id))
        self.ui.camera_title.setText(f"正在显示摄像头 #{map_name}")

        self.log_message(f"切换监视至摄像头 #{map_name}")

        self.ui.video_label.clear()

    # ------------------------------------------------
    # UI样式
    # ------------------------------------------------
    def init_css(self):
        self.ui.start.setStyleSheet(
            "QPushButton{font: 75 16px '黑体';background-color: rgb(60,60,60);"
            "color:rgb(255,255,255);border-radius: 5px;padding: 5px;}"
            "QPushButton:hover{background-color: rgb(125,125,125);}")
        self.ui.shutdown.setStyleSheet(
            "QPushButton{font: 75 16px '黑体';background-color: rgb(60,60,60);"
            "color:rgb(255,255,255);border-radius: 5px;padding: 5px;}"
            "QPushButton:hover{background-color: rgb(125,125,125);}")
        self.ui.view_state.setStyleSheet(
            "QPushButton{font: 75 16px '黑体';background-color: rgb(60,60,60);"
            "color:rgb(255,255,255);border-radius: 5px;padding: 5px;}"
            "QPushButton:hover{background-color: rgb(125,125,125);}")
        self.ui.listWidget.setStyleSheet("""
            QListWidget { font: 75 15px '黑体'; }
            QListWidget::item { padding: 10px; }
            QListWidget::item:selected { background-color: #0078d7; color: white; }
        """)
        self.ui.frame.setStyleSheet("background-color: #202020;")
        self.ui.camera_title.setStyleSheet("color: white; font-size: 20px")
        self.ui.message.verticalHeader().setVisible(False)
        self.ui.message.setSortingEnabled(False)
        self.ui.message.setColumnCount(9)
        self.ui.message.setHorizontalHeaderLabels([
            "目标序号",
            "相机编号",
            "方位",
            "俯仰",
            "距离",
            "方位(打击端)",
            "俯仰(打击端)",
            "云台转速(打击端)",
            "网捕速度",
        ])
        header = self.ui.message.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        # 默认黑屏样式
        self.ui.video_label.setStyleSheet("QLabel { background-color: black; color: white; border: 1px solid #555; }")
        self.ui.video_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.ui.video_label.setAlignment(Qt.AlignCenter)

    # ------------------------------------------------
    # UI图标设置
    # ------------------------------------------------
    def create_status_icon(self, color):
        pixmap = QPixmap(16, 16)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        painter.setBrush(QColor(color))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(0, 0, 16, 16)
        painter.end()
        return QIcon(pixmap)

    # ------------------------------------------------
    # 根据目标距离我们设备的位置以及方位角，计算相应的目标经纬度坐标
    # ------------------------------------------------
    def calculate_target_lat_lon(self, az_deg, dist_m):
        """
        根据中心点、方位角(0=北)、距离计算目标经纬度
        """
        # 将角度转为弧度
        # 数学上 0度通常指正东(X轴)，但雷达/地图 0度通常指正北(Y轴)
        # 0度(北) -> sin=0, cos=1 (对应Lat增量)
        # 90度(东) -> sin=1, cos=0 (对应Lon增量)

        rad = math.radians(az_deg)

        # 地球每度纬度对应的米数 (近似值)
        
        meters_per_lat = 111132.0
        # 地球每度经度对应的米数 (随纬度变化)
        meters_per_lon = 111132.0 * math.cos(math.radians(self.center_lat))

        # 计算偏移
        delta_lat = (dist_m * math.cos(rad)) / meters_per_lat
        delta_lon = (dist_m * math.sin(rad)) / meters_per_lon

        return self.center_lat + delta_lat, self.center_lon + delta_lon

    # ------------------------------------------------
    # 关闭事件
    # ------------------------------------------------
    def closeEvent(self, event):
        if self.video_thread is not None:
            self.video_thread.stop()
            self.video_thread.wait()
        if self.feedback_thread is not None:
            self.feedback_thread.stop()
            self.feedback_thread.wait()
        if self.json_distance_thread is not None:
            self.json_distance_thread.stop()
            self.json_distance_thread.wait()
        super().closeEvent(event)


if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)

    login_win = LoginWindow()
    login_win.show()

    sys.exit(app.exec_())
