import serial
import serial.tools.list_ports
import math
import os
import time
import pynmea2

DEFAULT_LONGITUDE = 104.3070
DEFAULT_LATITUDE = 30.9497
GPS_FIX_TIMEOUT_SECONDS = 60


def default_gps_port():
    env_port = os.getenv("GPS_PORT")
    if env_port:
        return env_port.strip()
    if os.name == "nt":
        return "COM8"
    return "/dev/ttyUSB3"


def _normalize_windows_com_port(port_name):
    normalized = port_name.strip().upper()
    if normalized.startswith("\\\\.\\"):
        normalized = normalized[4:]
    return normalized


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


def read_gps_fix(port=None, baudrate=115200, timeout_seconds=GPS_FIX_TIMEOUT_SECONDS, print_raw=True):
    if port is None:
        port = default_gps_port()

    ports = [item.device for item in serial.tools.list_ports.comports()]
    if os.name == "nt":
        available_ports = {_normalize_windows_com_port(item) for item in ports}
        if _normalize_windows_com_port(port) not in available_ports:
            return None, None, "fallback"

    if not port:
        return None, None, "fallback"

    longitude = None
    latitude = None

    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            deadline = time.monotonic() + timeout_seconds
            while time.monotonic() < deadline:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                if print_raw:
                    print(f"[GPS] raw line: {line}")

                if line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                    try:
                        msg = pynmea2.parse(line)
                    except pynmea2.ParseError:
                        continue

                    if str(getattr(msg, "gps_qual", "0")) == "0":
                        continue
                    if msg.longitude == 0 or msg.latitude == 0:
                        continue

                    longitude = round(msg.longitude, 4)
                    latitude = round(msg.latitude, 4)
                    break
    except serial.SerialException:
        return None, None, "serial_error"

    if longitude is None or latitude is None:
        return None, None, "no_fix"

    longitude, latitude = wgs84_to_gcj02(longitude, latitude)
    return longitude, latitude, port


def read_gps(port=None, baudrate=115200, timeout_seconds=GPS_FIX_TIMEOUT_SECONDS, print_raw=True):
    longitude, latitude, gps_source = read_gps_fix(
        port=port,
        baudrate=baudrate,
        timeout_seconds=timeout_seconds,
        print_raw=print_raw,
    )
    if longitude is None or latitude is None:
        return DEFAULT_LONGITUDE, DEFAULT_LATITUDE, gps_source
    return longitude, latitude, gps_source


if __name__ == "__main__":
    longitude, latitude, gps_source = read_gps()
    print(f"source={gps_source} longitude={longitude:.6f} latitude={latitude:.6f}")
