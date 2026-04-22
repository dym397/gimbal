import serial
import serial.tools.list_ports
import math
import time
import pynmea2

DEFAULT_LONGITUDE = 104.3070
DEFAULT_LATITUDE = 30.9497
GPS_FIX_TIMEOUT_SECONDS = 60
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
def read_gps(port="COM8", baudrate=115200, timeout_seconds=GPS_FIX_TIMEOUT_SECONDS):
    ports = [item.device for item in serial.tools.list_ports.comports()]
    if port not in ports:
        return DEFAULT_LONGITUDE, DEFAULT_LATITUDE, "fallback"

    longitude = None
    latitude = None

    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            deadline = time.monotonic() + timeout_seconds
            while time.monotonic() < deadline:
                line = ser.readline().decode("ascii", errors="ignore").strip()
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
        return DEFAULT_LONGITUDE, DEFAULT_LATITUDE, "serial_error"

    if longitude is None or latitude is None:
        return DEFAULT_LONGITUDE, DEFAULT_LATITUDE, "no_fix"

    longitude, latitude = wgs84_to_gcj02(longitude, latitude)
    return longitude, latitude, port


longitude, latitude, gps_source = read_gps()


if __name__ == "__main__":
    print(f"source={gps_source} longitude={longitude:.6f} latitude={latitude:.6f}")
