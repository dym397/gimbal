import sys
from pathlib import Path
from types import SimpleNamespace

import pynmea2


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
sys.path.insert(0, str(CORE_DIR))

import gps  # noqa: E402
from gps import gga_ellipsoid_height  # noqa: E402


def test_real_gga_frame_reconstructs_vendor_ellipsoid_height():
    msg = pynmea2.parse(
        "$GNGGA,123353.00,3024.48203234,N,10405.31940902,E,1,28,0.7,"
        "490.9668,M,-42.8244,M,,*5E"
    )

    assert abs(gga_ellipsoid_height(msg) - 448.1424) < 1.0e-6


def test_read_gps_fix_can_return_msl_altitude_for_rid_geometry():
    frame = (
        b"$GNGGA,123353.00,3024.48203234,N,10405.31940902,E,1,28,0.7,"
        b"490.9668,M,-42.8244,M,,*5E\r\n"
    )

    class FakeSerial:
        def __init__(self, *args, **kwargs):
            self.frames = [frame]

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            return False

        def readline(self):
            return self.frames.pop(0) if self.frames else b""

    original_serial = gps.serial.Serial
    original_comports = gps.serial.tools.list_ports.comports
    gps.serial.Serial = FakeSerial
    gps.serial.tools.list_ports.comports = lambda: [SimpleNamespace(device="COM8")]
    try:
        longitude, latitude, altitude, source = gps.read_gps_fix(
            port="COM8",
            timeout_seconds=0.1,
            print_raw=False,
            print_status=False,
            coordinate_system="wgs84",
            include_altitude=True,
            altitude_reference="msl",
        )
    finally:
        gps.serial.Serial = original_serial
        gps.serial.tools.list_ports.comports = original_comports

    assert abs(longitude - 104.088656817) < 1.0e-9
    assert abs(latitude - 30.408033872333333) < 1.0e-9
    assert altitude == 490.9668
    assert source == "COM8"
