import sys
from pathlib import Path

import pynmea2


CORE_DIR = Path(__file__).resolve().parents[1] / "core"
sys.path.insert(0, str(CORE_DIR))

from gps import gga_ellipsoid_height  # noqa: E402


def test_real_gga_frame_reconstructs_vendor_ellipsoid_height():
    msg = pynmea2.parse(
        "$GNGGA,123353.00,3024.48203234,N,10405.31940902,E,1,28,0.7,"
        "490.9668,M,-42.8244,M,,*5E"
    )

    assert abs(gga_ellipsoid_height(msg) - 448.1424) < 1.0e-6
