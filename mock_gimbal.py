import time
from typing import Optional, Tuple

from gimbal_interface import GimbalBase


class MockGimbalAdapter(GimbalBase):
    """
    Software gimbal model for SITL/closed-loop validation.

    Defaults are aligned to the GT06Z characteristics provided by the user:
    - azimuth speed: 45 deg/s
    - elevation speed: 16 deg/s
    - repeatability: 0.1 deg
    """

    def __init__(
        self,
        port: str = "MOCK_PORT",
        slew_rate: Optional[float] = None,
        az_base: float = 90.0,
        az_slew_rate: float = 45.0,
        el_slew_rate: float = 16.0,
        repeatability_deg: float = 0.1,
    ):
        self.port = port
        # Backward compatibility: if the old single slew_rate is provided,
        # use it for both axes.
        if slew_rate is not None:
            az_slew_rate = float(slew_rate)
            el_slew_rate = float(slew_rate)

        self.az_slew_rate = float(az_slew_rate)
        self.el_slew_rate = float(el_slew_rate)
        self.repeatability_deg = max(0.0, float(repeatability_deg))
        self.az_base = float(az_base)
        self._is_ready = False

        # Initial pose follows the system reference.
        self.curr_el = 0.0
        self.curr_az = self.az_base

        self.target_el = 0.0
        self.target_az = self.az_base

        self.last_update_time = time.time()

    def connect(self) -> bool:
        print(
            f"[MockGimbal] connected (port={self.port}, "
            f"az_rate={self.az_slew_rate:.1f}deg/s, "
            f"el_rate={self.el_slew_rate:.1f}deg/s, "
            f"repeatability={self.repeatability_deg:.2f}deg)"
        )
        self._is_ready = True
        self.last_update_time = time.time()
        return True

    def wait_ready(self, timeout: float = 5.0) -> bool:
        return self._is_ready

    def _snap_to_repeatability(self, angle: float) -> float:
        if self.repeatability_deg <= 0.0:
            return float(angle)
        return round(float(angle) / self.repeatability_deg) * self.repeatability_deg

    def set_attitude(self, elevation: float, azimuth: float, read_status: bool = False):
        self.target_el = self._snap_to_repeatability(elevation)
        self.target_az = self._snap_to_repeatability(azimuth % 360.0) % 360.0

    def get_attitude(self) -> Optional[Tuple[float, float, float]]:
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        step_el = self.el_slew_rate * dt
        step_az = self.az_slew_rate * dt

        if abs(self.target_el - self.curr_el) <= step_el:
            self.curr_el = self.target_el
        else:
            self.curr_el += step_el if self.target_el > self.curr_el else -step_el

        diff_az = (self.target_az - self.curr_az + 180.0) % 360.0 - 180.0
        if abs(diff_az) <= step_az:
            self.curr_az = self.target_az
        else:
            self.curr_az += step_az if diff_az > 0 else -step_az

        self.curr_el = self._snap_to_repeatability(self.curr_el)
        self.curr_az = self._snap_to_repeatability(self.curr_az % 360.0) % 360.0
        return (self.curr_el, self.curr_az, 0.0)

    def close(self):
        print("[MockGimbal] closed")
        self._is_ready = False
