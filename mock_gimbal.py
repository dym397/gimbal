import time
from typing import Tuple, Optional
from gimbal_interface import GimbalBase #

class MockGimbalAdapter(GimbalBase):
    """
    绾蒋浠舵ā鎷熶簯鍙伴┍鍔紝鐢ㄤ簬 SITL 闂幆娴嬭瘯銆?
    """
    def __init__(self, port: str = "MOCK_PORT", slew_rate: float = 60.0, az_base: float = 90.0):
        self.port = port
        self.slew_rate = slew_rate  # 妯℃嫙浜戝彴鏈€澶ц浆閫?(搴?绉?
        self.az_base = float(az_base)
        self._is_ready = False
        
        # 鍒濆鐘舵€佽涓虹郴缁熷熀鍑嗘按骞筹紙姘村钩90搴︼紝淇话0搴︼級
        self.curr_el = 0.0
        self.curr_az = self.az_base
        
        self.target_el = 0.0
        self.target_az = self.az_base
        
        self.last_update_time = time.time()

    def connect(self) -> bool:
        print(f"[MockGimbal] connected (port={self.port})")
        self._is_ready = True
        self.last_update_time = time.time()
        return True

    def wait_ready(self, timeout: float = 5.0) -> bool:
        return self._is_ready

    def set_attitude(self, elevation: float, azimuth: float, read_status: bool = False):
        self.target_el = elevation
        self.target_az = azimuth % 360.0
        # print(f"[MockGimbal] 鏀跺埌杩愬姩鎸囦护 -> 鐩爣 El: {self.target_el:.1f}, Az: {self.target_az:.1f}")

    def get_attitude(self) -> Optional[Tuple[float, float, float]]:
        # 1. 璁＄畻璺濈涓婃鏌ヨ缁忚繃鐨勬椂闂?
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        
        # 2. 妯℃嫙鐗╃悊杩愬姩 (绠€鍗曠殑鍖€閫熼€艰繎)
        step = self.slew_rate * dt
        
        # 淇话瑙?(El) 閫艰繎
        if abs(self.target_el - self.curr_el) <= step:
            self.curr_el = self.target_el
        else:
            self.curr_el += step if self.target_el > self.curr_el else -step
            
        # 鏂逛綅瑙?(Az) 閫艰繎 (闇€澶勭悊 360 搴︾幆缁?
        diff_az = (self.target_az - self.curr_az + 180.0) % 360.0 - 180.0
        if abs(diff_az) <= step:
            self.curr_az = self.target_az
        else:
            self.curr_az += step if diff_az > 0 else -step
            
        self.curr_az = self.curr_az % 360.0
        
        # 杩斿洖绗﹀悎鎺ュ彛鏍囧噯鐨勫厓缁?
        return (self.curr_el, self.curr_az, 0.0)

    def close(self):
        print("[MockGimbal] closed")
        self._is_ready = False
