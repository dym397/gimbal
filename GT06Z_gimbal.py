import serial
import time
import struct

class GT06ZGimbal:
    def __init__(self, port: str):
        self.port = port
        self.ser = None
        self.baudrate = 9600
        self.address = 0x00 
        
        # --- 状态记录 ---
        self.last_sent_el = None
        self.last_sent_az = None
        
        # --- 优化参数 ---
        self.epsilon = 0.2          # 角度变化死区
        self.min_interval = 0.1     # 最小发送间隔(秒)
        self.last_cmd_time = 0.0

    def open(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=0.1 
            )
            
            if self.ser.is_open:
                self.ser.reset_input_buffer()
                print(f"[GT06Z] Driver Ready on {self.port}.")
            return self.ser.is_open
        except Exception as e:
            print(f"[GT06Z] Open fail: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.stop() # 退出前确保停止
            self.ser.close()
            print("[GT06Z] Connection Closed.")

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _calc_checksum(self, payload_without_head: bytes) -> int:
        return sum(payload_without_head) & 0xFF

    def _send_frame(self, cmd1, cmd2, data_val):
        """
        发送标准7字节帧
        data_val: 16位整数, 内部会自动拆分为 DataH 和 DataL
        Frame: FF [Addr] [Cmd1] [Cmd2] [DataH] [DataL] [Check]
        """
        if not self.is_connected(): return
        
        data_val = int(data_val) & 0xFFFF
        data_h = (data_val >> 8) & 0xFF
        data_l = data_val & 0xFF
        
        payload = bytes([self.address, cmd1, cmd2, data_h, data_l])
        checksum = self._calc_checksum(payload)
        
        frame = bytes([0xFF]) + payload + bytes([checksum])
        
        # Debug: 打印发送的指令（调试时可开启）
        # print(f"[Tx] {frame.hex().upper()}")
        
        self.ser.write(frame)

    def stop(self):
        """发送停止指令"""
        self._send_frame(0x00, 0x00, 0x0000)

    # =================================================================
    # 核心功能：速度配置 (Item 1 策略)
    # =================================================================
    def set_speed(self, speed: int = 0x3F):
        """
        设置全局速度寄存器。
        策略：依次向四个方向发送极短的移动指令，将速度值写入寄存器，然后停止。
        
        :param speed: 1~63 (0x01 ~ 0x3F)。63为最快。
        """
        if not self.is_connected(): return

        # 1. 限制范围 1 ~ 63
        val = max(1, min(63, int(speed)))
        print(f"[GT06Z] Configuring Speed to: {val} (Range: 1-63)")

        # 2. 构造数据
        # Item 1: Left/Right 使用 Data1 (High Byte)
        # Item 1: Up/Down    使用 Data2 (Low Byte)
        data_pan_move = (val << 8) & 0xFF00  # DataH=Speed, DataL=0
        data_tilt_move = val & 0xFF          # DataH=0, DataL=Speed

        # --- 步骤 1: 写入水平速度 (右) ---
        self._send_frame(0x00, 0x02, data_pan_move)
        time.sleep(0.02)
        self.stop()
        time.sleep(0.01)

        # --- 步骤 2: 写入水平速度 (左) ---
        self._send_frame(0x00, 0x04, data_pan_move)
        time.sleep(0.02)
        self.stop()
        time.sleep(0.01)

        # --- 步骤 3: 写入垂直速度 (上) ---
        self._send_frame(0x00, 0x08, data_tilt_move)
        time.sleep(0.02)
        self.stop()
        time.sleep(0.01)

        # --- 步骤 4: 写入垂直速度 (下) ---
        self._send_frame(0x00, 0x10, data_tilt_move)
        time.sleep(0.02)
        self.stop()
        
        # 简单延时确保生效
        time.sleep(0.05)
        print("[GT06Z] Speed Configuration Done.")

    # =================================================================
    # 核心功能：绝对角度控制 (Item 8 & 9)
    # =================================================================
    def set_angles(self, elevation_deg: float, azimuth_deg: float):
        """
        发送绝对角度指令。
        云台将使用 set_speed 最后设定的速度运行。
        """
        if not self.is_connected(): return

        # 频率控制
        now = time.time()
        if (now - self.last_cmd_time) < self.min_interval:
            return 

        # 变化量死区检查 (减少不必要的指令发送)
        need_send_el = False
        if (self.last_sent_el is None) or (abs(elevation_deg - self.last_sent_el) > self.epsilon):
            need_send_el = True

        need_send_az = False
        if self.last_sent_az is None:
            need_send_az = True
        else:
            diff = abs(azimuth_deg - self.last_sent_az)
            if diff > 180: diff = 360 - diff
            if diff > self.epsilon:
                need_send_az = True
            
        if not need_send_el and not need_send_az:
            return

        self.ser.reset_input_buffer()

        # 1. 发送垂直 (Cmd: 00 4D)
        if need_send_el:
            el_cmd_val = 0
            if elevation_deg > 0:
                el_cmd_val = 3600 - int(elevation_deg * 10) 
            else:
                el_cmd_val = int(abs(elevation_deg) * 10)   
            
            el_cmd_val = max(0, min(3600, el_cmd_val))
            self._send_frame(0x00, 0x4D, el_cmd_val)
            self.last_sent_el = elevation_deg
            
            # 双轴同时动作时增加微小间隔
            if need_send_az:
                time.sleep(0.04) 

        # 2. 发送水平 (Cmd: 00 4B)
        if need_send_az:
            norm_az = azimuth_deg % 360.0
            az_cmd_val = int(norm_az * 10) 
            az_cmd_val = max(0, min(3600, az_cmd_val))
            self._send_frame(0x00, 0x4B, az_cmd_val)
            self.last_sent_az = azimuth_deg

        self.last_cmd_time = time.time()

    def _read_specific_response(self, expected_cmd_byte, retry=2):
        if not self.is_connected(): return None
        for _ in range(retry):
            if self.ser.in_waiting < 7:
                time.sleep(0.02)
            
            # 读取足够多的字节以防粘包
            data = self.ser.read(self.ser.in_waiting or 14) 
            if len(data) < 7: continue 
            
            # 滑动窗口查找帧头 FF
            for i in range(len(data) - 6):
                if data[i] != 0xFF: continue
                frame = data[i : i+7]
                
                # 校验和
                if (sum(frame[1:6]) & 0xFF) != frame[6]: continue 
                
                # 匹配指令码
                if frame[3] == expected_cmd_byte:
                    return (frame[4] << 8) | frame[5]
        return None 

    def query_angles(self):
        """查询当前角度 (Item 10)"""
        if not self.is_connected(): return None
        self.ser.reset_input_buffer()

        # 查询垂直
        self._send_frame(0x00, 0x53, 0)
        raw_el = self._read_specific_response(expected_cmd_byte=0x5B)
        if raw_el is not None:
            if raw_el <= 1800:
                self.last_sent_el = -(raw_el / 10.0) 
            else:
                self.last_sent_el = (3600 - raw_el) / 10.0 
        
        # 查询水平
        self._send_frame(0x00, 0x51, 0)
        raw_az = self._read_specific_response(expected_cmd_byte=0x59)
        if raw_az is not None:
            self.last_sent_az = raw_az / 10.0

        if self.last_sent_el is None: self.last_sent_el = 0.0
        if self.last_sent_az is None: self.last_sent_az = 0.0
            
        return (self.last_sent_el, self.last_sent_az)

# =================================================================
# Main 测试函数
# =================================================================
# if __name__ == "__main__":
#     # 请根据实际情况修改端口号，例如 "/dev/ttyUSB0" 或 "COM3"
#     PORT_NAME = "/dev/ttyUSB0"
    
#     gimbal = GT06ZGimbal(PORT_NAME)
    
#     if gimbal.open():
#         try:
#             print("\n=== Test Start ===")
            
#             # 1. 归位 (方便观察)
#             # print(">>> Resetting to (0, 0) with default speed...")
#             # gimbal.set_speed(63)
#             # time.sleep(5)
#             # gimbal.stop() 
            
#             gimbal.set_angles(0, 0.0)
#             gimbal.set_angles(20, 90.0)
#             time.sleep(3) # 等待归位
#             print(f"Current: {gimbal.query_angles()}")

#             # ---------------------------------------------------------
#             # 测试 Case 1: 慢速移动 (Slow Motion)
#             # ---------------------------------------------------------
#             print("\n>>> [Case 1] Setting LOW SPEED (10/63)...")
#             gimbal.set_speed(10)
            
#             print(">>> Moving to (Tilt=10, Pan=30)... Observe slow movement.")
#             gimbal.set_angles(10.0, 30.0)
            
#             # 轮询打印位置，观察变化过程
#             for i in range(5):
#                 time.sleep(1)
#                 print(f"   t+{i}s: {gimbal.query_angles()}")
            
#             # ---------------------------------------------------------
#             # 测试 Case 2: 全速移动 (Full Speed)
#             # ---------------------------------------------------------
#             print("\n>>> [Case 2] Setting HIGH SPEED (63/63)...")
#             gimbal.set_speed(63)
            
#             print(">>> Moving back to (Tilt=0, Pan=0)... Observe fast movement.")
#             gimbal.set_angles(0.0, 0.0)
            
#             # 快速检查
#             for i in range(3):
#                 time.sleep(0.5)
#                 print(f"   t+{i*0.5}s: {gimbal.query_angles()}")

#             print("\n=== Test Finished ===")
            
#         except KeyboardInterrupt:
#             print("\nTest Interrupted by User.")
#         finally:
#             gimbal.close()
#     else:
#         print("Error: Could not open serial port.")
