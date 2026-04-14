import serial
import struct
import time

class SDDMLaser:
    """
    SDDM 激光测距模块驱动
    基于《SDDM 激光测距模块 说明书》
    """
    def __init__(self, port, baudrate=115200, timeout=0.1):
        """
        初始化激光模块
        :param port: 串口号 (如 'COM4'、'\\\\.\\COM10' 或 '/dev/ttyUSB1')
        :param baudrate: 波特率，默认 115200 
        :param timeout: 读取超时时间
        """
        self.ser = None
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
        except Exception as e:
            raise RuntimeError(f"无法打开激光串口 {port}: {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.stop_measurement() # 尝试发送停止指令
            self.ser.close()

    def _calculate_crc(self, data: bytes) -> int:
        """
        CRC校验算法：将整条消息当做U8数组进行累加后，取低8位 
        """
        return sum(data) & 0xFF

    def _send_command(self, msg_code, mea_type, mea_times):
        """
        构造并发送指令 (0xFA 开头) [cite: 69]
        """
        # 协议头 (0xFA), MsgCode (0x01), BrdId (0xFF 广播), PayloadLen (0x04)
        # Payload: MeaType (2 bytes), MeaTimes (2 bytes) - 小端模式 
        
        # 构造 Payload 之前的部分
        header = bytearray([0xFA, msg_code, 0xFF, 0x04])
        
        # 构造 Payload (4 bytes)
        payload = struct.pack("<HH", mea_type, mea_times)
        
        # 计算 CRC (Header + Payload)
        frame_no_crc = header + payload
        crc = self._calculate_crc(frame_no_crc)
        
        # 完整帧
        full_frame = frame_no_crc + bytes([crc])
        
        self.ser.write(full_frame)
        self.ser.flush()

    def start_measurement(self, continuous=True):
        """
        启动测量 [cite: 74, 76]
        :param continuous: True为连续测量，False为单次测量
        """
        # MeaType: 1 (启动测量)
        # MeaTimes: 0 (无限次/连续) 或 1 (单次)
        mea_type = 0x0001
        mea_times = 0x0000 if continuous else 0x0001
        
        # 发送: FA 01 FF 04 01 00 00 00 FF (连续) [cite: 78]
        self._send_command(0x01, mea_type, mea_times)
        print("[Laser] 发送启动测量指令")

    def stop_measurement(self):
        """
        停止测量 [cite: 74, 79]
        """
        # MeaType: 0 (停止测量)
        # MeaTimes: 0
        # 发送: FA 01 FF 04 00 00 00 00 FE [cite: 79]
        self._send_command(0x01, 0x0000, 0x0000)
        print("[Laser] 发送停止测量指令")

    def read_distance(self, debug=False):
            """
            读取并解析一次测量数据
            :param debug: 是否打印原始数据用于调试
            """
            if not self.ser or not self.ser.is_open:
                return None

            try:
                # 寻找帧头 FB
                for _ in range(50): 
                    b = self.ser.read(1)
                    if len(b) == 0: return None
                    
                    if b[0] == 0xFB: 
                        # 读取剩余 8 字节
                        rest_data = self.ser.read(8)
                        if len(rest_data) < 8: return None
                        
                        full_frame = b + rest_data
                        
                        # --- 调试打印 (核心修改) ---
                        if debug:
                            print(f"[Laser RAW] {full_frame.hex()}")
                        # -------------------------

                        # 校验 CRC
                        if self._calculate_crc(full_frame[:-1]) != full_frame[8]:
                            if debug: print("CRC 校验失败")
                            return None
                        
                        # 解析
                        valid_flag, distance_dm = struct.unpack("<HH", full_frame[4:8])
                        
                        if valid_flag == 1:
                            return distance_dm / 10.0
                        else:
                            if debug: print(f"数据无效标志: {valid_flag} (可能处于盲区)")
                            return -1.0
                            
            except Exception as e:
                print(f"[Laser] Error: {e}")
                return None
            
            return None
