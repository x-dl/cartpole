# pendulum_control/hardware/encoder.py
# -*- coding: utf-8 -*-
import time
import threading
import math
from collections import deque
import serial

class EncoderReader:
    """
    一个在独立线程中读取串口编码器的类，以确保非阻塞的数据采集。
    此优化版本将所有计算移至后台线程，使得 get_data() 方法调用极为迅速。
    它使用 pyserial 进行通信。
    """
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200, 
                 encoder_resolution=16384, speed_calc_window_s=0.05):
        self.port = port
        self.baudrate = baudrate
        self.encoder_resolution = encoder_resolution
        
        self._lock = threading.Lock()
        self._shutdown_event = threading.Event()
        self._reader_thread = None
        self.is_running = False
        
        self._angle_rad = 0.0
        self._speed_rad_s = 0.0
        self._frames_per_second = 0.0

        self._angle_buffer = deque()
        self._time_buffer = deque()
        self.speed_calc_window_s = speed_calc_window_s

        self._frame_counter = 0
        self._last_fps_time = 0.0

    # ... 所有方法 (_reader_loop, start, stop, get_data) 保持不变 ...
    # 为了简洁，此处省略了您已提供的、无需修改的代码
    def _reader_loop(self):
        print(f"[Encoder Thread] Trying to open serial port: {self.port}...")
        ser = None
        try:
            ser = serial.Serial(
                port=self.port, baudrate=self.baudrate, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1
            )
        except serial.SerialException as e:
            print(f"[Encoder Thread] Error: Could not open port {self.port}. {e}")
            return
        print("[Encoder Thread] Port opened successfully. Starting read loop.")
        self._last_fps_time = time.time()
        internal_buffer = bytearray()
        while not self._shutdown_event.is_set():
            try:
                if ser.in_waiting > 0:
                    internal_buffer.extend(ser.read(ser.in_waiting))
                while len(internal_buffer) >= 9:
                    if internal_buffer[0] == 0 and internal_buffer[1] == 3 and internal_buffer[2] == 4:
                        frame = internal_buffer[:9]
                        del internal_buffer[:9]
                        position_bytes = frame[5:7]
                        current_raw_value = int.from_bytes(position_bytes, 'big')
                        current_angle = (current_raw_value / self.encoder_resolution) * (2 * math.pi)
                        if current_angle > math.pi: current_angle -= 2 * math.pi
                        current_time = time.time()
                        self._angle_buffer.append(current_angle)
                        self._time_buffer.append(current_time)
                        while self._time_buffer and (current_time - self._time_buffer[0]) > self.speed_calc_window_s:
                            self._angle_buffer.popleft()
                            self._time_buffer.popleft()
                        current_speed = 0.0
                        if len(self._angle_buffer) >= 2:
                            time_span = self._time_buffer[-1] - self._time_buffer[0]
                            if time_span > 0:
                                angle_span = self._angle_buffer[-1] - self._angle_buffer[0]
                                if angle_span > math.pi: angle_span -= 2 * math.pi
                                elif angle_span < -math.pi: angle_span += 2 * math.pi
                                current_speed = angle_span / time_span
                        with self._lock:
                            self._angle_rad = current_angle
                            self._speed_rad_s = current_speed
                            self._frame_counter += 1
                    else:
                        del internal_buffer[:1]
                current_time_fps = time.time()
                if current_time_fps - self._last_fps_time >= 1.0:
                    with self._lock:
                        self._frames_per_second = self._frame_counter / (current_time_fps - self._last_fps_time)
                        self._frame_counter = 0
                    self._last_fps_time = current_time_fps
                time.sleep(0.001)
            except serial.SerialException as e:
                print(f"[Encoder Thread] A serial error occurred: {e}")
                ser.close(); time.sleep(1)
                try: ser.open()
                except: print("[Encoder Thread] Failed to reopen port. Terminating."); break
            except Exception as e:
                print(f"[Encoder Thread] An unexpected error occurred: {e}"); time.sleep(0.05)
        if ser and ser.is_open: ser.close()
        print("[Encoder Thread] Serial port closed. Thread finished.")

    def start(self):
        if self.is_running: print("[EncoderReader] Reader is already running."); return
        print("[EncoderReader] Starting reader thread...")
        self._shutdown_event.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        self.is_running = True
        time.sleep(0.5)
        print("[EncoderReader] Reader thread started.")

    def stop(self):
        if not self.is_running: print("[EncoderReader] Reader is not running."); return
        print("[EncoderReader] Stopping reader thread...")
        self._shutdown_event.set()
        self._reader_thread.join(timeout=2)
        self.is_running = False
        print("[EncoderReader] Reader thread stopped.")

    def get_data(self):
        if not self.is_running: return 0.0, 0.0, 0.0
        with self._lock:
            return self._angle_rad, self._speed_rad_s, self._frames_per_second