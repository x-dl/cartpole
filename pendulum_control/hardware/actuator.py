# pendulum_control/hardware/actuator.py
# -*- coding: utf-8 -*-
import serial
import time
import struct
import math

class Actuator:
    """
    代表一个执行器电机并处理所有与其相关的通信。
    该类现在包含基于车轮半径的线位移和线速度计算。
    """
    # --- 类常量 ---
    FRAME_HEADER = 0xAA
    FRAME_TAIL = 0x2F
    CMD_GET_STATE = 0x06
    CMD_SET_CURRENT = 0x06
    EXPECTED_RESPONSE_LENGTH = 16

    def __init__(self, port: str, baudrate: int, axis_id: int, timeout: float = 0.004, wheel_radius_mm: float = 32.5):
        if not 0 <= axis_id <= 3:
            raise ValueError("Axis ID must be between 0 and 3.")
        self.port = port
        self.baudrate = baudrate
        self.axis_id = axis_id
        self.timeout = timeout
        self.wheel_radius_m = wheel_radius_mm / 1000.0
        
        self.ser = None
        self.is_connected = False
        
        self.position = 0.0
        self.current = 0.0
        self.last_position = None
        self.distance_traveled_cm = 0.0
        self.last_time = None
        self.velocity_cmps = 0.0

    def connect(self):
        """连接到串口并重置状态变量。"""
        if self.is_connected:
            print("Actuator is already connected.")
            return
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.is_connected = True
            self.last_position = None
            self.distance_traveled_cm = 0.0
            self.last_time = None
            self.velocity_cmps = 0.0
            print(f"Successfully opened serial port {self.port} for axis {self.axis_id}.")
        except serial.SerialException as e:
            self.is_connected = False
            print(f"Error opening serial port for actuator: {e}")
            raise

    def disconnect(self):
        """断开与串口的连接。"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.is_connected = False
            print(f"Actuator serial port {self.port} for axis {self.axis_id} closed.")

    def disable(self):
        """禁用电机（通过设置电流为0）。"""
        print("Disabling motor by setting current to 0.")
        self.set_current(0)
    
    # ... 其余方法 (_create_command, _parse_state_response, etc.) 保持不变 ...
    # 为了简洁，此处省略了您已提供的、无需修改的代码
    def _create_command(self, mode: str, current: float = 0.0) -> bytes:
        if mode == 'write':
            current_clamped = max(-10.0, min(10.0, current))
            rw_flag = 0x01
            command_id_val = self.CMD_SET_CURRENT
            current_bytes = struct.pack('<f', current_clamped)
            payload = bytearray([(rw_flag << 7) | (self.axis_id << 5) | command_id_val])
            payload.extend(bytearray(4))
            payload.extend(current_bytes)
            payload.extend(bytearray(4))
        else:  # 'read'
            rw_flag = 0x00
            command_id_val = self.CMD_GET_STATE
            payload = bytearray([(rw_flag << 7) | (self.axis_id << 5) | command_id_val])
            payload.extend(bytearray(12))
        
        checksum = sum(payload) & 0xFF
        command = bytearray([self.FRAME_HEADER])
        command.extend(payload)
        command.append(checksum)
        command.append(self.FRAME_TAIL)
        return bytes(command)

    def _parse_state_response(self, data: bytes) -> bool:
        if len(data) != self.EXPECTED_RESPONSE_LENGTH or data[0] != self.FRAME_HEADER or data[-1] != self.FRAME_TAIL:
            return False
        payload = data[1:-2]
        received_checksum = data[-2]
        calculated_checksum = sum(payload) & 0xFF
        if calculated_checksum != received_checksum:
            return False
        try:
            current_time = time.monotonic()
            pos, current = struct.unpack('<ff', payload[1:9])
            self.position = pos
            self.current = current
            if self.last_position is None:
                self.last_position = self.position
                self.last_time = current_time
            else:
                angular_change_deg = self.position - self.last_position
                if angular_change_deg > 180: angular_change_deg -= 360
                elif angular_change_deg < -180: angular_change_deg += 360
                angular_change_deg = -angular_change_deg
                angular_change_rad = math.radians(angular_change_deg)
                incremental_distance_m = self.wheel_radius_m * angular_change_rad
                incremental_distance_cm = incremental_distance_m * 100
                self.distance_traveled_cm += incremental_distance_cm
                if self.last_time is not None:
                    delta_t = current_time - self.last_time
                    if delta_t > 0: self.velocity_cmps = incremental_distance_cm / delta_t
                self.last_position = self.position
                self.last_time = current_time
            return True
        except struct.error:
            return False

    def send_command(self, command: bytes) -> bytes:
        if not self.is_connected:
            print("Error: Not connected. Cannot send command.")
            return b''
        self.ser.write(command)
        self.ser.flush()
        return self.ser.read(self.EXPECTED_RESPONSE_LENGTH)

    def set_current(self, current: float) -> bool:
        command = self._create_command(mode='write', current=current)
        response = self.send_command(command)
        return self._parse_state_response(response)

    def get_state(self) -> bool:
        command = self._create_command(mode='read')
        response = self.send_command(command)
        return self._parse_state_response(response)