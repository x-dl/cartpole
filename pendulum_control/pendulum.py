# pendulum_control/pendulum.py (最终精确复刻版)

# ... (文件头部和 __init__ 等保持不变，确保之前的修复都存在) ...
# ... (特别是 __init__.py 的修复) ...
import time
import math
import csv
import datetime
from .hardware.actuator import Actuator
from .hardware.encoder import EncoderReader
from .controllers import AnglePIDController, PositionPIDController

class InvertedPendulum:
    """
    倒立摆控制系统的核心类。
    封装了硬件接口、控制器和主控制循环。
    """
    # __init__ 和 _unpack_config 方法使用上一个版本修复后的代码即可，它已经包含了 stable_offset 的配置。
    def __init__(self, motor: Actuator, encoder: EncoderReader, config: dict):
        self.motor = motor
        self.encoder = encoder
        self.config = config
        self.is_running = False
        
        self._unpack_config(config)

        self.angle_pid = AnglePIDController(
            Kp=self.angle_pid_params['kp'], Ki=self.angle_pid_params['ki'], Kd=self.angle_pid_params['kd'],
            target_rad=self.upright_angle_rad, sample_time=1.0 / self.main_loop_freq,
            output_limits=self.angle_pid_params['output_limits']
        )
        self.pos_pid = PositionPIDController(
            Kp=self.pos_pid_params['kp'], Ki=self.pos_pid_params['ki'], Kd=self.pos_pid_params['kd'],
            setpoint=self.pos_pid_params['target_pos_cm'],
            output_limits=self.pos_pid_params['output_limits_rad']
        )

        self._control_loop_thread = None
        self._log_file = None
        self._log_writer = None
        self._program_start_time = 0
        self._last_control_mode = "Startup"
        self._transition_timer = 0
    
    def _unpack_config(self, config):
        self.main_loop_freq = config.get('main_loop_freq', 200.0)
        self.pos_loop_freq = config.get('pos_loop_freq', 40.0)
        self.upright_angle_rad = config.get('upright_angle_rad', math.pi)
        self.upright_threshold_rad = config.get('upright_threshold_rad', math.radians(20.0))
        self.swingup_gain = config.get('swingup_gain', 0.4)
        self.swingup_max_current = config.get('swingup_max_current', 1.0)
        self.transition_brake_kd = config.get('transition_brake_kd', 2.5)
        self.transition_duration_s = config.get('transition_duration_s', 0.2)
        self.angle_pid_params = config['angle_pid']
        self.pos_pid_params = config['pos_pid']
        self.enable_logging = config.get('enable_logging', True)
        self.stable_offset_params = config.get('stable_offset', {})
        self.stable_offset_enabled = self.stable_offset_params.get('enabled', False)
        self.stable_offset_speed_threshold_rad_s = self.stable_offset_params.get('speed_threshold_rad_s', 10.0)
        self.stable_offset_angle_rad = self.stable_offset_params.get('offset_angle_rad', math.radians(1.5))
    
    # ... 其他辅助方法保持不变 ...
    def _setup_logging(self):
        """初始化CSV日志文件。"""
        if not self.enable_logging: return
        try:
            filename = f"pendulum_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self._log_file = open(filename, 'w', newline='', encoding='utf-8')
            self._log_writer = csv.writer(self._log_file)
            header = [
                'timestamp_s', 'control_mode', 'angle_deg', 'speed_dps', 'norm_error_deg',
                'target_current_A', 'motor_current_A', 'P_term', 'I_term', 'D_term',
                'cart_pos_cm', 'cart_vel_cmps', 'pos_loop_output_deg', 'stable_offset_deg'
            ]
            self._log_writer.writerow(header)
            print(f"Logging data to: {filename}")
        except IOError as e:
            print(f"Error setting up logger: {e}"); self._log_writer = None

    def _swing_up_controller(self, angle_rad, speed_rad_s):
        """基于能量的起摆控制器。"""
        energy_to_go = 1 + math.cos(angle_rad)
        control_signal = -1 * self.swingup_gain * energy_to_go * math.copysign(1, speed_rad_s)
        return max(min(control_signal, self.swingup_max_current), -self.swingup_max_current)

    def run(self):
        print("--- Starting Inverted Pendulum Control System ---")
        try:
            self.motor.connect()
            self.encoder.start()
            self._setup_logging()
            if not self.encoder.is_running or not self.motor.is_connected:
                raise RuntimeError("Failed to initialize hardware.")
            self.is_running = True
            self._program_start_time = time.monotonic()
            self._control_loop()
        except (KeyboardInterrupt, RuntimeError) as e:
            print(f"\nSystem stop requested: {e}")
        finally:
            self.stop()
    
    def stop(self):
        print("\n--- Shutting Down System ---")
        self.is_running = False
        if self.motor and self.motor.is_connected: self.motor.disable(); self.motor.disconnect()
        if self.encoder and self.encoder.is_running: self.encoder.stop()
        if self._log_file: self._log_file.close(); print("Log file closed.")
        print("Cleanup complete. System stopped.")
    
    # *** 1:1 复刻循环逻辑 ***
    def _control_loop(self):
        """主控制循环 - 精确复刻版"""
        target_interval = 1.0 / self.main_loop_freq
        pos_loop_ratio = int(self.main_loop_freq / self.pos_loop_freq)
        loop_counter = 0
        angle_offset_rad = 0.0 # 位置环输出值
        stable_offset_rad = 0.0 # 稳定补偿值

        print(f"Control loop started. Main Freq: {self.main_loop_freq}Hz, Pos Freq: {self.pos_loop_freq}Hz.")
        print("Press Ctrl+C to stop.")

        while self.is_running:
            loop_start_time = time.monotonic()
            
            angle_rad, speed_rad_s, _ = self.encoder.get_data()
            if angle_rad is None: continue
            
            # --- 状态切换逻辑 (与原始代码完全一致) ---
            normalized_error = (angle_rad - self.upright_angle_rad + math.pi) % (2 * math.pi) - math.pi
            current_mode = self._last_control_mode
            kd_for_update = None
            is_in_pid_zone = abs(normalized_error) < self.upright_threshold_rad

            if self._transition_timer > 0:
                current_mode = "Transition"
                kd_for_update = self.transition_brake_kd
                self._transition_timer -= target_interval
                if self._transition_timer <= 0:
                    current_mode = "PID"
            elif is_in_pid_zone:
                if self._last_control_mode not in ["PID", "Transition"]:
                    print("\nEntering PID zone, starting transition brake...")
                    self.angle_pid.reset()
                    current_mode = "Transition"
                    kd_for_update = self.transition_brake_kd
                    self._transition_timer = self.transition_duration_s
                else:
                    current_mode = "PID"
            else:
                current_mode = "Swing-Up"
            
            # --- 计算目标电流 (与原始代码完全一致) ---
            if current_mode == "Swing-Up":
                target_current = self._swing_up_controller(angle_rad, speed_rad_s)
            else: # PID 和 Transition 模式
                # 1. 位置环计算
                if loop_counter % pos_loop_ratio == 0:
                    angle_offset_rad = self.pos_pid.compute(self.motor.distance_traveled_cm)
                
                # 2. 稳定补偿计算和目标角度设定
                if current_mode == "PID":
                    # 计算稳定补偿
                    stable_offset_rad = 0.0 # 每轮循环重置
                    if abs(speed_rad_s) < self.stable_offset_speed_threshold_rad_s:
                        if self.motor.velocity_cmps > 0:
                            stable_offset_rad = self.stable_offset_angle_rad
                        elif self.motor.velocity_cmps < 0:
                            stable_offset_rad = -self.stable_offset_angle_rad
                    # 设定PID模式下的目标角度
                    new_target_angle_rad = self.upright_angle_rad + angle_offset_rad + stable_offset_rad
                else: # Transition 模式
                    # Transition模式下无任何补偿
                    new_target_angle_rad = self.upright_angle_rad
                
                # 3. 更新PID控制器并计算输出
                self.angle_pid.target_position_rad = new_target_angle_rad # 直接更新目标
                target_current = self.angle_pid.update(angle_rad, speed_rad_s, kd_override=kd_for_update)
            
            self._last_control_mode = current_mode
            
            # --- 发送指令和日志记录 (与原始代码一致) ---
            if self.motor.set_current(-target_current):
                if loop_counter % 10 == 0: # 降低打印频率，避免刷屏
                    self._print_status(current_mode, angle_rad, speed_rad_s, normalized_error, target_current, angle_offset_rad, stable_offset_rad)
                if self._log_writer:
                    self._log_data(loop_start_time, current_mode, angle_rad, speed_rad_s, normalized_error, target_current, angle_offset_rad, stable_offset_rad)
            else:
                print(f"Failed to send command. Target: {-target_current:.4f} A", end='\r')

            loop_counter += 1
            loop_elapsed = time.monotonic() - loop_start_time
            sleep_time = target_interval - loop_elapsed
            if sleep_time > 0: time.sleep(sleep_time)

    def _print_status(self, mode, angle, speed, error, current, pos_offset, stable_offset):
        # ... 打印函数保持不变 ...
        status_str = (
            f"Mode: {mode:<10} | Angle: {math.degrees(angle):>7.2f}° | "
            f"Cart Vel: {self.motor.velocity_cmps:>6.1f}cm/s | "
            f"Target I: {current:>7.4f}A"
        )
        print(status_str, end='\r')
    
    def _log_data(self, timestamp, mode, angle, speed, error, current, pos_offset, stable_offset):
        # ... 日志函数保持不变 ...
        self._log_writer.writerow([
            timestamp - self._program_start_time, mode,
            math.degrees(angle), math.degrees(speed), math.degrees(error),
            current, self.motor.current,
            self.angle_pid.P_term, self.angle_pid.I_term, self.angle_pid.D_term,
            self.motor.distance_traveled_cm, self.motor.velocity_cmps,
            math.degrees(pos_offset), math.degrees(stable_offset)
        ])