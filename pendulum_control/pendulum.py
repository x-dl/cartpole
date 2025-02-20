# pendulum_control/pendulum.py (最终精确复刻版)
# ... (文件头部和 __init__ 等保持不变，确保之前的修复都存在) ...
# ... (特别是 __init__.py 的修复) ...
import time
import math
import statistics # 用于计算平均值
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
            target_rad=self.upright_angle_rad, # 使用理论垂直点
            sample_time=1.0 / self.main_loop_freq,
            output_limits=self.angle_pid_params['output_limits']
        )
        self.pos_pid = PositionPIDController(
            Kp=self.pos_pid_params['kp'], Ki=self.pos_pid_params['ki'], Kd=self.pos_pid_params['kd'],
            setpoint=self.pos_pid_params['target_pos_cm'],
            output_limits=self.pos_pid_params['output_limits_rad']
        )
        # *** 新增：用于存储校准偏移的变量 ***
        self.gravity_offset_rad = 0.0
        self._control_loop_thread = None
        self._log_file = None
        self._log_writer = None
        self._program_start_time = 0
        self._last_control_mode = "Startup"
        self._transition_timer = 0
    
    def _unpack_config(self, config):
        self.main_loop_freq = config.get('main_loop_freq', 200.0)
        self.pos_loop_freq = config.get('pos_loop_freq', 40.0)
         # *** 解包自校准相关的配置 ***
        self.calibration_params = config.get('calibration', {})
        self.auto_calibrate = self.calibration_params.get('enabled', False)
        self.calibration_duration_s = self.calibration_params.get('duration_s', 3.0)
        self.calibration_prompt = self.calibration_params.get('prompt', "请保持摆杆完全静止...")      
        # *** 将原始的 upright_angle_rad 作为理论值保存 ***
        self.upright_angle_rad = config.get('upright_angle_rad', math.pi)
        self.upright_threshold_rad = config.get('upright_threshold_rad', math.radians(20.0))
        self.swingup_gain = config.get('swingup_gain', 0.4)
        self.slope_ratio = config.get('slope_ratio', 0.4)
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
    # *** 校准方法 ***
    def _calibrate_zero_angle(self):
        if not self.auto_calibrate:
            print("自动校准已禁用。")
            self.gravity_offset_rad = 0.0
            return

        print("\n--- 开始自动水平校准 ---")
        # ... (提示和数据采集部分不变) ...
        print(self.calibration_params.get('prompt', "请保持摆杆完全静止..."))
        angle_readings = []
        start_time = time.monotonic()
        while time.monotonic() - start_time < self.calibration_params.get('duration_s', 3.0):
            angle_rad, _, _ = self.encoder.get_data()
            if angle_rad is not None: angle_readings.append(angle_rad)
            remaining_time = self.calibration_params.get('duration_s', 3.0) - (time.monotonic() - start_time)
            print(f"校准中... 剩余 {remaining_time:.1f} 秒", end='\r')
            time.sleep(0.01)

        print("\n校准数据采集完成。")
        if not angle_readings:
            print("警告：校准期间未能读取到角度数据。无偏移校正。")
            self.gravity_offset_rad = 0.0
            return

        # 计算并存储静止偏移量
        self.gravity_offset_rad = statistics.mean(angle_readings)
        
        print(f"检测到静止角度偏移: {math.degrees(self.gravity_offset_rad):.2f}°")
        print("后续所有角度读数将自动减去此偏移。")
        print("--- 校准完成 ---")


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
        """启动倒立摆控制系统。"""
        print("--- Starting Inverted Pendulum Control System ---")
        try:
            # 1. 初始化硬件
            self.motor.connect()
            self.encoder.start()
            self._setup_logging()

            if not self.encoder.is_running or not self.motor.is_connected:
                raise RuntimeError("Failed to initialize hardware.")

            # *** 在主循环前执行校准 ***
            self._calibrate_zero_angle()

            self.is_running = True
            self._program_start_time = time.monotonic()
            
            # 2. 运行主控制循环
            self._control_loop()

        except (KeyboardInterrupt, RuntimeError) as e:
            print(f"\nSystem stop requested: {e}")
        finally:
            # 3. 清理资源
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
        """主控制循环 - 使用校准后的目标角度"""
        # ... (循环初始化变量不变) ...
        target_interval = 1.0 / self.main_loop_freq
        pos_loop_ratio = int(self.main_loop_freq / self.pos_loop_freq)
        loop_counter = 0
        angle_offset_rad = 0.0 # 位置环输出值
        stable_offset_rad = 0.0 # 稳定补偿值


        print(f"\n控制循环已启动 (已应用角度偏移: {-math.degrees(self.gravity_offset_rad):.2f}°)... (按 Ctrl+C 停止)")
        
        while self.is_running:
            # ... (循环内部逻辑不变，但所有与目标角度的比较都将使用新值) ...
            loop_start_time = time.monotonic()
            
            # *** 在数据源头进行校正 ***
            raw_angle_rad, raw_speed_rad_s, _ = self.encoder.get_data()
            if raw_angle_rad is None: continue
            
            # 1. 计算修正后的角度，后续所有逻辑都使用这个
            angle_rad = raw_angle_rad - self.gravity_offset_rad
            speed_rad_s = raw_speed_rad_s # 速度不受静态偏移影响

            # 2. 所有后续计算都基于理论值和修正后的角度
            normalized_error = (angle_rad - self.upright_angle_rad + math.pi) % (2 * math.pi) - math.pi
            # ... (状态切换逻辑完全不变) ...
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

            
            if current_mode == "Swing-Up":
                # 起摆控制器现在接收的是修正后的角度，其能量模型恢复正常
                target_current = self._swing_up_controller(angle_rad , speed_rad_s)
                target_current -= self.gravity_offset_rad * self.slope_ratio
            else:
                if loop_counter % pos_loop_ratio == 0:
                    angle_offset_rad = self.pos_pid.compute(self.motor.distance_traveled_cm)
                
                # 目标角度始终是理论垂直点，加上动态补偿
                if current_mode == "PID":
                    stable_offset_rad = 0.0
                    slope_offset_rad = -self.gravity_offset_rad * self.slope_ratio * 0.01
                    # ... (稳定补偿逻辑不变) ...
                    if abs(speed_rad_s) < self.stable_offset_speed_threshold_rad_s:
                        if self.motor.velocity_cmps > 0: stable_offset_rad = self.stable_offset_angle_rad
                        elif self.motor.velocity_cmps < 0: stable_offset_rad = -self.stable_offset_angle_rad
                    new_target_angle_rad = self.upright_angle_rad + angle_offset_rad + stable_offset_rad + slope_offset_rad
                else: # Transition 模式
                    new_target_angle_rad = self.upright_angle_rad # 目标是理论垂直点
                
                self.angle_pid.target_position_rad = new_target_angle_rad
                # PID控制器接收的也是修正后的角度
                target_current = self.angle_pid.update(angle_rad, speed_rad_s, kd_override=kd_for_update)
            
            # ... (循环的剩余部分，如发送指令和日志，保持不变) ...
            self._last_control_mode = current_mode
            if self.motor.set_current(-target_current):
                if loop_counter % 10 == 0:
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