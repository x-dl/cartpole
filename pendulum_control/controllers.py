# pendulum_control/controllers.py
import time
import math

class AnglePIDController:
    """
    一个为倒立摆角度控制优化的PID控制器。
    - 处理角度的卷绕（wrap-around）问题。
    - 使用直接的速度输入作为微分项，以减少噪声。
    - 包含积分抗饱和（anti-windup）逻辑。
    """
    def __init__(self, Kp, Ki, Kd, target_rad=0, sample_time=0.01, output_limits=(-1.0, 1.0)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target_position_rad = target_rad
        self.sample_time = sample_time
        self.min_output, self.max_output = output_limits
        
        self.P_term = 0
        self.I_term = 0
        self.D_term = 0
        self._integral = 0

    def _normalize_error(self, error_rad):
        """将角度误差归一化到 [-pi, pi] 范围内。"""
        return (error_rad + math.pi) % (2 * math.pi) - math.pi

    def update(self, current_angle_rad, current_speed_rad_s, kd_override=None):
        """根据当前系统状态计算PID输出。"""
        error_rad = self.target_position_rad - current_angle_rad
        error_rad = self._normalize_error(error_rad)
        
        self.P_term = self.Kp * error_rad
        
        self._integral += error_rad * self.sample_time
        self._integral = max(min(self._integral, self.max_output), self.min_output)
        self.I_term = self.Ki * self._integral
        
        effective_kd = self.Kd if kd_override is None else kd_override
        # 微分项作用于速度的负反馈，而不是误差的微分，以避免“微分冲击”
        self.D_term = effective_kd * (-current_speed_rad_s)
        
        output = self.P_term + self.I_term + self.D_term
        return max(min(output, self.max_output), self.min_output)

    def set_target(self, new_target_rad):
        """更新目标位置并重置控制器状态。"""
        self.target_position_rad = new_target_rad
        self.reset()
    
    def reset(self):
        """重置积分器和内部状态。"""
        self._integral = 0
        self.P_term = 0
        self.I_term = 0
        self.D_term = 0

class PositionPIDController:
    """
    一个用于小车位置环的PID控制器。
    - 从位置误差计算微分项。
    """
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, output_limits=(-1.0, 1.0)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.min_output, self.max_output = output_limits
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = time.monotonic()

    def compute(self, pos_actual):
        """根据当前实际位置计算控制输出。"""
        current_time = time.monotonic()
        dt = current_time - self.previous_time
        
        if dt <= 0: return 0.0

        error = self.setpoint - pos_actual
        
        p_term = self.Kp * error

        self.integral += error * dt
        # 简单的积分抗饱和
        if self.min_output is not None:
             self.integral = max(self.integral, self.min_output / (self.Ki + 1e-9))
        if self.max_output is not None:
             self.integral = min(self.integral, self.max_output / (self.Ki + 1e-9))
        i_term = self.Ki * self.integral

        derivative = (error - self.previous_error) / dt
        d_term = self.Kd * derivative

        output = p_term + i_term + d_term

        if self.min_output is not None:
            output = max(output, self.min_output)
        if self.max_output is not None:
            output = min(output, self.max_output)

        self.previous_error = error
        self.previous_time = current_time

        return output

    def set_target(self, setpoint):
        """更新目标位置并重置。"""
        self.setpoint = setpoint
        self.reset()

    def reset(self):
        """重置积分项和误差历史。"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = time.monotonic()