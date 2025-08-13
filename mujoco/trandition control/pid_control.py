import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# --- 1. PID 控制器类 (保持上一版的鲁棒设计) ---
class PIDController:
    """
    一个鲁棒的PID控制器。
    - is_angle: 正确处理环形数据。
    - derivative_on_measurement: 使用“基于测量的微分”来避免“微分冲击”。
    """
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(-np.inf, np.inf), 
                 is_angle=False, derivative_on_measurement=False):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.is_angle = is_angle
        self.derivative_on_measurement = derivative_on_measurement
        self._integral = 0
        self._prev_error = 0
        self._prev_measurement = 0
        if self.Ki == 0:
            self._integral_limit_min = self._integral_limit_max = 0
        else:
            self._integral_limit_min = self.output_limits[0] / self.Ki
            self._integral_limit_max = self.output_limits[1] / self.Ki

    def compute(self, current_value, dt):
        if dt <= 0: return 0.0
        error = self.setpoint - current_value
        if self.is_angle:
            error = np.arctan2(np.sin(error), np.cos(error))
        p_term = self.Kp * error
        self._integral += error * dt
        self._integral = np.clip(self._integral, self._integral_limit_min, self._integral_limit_max)
        i_term = self.Ki * self._integral
        if not self.derivative_on_measurement:
            derivative = (error - self._prev_error) / dt
        else:
            measurement_change = current_value - self._prev_measurement
            if self.is_angle:
                measurement_change = np.arctan2(np.sin(measurement_change), np.cos(measurement_change))
            derivative = -measurement_change / dt
        d_term = self.Kd * derivative
        output = p_term + i_term + d_term
        self._prev_error = error
        self._prev_measurement = current_value
        return np.clip(output, self.output_limits[0], self.output_limits[1])

    def reset(self, current_measurement=0):
        self._integral = 0
        self._prev_error = 0
        # 初始化 prev_measurement 以避免第一次微分计算出错
        self._prev_measurement = current_measurement
        
    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

# --- 2. 主控制程序 (实现你描述的清晰流程) ---
def main():
    # --- 模型加载 ---
    xml_path = '../assets/cartpole.xml'
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    slide_joint_id = model.joint('slide').id
    hinge_joint_id = model.joint('hinge').id
    motor_id = model.actuator('slide').id
    pole_body_id = model.body('pole').id

    # --- 控制参数与阈值 ---
    # !! 核心参数，完全按照你的描述定义 !!
    BALANCE_ANGLE_THRESHOLD_RAD = np.deg2rad(20) # 20度的扇形区域
    TRANSITION_DURATION = 0.1                    # 0.2秒的过渡刹车时间

    # 起摆阶段参数
    SWINGUP_ENERGY_GAIN = 4.5  # 能量增益
    SWINGUP_POS_KP = 1.2       # 位置约束
    SWINGUP_POS_KD = 0.8       # 位置阻尼

    # 过渡阶段参数
    TRANSITION_DAMPING = 0.1  # 刹车力度，可能需要比之前大一些

    # PID稳定阶段参数
    ANGLE_KP = -15
    ANGLE_KI = 0
    ANGLE_KD = -0.5
    POS_KP = -0.5
    POS_KI = 0
    POS_KD = -0.2
    
    CONTROL_LIMIT = model.actuator_ctrlrange[motor_id]

    # --- 控制器初始化 ---
    angle_pid = PIDController(Kp=ANGLE_KP, Ki=ANGLE_KI, Kd=ANGLE_KD, setpoint=np.pi, 
                              is_angle=True, derivative_on_measurement=True, 
                              output_limits=CONTROL_LIMIT)
    pos_pid = PIDController(Kp=POS_KP, Ki=POS_KI, Kd=POS_KD, setpoint=0, output_limits=CONTROL_LIMIT)
    
    # --- 状态机变量 ---
    control_state = 'SWINGUP'
    transition_entry_time = 0.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # --- 获取状态 ---
            cart_pos = data.qpos[slide_joint_id]
            pole_angle_raw = data.qpos[hinge_joint_id]
            cart_vel = data.qvel[slide_joint_id]
            pole_vel = data.qvel[hinge_joint_id]
            pole_angle = np.arctan2(np.sin(pole_angle_raw), np.cos(pole_angle_raw))
            
            # 计算摆杆与垂直向上的绝对角度差 (这是我们判断成功/失败的唯一标准)
            # abs_angle_from_vertical = 0 表示完全垂直
            abs_angle_from_vertical = np.pi - abs(pole_angle)

            # --- 状态机 ---
            if control_state == 'SWINGUP':
                print(f"状态: SWINGUP | 离顶部角度: {np.rad2deg(abs_angle_from_vertical):.1f}°")
                
                # 能量泵浦 + 位置约束
                pole_mass = model.body(pole_body_id).mass[0]
                pole_length_com = np.abs(model.body(pole_body_id).ipos[2])
                gravity_acc = np.abs(model.opt.gravity[2])
                pole_inertia = model.body(pole_body_id).inertia[0]
                target_energy = pole_mass * gravity_acc * pole_length_com * 2
                potential_energy = pole_mass * gravity_acc * pole_length_com * (1 - np.cos(pole_angle))
                kinetic_energy = 0.5 * pole_inertia * pole_vel**2
                current_energy = potential_energy + kinetic_energy
                energy_error = target_energy - current_energy
                sign_term = np.sign(pole_vel * np.cos(pole_angle)) if pole_vel != 0 else 1.0
                energy_force = SWINGUP_ENERGY_GAIN * energy_error * sign_term
                centering_force = -SWINGUP_POS_KP * cart_pos - SWINGUP_POS_KD * cart_vel
                control_force = energy_force + centering_force
                
                # 检查是否进入稳定区域 (起摆成功)
                if abs_angle_from_vertical < BALANCE_ANGLE_THRESHOLD_RAD:
                    print(f"--- 起摆成功！进入 {TRANSITION_DURATION} 秒过渡刹车 ---")
                    control_state = 'TRANSITION'
                    transition_entry_time = time.time() # 记录进入过渡态的时间

            elif control_state == 'TRANSITION':
                print(f"状态: TRANSITION (刹车中) | 剩余时间: {TRANSITION_DURATION - (time.time() - transition_entry_time):.2f}s")
                
                # 施加阻尼力进行刹车
                control_force = TRANSITION_DAMPING * pole_vel
                
                # 检查刹车时间是否结束
                if time.time() - transition_entry_time > TRANSITION_DURATION:
                    print("--- 过渡结束，切换到 PID 稳定 ---")
                    control_state = 'BALANCE'
                    # 重置控制器，并将当前角度作为D项的初始测量值
                    angle_pid.reset(current_measurement=pole_angle) 
                    pos_pid.reset()
                
                # 在过渡期间如果就掉下去了，立刻返回起摆模式
                elif abs_angle_from_vertical > BALANCE_ANGLE_THRESHOLD_RAD:
                    print("--- (过渡时跌倒) 返回起摆模式 ---")
                    control_state = 'SWINGUP'
            
            elif control_state == 'BALANCE':
                # 动态设定目标点 (+pi 或 -pi)
                target_angle = np.pi if pole_angle > 0 else -np.pi
                angle_pid.set_setpoint(target_angle)
                
                dt = model.opt.timestep
                angle_force = angle_pid.compute(pole_angle, dt)
                pos_force = pos_pid.compute(cart_pos, dt)
                control_force = angle_force + pos_force
                
                print(f"状态: BALANCE | 离顶部角度: {np.rad2deg(abs_angle_from_vertical):.1f}° | 力: {control_force:.2f}")

                # 检查是否跌出稳定区域 (稳定失败)
                if abs_angle_from_vertical > BALANCE_ANGLE_THRESHOLD_RAD:
                    print("--- (稳定失败) 返回起摆模式 ---")
                    control_state = 'SWINGUP'
            else:
                control_force = 0.0

            # 应用控制
            data.ctrl[motor_id] = np.clip(control_force, CONTROL_LIMIT[0], CONTROL_LIMIT[1])
            mujoco.mj_step(model, data)
            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            print(f"--- 剩余时间: {model.opt.timestep:.5f}s ---")
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == '__main__':
    main()