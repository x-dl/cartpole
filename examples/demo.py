import math
from pendulum_control import InvertedPendulum, Actuator, EncoderReader

def main():
    """主函数：配置并运行倒立摆系统。"""
    
    ENCODER_SERIAL_PORT = '/dev/ttyUSB0'  
    MOTOR_SERIAL_PORT = '/dev/ttyUSB1'    

    CONTROL_CONFIG = {
        # *** 自动校准配置 ***
        'calibration': {
            'enabled': True,  # 设置为True来开启启动时自动校准
            'duration_s': 3.0, # 校准持续时间（秒）
            'prompt': "请确保摆杆在底部完全静止，即将开始水平校准..."
        },
        'main_loop_freq': 200.0,
        'pos_loop_freq': 40.0,
        'upright_angle_rad': math.pi,
        'upright_threshold_rad': math.radians(20.0),
        'swingup_gain': 0.4,
        'swingup_max_current': 1.0,
        'transition_brake_kd': 2.5,
        'transition_duration_s': 0.2,

        'stable_offset': {
            'enabled': True,
            'speed_threshold_rad_s': 10.0,
            'offset_angle_rad': math.radians(1.5)
        },

        'angle_pid': {
            'kp': 20.0, 'ki': 5.0, 'kd': 1.2,
            'output_limits': (-2.5, 2.5)
        },

        # *** 关键修正：精确复刻原始设置 ***
        'pos_pid': {
            'kp': 0.01, 'ki': 0.0, 'kd': 0.0,
            'target_pos_cm': 0.0,
            # 将输出限制设为0，从而在功能上禁用位置环，与原始代码行为一致
            'output_limits_rad': (-math.radians(0), math.radians(0)) 
        },
        
        'enable_logging': True
    }

    # ... 初始化和运行部分保持不变 ...
    motor = Actuator(
        port=MOTOR_SERIAL_PORT, baudrate=4000000, axis_id=0, wheel_radius_mm=32.5
    )
    encoder = EncoderReader(
        port=ENCODER_SERIAL_PORT, baudrate=115200, encoder_resolution=16384
    )
    pendulum_system = InvertedPendulum(motor, encoder, CONTROL_CONFIG)
    pendulum_system.run()

if __name__ == "__main__":
    main()