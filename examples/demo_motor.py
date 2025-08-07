# examples/test_motor.py
import time
import math
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from pendulum_control import Actuator

def main():
    """
    电机独立测试程序。
    功能：
    1. 连接到指定的串口电机。
    2. 发送一个预设的、随时间变化的正弦波电流指令。
    3. 实时显示发送的指令电流和从电机收到的反馈状态。
    4. 客户可以观察小车是否按照正弦波指令来回平滑运动。
    5. 按下 Ctrl+C 后安全停止电机并退出。
    """
    # --- 配置 ---
    # 请根据您的实际连接修改串口号
    MOTOR_SERIAL_PORT = '/dev/ttyUSB1'  # Linux: '/dev/ttyUSB1', Windows: 'COM4'
    MOTOR_BAUD_RATE = 4000000
    MOTOR_AXIS_ID = 0
    WHEEL_RADIUS_MM = 32.5

    # 测试参数
    TEST_FREQUENCY = 100.0  # 与电机通信的频率 (Hz)
    SINE_WAVE_PERIOD = 4.0    # 电流正弦波的周期 (秒)，即小车来回一次的时间
    SINE_WAVE_AMPLITUDE = 0.5 # 电流正弦波的振幅 (A)，可根据需要调整

    print("--- 电机性能测试 ---")
    print(f"端口: {MOTOR_SERIAL_PORT}, 波特率: {MOTOR_BAUD_RATE}")
    print(f"测试模式: 发送 {SINE_WAVE_AMPLITUDE}A 振幅, {SINE_WAVE_PERIOD}s 周期的正弦电流")
    print("正在初始化电机...")

    # 初始化电机执行器
    motor = Actuator(
        port=MOTOR_SERIAL_PORT,
        baudrate=MOTOR_BAUD_RATE,
        axis_id=MOTOR_AXIS_ID,
        wheel_radius_mm=WHEEL_RADIUS_MM
    )

    try:
        # 连接到电机
        motor.connect()

        if not motor.is_connected:
            print("\n错误：无法连接到电机。请检查端口和连接。")
            return
            
        print("\n电机已连接。开始发送测试指令... (按 Ctrl+C 停止)")
        print("-" * 120)
        # 打印表头
        print(f"{'目标电流 (A)':<15} | {'反馈角度 (deg)':<18} | {'反馈电流 (A)':<18} | {'线位移 (cm)':<18} | {'线速度 (cm/s)':<20}")
        print("-" * 120)

        start_time = time.monotonic()
        target_interval = 1.0 / TEST_FREQUENCY

        while True:
            loop_start_time = time.monotonic()
            
            # 1. 生成正弦波目标电流
            elapsed_time = loop_start_time - start_time
            target_current = SINE_WAVE_AMPLITUDE * math.sin(2 * math.pi * elapsed_time / SINE_WAVE_PERIOD)
            
            # 2. 发送指令并获取反馈
            # 注意：根据您的系统，可能需要发送 -target_current
            if motor.set_current(-target_current):
                # 3. 打印发送和反馈的数值
                print(
                    f"{target_current:<15.4f} | "
                    f"{motor.position:<18.2f} | "
                    f"{motor.current:<18.4f} | "
                    f"{motor.distance_traveled_cm:<18.2f} | "
                    f"{motor.velocity_cmps:<20.2f}",
                    end='\r'
                )
            else:
                print(f"发送指令: {target_current:<.4f} A, 但未能收到有效反馈。", end='\r')

            # 4. 精确延时以控制通信频率
            loop_elapsed = time.monotonic() - loop_start_time
            sleep_time = target_interval - loop_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n测试被用户中断。")
    except Exception as e:
        print(f"\n发生未知错误: {e}")
    finally:
        # 确保无论发生什么，都安全地禁用并断开电机
        if motor and motor.is_connected:
            print("\n正在停止并禁用电机...")
            motor.disable() # 发送0电流
            motor.disconnect()
        print("测试结束。")

if __name__ == "__main__":
    main()