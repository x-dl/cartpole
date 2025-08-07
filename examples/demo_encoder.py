import time
import math
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    
from pendulum_control import EncoderReader
"""
    编码器独立测试程序。
    功能：
    1. 连接到指定的串口编码器。
    2. 在后台线程中持续读取数据。
    3. 在前台清晰地打印实时角度、角速度和数据帧率。
    4. 按下 Ctrl+C 后安全退出。
    """
def main():
        # --- 配置 ---
    # 请根据您的实际连接修改串口号
    ENCODER_SERIAL_PORT = '/dev/ttyUSB0'  # Linux: '/dev/ttyUSB0', Windows: 'COM3'
    ENCODER_BAUDRATE = 115200
    ENCODER_RESOLUTION = 16384

    print("--- 编码器性能测试 ---")
    print(f"端口: {ENCODER_SERIAL_PORT}, 波特率: {ENCODER_BAUDRATE}")
    print("正在初始化编码器...")

    # 初始化编码器读取器
    encoder = EncoderReader(
        port=ENCODER_SERIAL_PORT,
        baudrate=ENCODER_BAUDRATE,
        encoder_resolution=ENCODER_RESOLUTION
    )

    try:
        # 启动后台读取线程
        encoder.start()

        if not encoder.is_running:
            print("\n错误：无法启动编码器读取器。请检查端口和连接。")
            return

        print("\n编码器已启动。正在读取数据... (按 Ctrl+C 停止)")
        print("-" * 60)
        # 打印表头
        print(f"{'角度 (rad)':<15} | {'角度 (deg)':<15} | {'角速度 (rad/s)':<20} | {'角速度 (deg/s)':<20} | {'帧率 (FPS)':<15}")
        print("-" * 100)
        
        # 主循环，用于获取和显示数据
        while True:
            # 从编码器获取最新数据
            angle_rad, speed_rad_s, fps = encoder.get_data()

            # 将弧度转换为度
            angle_deg = math.degrees(angle_rad)
            speed_deg_s = math.degrees(speed_rad_s)
            
            # 在同一行更新数据，以获得清爽的显示效果
            print(f"{angle_rad:<15.4f} | {angle_deg:<15.2f} | {speed_rad_s:<20.4f} | {speed_deg_s:<20.2f} | {fps:<15.1f}", end='\r')
            
            # 以较高频率刷新显示，但不需要与硬件通信同步
            time.sleep(0.02) # 50Hz 刷新率

    except KeyboardInterrupt:
        print("\n\n测试被用户中断。")
    except Exception as e:
        print(f"\n发生未知错误: {e}")
    finally:
        # 确保无论发生什么，都安全地停止后台线程
        if encoder and encoder.is_running:
            print("正在关闭编码器...")
            encoder.stop()
        print("测试结束。")

if __name__ == "__main__":
    main()