# evaluate.py (诊断版 - 打印观察值)

import time
from stable_baselines3 import PPO
from CartPoleEnv import CartPoleEnv
import numpy as np

np.set_printoptions(precision=3, suppress=True) # 设置打印格式，方便观察

model = PPO.load("ppo_cartpole_model")
env = CartPoleEnv(render_mode="human")

print("--- 开始诊断评估 (已减速) ---")
obs, info = env.reset()

for i in range(5000): 
    # 在执行动作前，打印当前状态
    print(f"Step: {env.current_step:4d}, Obs: {obs}")

    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    
    time.sleep(0.02)

    if terminated or truncated:
        # 打印导致终止的最后一次观察值
        print(f"!!! 回合终止于 Step: {env.current_step:4d}, 最后的观察值: {obs} !!!")
        print("2秒后重置...")
        time.sleep(2)
        obs, info = env.reset()

env.close()