# evaluate.py (无渲染评估版)

from stable_baselines3 import PPO
from CartPoleEnv import CartPoleEnv
import numpy as np

# --- 加载模型 ---
model_path = "ppo_cartpole_model"
model = PPO.load(model_path)

# --- 创建一个无渲染的环境 ---
# 将 render_mode 设置为 None 或者不设置
env = CartPoleEnv(render_mode=None)

print("--- 开始无渲染评估 ---")
print("将打印每一步的奖励值。如果奖励值持续为正且较大，说明策略有效。")
print("如果奖励值很快变为-10，说明策略失败导致环境重置。")

obs, info = env.reset()
episode_rewards = []
current_episode_reward = 0

for i in range(5000): # 运行更多步数来观察多个回合
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    
    current_episode_reward += reward
    print(f"步数: {i+1}, 奖励: {reward:.2f}, 当前回合总奖励: {current_episode_reward:.2f}")
    
    if terminated or truncated:
        print(f"--- 回合结束，总奖励: {current_episode_reward:.2f} ---")
        episode_rewards.append(current_episode_reward)
        current_episode_reward = 0
        obs, info = env.reset()

env.close()

# 打印最终的评估结果
if episode_rewards:
    print("\n--- 评估总结 ---")
    print(f"完成了 {len(episode_rewards)} 个回合。")
    print(f"平均回合奖励: {np.mean(episode_rewards):.2f}")
    print(f"最高回合奖励: {np.max(episode_rewards):.2f}")
else:
    print("\n--- 评估总结 ---")
    print("在5000步内未完成任何回合。")