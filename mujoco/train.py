# train.py

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from CartPoleEnv import CartPoleEnv # 导入我们自己的环境

# 创建环境
# VecEnv可以并行运行多个环境，加速训练
vec_env = make_vec_env(CartPoleEnv, n_envs=4)

# 创建PPO模型
# MlpPolicy: 使用标准的多层感知机作为策略和价值网络
# verbose=1: 打印训练过程中的信息
model = PPO(
    "MlpPolicy", 
    vec_env, 
    verbose=1,
    tensorboard_log="./ppo_cartpole_tensorboard/" # 可选：用于Tensorboard可视化
)

# 开始训练
# total_timesteps: 训练的总步数，可以先设小一点测试，之后再增大
print("--- 开始训练 ---")
model.learn(total_timesteps=1000000)
print("--- 训练完成 ---")

# 保存训练好的模型
model_path = "ppo_cartpole_model"
model.save(model_path)
print(f"模型已保存到: {model_path}.zip")

vec_env.close()