# CartPoleEnv.py (最终鲁棒版)

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mujoco
import mujoco.viewer
import os

class CartPoleEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, render_mode=None, max_episode_steps=2000):
        super().__init__()
        
        xml_path = 'cartpole_new2.xml'
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"找不到模型文件: {xml_path}")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(model)

        self.slide_joint_id = self.model.joint('slide').id
        self.hinge_joint_id = self.model.joint('hinge').id
        self.motor_id = self.model.actuator('slide').id

        actuator = self.model.actuator(self.motor_id)
        self.action_space = spaces.Box(low=actuator.ctrlrange[0], high=actuator.ctrlrange[1], shape=(1,), dtype=np.float32)

        # 定义一个巨大的值，用于裁剪观测值
        self.obs_clip_range = 100.0
        obs_dim = 5
        high = np.full(obs_dim, self.obs_clip_range)
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float64)

        self.render_mode = render_mode
        self.viewer = None
        self.max_episode_steps = max_episode_steps
        self.current_step = 0

    def _get_obs(self):
        cart_pos = self.data.qpos[self.slide_joint_id]
        pole_angle = self.data.qpos[self.hinge_joint_id]
        cart_vel = self.data.qvel[self.slide_joint_id]
        pole_vel = self.data.qvel[self.hinge_joint_id]
        
        # --- 解决方案 1: 观测值裁剪 ---
        # 强制将所有值都限制在一个巨大的、但不是无穷的范围内
        obs = np.array([cart_pos, cart_vel, np.cos(pole_angle), np.sin(pole_angle), pole_vel])
        return np.clip(obs, -self.obs_clip_range, self.obs_clip_range)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.hinge_joint_id] = 0
        self.current_step = 0
        observation = self._get_obs()
        info = {}
        if self.render_mode == "human": self.render()
        return observation, info

    def step(self, action):
        self.current_step += 1
        self.data.ctrl[self.motor_id] = action[0]
        
        # 捕获MuJoCo底层的物理仿真错误
        try:
            for _ in range(5):
                mujoco.mj_step(self.model, self.data)
        except mujoco.FatalError as e:
            print(f"!!! MuJoCo 物理仿真错误: {e} !!!")
            # 如果仿真爆炸，直接判为失败
            observation = self._get_obs() # 获取最后的状态
            reward = -20.0 # 给予比出界更大的惩罚
            terminated = True
            truncated = False
            return observation, reward, terminated, truncated, {}

        observation = self._get_obs()
        cart_pos, cart_vel, pole_cos, _, pole_vel = observation
        
        # --- 奖励函数 (增加对速度的惩罚) ---
        upright_reward = (-pole_cos + 1) / 2
        centering_reward = 1.0 - np.tanh(2 * np.abs(cart_pos))
        alive_bonus = 0.1
        control_penalty = -0.01 * np.square(action[0])
        # --- 解决方案 2: 速度惩罚 ---
        velocity_penalty = -0.01 * np.square(cart_vel) - 0.001 * np.square(pole_vel)

        reward = upright_reward + centering_reward * 0.5 + alive_bonus + control_penalty + velocity_penalty

        # --- 终止条件 ---
        terminated = False
        if not (-0.15 < cart_pos < 0.21): 
            terminated = True
            reward = -10.0
        
        truncated = False
        if self.current_step >= self.max_episode_steps:
            truncated = True
        
        if self.render_mode == "human": self.render()
        return observation, reward, terminated, truncated, {}

    def render(self):
        if self.render_mode == "human": self._render_frame()

    def _render_frame(self):
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None