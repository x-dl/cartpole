# CartPoleEnv.py (最终·纯粹分阶段奖励版)

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
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        self.slide_joint_id = self.model.joint('slide').id
        self.hinge_joint_id = self.model.joint('hinge').id
        self.motor_id = self.model.actuator('slide').id

        actuator = self.model.actuator(self.motor_id)
        self.action_space = spaces.Box(low=actuator.ctrlrange[0], high=actuator.ctrlrange[1], shape=(1,), dtype=np.float32)

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
        
        try:
            for _ in range(5):
                mujoco.mj_step(self.model, self.data)
        except mujoco.FatalError as e:
            observation = self._get_obs()
            reward = -20.0
            terminated = True
            truncated = False
            return observation, reward, terminated, truncated, {}

        observation = self._get_obs()
        cart_pos, cart_vel, pole_cos, _, pole_vel = observation
        
        # ==================== 纯粹分阶段奖励函数 ====================
        
        # 定义稳定区域: 顶部 ±30度
        upright_threshold = np.cos(np.deg2rad(30)) # 约 -0.866

        if pole_cos < upright_threshold:
            # --- 模式一：稳定模式 (杆子在顶部) ---
            # 目标：保持垂直、居中、静止
            
            # 奖励垂直: 越接近顶部 (cos=-1), 奖励越高, 最高为1
            reward_upright = (-pole_cos - upright_threshold) / (1 - upright_threshold)
            
            # 奖励居中: 越接近中心 (pos=0), 奖励越高, 最高为1
            reward_centering = 1.0 - np.tanh(5 * np.abs(cart_pos))
            
            # 奖励静止: 速度越小，奖励越高, 最高为1
            reward_quiet = 1.0 - np.tanh(0.5 * (np.square(cart_vel) + np.square(pole_vel)))

            # 总奖励是三者的加权和，额外再给一个大的存活奖励
            total_reward = 2.0 + (reward_upright + reward_centering + reward_quiet)
        else:
            # --- 模式二：起摆模式 (杆子在下面) ---
            # 唯一目标：把杆子弄上去。
            # 奖励高度，同时轻微惩罚控制量，避免无效的抖动
            reward_height = (-pole_cos + 1) / 2
            control_penalty = -0.001 * np.square(action[0])
            total_reward = reward_height + control_penalty

        # =================================================================

        terminated = False
        if not (-0.15 < cart_pos < 0.21): 
            terminated = True
            total_reward = -10.0
        
        truncated = False
        if self.current_step >= self.max_episode_steps:
            truncated = True
        
        if self.render_mode == "human": self.render()
        return observation, total_reward, terminated, truncated, {}

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