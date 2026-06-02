#!/usr/bin/env python3
"""
持久可视化训练模式：全屏窗口一直保持打开。
每个 step 由 env 自身渲染（render_mode='human'），绿色轨迹点。
按 ESC 优雅退出并保存模型。
"""
import numpy as np
import os
import sys
import pygame

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import env.truck_parking_env
from config.scene_config import (
    GOAL_X, GOAL_Y, START_X, START_Y, LANE_WIDTH,
    MAX_STEPS, TRAIN_TOTAL_STEPS,
)
from rl_solve.utils import RewardPlotCallback

from stable_baselines3 import DDPG
from stable_baselines3.her.her_replay_buffer import HerReplayBuffer
from stable_baselines3.common.noise import OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.callbacks import BaseCallback
import gymnasium as gym


class VisualProgressCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self._last_log = 0

    def _on_step(self):
        if self.num_timesteps - self._last_log >= 5000:
            if len(self.model.ep_info_buffer) > 0:
                rewards = [e['r'] for e in self.model.ep_info_buffer]
                lengths = [e['l'] for e in self.model.ep_info_buffer]
                if rewards:
                    print(f'[step {self.num_timesteps:7d}] '
                          f'eps={len(rewards)} '
                          f'avg_r={np.mean(rewards):.0f} '
                          f'avg_len={np.mean(lengths):.0f}')
            self._last_log = self.num_timesteps
        return True


def main():
    ckpt_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rl_ckpt')
    os.makedirs(ckpt_dir, exist_ok=True)

    raw_env = gym.make('TruckParking-v0',
                        render_mode='human',
                        fullscreen=True)
    raw_env.unwrapped.set_show_trajectory(True)

    n_actions = raw_env.action_space.shape[-1]
    noise = OrnsteinUhlenbeckActionNoise(
        mean=np.zeros(n_actions),
        sigma=0.15 * np.ones(n_actions),
        theta=0.15,
        dt=0.05,
    )

    model = DDPG(
        policy='MultiInputPolicy',
        env=raw_env,
        replay_buffer_class=HerReplayBuffer,
        replay_buffer_kwargs={
            'n_sampled_goal': 4,
            'goal_selection_strategy': 'future',
            'handle_timeout_termination': True,
        },
        action_noise=noise,
        learning_rate=1e-3,
        buffer_size=200000,
        learning_starts=2000,
        batch_size=256,
        gamma=0.98,
        tau=0.01,
        train_freq=1,
        gradient_steps=1,
        policy_kwargs={'net_arch': [256, 256, 128]},
        verbose=0,
    )

    visual_cb = VisualProgressCallback()

    print('═' * 60)
    print('  持久可视化训练模式')
    print('  全屏窗口实时显示每个训练 step')
    print('  绿色点 = 第五轮轨迹  按 ESC 保存模型并退出')
    print('═' * 60)
    print(f'  GOAL=({GOAL_X},{GOAL_Y})  START=({START_X},{START_Y})')
    print(f'  lane_width={LANE_WIDTH}  max_steps={MAX_STEPS}')
    print(f'  total_steps={TRAIN_TOTAL_STEPS}  OU σ=0.15')
    print('═' * 60)

    model.learn(
        total_timesteps=TRAIN_TOTAL_STEPS,
        callback=[visual_cb],
        log_interval=10,
    )

    final_path = os.path.join(ckpt_dir, 'truck_parking_final')
    model.save(final_path)
    print(f'训练完成! 模型: {final_path}')
    raw_env.close()


if __name__ == '__main__':
    main()
