#!/usr/bin/env python3
"""
测试训练脚本 - 简化版，用于验证正向出库任务的强化学习算法
"""
import numpy as np
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# GPU检测逻辑
import torch

def get_device():
    """自动检测并返回可用的设备（GPU优先）"""
    if torch.cuda.is_available():
        device = 'cuda'
        print(f'使用GPU: {torch.cuda.get_device_name(0)}')
    else:
        # 尝试检测ROCm/HIP
        try:
            # 检查是否有HIP设备
            if hasattr(torch.version, 'hip') and torch.version.hip is not None:
                # 尝试初始化HIP
                torch.hip.init()
                if torch.hip.is_available():
                    device = 'cuda'  # ROCm使用cuda作为设备类型
                    print(f'使用ROCm GPU: {torch.version.hip}')
                else:
                    device = 'cpu'
                    print('使用CPU')
            else:
                device = 'cpu'
                print('使用CPU')
        except Exception:
            device = 'cpu'
            print('使用CPU')
    return device

DEVICE = get_device()

import env.truck_parking_env
from config.scene_config import (
    GOAL_X, GOAL_Y, START_X, START_Y, LANE_WIDTH,
    MAX_STEPS, TRAIN_TOTAL_STEPS,
)

from stable_baselines3 import DDPG
from stable_baselines3.her.her_replay_buffer import HerReplayBuffer
from stable_baselines3.common.noise import OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.callbacks import BaseCallback
import gymnasium as gym


class ProgressCallback(BaseCallback):
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
    print('═' * 60)
    print('  半挂车正向出库 - 强化学习测试')
    print('═' * 60)
    print(f'  GOAL=({GOAL_X},{GOAL_Y})  START=({START_X},{START_Y})')
    print(f'  lane_width={LANE_WIDTH}  max_steps={MAX_STEPS}')
    print(f'  total_steps={TRAIN_TOTAL_STEPS}')
    print('═' * 60)

    raw_env = gym.make('TruckParking-v0', render_mode=None)

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
        buffer_size=50000,
        learning_starts=1000,
        batch_size=128,
        gamma=0.98,
        tau=0.01,
        train_freq=1,
        gradient_steps=1,
        policy_kwargs={'net_arch': [128, 128]},
        verbose=1,
    )

    progress_cb = ProgressCallback()

    print('\n开始训练...')
    model.learn(
        total_timesteps=20000,  # 使用较小的训练步数进行测试
        callback=[progress_cb],
        log_interval=10,
    )

    ckpt_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rl_ckpt')
    os.makedirs(ckpt_dir, exist_ok=True)
    final_path = os.path.join(ckpt_dir, 'truck_parking_test')
    model.save(final_path)
    print(f'\n测试完成! 模型已保存到: {final_path}')

    # 测试训练后的模型
    print('\n测试训练后的模型...')
    obs, _ = raw_env.reset()
    total_reward = 0
    for _ in range(MAX_STEPS):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = raw_env.step(action)
        total_reward += reward
        if terminated or truncated:
            break
    
    print(f'测试结果 - 总奖励: {total_reward:.2f}, 成功: {info.get("succeeded", False)}')
    raw_env.close()


if __name__ == '__main__':
    main()
