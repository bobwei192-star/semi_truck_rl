import sys
import numpy as np
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from stable_baselines3 import DDPG
from stable_baselines3.her.her_replay_buffer import HerReplayBuffer
from stable_baselines3.common.noise import OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import DummyVecEnv
import gymnasium as gym

import env.truck_parking_env
from rl_solve.utils import RewardPlotCallback, CurriculumCallback
from config.scene_config import (
    GOAL_X, GOAL_Y, START_X, START_Y, LANE_WIDTH,
    MAX_STEPS, TRAIN_TOTAL_STEPS, CURRICULUM_STAGES,
)


N_ENVS = 8  # 并行环境数量


def make_env(rank=0):
    """Factory to create a TruckParking env for VecEnv."""
    def _init():
        env = gym.make('TruckParking-v0', curriculum_stage=0)
        return env
    return _init


def train():
    ckpt_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rl_ckpt')
    os.makedirs(ckpt_dir, exist_ok=True)

    # Multi-env parallel (no rendering in subprocesses)
    env_fns = [make_env(i) for i in range(N_ENVS)]
    train_env = DummyVecEnv(env_fns)

    n_actions = train_env.action_space.shape[-1]
    noise = OrnsteinUhlenbeckActionNoise(
        mean=np.zeros(n_actions),
        sigma=0.15 * np.ones(n_actions),
        theta=0.15,
        dt=0.05,
    )

    model = DDPG(
        policy='MultiInputPolicy',
        env=train_env,
        replay_buffer_class=HerReplayBuffer,
        replay_buffer_kwargs={
            'n_sampled_goal': 4,
            'goal_selection_strategy': 'future',
            'handle_timeout_termination': True,
        },
        action_noise=noise,
        learning_rate=1e-3,
        buffer_size=int(2e6),
        learning_starts=2000 * N_ENVS,
        batch_size=512,
        gamma=0.98,
        tau=0.01,
        train_freq=1,
        gradient_steps=1,
        policy_kwargs={'net_arch': [512, 512, 256]},
        verbose=1,
        tensorboard_log=os.path.join(ckpt_dir, 'tensorboard'),
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=50000,
        save_path=ckpt_dir,
        name_prefix='truck_parking',
    )

    reward_plot_callback = RewardPlotCallback(
        ckpt_dir=ckpt_dir,
        plot_freq=5000,
    )

    curriculum_callback = CurriculumCallback(
        env=train_env,
        stages=CURRICULUM_STAGES,
        switch_window=20,
    )

    print(f'训练配置: OU噪声 σ=0.15 θ=0.15, 网络=[512,512,256]')
    print(f'HER: future, max_steps={MAX_STEPS}, lane_width={LANE_WIDTH}')
    print(f'起点: ({START_X}, {START_Y}), 目标: ({GOAL_X}, {GOAL_Y})')
    print(f'并行环境: {N_ENVS}个, 检查点: {ckpt_dir}')
    print(f'Curriculum: {len(CURRICULUM_STAGES)} stages')

    model.learn(
        total_timesteps=TRAIN_TOTAL_STEPS,
        callback=[checkpoint_callback, reward_plot_callback, curriculum_callback],
        log_interval=10,
    )

    final_path = os.path.join(ckpt_dir, 'truck_parking_final')
    model.save(final_path)
    print(f'训练完成! 模型: {final_path}')

    train_env.close()


if __name__ == '__main__':
    train()
