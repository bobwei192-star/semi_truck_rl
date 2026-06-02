import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback, EvalCallback
import gymnasium as gym

import env.truck_parking_env
from config.scene_config import CURRICULUM_STAGES, TRAIN_TOTAL_STEPS, GOAL_X, GOAL_Y
from rl_solve.utils import RewardPlotCallback, CurriculumCallback


N_ENVS = 4


def make_env(curriculum_stage=0, rank=0):
    def _init():
        env = gym.make('TruckParking-v0', curriculum_stage=curriculum_stage)
        return env
    return _init


def train_curriculum():
    ckpt_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rl_ckpt')
    os.makedirs(ckpt_dir, exist_ok=True)
    
    log_dir = os.path.join(ckpt_dir, 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    total_timesteps = TRAIN_TOTAL_STEPS
    stage_timesteps = total_timesteps // len(CURRICULUM_STAGES)
    
    print("=" * 60)
    print("课程学习强化训练 - PPO")
    print("=" * 60)
    print(f"总训练步数: {total_timesteps}")
    print(f"阶段数: {len(CURRICULUM_STAGES)}")
    print(f"每阶段步数: {stage_timesteps}")
    print(f"并行环境数: {N_ENVS}")
    print("=" * 60)
    
    model = None
    
    for stage_idx, stage_config in enumerate(CURRICULUM_STAGES):
        print(f"\n{'='*60}")
        print(f"Stage {stage_idx}: {stage_config.get('description', 'Unknown')}")
        print(f"起点: ({stage_config['start_x']}, {stage_config['start_y']})")
        print(f"目标: ({stage_config['goal_x']}, {stage_config['goal_y']})")
        print(f"最大步数: {stage_config['max_steps']}")
        print(f"{'='*60}\n")
        
        env_fns = [make_env(curriculum_stage=stage_idx, rank=i) for i in range(N_ENVS)]
        
        if N_ENVS > 1:
            train_env = SubprocVecEnv(env_fns)
        else:
            train_env = DummyVecEnv(env_fns)
        
        if model is None:
            model = PPO(
                "MultiInputPolicy",
                train_env,
                learning_rate=3e-4,
                n_steps=2048,
                batch_size=64,
                n_epochs=10,
                gamma=0.99,
                gae_lambda=0.95,
                clip_range=0.2,
                ent_coef=0.01,
                vf_coef=0.5,
                max_grad_norm=0.5,
                tensorboard_log=log_dir,
                verbose=1,
                device="auto",
            )
        else:
            model.set_env(train_env)
        
        callbacks = CallbackList([
            CheckpointCallback(
                save_freq=10000,
                save_path=ckpt_dir,
                name_prefix=f"stage_{stage_idx}_ppo",
            ),
            RewardPlotCallback(ckpt_dir, plot_freq=5000),
            CurriculumCallback(train_env, CURRICULUM_STAGES, switch_window=20),
        ])
        
        model.learn(
            total_timesteps=stage_timesteps,
            callback=callbacks,
            progress_bar=False,
        )
        
        model.save(os.path.join(ckpt_dir, f"stage_{stage_idx}_final"))
        
        print(f"\nStage {stage_idx} 完成!")
        
        train_env.close()
    
    model.save(os.path.join(ckpt_dir, "curriculum_final"))
    
    print("\n" + "=" * 60)
    print("训练完成!")
    print("=" * 60)


if __name__ == '__main__':
    train_curriculum()