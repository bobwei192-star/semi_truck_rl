#!/usr/bin/env python3
"""
课程学习可视化测试：复用环境自身的渲染
"""
import sys
import os
import numpy as np
import pygame
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from stable_baselines3 import DDPG
import gymnasium as gym

import truck_parking_env
from scene_config import CURRICULUM_STAGES, GOAL_X, GOAL_Y


def test_model(model_path=None, curriculum_stage=4, num_episodes=5):
    env = gym.make('TruckParking-v0',
                   render_mode='human',
                   fullscreen=True,
                   curriculum_stage=curriculum_stage)
    env.unwrapped.set_show_trajectory(True)
    
    if model_path and os.path.exists(model_path):
        model = DDPG.load(model_path, env=env)
        print(f"加载模型: {model_path}")
    else:
        model = None
        print("使用随机策略")
    
    print(f"\n{'='*60}")
    print(f"测试 Stage {curriculum_stage}: {CURRICULUM_STAGES[curriculum_stage].get('description', 'Unknown')}")
    stage = CURRICULUM_STAGES[curriculum_stage]
    print(f"起点: ({stage['start_x']}, {stage['start_y']})")
    print(f"目标: ({stage['goal_x']}, {stage['goal_y']})")
    print(f"{'='*60}")
    
    success_count = 0
    
    for ep in range(num_episodes):
        obs, _ = env.reset()
        done = False
        total_reward = 0.0
        step = 0
        
        while not done:
            if model is not None:
                action, _ = model.predict(obs, deterministic=True)
            else:
                action = env.action_space.sample()
            
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            done = terminated or truncated
            step += 1
            
            # 处理 ESC 键退出
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        done = True
                        print(f"用户中断")
        
        if info.get('succeeded', False):
            success_count += 1
        
        print(f"Episode {ep + 1}: reward={total_reward:.2f}, steps={step}, "
              f"success={info.get('succeeded', False)}, "
              f"collided={info.get('collided', False)}, "
              f"jackknifed={info.get('jackknifed', False)}")
    
    env.close()
    
    print(f"\n成功率: {success_count}/{num_episodes} ({100 * success_count / num_episodes:.1f}%)")


def test_all_stages(model_path=None, num_episodes=3):
    for stage_idx in range(len(CURRICULUM_STAGES)):
        test_model(model_path=model_path, curriculum_stage=stage_idx, num_episodes=num_episodes)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default=None, help='模型路径')
    parser.add_argument('--stage', type=int, default=0, help='课程阶段 (0-4)')
    parser.add_argument('--episodes', type=int, default=5, help='测试 episode 数')
    parser.add_argument('--all-stages', action='store_true', help='测试所有阶段')
    
    args = parser.parse_args()
    
    if args.all_stages:
        test_all_stages(model_path=args.model, num_episodes=args.episodes)
    else:
        test_model(model_path=args.model, curriculum_stage=args.stage, num_episodes=args.episodes)
