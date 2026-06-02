import sys
import numpy as np
import os
import re
import pygame

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from stable_baselines3 import DDPG
import gymnasium as gym

import truck_parking_env


def _find_best_model(ckpt_dir):
    final_path = os.path.join(ckpt_dir, 'truck_parking_final.zip')
    if os.path.exists(final_path):
        return final_path

    candidates = []
    for f in os.listdir(ckpt_dir):
        if f.endswith('.zip'):
            candidates.append(os.path.join(ckpt_dir, f))

    if not candidates:
        return None

    def extract_steps(p):
        basename = os.path.splitext(os.path.basename(p))[0]
        nums = re.findall(r'_(\d+)_steps', basename)
        if nums:
            return int(nums[-1])
        nums = re.findall(r'(\d{6,})', basename)
        if nums:
            return int(nums[-1])
        nums = re.findall(r'(\d+)', basename)
        if nums:
            return int(nums[-1])
        return 0

    candidates.sort(key=extract_steps, reverse=True)
    return candidates[0]


def infer():
    ckpt_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rl_ckpt')

    model_path = _find_best_model(ckpt_dir)
    if model_path is None:
        print(f'No model found in {ckpt_dir}.')
        return

    infer_env = gym.make('TruckParking-v0', render_mode='human', fullscreen=True)

    print(f'Loading: {model_path}')
    model = DDPG.load(model_path, env=infer_env)

    obs, _ = infer_env.reset()
    total_reward = 0
    step = 0

    print('ESC:退出  R:重置')
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    obs, _ = infer_env.reset()
                    total_reward = 0
                    step = 0

        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = infer_env.step(action)
        total_reward += reward
        step += 1

        u = infer_env.unwrapped
        if step % 20 == 0:
            print(f'[{step:4d}] dist={u._dist_to_goal():.1f}m  '
                  f'g={np.degrees(u.truck.g_rad):.0f}  r={reward:.1f}')

        if terminated or truncated:
            msg = ('Success' if info.get('succeeded') else
                   'Jackknife' if info.get('jackknifed') else
                   'Collision' if info.get('collided') else 'Timeout')
            print(f'End: {msg}  total_reward={total_reward:.0f}')
            obs, _ = infer_env.reset()
            total_reward = 0
            step = 0

    infer_env.close()


if __name__ == '__main__':
    infer()
