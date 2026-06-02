#!/usr/bin/env python3
"""
训练日志验证脚本：验证所有算法通路是否正常工作。
输出到 ~/software/AI/auto_polit/semi_truck_rl/verify_training.log
"""
import sys
import os
import time
import csv

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import gymnasium as gym

# 1. 验证环境注册
print("=" * 60)
print("[1/8] 验证 Gymnasium 环境注册...")
import env.truck_parking_env
try:
    env = gym.make('TruckParking-v0', curriculum_stage=0)
    print("  ✓ 环境 'TruckParking-v0' 注册成功")
except Exception as e:
    print(f"  ✗ 环境注册失败: {e}")
    sys.exit(1)

# 2. 验证场景配置导入
print("=" * 60)
print("[2/8] 验证场景配置导入...")
try:
    from scene_config import (
        GOAL_X, GOAL_Y, START_X, START_Y,
        INIT_HEADING_RAD, HEADING_NOISE_RAD, INIT_G_RAD, G_NOISE_RAD,
        START_POS_NOISE_M,
        REWARD_DIST_COEF, REWARD_HEADING_COEF, REWARD_SOFT_BOUNDARY,
        REWARD_COLLISION, REWARD_SUCCESS,
        JACKKNIFE_DEG, G_CONSISTENCY_THRESHOLD, G_MAG_THRESHOLD, G_RATE_THRESHOLD,
        SOFT_BOUNDARY_MARGIN,
        MAX_STEPS, FIXED_SPEED, MAX_STEERING_DEG, MAX_STEERING_RATE,
        CURRICULUM_STAGES, TRAIN_TOTAL_STEPS,
    )
    print("  ✓ 场景配置导入成功")
    print(f"  ✓ START=({START_X}, {START_Y}), GOAL=({GOAL_X}, {GOAL_Y})")
    print(f"  ✓ INIT_HEADING={np.degrees(INIT_HEADING_RAD):.1f}°, INIT_G={np.degrees(INIT_G_RAD):.1f}°")
    print(f"  ✓ Curriculum stages: {len(CURRICULUM_STAGES)}")
except Exception as e:
    print(f"  ✗ 配置导入失败: {e}")
    sys.exit(1)

# 3. 验证环境 reset
print("=" * 60)
print("[3/8] 验证环境 reset...")
try:
    obs, info = env.reset(seed=42)
    print(f"  ✓ reset() 成功")
    print(f"  ✓ 初始位置: ({env.unwrapped.truck.x_m:.2f}, {env.unwrapped.truck.y_m:.2f})")
    print(f"  ✓ 初始朝向: {np.degrees(env.unwrapped.truck.h_rad):.1f}°")
    print(f"  ✓ 初始铰接角: {np.degrees(env.unwrapped.truck.g_rad):.1f}°")
    print(f"  ✓ 初始方向盘: {env.unwrapped.truck.s_deg:.1f}°")
    print(f"  ✓ Observation keys: {list(obs.keys())}")
except Exception as e:
    print(f"  ✗ reset() 失败: {e}")
    sys.exit(1)

# 4. 验证环境 step
print("=" * 60)
print("[4/8] 验证环境 step...")
try:
    action = np.array([0.0])
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"  ✓ step() 成功")
    print(f"  ✓ Reward: {reward:.2f}")
    print(f"  ✓ Terminated: {terminated}, Truncated: {truncated}")
    print(f"  ✓ Info keys: {list(info.keys())}")
except Exception as e:
    print(f"  ✗ step() 失败: {e}")
    sys.exit(1)

# 5. 验证碰撞检测
print("=" * 60)
print("[5/8] 验证碰撞检测...")
try:
    collision = env.unwrapped._check_hard_collision()
    soft_boundary = env.unwrapped._check_soft_boundary()
    print(f"  ✓ 硬碰撞检测: {collision}")
    print(f"  ✓ 软边界检测: {soft_boundary}")
except Exception as e:
    print(f"  ✗ 碰撞检测失败: {e}")
    sys.exit(1)

# 6. 验证奖励计算
print("=" * 60)
print("[6/8] 验证奖励计算...")
try:
    # 测试各种场景
    env.reset(seed=42)
    
    # 测试正常 step
    obs, r, term, trunc, info = env.step(np.array([0.0]))
    print(f"  ✓ 正常 step reward: {r:.2f}")
    
    # 测试成功条件
    success = env.unwrapped._is_success()
    print(f"  ✓ 成功条件检查: {success}")
    
    # 测试折叠检测
    jackknifed = abs(env.unwrapped.truck.g_rad) > np.deg2rad(JACKKNIFE_DEG)
    print(f"  ✓ 折叠检测: {jackknifed}")
    
except Exception as e:
    print(f"  ✗ 奖励计算失败: {e}")
    sys.exit(1)

# 7. 验证 Curriculum Learning
print("=" * 60)
print("[7/8] 验证 Curriculum Learning...")
try:
    for i, stage in enumerate(CURRICULUM_STAGES):
        env_stage = gym.make('TruckParking-v0', curriculum_stage=i)
        env_stage.reset(seed=42)
        print(f"  ✓ Stage {i}: start=({stage['start_x']}, {stage['start_y']}), "
              f"max_steps={stage['max_steps']}, goal_tol={stage['goal_tol']}")
        env_stage.close()
except Exception as e:
    print(f"  ✗ Curriculum 验证失败: {e}")
    sys.exit(1)

# 8. 验证 DDPG+HER 算法配置
print("=" * 60)
print("[8/8] 验证 DDPG+HER 算法配置...")
try:
    from stable_baselines3 import DDPG
    from stable_baselines3.her.her_replay_buffer import HerReplayBuffer
    from stable_baselines3.common.noise import OrnsteinUhlenbeckActionNoise
    
    # 创建测试环境
    test_env = gym.make('TruckParking-v0', curriculum_stage=0)
    
    n_actions = test_env.action_space.shape[-1]
    noise = OrnsteinUhlenbeckActionNoise(
        mean=np.zeros(n_actions),
        sigma=0.15 * np.ones(n_actions),
        theta=0.15,
        dt=0.05,
    )
    
    model = DDPG(
        policy='MultiInputPolicy',
        env=test_env,
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
    
    print("  ✓ DDPG 模型创建成功")
    print(f"  ✓ Policy: MultiInputPolicy")
    print(f"  ✓ Replay buffer: HER (n_sampled_goal=4)")
    print(f"  ✓ Action noise: OrnsteinUhlenbeck (σ=0.15)")
    print(f"  ✓ Network arch: [256, 256, 128]")
    print(f"  ✓ Learning rate: 1e-3")
    print(f"  ✓ Buffer size: 200000")
    
    test_env.close()
    
except Exception as e:
    print(f"  ✗ DDPG+HER 配置失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("=" * 60)
print("✅ 所有算法通路验证通过！")
print("=" * 60)

env.close()
