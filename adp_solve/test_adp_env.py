"""
ADP 控制器集成测试
使用本项目的 Gymnasium 环境测试 ADP 算法
"""
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import gymnasium as gym
import env.truck_parking_env
from adp_solve.adp_controller import ADPController


def test_adp_on_env(steps=2000):
    """在 Gymnasium 环境中测试 ADP 控制器"""
    
    # 创建环境
    env = gym.make('TruckParking-v0', curriculum_stage=0)
    obs, _ = env.reset(seed=42)
    
    # 初始化 ADP 控制器
    adp = ADPController(
        truck_wheelbase=env.unwrapped.truck.truckWheelbase,
        trailer_wheelbase=env.unwrapped.truck.trailerWheelbase
    )
    
    print("=" * 60)
    print("ADP 控制器集成测试")
    print("=" * 60)
    print(f"初始 LQR 增益 K: {adp.K}")
    print(f"初始位置: ({env.unwrapped.truck.x_m:.2f}, {env.unwrapped.truck.y_m:.2f})")
    print(f"目标位置: (57.5, 53.62)")
    print("=" * 60)
    
    # 记录轨迹
    trajectory = []
    total_reward = 0
    
    for step in range(steps):
        # 获取当前状态
        state = np.array([
            env.unwrapped.truck.x_m,
            env.unwrapped.truck.y_m,
            env.unwrapped.truck.h_rad,
            env.unwrapped.truck.g_rad
        ])
        
        # 计算 ADP 控制输入
        u = adp.compute_control(state, velocity=env.unwrapped.truck.v_mps)
        
        # 归一化到 [-1, 1]
        action = np.array([u / 35.0])
        
        # 执行动作
        obs, reward, terminated, truncated, info = env.step(action)
        
        total_reward += reward
        trajectory.append([
            env.unwrapped.truck.x_m,
            env.unwrapped.truck.y_m,
            env.unwrapped.truck.h_rad,
            env.unwrapped.truck.g_rad,
            u
        ])
        
        # 每 100 步打印一次
        if step % 100 == 0:
            print(f"Step {step:4d}: pos=({env.unwrapped.truck.x_m:.2f}, {env.unwrapped.truck.y_m:.2f}), "
                  f"h={np.degrees(env.unwrapped.truck.h_rad):.1f}°, "
                  f"g={np.degrees(env.unwrapped.truck.g_rad):.1f}°, "
                  f"u={u:.2f}°")
        
        if terminated or truncated:
            print(f"\nEpisode ended at step {step}")
            print(f"  Success: {info.get('succeeded', False)}")
            print(f"  Collided: {info.get('collided', False)}")
            print(f"  Jackknifed: {info.get('jackknifed', False)}")
            break
    else:
        print(f"\nReached max steps ({steps})")
    
    print(f"\n总奖励: {total_reward:.2f}")
    print(f"最终位置: ({env.unwrapped.truck.x_m:.2f}, {env.unwrapped.truck.y_m:.2f})")
    print(f"最终朝向: {np.degrees(env.unwrapped.truck.h_rad):.1f}°")
    print(f"最终铰接角: {np.degrees(env.unwrapped.truck.g_rad):.1f}°")
    
    env.close()
    
    return np.array(trajectory)


if __name__ == '__main__':
    trajectory = test_adp_on_env(steps=2000)
