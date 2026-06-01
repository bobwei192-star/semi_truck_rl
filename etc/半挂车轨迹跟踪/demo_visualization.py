#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图形化演示 - 展示半挂车轨迹跟踪系统的可视化效果
"""

import numpy as np
import matplotlib.pyplot as plt
from set_parameters import set_parameters
from create_trajectory import create_trajectory
from plot_functions import plot_map

def demo_visualization():
    """图形化演示函数"""
    print("半挂车轨迹跟踪系统 - 图形化演示")
    print("=" * 50)
    
    # 设置参数
    print("设置系统参数...")
    Params = set_parameters()
    
    # 创建轨迹
    print("创建轨迹...")
    Trajectory = create_trajectory(Params)
    
    # 生成模拟数据（简化版本）
    print("生成模拟数据...")
    Nsteps = min(1000, Params['Sim']['Nsteps'])  # 限制步数以加快演示
    Time = Params['Sim']['Time'][:Nsteps]
    
    # 创建模拟状态数据（简化版本，实际应该运行模拟器）
    States = np.zeros((4, Nsteps))
    
    # 使用轨迹作为基础，添加一些噪声来模拟跟踪过程
    traj_states = Trajectory['States'][0]  # 正向轨迹
    
    for i in range(Nsteps):
        traj_idx = min(i % len(traj_states), len(traj_states) - 1)
        base_state = traj_states[traj_idx]
        
        # 添加一些跟踪误差
        noise = np.random.normal(0, 0.5, 4)
        States[:, i] = base_state + noise
    
    # 绘制轨迹跟踪地图
    print("绘制轨迹跟踪地图...")
    plot_map(States, Params, Trajectory)
    
    # 绘制输入信号示例
    print("绘制输入信号示例...")
    Inputs = np.array([
        np.ones(Nsteps) * 1.5 + np.random.normal(0, 0.1, Nsteps),  # 速度
        np.sin(Time * 0.5) * 0.2 + np.random.normal(0, 0.05, Nsteps)  # 转向角度
    ])
    
    plt.figure(figsize=(12, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(Time, Inputs[0, :])
    plt.ylabel('速度 (m/s)')
    plt.title('控制输入示例')
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(Time, Inputs[1, :] * 180 / np.pi)  # 转换为度
    plt.ylabel('转向角度 (度)')
    plt.xlabel('时间 (秒)')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # 绘制估计器性能示例
    print("绘制估计器性能示例...")
    Trace_Cov = np.exp(-Time * 0.1) * 10 + np.random.normal(0, 0.1, Nsteps)  # 指数衰减
    State_Error = np.random.normal(0, 0.3, (4, Nsteps))
    
    plt.figure(figsize=(12, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(Time, Trace_Cov)
    plt.ylabel('协方差迹')
    plt.title('估计器性能示例')
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(Time, State_Error[0, :], label='x误差')
    plt.plot(Time, State_Error[1, :], label='y误差')
    plt.ylabel('位置误差 (m)')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    plt.plot(Time, State_Error[2, :] * 180 / np.pi, label='θ误差')
    plt.plot(Time, State_Error[3, :] * 180 / np.pi, label='φ误差')
    plt.ylabel('角度误差 (度)')
    plt.xlabel('时间 (秒)')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    print("图形化演示完成！")

if __name__ == "__main__":
    demo_visualization()
