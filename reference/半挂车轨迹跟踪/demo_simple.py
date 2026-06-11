#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化图形演示 - 避免中文字体问题
"""

import numpy as np
import matplotlib.pyplot as plt
from set_parameters import set_parameters
from create_trajectory import create_trajectory

def demo_simple():
    """简化图形演示"""
    print("半挂车轨迹跟踪系统 - 简化图形演示")
    
    # 设置参数
    Params = set_parameters()
    
    # 创建轨迹
    Trajectory = create_trajectory(Params)
    
    # 绘制轨迹地图（英文标签）
    plt.figure(figsize=(10, 8))
    
    # 绘制参考轨迹
    traj_x = [state[0] for state in Trajectory['States'][0]]
    traj_y = [state[1] for state in Trajectory['States'][0]]
    plt.plot(traj_x, traj_y, 'r--', label='Reference Trajectory', linewidth=2)
    
    # 创建模拟路径（跟随轨迹但有误差）
    sim_x = []
    sim_y = []
    for i, state in enumerate(Trajectory['States'][0]):
        # 添加一些跟踪误差
        error = np.random.normal(0, 0.3, 2)
        sim_x.append(state[0] + error[0])
        sim_y.append(state[1] + error[1])
    
    plt.plot(sim_x, sim_y, 'b-', label='Actual Path', linewidth=1)
    
    # 标记起点和终点
    plt.plot(traj_x[0], traj_y[0], 'go', markersize=10, label='Start')
    plt.plot(traj_x[-1], traj_y[-1], 'ro', markersize=10, label='End')
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Tractor-Trailer Path Tracking')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.show()
    
    # 绘制控制输入示例
    Time = np.linspace(0, 50, 500)
    speed = 1.5 + 0.5 * np.sin(Time * 0.2)
    steering = 0.1 * np.sin(Time * 0.5)
    
    plt.figure(figsize=(12, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(Time, speed)
    plt.ylabel('Speed (m/s)')
    plt.title('Control Inputs')
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 1, 2)
    plt.plot(Time, steering * 180 / np.pi)  # 转换为度
    plt.ylabel('Steering Angle (deg)')
    plt.xlabel('Time (s)')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    print("简化图形演示完成！")

if __name__ == "__main__":
    demo_simple()
