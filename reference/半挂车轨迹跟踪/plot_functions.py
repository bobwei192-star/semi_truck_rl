#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
绘图函数模块 - Python版本
绘制各种结果图表
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def plot_inputs(Time, Inputs):
    """
    绘制输入信号
    
    参数:
        Time: 时间向量
        Inputs: 输入矩阵
    """
    plt.figure(figsize=(12, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(Time, Inputs[0, :])
    plt.ylabel('速度 (m/s)')
    plt.title('控制输入')
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(Time, Inputs[1, :] * 180 / np.pi)  # 转换为度
    plt.ylabel('转向角度 (度)')
    plt.xlabel('时间 (秒)')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

def plot_estimator(Time, Trace_Cov, State_Error):
    """
    绘制估计器性能
    
    参数:
        Time: 时间向量
        Trace_Cov: 协方差迹
        State_Error: 状态误差
    """
    plt.figure(figsize=(12, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(Time, Trace_Cov)
    plt.ylabel('协方差迹')
    plt.title('估计器性能')
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

def animate_system(States, Params, Trajectory, Goal_Idx):
    """
    动画显示系统运动
    
    参数:
        States: 状态矩阵
        Params: 参数结构体
        Trajectory: 轨迹结构体
        Goal_Idx: 目标点索引
    """
    print("动画功能待实现...")
    # 这里可以添加matplotlib动画代码
    # 由于动画代码比较复杂，这里先提供一个占位符
    
    # 简单绘制轨迹和状态
    plt.figure(figsize=(10, 8))
    
    # 绘制轨迹
    traj_x = [state[0] for state in Trajectory['States'][0]]
    traj_y = [state[1] for state in Trajectory['States'][0]]
    plt.plot(traj_x, traj_y, 'r--', label='参考轨迹')
    
    # 绘制实际路径
    plt.plot(States[0, :], States[1, :], 'b-', label='实际路径')
    
    # 绘制起点和终点
    plt.plot(States[0, 0], States[1, 0], 'go', markersize=10, label='起点')
    plt.plot(States[0, -1], States[1, -1], 'ro', markersize=10, label='终点')
    
    plt.xlabel('X位置 (m)')
    plt.ylabel('Y位置 (m)')
    plt.title('半挂车轨迹跟踪')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def plot_map(States, Params, Trajectory):
    """
    绘制地图
    
    参数:
        States: 状态矩阵
        Params: 参数结构体
        Trajectory: 轨迹结构体
    """
    plt.figure(figsize=(10, 8))
    
    # 绘制轨迹
    traj_x = [state[0] for state in Trajectory['States'][0]]
    traj_y = [state[1] for state in Trajectory['States'][0]]
    plt.plot(traj_x, traj_y, 'r--', label='参考轨迹')
    
    # 绘制实际路径
    plt.plot(States[0, :], States[1, :], 'b-', label='实际路径')
    
    plt.xlabel('X位置 (m)')
    plt.ylabel('Y位置 (m)')
    plt.title('轨迹跟踪地图')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
