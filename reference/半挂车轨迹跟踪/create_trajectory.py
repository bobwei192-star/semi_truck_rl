#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹创建模块 - Python版本
绘制半挂车行驶的道路
"""

import numpy as np
from set_gain_matrix import set_gain_matrix

def create_trajectory(Params):
    """
    创建轨迹函数 - 定义轨迹为一系列点和相关增益
    
    参数:
        Params: 参数结构体
    
    返回:
        Trajectory: 轨迹结构体
    """
    
    Npts = Params['Traj']['Npts']
    
    if not Params['Traj']['Func']:
        # 设置约束点
        SetPts = Params['Traj']['SetPts']
        Order = Params['Traj']['Order']
        
        # 多项式拟合
        P = np.polyfit(SetPts[:, 0], SetPts[:, 1], Order)
        x = np.linspace(np.min(SetPts[:, 0]), np.max(SetPts[:, 0]), Npts + 1)
        y = np.polyval(P, x)
    else:
        x = Params['Traj']['x']
        y = Params['Traj']['y']
    
    # 现在我们需要期望的挂车角度theta，定义为挂车中心线与正y轴之间的角度
    
    # 连续点之间的向量方向
    x_step = np.diff(x)
    y_step = np.diff(y)
    angle = np.arctan2(y_step, x_step)  # 与正x轴的角度
    th = angle - np.pi / 2
    
    # 假设驾驶室角度应始终尝试居中
    phi = np.zeros_like(th)
    
    # 截断向量
    x = x[:-1]
    y = y[:-1]
    
    # 现在进行增益矩阵计算
    Trajectory = {
        'States': [[None] * Npts, [None] * Npts],  # 正向和反向
        'Inputs': [[None] * Npts, [None] * Npts],
        'Gains': [[None] * Npts, [None] * Npts]
    }
    
    for i in range(Npts):
        # 正向行驶
        State = np.array([x[i], y[i], th[i], phi[i]])
        Input = np.array([1, 0])  # [速度; 转向]
        Trajectory['States'][0][i] = State
        Trajectory['Inputs'][0][i] = Input
        Trajectory['Gains'][0][i] = set_gain_matrix(State, Input, Params)
        
        # 反向行驶
        State_Rev = np.array([x[i], y[i], th[i] + np.pi, phi[i]])
        Input_Rev = np.array([-1, 0])  # [速度; 转向]
        Trajectory['States'][1][i] = State_Rev
        Trajectory['Inputs'][1][i] = Input_Rev
        Trajectory['Gains'][1][i] = set_gain_matrix(State_Rev, Input_Rev, Params)
    
    return Trajectory
