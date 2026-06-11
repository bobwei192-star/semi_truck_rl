#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
目标点选择模块 - Python版本
在轨迹上选择目标点
"""

import numpy as np

def select_goal_point(Xhat, Params, Trajectory):
    """
    选择目标点函数 - 在轨迹上找到最近的目标点
    
    参数:
        Xhat: 当前状态估计
        Params: 参数结构体
        Trajectory: 轨迹结构体
    
    返回:
        Goal_Idx: 目标点索引
    """
    
    # 确定行驶方向
    if Params['Sim']['Direction'] == 'reverse':
        Dir = 1
    else:
        Dir = 0
    
    Npts = Params['Traj']['Npts']
    Min_Dist = Params['Ctl']['Minimum_Goal_Distance']
    
    # 计算到所有轨迹点的距离
    distances = []
    for i in range(Npts):
        goal_state = Trajectory['States'][Dir][i]
        dist = np.sqrt((Xhat[0] - goal_state[0])**2 + (Xhat[1] - goal_state[1])**2)
        distances.append(dist)
    
    distances = np.array(distances)
    
    # 找到距离大于最小距离的第一个点
    for i in range(Npts):
        if distances[i] >= Min_Dist:
            return i
    
    # 如果没有找到合适的点，返回最后一个点
    return Npts - 1
