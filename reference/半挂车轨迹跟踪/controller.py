#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制器模块 - Python版本
使用LQR控制器提供转向命令
"""

import numpy as np

# 使用全局变量来模拟MATLAB的persistent变量
K_Prev = None

def controller(State, Params, Goal_Idx, Trajectory):
    """
    控制器函数 - 计算控制命令
    
    参数:
        State: 当前状态
        Params: 参数结构体
        Goal_Idx: 目标点索引
        Trajectory: 轨迹结构体
    
    返回:
        Command: 控制命令
    """
    global K_Prev
    
    # 确定行驶方向
    if Params['Sim']['Direction'] == 'reverse':
        Dir = 1  # Python索引从0开始，但这里使用1表示反向
    else:
        Dir = 0  # 0表示正向
    
    if Goal_Idx < 1:
        # 到达道路末端，命令停止
        Exit_Flag = True
        Goal_Idx = Params['Traj']['Npts'] - 1  # 选择最后一个点（Python索引从0开始）
    else:
        Exit_Flag = False
    
    # 获取目标状态和增益矩阵
    Goal = Trajectory['States'][Dir][Goal_Idx]
    K_New = Trajectory['Gains'][Dir][Goal_Idx]
    
    # 初始化前一个增益矩阵
    if K_Prev is None:
        K_Prev = K_New
    
    # 平滑控制器切换
    alpha = Params['Ctl']['Gain_Smoothing']
    K = K_New * (1 - alpha) + K_Prev * alpha
    
    # 确定控制命令
    Command = -K @ (State - Goal)
    
    # 如果到达道路末端，命令停止
    if Exit_Flag:
        Command[0] = 0
    
    K_Prev = K
    
    return Command
