#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模拟器模块 - Python版本
模拟系统尝试跟踪轨迹
"""

import numpy as np
from dynamics import dynamics
from sensors import sensors
from estimator import estimator
from select_goal_point import select_goal_point
from controller import controller
from actuator_model import actuator_model

def simulator(Params, Trajectory):
    """
    模拟器函数 - 模拟系统跟踪轨迹
    
    参数:
        Params: 参数结构体
        Trajectory: 轨迹结构体
    
    返回:
        Results: 包含模拟结果的字典
    """
    
    Nsteps = Params['Sim']['Nsteps']
    Nx = Params['Dyn']['Nx']
    Nu = Params['Dyn']['Nu']
    
    # 初始化输出
    Time = Params['Sim']['Time']
    States = np.zeros((Nx, Nsteps))
    Inputs = np.zeros((Nu, Nsteps))
    Goal_Idx = np.zeros(Nsteps, dtype=int)  # 存储目标轨迹状态
    
    dt = Params['Sim']['dt']  # 设置时间步长
    Goal_Idx[0] = 0  # Python索引从0开始
    
    Params['Ctl']['dt'] = dt
    
    # 确定卡车行驶方向
    if Params['Sim']['Direction'] == 'reverse':
        States[:, 0] = Trajectory['States'][1][0] + Params['Sim']['IC_Error']  # 使用反向值
        Inputs[:, 0] = np.array([Params['Ctl']['Actuator_Limits'][0, 0], 0])  # 反向速度
    else:
        States[:, 0] = Trajectory['States'][0][0] + Params['Sim']['IC_Error']  # 使用正向值
        Inputs[:, 0] = np.array([Params['Ctl']['Actuator_Limits'][0, 1], 0])  # 正向速度
    
    # 初始化估计器
    Xhat0_Err = Params['Est']['Xhat0_Err']
    Xhat = States[:, 0] + Xhat0_Err
    P = Params['Est']['P0']
    
    # 跟踪状态误差
    State_Error = np.zeros((Nx, Nsteps))
    Trace_Cov = np.zeros(Nsteps)
    
    State_Error[:, 0] = States[:, 0] - Xhat
    Trace_Cov[0] = np.trace(P)
    
    # 使用欧拉积分进行模拟，输入采用零阶保持
    for k in range(Nsteps - 1):
        # 从步骤K到步骤(K+1)运行动力学
        dStates = dynamics(States[:, k], Inputs[:, k], Params)
        States[:, k + 1] = States[:, k] + dStates * dt
        
        # 测量新状态(K+1)
        Z = sensors(States[:, k + 1], Params)  # 测量状态
        
        # 调用估计器
        Xhat, P = estimator(Z, Inputs[:, k], Params, Xhat, P)  # 估计状态
        
        # 在轨迹上找到目标点
        Goal_Idx[k] = select_goal_point(Xhat, Params, Trajectory)
        
        # 运行控制器
        Command = controller(Xhat, Params, Goal_Idx[k], Trajectory)
        
        # 运行执行器模型
        Inputs[:, k + 1] = actuator_model(Command, Params)
        
        # 记录估计器性能
        State_Error[:, k + 1] = States[:, k + 1] - Xhat
        Trace_Cov[k + 1] = np.trace(P)
    
    # 返回结果
    Results = {
        'Time': Time,
        'States': States,
        'Inputs': Inputs,
        'Goal_Idx': Goal_Idx,
        'Trace_Cov': Trace_Cov,
        'State_Error': State_Error
    }
    
    return Results
