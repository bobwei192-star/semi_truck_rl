#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
参数设置模块 - Python版本
"""

import numpy as np
from trajectory_examples import trajectory_examples

def set_parameters():
    """
    设置系统参数
    返回包含所有参数的字典
    """
    Params = {}
    
    # 轨迹参数
    Params['Traj'] = {}
    Params['Traj']['Npts'] = 100
    
    # 运行模拟时，最后要跳过多少个点。这一点是需要的，因为控制器在驾驶时是 "向前看 "的。
    Params['Traj']['End_Buffer'] = 5
    
    # 以下代码行用于加载示例轨迹
    Example_Number = 7
    trajectory_params = trajectory_examples(Example_Number)
    
    # 合并轨迹参数
    Params['Traj'].update(trajectory_params['Traj'])
    
    # 模拟器参数
    Params['Sim'] = {}
    Params['Sim']['Tspan'] = [0, trajectory_params['Simulation_Duration']]  # 秒
    Params['Sim']['Nsteps'] = int(np.ceil(np.diff(Params['Sim']['Tspan'])[0] * trajectory_params['Control_Frequency']))
    Params['Sim']['Direction'] = 'forward'  # 'forward' 或 'reverse'
    Params['Sim']['RoadWidth'] = 80
    Params['Sim']['Frames_Per_Second'] = 24
    Params['Sim']['Speed_Up_Video'] = 10  # 比实时快多少倍
    Time = np.linspace(Params['Sim']['Tspan'][0], Params['Sim']['Tspan'][1], Params['Sim']['Nsteps'])
    dt = Time[1] - Time[0]
    Params['Sim']['dt'] = dt
    Params['Sim']['Time'] = Time
    Params['Sim']['IC_Error'] = trajectory_params['Sim']['IC_Error']
    
    # 估计器参数
    # 使用扩展卡尔曼滤波器进行状态估计
    Params['Est'] = {}
    
    x_var = 0.5   # 米
    y_var = 0.5   # 米
    phi_var = (np.pi/180) * 4   # 弧度
    Params['Est']['Sensor_Noise_Variance'] = np.array([x_var, y_var, phi_var])
    Params['Est']['R'] = np.diag(Params['Est']['Sensor_Noise_Variance']**2)
    
    dx_var = 0.03  # (m/s) x方向过程误差
    dy_var = 0.03  # (m/s) y方向过程误差
    dth_var = (np.pi/180) * 1   # (rad/s) 方向过程误差
    dphi = (np.pi/180) * 2   # (rad/s) 驾驶室角度过程误差
    Process_Noise_Variance = np.array([dx_var, dy_var, dth_var, dphi])
    Params['Est']['Q'] = np.diag(Process_Noise_Variance**2)
    
    x0_err = 0.1   # (m) 初始猜测误差
    y0_err = 0.1   # (m) 初始猜测误差
    th0_err = (np.pi/180) * 2    # (rad) 初始猜测误差
    phi0_err = (np.pi/180) * 4     # (rad) 初始猜测误差
    Xhat0_Err = np.random.randn(4) * np.array([x0_err, y0_err, th0_err, phi0_err])
    Params['Est']['Xhat0_Err'] = Xhat0_Err
    Params['Est']['P0'] = np.diag(Xhat0_Err**2)
    
    # 传感器: [x; y; phi]
    Params['Est']['H'] = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    # 控制参数
    # 使用线性二次调节器提供转向命令
    Params['Ctl'] = {}
    
    Params['Ctl']['Gain_Smoothing'] = 0.6  # K = Knew*(1-a) + Kold*(a); a 在 [0,1] 之间
    Params['Ctl']['Steering_Smoothing'] = 0.6  # 同上
    
    Max_Speed = 2
    Max_Steer = (np.pi/180) * 60
    
    Params['Ctl']['Actuator_Limits'] = np.array([
        [-Max_Speed, Max_Speed],    # 速度限制
        [-Max_Steer, Max_Steer]     # 转向限制
    ])
    
    Max_Turning_Rate = Max_Steer / 2   # (rad/s) 大约2秒达到最大转向
    Max_Acceleration = Max_Speed / 4   # (m/s) 大约2秒达到慢速运行速度
    Params['Ctl']['Rate_Limits'] = np.array([
        [-Max_Acceleration, Max_Acceleration],
        [-Max_Turning_Rate, Max_Turning_Rate]
    ])
    
    Params['Ctl']['Max_Target_Angle'] = (np.pi/180) * 45   # 接受与行驶方向偏离此值的目标
    
    Max_Tol_State_Error = np.array([
        trajectory_params['Max_Pos_Err'],   # (m) x位置
        trajectory_params['Max_Pos_Err'],   # (m) y位置
        (np.pi/180) * 1e4,  # (rad) 挂车角度theta (忽略这个)
        (np.pi/180) * 1e4   # (rad) 驾驶室角度phi
    ])
    Params['Ctl']['Q'] = np.diag(Max_Tol_State_Error**(-2))
    
    Max_Tol_Actuator_Effort = np.array([
        Max_Speed,    # (m/s) 驾驶室速度
        Max_Steer     # (rad) 转向角度
    ])
    Params['Ctl']['R'] = np.diag(Max_Tol_Actuator_Effort**(-2))
    
    # 向前看距离已移至轨迹示例
    Params['Ctl']['Minimum_Goal_Distance'] = trajectory_params['Ctl']['Minimum_Goal_Distance']
    
    # 动力学参数
    # 这些是半挂车的物理参数
    Params['Dyn'] = {}
    Params['Dyn']['Lt'] = 15   # 挂车长度 (m)
    Params['Dyn']['Lc'] = 4    # 驾驶室长度 (m)
    Params['Dyn']['Nu'] = 2    # 输入数量
    Params['Dyn']['Nx'] = 4    # 状态数量
    
    return Params
