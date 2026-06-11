#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
增益矩阵设置模块 - Python版本
计算LQR增益矩阵
"""

import numpy as np
from scipy.linalg import solve_continuous_are

def set_gain_matrix(State, Input, Params):
    """
    设置增益矩阵函数 - 计算LQR增益
    
    参数:
        State: 状态
        Input: 输入
        Params: 参数结构体
    
    返回:
        K: LQR增益矩阵
    """
    
    # 计算线性化系统矩阵
    A = compute_A_matrix(State, Input, Params)
    B = compute_B_matrix(State, Input, Params)
    
    # 获取LQR权重矩阵
    Q = Params['Ctl']['Q']
    R = Params['Ctl']['R']
    
    # 求解连续时间代数Riccati方程
    try:
        P = solve_continuous_are(A, B, Q, R)
        # 计算LQR增益
        K = np.linalg.inv(R) @ B.T @ P
    except:
        # 如果求解失败，返回零矩阵
        K = np.zeros((2, 4))
    
    return K

def compute_A_matrix(State, Input, Params):
    """
    计算A矩阵（系统矩阵）
    简化版本 - 在实际应用中应该根据动力学方程计算精确的雅可比矩阵
    """
    Nx = Params['Dyn']['Nx']
    return np.eye(Nx)  # 简化版本

def compute_B_matrix(State, Input, Params):
    """
    计算B矩阵（输入矩阵）
    简化版本 - 在实际应用中应该根据动力学方程计算精确的雅可比矩阵
    """
    Nx = Params['Dyn']['Nx']
    Nu = Params['Dyn']['Nu']
    return np.ones((Nx, Nu))  # 简化版本
