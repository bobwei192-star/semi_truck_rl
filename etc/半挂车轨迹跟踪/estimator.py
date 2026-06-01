#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
估计器模块 - Python版本
使用扩展卡尔曼滤波器进行状态估计
"""

import numpy as np
from dynamics import dynamics

def estimator(Z, Input, Params, Xhat, P):
    """
    估计器函数 - 扩展卡尔曼滤波器
    
    参数:
        Z: 测量值
        Input: 输入
        Params: 参数结构体
        Xhat: 当前状态估计
        P: 当前协方差矩阵
    
    返回:
        Xhat_new: 新状态估计
        P_new: 新协方差矩阵
    """
    
    # 预测步骤
    dt = Params['Sim']['dt']
    
    # 使用动力学模型进行状态预测
    dStates = dynamics(Xhat.reshape(-1, 1), Input.reshape(-1, 1), Params)
    Xhat_pred = Xhat + dStates.flatten() * dt
    
    # 计算雅可比矩阵（简化版本）
    A = compute_jacobian(Xhat, Input, Params)
    
    # 预测协方差
    Q = Params['Est']['Q']
    P_pred = A @ P @ A.T + Q
    
    # 更新步骤
    H = Params['Est']['H']
    R = Params['Est']['R']
    
    # 计算卡尔曼增益
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    
    # 更新状态估计
    Z_pred = H @ Xhat_pred
    Xhat_new = Xhat_pred + K @ (Z - Z_pred)
    
    # 更新协方差
    I = np.eye(P_pred.shape[0])
    P_new = (I - K @ H) @ P_pred
    
    return Xhat_new, P_new

def compute_jacobian(State, Input, Params):
    """
    计算雅可比矩阵（简化版本）
    
    在实际应用中，应该根据动力学方程计算精确的雅可比矩阵
    这里使用单位矩阵作为简化
    """
    Nx = Params['Dyn']['Nx']
    return np.eye(Nx)  # 简化版本
