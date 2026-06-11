#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动力学模型模块 - Python版本
计算半挂车系统的动力学
"""

import numpy as np

def dynamics(States, Inputs, Params):
    """
    计算半挂车系统的动力学
    
    参数:
        States: (4xN) 状态矩阵
        Inputs: (2xN) 输入矩阵  
        Params: 参数结构体
    
    返回:
        dStates: 状态导数
    """
    
    # States 是 (4xN) 状态矩阵
    # Inputs 是 (2xN) 输入矩阵
    # Params 是参数结构体
    
    # x = States[0, :]
    # y = States[1, :]
    th = States[2, :]  # 挂车角度
    phi = States[3, :]  # 驾驶室角度
    
    v = Inputs[0, :]  # 速度
    psi = Inputs[1, :]  # 转向角度
    
    Lt = Params['Dyn']['Lt']  # 挂车长度
    Lc = Params['Dyn']['Lc']  # 驾驶室长度
    
    # 计算状态导数
    dStates = np.array([
        -v * np.cos(phi) * np.cos(psi) * np.sin(th),
        v * np.cos(phi) * np.cos(psi) * np.cos(th),
        (v * np.cos(psi) * np.sin(phi)) / Lt,
        (v * (Lt * np.sin(psi) - Lc * np.cos(psi) * np.sin(phi))) / (Lc * Lt)
    ])
    
    return dStates
