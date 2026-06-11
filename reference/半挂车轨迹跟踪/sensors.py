#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
传感器模块 - Python版本
测量系统状态
"""

import numpy as np

def sensors(State, Params):
    """
    传感器函数 - 测量状态
    
    参数:
        State: 当前状态
        Params: 参数结构体
    
    返回:
        Z: 测量值 [x, y, phi]
    """
    # 传感器测量: [x; y; phi]
    Z = np.array([State[0], State[1], State[3]])
    
    # 添加传感器噪声
    noise_variance = Params['Est']['Sensor_Noise_Variance']
    noise = np.random.randn(3) * np.sqrt(noise_variance)
    Z = Z + noise
    
    return Z
