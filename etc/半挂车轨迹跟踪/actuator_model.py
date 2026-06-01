#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
执行器模型模块 - Python版本
处理控制命令的执行器限制
"""

import numpy as np

def actuator_model(Command, Params):
    """
    执行器模型函数 - 应用执行器限制
    
    参数:
        Command: 控制命令
        Params: 参数结构体
    
    返回:
        Input: 经过限制的输入
    """
    
    # 应用执行器限制
    Actuator_Limits = Params['Ctl']['Actuator_Limits']
    
    # 限制速度
    speed = np.clip(Command[0], Actuator_Limits[0, 0], Actuator_Limits[0, 1])
    
    # 限制转向角度
    steering = np.clip(Command[1], Actuator_Limits[1, 0], Actuator_Limits[1, 1])
    
    Input = np.array([speed, steering])
    
    return Input
