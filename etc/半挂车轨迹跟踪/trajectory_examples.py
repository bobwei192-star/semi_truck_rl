#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹示例模块 - Python版本
包含几个示例"道路"
"""

import numpy as np

def trajectory_examples(example_number):
    """
    轨迹示例函数
    根据示例编号返回相应的轨迹参数
    
    参数:
        example_number: 示例编号 (1-8)
    
    返回:
        包含轨迹参数的字典
    """
    Params = {}
    Params['Traj'] = {}
    Params['Sim'] = {}
    Params['Ctl'] = {}
    
    Params['Sim']['Example_Number'] = example_number
    
    if example_number == 1:
        Params['Traj']['Func'] = False
        Params['Traj']['SetPts'] = np.array([
            [0, 0],
            [10, 5],
            [20, 9],
            [40, 9],
            [60, 0],
            [80, -10]
        ])
        Params['Traj']['Order'] = 4
        Params['Sim']['IC_Error'] = np.array([0, 0, 0, 0])
        Max_Pos_Err = 0.5
        Params['Ctl']['Minimum_Goal_Distance'] = 1  # (m) 向前看多远
        Simulation_Duration = 60
        Control_Frequency = 50   # Hz
        
    elif example_number == 2:
        # 这个看起来工作得相当好
        Params['Traj']['Func'] = False
        Params['Traj']['SetPts'] = np.array([
            [0, 0],
            [10, 5],
            [20, 9],
            [40, 9],
            [60, 0],
            [65, -2.5],
            [70, -5],
            [75, -8],
            [80, -12]
        ])
        Params['Traj']['Order'] = 4
        Params['Sim']['IC_Error'] = np.array([0, 0, 0, 0])
        Max_Pos_Err = 0.5
        Params['Ctl']['Minimum_Goal_Distance'] = 1  # (m) 向前看多远
        Simulation_Duration = 50
        Control_Frequency = 50   # Hz
        
    elif example_number == 3:
        Params['Traj']['Func'] = False
        Params['Traj']['SetPts'] = np.array([
            [0, 0],
            [5, 0],
            [10, 1],
            [15, 3],
            [20, 2],
            [25, 0],
            [30, -3],
            [35, -8],
            [40, -13],
            [45, -20],
            [50, -30],
            [55, -40]
        ])
        Params['Traj']['Order'] = 4
        Params['Sim']['IC_Error'] = np.array([0, 0, 0, 0])
        Max_Pos_Err = 1.5
        Params['Ctl']['Minimum_Goal_Distance'] = 1  # (m) 向前看多远
        Simulation_Duration = 125
        Control_Frequency = 40   # Hz
        
    elif example_number == 4:
        # 演示中使用 - 主要结果
        Params['Traj']['Func'] = True
        x = np.linspace(0, 150, Params['Traj'].get('Npts', 100) + 1)
        Params['Traj']['x'] = x
        Params['Traj']['y'] = np.sin(x/20) * 25 * x**2 / (150**2)
        Params['Sim']['IC_Error'] = np.array([0, 0, 0, 0])
        Max_Pos_Err = 1.5  # 用于设置LQR增益
        Params['Ctl']['Minimum_Goal_Distance'] = 1  # (m) 向前看多远
        Simulation_Duration = 90
        Control_Frequency = 50   # Hz
        
    elif example_number == 5:
        # 工作正常
        Params['Traj']['Func'] = True
        x = np.linspace(0, 150, Params['Traj'].get('Npts', 100) + 1)
        Params['Traj']['x'] = x
        Params['Traj']['y'] = np.sin(x/15) * 25
        Params['Sim']['IC_Error'] = np.array([0, 0, 0, 0])
        Max_Pos_Err = 1.5
        Params['Ctl']['Minimum_Goal_Distance'] = 1  # (m) 向前看多远
        Simulation_Duration = 160
        Control_Frequency = 10   # Hz
        
    elif example_number == 6:
        # 演示中使用 - 附录1
        # 第5个例子的长版本（多次转弯）-- 很好的演示
        Params['Traj']['Func'] = True
        Params['Traj']['Npts'] = 250  # 覆盖点数
        Params['Traj']['Max_Rd_Idx'] = 245
        x = np.linspace(0, 350, Params['Traj']['Npts'] + 1)
        Params['Traj']['x'] = x
        Params['Traj']['y'] = np.sin(x/20 + np.pi/3) * np.cos(x/100) * 25
        Params['Sim']['IC_Error'] = np.array([0, 0, 0, 0])
        Max_Pos_Err = 1.5  # 用于设置LQR增益
        Params['Ctl']['Minimum_Goal_Distance'] = 1  # (m) 向前看多远
        Simulation_Duration = 235
        Control_Frequency = 50   # Hz
        
    elif example_number == 7:
        # 余弦道路
        Params['Traj']['Npts'] = 150
        Freq = 0.35
        Amp = 25
        Params['Traj']['Func'] = True
        x = np.linspace(0, 150, Params['Traj']['Npts'] + 1)
        Params['Traj']['x'] = x
        Params['Traj']['y'] = Amp * np.cos((2 * np.pi / Amp) * x * Freq + 0.1)
        Params['Sim']['IC_Error'] = np.array([0, 0, 0, 0])
        Max_Pos_Err = 1.5
        Params['Ctl']['Minimum_Goal_Distance'] = 1  # (m) 向前看多远
        Simulation_Duration = 250
        Control_Frequency = 50   # Hz
        
    elif example_number == 8:
        # 第5个例子的超长版本（多次转弯）-- 很好的演示
        Params['Traj']['Func'] = True
        Params['Traj']['Npts'] = 1000  # 覆盖点数
        Params['Traj']['Max_Rd_Idx'] = 995
        x = np.linspace(0, 350, Params['Traj']['Npts'] + 1)
        Params['Traj']['x'] = x
        Params['Traj']['y'] = np.sin(x/20 + np.pi/3) * np.cos(x/100) * 25
        Params['Sim']['IC_Error'] = np.array([0, 0, 0, 0])
        Max_Pos_Err = 1.5  # 用于设置LQR增益
        Params['Ctl']['Minimum_Goal_Distance'] = 2  # (m) 向前看多远
        Simulation_Duration = 235 * 4
        Control_Frequency = 50   # Hz
        
    else:
        raise ValueError('无效的示例编号')
    
    # 添加额外的参数
    Params['Max_Pos_Err'] = Max_Pos_Err
    Params['Simulation_Duration'] = Simulation_Duration
    Params['Control_Frequency'] = Control_Frequency
    
    return Params
