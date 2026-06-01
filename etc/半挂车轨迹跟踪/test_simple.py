#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化测试版本 - 验证基本功能
"""

import numpy as np
import matplotlib.pyplot as plt

def test_basic_functionality():
    """测试基本功能"""
    print("测试半挂车轨迹跟踪系统的基本功能...")
    
    # 测试参数设置
    print("1. 测试参数设置...")
    try:
        from set_parameters import set_parameters
        Params = set_parameters()
        print("✓ 参数设置成功")
        print(f"  状态数量: {Params['Dyn']['Nx']}")
        print(f"  输入数量: {Params['Dyn']['Nu']}")
        print(f"  模拟步数: {Params['Sim']['Nsteps']}")
    except Exception as e:
        print(f"✗ 参数设置失败: {e}")
        return
    
    # 测试轨迹创建
    print("2. 测试轨迹创建...")
    try:
        from create_trajectory import create_trajectory
        Trajectory = create_trajectory(Params)
        print("✓ 轨迹创建成功")
        print(f"  轨迹点数: {len(Trajectory['States'][0])}")
    except Exception as e:
        print(f"✗ 轨迹创建失败: {e}")
        return
    
    # 测试动力学模型
    print("3. 测试动力学模型...")
    try:
        from dynamics import dynamics
        test_state = np.array([[0], [0], [0], [0]])  # 4x1状态
        test_input = np.array([[1], [0]])  # 2x1输入
        dStates = dynamics(test_state, test_input, Params)
        print("✓ 动力学模型成功")
        print(f"  状态导数形状: {dStates.shape}")
    except Exception as e:
        print(f"✗ 动力学模型失败: {e}")
        return
    
    # 测试控制器
    print("4. 测试控制器...")
    try:
        from controller import controller
        test_state = np.array([0, 0, 0, 0])
        goal_idx = 0
        command = controller(test_state, Params, goal_idx, Trajectory)
        print("✓ 控制器成功")
        print(f"  控制命令: {command}")
    except Exception as e:
        print(f"✗ 控制器失败: {e}")
        return
    
    # 测试绘图功能
    print("5. 测试绘图功能...")
    try:
        # 创建一些测试数据
        Time = np.linspace(0, 10, 100)
        Inputs = np.array([
            np.sin(Time) + 1,  # 速度
            np.cos(Time) * 0.1  # 转向角度
        ])
        
        from plot_functions import plot_inputs
        print("✓ 绘图功能可用")
        # 注释掉实际绘图，避免阻塞
        # plot_inputs(Time, Inputs)
    except Exception as e:
        print(f"✗ 绘图功能失败: {e}")
        return
    
    print("\n🎉 所有基本功能测试通过！")
    print("系统已成功转换为Python版本")

if __name__ == "__main__":
    test_basic_functionality()
