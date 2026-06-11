#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
半挂车轨迹跟踪系统 - Python版本
MATLAB代码转换为Python
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are
import time
import os

# 导入自定义模块
from set_parameters import set_parameters
from create_trajectory import create_trajectory
from simulator import simulator
from plot_functions import plot_inputs, plot_estimator, animate_system

def main():
    """主程序 - 半挂车轨迹跟踪系统"""
    print("半挂车轨迹跟踪系统 - Python版本")
    print("=" * 50)
    
    # 获取用户定义的参数
    print("设置系统参数...")
    Params = set_parameters()
    
    # 创建轨迹（描述道路）
    print("创建轨迹...")
    Trajectory = create_trajectory(Params)
    
    # 运行模拟 - 驾驶半挂车沿着道路行驶
    print("开始模拟...")
    start_time = time.time()
    Results = simulator(Params, Trajectory)
    end_time = time.time()
    
    print(f"模拟完成，耗时: {end_time - start_time:.2f} 秒")
    
    # 解包结果
    Time = Results['Time']
    Goal_Idx = Results['Goal_Idx']
    States = Results['States']
    Inputs = Results['Inputs']
    Trace_Cov = Results['Trace_Cov']
    State_Error = Results['State_Error']
    
    # 显示结果给用户
    print("生成结果图表...")
    
    # 渲染视频或动画
    Render_Video = False
    if Render_Video:
        # 这里可以添加视频生成功能
        print("视频渲染功能待实现...")
    else:
        animate_system(States, Params, Trajectory, Goal_Idx)
    
    # 绘制输入信号
    plot_inputs(Time, Inputs)
    
    # 绘制估计器性能
    plot_estimator(Time, Trace_Cov, State_Error)
    
    print("程序执行完成！")

if __name__ == "__main__":
    main()
