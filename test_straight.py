#!/usr/bin/env python3
"""测试直线倒车入库"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle

def test_straight_reverse():
    """测试直线倒车"""
    
    print("="*60)
    print("测试直线倒车入库")
    print("="*60)
    
    # 创建车辆
    truck = TruckTrailerOnAxle()
    
    # 设置初始状态
    init_x = 58.25
    init_y = 90.91
    init_h = np.pi / 2  # 车头朝下
    init_g = 0.0
    
    truck.set_state([init_x, init_y, init_h, init_g])
    truck.v_mps = -1.0  # 倒车，向上移动
    truck.s_deg = 0
    
    goal_x = 58.25
    goal_y = 50.56
    
    print(f"\n初始位置: ({init_x:.2f}, {init_y:.2f})")
    print(f"目标位置: ({goal_x:.2f}, {goal_y:.2f})")
    print(f"初始航向: {np.rad2deg(init_h):.1f}°")
    print()
    
    # 仿真循环
    max_steps = 2000
    for step in range(max_steps):
        # 检查是否到达终点
        dist_to_end = np.sqrt((truck.x_m - goal_x)**2 + (truck.y_m - goal_y)**2)
        if dist_to_end < 1.0:
            print(f"\n✓ 到达终点!")
            print(f"  总步数: {step}")
            print(f"  最终距离: {dist_to_end:.2f}m")
            print(f"  最终位置: ({truck.x_m:.2f}, {truck.y_m:.2f})")
            print(f"  最终航向: {np.rad2deg(truck.h_rad):.1f}°")
            print(f"  最终铰接角: {np.rad2deg(truck.g_rad):.1f}°")
            return True
        
        # 简单的反馈控制：保持直线
        error_x = goal_x - truck.x_m
        error_h = np.pi/2 - truck.h_rad  # 目标是保持90度
        
        # 转向控制
        kp_x = 0.5  # 位置比例增益
        kp_h = 0.3  # 航向比例增益
        
        desired_steering = kp_x * error_x + kp_h * error_h
        
        # 限制转向角
        desired_steering_deg = np.rad2deg(desired_steering)
        desired_steering_deg = np.clip(desired_steering_deg, -35.0, 35.0)
        
        truck.set_steering(desired_steering_deg)
        truck.drive(0.05)  # 时间步长
        
        # 日志
        if step % 100 == 0:
            print(f"Step {step:4d}: pos=({truck.x_m:6.2f}, {truck.y_m:6.2f}), "
                  f"heading={np.rad2deg(truck.h_rad):6.1f}°, "
                  f"g={np.rad2deg(truck.g_rad):6.1f}°, "
                  f"delta={desired_steering_deg:5.1f}°, "
                  f"dist={dist_to_end:5.2f}m")
    
    print(f"\n✗ 超时了! 最终距离: {dist_to_end:.2f}m")
    return False

if __name__ == "__main__":
    success = test_straight_reverse()
    sys.exit(0 if success else 1)
