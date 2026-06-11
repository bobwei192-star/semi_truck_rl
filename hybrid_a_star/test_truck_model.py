#!/usr/bin/env python3
"""测试牵引车-挂车模型"""

import sys
import os
sys.path.insert(0, '/home/worker/semi_truck_rl')

import numpy as np
from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle

def compute_trailer_center(truck):
    """计算挂车中心点"""
    x = truck.x_m
    y = truck.y_m
    h = truck.h_rad
    g = truck.g_rad
    
    # 挂车后轴中心位置
    # 从车头位置向后移动truckWheelbase到挂车前轴，再向后移动trailerWheelbase/2到挂车中心
    trailer_x = x - (truck.truckWheelbase + truck.trailerWheelbase/2) * np.cos(h + g)
    trailer_y = y - (truck.truckWheelbase + truck.trailerWheelbase/2) * np.sin(h + g)
    
    return trailer_x, trailer_y

# 创建车辆
truck = TruckTrailerOnAxle()

# 设置初始状态
truck.set_state([56.25, 9.91, -np.pi/2, 0.0])
truck.v_mps = 2.0  # 倒车速度（负值表示倒车）

print("初始状态：")
print(f"  车头位置: ({truck.x_m:.2f}, {truck.y_m:.2f})")
print(f"  车头朝向: {np.rad2deg(truck.h_rad):.1f}°")
print(f"  铰接角: {np.rad2deg(truck.g_rad):.1f}°")
trailer_x, trailer_y = compute_trailer_center(truck)
print(f"  挂车中心: ({trailer_x:.2f}, {trailer_y:.2f})")

# 测试1：直线倒车
print("\n测试1：直线倒车（方向盘=0）")
for i in range(5):
    truck.set_steering(0)
    truck.drive(1.0)
    trailer_x, trailer_y = compute_trailer_center(truck)
    print(f"  Step {i+1}: pos=({truck.x_m:.2f}, {truck.y_m:.2f}), g={np.rad2deg(truck.g_rad):.1f}°, trailer=({trailer_x:.2f}, {trailer_y:.2f})")

# 重置
truck.set_state([56.25, 9.91, -np.pi/2, 0.0])
truck.v_mps = 2.0

# 测试2：向左打方向盘倒车
print("\n测试2：向左打方向盘10°倒车")
for i in range(10):
    truck.set_steering(10)
    truck.drive(1.0)
    trailer_x, trailer_y = compute_trailer_center(truck)
    print(f"  Step {i+1}: pos=({truck.x_m:.2f}, {truck.y_m:.2f}), g={np.rad2deg(truck.g_rad):.1f}°, trailer=({trailer_x:.2f}, {trailer_y:.2f})")

# 重置
truck.set_state([56.25, 9.91, -np.pi/2, 0.0])
truck.v_mps = 2.0

# 测试3：向右打方向盘倒车
print("\n测试3：向右打方向盘10°倒车")
for i in range(10):
    truck.set_steering(-10)
    truck.drive(1.0)
    trailer_x, trailer_y = compute_trailer_center(truck)
    print(f"  Step {i+1}: pos=({truck.x_m:.2f}, {truck.y_m:.2f}), g={np.rad2deg(truck.g_rad):.1f}°, trailer=({trailer_x:.2f}, {trailer_y:.2f})")
