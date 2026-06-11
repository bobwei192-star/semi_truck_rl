#!/usr/bin/env python3
"""测试泊车控制器"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from another_solve.path_planner import PathPlanner
from another_solve.mpc_controller import SimpleMPCController
from config.scene_config import DT

def test_parking():
    """测试泊车过程"""
    
    print("="*60)
    print("测试泊车控制器")
    print("="*60)
    
    # 创建车辆
    truck = TruckTrailerOnAxle()
    
    # 创建路径规划器
    path_planner = PathPlanner()
    
    # 生成路径
    result = path_planner.generate_parking_path()
    path = result['path']
    print(f"\n路径生成完成，路径点数: {len(path)}")
    print(f"  起点: ({path[0, 0]:.2f}, {path[0, 1]:.2f})")
    print(f"  终点: ({path[-1, 0]:.2f}, {path[-1, 1]:.2f})")
    
    # 创建控制器
    controller = SimpleMPCController()
    controller.set_reference_path(path)
    
    # 设置初始状态
    init_x = 58.25
    init_y = 90.91
    init_h = np.pi / 2  # 车头朝下
    init_g = 0.0
    
    truck.set_state([init_x, init_y, init_h, init_g])
    truck.v_mps = -1.0  # 倒车，向上移动
    truck.s_deg = 0
    
    print(f"\n初始位置: ({init_x:.2f}, {init_y:.2f})")
    print(f"目标位置: (58.25, 50.56)")
    print(f"初始航向: {np.rad2deg(init_h):.1f}°")
    print()
    
    # 仿真循环
    max_steps = 3000
    for step in range(max_steps):
        # 检查是否到达终点
        dist_to_end = np.sqrt((truck.x_m - 58.25)**2 + (truck.y_m - 50.56)**2)
        if dist_to_end < 1.5:
            print(f"\n✓ 泊车成功!")
            print(f"  总步数: {step}")
            print(f"  最终距离: {dist_to_end:.2f}m")
            print(f"  最终位置: ({truck.x_m:.2f}, {truck.y_m:.2f})")
            print(f"  最终航向: {np.rad2deg(truck.h_rad):.1f}°")
            return True
        
        # 当前状态
        current_state = np.array([
            truck.x_m,
            truck.y_m,
            truck.h_rad,
            truck.h_rad - truck.g_rad
        ])
        
        # 参考状态（使用最近的点）
        distances = np.sqrt((path[:, 0] - truck.x_m)**2 + (path[:, 1] - truck.y_m)**2)
        nearest_idx = np.argmin(distances)
        state_ref = np.array([
            path[nearest_idx, 0],
            path[nearest_idx, 1],
            truck.h_rad,
            truck.h_rad - truck.g_rad
        ])
        
        # 计算控制量
        delta = controller.compute_control(current_state, state_ref)
        delta_deg = np.rad2deg(delta)
        
        # 应用控制
        truck.set_steering(delta_deg)
        truck.drive(DT)
        
        # 日志
        if step % 100 == 0:
            print(f"Step {step:4d}: pos=({truck.x_m:6.2f}, {truck.y_m:6.2f}), "
                  f"heading={np.rad2deg(truck.h_rad):6.1f}°, "
                  f"g={np.rad2deg(truck.g_rad):6.1f}°, "
                  f"delta={delta_deg:5.1f}°, "
                  f"dist={dist_to_end:5.2f}m")
    
    print(f"\n✗ 超时了! 最终距离: {dist_to_end:.2f}m")
    return False

if __name__ == "__main__":
    success = test_parking()
    sys.exit(0 if success else 1)
