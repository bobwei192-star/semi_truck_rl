#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
全局停车约束验证模块

所有算法调试时必须服从以下约束：

1. 车辆运动过程中，车身所有部分（包括8个角）不得碰撞：
   - 1001线（左车道左边界，X=50.0m）
   - 1002线（右车道右边界/车库左边界，X=60.0m）

2. 车辆停下后，第五轮（铰接点）需与GOAL_X, GOAL_Y重合（容差0.5m）

3. 车辆停下后，车身所有部分需在车库边界内：
   - 1002线（车库左边界，X=60.0m）
   - 1003线（车库右边界，X=56.0m）
   - 1014线（车库上边界，Y=50.5m）
   - 1015线（车库下边界，Y=69.0m）

车辆8个角编号：
    牵引车头（编号1-4）:
    1 - 车头右前角
    2 - 车头右后角
    3 - 车头左后角
    4 - 车头左前角
    
    挂车（编号5-8）:
    5 - 挂车右前角（靠近车头）
    6 - 挂车右后角
    7 - 挂车左后角
    8 - 挂车左前角（靠近车头）
"""

import numpy as np
from config.scene_config import (
    GOAL_X, GOAL_Y, GOAL_POS_TOL_M,
    ROAD_CENTER_X, LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH,
    PARKING_TOP_Y, PARKING_BTM_Y
)

# 车道边界
LANE_LEFT_BOUND_X = ROAD_CENTER_X - LEFT_LANE_WIDTH   # 1001线: X=50.0m
LANE_RIGHT_BOUND_X = ROAD_CENTER_X + RIGHT_LANE_WIDTH  # 1002线: X=60.0m

# 车库边界
GARAGE_LEFT_BOUND_X = LANE_RIGHT_BOUND_X    # 1002线: X=60.0m
GARAGE_RIGHT_BOUND_X = ROAD_CENTER_X         # 1003线: X=56.0m
GARAGE_TOP_BOUND_Y = PARKING_TOP_Y           # 1014线: Y=50.5m
GARAGE_BOTTOM_BOUND_Y = PARKING_BTM_Y        # 1015线: Y=69.0m


class ParkingConstraints:
    """停车约束检查器 - 全局约束验证"""
    
    def __init__(self):
        self.trajectory_violations = []  # 轨迹违规记录
        
    def get_corner_positions(self, truck):
        """获取车辆8个角的位置（世界坐标，单位：米）
        
        返回格式: [(x1,y1), (x2,y2), ..., (x8,y8)]
        """
        x, y, h, g = truck.x_m, truck.y_m, truck.h_rad, truck.g_rad
        
        # 车头尺寸
        t_len = truck.truckWheelbase + 2 * truck.truckMargin
        t_wid = truck.truckWidth
        
        # 车头4个角的相对位置（相对于车头中心点）
        t_corners_rel = np.array([
            [t_len/2, -t_wid/2],   # 1: 右前
            [-t_len/2, -t_wid/2],  # 2: 右后
            [-t_len/2, t_wid/2],   # 3: 左后
            [t_len/2, t_wid/2]     # 4: 左前
        ])
        
        # 旋转并转换到世界坐标
        cos_h, sin_h = np.cos(h), np.sin(h)
        rot_matrix = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        truck_corners = np.dot(t_corners_rel, rot_matrix.T) + np.array([x, y])
        
        # 挂车尺寸
        tr_len = truck.trailerWheelbase + truck.trailerMarginFront + truck.trailerMarginBack
        tr_wid = truck.trailerWidth
        
        # 挂车中心点
        tr_center_rel_x = -t_len/2 + truck.truckMargin + (truck.trailerMarginFront - tr_len/2) * np.cos(g)
        tr_center_rel_y = (truck.trailerMarginFront - tr_len/2) * np.sin(g)
        tr_center_x = x + tr_center_rel_x * np.cos(h) - tr_center_rel_y * np.sin(h)
        tr_center_y = y + tr_center_rel_x * np.sin(h) + tr_center_rel_y * np.cos(h)
        
        # 挂车4个角的相对位置
        tr_corners_rel = np.array([
            [tr_len/2, -tr_wid/2],   # 5: 右前（靠近车头）
            [-tr_len/2, -tr_wid/2],  # 6: 右后
            [-tr_len/2, tr_wid/2],   # 7: 左后
            [tr_len/2, tr_wid/2]     # 8: 左前（靠近车头）
        ])
        
        # 旋转并转换到世界坐标
        tr_h = h + g
        cos_trh, sin_trh = np.cos(tr_h), np.sin(tr_h)
        rot_matrix_tr = np.array([[cos_trh, -sin_trh], [sin_trh, cos_trh]])
        trailer_corners = np.dot(tr_corners_rel, rot_matrix_tr.T) + np.array([tr_center_x, tr_center_y])
        
        return np.vstack([truck_corners, trailer_corners])
    
    def get_fifth_wheel_position(self, truck):
        """获取第五轮（铰接点）位置"""
        x = truck.x_m - truck.truckWheelbase * np.cos(truck.h_rad)
        y = truck.y_m - truck.truckWheelbase * np.sin(truck.h_rad)
        return x, y
    
    def check_lane_boundary_violation(self, truck, step=None):
        """检查车辆8个角是否越过1001或1002线
        
        Args:
            truck: 车辆对象
            step: 当前步数（用于记录轨迹违规）
        
        Returns:
            violations: 违规列表
        """
        violations = []
        corners = self.get_corner_positions(truck)
        
        corner_names = [
            "角1(车头右前)", "角2(车头右后)", "角3(车头左后)", "角4(车头左前)",
            "角5(挂车右前)", "角6(挂车右后)", "角7(挂车左后)", "角8(挂车左前)"
        ]
        
        for i, (cx, cy) in enumerate(corners):
            # 检查1001线（左边界）
            if cx < LANE_LEFT_BOUND_X:
                v = {
                    'line': '1001',
                    'corner': i + 1,
                    'corner_name': corner_names[i],
                    'position': (cx, cy),
                    'bound': LANE_LEFT_BOUND_X,
                    'violation': LANE_LEFT_BOUND_X - cx,
                    'step': step
                }
                violations.append(v)
                if step is not None:
                    self.trajectory_violations.append(v)
            
            # 检查1002线（右边界）
            if cx > LANE_RIGHT_BOUND_X:
                v = {
                    'line': '1002',
                    'corner': i + 1,
                    'corner_name': corner_names[i],
                    'position': (cx, cy),
                    'bound': LANE_RIGHT_BOUND_X,
                    'violation': cx - LANE_RIGHT_BOUND_X,
                    'step': step
                }
                violations.append(v)
                if step is not None:
                    self.trajectory_violations.append(v)
        
        return violations
    
    def check_goal_position(self, truck):
        """检查第五轮（铰接点）是否到达目标位置"""
        fifth_wheel_x, fifth_wheel_y = self.get_fifth_wheel_position(truck)
        
        dist_to_goal = np.sqrt((fifth_wheel_x - GOAL_X)**2 + (fifth_wheel_y - GOAL_Y)**2)
        
        return {
            'fifth_wheel_x': fifth_wheel_x,
            'fifth_wheel_y': fifth_wheel_y,
            'goal_x': GOAL_X,
            'goal_y': GOAL_Y,
            'distance': dist_to_goal,
            'within_tolerance': dist_to_goal <= GOAL_POS_TOL_M,
            'tolerance': GOAL_POS_TOL_M
        }
    
    def check_garage_boundary(self, truck):
        """检查车身8个角是否在车库边界内"""
        violations = []
        corners = self.get_corner_positions(truck)
        
        corner_names = [
            "角1(车头右前)", "角2(车头右后)", "角3(车头左后)", "角4(车头左前)",
            "角5(挂车右前)", "角6(挂车右后)", "角7(挂车左后)", "角8(挂车左前)"
        ]
        
        for i, (cx, cy) in enumerate(corners):
            # 检查X方向（车库左右边界）
            if cx > GARAGE_LEFT_BOUND_X:  # 超过左边界1002线
                violations.append({
                    'type': 'garage_left_bound',
                    'line': '1002',
                    'corner': i + 1,
                    'corner_name': corner_names[i],
                    'position': (cx, cy),
                    'violation': cx - GARAGE_LEFT_BOUND_X
                })
            
            if cx < GARAGE_RIGHT_BOUND_X:  # 超过右边界1003线
                violations.append({
                    'type': 'garage_right_bound',
                    'line': '1003',
                    'corner': i + 1,
                    'corner_name': corner_names[i],
                    'position': (cx, cy),
                    'violation': GARAGE_RIGHT_BOUND_X - cx
                })
            
            # 检查Y方向（车库上下边界）
            if cy < GARAGE_TOP_BOUND_Y:  # 超过上边界1014线
                violations.append({
                    'type': 'garage_top_bound',
                    'line': '1014',
                    'corner': i + 1,
                    'corner_name': corner_names[i],
                    'position': (cx, cy),
                    'violation': GARAGE_TOP_BOUND_Y - cy
                })
            
            if cy > GARAGE_BOTTOM_BOUND_Y:  # 超过下边界1015线
                violations.append({
                    'type': 'garage_bottom_bound',
                    'line': '1015',
                    'corner': i + 1,
                    'corner_name': corner_names[i],
                    'position': (cx, cy),
                    'violation': cy - GARAGE_BOTTOM_BOUND_Y
                })
        
        return violations
    
    def run_all_checks(self, truck, step=None):
        """运行所有约束检查"""
        results = {
            'lane_boundary': self.check_lane_boundary_violation(truck, step),
            'goal_position': self.check_goal_position(truck),
            'garage_boundary': self.check_garage_boundary(truck)
        }
        return results
    
    def print_violations(self, results):
        """打印约束违反情况"""
        print("\n" + "="*60)
        print("全局停车约束检查报告")
        print("="*60)
        
        # 车道边界检查
        print("\n1. 车道边界检查（1001/1002线）- 运动过程中8个角是否越界")
        if results['lane_boundary']:
            for v in results['lane_boundary']:
                print(f"   ⚠️  违反 {v['line']}线: {v['corner_name']} 越界 {v['violation']:.3f}m")
        else:
            print("   ✅ 所有8个角未越过车道边界")
        
        # 目标位置检查
        print("\n2. 第五轮位置检查")
        g = results['goal_position']
        print(f"   第五轮位置: ({g['fifth_wheel_x']:.3f}, {g['fifth_wheel_y']:.3f})")
        print(f"   目标位置: ({g['goal_x']:.3f}, {g['goal_y']:.3f})")
        print(f"   距离: {g['distance']:.3f}m (容差: {g['tolerance']}m)")
        if g['within_tolerance']:
            print("   ✅ 第五轮与目标重合（在容差范围内）")
        else:
            print("   ⚠️  第五轮未与目标重合")
        
        # 车库边界检查
        print("\n3. 车库边界检查（1003/1002/1014/1015线）- 8个角是否在车库内")
        print(f"   车库范围: X[{GARAGE_RIGHT_BOUND_X:.2f}, {GARAGE_LEFT_BOUND_X:.2f}], Y[{GARAGE_TOP_BOUND_Y:.2f}, {GARAGE_BOTTOM_BOUND_Y:.2f}]")
        if results['garage_boundary']:
            for v in results['garage_boundary']:
                print(f"   ⚠️  违反 {v['line']}线: {v['corner_name']} 越界 {v['violation']:.3f}m")
        else:
            print("   ✅ 所有8个角都在车库内")
        
        print("\n" + "="*60)
        
        # 总体结果
        has_violations = len(results['lane_boundary']) > 0 or \
                        not results['goal_position']['within_tolerance'] or \
                        len(results['garage_boundary']) > 0
        
        return not has_violations
    
    def get_trajectory_summary(self):
        """获取轨迹违规汇总"""
        return self.trajectory_violations
    
    def reset_trajectory_tracking(self):
        """重置轨迹追踪"""
        self.trajectory_violations = []


# 全局约束实例
constraints = ParkingConstraints()


def check_constraints(truck, step=None):
    """便捷函数：检查所有约束"""
    return constraints.run_all_checks(truck, step)


def verify_parking_success(truck):
    """验证停车是否成功满足所有约束"""
    results = constraints.run_all_checks(truck)
    success = constraints.print_violations(results)
    return success


def get_corner_positions(truck):
    """便捷函数：获取8个角位置"""
    return constraints.get_corner_positions(truck)


def get_fifth_wheel_position(truck):
    """便捷函数：获取第五轮位置"""
    return constraints.get_fifth_wheel_position(truck)
