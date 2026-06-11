# -*- coding: utf-8 -*-
"""
路径规划模块
基于论文 j.cnki.xk.2023.2088 的垂直泊车路径规划方法:
1. 碰撞分析确定圆弧路径起始点可行域
2. B样条曲线生成平滑泊车路径
"""

import numpy as np
from typing import List, Tuple, Optional
from scipy.interpolate import BSpline, splprep, splev

from another_solve.config import VEHICLE, PARKING, VehicleParams, ParkingSpaceParams
from config.scene_config import START_X, START_Y, GOAL_X, GOAL_Y


class CollisionAnalyzer:
    """碰撞分析器 - 确定圆弧路径起始点可行域"""
    
    def __init__(self, vehicle: VehicleParams = None, parking: ParkingSpaceParams = None):
        self.vehicle = vehicle or VEHICLE
        self.parking = parking or PARKING
        self.R_min = self.parking.min_turning_radius  # 最小转弯半径
    
    def check_left_collision(self, x_M: float) -> bool:
        """
        车位左侧碰撞检测 (式5)
        
        Args:
            x_M: 挂车后轴中心在圆弧路径起点处的横坐标
        
        Returns:
            True if safe (no collision), False if collision risk
        """
        # x_M >= sqrt((R_min + W/2)^2 + (L1r)^2) - Pw + Delta
        W = self.vehicle.W
        L1r = self.vehicle.L1r
        Pw = self.parking.PW
        Delta = self.parking.safety_margin
        
        threshold = np.sqrt((self.R_min + W/2)**2 + L1r**2) - Pw + Delta
        return x_M >= threshold
    
    def check_right_collision(self, x_M: float, y_M: float) -> bool:
        """
        车位右侧碰撞检测 (式10)
        
        Args:
            x_M: 挂车后轴中心横坐标
            y_M: 挂车后轴中心纵坐标
        
        Returns:
            True if safe, False if collision risk
        """
        W = self.vehicle.W
        Delta = self.parking.safety_margin
        
        # x_M^2 + (R_min - y_M)^2 <= (R_min - W/2 - Delta)^2
        left_side = x_M**2 + (self.R_min - y_M)**2
        right_side = (self.R_min - W/2 - Delta)**2
        
        return left_side <= right_side
    
    def check_traffic_lane_collision(self, y_M: float) -> bool:
        """
        通车道上边缘碰撞检测 (式14)
        
        Args:
            y_M: 挂车后轴中心纵坐标
        
        Returns:
            True if safe, False if collision risk
        """
        W = self.vehicle.W
        L1 = self.vehicle.L1
        L0f = self.vehicle.L0f
        Rw = self.parking.RW
        Delta = self.parking.safety_margin
        
        # y_M <= Rw - Delta + R_min - sqrt((R_min + W/2)^2 + (L1 + L0f)^2)
        threshold = Rw - Delta + self.R_min - np.sqrt((self.R_min + W/2)**2 + (L1 + L0f)**2)
        return y_M <= threshold
    
    def get_feasible_region(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        计算圆弧路径起始点可行域
        
        Returns:
            Tuple of (x_range, y_range, valid_points)
        """
        # 根据式5、10、14确定可行域边界
        W = self.vehicle.W
        L1r = self.vehicle.L1r
        
        # x最小值 (式5)
        x_min = np.sqrt((self.R_min + W/2)**2 + L1r**2) - self.parking.PW + self.parking.safety_margin
        
        # x最大值 (式10边界)
        x_max = self.R_min - W/2 - self.parking.safety_margin
        
        # y最大值 (式14)
        L1 = self.vehicle.L1
        L0f = self.vehicle.L0f
        y_max = self.parking.RW - self.parking.safety_margin + self.R_min - \
                np.sqrt((self.R_min + W/2)**2 + (L1 + L0f)**2)
        
        # y最小值 (需要满足式10)
        y_min = 0  # 假设最低从通车道下边缘开始
        
        # 生成可行区域网格
        x_range = np.linspace(x_min, x_max, 50)
        y_range = np.linspace(y_min, y_max, 50)
        
        return x_range, y_range, (x_min, x_max, y_min, y_max)
    
    def select_arc_start_point(self, x: float = None, y: float = None) -> Tuple[float, float]:
        """
        选择圆弧路径起始点
        
        Args:
            x: 指定横坐标 (可选)
            y: 指定纵坐标 (可选)
        
        Returns:
            圆弧路径起始点坐标 (x_s, y_s)
        """
        x_min, x_max, y_min, y_max = self.get_feasible_region()[2]
        
        # 默认选择可行域中间范围
        if x is None:
            x = (x_min + x_max) / 2
        if y is None:
            y = (y_min + y_max) / 2
        
        # 确保在可行域内
        x = np.clip(x, x_min, x_max)
        y = np.clip(y, y_min, y_max)
        
        return x, y


class PathPlanner:
    """B样条路径规划器"""
    
    def __init__(self, vehicle: VehicleParams = None, parking: ParkingSpaceParams = None):
        self.vehicle = vehicle or VEHICLE
        self.parking = parking or PARKING
        self.collision_analyzer = CollisionAnalyzer(vehicle, parking)
        self.R_min = self.parking.min_turning_radius
    
    def generate_arc_control_points(self, arc_start: Tuple[float, float],
                                    arc_end: Tuple[float, float],
                                    turning_center: Tuple[float, float],
                                    angle_interval_deg: float = 15.0) -> List[Tuple[float, float]]:
        """
        生成圆弧路径控制点 (式15, 16)
        
        Args:
            arc_start: 圆弧起点坐标
            arc_end: 圆弧终点坐标
            turning_center: 转向中心坐标
            angle_interval_deg: 控制点间隔角度
        
        Returns:
            圆弧路径控制点列表
        """
        x0, y0 = turning_center
        
        # 计算起点和终点角度
        start_angle = np.arctan2(arc_start[1] - y0, arc_start[0] - x0)
        end_angle = np.arctan2(arc_end[1] - y0, arc_end[0] - x0)
        
        # 确保角度正确 (倒车时顺时针)
        if end_angle > start_angle:
            end_angle -= 2 * np.pi
        
        # 生成控制点
        angles = np.linspace(start_angle, end_angle, int(abs(end_angle - start_angle) / np.deg2rad(angle_interval_deg)) + 1)
        control_points = []
        
        for theta in angles:
            # 式15, 16: x = x0 - R_min * sin(theta), y = y0 + R_min * cos(theta)
            x = x0 - self.R_min * np.sin(theta)
            y = y0 + self.R_min * np.cos(theta)
            control_points.append((x, y))
        
        return control_points
    
    def generate_parking_path(self, 
                              start_point: Tuple[float, float] = None,
                              arc_start: Tuple[float, float] = None,
                              straight_transition_length: float = 2.0) -> dict:
        """
        生成完整泊车路径 - 简化版本，适合纯追踪算法
        路径在项目坐标系（米）下生成
        
        Args:
            start_point: 泊车起点（项目坐标系）
            arc_start: 圆弧路径起点（项目坐标系）
            straight_transition_length: 直线过渡段长度
        
        Returns:
            dict: {
                'control_points': 控制点列表,
                'path': 路径坐标数组,
                'arc_start': 圆弧起点,
                'arc_end': 圆弧终点,
                'turning_center': 转向中心,
                'end_point': 终点
            }
        """
        # 1. 设置起点和圆弧起点（项目坐标系）
        # 使用传入的参数，如果没有传入则使用默认值
        if arc_start is None:
            arc_start = (GOAL_X, START_Y + (GOAL_Y - START_Y) * 0.5)
        if start_point is None:
            start_point = (START_X, START_Y)
        
        # 2. 定义简单的直线入库路径（先测试直线）
        # 库位中心（使用传入的终点）
        end_point = (GOAL_X, GOAL_Y)
        
        path_points = []
        
        # 简单直线：从起点到终点
        num_points = 200
        for i in range(num_points + 1):
            t = i / num_points
            x = start_point[0] + t * (end_point[0] - start_point[0])
            y = start_point[1] + t * (end_point[1] - start_point[1])
            path_points.append((x, y))
        
        path = np.array(path_points)
        
        arc_start = (GOAL_X, START_Y + (GOAL_Y - START_Y) * 0.5)
        arc_end = (GOAL_X, START_Y + (GOAL_Y - START_Y) * 0.2)
        turning_center = (GOAL_X, START_Y + (GOAL_Y - START_Y) * 0.35)
        
        return {
            'control_points': [start_point, arc_start, arc_end, end_point],
            'path': path,
            'arc_start': arc_start,
            'arc_end': arc_end,
            'turning_center': turning_center,
            'end_point': end_point,
            'start_point': start_point
        }
    
    def fit_bspline(self, control_points: List[Tuple[float, float]], 
                    degree: int = 3, num_points: int = 200) -> np.ndarray:
        """
        使用B样条曲线拟合控制点
        
        Args:
            control_points: 控制点列表
            degree: B样条阶数
            num_points: 生成的路径点数量
        
        Returns:
            路径坐标数组 (N x 2)
        """
        if len(control_points) < 4:
            # 控制点太少,直接返回
            return np.array(control_points)
        
        points = np.array(control_points)
        
        try:
            # 使用scipy的splprep生成B样条
            tck, u = splprep([points[:, 0], points[:, 1]], s=0, k=min(degree, len(points)-1))
            
            # 生成均匀分布的路径点
            u_new = np.linspace(0, 1, num_points)
            x_new, y_new = splev(u_new, tck)
            
            path = np.column_stack([x_new, y_new])
        except Exception as e:
            # 如果B样条拟合失败,使用线性插值
            print(f"B样条拟合失败: {e}, 使用线性插值")
            path = self._linear_interpolate(points, num_points)
        
        return path
    
    def _linear_interpolate(self, points: np.ndarray, num_points: int) -> np.ndarray:
        """线性插值"""
        # 计算累积距离
        distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
        cumulative = np.concatenate([[0], np.cumsum(distances)])
        
        # 均匀采样
        sample_distances = np.linspace(0, cumulative[-1], num_points)
        
        # 插值
        x_interp = np.interp(sample_distances, cumulative, points[:, 0])
        y_interp = np.interp(sample_distances, cumulative, points[:, 1])
        
        return np.column_stack([x_interp, y_interp])
    
    def get_reference_info(self, path: np.ndarray, current_pos: Tuple[float, float],
                           lookahead_distance: float = 1.0) -> dict:
        """
        获取参考信息 (论文2.5节)
        
        Args:
            path: 参考路径
            current_pos: 当前位置 (x, y)
            lookahead_distance: 前瞻距离
        
        Returns:
            dict: 参考状态和控制信息
        """
        # 找到距离当前位置最近的路径点
        distances = np.sqrt(np.sum((path - current_pos)**2, axis=1))
        nearest_idx = np.argmin(distances)
        
        # 获取参考位置
        x_r, y_r = path[nearest_idx]
        
        # 计算参考航向角 (式45)
        if nearest_idx > 0:
            dx = path[nearest_idx, 0] - path[nearest_idx - 1, 0]
            dy = path[nearest_idx, 1] - path[nearest_idx - 1, 1]
            phi1_ref = np.arctan2(dy, dx)
        else:
            phi1_ref = np.arctan2(path[1, 1] - path[0, 1], path[1, 0] - path[0, 0])
        
        # 计算铰接点参考位置 (式20)
        # x_O = x_C + L1 * cos(phi1)
        # y_O = y_C + L1 * sin(phi1)
        x_O = x_r + self.vehicle.L1 * np.cos(phi1_ref)
        y_O = y_r + self.vehicle.L1 * np.sin(phi1_ref)
        
        # 计算牵引车参考航向角 (式46)
        if nearest_idx > 0:
            # 使用前一时刻的铰接点位置
            x_O_prev = path[nearest_idx - 1, 0] + self.vehicle.L1 * np.cos(phi1_ref)
            y_O_prev = path[nearest_idx - 1, 1] + self.vehicle.L1 * np.sin(phi1_ref)
            phi0_ref = np.arctan2(y_O - y_O_prev, x_O - x_O_prev)
        else:
            phi0_ref = phi1_ref  # 假设初始铰接角为0
        
        # 计算参考前轮转角 (式28简化)
        # 需要根据航向角变化率计算
        # 这里简化为基于航向角差估计
        phi_diff = phi0_ref - phi1_ref
        delta_ref = np.arctan(self.vehicle.Lb / self.vehicle.L1 * np.sin(phi_diff))
        
        # 添加前瞻: 向前看几个点获取更好的参考
        lookahead_idx = min(nearest_idx + 5, len(path) - 1)
        x_ref = path[lookahead_idx, 0]
        y_ref = path[lookahead_idx, 1]
        
        return {
            'x_ref': x_ref,
            'y_ref': y_ref,
            'phi0_ref': phi0_ref,
            'phi1_ref': phi1_ref,
            'delta_ref': delta_ref,
            'nearest_idx': nearest_idx,
            'distance_to_path': distances[nearest_idx]
        }


def visualize_feasible_region():
    """可视化可行域"""
    analyzer = CollisionAnalyzer()
    x_range, y_range, bounds = analyzer.get_feasible_region()
    
    print(f"可行域范围:")
    print(f"  X: [{bounds[0]:.2f}, {bounds[1]:.2f}] m")
    print(f"  Y: [{bounds[2]:.2f}, {bounds[3]:.2f}] m")
    print(f"  最小转弯半径: {analyzer.R_min:.2f} m")
    
    return bounds


if __name__ == "__main__":
    # 测试路径规划
    planner = PathPlanner()
    
    # 生成泊车路径
    result = planner.generate_parking_path()
    
    print("泊车路径规划结果:")
    print(f"  起始点: {result['start_point']}")
    print(f"  圆弧起点: {result['arc_start']}")
    print(f"  圆弧终点: {result['arc_end']}")
    print(f"  转向中心: {result['turning_center']}")
    print(f"  终点: {result['end_point']}")
    print(f"  路径点数: {len(result['path'])}")
    print(f"  控制点数: {len(result['control_points'])}")
    
    # 可视化可行域
    visualize_feasible_region()