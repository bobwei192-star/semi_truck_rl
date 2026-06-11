# -*- coding: utf-8 -*-
"""
车辆运动学模型
基于论文 j.cnki.xk.2023.2088 第2.1节

以挂车后轴中心为研究对象建立运动学模型
状态量: X = [x_C, y_C, phi_0, phi_1]^T
控制量: u = delta (前轮转角)
"""

import numpy as np
from typing import Tuple

from another_solve.config import VEHICLE, PARKING, MPC


class TruckTrailerKinematics:
    """牵引挂车运动学模型"""
    
    def __init__(self, vehicle=None):
        self.vehicle = vehicle or VEHICLE
        self.v = self.vehicle.parking_speed_ms  # 恒定倒车速度 (m/s)
        
        # 参数
        self.L0 = self.vehicle.L0   # 牵引车轴距
        self.L1 = self.vehicle.L1   # 铰接点到挂车后轴中心距离
        self.Lb = self.vehicle.Lb   # 铰接点到牵引车后轴中心距离
    
    def state_derivative(self, state: np.ndarray, delta: float) -> np.ndarray:
        """
        计算状态导数 (式19, 25, 31)
        
        Args:
            state: [x_C, y_C, phi_0, phi_1]^T
            delta: 前轮转角 (rad)
        
        Returns:
            d_state: [dx_C, dy_C, dphi_0, dphi_1]^T
        """
        x_C, y_C, phi_0, phi_1 = state
        
        # 式19: 挂车后轴中心速度
        dx_C = self.v * np.cos(phi_1)
        dy_C = self.v * np.sin(phi_1)
        
        # 式31: 挂车航向角变化率
        # dphi_1 = v/L1 * (Lb*tan(delta)/(L0*cos(phi_0-phi_1)) + tan(phi_0-phi_1))
        phi_diff = phi_0 - phi_1
        
        # 避免除以零
        cos_phi_diff = np.cos(phi_diff)
        if abs(cos_phi_diff) < 1e-6:
            cos_phi_diff = np.sign(cos_phi_diff) * 1e-6
        
        dphi_1 = self.v / self.L1 * (
            self.Lb * np.tan(delta) / (self.L0 * cos_phi_diff) + np.tan(phi_diff)
        )
        
        # 式25: 牵引车航向角变化率
        # dphi_0 = (dphi_1 * L1 * cos(phi_0-phi_1) - v * sin(phi_0-phi_1)) / Lb
        dphi_0 = (dphi_1 * self.L1 * np.cos(phi_diff) - self.v * np.sin(phi_diff)) / self.Lb
        
        return np.array([dx_C, dy_C, dphi_0, dphi_1])
    
    def step(self, state: np.ndarray, delta: float, dt: float) -> np.ndarray:
        """
        单步积分 (欧拉法)
        
        Args:
            state: 当前状态
            delta: 前轮转角
            dt: 时间步长
        
        Returns:
            新状态
        """
        d_state = self.state_derivative(state, delta)
        new_state = state + d_state * dt
        
        # 角度归一化到 [-pi, pi]
        new_state[2] = self._normalize_angle(new_state[2])
        new_state[3] = self._normalize_angle(new_state[3])
        
        # 限制铰接角
        max_g = np.deg2rad(self.vehicle.max_articulation_deg)
        new_state[2] = np.clip(new_state[2], new_state[3] - max_g, new_state[3] + max_g)
        
        return new_state
    
    def _normalize_angle(self, angle: float) -> float:
        """归一化角度到 [-pi, pi]"""
        return ((angle + np.pi) % (2 * np.pi)) - np.pi
    
    def compute_jacobian(self, state_ref: np.ndarray, delta_ref: float, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算线性化雅可比矩阵 (式34, 35)
        
        Args:
            state_ref: 参考状态 [x_C_ref, y_C_ref, phi_0_ref, phi_1_ref]
            delta_ref: 参考前轮转角
            dt: 采样时间
        
        Returns:
            A: 状态转移矩阵 (4x4)
            B: 控制矩阵 (4x1)
        """
        x_C_ref, y_C_ref, phi_0_ref, phi_1_ref = state_ref
        
        # 使用参考值计算
        phi_diff = phi_0_ref - phi_1_ref
        
        # 避免除以零
        cos_phi_diff = np.cos(phi_diff)
        if abs(cos_phi_diff) < 1e-6:
            cos_phi_diff = np.sign(cos_phi_diff) * 1e-6
        
        cos_delta_ref = np.cos(delta_ref)
        if abs(cos_delta_ref) < 1e-6:
            cos_delta_ref = np.sign(cos_delta_ref) * 1e-6
        
        # 式34: 状态转移矩阵
        A = np.zeros((4, 4))
        A[0, 3] = -self.v * np.sin(phi_1_ref) * dt
        A[1, 3] = self.v * np.cos(phi_1_ref) * dt
        
        # P = v / (L1 * cos^2(phi_0_ref - phi_1_ref))
        P = self.v / (self.L1 * cos_phi_diff**2)
        
        # Q = tan(delta_ref) * Lb * sin(phi_0_ref - phi_1_ref) / L0 + 1
        Q = np.tan(delta_ref) * self.Lb * np.sin(phi_diff) / self.L0 + 1
        
        A[3, 2] = P * Q * dt
        A[3, 3] = 1 - P * Q * dt
        
        # 式35: 控制矩阵
        B = np.zeros((4, 1))
        B[2, 0] = self.v / (self.L0 * cos_delta_ref**2) * dt
        B[3, 0] = self.v / self.L1 * (
            self.Lb / (self.L0 * cos_phi_diff * cos_delta_ref**2)
        ) * dt
        
        # 加上单位矩阵部分 (离散化)
        A[0, 0] = 1
        A[1, 1] = 1
        A[2, 2] = 1
        
        return A, B
    
    def compute_articulation_angle(self, phi_0: float, phi_1: float) -> float:
        """
        计算铰接角 (g = phi_0 - phi_1)
        
        Args:
            phi_0: 牵引车航向角
            phi_1: 挂车航向角
        
        Returns:
            铰接角 (rad)
        """
        return phi_0 - phi_1
    
    def compute_hitch_point(self, state: np.ndarray) -> Tuple[float, float]:
        """
        计算铰接点位置 (式20)
        
        Args:
            state: [x_C, y_C, phi_0, phi_1]
        
        Returns:
            (x_O, y_O)
        """
        x_C, y_C, phi_0, phi_1 = state
        x_O = x_C + self.L1 * np.cos(phi_1)
        y_O = y_C + self.L1 * np.sin(phi_1)
        return x_O, y_O
    
    def compute_truck_rear_axle(self, state: np.ndarray) -> Tuple[float, float]:
        """
        计算牵引车后轴中心位置 (式21)
        
        Args:
            state: [x_C, y_C, phi_0, phi_1]
        
        Returns:
            (x_B, y_B)
        """
        x_C, y_C, phi_0, phi_1 = state
        x_B = x_C + self.L1 * np.cos(phi_1) - self.Lb * np.cos(phi_0)
        y_B = y_C + self.L1 * np.sin(phi_1) - self.Lb * np.sin(phi_0)
        return x_B, y_B
    
    def compute_truck_front_axle(self, state: np.ndarray) -> Tuple[float, float]:
        """
        计算牵引车前轴中心位置
        
        Args:
            state: [x_C, y_C, phi_0, phi_1]
        
        Returns:
            (x_A, y_A)
        """
        x_B, y_B = self.compute_truck_rear_axle(state)
        x_A = x_B + self.L0 * np.cos(state[2])
        y_A = y_B + self.L0 * np.sin(state[2])
        return x_A, y_A
    
    def get_vehicle_corners(self, state: np.ndarray) -> dict:
        """
        获取车辆四个角的坐标 (用于碰撞检测)
        
        Args:
            state: [x_C, y_C, phi_0, phi_1]
        
        Returns:
            dict: 各角点坐标
        """
        x_C, y_C, phi_0, phi_1 = state
        W = self.vehicle.W
        L1r = self.vehicle.L1r
        
        # 挂车矩形框
        # 后轴中心
        x_M, y_M = x_C, y_C
        
        # 挂车方向向量
        cos_phi1 = np.cos(phi_1)
        sin_phi1 = np.sin(phi_1)
        
        # 挂车宽度方向
        cos_perp = -sin_phi1
        sin_perp = cos_phi1
        
        # 挂车四个角 (E, F, G, H)
        # H: 左后角
        x_H = x_M + L1r * cos_phi1 + W/2 * cos_perp
        y_H = y_M + L1r * sin_phi1 + W/2 * sin_perp
        
        # G: 左前角 (铰接点)
        x_G = x_M + self.L1 * cos_phi1 + W/2 * cos_perp
        y_G = y_M + self.L1 * sin_phi1 + W/2 * sin_perp
        
        # F: 右前角
        x_F = x_M + self.L1 * cos_phi1 - W/2 * cos_perp
        y_F = y_M + self.L1 * sin_phi1 - W/2 * sin_perp
        
        # E: 右后角
        x_E = x_M + L1r * cos_phi1 - W/2 * cos_perp
        y_E = y_M + L1r * sin_phi1 - W/2 * sin_perp
        
        # 牵引车矩形框 (A, B, C, D)
        x_B, y_B = self.compute_truck_rear_axle(state)
        x_O, y_O = self.compute_hitch_point(state)
        
        cos_phi0 = np.cos(phi_0)
        sin_phi0 = np.sin(phi_0)
        cos_perp0 = -sin_phi0
        sin_perp0 = cos_phi0
        
        # A: 牵引车前轴中心
        x_A = x_B + self.L0 * cos_phi0
        y_A = y_B + self.L0 * sin_phi0
        
        # 牵引车四个角
        x_D = x_A + W/2 * cos_perp0
        y_D = y_A + W/2 * sin_perp0
        
        x_C_truck = x_A - W/2 * cos_perp0
        y_C_truck = y_A - W/2 * sin_perp0
        
        x_B_left = x_B + W/2 * cos_perp0
        y_B_left = y_B + W/2 * sin_perp0
        
        x_B_right = x_B - W/2 * cos_perp0
        y_B_right = y_B - W/2 * sin_perp0
        
        return {
            'trailer': {
                'E': (x_E, y_E),
                'F': (x_F, y_F),
                'G': (x_G, y_G),
                'H': (x_H, y_H)
            },
            'truck': {
                'A': (x_A, y_A),
                'B_left': (x_B_left, y_B_left),
                'B_right': (x_B_right, y_B_right),
                'C': (x_C_truck, y_C_truck),
                'D': (x_D, y_D)
            },
            'hitch': (x_O, y_O),
            'truck_rear': (x_B, y_B)
        }


if __name__ == "__main__":
    # 测试运动学模型
    kinematics = TruckTrailerKinematics()
    
    # 初始状态: 挂车后轴中心在原点, 两车都朝右 (phi=0)
    state = np.array([0.0, 0.0, 0.0, 0.0])
    delta = np.deg2rad(5.0)  # 前轮转角5度
    dt = 0.1
    
    print("初始状态:", state)
    print(f"前轮转角: {np.rad2deg(delta):.2f} deg")
    
    # 模拟10步
    for i in range(10):
        state = kinematics.step(state, delta, dt)
        print(f"Step {i+1}: x={state[0]:.3f}, y={state[1]:.3f}, "
              f"phi_0={np.rad2deg(state[2]):.2f} deg, phi_1={np.rad2deg(state[3]):.2f} deg")
    
    # 计算铰接点位置
    x_O, y_O = kinematics.compute_hitch_point(state)
    print(f"\n铰接点位置: ({x_O:.3f}, {y_O:.3f})")
    
    # 计算雅可比矩阵
    A, B = kinematics.compute_jacobian(state, delta, dt)
    print(f"\n状态转移矩阵 A:\n{A}")
    print(f"控制矩阵 B:\n{B}")
