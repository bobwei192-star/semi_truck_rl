# -*- coding: utf-8 -*-
"""
MPC 控制器
基于论文 j.cnki.xk.2023.2088 第2.2-2.5节

使用二次规划 (QP) 求解最优控制序列
"""

import numpy as np
from typing import Tuple, List, Optional
from scipy.optimize import minimize, LinearConstraint

from another_solve.config import VEHICLE, PARKING, MPC as MPC_CONFIG
from another_solve.kinematics import TruckTrailerKinematics


class MPCController:
    """模型预测控制器"""
    
    def __init__(self, vehicle=None, parking=None, mpc_params=None):
        self.vehicle = vehicle or VEHICLE
        self.parking = parking or PARKING
        self.mpc = mpc_params or MPC_CONFIG
        
        # 运动学模型
        self.kinematics = TruckTrailerKinematics(self.vehicle)
        
        # MPC参数
        self.Np = self.mpc.Np  # 预测时域
        self.Nc = self.mpc.Nc  # 控制时域
        self.Ts = self.mpc.Ts  # 采样时间
        
        # 约束
        self.delta_max = np.deg2rad(self.vehicle.max_steering_deg)
        self.delta_rate_max = np.deg2rad(self.vehicle.max_steering_rate_deg) * self.Ts
        
        # 当前控制量 (用于增量计算)
        self.current_delta = 0.0
        
        # 权重矩阵
        self.Q = self.mpc.Q
        self.R = self.mpc.R
    
    def compute_prediction_matrices(self, state_ref: np.ndarray, delta_ref: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        计算预测模型矩阵 (式37-39)
        
        Args:
            state_ref: 参考状态 [x_C_ref, y_C_ref, phi_0_ref, phi_1_ref]
            delta_ref: 参考前轮转角
        
        Returns:
            A_bar, B_bar: 扩展状态空间矩阵
        """
        # 计算线性化矩阵
        A, B = self.kinematics.compute_jacobian(state_ref, delta_ref, self.Ts)
        
        # 扩展状态空间 (式37)
        # A_bar = [A, B; 0, I]
        n = A.shape[0]
        m = B.shape[1]
        
        A_bar = np.zeros((n + m, n + m))
        A_bar[:n, :n] = A
        A_bar[:n, n:] = B
        A_bar[n:, n:] = np.eye(m)
        
        # B_bar = [B; I]
        B_bar = np.zeros((n + m, m))
        B_bar[:n, :] = B
        B_bar[n:, :] = np.eye(m)
        
        return A_bar, B_bar
    
    def build_qp_matrices(self, state_ref: np.ndarray, delta_ref: float,
                          lambda_t: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        构建QP问题的矩阵
        
        Args:
            state_ref: 参考状态
            delta_ref: 参考前轮转角
            lambda_t: 当前扩展状态 [x_tilde, delta_prev]
        
        Returns:
            H, g, A_ineq, b_ineq: QP问题标准形式
        """
        # 计算预测矩阵
        A_bar, B_bar = self.compute_prediction_matrices(state_ref, delta_ref)
        
        n = A_bar.shape[0]
        m = B_bar.shape[1]
        
        # 构建预测输出矩阵 (式39)
        # Y = A_t * lambda(t|t) + B_t * Delta_U
        
        # 构建A_t (状态转移矩阵的幂)
        A_t = np.zeros((self.Np * n, n))
        for i in range(self.Np):
            A_t[i*n:(i+1)*n, :] = np.linalg.matrix_power(A_bar, i+1)
        
        # 构建B_t (控制输入矩阵)
        B_t = np.zeros((self.Np * n, self.Nc * m))
        for i in range(self.Np):
            for j in range(min(i+1, self.Nc)):
                power = i - j
                B_t[i*n:(i+1)*n, j*m:(j+1)*m] = np.linalg.matrix_power(A_bar, power) @ B_bar
        
        # 构建输出矩阵 C
        C = np.hstack([np.eye(4), np.zeros((4, 1))])
        
        # 扩展C
        C_expanded = np.zeros((self.Np * 4, self.Np * n))
        for i in range(self.Np):
            C_expanded[i*4:(i+1)*4, i*n:(i+1)*n] = C
        
        # 目标函数: J = ||Y - Y_ref||_Q^2 + ||Delta_U||_R^2
        # 简化为标准QP形式: 1/2 * Delta_U^T * H * Delta_U + g^T * Delta_U
        
        # 计算参考输出
        Y_ref = np.zeros(self.Np * 4)
        
        # 构建Hessian矩阵
        Q_expanded = np.kron(np.eye(self.Np), self.Q)
        R_expanded = np.kron(np.eye(self.Nc), self.R)
        
        # H = 2 * (B_t^T * Q * B_t + R)
        H = 2 * (B_t.T @ Q_expanded @ B_t + R_expanded)
        
        # g = 2 * B_t^T * Q * (A_t * lambda_t - Y_ref)
        g = 2 * B_t.T @ Q_expanded @ (A_t @ lambda_t - Y_ref)
        
        # 约束条件 (式44)
        # Delta_U_min <= Delta_U <= Delta_U_max
        # U_min <= I * Delta_U + U_t <= U_max
        
        # 构建约束矩阵
        A_ineq = np.vstack([np.eye(self.Nc * m), -np.eye(self.Nc * m)])
        
        # 控制增量约束
        delta_min_vec = np.ones(self.Nc * m) * (-self.delta_rate_max)
        delta_max_vec = np.ones(self.Nc * m) * self.delta_rate_max
        
        b_ineq = np.concatenate([delta_max_vec, -delta_min_vec])
        
        return H, g, A_ineq, b_ineq
    
    def solve(self, state: np.ndarray, state_ref: np.ndarray, delta_ref: float) -> float:
        """
        求解MPC优化问题
        
        Args:
            state: 当前实际状态 [x_C, y_C, phi_0, phi_1]
            state_ref: 参考状态
            delta_ref: 参考前轮转角
        
        Returns:
            最优前轮转角增量
        """
        # 构建扩展状态
        lambda_t = np.zeros(5)
        lambda_t[:4] = state - state_ref
        lambda_t[4] = self.current_delta
        
        # 构建QP矩阵
        H, g, A_ineq, b_ineq = self.build_qp_matrices(state_ref, delta_ref, lambda_t)
        
        # 使用scipy求解QP
        # 目标函数: 1/2 * x^T * H * x + g^T * x
        def objective(x):
            return 0.5 * x.T @ H @ x + g.T @ x
        
        def gradient(x):
            return H @ x + g
        
        # 初始猜测
        x0 = np.zeros(self.Nc)
        
        # 约束
        constraints = []
        if A_ineq.shape[0] > 0:
            # 线性不等式约束: A_ineq @ x <= b_ineq
            for i in range(A_ineq.shape[0]):
                constraints.append({
                    'type': 'ineq',
                    'fun': lambda x, i=i: b_ineq[i] - A_ineq[i] @ x
                })
        
        # 求解
        try:
            result = minimize(
                objective,
                x0,
                method='SLSQP',
                jac=gradient,
                constraints=constraints,
                options={'maxiter': 100, 'ftol': 1e-6}
            )
            
            if result.success:
                # 返回第一个控制增量
                delta_optimal = result.x[0]
            else:
                # 如果优化失败,使用零控制
                delta_optimal = 0.0
        except Exception as e:
            print(f"MPC求解失败: {e}")
            delta_optimal = 0.0
        
        return delta_optimal
    
    def update(self, state: np.ndarray, reference_trajectory: np.ndarray,
               reference_delta: np.ndarray, current_idx: int) -> float:
        """
        更新MPC控制器
        
        Args:
            state: 当前实际状态
            reference_trajectory: 参考轨迹
            reference_delta: 参考前轮转角序列
            current_idx: 当前参考点索引
        
        Returns:
            最优前轮转角
        """
        # 获取参考状态和控制量
        if current_idx < len(reference_trajectory):
            state_ref = reference_trajectory[current_idx]
        else:
            state_ref = reference_trajectory[-1]
        
        if current_idx < len(reference_delta):
            delta_ref = reference_delta[current_idx]
        else:
            delta_ref = reference_delta[-1]
        
        # 求解MPC
        delta_optimal = self.solve(state, state_ref, delta_ref)
        
        # 更新当前控制量
        self.current_delta += delta_optimal
        
        # 限制前轮转角
        self.current_delta = np.clip(self.current_delta, -self.delta_max, self.delta_max)
        
        return self.current_delta
    
    def reset(self):
        """重置控制器"""
        self.current_delta = 0.0


class SimpleMPCController:
    """简化版MPC控制器 (使用纯追踪算法)"""
    
    def __init__(self, vehicle=None, parking=None, mpc_params=None):
        self.vehicle = vehicle or VEHICLE
        self.parking = parking or PARKING
        self.mpc = mpc_params or MPC_CONFIG
        
        self.kinematics = TruckTrailerKinematics(self.vehicle)
        
        # 纯追踪参数
        self.lookahead_distance = 10.0  # 前瞻距离 (m) - 增加以提高稳定性
        
        # 约束
        self.delta_max = np.deg2rad(self.vehicle.max_steering_deg)
        
        # 参考路径缓存
        self.reference_path = None
    
    def set_reference_path(self, path):
        """设置参考路径"""
        self.reference_path = path
    
    def compute_control(self, state: np.ndarray, state_ref: np.ndarray) -> float:
        """
        计算控制量 (使用纯追踪算法)
        
        Args:
            state: 当前状态 [x_C, y_C, phi_0, phi_1]
            state_ref: 参考状态 [x_ref, y_ref, phi0_ref, phi1_ref]
        
        Returns:
            前轮转角 (rad)
        """
        if self.reference_path is None:
            # 如果没有参考路径，使用简化控制律
            error = state_ref - state
            delta = 2.0 * error[1] + 1.0 * (state_ref[2] - state[2])
            return np.clip(delta, -self.delta_max, self.delta_max)
        
        x_c, y_c, phi0, phi1 = state
        
        # 找到前瞻点
        lookahead_point = self._find_lookahead_point(x_c, y_c)
        
        if lookahead_point is None:
            # 使用参考状态进行反馈控制
            error = state_ref - state
            delta = 2.0 * error[1] + 1.0 * (state_ref[2] - state[2])
            return np.clip(delta, -self.delta_max, self.delta_max)
        
        x_l, y_l = lookahead_point
        
        # 纯追踪算法
        # 计算前瞻点相对于车辆坐标系的位置
        dx = x_l - x_c
        dy = y_l - y_c
        
        # 转换到车辆坐标系
        x_veh = dx * np.cos(phi0) + dy * np.sin(phi0)
        y_veh = -dx * np.sin(phi0) + dy * np.cos(phi0)
        
        # 纯追踪转向角计算（倒车模式）
        L = self.lookahead_distance
        Lb = self.vehicle.Lb
        
        # 防止除以零
        if L < 0.1:
            delta = 0.0
        else:
            # 对于倒车的牵引车-挂车系统，调整转向方向
            # 前瞻点在车辆坐标系中的横向偏差
            # 倒车时：向左打方向盘，车辆会向右移动
            # 使用比例控制来跟踪路径
            kp = 0.3  # 增加比例增益
            delta = -kp * y_veh  # 倒车时反转方向
        
        # 限制范围
        delta = np.clip(delta, -self.delta_max, self.delta_max)
        
        return delta
    
    def _find_lookahead_point(self, x, y):
        """找到前瞻点"""
        if self.reference_path is None:
            return None
        
        path = np.array(self.reference_path)
        
        # 计算到每个路径点的距离
        distances = np.sqrt((path[:, 0] - x)**2 + (path[:, 1] - y)**2)
        
        # 找到最近的路径点
        nearest_idx = np.argmin(distances)
        
        # 从最近的点开始往后查找
        for i in range(nearest_idx, len(path)):
            dx = path[i, 0] - path[nearest_idx, 0]
            dy = path[i, 1] - path[nearest_idx, 1]
            cum_dist = np.sqrt(dx**2 + dy**2)
            
            # 从最近点到第i点的累积距离
            if cum_dist >= self.lookahead_distance:
                # 线性插值找到精确的前瞻点
                if i == nearest_idx:
                    return (path[i, 0], path[i, 1])
                
                # 在i-1和i之间插值
                dx_prev = path[i-1, 0] - path[nearest_idx, 0]
                dy_prev = path[i-1, 1] - path[nearest_idx, 1]
                cum_dist_prev = np.sqrt(dx_prev**2 + dy_prev**2)
                
                t = (self.lookahead_distance - cum_dist_prev) / (cum_dist - cum_dist_prev + 1e-6)
                x_interp = path[i-1, 0] + t * (path[i, 0] - path[i-1, 0])
                y_interp = path[i-1, 1] + t * (path[i, 1] - path[i-1, 1])
                
                return (x_interp, y_interp)
        
        # 如果所有点都在前瞻距离内，返回最后一个点
        return (path[-1, 0], path[-1, 1])


if __name__ == "__main__":
    # 测试MPC控制器
    mpc = MPCController()
    
    # 初始状态
    state = np.array([0.0, 0.0, 0.0, 0.0])
    state_ref = np.array([1.0, 1.0, 0.0, 0.0])
    delta_ref = 0.0
    
    # 求解
    delta = mpc.solve(state, state_ref, delta_ref)
    print(f"最优前轮转角增量: {np.rad2deg(delta):.4f} deg")
    print(f"当前前轮转角: {np.rad2deg(mpc.current_delta):.4f} deg")
