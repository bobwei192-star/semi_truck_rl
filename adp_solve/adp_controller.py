"""
ADP (Approximate Dynamic Programming) Controller for Truck-Trailer
基于 beep-beep-hiss 项目的 ADP 算法
"""
import numpy as np
from scipy.integrate import odeint
from scipy.linalg import solve_continuous_are as care
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from truckTrailerOnAxle import TruckTrailerOnAxle


class ADPController:
    """
    近似动态规划控制器
    """
    def __init__(self, truck_wheelbase=3.0, trailer_wheelbase=11.0,
                 Q=None, R=1000.0, noise_level=0.01, goal_x=None, goal_y=None):
        """
        Parameters:
        -----------
        truck_wheelbase : float
            车头轴距 (m)
        trailer_wheelbase : float
            车厢轴距 (m)
        Q : np.ndarray
            状态权重矩阵 (3x3)
        R : float
            控制权重
        noise_level : float
            探索噪声水平
        goal_x : float
            目标 X 坐标
        goal_y : float
            目标 Y 坐标
        """
        self.d1 = truck_wheelbase
        self.d2 = trailer_wheelbase
        self.noise_level = noise_level
        
        # 目标位置
        self.goal_x = goal_x if goal_x is not None else 57.5
        self.goal_y = goal_y if goal_y is not None else 53.62
        
        # 默认 Q 矩阵
        if Q is None:
            self.Q = np.eye(3)
        else:
            self.Q = Q
        self.R = R
        
        # 初始 LQR 增益
        self.K = self._compute_lqr_gain()
        
        # ADP 学习数据
        self.x_base_history = []
        self.q_history = []
        self.k_history = []
        self.s_history = []
        
        # 价值函数参数 (6维基函数)
        self.Ve = np.zeros((3, 3))
        
    def _compute_lqr_gain(self):
        """计算初始 LQR 反馈增益"""
        # 线性化系统矩阵
        A = np.array([
            [0, -1, 0],
            [0, 0, 0],
            [0, 0, 1/self.d2]
        ])
        B = np.array([[0], [-1/self.d1], [1/self.d1]])
        
        # 求解 CARE (Continuous Algebraic Riccati Equation)
        P = care(A, B, self.Q, np.array([[self.R]]))
        
        # 计算增益 K = R^-1 * B^T * P
        K = np.linalg.solve(np.array([[self.R]]), B.T @ P).flatten()
        
        # 限制增益大小，防止铰接角过大
        K = np.clip(K, -0.5, 0.5)
        
        return K
    
    def _cost_base(self, x):
        """基函数: [x1^2, 2*x1*x2, 2*x1*x3, x2^2, x2*x3, x3^2]"""
        x1, x2, x3 = x
        return np.array([
            x1*x1,
            2*x1*x2,
            2*x1*x3,
            x2*x2,
            x2*x3,
            x3*x3
        ])
    
    def _base_sigma(self, x1, x2, x3):
        """sigma 基函数"""
        return np.array([
            x1 * (x2 - np.sin(x2)),
            x2 * (x2 - np.sin(x2)),
            x3 * (x2 - np.sin(x2)),
            x1 * (x3 - np.sin(x3)),
            x2 * (x3 - np.sin(x3)),
            x3 * (x3 - np.sin(x3))
        ])
    
    def compute_control(self, state, t=0, velocity=-1, goal_y=None):
        """
        计算控制输入 - 支持弧线倒车
        
        Parameters:
        -----------
        state : np.ndarray
            [x, y, h, g] 状态向量
        velocity : float
            速度 (正=前进, 负=后退)
        goal_y : float
            目标 Y 坐标（可选，默认使用 self.goal_y）
            
        Returns:
        --------
        u : float
            控制输入 (转向角, 度)
        """
        x, y, h, g = state
        
        # 使用目标 Y 坐标
        target_y = goal_y if goal_y is not None else self.goal_y
        
        # 使用当前 ADP 增益
        if velocity <= 0:  # 倒车
            # 计算到目标的距离
            dist_y = target_y - y
            dist_to_goal = abs(dist_y)
            
            # 计算 x 方向偏移
            x_error = x - self.goal_x
            
            # ========== 弧线倒车策略 ==========
            # 阶段1: 远距离 (dist_y > 30m) - 优先调整 x 位置
            # 阶段2: 中距离 (10m < dist_y <= 30m) - 平衡 x 和 heading
            # 阶段3: 近距离 (dist_y <= 10m) - 精确对齐
            
            if dist_to_goal > 30:  # 阶段1: 远距离，保持稳定
                # 策略：保持直线倒车，小幅度                # 限制铰接角，防止超出边界
                if abs(g) > np.deg2rad(1):  # 铰接角超过 1°
                    # 反向修正，减小铰接角
                    u = np.sign(g) * 0.1
                else:
                    # 保持直线
                    u = 0.0
                    
            elif dist_to_goal > 10:  # 阶段2: 中距离，平衡调整
                # 限制铰接角
                if abs(g) > np.deg2rad(1):  # 铰接角超过 1°
                    # 反向修正，减小铰接角
                    u = np.sign(g) * 0.1
                else:
                    # 使用 ADP 增益
                    state_error = np.array([y - target_y, h + np.pi/2, g])
                    u = -self.K @ state_error * 0.1
                    
                    # 添加 x 修正
                    if abs(x_error) > 0.3:
                        x_correction = -x_error * 0.02
                        x_correction = np.clip(x_correction, -0.2, 0.2)
                        u += x_correction
                    
            else:  # 阶段3: 近距离，精确对齐
                # 使用非常保守的策略
                # 限制铰接角
                if abs(g) > np.deg2rad(0.5):  # 铰接角超过 0.5°
                    # 反向修正，减小铰接角
                    u = np.sign(g) * 0.05
                else:
                    # 只修正 heading
                    heading_error = -np.pi/2 - h  # 目标 heading = -90°
                    # 归一化到 [-pi, pi]
                    while heading_error > np.pi:
                        heading_error -= 2 * np.pi
                    while heading_error < -np.pi:
                        heading_error += 2 * np.pi
                    
                    # 小角度修正 heading
                    u = heading_error * 0.05
                    u = np.clip(u, -0.2, 0.2)
        else:  # 前进
            u = 0.0
        
        # 添加探索噪声
        noise = self.noise_level * np.random.randn()
        u += noise
        
        # 饱和限制 (35度)
        steering_limit = np.tan(np.deg2rad(35))
        u = np.clip(u, -steering_limit, steering_limit)
        
        # 转换为度
        u = np.degrees(u)
        
        return u
    
    def update_policy(self):
        """
        使用收集的数据更新策略 (ADP 学习)
        """
        if len(self.x_base_history) < 10:
            return
        
        # 转换为 numpy 数组
        x_base = np.array(self.x_base_history)
        q_base = np.array(self.q_history)
        k_base = np.array(self.k_history)
        s_base = np.array(self.s_history)
        
        # 计算差分
        dx = x_base[1:] - x_base[:-1]
        dq = q_base[1:] - q_base[:-1]
        dk = k_base[1:] - k_base[:-1]
        ds = s_base[1:] - s_base[:-1]
        
        # 最小二乘求解
        # l = [x_base, k_base, s_base] \ -q_base
        X = np.hstack([dx, dk, ds])
        l = np.linalg.lstsq(X, -dq, rcond=None)[0]
        
        # 提取价值函数参数
        self.Ve = np.array([
            [l[0], l[1]/2, l[2]/2],
            [l[1]/2, l[3], l[4]/2],
            [l[2]/2, l[4]/2, l[5]]
        ])
        
        # 更新增益
        Ke = (1.0 / self.R) * l[6:9] * 0.5
        self.K = Ke
        
        # 清空历史数据
        self.x_base_history = []
        self.q_history = []
        self.k_history = []
        self.s_history = []
    
    def collect_data(self, state, q, k, s):
        """收集 ADP 学习数据"""
        x, y, h, g = state
        
        # 计算基函数
        x_base = self._cost_base(np.array([y, h, g]))
        
        self.x_base_history.append(x_base)
        self.q_history.append(q)
        self.k_history.append(k)
        self.s_history.append(s)
    
    def reset(self):
        """重置控制器"""
        self.K = self._compute_lqr_gain()
        self.x_base_history = []
        self.q_history = []
        self.k_history = []
        self.s_history = []
        self.Ve = np.zeros((3, 3))


def run_adp_simulation(steps=1000, dt=0.05, learn_interval=30):
    """
    运行 ADP 仿真
    
    Parameters:
    -----------
    steps : int
        仿真步数
    dt : float
        时间步长
    learn_interval : int
        策略更新间隔
    """
    # 初始化车辆
    truck = TruckTrailerOnAxle()
    truck.set_state([57.5, 9.91, -np.pi/2, 0.0])
    truck.v_mps = -1.0
    
    # 初始化 ADP 控制器
    adp = ADPController(
        truck_wheelbase=truck.truckWheelbase,
        trailer_wheelbase=truck.trailerWheelbase
    )
    
    # 记录轨迹
    trajectory = []
    
    for step in range(steps):
        # 获取当前状态
        state = np.array([truck.x_m, truck.y_m, truck.h_rad, truck.g_rad])
        
        # 计算控制输入
        u = adp.compute_control(state, velocity=truck.v_mps)
        
        # 应用控制
        truck.set_steering(np.degrees(u))
        truck.drive(dt)
        
        # 记录轨迹
        trajectory.append([truck.x_m, truck.y_m, truck.h_rad, truck.g_rad, u])
        
        # 策略更新
        if step > 0 and step % learn_interval == 0:
            # 收集数据并更新策略
            adp.collect_data(state, 0.0, adp.K, np.zeros(6))
            adp.update_policy()
        
        # 检查是否到达目标
        if truck.y_m > 53.0:
            print(f"到达目标! step={step}")
            break
    
    return np.array(trajectory)


if __name__ == '__main__':
    print("ADP Controller Test")
    print("=" * 60)
    
    # 测试 ADP 控制器
    adp = ADPController()
    print(f"初始 LQR 增益 K: {adp.K}")
    
    # 测试控制计算
    state = np.array([57.5, 9.91, -np.pi/2, 0.0])
    u = adp.compute_control(state)
    print(f"控制输入 u: {u:.4f}")
    
    # 运行仿真
    print("\n运行 ADP 仿真...")
    trajectory = run_adp_simulation(steps=500)
    print(f"仿真完成，轨迹点数: {len(trajectory)}")
    if len(trajectory) > 0:
        print(f"最终位置: ({trajectory[-1][0]:.2f}, {trajectory[-1][1]:.2f})")
        print(f"最终朝向: {np.degrees(trajectory[-1][2]):.2f}°")
        print(f"最终铰接角: {np.degrees(trajectory[-1][3]):.2f}°")
