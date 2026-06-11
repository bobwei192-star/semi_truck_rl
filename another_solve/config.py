# -*- coding: utf-8 -*-
"""
车辆和车位参数配置
基于论文 j.cnki.xk.2023.2088 的参数设置
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class VehicleParams:
    """牵引挂车参数 (基于 TruckSim Cab-Over + European Trailer)"""
    
    # 牵引车参数
    L0: float = 4.135       # 牵引车轴距 (m)
    La: float = 3.8         # 铰接点到牵引车前轴中心距离 (m)
    Lb: float = 0.335      # 铰接点到牵引车后轴中心距离 (m)
    L0f: float = 5.3       # 铰接点到牵引车前端距离 (m)
    
    # 挂车参数
    L1: float = 7.9        # 铰接点到挂车后轴中心距离 (m)
    L1r: float = 4.2      # 挂车后悬 (m)
    
    # 车宽
    W: float = 2.438       # 车宽 (m)
    
    # 控制约束
    max_steering_deg: float = 35.0    # 最大前轮转角 (度)
    max_steering_rate_deg: float = 60.0  # 前轮转角变化率 (度/秒)
    max_articulation_deg: float = 90.0   # 最大铰接角 (度)
    
    # 泊车速度
    parking_speed_ms: float = -2.0 / 3.6  # 泊车速度 (m/s), -2 km/h
    
    @property
    def max_steering_rad(self) -> float:
        return np.deg2rad(self.max_steering_deg)
    
    @property
    def max_articulation_rad(self) -> float:
        return np.deg2rad(self.max_articulation_deg)


@dataclass  
class ParkingSpaceParams:
    """车位参数 (基于 JGJ100-98 规范)"""
    
    PL: float = 19.0      # 车位长度 (m)
    PW: float = 4.5       # 车位宽度 (m)
    RW: float = 16.0      # 通车道宽度 (m)
    
    # 安全距离
    safety_margin: float = 0.5   # 预留安全距离 (m)
    
    # 圆弧路径参数
    max_virtual_steering_deg: float = 45.0  # 挂车最大虚拟转向角 (度)
    
    @property
    def min_turning_radius(self) -> float:
        """挂车最小转弯半径 (根据式1计算)"""
        # R_min = L1 / tan(alpha)
        vehicle = VehicleParams()
        alpha = np.deg2rad(self.max_virtual_steering_deg)
        return vehicle.L1 / np.tan(alpha)


@dataclass
class MPCParams:
    """MPC控制器参数"""
    
    # 预测时域和控制时域
    Np: int = 20          # 预测时域
    Nc: int = 10          # 控制时域
    
    # 采样时间
    Ts: float = 0.1       # 采样时间 (s)
    
    # 权重矩阵
    Q_x: float = 10.0     # x位置权重
    Q_y: float = 10.0     # y位置权重
    Q_phi0: float = 5.0   # 牵引车航向角权重
    Q_phi1: float = 5.0   # 挂车航向角权重
    
    R_delta: float = 100.0  # 前轮转角增量权重
    
    # 松弛因子权重
    rho: float = 1000.0
    
    @property
    def Q(self) -> np.ndarray:
        """状态权重矩阵"""
        return np.diag([self.Q_x, self.Q_y, self.Q_phi0, self.Q_phi1])
    
    @property
    def R(self) -> np.ndarray:
        """控制权重矩阵"""
        return np.array([[self.R_delta]])


# 默认参数实例
VEHICLE = VehicleParams()
PARKING = ParkingSpaceParams()
MPC = MPCParams()
