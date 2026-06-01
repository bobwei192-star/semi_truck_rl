#!/usr/bin/env python3
"""
简化的半挂卡车模拟器 - Python版本
基于MATLAB代码的简化实现
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def truck_dynamics(state, t, params):
    """
    简化的卡车动力学模型
    state: [x, y, theta, v, gamma, phi, phi_dot]
    params: 包含所有参数的字典
    """
    # 解包状态
    x, y, theta, v, gamma, phi, phi_dot = state
    
    # 解包参数
    m_truck = params['m_truck']
    m_trailer = params['m_trailer']
    L1 = params['L1']  # 前轴到CG距离
    L2 = params['L2']  # 后轴到CG距离
    L3 = params['L3']  # 拖车CG到铰接点距离
    L4 = params['L4']  # 拖车后轴到铰接点距离
    thrust = params['thrust']
    wheel_torque = params['wheel_torque']
    drag_coeff = params['drag_coeff']
    friction_coeff = params['friction_coeff']
    
    # 简化的运动学关系
    # 卡车角速度
    if np.cos(phi) * (L1 + L2) != 0:
        omega_truck = (v * np.sin(phi)) / (np.cos(phi) * (L1 + L2))
    else:
        omega_truck = 0.0
    
    # 卡车横向速度
    if np.cos(phi) * (L1 + L2) != 0:
        v_lat = (L2 * v * np.sin(phi)) / (np.cos(phi) * (L1 + L2))
    else:
        v_lat = 0.0
    
    # 拖车角速度
    if L4 * np.cos(phi) * (L1 + L2) != 0:
        omega_trailer = -(v * (L4 * np.sin(phi) + 
                          (L1 + L2) * np.cos(phi) * np.sin(gamma))) / \
                        (L4 * np.cos(phi) * (L1 + L2))
    else:
        omega_trailer = 0.0
    
    # 简化的动力学
    total_mass = m_truck + m_trailer
    drag_force = 0.5 * drag_coeff * v**2
    friction_force = friction_coeff * total_mass * 9.81
    net_force = thrust - drag_force - friction_force
    v_dot = net_force / total_mass if total_mass > 0 else 0
    
    # 转向轮动力学 (简化)
    J_wheel = 1.0  # 车轮转动惯量
    damping = 10.0  # 阻尼系数
    wheel_acc = (wheel_torque - damping * phi_dot) / J_wheel
    
    # 状态导数
    dx = v * np.cos(theta) - v_lat * np.sin(theta)
    dy = v * np.sin(theta) + v_lat * np.cos(theta)
    dtheta = omega_truck
    dv = v_dot
    dgamma = omega_trailer
    dphi = phi_dot
    dphi_dot = wheel_acc
    
    return [dx, dy, dtheta, dv, dgamma, dphi, dphi_dot]

def run_simple_simulation():
    """运行简化仿真"""
    print("开始简化半挂卡车仿真...")
    
    # 设置参数
    params = {
        'm_truck': 1500.0,      # 卡车质量 (kg)
        'm_trailer': 3000.0,    # 拖车质量 (kg)
        'L1': 1.0,              # 前轴到CG距离 (m)
        'L2': 1.0,              # 后轴到CG距离 (m)
        'L3': 6.0,              # 拖车CG到铰接点距离 (m)
        'L4': 12.0,             # 拖车后轴到铰接点距离 (m)
        'thrust': 100.0,        # 推力 (N)
        'wheel_torque': 1.0,    # 转向扭矩 (N·m)
        'drag_coeff': 0.5,      # 阻力系数
        'friction_coeff': 0.002 # 摩擦系数
    }
    
    # 初始状态
    # [x, y, theta, v, gamma, phi, phi_dot]
    initial_state = [0.0, 0.0, np.radians(45), 0.0, 0.0, 0.0, 0.0]
    
    # 时间设置
    t_end = 30.0  # 仿真时间 (s)
    dt = 0.1      # 时间步长 (s)
    t = np.arange(0, t_end + dt, dt)
    
    # 运行仿真
    print("运行仿真...")
    solution = odeint(truck_dynamics, initial_state, t, args=(params,))
    
    print("仿真完成!")
    
    # 提取结果
    x = solution[:, 0]
    y = solution[:, 1]
    theta = solution[:, 2]
    v = solution[:, 3]
    gamma = solution[:, 4]
    phi = solution[:, 5]
    
    # 绘制结果
    plot_results(t, x, y, theta, v, gamma, phi)
    
    return solution

def plot_results(t, x, y, theta, v, gamma, phi):
    """绘制仿真结果"""
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    
    # 轨迹图
    axes[0, 0].plot(x, y, 'b-', linewidth=2)
    axes[0, 0].set_xlabel('x (m)')
    axes[0, 0].set_ylabel('y (m)')
    axes[0, 0].set_title('卡车轨迹')
    axes[0, 0].grid(True)
    axes[0, 0].set_aspect('equal')
    
    # 速度
    axes[0, 1].plot(t, v, 'r-', linewidth=2)
    axes[0, 1].set_xlabel('时间 (s)')
    axes[0, 1].set_ylabel('速度 (m/s)')
    axes[0, 1].set_title('卡车速度')
    axes[0, 1].grid(True)
    
    # 卡车角度
    axes[0, 2].plot(t, np.degrees(theta), 'g-', linewidth=2)
    axes[0, 2].set_xlabel('时间 (s)')
    axes[0, 2].set_ylabel('角度 (度)')
    axes[0, 2].set_title('卡车方向角')
    axes[0, 2].grid(True)
    
    # 车轮角度
    axes[1, 0].plot(t, np.degrees(phi), 'm-', linewidth=2)
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].set_ylabel('角度 (度)')
    axes[1, 0].set_title('车轮转向角')
    axes[1, 0].grid(True)
    
    # 拖车相对角度
    axes[1, 1].plot(t, np.degrees(gamma), 'c-', linewidth=2)
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].set_ylabel('角度 (度)')
    axes[1, 1].set_title('拖车相对角度')
    axes[1, 1].grid(True)
    
    # 位置随时间变化
    axes[1, 2].plot(t, x, 'b-', label='x', linewidth=2)
    axes[1, 2].plot(t, y, 'r-', label='y', linewidth=2)
    axes[1, 2].set_xlabel('时间 (s)')
    axes[1, 2].set_ylabel('位置 (m)')
    axes[1, 2].set_title('位置随时间变化')
    axes[1, 2].legend()
    axes[1, 2].grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # 创建动画
    create_simple_animation(x, y, theta, gamma, t)

def create_simple_animation(x, y, theta, gamma, t):
    """创建简化动画"""
    print("生成动画...")
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 卡车和拖车尺寸
    truck_length = 4.0
    truck_width = 2.0
    trailer_length = 12.0
    trailer_width = 2.5
    
    def animate(frame):
        ax.clear()
        
        # 当前状态
        current_x = x[frame]
        current_y = y[frame]
        current_theta = theta[frame]
        current_gamma = gamma[frame]
        
        # 卡车旋转矩阵
        truck_rot = np.array([
            [np.cos(current_theta), -np.sin(current_theta)],
            [np.sin(current_theta), np.cos(current_theta)]
        ])
        
        # 卡车角点
        truck_points = np.array([
            [truck_length/2, truck_width/2],
            [truck_length/2, -truck_width/2],
            [-truck_length/2, -truck_width/2],
            [-truck_length/2, truck_width/2],
            [truck_length/2, truck_width/2]
        ]).T
        
        # 旋转和平移卡车
        truck_points_rot = truck_rot @ truck_points
        truck_points_final = truck_points_rot + np.array([[current_x], [current_y]])
        
        # 绘制卡车
        ax.plot(truck_points_final[0, :], truck_points_final[1, :], 'b-', linewidth=2, label='卡车')
        
        # 拖车旋转矩阵
        trailer_theta = current_theta + current_gamma
        trailer_rot = np.array([
            [np.cos(trailer_theta), -np.sin(trailer_theta)],
            [np.sin(trailer_theta), np.cos(trailer_theta)]
        ])
        
        # 拖车铰接点位置 (卡车后部中心)
        hitch_x = current_x - (truck_length/2) * np.cos(current_theta)
        hitch_y = current_y - (truck_length/2) * np.sin(current_theta)
        
        # 拖车角点
        trailer_points = np.array([
            [0, trailer_width/2],
            [0, -trailer_width/2],
            [-trailer_length, -trailer_width/2],
            [-trailer_length, trailer_width/2],
            [0, trailer_width/2]
        ]).T
        
        # 旋转和平移拖车
        trailer_points_rot = trailer_rot @ trailer_points
        trailer_points_final = trailer_points_rot + np.array([[hitch_x], [hitch_y]])
        
        # 绘制拖车
        ax.plot(trailer_points_final[0, :], trailer_points_final[1, :], 'r-', linewidth=2, label='拖车')
        
        # 绘制轨迹
        ax.plot(x[:frame+1], y[:frame+1], 'g-', linewidth=1, alpha=0.7, label='轨迹')
        
        # 设置图形
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title(f'半挂卡车仿真 - 时间: {t[frame]:.1f}s')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        # 自动调整坐标轴
        margin = 10
        all_x = np.concatenate([truck_points_final[0, :], trailer_points_final[0, :]])
        all_y = np.concatenate([truck_points_final[1, :], trailer_points_final[1, :]])
        ax.set_xlim([np.min(all_x) - margin, np.max(all_x) + margin])
        ax.set_ylim([np.min(all_y) - margin, np.max(all_y) + margin])
    
    # 创建动画 (只显示前100帧以加快速度)
    n_frames = min(100, len(t))
    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, animate, frames=n_frames, interval=50, repeat=True)
    
    plt.tight_layout()
    plt.show()
    
    return anim

if __name__ == "__main__":
    print("简化半挂卡车模拟器")
    print("=" * 40)
    
    try:
        # 检查必要的库
        import numpy as np
        import matplotlib.pyplot as plt
        from scipy.integrate import odeint
        
        # 运行仿真
        results = run_simple_simulation()
        print("仿真成功完成!")
        
    except ImportError as e:
        print(f"缺少必要的库: {e}")
        print("请安装: pip install numpy matplotlib scipy")
    except Exception as e:
        print(f"仿真过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
