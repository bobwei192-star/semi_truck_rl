#!/usr/bin/env python3
"""
稳定的半挂卡车模拟器 - Python版本
简化且数值稳定的实现
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def truck_dynamics_simple(state, t):
    """
    简化的卡车动力学模型 - 数值稳定版本
    state: [x, y, theta, v, gamma, phi]
    """
    # 解包状态
    x, y, theta, v, gamma, phi = state
    
    # 固定参数
    m_truck = 1500.0      # 卡车质量 (kg)
    m_trailer = 3000.0    # 拖车质量 (kg)
    L1 = 1.0              # 前轴到CG距离 (m)
    L2 = 1.0              # 后轴到CG距离 (m)
    L3 = 6.0              # 拖车CG到铰接点距离 (m)
    L4 = 12.0             # 拖车后轴到铰接点距离 (m)
    thrust = 100.0        # 推力 (N)
    wheel_torque = 1.0    # 转向扭矩 (N·m)
    drag_coeff = 0.5      # 阻力系数
    friction_coeff = 0.002 # 摩擦系数
    
    # 简化的运动学关系 - 添加数值稳定性检查
    if abs(np.cos(phi)) < 1e-6:
        omega_truck = 0.0
        v_lat = 0.0
    else:
        omega_truck = (v * np.sin(phi)) / (np.cos(phi) * (L1 + L2))
        v_lat = (L2 * v * np.sin(phi)) / (np.cos(phi) * (L1 + L2))
    
    # 拖车角速度
    if abs(L4 * np.cos(phi) * (L1 + L2)) < 1e-6:
        omega_trailer = 0.0
    else:
        omega_trailer = -(v * (L4 * np.sin(phi) + 
                          (L1 + L2) * np.cos(phi) * np.sin(gamma))) / \
                        (L4 * np.cos(phi) * (L1 + L2))
    
    # 简化的动力学 - 限制数值范围
    total_mass = m_truck + m_trailer
    drag_force = 0.5 * drag_coeff * min(v**2, 1000)  # 限制阻力计算
    friction_force = friction_coeff * total_mass * 9.81
    net_force = thrust - drag_force - friction_force
    v_dot = net_force / total_mass if total_mass > 0 else 0
    
    # 转向轮动力学 (简化且稳定)
    J_wheel = 1.0    # 车轮转动惯量
    damping = 10.0   # 阻尼系数
    phi_dot = (wheel_torque - damping * phi) / J_wheel
    
    # 限制角度变化率
    phi_dot = np.clip(phi_dot, -0.5, 0.5)
    
    # 状态导数
    dx = v * np.cos(theta) - v_lat * np.sin(theta)
    dy = v * np.sin(theta) + v_lat * np.cos(theta)
    dtheta = omega_truck
    dv = v_dot
    dgamma = omega_trailer
    dphi = phi_dot
    
    return [dx, dy, dtheta, dv, dgamma, dphi]

def run_stable_simulation():
    """运行稳定仿真"""
    print("开始稳定半挂卡车仿真...")
    
    # 初始状态
    # [x, y, theta, v, gamma, phi]
    initial_state = [0.0, 0.0, np.radians(45), 0.0, 0.0, 0.0]
    
    # 时间设置
    t_end = 30.0  # 仿真时间 (s)
    dt = 0.1      # 时间步长 (s)
    t = np.arange(0, t_end + dt, dt)
    
    # 运行仿真
    print("运行仿真...")
    solution = odeint(truck_dynamics_simple, initial_state, t)
    
    print("仿真完成!")
    
    # 提取结果
    x = solution[:, 0]
    y = solution[:, 1]
    theta = solution[:, 2]
    v = solution[:, 3]
    gamma = solution[:, 4]
    phi = solution[:, 5]
    
    # 绘制结果
    plot_stable_results(t, x, y, theta, v, gamma, phi)
    
    return solution

def plot_stable_results(t, x, y, theta, v, gamma, phi):
    """绘制稳定仿真结果"""
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
    plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号
    
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
    
    try:
        plt.tight_layout()
        plt.show()
    except Exception as e:
        print(f"图表显示警告: {e}")
        plt.show()
    
    # 创建简单动画
    create_stable_animation(x, y, theta, gamma, t)

def create_stable_animation(x, y, theta, gamma, t):
    """创建稳定动画"""
    print("生成动画...")
    
    # 卡车和拖车尺寸
    truck_length = 4.0
    truck_width = 2.0
    trailer_length = 12.0
    trailer_width = 2.5
    
    # 只显示前50帧以加快速度
    n_frames = min(50, len(t))
    
    for frame in range(0, n_frames, 5):  # 每5帧显示一次
        plt.figure(figsize=(10, 8))
        
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
        plt.plot(truck_points_final[0, :], truck_points_final[1, :], 'b-', linewidth=2, label='卡车')
        
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
        plt.plot(trailer_points_final[0, :], trailer_points_final[1, :], 'r-', linewidth=2, label='拖车')
        
        # 绘制轨迹
        plt.plot(x[:frame+1], y[:frame+1], 'g-', linewidth=1, alpha=0.7, label='轨迹')
        
        # 设置图形
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title(f'半挂卡车仿真 - 时间: {t[frame]:.1f}s')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # 自动调整坐标轴
        margin = 10
        all_x = np.concatenate([truck_points_final[0, :], trailer_points_final[0, :]])
        all_y = np.concatenate([truck_points_final[1, :], trailer_points_final[1, :]])
        plt.xlim([np.min(all_x) - margin, np.max(all_x) + margin])
        plt.ylim([np.min(all_y) - margin, np.max(all_y) + margin])
        
        plt.tight_layout()
        plt.show()
        
        if frame < n_frames - 5:  # 不是最后一帧时暂停
            input("按回车键继续下一帧...")

def main():
    """主函数"""
    print("稳定半挂卡车模拟器")
    print("=" * 40)
    
    try:
        # 检查必要的库
        import numpy as np
        import matplotlib.pyplot as plt
        from scipy.integrate import odeint
        
        # 运行仿真
        results = run_stable_simulation()
        print("仿真成功完成!")
        
    except ImportError as e:
        print(f"缺少必要的库: {e}")
        print("请安装: pip install numpy matplotlib scipy")
    except Exception as e:
        print(f"仿真过程中出现错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
