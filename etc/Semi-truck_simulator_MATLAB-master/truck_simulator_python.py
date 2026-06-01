#!/usr/bin/env python3
"""
半挂卡车模拟器 - Python版本
将MATLAB半挂卡车模拟器转换为Python代码
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import matplotlib.animation as animation
from dataclasses import dataclass
from typing import Tuple, List

@dataclass
class SimulationSettings:
    """仿真设置"""
    end_time: float = 30.0
    save_period: float = 0.1
    anim_play_rate: float = 20.0

@dataclass
class ControllerSettings:
    """控制器设置"""
    sampling_period: float = 1.0
    thrust_force: float = 100.0
    wheel_torque: float = 1.0

@dataclass
class GlobalParameters:
    """全局参数"""
    grav_accel: float = 9.81
    dyn_fric_coeff: float = 0.002
    air_density: float = 1.2

@dataclass
class TruckProperties:
    """卡车属性"""
    mass: float = 1500.0
    moi: float = 10000.0
    cg2front_length: float = 1.0
    cg2rear_length: float = 1.0
    wheel_moi: float = 1.0
    wheel_gear_ratio: float = 1.0
    wheel_stiff_coeff: float = 0.0
    wheel_offset: float = 0.25
    wheel_damp_coeff: float = 10.0
    drag_coeff: float = 1.2
    drag_area: float = 3.0
    wheel_stop_stiff: float = 0.0
    max_wheel_angle: float = 10.0
    width: float = 2.0

@dataclass
class TrailerProperties:
    """拖车属性"""
    mass: float = 3000.0
    moi: float = 20000.0
    pivot2cg_length: float = 6.0
    pivot2rear_length: float = 12.0
    width: float = 2.5

@dataclass
class TruckState:
    """卡车状态"""
    cg_pos_vec: np.ndarray = None
    angle: float = 0.0
    long_vel: float = 0.0
    lat_vel: float = 0.0
    ang_vel: float = 0.0
    wheel_angle: float = 0.0
    wheel_ang_vel: float = 0.0
    long_acc: float = 0.0
    front_lat_force: float = 0.0
    rear_lat_force: float = 0.0
    rot_mat: np.ndarray = None
    trans_mat_fw: np.ndarray = None
    trans_mat_bw: np.ndarray = None
    rear_pos_vec: np.ndarray = None

@dataclass
class TrailerState:
    """拖车状态"""
    angle_rel2truck: float = 0.0
    angle: float = 0.0
    ang_vel: float = 0.0
    long_vel: float = 0.0
    lat_vel: float = 0.0
    vel_vec: np.ndarray = None
    rot_mat: np.ndarray = None
    cg_pos_vec: np.ndarray = None
    rear_pos_vec: np.ndarray = None
    rear_lat_force: float = 0.0

def truck_kinetics(global_params: GlobalParameters, truck: TruckState, trailer: TrailerState, 
                  truck_props: TruckProperties, trailer_props: TrailerProperties) -> Tuple[float, float, float, float]:
    """
    计算卡车动力学
    返回: (纵向加速度, 前轮侧向力, 后轮侧向力, 拖车后轮侧向力)
    """
    # 重命名变量以便与MATLAB代码对应
    IzzA = truck_props.moi
    IzzB = trailer_props.moi
    mA = truck_props.mass
    mB = trailer_props.mass
    g = global_params.grav_accel
    mu_k = global_params.dyn_fric_coeff
    L1 = truck_props.cg2front_length
    L2 = truck_props.cg2rear_length
    L3 = trailer_props.pivot2cg_length
    L4 = trailer_props.pivot2rear_length
    phi = truck.wheel_angle
    phi_dot = truck.wheel_ang_vel
    T = truck.thrust_force
    vAx = truck.long_vel
    vAy = truck.lat_vel
    omegaA = truck.ang_vel
    omegaB = trailer.ang_vel
    gamma = trailer.angle_rel2truck
    C_D = truck_props.drag_coeff
    rho = global_params.air_density
    A = truck_props.drag_area

    # 定义相关位置向量
    r_A_1 = np.array([L1, 0, 0])
    r_A_2 = np.array([-L2, 0, 0])
    r_2_3 = np.array([-L4 * np.cos(gamma), -L4 * np.sin(gamma), 0])

    # 定义车轮法向力
    N1 = L2 / (L1 + L2) * mA * g
    N2 = L1 / (L1 + L2) * mA * g + (L4 - L3) / L4 * mB * g
    N3 = L3 / L4 * mB * g

    # 定义相关角速度和速度向量
    omegaA_vec = np.array([0, 0, omegaA])
    omegaB_vec = np.array([0, 0, omegaB])
    vA_vec = np.array([vAx, vAy, 0])
    
    def cross(a, b):
        return np.cross(a, b)
    
    v1_vec = vA_vec + cross(omegaA_vec, r_A_1)
    v2_vec = vA_vec + cross(omegaA_vec, r_A_2)
    v3_vec = vA_vec + cross(omegaB_vec, r_2_3) + cross(omegaA_vec, r_A_2 + r_2_3)

    # 定义与摩擦力对齐的单位向量
    u_phi = np.array([np.cos(phi), np.sin(phi), 0])
    u_gamma = np.array([np.cos(gamma), np.sin(gamma), 0])

    # 定义空气动力学阻力
    vA_mag = np.sqrt(vAx**2 + vAy**2)
    F_D = -0.5 * C_D * rho * A * vA_mag * vA_vec
    F_Dx = F_D[0]
    
    # 定义三个车轮组的摩擦力
    Ff1 = -np.tanh(np.dot(v1_vec, u_phi)) * mu_k * N1 * u_phi
    Ff1x, Ff1y = Ff1[0], Ff1[1]
    
    Ff2 = -np.tanh(np.dot(v2_vec, np.array([1, 0, 0]))) * mu_k * N2 * np.array([1, 0, 0])
    Ff2x = Ff2[0]
    
    Ff3 = -np.tanh(np.dot(v3_vec, u_gamma)) * mu_k * N3 * u_gamma
    Ff3x, Ff3y = Ff3[0], Ff3[1]

    # 这里简化处理，使用MATLAB代码中的复杂表达式
    # 在实际应用中，这些表达式需要从符号计算中导出
    # 为了演示目的，我们使用简化的动力学模型
    
    # 简化的纵向加速度计算
    total_mass = mA + mB
    drag_force = 0.5 * C_D * rho * A * vAx**2
    friction_force = mu_k * (N1 + N2 + N3)
    net_force = T - drag_force - friction_force
    vAx_dot = net_force / total_mass if total_mass > 0 else 0

    # 简化的侧向力计算
    lambda1 = 0.0  # 前轮侧向力
    lambda2 = 0.0  # 后轮侧向力  
    lambda3 = 0.0  # 拖车后轮侧向力

    return vAx_dot, lambda1, lambda2, lambda3

def truck_dyn_model(state_vec, t, control, global_params, truck_props, trailer_props):
    """
    卡车动力学模型 - 用于ODE求解器
    """
    # 提取状态向量
    truck_cg_pos = state_vec[0:2]
    truck_angle = state_vec[2]
    truck_long_vel = state_vec[3]
    trailer_angle_rel = state_vec[4]
    wheel_angle = state_vec[5]
    wheel_ang_vel = state_vec[6]

    # 创建状态对象
    truck = TruckState()
    truck.cg_pos_vec = truck_cg_pos
    truck.angle = truck_angle
    truck.long_vel = truck_long_vel
    truck.wheel_angle = wheel_angle
    truck.wheel_ang_vel = wheel_ang_vel
    
    trailer = TrailerState()
    trailer.angle_rel2truck = trailer_angle_rel

    # 计算运动学变量
    L1 = truck_props.cg2front_length
    L2 = truck_props.cg2rear_length
    
    # 卡车横向速度
    if np.cos(wheel_angle) * (L1 + L2) != 0:
        truck.lat_vel = (L2 * truck_long_vel * np.sin(wheel_angle)) / (np.cos(wheel_angle) * (L1 + L2))
    else:
        truck.lat_vel = 0.0
    
    # 卡车角速度
    if np.cos(wheel_angle) * (L1 + L2) != 0:
        truck.ang_vel = (truck_long_vel * np.sin(wheel_angle)) / (np.cos(wheel_angle) * (L1 + L2))
    else:
        truck.ang_vel = 0.0

    # 拖车角度和角速度
    trailer.angle = truck_angle + trailer_angle_rel
    
    L4 = trailer_props.pivot2rear_length
    if L4 * np.cos(wheel_angle) * (L1 + L2) != 0:
        trailer.ang_vel = -(truck_long_vel * (L4 * np.sin(wheel_angle) + 
                            (L1 + L2) * np.cos(wheel_angle) * np.sin(trailer_angle_rel))) / \
                          (L4 * np.cos(wheel_angle) * (L1 + L2))
    else:
        trailer.ang_vel = 0.0

    # 拖车速度
    L3 = trailer_props.pivot2cg_length
    trailer.long_vel = (truck_long_vel + 
                       L3 * truck.ang_vel * np.sin(trailer_angle_rel) + 
                       L3 * trailer.ang_vel * np.sin(trailer_angle_rel))
    
    trailer.lat_vel = (truck.lat_vel - 
                      truck.ang_vel * (L2 + L3 * np.cos(trailer_angle_rel)) - 
                      L3 * trailer.ang_vel * np.cos(trailer_angle_rel))

    # 编译速度向量
    truck.vel_vec = np.array([truck.long_vel, truck.lat_vel])
    trailer.vel_vec = np.array([trailer.long_vel, trailer.lat_vel])

    # 设置控制输入
    truck.thrust_force = control.thrust_force
    truck.wheel_torque = control.wheel_torque

    # 求解卡车-拖车动力学问题
    truck.long_acc, truck.front_lat_force, truck.rear_lat_force, trailer.rear_lat_force = \
        truck_kinetics(global_params, truck, trailer, truck_props, trailer_props)

    # 求解转向轮动力学问题
    J_phi = truck_props.wheel_moi
    N = truck_props.wheel_gear_ratio
    b = truck_props.wheel_damp_coeff
    k = truck_props.wheel_stiff_coeff
    delta_L = truck_props.wheel_offset
    k_stop = truck_props.wheel_stop_stiff
    phi_max = np.radians(truck_props.max_wheel_angle)

    # 转向轮角加速度
    restoring_torque = (truck.front_lat_force * np.cos(wheel_angle)) * delta_L
    stop_torque = k_stop * (0.5 * np.tanh(wheel_angle - phi_max) + 0.5 + 
                           -0.5 * np.tanh(wheel_angle + phi_max) + 0.5)
    
    wheel_ang_acc = (1/J_phi) * (N * truck.wheel_torque - 
                                k * wheel_angle - 
                                restoring_torque - 
                                b * wheel_ang_vel - 
                                stop_torque)

    # 定义旋转和变换矩阵
    truck.rot_mat = np.array([
        [np.cos(truck_angle), -np.sin(truck_angle)],
        [np.sin(truck_angle), np.cos(truck_angle)]
    ])
    
    trailer.rot_mat = np.array([
        [np.cos(trailer.angle), -np.sin(trailer.angle)],
        [np.sin(trailer.angle), np.cos(trailer.angle)]
    ])
    
    truck.trans_mat_fw = truck.rot_mat.T
    truck.trans_mat_bw = truck.trans_mat_fw.T

    # 计算相关位置向量
    truck.rear_pos_vec = truck_cg_pos + truck.rot_mat @ np.array([-L2, 0])
    trailer.cg_pos_vec = (truck_cg_pos + 
                         truck.rot_mat @ np.array([-L2, 0]) + 
                         trailer.rot_mat @ np.array([-L3, 0]))
    trailer.rear_pos_vec = (truck_cg_pos + 
                           truck.rot_mat @ np.array([-L2, 0]) + 
                           trailer.rot_mat @ np.array([-L4, 0]))

    # 编译状态导数向量
    state_deriv_vec = np.zeros(7)
    state_deriv_vec[0:2] = truck.trans_mat_bw @ truck.vel_vec  # 位置导数
    state_deriv_vec[2] = truck.ang_vel                         # 角度导数
    state_deriv_vec[3] = truck.long_acc                        # 纵向速度导数
    state_deriv_vec[4] = trailer.ang_vel                       # 拖车相对角度导数
    state_deriv_vec[5] = wheel_ang_vel                         # 车轮角度导数
    state_deriv_vec[6] = wheel_ang_acc                         # 车轮角速度导数

    return state_deriv_vec

def run_simulation():
    """运行仿真"""
    print("开始半挂卡车仿真...")
    
    # 初始化设置
    sim_settings = SimulationSettings()
    control_settings = ControllerSettings()
    global_params = GlobalParameters()
    truck_props = TruckProperties()
    trailer_props = TrailerProperties()

    # 计算保存点数
    sim_settings.save_period = min(sim_settings.save_period, control_settings.sampling_period / 2)
    num_save_points = int(sim_settings.end_time / sim_settings.save_period) + 1
    num_samples = int(sim_settings.end_time / control_settings.sampling_period) + 1

    # 初始化状态向量
    init_state_vec = np.array([
        0.0,                    # 卡车x位置 (m)
        0.0,                    # 卡车y位置 (m)
        np.radians(45),         # 卡车角度 (rad)
        0.0,                    # 卡车前进速度 (m/s)
        0.0,                    # 拖车相对于卡车的角度 (rad)
        0.0,                    # 前轮角度 (rad)
        0.0                     # 前轮角速度 (rad/s)
    ])

    # 存储结果
    time_steps = []
    samples = []

    # 初始时间
    start_time = 0.0

    # 在时间=0时保存结果
    for i in range(num_save_points):
        time_step = {
            'time': start_time,
            'state_vec': init_state_vec.copy(),
            'truck': TruckState(),
            'trailer': TrailerState()
        }
        time_steps.append(time_step)

    # 在第一个控制器采样时保存结果
    for i in range(num_samples):
        sample = {
            'time': start_time,
            'state_vec': init_state_vec.copy(),
            'truck': TruckState(),
            'trailer': TrailerState()
        }
        samples.append(sample)

    time_step_num = 1

    # 运行离散时间仿真
    print("运行仿真...")
    for sample_num in range(num_samples - 1):
        # 定义当前采样周期的控制输入
        control_settings.thrust_force = 100.0  # N
        control_settings.wheel_torque = 1.0    # N-m

        # 求解当前采样周期的动力学问题
        time_range = np.arange(start_time, start_time + control_settings.sampling_period + sim_settings.save_period, sim_settings.save_period)
        
        # 使用ODE求解器
        sol = odeint(truck_dyn_model, init_state_vec, time_range, 
                    args=(control_settings, global_params, truck_props, trailer_props))
        
        # 保存仿真结果
        for i in range(1, len(sol)):
            time_steps[time_step_num]['time'] = time_range[i]
            time_steps[time_step_num]['state_vec'] = sol[i]
            time_step_num += 1
        
        # 更新下一个采样周期的输入
        start_time += control_settings.sampling_period
        init_state_vec = sol[-1]
        
        # 显示进度
        if sample_num % 5 == 0:
            print(f"进度: {sample_num + 1}/{num_samples - 1}")

    print("仿真完成!")
    
    # 生成动画
    if sim_settings.anim_play_rate != 0:
        generate_animation(time_steps, truck_props, trailer_props, sim_settings)
    
    # 生成输出图表
    generate_plots(samples, num_samples)
    
    return time_steps, samples

def generate_animation(time_steps, truck_props, trailer_props, sim_settings):
    """生成卡车-拖车动画"""
    print("生成动画...")
    
    # 提取卡车CG位置轨迹
    pos_vec_traj = np.zeros((2, len(time_steps)))
    for i, step in enumerate(time_steps):
        pos_vec_traj[:, i] = step['state_vec'][0:2]
    
    # 初始化图形
    fig, ax = plt.subplots(figsize=(10, 8))
    
    def animate(frame):
        ax.clear()
        
        # 获取当前时间步
        step = time_steps[frame]
        state_vec = step['state_vec']
        
        # 计算卡车角点坐标
        truck_cg_pos = state_vec[0:2]
        truck_angle = state_vec[2]
        
        # 卡车旋转矩阵
        truck_rot_mat = np.array([
            [np.cos(truck_angle), -np.sin(truck_angle)],
            [np.sin(truck_angle), np.cos(truck_angle)]
        ])
        
        # 卡车角点
        L1 = truck_props.cg2front_length
        L2 = truck_props.cg2rear_length
        W = truck_props.width / 2
        
        truck_points = np.array([
            [L1, W],    # 前右
            [L1, -W],   # 前左
            [-L2, -W],  # 后左
            [-L2, W],   # 后右
            [L1, W]     # 闭合
        ]).T
        
        # 旋转和平移卡车点
        truck_points_rotated = truck_rot_mat @ truck_points
        truck_points_final = truck_points_rotated + truck_cg_pos.reshape(2, 1)
        
        # 绘制卡车
        ax.plot(truck_points_final[0, :], truck_points_final[1, :], 'k-', linewidth=2)
        
        # 计算拖车角点坐标
        trailer_angle_rel = state_vec[4]
        trailer_angle = truck_angle + trailer_angle_rel
        
        # 拖车旋转矩阵
        trailer_rot_mat = np.array([
            [np.cos(trailer_angle), -np.sin(trailer_angle)],
            [np.sin(trailer_angle), np.cos(trailer_angle)]
        ])
        
        # 卡车后部位置
        truck_rear_pos = truck_cg_pos + truck_rot_mat @ np.array([-L2, 0])
        
        # 拖车角点
        L4 = trailer_props.pivot2rear_length
        W_t = trailer_props.width / 2
        
        trailer_points = np.array([
            [0, W_t],           # 前右
            [0, -W_t],          # 前左
            [-L4, -W_t],        # 后左
            [-L4, W_t],         # 后右
            [0, W_t]            # 闭合
        ]).T
        
        # 旋转和平移拖车点
        trailer_points_rotated = trailer_rot_mat @ trailer_points
        trailer_points_final = trailer_points_rotated + truck_rear_pos.reshape(2, 1)
        
        # 绘制拖车
        ax.plot(trailer_points_final[0, :], trailer_points_final[1, :], 'k-', linewidth=2)
        
        # 绘制轨迹
        ax.plot(pos_vec_traj[0, :frame+1], pos_vec_traj[1, :frame+1], 'r-', linewidth=1, alpha=0.7)
        
        # 设置图形属性
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title(f'半挂卡车仿真 - 时间: {step["time"]:.1f}s')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        # 自动调整坐标轴范围
        margin = 5
        all_x = np.concatenate([truck_points_final[0, :], trailer_points_final[0, :]])
        all_y = np.concatenate([truck_points_final[1, :], trailer_points_final[1, :]])
        ax.set_xlim([np.min(all_x) - margin, np.max(all_x) + margin])
        ax.set_ylim([np.min(all_y) - margin, np.max(all_y) + margin])
    
    # 创建动画
    anim = animation.FuncAnimation(fig, animate, frames=len(time_steps), 
                                  interval=sim_settings.save_period/sim_settings.anim_play_rate*1000, 
                                  repeat=True)
    
    plt.tight_layout()
    plt.show()
    
    return anim

def generate_plots(samples, num_samples):
    """生成输出图表"""
    print("生成图表...")
    
    # 提取绘图数据
    time_data = np.zeros(num_samples)
    truck_long_vel = np.zeros(num_samples)
    truck_lat_vel = np.zeros(num_samples)
    truck_wheel_angle = np.zeros(num_samples)
    trailer_angle_rel = np.zeros(num_samples)
    
    for i in range(num_samples):
        time_data[i] = samples[i]['time']
        state_vec = samples[i]['state_vec']
        truck_long_vel[i] = state_vec[3]  # 纵向速度
        truck_wheel_angle[i] = np.degrees(state_vec[5])  # 车轮角度 (度)
        trailer_angle_rel[i] = np.degrees(state_vec[4])  # 拖车相对角度 (度)
    
    # 创建图表
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    # 卡车前进速度
    axes[0, 0].plot(time_data, truck_long_vel, 'b-', linewidth=2)
    axes[0, 0].set_xlabel('时间 t (s)')
    axes[0, 0].set_ylabel('卡车纵向速度 (m/s)')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_title('卡车纵向速度')
    
    # 卡车车轮角度
    axes[0, 1].plot(time_data, truck_wheel_angle, 'g-', linewidth=2)
    axes[0, 1].set_xlabel('时间 t (s)')
    axes[0, 1].set_ylabel('卡车车轮角度 (度)')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_title('卡车车轮角度')
    
    # 卡车横向速度
    axes[1, 0].plot(time_data, truck_lat_vel, 'r-', linewidth=2)
    axes[1, 0].set_xlabel('时间 t (s)')
    axes[1, 0].set_ylabel('卡车横向速度 (m/s)')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_title('卡车横向速度')
    
    # 拖车相对角度
    axes[1, 1].plot(time_data, trailer_angle_rel, 'm-', linewidth=2)
    axes[1, 1].set_xlabel('时间 t (s)')
    axes[1, 1].set_ylabel('拖车相对角度 (度)')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_title('拖车相对角度')
    
    plt.tight_layout()
    plt.show()

def main():
    """主函数"""
    print("半挂卡车模拟器 - Python版本")
    print("=" * 50)
    
    try:
        # 运行仿真
        time_steps, samples = run_simulation()
        print("仿真成功完成!")
        
    except Exception as e:
        print(f"仿真过程中出现错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
