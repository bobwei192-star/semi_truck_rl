# -*- coding: utf-8 -*-
"""
仿真脚本 - 使用本项目的车辆模型和环境进行泊车仿真

集成:
- vehicle.truckTrailerOnAxle: 本项目的车辆模型
- map.dual_lane_road: 本项目的道路环境
- another_solve: 论文中的MPC控制器
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from map.dual_lane_road import DualLaneRoad
from config.scene_config import SCALE, LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH

from another_solve.config import VEHICLE, PARKING
from another_solve.path_planner import PathPlanner, CollisionAnalyzer
from another_solve.kinematics import TruckTrailerKinematics
from another_solve.mpc_controller import SimpleMPCController


class IntegratedParkingSimulator:
    """集成泊车仿真器"""
    
    def __init__(self):
        # 使用本项目的车辆模型
        self.truck = TruckTrailerOnAxle()
        
        # 使用本项目的道路环境 (延迟初始化,避免pygame问题)
        self.road = None
        
        # 论文中的模块
        self.path_planner = PathPlanner()
        self.kinematics = TruckTrailerKinematics()
        self.controller = SimpleMPCController()
        
        # 仿真结果
        self.trajectory = []
        self.control_history = []
        self.error_history = []
    
    def run(self, start_point=(8.15, 7.2), arc_start=(6.15, 7.2),
            max_steps=2000, dt=0.1):
        """
        运行泊车仿真
        
        Args:
            start_point: 泊车起始点
            arc_start: 圆弧路径起始点
            max_steps: 最大仿真步数
            dt: 时间步长
        
        Returns:
            仿真结果
        """
        print("=" * 60)
        print("集成泊车仿真")
        print("=" * 60)
        
        # 1. 路径规划
        print("\n[1] 路径规划...")
        path_result = self.path_planner.generate_parking_path(
            start_point=start_point,
            arc_start=arc_start
        )
        
        path = path_result['path']
        print(f"  路径点数: {len(path)}")
        
        # 2. 生成参考信息
        print("\n[2] 生成参考信息...")
        reference_trajectory = []
        reference_delta = []
        
        for i in range(len(path)):
            ref_info = self.path_planner.get_reference_info(path, path[i])
            
            state_ref = np.array([
                ref_info['x_ref'],
                ref_info['y_ref'],
                ref_info['phi0_ref'],
                ref_info['phi1_ref']
            ])
            
            reference_trajectory.append(state_ref)
            reference_delta.append(ref_info['delta_ref'])
        
        reference_trajectory = np.array(reference_trajectory)
        reference_delta = np.array(reference_delta)
        
        print(f"  参考轨迹点数: {len(reference_trajectory)}")
        
        # 3. 初始化车辆状态
        print("\n[3] 初始化车辆状态...")
        
        # 使用本项目的车辆模型设置初始状态
        # 状态: [x, y, h, g] -> 对应论文中的 [x_C, y_C, phi_0, phi_1]
        # 注意: 本项目模型中 h 是牵引车航向角, g 是铰接角
        # 论文中 phi_0 是牵引车航向角, phi_1 是挂车航向角
        # 关系: h = phi_0, g = phi_0 - phi_1
        
        initial_state = np.array([start_point[0], start_point[1], 0.0, 0.0])
        self.truck.set_state(initial_state)
        self.truck.set_speed(-1.0)  # 倒车速度 1 m/s
        
        print(f"  初始状态: x={start_point[0]:.2f}, y={start_point[1]:.2f}")
        print(f"  倒车速度: {self.truck.v_mps} m/s")
        
        # 4. MPC控制仿真
        print("\n[4] MPC控制仿真...")
        
        self.trajectory = []
        self.control_history = []
        self.error_history = []
        
        for step in range(max_steps):
            # 获取当前状态
            current_state = self.truck.get_state()
            
            # 找到最近参考点
            ref_info = self.path_planner.get_reference_info(
                path, (current_state[0], current_state[1])
            )
            
            current_idx = ref_info['nearest_idx']
            
            # 检查是否到达终点
            if current_idx >= len(reference_trajectory) - 1:
                print(f"\n  到达终点! 总步数: {step}")
                break
            
            # 获取参考状态
            state_ref = reference_trajectory[current_idx]
            delta_ref = reference_delta[current_idx]
            
            # 计算控制量 (使用本项目的车辆模型)
            # 将论文状态转换为本项目状态
            # 论文: [x_C, y_C, phi_0, phi_1]
            # 本项目: [x, y, h, g] where h=phi_0, g=phi_0-phi_1
            
            paper_state = np.array([
                current_state[0],
                current_state[1],
                current_state[2],  # phi_0 = h
                current_state[2] - current_state[3]  # phi_1 = h - g
            ])
            
            # 计算控制量
            delta = self.controller.compute_control(paper_state, state_ref)
            
            # 转换为本项目的转向角 (度)
            delta_deg = np.rad2deg(delta)
            
            # 设置转向角
            self.truck.set_steering(delta_deg)
            
            # 记录
            self.trajectory.append(current_state.copy())
            self.control_history.append(delta_deg)
            
            # 计算误差
            error = np.linalg.norm(current_state[:2] - state_ref[:2])
            self.error_history.append(error)
            
            # 更新状态
            self.truck.drive(dt)
            
            # 打印进度
            if step % 100 == 0:
                print(f"  Step {step}: pos=({current_state[0]:.2f}, {current_state[1]:.2f}), "
                      f"h={np.rad2deg(current_state[2]):.2f} deg, "
                      f"g={np.rad2deg(current_state[3]):.2f} deg, "
                      f"delta={delta_deg:.2f} deg, "
                      f"error={error:.4f} m")
            
            # 检查是否到达终点附近
            end = path_result['end_point']
            if np.linalg.norm(current_state[:2] - np.array(end)) < 0.5:
                print(f"\n  到达终点附近! 总步数: {step}")
                break
        
        else:
            print(f"\n  达到最大步数限制 ({max_steps})")
        
        # 5. 返回结果
        return {
            'trajectory': np.array(self.trajectory),
            'control_history': np.array(self.control_history),
            'error_history': np.array(self.error_history),
            'path': path,
            'truck': self.truck
        }
    
    def visualize(self, result, save_path=None):
        """
        可视化结果
        
        Args:
            result: 仿真结果
            save_path: 保存路径
        """
        import matplotlib.pyplot as plt
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # 1. 轨迹跟踪
        ax = axes[0, 0]
        
        # 参考路径
        path = result['path']
        ax.plot(path[:, 0], path[:, 1], 'r--', label='Reference Path', linewidth=2)
        
        # 实际轨迹
        traj = result['trajectory']
        ax.plot(traj[:, 0], traj[:, 1], 'b-', label='Actual Trajectory', linewidth=1.5)
        ax.scatter(traj[0, 0], traj[0, 1], c='g', s=100, marker='o', label='Start')
        ax.scatter(traj[-1, 0], traj[-1, 1], c='r', s=100, marker='x', label='End')
        
        # 车位
        pw = PARKING.PW
        pl = PARKING.PL
        rect = plt.Rectangle((-pw/2, 0), pw, pl,
                              fill=False, edgecolor='gray',
                              linestyle='--', linewidth=2, label='Parking Space')
        ax.add_patch(rect)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Trajectory Tracking')
        ax.legend()
        ax.grid(True)
        ax.axis('equal')
        
        # 2. 控制量
        ax = axes[0, 1]
        control = result['control_history']
        ax.plot(control, 'b-', linewidth=1)
        ax.axhline(y=VEHICLE.max_steering_deg, color='r', linestyle='--', label='Max')
        ax.axhline(y=-VEHICLE.max_steering_deg, color='r', linestyle='--')
        ax.set_xlabel('Step')
        ax.set_ylabel('Steering Angle (deg)')
        ax.set_title('Control Input')
        ax.legend()
        ax.grid(True)
        
        # 3. 跟踪误差
        ax = axes[1, 0]
        error = result['error_history']
        ax.plot(error, 'g-', linewidth=1)
        ax.set_xlabel('Step')
        ax.set_ylabel('Error (m)')
        ax.set_title('Tracking Error')
        ax.grid(True)
        
        # 4. 航向角变化
        ax = axes[1, 1]
        ax.plot(np.rad2deg(traj[:, 2]), 'b-', label='h (Truck Heading)', linewidth=1.5)
        ax.plot(np.rad2deg(traj[:, 3]), 'r-', label='g (Hitch Angle)', linewidth=1.5)
        ax.set_xlabel('Step')
        ax.set_ylabel('Angle (deg)')
        ax.set_title('Angles')
        ax.legend()
        ax.grid(True)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"结果已保存到: {save_path}")
        
        plt.show()


def main():
    """主函数"""
    print("=" * 60)
    print("集成泊车仿真 - 使用本项目的车辆模型和环境")
    print("=" * 60)
    
    # 创建仿真器
    simulator = IntegratedParkingSimulator()
    
    # 运行仿真
    result = simulator.run(
        start_point=(8.15, 7.2),
        arc_start=(6.15, 7.2),
        max_steps=2000,
        dt=0.1
    )
    
    # 可视化
    try:
        simulator.visualize(result, save_path='/home/worker/semi_truck_rl/another_solve/integrated_parking.png')
    except Exception as e:
        print(f"可视化失败: {e}")
    
    # 打印统计信息
    print("\n" + "=" * 60)
    print("仿真统计")
    print("=" * 60)
    print(f"总步数: {len(result['trajectory'])}")
    print(f"平均跟踪误差: {np.mean(result['error_history']):.4f} m")
    print(f"最大跟踪误差: {np.max(result['error_history']):.4f} m")
    print(f"最终位置: ({result['trajectory'][-1, 0]:.4f}, {result['trajectory'][-1, 1]:.4f})")
    print(f"最终航向角: h={np.rad2deg(result['trajectory'][-1, 2]):.2f} deg, "
          f"g={np.rad2deg(result['trajectory'][-1, 3]):.2f} deg")


if __name__ == "__main__":
    main()
