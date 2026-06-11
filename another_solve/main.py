# -*- coding: utf-8 -*-
"""
MPC 泊车控制主程序
基于论文 j.cnki.xk.2023.2088 的完整流程实现

流程:
1. 路径规划: B样条曲线生成泊车路径
2. 参考信息生成: 计算参考状态和控制量
3. MPC控制: 跟踪参考路径
4. 仿真验证
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from another_solve.config import VEHICLE, PARKING, MPC as MPC_CONFIG
from another_solve.path_planner import PathPlanner, CollisionAnalyzer
from another_solve.kinematics import TruckTrailerKinematics
from another_solve.mpc_controller import MPCController, SimpleMPCController


class ParkingMPC:
    """基于MPC的泊车控制器"""
    
    def __init__(self, start_point=None, arc_start=None, use_simple_mpc=False):
        """
        初始化
        
        Args:
            start_point: 泊车起始点 (x, y)
            arc_start: 圆弧路径起始点 (x, y)
            use_simple_mpc: 是否使用简化版MPC
        """
        # 参数
        self.vehicle = VEHICLE
        self.parking = PARKING
        self.mpc_params = MPC_CONFIG
        
        # 模块
        self.path_planner = PathPlanner(self.vehicle, self.parking)
        self.kinematics = TruckTrailerKinematics(self.vehicle)
        
        # MPC控制器
        if use_simple_mpc:
            self.controller = SimpleMPCController(self.vehicle, self.parking, self.mpc_params)
        else:
            self.controller = MPCController(self.vehicle, self.parking, self.mpc_params)
        
        # 路径
        self.path_result = None
        self.reference_trajectory = None
        self.reference_delta = None
        
        # 起始点
        self.start_point = start_point
        self.arc_start = arc_start
        
        # 仿真结果
        self.trajectory = []
        self.control_history = []
        self.error_history = []
    
    def plan_path(self):
        """路径规划"""
        print("=" * 60)
        print("路径规划")
        print("=" * 60)
        
        # 生成泊车路径
        self.path_result = self.path_planner.generate_parking_path(
            start_point=self.start_point,
            arc_start=self.arc_start
        )
        
        print(f"起始点: {self.path_result['start_point']}")
        print(f"圆弧起点: {self.path_result['arc_start']}")
        print(f"圆弧终点: {self.path_result['arc_end']}")
        print(f"转向中心: {self.path_result['turning_center']}")
        print(f"终点: {self.path_result['end_point']}")
        print(f"路径点数: {len(self.path_result['path'])}")
        print(f"控制点数: {len(self.path_result['control_points'])}")
        
        return self.path_result
    
    def generate_reference(self):
        """生成参考信息 (论文2.5节)"""
        print("\n" + "=" * 60)
        print("生成参考信息")
        print("=" * 60)
        
        if self.path_result is None:
            raise ValueError("请先执行路径规划")
        
        path = self.path_result['path']
        
        # 计算每个参考点的参考信息
        self.reference_trajectory = []
        self.reference_delta = []
        
        for i in range(len(path)):
            # 获取参考信息
            ref_info = self.path_planner.get_reference_info(path, path[i])
            
            # 构建参考状态
            state_ref = np.array([
                ref_info['x_ref'],
                ref_info['y_ref'],
                ref_info['phi0_ref'],
                ref_info['phi1_ref']
            ])
            
            self.reference_trajectory.append(state_ref)
            self.reference_delta.append(ref_info['delta_ref'])
        
        self.reference_trajectory = np.array(self.reference_trajectory)
        self.reference_delta = np.array(self.reference_delta)
        
        print(f"参考轨迹点数: {len(self.reference_trajectory)}")
        print(f"参考前轮转角范围: [{np.rad2deg(np.min(self.reference_delta)):.2f}, "
              f"{np.rad2deg(np.max(self.reference_delta)):.2f}] deg")
        
        return self.reference_trajectory, self.reference_delta
    
    def run_simulation(self, max_steps=2000, dt=0.1):
        """
        运行仿真
        
        Args:
            max_steps: 最大仿真步数
            dt: 时间步长
        
        Returns:
            仿真结果
        """
        print("\n" + "=" * 60)
        print("MPC泊车仿真")
        print("=" * 60)
        
        if self.path_result is None:
            self.plan_path()
        
        if self.reference_trajectory is None:
            self.generate_reference()
        
        # 初始状态
        start = self.path_result['start_point']
        
        # 初始状态: [x_C, y_C, phi_0, phi_1]
        # 初始航向角为0 (朝右)
        state = np.array([start[0], start[1], 0.0, 0.0])
        
        # 重置控制器
        if hasattr(self.controller, 'reset'):
            self.controller.reset()
        
        # 仿真
        self.trajectory = [state.copy()]
        self.control_history = []
        self.error_history = []
        
        for step in range(max_steps):
            # 找到当前位置对应的参考点
            ref_info = self.path_planner.get_reference_info(
                self.path_result['path'],
                (state[0], state[1])
            )
            
            current_idx = ref_info['nearest_idx']
            
            # 检查是否到达终点
            if current_idx >= len(self.reference_trajectory) - 1:
                print(f"\n到达终点! 总步数: {step}")
                break
            
            # 获取参考状态和控制量
            state_ref = self.reference_trajectory[current_idx]
            delta_ref = self.reference_delta[current_idx]
            
            # 计算控制量
            if isinstance(self.controller, SimpleMPCController):
                delta = self.controller.compute_control(state, state_ref)
            else:
                delta = self.controller.solve(state, state_ref, delta_ref)
            
            # 记录控制量
            self.control_history.append(delta)
            
            # 计算误差
            error = np.linalg.norm(state[:2] - state_ref[:2])
            self.error_history.append(error)
            
            # 更新状态
            state = self.kinematics.step(state, delta, dt)
            self.trajectory.append(state.copy())
            
            # 打印进度
            if step % 100 == 0:
                print(f"Step {step}: pos=({state[0]:.2f}, {state[1]:.2f}), "
                      f"phi_0={np.rad2deg(state[2]):.2f} deg, "
                      f"phi_1={np.rad2deg(state[3]):.2f} deg, "
                      f"delta={np.rad2deg(delta):.2f} deg, "
                      f"error={error:.4f} m")
            
            # 检查是否到达终点附近
            end = self.path_result['end_point']
            if np.linalg.norm(state[:2] - np.array(end)) < 0.5:
                print(f"\n到达终点附近! 总步数: {step}")
                break
        
        else:
            print(f"\n达到最大步数限制 ({max_steps})")
        
        return {
            'trajectory': np.array(self.trajectory),
            'control_history': np.array(self.control_history),
            'error_history': np.array(self.error_history),
            'path': self.path_result['path']
        }
    
    def visualize(self, save_path=None):
        """
        可视化结果
        
        Args:
            save_path: 保存路径
        """
        import matplotlib.pyplot as plt
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # 1. 轨迹跟踪
        ax = axes[0, 0]
        
        # 参考路径
        if self.path_result is not None:
            path = self.path_result['path']
            ax.plot(path[:, 0], path[:, 1], 'r--', label='Reference Path', linewidth=2)
        
        # 实际轨迹
        if len(self.trajectory) > 0:
            traj = np.array(self.trajectory)
            ax.plot(traj[:, 0], traj[:, 1], 'b-', label='Actual Trajectory', linewidth=1.5)
            ax.scatter(traj[0, 0], traj[0, 1], c='g', s=100, marker='o', label='Start')
            ax.scatter(traj[-1, 0], traj[-1, 1], c='r', s=100, marker='x', label='End')
        
        # 车位
        if self.parking is not None:
            # 绘制车位边界
            pw = self.parking.PW
            pl = self.parking.PL
            # 车位在坐标系中的位置 (简化)
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
        if len(self.control_history) > 0:
            control = np.array(self.control_history)
            ax.plot(np.rad2deg(control), 'b-', linewidth=1)
            ax.axhline(y=self.vehicle.max_steering_deg, color='r', linestyle='--', label='Max')
            ax.axhline(y=-self.vehicle.max_steering_deg, color='r', linestyle='--')
            ax.set_xlabel('Step')
            ax.set_ylabel('Steering Angle (deg)')
            ax.set_title('Control Input')
            ax.legend()
            ax.grid(True)
        
        # 3. 跟踪误差
        ax = axes[1, 0]
        if len(self.error_history) > 0:
            ax.plot(self.error_history, 'g-', linewidth=1)
            ax.set_xlabel('Step')
            ax.set_ylabel('Error (m)')
            ax.set_title('Tracking Error')
            ax.grid(True)
        
        # 4. 航向角变化
        ax = axes[1, 1]
        if len(self.trajectory) > 0:
            traj = np.array(self.trajectory)
            ax.plot(np.rad2deg(traj[:, 2]), 'b-', label='phi_0 (Truck)', linewidth=1.5)
            ax.plot(np.rad2deg(traj[:, 3]), 'r-', label='phi_1 (Trailer)', linewidth=1.5)
            ax.set_xlabel('Step')
            ax.set_ylabel('Heading Angle (deg)')
            ax.set_title('Heading Angles')
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
    print("牵引挂车垂直泊车 MPC 控制器")
    print("基于论文: j.cnki.xk.2023.2088")
    print("=" * 60)
    
    # 创建泊车控制器
    # 使用论文中的参数
    start_point = (8.15, 7.2)  # 泊车起始点
    arc_start = (6.15, 7.2)    # 圆弧路径起始点
    
    controller = ParkingMPC(
        start_point=start_point,
        arc_start=arc_start,
        use_simple_mpc=True  # 使用简化版MPC
    )
    
    # 路径规划
    controller.plan_path()
    
    # 生成参考信息
    controller.generate_reference()
    
    # 运行仿真
    result = controller.run_simulation(max_steps=2000, dt=0.1)
    
    # 可视化
    try:
        controller.visualize(save_path='/home/worker/semi_truck_rl/another_solve/parking_result.png')
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
    print(f"最终航向角: phi_0={np.rad2deg(result['trajectory'][-1, 2]):.2f} deg, "
          f"phi_1={np.rad2deg(result['trajectory'][-1, 3]):.2f} deg")


if __name__ == "__main__":
    main()
