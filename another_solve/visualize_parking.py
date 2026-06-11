# -*- coding: utf-8 -*-
"""
MPC 泊车可视化脚本 - 在地图上实时显示泊车过程

使用 pygame 在项目的道路地图上可视化 MPC 泊车控制器
参考 scripts/train_stage0_visual.py 的绘制方式
"""

import pygame
import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from map.dual_lane_road import DualLaneRoad
from config.scene_config import (
    SCALE, LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH, DT,
    MAX_STEERING_DEG, MAX_STEERING_RATE,
    GOAL_X, GOAL_Y, START_X, START_Y, INIT_HEADING_RAD,
    ROAD_CENTER_X, ROAD_Y_START, ROAD_Y_END,
    PARKING_LINE_Y, PARKING_TOP_Y, PARKING_BTM_Y,
    CURRICULUM_STAGES
)
from render.render_utils import (
    WHITE, BLACK, RED, GREEN, BLUE, YELLOW,
    draw_truck as _draw_truck_fn,
    draw_trajectory as _draw_trajectory_fn,
)

pygame.init()

USE_FULLSCREEN = True

if USE_FULLSCREEN:
    info = pygame.display.Info()
    WIDTH, HEIGHT = info.current_w, info.current_h
else:
    WIDTH, HEIGHT = 1200, 900

screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF)
pygame.display.set_caption("MPC 垂直泊车可视化")

font = pygame.font.SysFont(None, 28)


class MPCParkingVisualizer:
    """MPC 泊车可视化器"""
    
    def __init__(self):
        # 车辆模型
        self.truck = TruckTrailerOnAxle()
        
        # 道路环境 - 与 train_stage0_visual.py 保持一致
        self.road = DualLaneRoad(
            center_x=WIDTH / (2 * SCALE),
            y_start=0,
            y_end=HEIGHT / SCALE,
            left_lane_width=LEFT_LANE_WIDTH,
            right_lane_width=RIGHT_LANE_WIDTH,
            scale=SCALE,
            road_id=1
        )
        
        # 路径规划器
        from another_solve.path_planner import PathPlanner
        self.path_planner = PathPlanner()
        
        # MPC 控制器
        from another_solve.mpc_controller import SimpleMPCController
        self.controller = SimpleMPCController()
        
        # 轨迹记录
        self.trajectory = []
        
        # 状态
        self.step_count = 0
        self.max_steps = 3000
        self.parking_done = False
        
        # 参考路径 (论文坐标系)
        self.reference_path_px = []  # 像素坐标
        self.raw_path = []  # 米坐标
        
        # 初始化路径
        self._plan_path()
    
    def _plan_path(self):
        """规划泊车路径 - 使用项目坐标系（米）"""
        # 使用 config/scene_config.py 中的配置值
        start_point = (START_X, START_Y)
        end_point = (GOAL_X, GOAL_Y)
        
        # 生成路径
        result = self.path_planner.generate_parking_path(
            start_point=start_point,
            arc_start=(GOAL_X, START_Y + (GOAL_Y - START_Y) * 0.3)
        )
        
        # 保存原始路径（项目坐标系）
        self.raw_path = result['path']
        
        # 库位中心（使用配置文件中的目标位置）
        self.parking_center_x = GOAL_X
        self.parking_center_y = GOAL_Y
        
        # 不需要坐标转换了，直接使用项目坐标系
        self.offset_x = 0.0
        self.offset_y = 0.0
        
        # 设置控制器的参考路径（项目坐标系）
        self.controller.set_reference_path(self.raw_path)
        
        self.reference_path_px = []
        for p in self.raw_path:
            px = p[0] * SCALE
            py = p[1] * SCALE
            self.reference_path_px.append((px, py))
        
        print(f"路径规划完成:")
        print(f"  路径点数: {len(self.raw_path)}")
        print(f"  起点: ({self.raw_path[0][0]:.2f}, {self.raw_path[0][1]:.2f})")
        print(f"  终点: ({self.raw_path[-1][0]:.2f}, {self.raw_path[-1][1]:.2f})")
        print(f"  库位中心: ({self.parking_center_x:.2f}, {self.parking_center_y:.2f})")
    
    def reset(self):
        """重置环境 - 从库外开始"""
        # 使用 config/scene_config.py 中的配置值
        init_x = START_X
        init_y = START_Y
        init_h = INIT_HEADING_RAD  # -90°, 车头朝上
        init_g = 0.0
        
        self.truck.set_state([init_x, init_y, init_h, init_g])
        self.truck.v_mps = -0.5  # 降低倒车速度，增加稳定性
        self.truck.s_deg = 0
        
        self.trajectory = []
        self.step_count = 0
        self.parking_done = False
        
        # 库位中心
        print(f"\n重置环境")
        print(f"  初始位置: ({init_x:.2f}, {init_y:.2f})")
        print(f"  目标位置: ({self.parking_center_x:.2f}, {self.parking_center_y:.2f})")
    
    def step(self):
        """执行一步控制"""
        if self.parking_done:
            return
        
        # 获取当前位置
        current_x = self.truck.x_m
        current_y = self.truck.y_m
        
        # 计算挂车中心点位置
        trailer_hitch_x = self.truck.x_m - self.truck.truckWheelbase * np.cos(self.truck.h_rad)
        trailer_hitch_y = self.truck.y_m - self.truck.truckWheelbase * np.sin(self.truck.h_rad)
        trailer_heading = self.truck.h_rad - self.truck.g_rad
        
        # 挂车中心点（挂车长度的一半处）
        trailer_center_x = trailer_hitch_x - (self.truck.trailerWheelbase / 2) * np.cos(trailer_heading)
        trailer_center_y = trailer_hitch_y - (self.truck.trailerWheelbase / 2) * np.sin(trailer_heading)
        
        # 检查是否到达终点（同时考虑车头和挂车）
        truck_dist_to_end = np.sqrt((current_x - self.parking_center_x)**2 + (current_y - self.parking_center_y)**2)
        trailer_dist_to_end = np.sqrt((trailer_center_x - self.parking_center_x)**2 + (trailer_center_y - self.parking_center_y)**2)
        
        # 当车头和挂车都接近终点时才算完成
        if truck_dist_to_end < 1.0 and trailer_dist_to_end < 1.5 and abs(self.truck.g_rad) < np.deg2rad(15):
            self.parking_done = True
            print(f"\n到达终点! 总步数: {self.step_count}, 车头距离: {truck_dist_to_end:.2f}m, 挂车距离: {trailer_dist_to_end:.2f}m")
            return
        
        # 调试：检查是否满足终点条件
        if self.step_count % 50 == 0:
            print(f"调试: 车头距离={truck_dist_to_end:.2f}m, 挂车距离={trailer_dist_to_end:.2f}m, 铰接角={np.rad2deg(self.truck.g_rad):.1f}°")
        
        # 显示挂车信息
        dist_to_end = truck_dist_to_end
        
        # 计算挂车横向偏差（相对于期望路径 x = GOAL_X）
        # lateral_error > 0 表示挂车在目标右边，需要向左移
        # lateral_error < 0 表示挂车在目标左边，需要向右移
        lateral_error = trailer_center_x - self.parking_center_x
        
        # 当前铰接角 (g = h - trailer_heading)
        # g > 0: 车头相对于挂车向右转
        # g < 0: 车头相对于挂车向左转
        hinge_angle = self.truck.g_rad
        
        # 控制策略：按照用户描述的逻辑
        # 物理原理（从测试得出）：
        # - 倒车时(v_mps < 0)，向左打方向盘(s_deg > 0)，车头向左上方移动，铰接角g增加
        # - 铰接角g的变化：向左打g增加，向右打g减少
        
        # 当前策略：
        # 1. 车在库位前方偏左(56.25)，目标在(58.25, 53.56)
        # 2. 需要起步时向左打方向盘，让挂车向右偏
        # 3. 当铰接角达到一定程度时，向右打方向盘修正（但不要过度）
        # 4. 保持铰接角在小范围(-25° ~ 25°)
        
        # 控制策略：基于横向偏差的目标铰接角控制
        # 核心思想：根据横向偏差计算目标铰接角，用强反馈控制跟踪
        
        # 目标铰接角：根据横向偏差计算
        # 挂车在目标左边（lateral_error < 0），需要正铰接角让挂车向右移动
        # 挂车在目标右边（lateral_error > 0），需要负铰接角让挂车向左移动
        # 因此：target_hinge 与 lateral_error 反号
        max_target_hinge = np.deg2rad(6)  # 最大目标铰接角6度
        k_target = 0.5  # 目标铰接角系数
        target_hinge = -k_target * lateral_error  # lateral_error < 0 -> target_hinge > 0
        target_hinge = np.clip(target_hinge, -max_target_hinge, max_target_hinge)
        
        # 强反馈控制跟踪目标铰接角
        k_hinge = 10.0
        hinge_error = hinge_angle - target_hinge
        desired_steering = -k_hinge * hinge_error
        
        # 限制最大转向角
        max_steer = np.deg2rad(20)
        desired_steering = np.clip(desired_steering, -max_steer, max_steer)
        
        # 接近目标时逐渐回正
        if trailer_dist_to_end < 10.0:
            factor = trailer_dist_to_end / 10.0
            target_hinge = target_hinge * factor
            hinge_error = hinge_angle - target_hinge
            desired_steering = -k_hinge * hinge_error
            desired_steering = np.clip(desired_steering, -max_steer, max_steer)
        
        # 铰接角安全约束
        max_hinge = np.deg2rad(15)
        if hinge_angle > max_hinge:
            desired_steering = np.deg2rad(-30)
        elif hinge_angle < -max_hinge:
            desired_steering = np.deg2rad(30)
        
        # 添加阻尼项，防止震荡
        # damping = 0.1 * self.truck.s_deg / 35.0  # 基于当前转向角的阻尼
        # desired_steering -= damping
        
        # 限制转向角（35度）
        delta_deg = np.clip(np.rad2deg(desired_steering), -35.0, 35.0)
        
        # 调试日志 (每100步输出)
        if self.step_count % 100 == 0:
            print(f"Step {self.step_count}: 车头=({current_x:.2f}, {current_y:.2f}), 挂车=({trailer_center_x:.2f}, {trailer_center_y:.2f}), "
                  f"heading={np.rad2deg(self.truck.h_rad):.1f}°, trailer_h={np.rad2deg(trailer_heading):.1f}°, "
                  f"g={np.rad2deg(self.truck.g_rad):.1f}°, "
                  f"delta={delta_deg:.1f}°, "
                  f"dist_to_end={dist_to_end:.2f}m")
        
        # 直接应用转向角
        self.truck.set_steering(delta_deg)
        self.truck.drive(DT)
        
        # 记录轨迹 (像素坐标)
        self.trajectory.append((self.truck.x_m * SCALE, self.truck.y_m * SCALE))
        self.step_count += 1
        
        # 检查是否超时
        if self.step_count >= self.max_steps:
            self.parking_done = True
            print(f"\n达到最大步数限制: {self.max_steps}")
    
    def draw_info_panel(self):
        """绘制信息面板"""
        # 计算到库位中心的距离
        parking_center_x = ROAD_CENTER_X + RIGHT_LANE_WIDTH / 2
        parking_center_y = (PARKING_TOP_Y + PARKING_BTM_Y) / 2
        dist_to_goal = np.sqrt((self.truck.x_m - parking_center_x)**2 + (self.truck.y_m - parking_center_y)**2)
        
        lines = [
            f"MPC 垂直泊车控制器",
            f"",
            f"步数: {self.step_count}/{self.max_steps}",
            f"位置: ({self.truck.x_m:.2f}, {self.truck.y_m:.2f})",
            f"目标: ({parking_center_x:.2f}, {parking_center_y:.2f})",
            f"航向角: {np.rad2deg(self.truck.h_rad):.1f}°",
            f"铰接角: {np.rad2deg(self.truck.g_rad):.1f}°",
            f"转向角: {self.truck.s_deg:.1f}°",
            f"到终点距离: {dist_to_goal:.2f}m",
            f"",
            f"状态: {'完成' if self.parking_done else '执行中'}",
            f"",
            f"按 R 重置 | ESC 退出",
        ]
        
        for i, line in enumerate(lines):
            color = GREEN if self.parking_done and i == 10 else WHITE
            text = font.render(line, True, color)
            screen.blit(text, (10, 10 + i * 28))
    
    def draw_reference_path(self):
        """绘制参考路径"""
        if len(self.reference_path_px) > 1:
            for i in range(1, len(self.reference_path_px)):
                pygame.draw.circle(
                    screen, YELLOW,
                    (int(self.reference_path_px[i][0]), int(self.reference_path_px[i][1])),
                    2
                )
    
    def run(self):
        """运行可视化"""
        clock = pygame.time.Clock()
        
        print("\n" + "=" * 60)
        print("MPC 垂直泊车可视化")
        print("=" * 60)
        print("按 ESC 退出")
        print("按 R 重置")
        print("=" * 60 + "\n")
        
        self.reset()
        
        running = True
        paused = False
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        self.reset()
                    elif event.key == pygame.K_SPACE:
                        paused = not paused
            
            # 执行控制步骤
            if not paused and not self.parking_done:
                self.step()
            
            # 绘制 - 与 train_stage0_visual.py 一致
            screen.fill(BLACK)
            
            # 绘制道路
            self.road.draw(screen)
            
            # 绘制参考路径
            self.draw_reference_path()
            
            # 绘制实际轨迹
            if len(self.trajectory) > 1:
                _draw_trajectory_fn(screen, self.trajectory)
            
            # 绘制终点和目标容差圈 (库位中心)
            parking_center_x = ROAD_CENTER_X + RIGHT_LANE_WIDTH / 2
            parking_center_y = (PARKING_TOP_Y + PARKING_BTM_Y) / 2
            goal_x_px = parking_center_x * SCALE
            goal_y_px = parking_center_y * SCALE
            pygame.draw.circle(screen, GREEN, (int(goal_x_px), int(goal_y_px)), int(0.5 * SCALE), 2)
            
            # 绘制车辆
            _draw_truck_fn(screen, self.truck, SCALE)
            
            # 绘制信息面板
            self.draw_info_panel()
            
            pygame.display.flip()
            clock.tick(60)
        
        pygame.quit()
        
        print("\n" + "=" * 60)
        print("仿真结束")
        print(f"总步数: {self.step_count}")
        print(f"轨迹点数: {len(self.trajectory)}")
        print(f"最终位置: ({self.truck.x_m:.2f}, {self.truck.y_m:.2f})")
        print(f"目标位置: ({GOAL_X:.2f}, {GOAL_Y:.2f})")
        print("=" * 60)


def main():
    visualizer = MPCParkingVisualizer()
    visualizer.run()


if __name__ == "__main__":
    main()
