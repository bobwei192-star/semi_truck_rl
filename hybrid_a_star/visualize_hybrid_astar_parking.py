#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hybrid A* 牵引车-挂车垂直泊车可视化
使用本项目的车辆模型、地图、配置和渲染系统
"""

import sys
import os
import numpy as np
import pygame

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.scene_config import (
    START_X, START_Y, GOAL_X, GOAL_Y, INIT_HEADING_RAD, INIT_G_RAD,
    DT, FIXED_SPEED, MAX_STEERING_DEG, MAX_STEPS,
    ROAD_CENTER_X, LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH, SCALE,
    PARKING_TOP_Y, PARKING_BTM_Y, PARKING_LINE_Y
)
from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from map.dual_lane_road import DualLaneRoad
from constraints.parking_constraints import verify_parking_success
from render.render_utils import draw_corner_numbers


# 颜色定义
TRUCK_BORDER = (255, 0, 0)
TRAILER_BORDER = (165, 42, 42)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 215, 0)
GRAY = (160, 160, 160)


class ParkingController:
    """泊车控制器 - 倒车入库精确控制"""
    
    def __init__(self, truck):
        self.truck = truck
        self.max_steer = np.deg2rad(MAX_STEERING_DEG)
        self.phase = 0
    
    def compute_control(self, goal_x, goal_y, path_x, path_y):
        """计算转向控制量 - 倒车入库专用
        
        关键理解：
        - 车辆倒车，速度为负
        - 车头朝向h=-90°（朝上，Y轴负方向）
        - 第五轮在车头后方（Y轴正方向）
        - 倒车入库需要：先右摆挂车，再逐渐回正，直线倒入车库
        
        控制策略：
        - Y < 35: 建立负铰接角，让挂车向右偏移
        - 35 <= Y < 45: 逐渐回正铰接角
        - Y >= 45: 完全回正，直线倒入车库
        """
        current_x = self.truck.x_m
        current_y = self.truck.y_m
        current_h = self.truck.h_rad
        current_g = self.truck.g_rad
        
        # 计算第五轮（铰接点）位置
        fifth_wheel_x = current_x - self.truck.truckWheelbase * np.cos(current_h)
        fifth_wheel_y = current_y - self.truck.truckWheelbase * np.sin(current_h)
        
        # 目标位置
        goal_x = 58.25
        goal_y = 53.56
        
        # 横向误差
        lateral_error = fifth_wheel_x - goal_x
        
        # 根据第五轮Y坐标分阶段控制
        if fifth_wheel_y < 38.0:
            # 阶段1：远处 - 建立负铰接角，让挂车向右偏移
            self.phase = 1
            target_g = np.deg2rad(-1.78)  # 最优参数
            k_g = 2.5
            steering = k_g * (target_g - current_g)
            steering = np.clip(steering, -np.deg2rad(5), np.deg2rad(5))
            
        elif fifth_wheel_y < 44.0:
            # 阶段2：过渡 - 快速回正，建立正铰接角让挂车向左
            self.phase = 2
            ratio = (fifth_wheel_y - 38.0) / (44.0 - 38.0)
            
            if ratio > 0.5:
                sub_ratio = (ratio - 0.5) / 0.5
                target_g = np.deg2rad(2.5 * sub_ratio)  # 保持原值
            else:
                target_g = np.deg2rad(-1.78 * (1 - ratio / 0.5))
            
            k_g = 4.0
            steering = k_g * (target_g - current_g)
            steering = np.clip(steering, -np.deg2rad(8), np.deg2rad(8))
            
        elif fifth_wheel_y < 50.0:
            # 阶段3：接近车库 - 保持正铰接角
            self.phase = 3
            
            # 根据横向误差调整
            if lateral_error < 0.14:  # 在目标左边，需要向右
                k_lateral = 1.38
                target_g = k_lateral * lateral_error
                target_g = np.clip(target_g, np.deg2rad(-0.1), np.deg2rad(2.5))
                steering = 4.0 * (target_g - current_g)
            else:
                k_hinge = 14.0
                steering = -k_hinge * current_g
            
            steering = np.clip(steering, -np.deg2rad(6), np.deg2rad(6))
            
        else:
            # 阶段4：进入车库 - 保持小幅正铰接角
            self.phase = 4
            
            k_hinge = 28.0
            steering = -k_hinge * current_g
            
            # 如果需要，继续调整
            if lateral_error < 0.08:  # 在目标左边
                target_g = np.deg2rad(0.32)  # 最优参数
                steering = 3.0 * (target_g - current_g)
            
            steering = np.clip(steering, -np.deg2rad(3), np.deg2rad(3))
        
        return np.rad2deg(steering)
    
    def reset(self):
        """重置控制器状态"""
        self.phase = 0


def draw_truck(surface, truck, scale, offset_x=0, offset_y=0):
    """绘制车辆"""
    # 应用坐标偏移
    x_px = truck.x_m * scale + offset_x
    y_px = truck.y_m * scale + offset_y
    
    # 车辆尺寸
    t_len = (truck.truckWheelbase + 2 * truck.truckMargin) * scale
    t_wid = truck.truckWidth * scale
    tr_len = (truck.trailerWheelbase + truck.trailerMarginFront + truck.trailerMarginBack) * scale
    tr_wid = truck.trailerWidth * scale
    
    # 绘制车头
    sf = 2
    t_big = pygame.Surface((int(t_len * sf), int(t_wid * sf)), pygame.SRCALPHA)
    pygame.draw.rect(t_big, TRUCK_BORDER, t_big.get_rect(), width=4 * sf)
    t_surf = pygame.transform.smoothscale(t_big, (int(t_len), int(t_wid)))
    
    # 绘制挂车
    tr_big = pygame.Surface((int(tr_len * sf), int(tr_wid * sf)), pygame.SRCALPHA)
    pygame.draw.rect(tr_big, TRAILER_BORDER, tr_big.get_rect(), width=4 * sf)
    tr_surf = pygame.transform.smoothscale(tr_big, (int(tr_len), int(tr_wid)))
    
    # 旋转
    rot_t = pygame.transform.rotate(t_surf, -np.degrees(truck.h_rad))
    rot_tr = pygame.transform.rotate(tr_surf, -np.degrees(truck.h_rad + truck.g_rad))
    
    # 挂车中心点
    cnx = x_px + (-t_len / 2 + truck.truckMargin * scale) * np.cos(truck.h_rad)
    cny = y_px + (-t_len / 2 + truck.truckMargin * scale) * np.sin(truck.h_rad)
    tr_cx = cnx + (truck.trailerMarginFront - (truck.trailerWheelbase + truck.trailerMarginBack) / 2) * scale * np.cos(truck.h_rad + truck.g_rad)
    tr_cy = cny + (truck.trailerMarginFront - (truck.trailerWheelbase + truck.trailerMarginBack) / 2) * scale * np.sin(truck.h_rad + truck.g_rad)
    
    # 绘制
    surface.blit(rot_t, rot_t.get_rect(center=(int(x_px), int(y_px))))
    surface.blit(rot_tr, rot_tr.get_rect(center=(int(tr_cx), int(tr_cy))))
    
    # 绘制轮胎
    wl = truck.tireLen * scale
    ww = truck.tireWidth * scale
    for side in [-1, 1]:
        wx = (truck.truckWheelbase + truck.truckMargin) * scale - t_len / 2
        wy = side * t_wid / 2
        ws = pygame.Surface((int(wl), int(ww)), pygame.SRCALPHA)
        ws.fill(BLACK)
        rw = pygame.transform.rotate(ws, -np.degrees(truck.h_rad) - truck.s_deg)
        wp = (x_px + wx * np.cos(truck.h_rad) - wy * np.sin(truck.h_rad),
              y_px + wx * np.sin(truck.h_rad) + wy * np.cos(truck.h_rad))
        surface.blit(rw, rw.get_rect(center=wp))
    
    # 绘制车头中心点
    pygame.draw.circle(surface, RED, (int(x_px), int(y_px)), 5)


def draw_road(surface, road, scale, offset_x=0, offset_y=0):
    """绘制道路"""
    road.draw(surface)


def draw_trajectory(surface, points, color=GREEN, radius=2, offset_x=0, offset_y=0):
    """绘制轨迹"""
    if len(points) < 2:
        return
    for i in range(1, len(points)):
        px, py = points[i - 1]
        cx, cy = points[i]
        # 应用坐标偏移
        px += offset_x
        py += offset_y
        cx += offset_x
        cy += offset_y
        if abs(cx - px) > 200 or abs(cy - py) > 200:
            continue
        pygame.draw.circle(surface, color, (int(cx), int(cy)), radius)


class ParkingVisualizer:
    """泊车可视化器"""
    
    def __init__(self):
        # 初始化 Pygame
        os.environ.setdefault('SDL_VIDEODRIVER', 'x11')
        os.environ.setdefault('MESA_GL_VERSION_OVERRIDE', '3.3')
        os.environ.setdefault('MESA_GLSL_VERSION_OVERRIDE', '330')
        if not pygame.get_init():
            pygame.init()
        
        # 全屏模式
        info = pygame.display.Info()
        self.width, self.height = info.current_w, info.current_h
        self.screen = pygame.display.set_mode(
            (self.width, self.height),
            pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        pygame.display.set_caption('Hybrid A* Trailer Parking')
        
        # 渲染缩放比例
        self.scale = SCALE
        
        # 计算坐标偏移，使场景居中显示
        # 场景范围：X: 50-62, Y: 0-70
        # 起点X=56.25, Y=9.91; 目标X=58.25, Y=53.56
        scene_width = 20.0 * self.scale  # 20米宽
        scene_height = 75.0 * self.scale  # 75米高
        
        # 使场景中心在窗口中心
        scene_center_x = 56.0  # 场景中心X
        scene_center_y = 35.0  # 场景中心Y
        
        self.offset_x = self.width / 2 - scene_center_x * self.scale
        self.offset_y = self.height / 2 - scene_center_y * self.scale
        
        print(f"窗口尺寸: {self.width}x{self.height}")
        print(f"坐标偏移: offset_x={self.offset_x:.1f}, offset_y={self.offset_y:.1f}")
        print(f"起点像素: ({START_X * self.scale + self.offset_x:.1f}, {START_Y * self.scale + self.offset_y:.1f})")
        print(f"目标像素: ({GOAL_X * self.scale + self.offset_x:.1f}, {GOAL_Y * self.scale + self.offset_y:.1f})")
        
        # 创建车辆
        self.truck = TruckTrailerOnAxle()
        
        # 创建地图
        self.road = DualLaneRoad(
            center_x=ROAD_CENTER_X,
            y_start=0.0,
            y_end=70.0,
            goal_x=GOAL_X,
            goal_y=GOAL_Y
        )
        self.road.set_start_position(START_X, START_Y)
        
        # 创建控制器
        self.controller = ParkingController(self.truck)
        
        # 状态变量
        self.parking_done = False
        self.parking_success = False
        self.step_count = 0
        self.trajectory = []
        
        # 目标位置
        self.goal_x = GOAL_X
        self.goal_y = GOAL_Y
    
    def reset(self):
        """重置环境"""
        self.truck.set_state([START_X, START_Y, INIT_HEADING_RAD, INIT_G_RAD])
        self.truck.v_mps = FIXED_SPEED
        self.truck.set_steering(0)
        
        self.parking_done = False
        self.parking_success = False
        self.step_count = 0
        self.trajectory = []
        self.controller.reset()
        
        print(f"\n重置环境")
        print(f"  初始位置: ({START_X:.2f}, {START_Y:.2f})")
        print(f"  目标位置: ({self.goal_x:.2f}, {self.goal_y:.2f})")
    
    def compute_trailer_center(self):
        """计算挂车中心点（几何中心）"""
        x = self.truck.x_m
        y = self.truck.y_m
        h = self.truck.h_rad
        g = self.truck.g_rad
        
        # 挂车前轴位置 = 车头位置 - truckWheelbase * 方向向量
        # 挂车中心 = 挂车前轴位置 - trailerWheelbase/2 * 挂车方向向量
        trailer_x = x - self.truck.truckWheelbase * np.cos(h) - self.truck.trailerWheelbase/2 * np.cos(h + g)
        trailer_y = y - self.truck.truckWheelbase * np.sin(h) - self.truck.trailerWheelbase/2 * np.sin(h + g)
        
        return trailer_x, trailer_y
    
    def check_goal_reached(self):
        """检查是否到达目标 - 必须满足所有约束"""
        # 计算第五轮（铰接点）位置
        fifth_wheel_x = self.truck.x_m - self.truck.truckWheelbase * np.cos(self.truck.h_rad)
        fifth_wheel_y = self.truck.y_m - self.truck.truckWheelbase * np.sin(self.truck.h_rad)
        
        # 检查第五轮是否接近目标位置
        dist_to_goal = np.sqrt((fifth_wheel_x - self.goal_x)**2 + (fifth_wheel_y - self.goal_y)**2)
        
        # 检查铰接角
        hinge_angle = abs(self.truck.g_rad)
        
        # 条件检查
        in_garage_y = 50.5 <= fifth_wheel_y <= 56.0  # 车库Y范围50.5-69.0
        x_close = abs(fifth_wheel_x - self.goal_x) < 1.5
        
        # 调试输出（每50步打印一次）
        if self.step_count % 50 == 0 and self.step_count > 0:
            print(f"  [DEBUG] fifth_wheel=({fifth_wheel_x:.2f}, {fifth_wheel_y:.2f}), "
                  f"dist={dist_to_goal:.2f}, in_y={in_garage_y}, x_close={x_close}")
        
        if in_garage_y and x_close and hinge_angle < np.deg2rad(10) and dist_to_goal < 2.0:
            return True
        
        return False
    
    def run(self):
        """运行可视化"""
        clock = pygame.time.Clock()
        running = True
        
        self.reset()
        
        # 主循环
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        self.reset()
            
            if not self.parking_done and self.step_count < MAX_STEPS:
                # 计算控制量
                steering_deg = self.controller.compute_control(self.goal_x, self.goal_y, [], [])
                
                # 应用控制
                self.truck.set_steering(steering_deg)
                self.truck.drive(DT)
                
                # 记录轨迹
                self.trajectory.append((self.truck.x_m * self.scale, self.truck.y_m * self.scale))
                
                # 检查是否到达目标
                if self.check_goal_reached():
                    self.parking_done = True
                    trailer_x, trailer_y = self.compute_trailer_center()
                    truck_dist = np.sqrt((self.truck.x_m - self.goal_x)**2 + (self.truck.y_m - self.goal_y)**2)
                    trailer_dist = np.sqrt((trailer_x - self.goal_x)**2 + (trailer_y - self.goal_y)**2)
                    print(f"\n到达终点! 总步数: {self.step_count}")
                    print(f"  车头距离: {truck_dist:.2f}m")
                    print(f"  挂车距离: {trailer_dist:.2f}m")
                    print(f"  铰接角: {np.rad2deg(self.truck.g_rad):.1f}°")
                    
                    # 运行约束检查
                    self.parking_success = verify_parking_success(self.truck)
                
                self.step_count += 1
                
                # 打印状态
                if self.step_count % 100 == 0:
                    trailer_x, trailer_y = self.compute_trailer_center()
                    truck_dist = np.sqrt((self.truck.x_m - self.goal_x)**2 + (self.truck.y_m - self.goal_y)**2)
                    trailer_dist = np.sqrt((trailer_x - self.goal_x)**2 + (trailer_y - self.goal_y)**2)
                    print(f"Step {self.step_count}: "
                          f"pos=({self.truck.x_m:.2f}, {self.truck.y_m:.2f}), "
                          f"g={np.rad2deg(self.truck.g_rad):.1f}°, "
                          f"delta={self.truck.s_deg:.1f}°, "
                          f"truck_dist={truck_dist:.2f}m, "
                          f"trailer_dist={trailer_dist:.2f}m")
            
            # 绘制背景
            self.screen.fill(WHITE)
            
            # 绘制道路
            draw_road(self.screen, self.road, self.scale, self.offset_x, self.offset_y)
            
            # 绘制轨迹
            draw_trajectory(self.screen, self.trajectory, color=GREEN, radius=2, 
                          offset_x=self.offset_x, offset_y=self.offset_y)
            
            # 绘制车辆
            draw_truck(self.screen, self.truck, self.scale, self.offset_x, self.offset_y)
            
            # 绘制车辆8个角的编号
            draw_corner_numbers(self.screen, self.truck, self.scale, self.offset_x, self.offset_y)
            
            # 绘制状态信息
            font = pygame.font.SysFont(None, 24)
            status_text = f"Step: {self.step_count}, "
            status_text += f"Pos: ({self.truck.x_m:.1f}, {self.truck.y_m:.1f}), "
            status_text += f"G: {np.rad2deg(self.truck.g_rad):.1f}°, "
            status_text += f"Delta: {self.truck.s_deg:.1f}°"
            
            text_surface = font.render(status_text, True, BLACK)
            self.screen.blit(text_surface, (10, 10))
            
            if self.parking_done:
                success_text = font.render("PARKING SUCCESS!", True, GREEN)
                self.screen.blit(success_text, (10, 40))
            
            pygame.display.flip()
            clock.tick(60)
        
        # 仿真结束
        print("\n" + "="*60)
        print("仿真结束")
        print(f"  总步数: {self.step_count}")
        print(f"  最终位置: ({self.truck.x_m:.2f}, {self.truck.y_m:.2f})")
        print(f"  目标位置: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        
        # 运行全局约束检查
        self.parking_success = verify_parking_success(self.truck)
        
        pygame.quit()


if __name__ == "__main__":
    visualizer = ParkingVisualizer()
    visualizer.run()
