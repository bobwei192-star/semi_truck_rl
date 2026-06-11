"""
ADP 控制器可视化测试
使用项目统一的渲染模块（dual_lane_road, render_utils）
"""
import sys
import os
import numpy as np
import pygame

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import gymnasium as gym
import env.truck_parking_env
from adp_solve.adp_controller import ADPController
from scene_config import (
    SCALE, GOAL_X, GOAL_Y, START_X, START_Y,
    ROAD_CENTER_X, ROAD_Y_START, ROAD_Y_END,
    LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH,
    PARKING_LINE_Y, PARKING_TOP_Y, PARKING_BTM_Y,
)
from dual_lane_road import DualLaneRoad
from render_utils import (
    WHITE, BLACK, RED, GREEN, BLUE, YELLOW,
    draw_truck, init_pygame_screen,
)


def run_adp_visual(steps=2000):
    """可视化运行 ADP 控制器"""
    
    pygame.init()
    screen, WIDTH, HEIGHT = init_pygame_screen(fullscreen=True)
    pygame.display.set_caption("ADP Controller - Semi-Truck Reverse Parking")
    
    font = pygame.font.SysFont(None, 36)
    small_font = pygame.font.SysFont(None, 24)
    
    # 创建环境（不使用环境自带渲染）
    # 使用最后一个 stage（距离目标最远）
    env = gym.make('TruckParking-v0', curriculum_stage=4)
    env.unwrapped.set_show_trajectory(True)
    obs, _ = env.reset(seed=42)
    
    # 创建道路渲染
    road = DualLaneRoad(
        center_x=ROAD_CENTER_X,
        y_start=ROAD_Y_START,
        y_end=ROAD_Y_END,
        left_lane_width=LEFT_LANE_WIDTH,
        right_lane_width=RIGHT_LANE_WIDTH,
        scale=SCALE,
        road_id=1,
        verbose=False,
    )
    
    # ADP 控制器
    # 调整 Q 和 R 使控制器更稳定
    Q = np.array([
        [0.1, 0, 0],   # y 误差权重（降低）
        [0, 10, 0],    # heading 误差权重（增加）
        [0, 0, 5]      # g 误差权重（增加）
    ])
    R = 10000.0  # 控制权重（增大，减少方向盘动作）
    
    adp = ADPController(
        truck_wheelbase=env.unwrapped.truck.truckWheelbase,
        trailer_wheelbase=env.unwrapped.truck.trailerWheelbase,
        Q=Q,
        R=R,
        goal_x=GOAL_X  # 传递目标 X 坐标
    )
    
    print("=" * 60)
    print("ADP 控制器可视化测试")
    print("=" * 60)
    print(f"初始 LQR 增益 K: {adp.K}")
    print(f"初始位置: ({env.unwrapped.truck.x_m:.2f}, {env.unwrapped.truck.y_m:.2f})")
    print(f"目标位置: ({GOAL_X}, {GOAL_Y})")
    print("=" * 60)
    
    trajectory_points = []
    total_reward = 0
    step = 0
    success = False
    clock = pygame.time.Clock()
    
    while step < steps:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print("用户退出")
                    running = False
                    break
        
        # 计算控制
        state = np.array([
            env.unwrapped.truck.x_m,
            env.unwrapped.truck.y_m,
            env.unwrapped.truck.h_rad,
            env.unwrapped.truck.g_rad
        ])
        
        u = adp.compute_control(state, velocity=env.unwrapped.truck.v_mps)
        action = np.array([u / 35.0])
        
        # 执行动作
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        
        # 记录轨迹
        trajectory_points.append((env.unwrapped.truck.x_m * SCALE, env.unwrapped.truck.y_m * SCALE))
        
        # 渲染
        screen.fill(WHITE)
        
        # 绘制道路
        road.draw(screen)
        
        # 绘制目标点
        goal_px = int(GOAL_X * SCALE)
        goal_py = int(GOAL_Y * SCALE)
        pygame.draw.circle(screen, GREEN, (goal_px, goal_py), 15, 3)
        pygame.draw.circle(screen, GREEN, (goal_px, goal_py), 5)
        
        # 绘制目标区域
        parking_left = int((GOAL_X - 1.5) * SCALE)
        parking_right = int((GOAL_X + 1.5) * SCALE)
        parking_top = int(PARKING_TOP_Y * SCALE)
        parking_btm = int(PARKING_BTM_Y * SCALE)
        pygame.draw.rect(screen, YELLOW, (parking_left, parking_top, parking_right - parking_left, parking_btm - parking_top), 2)
        
        # 绘制轨迹（绿色点）
        for px, py in trajectory_points[-500:]:
            pygame.draw.circle(screen, (0, 200, 0), (int(px), int(py)), 2)
        
        # 绘制车辆
        draw_truck(screen, env.unwrapped.truck, SCALE)
        
        # 绘制状态信息
        status_text = f"Step: {step}  Reward: {total_reward:.1f}  Steering: {u:.1f} deg"
        status_surface = small_font.render(status_text, True, WHITE)
        screen.blit(status_surface, (20, 20))
        
        pos_text = f"Pos: ({env.unwrapped.truck.x_m:.2f}, {env.unwrapped.truck.y_m:.2f}) m"
        pos_surface = small_font.render(pos_text, True, WHITE)
        screen.blit(pos_surface, (20, 50))
        
        heading_deg = np.degrees(env.unwrapped.truck.h_rad)
        g_deg = np.degrees(env.unwrapped.truck.g_rad)
        heading_text = f"Heading: {heading_deg:.1f} deg  Articulation: {g_deg:.1f} deg"
        heading_surface = small_font.render(heading_text, True, WHITE)
        screen.blit(heading_surface, (20, 80))
        
        dist_to_goal = np.sqrt((env.unwrapped.truck.x_m - GOAL_X)**2 + (env.unwrapped.truck.y_m - GOAL_Y)**2)
        dist_text = f"Distance to goal: {dist_to_goal:.2f} m"
        dist_surface = small_font.render(dist_text, True, WHITE)
        screen.blit(dist_surface, (20, 110))
        
        # 绘制 ADP 增益信息
        k_text = f"ADP Gain K: [{adp.K[0]:.3f}, {adp.K[1]:.3f}, {adp.K[2]:.3f}]"
        k_surface = small_font.render(k_text, True, BLUE)
        screen.blit(k_surface, (20, HEIGHT - 30))
        
        pygame.display.flip()
        clock.tick(60)
        step += 1
        
        if terminated or truncated:
            success = info.get('succeeded', False)
            result_text = "SUCCESS!" if success else "FAILED"
            result_color = GREEN if success else RED
            result_surface = font.render(result_text, True, result_color)
            screen.blit(result_surface, (WIDTH // 2 - 80, HEIGHT // 2))
            pygame.display.flip()
            pygame.time.wait(3000)
            break
    
    print(f"\nEpisode ended at step {step}")
    print(f"  Success: {success}")
    print(f"  Collided: {info.get('collided', False)}")
    print(f"  Jackknifed: {info.get('jackknifed', False)}")
    print(f"  Total Reward: {total_reward:.2f}")
    
    env.close()
    pygame.quit()
    
    return success


if __name__ == '__main__':
    print("启动 ADP 可视化测试...")
    print("按 ESC 退出")
    success = run_adp_visual(steps=2000)
    print(f"\n测试结果: {'成功' if success else '失败'}")