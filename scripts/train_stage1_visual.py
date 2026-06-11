#!/usr/bin/env python3
"""
Stage 1 可视化训练脚本 - 短距离直线倒车
目的：在保持铰接角稳定的前提下，学习距离控制
距离：3米
"""
import pygame
import numpy as np
import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from map.dual_lane_road import DualLaneRoad
from config.scene_config import (
    SCALE, LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH, DT,
    MAX_STEERING_DEG, MAX_STEERING_RATE,
)
from config.scene_config import CURRICULUM_STAGES
from render.render_utils import (
    WHITE, BLACK, RED, GREEN, BLUE,
    draw_truck as _draw_truck_fn,
    draw_trajectory as _draw_trajectory_fn,
)

pygame.init()

USE_FULLSCREEN = True

if USE_FULLSCREEN:
    info = pygame.display.Info()
    WIDTH, HEIGHT = info.current_w, info.current_h
    print(f"使用全屏模式，屏幕分辨率: {WIDTH}x{HEIGHT}")
else:
    WIDTH, HEIGHT = 1200, 900

screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF)
pygame.display.set_caption("Stage 1: 短距离直线倒车")

font = pygame.font.SysFont(None, 28)

stage_config = CURRICULUM_STAGES[1]
GOAL_X = stage_config['goal_x']
GOAL_Y = stage_config['goal_y']
START_X = stage_config['start_x']
START_Y = stage_config['start_y']
MAX_STEPS = stage_config['max_steps']
GOAL_TOL = stage_config['goal_tol']
MAX_G_RAD = stage_config['max_g_rad']

print(f"\n{'='*60}")
print(f"Stage 1: {stage_config['description']}")
print(f"起点: ({START_X}, {START_Y}) -> Y较小（屏幕上方）")
print(f"目标: ({GOAL_X}, {GOAL_Y}) -> Y较大（屏幕下方）")
print(f"距离: {np.sqrt((GOAL_X-START_X)**2 + (GOAL_Y-START_Y)**2):.1f}m")
print(f"最大步数: {MAX_STEPS}")
print(f"目标容差: {GOAL_TOL}m, 最大铰接角: {np.rad2deg(MAX_G_RAD):.1f}°")
print(f"注意: Y轴向下，倒车从小Y向大Y移动（向下）")
print(f"{'='*60}\n")


class SimpleTruckEnv:
    def __init__(self):
        self.truck = TruckTrailerOnAxle()
        self.road = DualLaneRoad(
            center_x=WIDTH / (2 * SCALE),
            y_start=0,
            y_end=HEIGHT / SCALE,
            left_lane_width=LEFT_LANE_WIDTH,
            right_lane_width=RIGHT_LANE_WIDTH,
            scale=SCALE,
            road_id=1
        )
        self.reset()
        self.trajectory = []
        self.prev_g = 0
        self.prev_dist = 0
        
    def reset(self):
        init_g = np.random.uniform(-0.03, 0.03)
        init_h = -np.pi/2 + np.random.uniform(-0.03, 0.03)
        self.truck.set_state([START_X, START_Y, init_h, init_g])
        self.truck.v_mps = -1.0
        self.truck.s_deg = 0
        self.trajectory = []
        self.step_count = 0
        self.prev_g = init_g
        self.prev_dist = np.sqrt((START_X - GOAL_X)**2 + (START_Y - GOAL_Y)**2)
        return self._get_state()
    
    def _get_state(self):
        rel_x = self.truck.x_m - GOAL_X
        rel_y = self.truck.y_m - GOAL_Y
        return np.array([
            rel_x, rel_y,
            np.cos(self.truck.h_rad), np.sin(self.truck.h_rad),
            np.cos(self.truck.g_rad), np.sin(self.truck.g_rad),
            self.truck.v_mps / 2.0,
            self.truck.g_rad / (np.pi/2),
            self.truck.s_deg / MAX_STEERING_DEG,
        ])
    
    def step(self, action):
        target_steering = action[0] * MAX_STEERING_DEG
        
        max_steering_delta = MAX_STEERING_RATE * DT
        new_steering = np.clip(
            target_steering,
            self.truck.s_deg - max_steering_delta,
            self.truck.s_deg + max_steering_delta
        )
        self.truck.set_steering(new_steering)
        self.truck.drive(DT)
        
        self.trajectory.append((self.truck.x_m * SCALE, self.truck.y_m * SCALE))
        self.step_count += 1
        
        dist = np.sqrt((self.truck.x_m - GOAL_X)**2 + (self.truck.y_m - GOAL_Y)**2)
        
        reward = 0.0
        
        dist_delta = self.prev_dist - dist
        reward += dist_delta * 15.0
        
        reward -= abs(self.truck.g_rad) * 4.5
        
        dg = (self.truck.g_rad - self.prev_g) / DT
        reward -= abs(dg) * 2.8
        
        heading_error = abs(self.truck.h_rad - (-np.pi/2))
        reward -= heading_error * 2.0
        
        succeeded = dist < GOAL_TOL and abs(self.truck.g_rad) < MAX_G_RAD
        jackknifed = abs(self.truck.g_rad) > np.deg2rad(85)
        
        if succeeded:
            reward += 120.0
        if jackknifed:
            reward -= 500.0
        
        done = succeeded or jackknifed or self.step_count >= MAX_STEPS
        
        self.prev_g = self.truck.g_rad
        self.prev_dist = dist
        
        info = {
            'succeeded': succeeded,
            'jackknifed': jackknifed,
            'dist': dist,
            'g_rad': self.truck.g_rad,
        }
        
        return self._get_state(), reward, done, info


def random_policy(state):
    return np.array([np.random.uniform(-0.3, 0.3)])


def draw_info_panel(surface, env, episode, total_reward, success_count, total_episodes):
    success_rate = 100*success_count/max(1,total_episodes)
    status_color = GREEN if success_rate >= 80 else RED
    
    lines = [
        f"Stage 1: 短距离直线倒车（3m）",
        f"",
        f"Episode: {episode}  成功率: {success_rate:.1f}% (目标≥80%)",
        f"步数: {env.step_count}/{MAX_STEPS}  奖励: {total_reward:.1f}",
        f"",
        f"位置误差: {np.sqrt((env.truck.x_m-GOAL_X)**2 + (env.truck.y_m-GOAL_Y)**2):.2f}m (容差{GOAL_TOL}m)",
        f"铰接角: {np.rad2deg(env.truck.g_rad):.1f}° (目标<{np.rad2deg(MAX_G_RAD):.1f}°)",
        f"",
        f"✓ 成功: {success_count}  ✗ 失败: {total_episodes-success_count}",
        f"",
        f"按 R 重置 | ESC 退出",
    ]
    
    for i, line in enumerate(lines):
        color = status_color if i == 0 else (GREEN if '✓' in line else WHITE)
        text = font.render(line, True, color)
        surface.blit(text, (10, 10 + i * 30))


def main():
    env = SimpleTruckEnv()
    clock = pygame.time.Clock()
    
    episode = 0
    success_count = 0
    total_episodes = 0
    
    state = env.reset()
    done = False
    total_reward = 0
    
    print("开始训练循环...")
    print("按 ESC 退出")
    print("按 R 手动重置\n")
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    state = env.reset()
                    done = False
                    total_reward = 0
                    episode += 1
        
        if not done:
            action = random_policy(state)
            state, reward, done, info = env.step(action)
            total_reward += reward
            
            if info['succeeded']:
                success_count += 1
                print(f"✓ Episode {episode+1}: 成功! 奖励={total_reward:.1f}, 步数={env.step_count}, 铰接角={np.rad2deg(info['g_rad']):.1f}°")
            elif info['jackknifed']:
                print(f"✗ Episode {episode+1}: Jackknife! 铰接角={np.rad2deg(info['g_rad']):.1f}°")
            elif env.step_count >= MAX_STEPS:
                print(f"✗ Episode {episode+1}: 超时! 距离={info['dist']:.2f}m, 铰接角={np.rad2deg(info['g_rad']):.1f}°")
            
            if done:
                total_episodes += 1
                episode += 1
                state = env.reset()
                done = False
                total_reward = 0
        
        screen.fill(BLACK)
        env.road.draw(screen)
        
        if len(env.trajectory) > 1:
            _draw_trajectory_fn(screen, env.trajectory[-500:])
        
        _draw_truck_fn(screen, env.truck, SCALE)
        
        goal_x_px = GOAL_X * SCALE
        goal_y_px = GOAL_Y * SCALE
        pygame.draw.circle(screen, GREEN, (int(goal_x_px), int(goal_y_px)), int(GOAL_TOL * SCALE), 2)
        
        draw_info_panel(screen, env, episode, total_reward, success_count, total_episodes)
        
        pygame.display.flip()
        clock.tick(60)
    
    print(f"\n训练结束!")
    print(f"总Episode: {total_episodes}")
    print(f"成功次数: {success_count}")
    print(f"成功率: {100*success_count/max(1,total_episodes):.1f}%")
    
    pygame.quit()


if __name__ == '__main__':
    main()
