import pygame
import numpy as np
import sys
import os
import time

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from map.dual_lane_road import DualLaneRoad
from config.scene_config import (
    SCALE, LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH, START_HEADING_OFFSET_RAD, START_G_INIT_RAD,
    START_X, START_Y,
    INIT_HEADING_RAD, HEADING_NOISE_RAD, INIT_G_RAD, G_NOISE_RAD,
)
from render.render_utils import (
    WHITE, BLACK, RED, GREEN,
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
    WINDOW_WIDTH = 800
    WINDOW_HEIGHT = 600
    WIDTH, HEIGHT = WINDOW_WIDTH, WINDOW_HEIGHT
    print(f"使用窗口模式，窗口尺寸: {WIDTH}x{HEIGHT}")

os.environ['SDL_VIDEODRIVER'] = 'x11'
os.environ['MESA_GL_VERSION_OVERRIDE'] = '3.3'
os.environ['MESA_GLSL_VERSION_OVERRIDE'] = '330'

if USE_FULLSCREEN:
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF)
    print(f"使用全屏模式: {WIDTH}x{HEIGHT} 像素")
else:
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
    print(f"使用窗口模式: {WIDTH}x{HEIGHT} 像素")

pygame.display.set_caption("Semi-Truck Manual Control")

truck = TruckTrailerOnAxle()
INIT_X = START_X
INIT_Y = START_Y
INIT_H = INIT_HEADING_RAD + START_HEADING_OFFSET_RAD
INIT_G = INIT_G_RAD + START_G_INIT_RAD
truck.set_state([INIT_X, INIT_Y, INIT_H, INIT_G])

road = DualLaneRoad(
    center_x=WIDTH / (2 * SCALE),
    y_start=0,
    y_end=HEIGHT / SCALE,
    left_lane_width=LEFT_LANE_WIDTH,
    right_lane_width=RIGHT_LANE_WIDTH,
    scale=SCALE,
    road_id=1
)
print(f"道路边界: 左={road.get_road_boundaries()[0]:.2f}m, 右={road.get_road_boundaries()[1]:.2f}m")

start_x = INIT_X
start_y = INIT_Y
road.set_start_position(start_x, start_y)

steering_increment = 2.0
speed_increment = 1.0
max_speed = 5.0

font = pygame.font.SysFont(None, 24)

trajectory_points = []

log_file = open("truck_position_log.txt", "w")
log_file.write("时间戳,帧号,x_m,y_m,h_rad(度),g_rad(度),v_mps,s_deg,x_px,y_px,dt,触发环绕\n")
frame_count = 0
start_time = time.time()


def log_position(frame, dt, triggered_wrap=False):
    global frame_count
    timestamp = time.time() - start_time
    x_px = truck.x_m * SCALE
    y_px = truck.y_m * SCALE
    log_line = f"{timestamp:.3f},{frame},{truck.x_m:.6f},{truck.y_m:.6f},{np.degrees(truck.h_rad):.6f},{np.degrees(truck.g_rad):.6f},{truck.v_mps:.6f},{truck.s_deg:.6f},{x_px:.2f},{y_px:.2f},{dt:.6f},{triggered_wrap}\n"
    log_file.write(log_line)
    if frame % 10 == 0:
        print(f"帧{frame}: 位置=({truck.x_m:.2f}, {truck.y_m:.2f})m, 航向={np.degrees(truck.h_rad):.1f}°, 速度={truck.v_mps:.2f}m/s")


def draw_info(surface, truck_obj):
    info_lines = [
        f"Position: ({truck_obj.x_m:.2f}, {truck_obj.y_m:.2f}) m",
        f"Heading: {np.degrees(truck_obj.h_rad):.1f} deg",
        f"Trailer angle: {np.degrees(truck_obj.g_rad):.1f} deg",
        f"Speed: {truck_obj.v_mps:.2f} m/s",
        f"Steering: {truck_obj.s_deg:.1f} deg",
        f"Frame: {frame_count}",
        "",
        "Controls:",
        "W/Up:   Accelerate",
        "S/Down: Reverse",
        "A/Left: Turn Left",
        "D/Right: Turn Right",
        "R:      Reset",
        "ESC:    Exit",
    ]
    for i, line in enumerate(info_lines):
        text = font.render(line, True, BLACK)
        surface.blit(text, (10, 10 + i * 25))


def main():
    global frame_count
    clock = pygame.time.Clock()
    running = True

    print(f"屏幕尺寸: {WIDTH}x{HEIGHT} 像素, SCALE={SCALE} 像素/米")
    print(f"初始位置: ({truck.x_m:.2f}, {truck.y_m:.2f}) 米")

    while running:
        dt = clock.tick(60) / 1000.0
        frame_count += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        keys = pygame.key.get_pressed()

        if keys[pygame.K_w] or keys[pygame.K_UP]:
            truck.v_mps = min(truck.v_mps + speed_increment * dt * 10, max_speed)
        elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
            truck.v_mps = max(truck.v_mps - speed_increment * dt * 10, -max_speed)
        else:
            if truck.v_mps > 0:
                truck.v_mps = max(truck.v_mps - speed_increment * dt * 5, 0)
            elif truck.v_mps < 0:
                truck.v_mps = min(truck.v_mps + speed_increment * dt * 5, 0)

        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            truck.set_steering(truck.s_deg - steering_increment * dt * 30)
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            truck.set_steering(truck.s_deg + steering_increment * dt * 30)
        else:
            if truck.s_deg > 0:
                truck.set_steering(max(truck.s_deg - steering_increment * dt * 20, 0))
            elif truck.s_deg < 0:
                truck.set_steering(min(truck.s_deg + steering_increment * dt * 20, 0))

        mouse_buttons = pygame.mouse.get_pressed()
        if mouse_buttons[0]:
            truck.set_steering(truck.s_deg - steering_increment * dt * 30)
        if mouse_buttons[2]:
            truck.set_steering(truck.s_deg + steering_increment * dt * 30)

        if keys[pygame.K_r]:
            truck.set_state([INIT_X, INIT_Y, INIT_H, INIT_G])
            truck.v_mps = 0
            truck.s_deg = 0
            trajectory_points.clear()
            print(f"Frame {frame_count}: 重置到起点 ({INIT_X:.2f}, {INIT_Y:.2f})m")

        log_position(frame_count, dt, triggered_wrap=False)

        truck.drive(dt)

        trajectory_points.append((truck.x_m * SCALE, truck.y_m * SCALE))

        x_tolerance = 2.0
        y_tolerance = 2.0
        triggered_wrap = False

        x_px = truck.x_m * SCALE
        if x_px < -x_tolerance * SCALE:
            truck.x_m = WIDTH / SCALE
            triggered_wrap = True
        elif x_px > WIDTH + x_tolerance * SCALE:
            truck.x_m = 0
            triggered_wrap = True

        y_px = truck.y_m * SCALE
        if y_px < -y_tolerance * SCALE:
            truck.y_m = HEIGHT / SCALE
            triggered_wrap = True
        elif y_px > HEIGHT + y_tolerance * SCALE:
            truck.y_m = 0
            triggered_wrap = True

        if triggered_wrap:
            log_position(frame_count, dt, triggered_wrap=True)

        screen.fill(WHITE)
        road.draw(screen)
        _draw_trajectory_fn(screen, trajectory_points)
        _draw_truck_fn(screen, truck, SCALE)
        draw_info(screen, truck)
        pygame.display.flip()

    log_file.close()
    print(f"日志已保存到 truck_position_log.txt，共记录 {frame_count} 帧")
    pygame.quit()


if __name__ == "__main__":
    main()
