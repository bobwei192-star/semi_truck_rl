import pygame
import numpy as np
import sys
import os
import time

# Add current directory to path to import modules
sys.path.append(os.path.dirname(__file__))

from truckTrailerOnAxle import TruckTrailerOnAxle
from dual_lane_road import DualLaneRoad

# Initialize Pygame
pygame.init()

# 屏幕配置选项
USE_FULLSCREEN = True  # 启用全屏，让窗口平铺屏幕

if USE_FULLSCREEN:
    # 获取当前屏幕分辨率
    info = pygame.display.Info()
    WIDTH, HEIGHT = info.current_w, info.current_h
    print(f"使用全屏模式，屏幕分辨率: {WIDTH}x{HEIGHT}")
else:
    # 使用窗口模式
    WINDOW_WIDTH = 800      # 窗口宽度
    WINDOW_HEIGHT = 600     # 窗口高度
    WIDTH, HEIGHT = WINDOW_WIDTH, WINDOW_HEIGHT
    print(f"使用窗口模式，窗口尺寸: {WIDTH}x{HEIGHT}")

# 使用软件渲染，避免OpenGL硬件加速问题
os.environ['SDL_VIDEODRIVER'] = 'x11'
os.environ['MESA_GL_VERSION_OVERRIDE'] = '3.3'
os.environ['MESA_GLSL_VERSION_OVERRIDE'] = '330'

# 设置显示模式（全屏或窗口）
if USE_FULLSCREEN:
    # 全屏模式
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF)
    print(f"使用全屏模式: {WIDTH}x{HEIGHT} 像素")
else:
    # 窗口模式
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
    print(f"使用窗口模式: {WIDTH}x{HEIGHT} 像素")

pygame.display.set_caption("Semi-Truck Manual Control")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
GRAY = (200, 200, 200)
# 新增颜色：车头边框红色，挂车边框棕红色
TRUCK_BORDER = (255, 0, 0)       # 车头边框：红色
TRAILER_BORDER = (165, 42, 42)   # 挂车边框：棕红色 (Brown Red)

# Scale factor for visualization (pixels per meter)
SCALE = 20

# Create truck
truck = TruckTrailerOnAxle()
INIT_X = WIDTH / (2 * SCALE) - 2
INIT_Y = HEIGHT / (2 * SCALE) - 29
INIT_H = -np.pi / 2
INIT_G = 0.0
truck.set_state([INIT_X, INIT_Y, INIT_H, INIT_G])

road = DualLaneRoad(
    center_x=WIDTH / (2 * SCALE),
    y_start=0,
    y_end=HEIGHT / SCALE,
    lane_width=4.5,
    scale=SCALE,
    road_id=1
)
print(f"道路信息: {road}")
print(f"道路边界: 左={road.get_road_boundaries()[0]:.2f}m, 右={road.get_road_boundaries()[1]:.2f}m")
print(f"几何线段: {road.get_all_lines_info()}")

start_x = INIT_X
start_y = INIT_Y
road.set_start_position(start_x, start_y)

# Control parameters
steering_increment = 2.0  # degrees per key press (increased for better responsiveness)
speed_increment = 1.0     # m/s per key press (increased for better responsiveness)
max_speed = 5.0           # maximum speed

# Font for displaying information
font = pygame.font.SysFont(None, 24)

# 日志记录
log_file = open("truck_position_log.txt", "w")
log_file.write("时间戳,帧号,x_m,y_m,h_rad(度),g_rad(度),v_mps,s_deg,x_px,y_px,dt,触发环绕\n")
frame_count = 0
start_time = time.time()

def log_position(frame, dt, triggered_wrap=False):
    """记录车辆位置信息"""
    global frame_count
    timestamp = time.time() - start_time
    x_px = truck.x_m * SCALE
    y_px = truck.y_m * SCALE
    
    log_line = f"{timestamp:.3f},{frame},{truck.x_m:.6f},{truck.y_m:.6f},{np.degrees(truck.h_rad):.6f},{np.degrees(truck.g_rad):.6f},{truck.v_mps:.6f},{truck.s_deg:.6f},{x_px:.2f},{y_px:.2f},{dt:.6f},{triggered_wrap}\n"
    log_file.write(log_line)
    
    # 每10帧也在控制台输出一次
    if frame % 10 == 0:
        print(f"帧{frame}: 位置=({truck.x_m:.2f}, {truck.y_m:.2f})m, 像素=({x_px:.1f}, {y_px:.1f}), 速度={truck.v_mps:.2f}m/s, 航向={np.degrees(truck.h_rad):.1f}°")

def draw_truck(surface, truck):
    """Draw the truck and trailer on the surface with anti-aliasing"""
    # Convert from meters to pixels
    x_px = truck.x_m * SCALE
    y_px = truck.y_m * SCALE
    
    # Truck dimensions (including margins)
    truck_total_length = (truck.truckWheelbase + 2 * truck.truckMargin) * SCALE
    truck_total_width = truck.truckWidth * SCALE
    
    # Trailer dimensions (including margins)
    trailer_total_length = (truck.trailerWheelbase + truck.trailerMarginFront + truck.trailerMarginBack) * SCALE
    trailer_total_width = truck.trailerWidth * SCALE
    
    # 使用抗锯齿技术：创建更大的表面进行绘制，然后缩小
    scale_factor = 2  # 2倍超采样
    truck_surface_large = pygame.Surface((truck_total_length * scale_factor, truck_total_width * scale_factor), pygame.SRCALPHA)
    # 在更大的表面上绘制更粗的边框
    border_width_large = 4 * scale_factor
    pygame.draw.rect(truck_surface_large, TRUCK_BORDER, truck_surface_large.get_rect(), width=border_width_large)
    
    trailer_surface_large = pygame.Surface((trailer_total_length * scale_factor, trailer_total_width * scale_factor), pygame.SRCALPHA)
    pygame.draw.rect(trailer_surface_large, TRAILER_BORDER, trailer_surface_large.get_rect(), width=border_width_large)
    
    # 缩小到原始尺寸（使用平滑缩放）
    truck_surface = pygame.transform.smoothscale(truck_surface_large, (truck_total_length, truck_total_width))
    trailer_surface = pygame.transform.smoothscale(trailer_surface_large, (trailer_total_length, trailer_total_width))
    
    # 使用抗锯齿旋转
    rotated_truck = pygame.transform.rotate(truck_surface, -np.degrees(truck.h_rad))
    rotated_trailer = pygame.transform.rotate(trailer_surface, -np.degrees(truck.h_rad + truck.g_rad))
    
    # Calculate connection point (fifth wheel) relative to truck center
    # Connection point is at the rear of the truck
    connection_offset_x = -truck_total_length/2 + truck.truckMargin * SCALE
    connection_offset_y = 0
    
    # Apply truck rotation to connection offset
    conn_x = x_px + (connection_offset_x * np.cos(truck.h_rad) - connection_offset_y * np.sin(truck.h_rad))
    conn_y = y_px + (connection_offset_x * np.sin(truck.h_rad) + connection_offset_y * np.cos(truck.h_rad))
    
    # Calculate trailer center relative to connection point
    # Trailer's front is at trailerMarginFront from connection point
    trailer_center_offset_x = (truck.trailerMarginFront - (truck.trailerWheelbase + truck.trailerMarginBack)/2) * SCALE
    trailer_center_offset_y = 0
    
    # Apply trailer rotation to center offset
    trailer_center_x = conn_x + (trailer_center_offset_x * np.cos(truck.h_rad + truck.g_rad) - 
                                 trailer_center_offset_y * np.sin(truck.h_rad + truck.g_rad))
    trailer_center_y = conn_y + (trailer_center_offset_x * np.sin(truck.h_rad + truck.g_rad) + 
                                 trailer_center_offset_y * np.cos(truck.h_rad + truck.g_rad))
    
    # Get rects for positioning
    truck_rect = rotated_truck.get_rect(center=(x_px, y_px))
    trailer_rect = rotated_trailer.get_rect(center=(trailer_center_x, trailer_center_y))
    
    # Draw on screen
    surface.blit(rotated_truck, truck_rect)
    surface.blit(rotated_trailer, trailer_rect)
    
    # Draw wheels
    wheel_length = truck.tireLen * SCALE
    wheel_width = truck.tireWidth * SCALE
    
    # Front wheels (steering)
    front_wheel_angle = truck.s_deg
    for side in [-1, 1]:
        # Wheel position relative to truck center
        # Based on MATLAB version: wheels are at truckWheelbase distance from origin
        # and spaced by truckWidth/2 on each side
        wheel_offset_x = (truck.truckWheelbase + truck.truckMargin) * SCALE - truck_total_length/2
        wheel_offset_y = side * truck_total_width/2  # 修正：使用车宽的一半，而不是三分之一
        
        # 优化轮子绘制：使用抗锯齿
        wheel_scale_factor = 2
        wheel_surface_large = pygame.Surface((wheel_length * wheel_scale_factor, wheel_width * wheel_scale_factor), pygame.SRCALPHA)
        pygame.draw.rect(wheel_surface_large, BLACK, wheel_surface_large.get_rect())
        wheel_surface = pygame.transform.smoothscale(wheel_surface_large, (wheel_length, wheel_width))
        rotated_wheel = pygame.transform.rotate(wheel_surface, -np.degrees(truck.h_rad) - front_wheel_angle)
        
        wheel_pos = (
            x_px + (wheel_offset_x * np.cos(truck.h_rad) - wheel_offset_y * np.sin(truck.h_rad)),
            y_px + (wheel_offset_x * np.sin(truck.h_rad) + wheel_offset_y * np.cos(truck.h_rad))
        )
        wheel_rect = rotated_wheel.get_rect(center=wheel_pos)
        surface.blit(rotated_wheel, wheel_rect)
    
        # 绘制第五轮位置（红色圆点）
        fifth_wheel_radius = 5
        pygame.draw.circle(surface, RED, (int(x_px), int(y_px)), fifth_wheel_radius)

def draw_info(surface, truck):
    """Display truck information on screen"""
    info_lines = [
        f"Position: ({truck.x_m:.2f}, {truck.y_m:.2f}) m",
        f"Heading: {np.degrees(truck.h_rad):.1f} deg",
        f"Trailer angle: {np.degrees(truck.g_rad):.1f} deg",
        f"Speed: {truck.v_mps:.2f} m/s",
        f"Steering: {truck.s_deg:.1f} deg",
        f"Frame: {frame_count}",
        "",
        "Controls:",
        "W/Up:   Accelerate",
        "S/Down: Reverse",
        "A/Left: Turn Left",
        "D/Right: Turn Right",
        "LMB:    Turn Left",
        "RMB:    Turn Right",
        "R:      Reset Position",
        "ESC:    Exit",
        "L:      Save log and exit"
    ]
    
    for i, line in enumerate(info_lines):
        text = font.render(line, True, BLACK)
        surface.blit(text, (10, 10 + i * 25))

def main():
    global frame_count
    clock = pygame.time.Clock()
    running = True
    
    print("开始记录车辆位置日志...")
    print(f"屏幕尺寸: {WIDTH}x{HEIGHT} 像素, SCALE={SCALE} 像素/米")
    print(f"世界坐标范围: X: 0 到 {WIDTH/SCALE:.2f} 米, Y: 0 到 {HEIGHT/SCALE:.2f} 米")
    print(f"中心位置: ({WIDTH/(2*SCALE):.2f}, {HEIGHT/(2*SCALE):.2f}) 米")
    print(f"初始位置: ({truck.x_m:.2f}, {truck.y_m:.2f}) 米")
    print(f"环绕边界: X < {-1.0} 米或 X > {WIDTH/SCALE + 1.0:.2f} 米时触发 (容差={1.0}米)")
    print(f"具体条件: x_px < {-1.0*SCALE} 或 x_px > {WIDTH + 1.0*SCALE}")
    print(f"日志文件: truck_position_log.txt")
    print("按ESC退出，按L保存日志并退出")
    
    while running:
        dt = clock.tick(60) / 1000.0  # Convert to seconds
        frame_count += 1
        
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_l:
                    print("保存日志并退出...")
                    running = False
        
        # Handle continuous key presses
        keys = pygame.key.get_pressed()
        
        # Speed control (W/Up for forward, S/Down for reverse)
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            truck.v_mps = min(truck.v_mps + speed_increment * dt * 10, max_speed)
        elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
            truck.v_mps = max(truck.v_mps - speed_increment * dt * 10, -max_speed)
        else:
            # Gradually slow down if no speed keys are pressed
            if truck.v_mps > 0:
                truck.v_mps = max(truck.v_mps - speed_increment * dt * 5, 0)
            elif truck.v_mps < 0:
                truck.v_mps = min(truck.v_mps + speed_increment * dt * 5, 0)
        
        # Steering control (A/Left for left, D/Right for right)
        # Fix: Swap A and D to correct reversed steering
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            truck.set_steering(truck.s_deg - steering_increment * dt * 30)  # Left turn
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            truck.set_steering(truck.s_deg + steering_increment * dt * 30)  # Right turn
        else:
            # Gradually return steering to center
            if truck.s_deg > 0:
                truck.set_steering(max(truck.s_deg - steering_increment * dt * 20, 0))
            elif truck.s_deg < 0:
                truck.set_steering(min(truck.s_deg + steering_increment * dt * 20, 0))
        
        # Mouse control (left button for left turn, right button for right turn)
        mouse_buttons = pygame.mouse.get_pressed()
        if mouse_buttons[0]:  # Left mouse button
            truck.set_steering(truck.s_deg - steering_increment * dt * 30)  # Left turn
        if mouse_buttons[2]:  # Right mouse button
            truck.set_steering(truck.s_deg + steering_increment * dt * 30)  # Right turn
        
        # Reset position with R key
        if keys[pygame.K_r]:
            truck.set_state([INIT_X, INIT_Y, INIT_H, INIT_G])
            truck.v_mps = 0
            truck.s_deg = 0
            print(f"Frame {frame_count}: Reset to start ({INIT_X:.2f}, {INIT_Y:.2f})m")
        
        # 记录驱动前的位置
        log_position(frame_count, dt, triggered_wrap=False)
        
        # Update truck position
        truck.drive(dt)
        
        # Wrap around screen edges with tolerance
        # 使用更大的容差和更智能的环绕逻辑
        x_tolerance = 2.0  # X方向容差增加到2.0米
        y_tolerance = 2.0  # Y方向容差增加到2.0米
        triggered_wrap = False
        
        # 调试信息：打印当前位置
        if frame_count % 30 == 0:  # 每30帧打印一次
            print(f"帧{frame_count}: 位置=({truck.x_m:.3f}, {truck.y_m:.3f})m, 像素=({truck.x_m*SCALE:.1f}, {truck.y_m*SCALE:.1f}), 速度={truck.v_mps:.2f}m/s")
        
        # 检查X方向环绕
        x_px = truck.x_m * SCALE
        # 调试：打印关键值
        if frame_count % 30 == 0 or abs(x_px - WIDTH) < 100:
            print(f"帧{frame_count}: 环绕检查: x_px={x_px:.1f}, WIDTH={WIDTH}, 右边界={WIDTH + x_tolerance*SCALE:.1f}, 条件: {x_px:.1f} > {WIDTH + x_tolerance*SCALE:.1f} = {x_px > WIDTH + x_tolerance*SCALE}")
        
        # 改进的环绕逻辑：使用更宽松的条件
        if x_px < -x_tolerance * SCALE:
            old_x = truck.x_m
            truck.x_m = WIDTH / SCALE
            triggered_wrap = True
            print(f"帧{frame_count}: X方向环绕(左侧)! {old_x:.3f}m -> {truck.x_m:.3f}m (因为 {x_px:.1f} < {-x_tolerance*SCALE})")
        elif x_px > WIDTH + x_tolerance * SCALE:
            old_x = truck.x_m
            truck.x_m = 0
            triggered_wrap = True
            print(f"帧{frame_count}: X方向环绕(右侧)! {old_x:.3f}m -> {truck.x_m:.3f}m (因为 {x_px:.1f} > {WIDTH + x_tolerance*SCALE})")
        # 添加额外检查：如果车辆非常接近边界，也打印信息
        elif x_px > WIDTH - 10:  # 距离右边界10像素内
            print(f"帧{frame_count}: 接近右边界: x_px={x_px:.1f}, 距离={WIDTH - x_px:.1f}像素")
            
        # 检查Y方向环绕
        y_px = truck.y_m * SCALE
        if y_px < -y_tolerance * SCALE:
            old_y = truck.y_m
            truck.y_m = HEIGHT / SCALE
            triggered_wrap = True
            print(f"帧{frame_count}: Y方向环绕! {old_y:.3f}m -> {truck.y_m:.3f}m (因为 {y_px:.1f} < {-y_tolerance*SCALE})")
        elif y_px > HEIGHT + y_tolerance * SCALE:
            old_y = truck.y_m
            truck.y_m = 0
            triggered_wrap = True
            print(f"帧{frame_count}: Y方向环绕! {old_y:.3f}m -> {truck.y_m:.3f}m (因为 {y_px:.1f} > {HEIGHT + y_tolerance*SCALE})")
        
        # 添加额外检查：如果车辆位置异常，也打印信息
        if truck.x_m < -5 or truck.x_m > WIDTH/SCALE + 5 or truck.y_m < -5 or truck.y_m > HEIGHT/SCALE + 5:
            print(f"帧{frame_count}: 警告! 车辆位置异常: ({truck.x_m:.3f}, {truck.y_m:.3f})m")
        
        # 如果触发了环绕，再记录一次
        if triggered_wrap:
            log_position(frame_count, dt, triggered_wrap=True)
        
        # Draw everything
        screen.fill(WHITE)
        
        road.draw(screen)
        
        # Draw truck and info
        draw_truck(screen, truck)
        draw_info(screen, truck)
        
        # Update display
        pygame.display.flip()
    
    # 关闭日志文件
    log_file.close()
    print(f"日志已保存到 truck_position_log.txt，共记录 {frame_count} 帧")
    pygame.quit()

if __name__ == "__main__":
    main()
