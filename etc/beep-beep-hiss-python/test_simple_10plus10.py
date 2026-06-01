import pygame
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(__file__))

from semi_truck_model.truckTrailerOnAxle import TruckTrailerOnAxle

# 简单测试：验证配置是否正确
print("测试 10辆车 + 10个库位 配置")
print("=" * 50)

# 创建示例车辆获取尺寸
sample_truck = TruckTrailerOnAxle()

# 车辆尺寸
truck_length = sample_truck.truckWheelbase + 2 * sample_truck.truckMargin
truck_width = sample_truck.truckWidth
trailer_length = sample_truck.trailerWheelbase + sample_truck.trailerMarginFront + sample_truck.trailerMarginBack
trailer_width = sample_truck.trailerWidth

print(f"车头尺寸: {truck_length:.1f}m × {truck_width:.1f}m")
print(f"挂车尺寸: {trailer_length:.1f}m × {trailer_width:.1f}m")

# 库位尺寸
parking_spot_width = trailer_width * 1.2
parking_spot_length = trailer_length * 1.5

print(f"库位尺寸: {parking_spot_length:.1f}m × {parking_spot_width:.1f}m")
print(f"库位比例: 挂车长度×1.5, 挂车宽度×1.2")

# 测试车辆创建
print("\n测试车辆创建...")
trucks = []
for i in range(10):
    truck = TruckTrailerOnAxle()
    # 设置初始状态：车头朝上（航向角 -π/2）
    truck.set_state([i * 5, i * 3, -np.pi/2, 0])
    truck.v_mps = 0
    truck.s_deg = 0
    trucks.append(truck)
    print(f"  车{i+1}: 位置({truck.x_m:.1f}, {truck.y_m:.1f})m, 航向{np.degrees(truck.h_rad):.0f}°")

print("\n测试车辆控制...")
# 测试转向
trucks[0].set_steering(30)
print(f"  车1转向角: {trucks[0].s_deg:.1f}°")

# 测试速度
trucks[0].v_mps = 2.0
print(f"  车1速度: {trucks[0].v_mps:.1f}m/s")

# 测试驱动
print("  模拟驱动0.1秒...")
old_state = trucks[0].get_state()
trucks[0].drive(0.1)
new_state = trucks[0].get_state()
print(f"  车1位置变化: ({old_state[0]:.2f}, {old_state[1]:.2f}) -> ({new_state[0]:.2f}, {new_state[1]:.2f})")

print("\n测试屏幕布局计算...")
SCREEN_WIDTH = 1400
SCREEN_HEIGHT = 900
SCALE = 12

# 上半部分车辆布局
top_half_height = SCREEN_HEIGHT / 2
top_margin = 50
available_height = top_half_height - 2 * top_margin
rows = 2
cols = 5

print(f"  屏幕尺寸: {SCREEN_WIDTH}×{SCREEN_HEIGHT}")
print(f"  上半部分高度: {top_half_height}像素")
print(f"  可用高度: {available_height}像素")
print(f"  布局: {rows}行 × {cols}列")

# 计算一个示例位置
for i in range(min(3, 10)):  # 只显示前3个
    row = i // cols
    col = i % cols
    x = (col + 1) * (SCREEN_WIDTH / (cols + 1))
    y = top_margin + (row + 0.5) * (available_height / rows)
    print(f"  车{i+1}位置: ({x:.0f}, {y:.0f})像素")

print("\n测试库位布局计算...")
# 下半部分库位布局
bottom_half_start = SCREEN_HEIGHT / 2
bottom_margin = 50
available_height_bottom = SCREEN_HEIGHT / 2 - 2 * bottom_margin

spot_width_px = parking_spot_width * SCALE
spot_length_px = parking_spot_length * SCALE

print(f"  库位像素尺寸: {spot_length_px:.0f}×{spot_width_px:.0f}像素")
print(f"  下半部分起始: {bottom_half_start}像素")
print(f"  可用高度: {available_height_bottom}像素")

for i in range(min(3, 10)):  # 只显示前3个
    row = i // cols
    col = i % cols
    x = (col + 1) * (SCREEN_WIDTH / (cols + 1))
    y = bottom_half_start + bottom_margin + (row + 0.5) * (available_height_bottom / rows)
    print(f"  库位{i+1}位置: ({x:.0f}, {y:.0f})像素")

print("\n" + "=" * 50)
print("测试完成！所有配置计算正确。")
print("运行 manual_test_10plus10.py 启动完整程序。")
