import numpy as np
from semi_truck_model.truckTrailerOnAxle import TruckTrailerOnAxle

print("测试角度归一化修复")
print("=" * 50)

truck = TruckTrailerOnAxle()

# 测试大角度归一化
test_angles = [0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi, 10*np.pi, -5*np.pi, 100*np.pi]

for angle in test_angles:
    truck.set_state([0, 0, angle, 0])
    normalized = truck.h_rad
    print(f"输入角度: {angle:.4f}弧度 ({np.degrees(angle):.1f}度)")
    print(f"归一化后: {normalized:.4f}弧度 ({np.degrees(normalized):.1f}度)")
    print(f"是否在[-π,π]范围内: {-np.pi <= normalized <= np.pi}")
    print()

# 测试长时间运行角度增长
print("\n测试长时间运行角度增长")
print("=" * 50)

truck.set_state([0, 0, 0, 0])
truck.v_mps = 2.0
truck.set_steering(10)  # 固定转向角

angles = []
for i in range(100):
    truck.drive(0.1)
    angles.append(truck.h_rad)
    
print(f"100步后航向角: {truck.h_rad:.4f}弧度 ({np.degrees(truck.h_rad):.1f}度)")
print(f"角度范围: [{min(angles):.4f}, {max(angles):.4f}]弧度")
print(f"是否保持在[-π,π]范围内: {all(-np.pi <= a <= np.pi for a in angles)}")

# 测试边界环绕
print("\n测试边界环绕逻辑")
print("=" * 50)

SCALE = 20
WIDTH, HEIGHT = 800, 600

# 模拟接近边界的情况
test_positions = [
    (-0.05, 300, "略超出左边界"),
    (WIDTH/SCALE + 0.05, 300, "略超出右边界"),
    (400, -0.05, "略超出上边界"),
    (400, HEIGHT/SCALE + 0.05, "略超出下边界"),
    (-0.2, 300, "明显超出左边界"),
    (WIDTH/SCALE + 0.2, 300, "明显超出右边界"),
]

for x_m, y_m, desc in test_positions:
    x_px = x_m * SCALE
    print(f"\n{desc}:")
    print(f"  位置: ({x_m:.2f}m, {y_m:.2f}m) = ({x_px:.1f}像素, {y_m*SCALE:.1f}像素)")
    print(f"  是否触发环绕: ", end="")
    
    tolerance = 0.1
    if x_px < -tolerance * SCALE:
        print(f"是 -> 跳到右侧 (x = {WIDTH/SCALE:.1f}m)")
    elif x_px > WIDTH + tolerance * SCALE:
        print(f"是 -> 跳到左侧 (x = 0m)")
    else:
        print("否")
