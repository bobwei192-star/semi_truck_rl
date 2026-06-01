import numpy as np
from semi_truck_model.truckTrailerOnAxle import TruckTrailerOnAxle

truck = TruckTrailerOnAxle()
SCALE = 20

# 测试大角度
test_angles = [0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi, 10*np.pi, 100*np.pi]

for angle in test_angles:
    truck.set_state([10, 10, angle, 0])
    
    x_px = truck.x_m * SCALE
    y_px = truck.y_m * SCALE
    
    # 计算连接点
    truck_total_length = (truck.truckWheelbase + 2 * truck.truckMargin) * SCALE
    connection_offset_x = -truck_total_length/2 + truck.truckMargin * SCALE
    connection_offset_y = 0
    
    conn_x = x_px + (connection_offset_x * np.cos(truck.h_rad) - connection_offset_y * np.sin(truck.h_rad))
    conn_y = y_px + (connection_offset_x * np.sin(truck.h_rad) + connection_offset_y * np.cos(truck.h_rad))
    
    print(f"角度: {angle:.2f}弧度 ({np.degrees(angle):.1f}度)")
    print(f"  cos(h_rad): {np.cos(truck.h_rad):.6f}, sin(h_rad): {np.sin(truck.h_rad):.6f}")
    print(f"  连接点位置: ({conn_x:.2f}, {conn_y:.2f})")
    print()
    
# 测试数值稳定性
print("测试数值稳定性:")
truck.set_state([1000, 1000, 1000*np.pi, 0])
x_px = truck.x_m * SCALE
y_px = truck.y_m * SCALE
print(f"大位置: x_m={truck.x_m}, y_m={truck.y_m}")
print(f"像素位置: x_px={x_px}, y_px={y_px}")
print(f"cos(1000π)={np.cos(truck.h_rad):.15f}, sin(1000π)={np.sin(truck.h_rad):.15f}")
