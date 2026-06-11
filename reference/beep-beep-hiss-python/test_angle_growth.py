import numpy as np
from semi_truck_model.truckTrailerOnAxle import TruckTrailerOnAxle

truck = TruckTrailerOnAxle()
truck.set_state([10, 10, 0, 0])
truck.v_mps = 5.0
truck.set_steering(30)  # 大转向角

print('初始状态:', truck.get_state())
print('初始航向角(度):', np.degrees(truck.h_rad))

for i in range(100):
    truck.drive(0.1)
    if i % 10 == 0:
        print(f'第{i+1}步后: 航向角={np.degrees(truck.h_rad):.2f}°, x={truck.x_m:.2f}, y={truck.y_m:.2f}')
        
# 检查角度是否被归一化
print('\n检查角度值:')
truck.set_state([10, 10, 10*np.pi, 0])  # 10π弧度 = 1800度
print(f'设置航向角为10π ({10*np.pi}弧度, {np.degrees(10*np.pi)}度)')
print(f'实际航向角: {truck.h_rad}弧度, {np.degrees(truck.h_rad)}度')
