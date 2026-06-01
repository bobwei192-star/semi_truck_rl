import numpy as np
from semi_truck_model.truckTrailerOnAxle import TruckTrailerOnAxle

truck = TruckTrailerOnAxle()
truck.set_state([10, 10, 0, 0])
truck.v_mps = 5.0
truck.set_steering(10)

print('初始状态:', truck.get_state())

for i in range(10):
    truck.drive(0.1)
    print(f'第{i+1}步后状态:', truck.get_state())
    print(f'  位置变化: x={truck.x_m:.2f}, y={truck.y_m:.2f}')
    
# 测试边界情况
print('\n测试边界情况:')
truck.set_state([100, 100, 0, 0])
truck.v_mps = 10.0
truck.set_steering(0)

for i in range(5):
    truck.drive(0.1)
    print(f'边界测试第{i+1}步: x={truck.x_m:.2f}, y={truck.y_m:.2f}')
