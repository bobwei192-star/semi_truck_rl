# -*- coding: utf-8 -*-
"""倒车运动学符号探针: 确认 v<0 时方向盘符号对 x 位移和铰接角 g 的影响"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from config.scene_config import DT


def probe(swa_deg, steps=20):
    truck = TruckTrailerOnAxle()
    truck.set_state([56.25, 9.91, -np.pi / 2, 0.0])
    truck.v_mps = -1.0
    truck.set_steering(swa_deg)
    for _ in range(steps):
        truck.drive(DT)
    return truck.x_m, truck.y_m, np.rad2deg(truck.h_rad), np.rad2deg(truck.g_rad)


for swa in [-35, -20, -10, -5, 0, 5, 10, 20, 35]:
    x, y, h, g = probe(swa)
    print(f"swa={swa:+4d}°  x={x:8.3f}  y={y:7.3f}  h={h:8.2f}°  g={g:+7.2f}°")
