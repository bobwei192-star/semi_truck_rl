# -*- coding: utf-8 -*-
"""挂车中心响应探针: 恒定转角下挂车中心的横向漂移方向与速率"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from config.scene_config import DT

TRAILER_LEN = 11.0


def tc_of(truck):
    th = truck.h_rad + truck.g_rad
    return (truck.x_m - TRAILER_LEN / 2 * np.cos(th),
            truck.y_m - TRAILER_LEN / 2 * np.sin(th))


def probe(swa_deg, steps=100):
    truck = TruckTrailerOnAxle()
    truck.set_state([56.25, 9.91, -np.pi / 2, 0.0])
    truck.v_mps = -1.0
    truck.set_steering(swa_deg)
    for _ in range(steps):
        truck.drive(DT)
    x, y = tc_of(truck)
    return x, y, np.rad2deg(truck.h_rad), np.rad2deg(truck.g_rad)


for swa in [-35, -20, -10, -5, 5, 10, 20, 35]:
    x, y, h, g = probe(swa)
    print(f"swa={swa:+4d}° 100步(5s): tc=({x:8.3f},{y:7.3f})  h={h:8.2f}°  g={g:+7.2f}°  "
          f"tc_dx={x-56.25:+.3f}  tc_dy={y-18.41:+.3f}")
