# -*- coding: utf-8 -*-
"""定符号实验: 内环保持恒定铰接角, 观察挂车中心漂移方向"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from config.scene_config import DT

TRAILER_LEN = 11.0
Lc = 3.0


def tc_of(truck):
    th = truck.h_rad + truck.g_rad
    return (truck.x_m - TRAILER_LEN / 2 * np.cos(th),
            truck.y_m - TRAILER_LEN / 2 * np.sin(th))


def run(g_des_deg, seconds=15):
    truck = TruckTrailerOnAxle()
    truck.set_state([56.25, 9.91, -np.pi / 2, 0.0])
    truck.v_mps = -1.0
    g_des = np.deg2rad(g_des_deg)
    for _ in range(int(seconds / DT)):
        swa = 3.0 * (g_des - truck.g_rad)
        swa = np.clip(swa, -np.deg2rad(35), np.deg2rad(35))
        truck.set_steering(np.rad2deg(swa))
        truck.drive(DT)
    x, y = tc_of(truck)
    return x, y, np.rad2deg(truck.h_rad), np.rad2deg(truck.g_rad)


for g_des in [-15, -10, -5, -2, 0, 2, 5, 10, 15]:
    x, y, h, g = run(g_des)
    print(f"g_des={g_des:+4d}°: tc=({x:8.3f},{y:7.3f})  dx={x-56.25:+7.3f}  "
          f"dy={y-18.41:+7.3f}  h={h:8.2f}°  g={g:+7.2f}°")
