# -*- coding: utf-8 -*-
"""控制器增益网格搜索 (角点约束版)"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from another_solve.parking_three_lines import plan_path, run_once, corners_in_slot


def score(res):
    truck = res['truck']
    corners = truck.get_corner_positions()
    if res['violations'] or not corners_in_slot(corners):
        return None
    # 评分: 角点最小边距越大越好
    x_min = corners[:, 0].min()
    x_max = corners[:, 0].max()
    margin = min(x_min - 56.0, 60.0 - x_max, 69.0 - corners[:, 1].max())
    return margin, abs(np.rad2deg(truck.g_rad)), res['psis'][-1]


if __name__ == '__main__':
    path = plan_path()
    results = []
    for k_psi in [1.0, 2.0, 3.0, 4.0]:
        for k_lat in [0.03, 0.05, 0.07, 0.1]:
            for k_st in [2.0, 3.0, 4.0]:
                for g_max in [5, 8, 10]:
                    try:
                        res = run_once(path, k_psi=k_psi, k_lateral=k_lat,
                                       k_steer=k_st, g_des_max_deg=g_max)
                        s = score(res)
                    except Exception:
                        continue
                    if s:
                        results.append((s[0], s[1], s[2], k_psi, k_lat, k_st, g_max, res['steps']))
    print(f"total=192, corner-safe success={len(results)}")
    results.sort(key=lambda r: (-r[0], r[1]))
    for r in results[:15]:
        print(f"margin={r[0]:.3f}m |g|={r[1]:.2f}° psi_end={np.rad2deg(r[2]):+.2f}° | "
              f"k_psi={r[3]} k_lat={r[4]} k_st={r[5]} g_max={r[6]} steps={r[7]}")
