# -*- coding: utf-8 -*-
"""起点随机泛化测试: 当前规划器 vs 自适应规划器

当前 plan_path 的段1是固定斜率直线, 起点偏移大时入库瞬间挂车摆尾出库线.
自适应版 plan_path_adaptive 用五次平滑 S 曲线做横移段 (曲率有界),
并按路径所需曲率自动放宽 g_des_max.
"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import another_solve.parking_three_lines as P
from config.scene_config import GOAL_Y, START_X, START_Y

TR = P.TRAILER_LEN


def smoothstep(s):
    return s * s * s * (10 - 15 * s + 6 * s * s)


def plan_path_adaptive(x0, y0, dock_x=None, join_y=None):
    """起点自适应两段规划: S 曲线横移段 + 垂直入库段

    join_y 默认取 tc_goal_y-6: S 曲线尽量延伸以拉长过渡段 L,
    降低峰值铰接角, 避免车头/挂车角点扫出道路边界.
    """
    if dock_x is None:
        dock_x = (P.SLOT_LEFT + P.SLOT_RIGHT) / 2
    if join_y is None:
        join_y = GOAL_Y + TR / 2 - 0.45 - 6
    tc_start = np.array([x0, y0 + TR / 2])
    tc_goal = np.array([dock_x, GOAL_Y + TR / 2 - 0.45])

    dy = join_y - tc_start[1]
    n = max(40, int(dy * 4))
    ys = np.linspace(tc_start[1], join_y, n, endpoint=False)
    s = (ys - tc_start[1]) / dy
    xs = x0 + (dock_x - x0) * smoothstep(s)
    seg_v = np.linspace(np.array([dock_x, join_y]), tc_goal, 80)
    return np.vstack([np.column_stack([xs, ys]), seg_v])


def max_curvature(path):
    """离散折线最大曲率 (三点外接圆法)"""
    p1, p2, p3 = path[:-2], path[1:-1], path[2:]
    a = np.linalg.norm(p2 - p1, axis=1)
    b = np.linalg.norm(p3 - p2, axis=1)
    c = np.linalg.norm(p3 - p1, axis=1)
    cross = np.abs((p2[:, 0] - p1[:, 0]) * (p3[:, 1] - p1[:, 1])
                   - (p2[:, 1] - p1[:, 1]) * (p3[:, 0] - p1[:, 0]))
    area2 = cross / 2.0
    k = 4 * area2 / (a * b * c)
    return float(np.nanmax(k)) if len(k) else 0.0


def g_max_for(path, cap=22.0):
    """按路径最大曲率推所需铰接角, 留 1.6 倍跟踪余量"""
    k = max_curvature(path)
    g_need = np.degrees(np.arcsin(min(1.0, TR * k)))
    return float(min(cap, max(8.0, 1.6 * g_need)))


def try_park(path, g_max, **ck):
    ckwargs = dict(k_psi=6.0, k_lateral=0.01, k_steer=6.0, g_des_max_deg=g_max)
    ckwargs.update(ck)
    try:
        return P.run_once(path, **ckwargs)
    except Exception:
        return None


if __name__ == '__main__':
    rng = np.random.default_rng(42)
    N = 50
    fails_old, fails_new = [], []
    ok_old = ok_new = 0
    for i in range(N):
        x0 = float(rng.uniform(51.6, 58.4))
        y0 = float(rng.uniform(5.0, 12.0))
        # 旧规划器 (固定斜率直线段1)
        P.START_X, P.START_Y = x0, y0
        r_old = try_park(P.plan_path(), 5.0)
        if r_old and r_old['success']:
            ok_old += 1
        else:
            v = r_old['violations'][0][4] if r_old and r_old['violations'] else 'timeout'
            fails_old.append((round(x0, 2), round(y0, 2), v))
        # 自适应规划器
        path_new = plan_path_adaptive(x0, y0)
        r_new = try_park(path_new, g_max_for(path_new))
        if r_new and r_new['success']:
            ok_new += 1
        else:
            v = r_new['violations'][0][4] if r_new and r_new['violations'] else 'timeout'
            fails_new.append((round(x0, 2), round(y0, 2), v))
    P.START_X, P.START_Y = START_X, START_Y
    print(f"随机起点 {N} 次 (x∈[51.6,58.4], y∈[5,12], 朝向-90°):")
    print(f"  旧规划器:   {ok_old}/{N}")
    print(f"  自适应规划器: {ok_new}/{N}")
    print("旧失败样本 (x0, y0, 首个违规):", fails_old[:8])
    print("新失败样本 (x0, y0, 首个违规):", fails_new[:8])
