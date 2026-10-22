# -*- coding: utf-8 -*-
"""倒车入库基线测试用例

固化当前版本 (tag: parking-baseline-v1) 的入库行为作为基线:
  1. 默认增益下必须入库成功, 全程无角点违规
  2. 关键指标 (步数/终点位姿/末端铰接角/最大横向误差/角点最小边距)
     与 tests/parking_baseline.json 一致
  3. 带噪实际轨迹 (NOISE_STD, ACTUAL_SEED) 也必须入库成功

以后若改动起点位置/算法导致本测试不通过, 可回退到基线版本:
  git tag --list 'parking-baseline*'          # 查看基线版本
  git checkout parking-baseline-v1 -- another_solve/parking_three_lines.py

用法:
  python3 tests/test_parking_baseline.py            # 运行测试
  python3 tests/test_parking_baseline.py --update   # 重新生成基线数据 (确认改动是预期的之后)
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import another_solve.parking_three_lines as P

BASELINE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'parking_baseline.json')
FLOAT_TOL = 0.05     # 浮点指标允许偏差


def measure():
    path = P.plan_path()
    ideal = P.run_once(path, noise_std=0.0)
    truck = ideal['truck']
    corners = truck.get_corner_positions()
    margin = min(corners[:, 0].min() - P.SLOT_LEFT,
                 P.SLOT_RIGHT - corners[:, 0].max(),
                 corners[:, 1].min() - P.SLOT_TOP,
                 P.SLOT_BTM - corners[:, 1].max())
    tc = ideal['tc_states'][-1]
    return {
        'steps': int(ideal['steps']),
        'success': bool(ideal['success']),
        'violations': len(ideal['violations']),
        'tc_end': [round(float(tc[0]), 3), round(float(tc[1]), 3)],
        'g_end_deg': round(float(np.rad2deg(truck.g_rad)), 3),
        'max_lateral_err': round(float(np.abs(ideal['errors']).max()), 3),
        'corner_margin': round(float(margin), 3),
    }


def run_test():
    with open(BASELINE, encoding='utf-8') as f:
        base = json.load(f)
    cur = measure()
    failures = []

    for key in ['success', 'violations', 'steps']:
        if cur[key] != base[key]:
            failures.append(f"{key}: 期望 {base[key]}, 实际 {cur[key]}")
    for key in ['tc_end', 'g_end_deg', 'max_lateral_err', 'corner_margin']:
        v_base, v_cur = base[key], cur[key]
        if isinstance(v_base, list):
            if any(abs(a - b) > FLOAT_TOL for a, b in zip(v_base, v_cur)):
                failures.append(f"{key}: 期望 {v_base}, 实际 {v_cur}")
        elif abs(v_base - v_cur) > FLOAT_TOL:
            failures.append(f"{key}: 期望 {v_base}, 实际 {v_cur}")

    actual = P.run_once(P.plan_path(), noise_std=P.NOISE_STD, seed=P.ACTUAL_SEED)
    if not actual['success'] or actual['violations']:
        failures.append(f"带噪运行(ACTUAL_SEED={P.ACTUAL_SEED})未通过约束: "
                        f"success={actual['success']}, violations={len(actual['violations'])}")

    print("当前指标:", json.dumps(cur, ensure_ascii=False))
    print("基线指标:", json.dumps(base, ensure_ascii=False))
    if failures:
        print("\n[FAIL] 与基线不一致 / 未通过:")
        for msg in failures:
            print("  -", msg)
        return 1
    print(f"\n[PASS] 与基线 parking-baseline-v2 一致, 带噪运行成功")
    return 0


if __name__ == '__main__':
    if '--update' in sys.argv:
        cur = measure()
        with open(BASELINE, 'w', encoding='utf-8') as f:
            json.dump(cur, f, ensure_ascii=False, indent=2)
        print("基线已更新:", json.dumps(cur, ensure_ascii=False))
        sys.exit(0)
    sys.exit(run_test())
