# -*- coding: utf-8 -*-
"""半挂车侧方位入库 — 左车道侧方位库位 (与倒车入库同一地图)

库位: 左车道 x∈[50,54] (路缘1001 与 新增虚线2001 之间), y∈[43.5,62.5]
 maneuver: 挂车后轴中心沿 S 曲线西移入位 (曲率有界五次平滑), 车头留在行车道
约束: 挂车 4 角点在库位 y 范围内必须 x∈[50,54], 车头 4 角点全程不出道路 [50,60]
成功: 挂车 4 角点全部在库位内, 全程无违规

用法: python3 another_solve/parking_parallel.py [--headless]
"""
import os
import sys

import numpy as np
import pygame

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from config.scene_config import (
    SCALE, DT, START_X, START_Y, ROAD_CENTER_X, ROAD_Y_END,
    INIT_HEADING_RAD,
)
from map.geometry_line import GeometryLine
from another_solve.parking_three_lines import (
    ParkingController, x_plan_at, local_psi_ref,
    draw_dotted, draw_solid, load_font, compute_camera,
    truck_from_state, screen_to_world, point_on_truck, draw_help,
    PLAN_COLOR, CTRL_COLOR, ACTUAL_COLOR,
    MAX_STEPS, V_REVERSE, NOISE_STD, ACTUAL_SEED, STEER_RATE, corners_in_slot,
)

# 侧方位库位 (左车道)
SLOT_P_LEFT = 50.0        # 路缘线 1001
SLOT_P_RIGHT = 54.0       # 库位内侧虚线
SLOT_P_Y0 = 46.0          # 南端边界
SLOT_P_Y1 = 65.0          # 北端边界
DOCK_X = (SLOT_P_LEFT + SLOT_P_RIGHT) / 2     # 52.0
TC_GOAL_Y = 54.5                              # 挂车后轴中心目标 y (车尾留 0.5m)
JOIN_Y = TC_GOAL_Y                            # S 曲线直接延伸到停车位 (拉长过渡段降峰值铰接角)

HELP_LINES_SIDE = (
    "鼠标左键拖拽车辆 | L 拉直挂车",
    "空格 暂停/继续 | R 重置",
    "UP/DOWN 倍速 | ESC 退出",
)


def smoothstep(s):
    return s * s * s * (10 - 15 * s + 6 * s * s)


def max_curvature(path):
    p1, p2, p3 = path[:-2], path[1:-1], path[2:]
    a = np.linalg.norm(p2 - p1, axis=1)
    b = np.linalg.norm(p3 - p2, axis=1)
    c = np.linalg.norm(p3 - p1, axis=1)
    cross = np.abs((p2[:, 0] - p1[:, 0]) * (p3[:, 1] - p1[:, 1])
                   - (p2[:, 1] - p1[:, 1]) * (p3[:, 0] - p1[:, 0]))
    with np.errstate(divide='ignore', invalid='ignore'):
        k = 4 * (cross / 2) / (a * b * c)
    return float(np.nanmax(k)) if len(k) else 0.0


def g_max_for(path, cap=18.0):
    k = max_curvature(path)
    g_need = np.degrees(np.arcsin(min(1.0, 11.0 * k)))
    return float(min(cap, max(8.0, 1.7 * g_need)))


def plan_path_parallel(x0=START_X, y0=START_Y):
    """S 曲线西移入位: (x0, y0+5.5) -> (DOCK_X, JOIN_Y) -> (DOCK_X, TC_GOAL_Y)"""
    tc_start = np.array([x0, y0 + 5.5])
    n = max(40, int((JOIN_Y - tc_start[1]) * 4))
    ys = np.linspace(tc_start[1], JOIN_Y, n, endpoint=False)
    s = (ys - tc_start[1]) / (JOIN_Y - tc_start[1])
    xs = x0 + (DOCK_X - x0) * smoothstep(s)
    seg_v = np.linspace(np.array([DOCK_X, JOIN_Y]), np.array([DOCK_X, TC_GOAL_Y]), 4)
    return np.vstack([np.column_stack([xs, ys]), seg_v])


def corners_violations_parallel(corners):
    """全程约束: 所有角点不出道路 [50,60]x[0,70] (库位标线可跨越)"""
    bad = []
    for i, (cx, cy) in enumerate(corners):
        if cx < 50.0 - 1e-6 or cx > 60.0 + 1e-6 or cy < 0 or cy > 70.0:
            bad.append((i, round(cx, 2), round(cy, 2)))
    return bad


def trailer_in_slot(corners):
    return all(SLOT_P_LEFT <= cx <= SLOT_P_RIGHT and SLOT_P_Y0 <= cy <= SLOT_P_Y1
               for cx, cy in corners[4:])


def run_parallel(path, x0=START_X, y0=START_Y, noise_std=0.0, seed=0, **ck):
    rng = np.random.default_rng(seed)
    truck = TruckTrailerOnAxle()
    nx = x0 + rng.normal(0, 0.15) if noise_std else x0
    nh = INIT_HEADING_RAD + rng.normal(0, np.deg2rad(0.5)) if noise_std else INIT_HEADING_RAD
    truck.set_state([nx, y0, nh, 0.0])
    truck.v_mps = V_REVERSE

    controller = ParkingController(path, **ck)
    prev_applied = 0.0
    states, tc_states, errors, psis, violations = [], [], [], [], []
    success = False
    for step in range(MAX_STEPS):
        swa_deg, e, psi = controller.compute(truck)
        if noise_std:
            swa_deg += rng.normal(0, noise_std)
        swa_deg = float(np.clip(swa_deg, -35, 35))
        swa_deg = float(np.clip(swa_deg,
                                prev_applied - np.rad2deg(STEER_RATE),
                                prev_applied + np.rad2deg(STEER_RATE)))
        prev_applied = swa_deg
        truck.set_steering(swa_deg)
        truck.drive(DT)

        states.append((truck.x_m, truck.y_m, truck.h_rad, truck.g_rad, truck.s_deg))
        tc_states.append(controller.trailer_center(truck))
        errors.append(e)
        psis.append(psi)
        bad = corners_violations_parallel(truck.get_corner_positions())
        if bad:
            violations.append((step, *bad[0]))

        if tc_states[-1][1] >= TC_GOAL_Y:
            success = (not violations) and trailer_in_slot(truck.get_corner_positions())
            break

    return {
        'states': states, 'tc_states': np.array(tc_states),
        'errors': np.array(errors), 'psis': np.array(psis),
        'violations': violations, 'truck': truck, 'controller': controller,
        'success': success, 'steps': step + 1,
    }


def load_runs(x0=START_X, y0=START_Y, g_cap=18.0):
    path = plan_path_parallel(x0, y0)
    gm = g_max_for(path, g_cap)
    ideal = run_parallel(path, x0, y0, g_des_max_deg=gm)
    actual = run_parallel(path, x0, y0, noise_std=NOISE_STD, seed=ACTUAL_SEED,
                          g_des_max_deg=gm)
    return {'plan': path[:, :2], 'control': ideal['tc_states'],
            'actual': actual, 'g_max': gm}


def add_parallel_slot_lines(road):
    """在道路上添加左侧侧方位库位标线"""
    Y = (204, 153, 0)
    lines = [
        (SLOT_P_RIGHT, SLOT_P_Y0, SLOT_P_RIGHT, SLOT_P_Y1, 2001),   # 内侧纵向线
        (SLOT_P_LEFT, SLOT_P_Y0, SLOT_P_RIGHT, SLOT_P_Y0, 2002),    # 南端横线
        (SLOT_P_LEFT, SLOT_P_Y1, SLOT_P_RIGHT, SLOT_P_Y1, 2003),    # 北端横线
    ]
    for x0_, y0_, x1_, y1_, lid in lines:
        ln = GeometryLine(start=(x0_, y0_), end=(x1_, y1_), color=Y,
                          width=3, line_type="dashed", dash_length=0.75, scale=SCALE)
        ln.line_id = lid
        road.geometry_lines.append(ln)


def render_frame(screen, runs, idx, road, font_small, font_big, override=None, cam=None):
    from render.render_utils import draw_truck

    actual = runs['actual']
    states = actual['states']
    idx = max(0, min(idx, len(states) - 1))
    if override is not None:
        x, y, h, g, s_deg = override
    else:
        x, y, h, g, s_deg = states[idx]
    view_w, view_h = screen.get_size()
    cam_x, cam_y = cam if cam is not None else compute_camera(y, view_w, view_h)

    world = pygame.Surface((int(80 * SCALE), int(80 * SCALE)))
    world.fill((255, 255, 255))
    road.draw(world)

    draw_dotted(world, runs['plan'], PLAN_COLOR, radius=3, every=3)
    draw_solid(world, runs['actual']['tc_states'][:idx + 1], ACTUAL_COLOR, width=3)
    draw_dotted(world, runs['control'][:idx + 1], CTRL_COLOR, radius=3, every=12)

    pygame.draw.circle(world, (0, 160, 0), (int(DOCK_X * SCALE), int(TC_GOAL_Y * SCALE)),
                       int(0.5 * SCALE), 2)

    truck = truck_from_state((x, y, h, g, s_deg))
    draw_truck(world, truck, SCALE)

    screen.fill((255, 255, 255))
    area = pygame.Rect(int(cam_x * SCALE), int(cam_y * SCALE), view_w, view_h)
    screen.blit(world, (0, 0), area=area)

    legend = [
        (PLAN_COLOR, "红色点状虚线 = 规划线 (S曲线入位)"),
        (CTRL_COLOR, "黄色点状虚线 = 控制线 (理想控制)"),
        (ACTUAL_COLOR, "蓝色实线 = 实际轨迹 (带扰动)"),
    ]
    for i, (color, text) in enumerate(legend):
        ly = 12 + i * 30
        pygame.draw.line(screen, color, (14, ly + 12), (56, ly + 12), 3)
        for dx in range(16, 56, 10):
            pygame.draw.circle(screen, color, (dx, ly + 12), 3)
        screen.blit(font_small.render(text, True, (30, 30, 30)), (66, ly))

    manual = override is not None
    tc_x, tc_y = runs['actual']['controller'].trailer_center(truck)
    done = (not manual) and idx >= len(states) - 1
    cur_bad = corners_violations_parallel(truck.get_corner_positions())
    if manual:
        lines = [
            "手动位姿: 左键拖拽移动车辆 | L 拉直挂车",
            f"牵引车参考点: ({x:.2f}, {y:.2f}) m",
            f"挂车后轴中心: ({tc_x:.2f}, {tc_y:.2f}) m",
            f"航向: {np.rad2deg(h):+.1f}°   铰接角: {np.rad2deg(g):+.1f}°   "
            f"转向角: {s_deg:+.1f}°",
            f"约束: {'全部满足' if not cur_bad else f'{len(cur_bad)}个角点越界'}",
        ]
    else:
        lines = [
            f"步数: {idx + 1}/{len(states)}   侧方位库位 x∈[{SLOT_P_LEFT:.0f},{SLOT_P_RIGHT:.0f}]",
            f"挂车后轴中心: ({tc_x:.2f}, {tc_y:.2f}) m",
            f"航向误差 ψ: {np.rad2deg(runs['actual']['psis'][idx]):+.2f}°   "
            f"横向误差 e: {runs['actual']['errors'][idx]:+.2f} m",
            f"铰接角: {np.rad2deg(g):+.1f}°   转向角: {s_deg:+.1f}°",
            f"约束: {'全部满足' if not cur_bad else f'{len(cur_bad)}个角点越界'}",
        ]
    for i, text in enumerate(lines):
        screen.blit(font_small.render(text, True, (30, 30, 30)),
                    (14, view_h - 24 - (len(lines) - 1 - i) * 26))

    # 操作提示 (右上角)
    draw_help(screen, font_small, view_w, view_h, HELP_LINES_SIDE)

    if manual:
        surf = font_small.render("手动位姿 (空格继续回放 / R 重置)", True, (70, 90, 170))
        screen.blit(surf, (view_w // 2 - surf.get_width() // 2, 120))
    elif done:
        status = "侧方位入库成功" if runs['actual']['success'] else "未成功停入"
        color = (0, 150, 60) if runs['actual']['success'] else (
            (190, 130, 0) if not cur_bad else (220, 40, 40))
        surf = font_big.render(status, True, color)
        screen.blit(surf, (view_w // 2 - surf.get_width() // 2, 120))


def ensure_display():
    if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
        return
    if os.path.exists('/tmp/.X11-unix/X0'):
        os.environ['DISPLAY'] = ':0'
        print("检测到 WSLg, 已自动设置 DISPLAY=:0")
    elif '--headless' not in sys.argv:
        sys.exit("未检测到图形显示 (DISPLAY/WAYLAND_DISPLAY 均未设置).")


def main():
    headless = '--headless' in sys.argv
    if headless:
        os.environ['SDL_VIDEODRIVER'] = 'dummy'
    else:
        ensure_display()
    os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')

    pygame.init()
    if headless:
        screen = pygame.display.set_mode((900, 1400))
    else:
        try:
            info = pygame.display.Info()
            screen = pygame.display.set_mode(
                (info.current_w, info.current_h),
                pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF)
        except pygame.error:
            screen = pygame.display.set_mode((1280, 900))
    pygame.display.set_caption("半挂车侧方位入库 — S曲线入位三线可视化")

    font_small = load_font(22)
    font_big = load_font(40)

    print("正在计算侧方位入库轨迹...")
    runs = load_runs()
    total = len(runs['actual']['states'])
    print(f"规划点 {len(runs['plan'])}, 控制线 {len(runs['control'])} 点, "
          f"实际轨迹 {total} 步, 入位成功={runs['actual']['success']}, "
          f"g_max={runs['g_max']:.1f}°")

    from map.dual_lane_road import DualLaneRoad
    from config.scene_config import LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH
    road = DualLaneRoad(center_x=ROAD_CENTER_X, y_start=0, y_end=ROAD_Y_END,
                        left_lane_width=LEFT_LANE_WIDTH, right_lane_width=RIGHT_LANE_WIDTH,
                        scale=SCALE, road_id=1)
    road.set_start_position(START_X, START_Y)
    add_parallel_slot_lines(road)

    clock = pygame.time.Clock()
    idx = total - 1 if headless else 0
    speed = 2
    paused = False
    override = None         # 手动位姿 [x, y, h, g, s_deg]; None = 回放轨迹
    cam_hold = None         # 手动位姿时冻结相机, 拖拽时画面不跟车跑
    dragging = False
    drag_offset = (0.0, 0.0)
    running = True
    while running:
        view_w, view_h = screen.get_size()
        cur_state = override if override is not None else runs['actual']['states'][idx]
        cam_now = cam_hold if cam_hold is not None else compute_camera(
            cur_state[1], view_w, view_h)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                    if not paused:              # 继续回放时退出手动位姿
                        override = None
                        cam_hold = None
                        dragging = False
                elif event.key == pygame.K_r:
                    idx = 0
                    override = None
                    cam_hold = None
                    dragging = False
                elif event.key == pygame.K_l:   # 拉直: 铰接角归零
                    if override is None:
                        override = list(cur_state)
                        cam_hold = cam_now
                    override[3] = 0.0
                    paused = True
                elif event.key == pygame.K_UP:
                    speed = min(16, speed * 2)
                elif event.key == pygame.K_DOWN:
                    speed = max(1, speed // 2)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                disp = override if override is not None else runs['actual']['states'][idx]
                wx, wy = screen_to_world(event.pos, cam_now)
                if point_on_truck(truck_from_state(disp), wx, wy):
                    dragging = True
                    paused = True
                    if override is None:
                        override = list(disp)
                        cam_hold = cam_now
                    drag_offset = (override[0] - wx, override[1] - wy)
            elif event.type == pygame.MOUSEMOTION and dragging:
                if not event.buttons[0]:
                    dragging = False
                else:
                    wx, wy = screen_to_world(event.pos, cam_hold)
                    override[0] = wx + drag_offset[0]
                    override[1] = wy + drag_offset[1]
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1 and dragging:
                # 相机保持冻结: 松手后车辆仍停在鼠标落点, 不跳回画面中央
                dragging = False
        if override is None and not paused and idx < total - 1:
            idx = min(idx + speed, total - 1)

        render_frame(screen, runs, idx, road, font_small, font_big,
                     override=override, cam=cam_hold)
        pygame.display.flip()

        if headless:
            out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'parking_parallel.png')
            pygame.image.save(screen, out)
            print(f"截图已保存: {out}")
            running = False
        else:
            clock.tick(60)

    pygame.quit()

    t = runs['actual']['truck']
    tc = runs['actual']['controller'].trailer_center(t)
    print("=" * 60)
    print(f"侧方位入库: {total} 步, {'成功' if runs['actual']['success'] else '失败'}")
    print(f"  挂车后轴中心终点: ({tc[0]:.2f}, {tc[1]:.2f}) (库位中心 x={DOCK_X})")
    print(f"  末端铰接角: {np.rad2deg(t.g_rad):+.2f}°")
    print(f"  最大横向误差: {np.abs(runs['actual']['errors']).max():.2f}m")
    print("=" * 60)


if __name__ == '__main__':
    main()
