# -*- coding: utf-8 -*-
"""
半挂车倒车入库 — 规划 + 控制 + 三线可视化

轨迹含义 (牵引车参考点轨迹):
  红色点状虚线 = 规划线   (PathPlanner 规划的参考路径)
  黄色点状虚线 = 控制线   (理想控制跟踪轨迹, 无扰动)
  蓝色实线     = 实际轨迹 (带转向噪声/初始扰动的实际执行轨迹)

运行:
  .venv/bin/python another_solve/parking_three_lines.py            # 实时可视化
  .venv/bin/python another_solve/parking_three_lines.py --headless # 无头验证 + 保存截图

按键: SPACE 暂停 | R 重置 | UP/DOWN 倍速 | ESC 退出
"""

import os
import sys

import numpy as np
import pygame

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from map.dual_lane_road import DualLaneRoad
from config.scene_config import (
    SCALE, DT, START_X, START_Y, GOAL_X, GOAL_Y, ROAD_CENTER_X, ROAD_Y_END,
    RIGHT_LANE_WIDTH, LEFT_LANE_WIDTH, PARKING_TOP_Y, PARKING_BTM_Y,
    INIT_HEADING_RAD, MAX_STEERING_RATE,
)
from another_solve.path_planner import PathPlanner

MAX_STEPS = 2500
V_REVERSE = -1.0            # 倒车速度 m/s
STEER_RATE = MAX_STEERING_RATE * DT   # 每步转向增量上限 (3°/步)
TRAILER_LEN = 11.0          # 挂车轴距, 挂车后轴中心 = 铰点后方 L/2

NOISE_STD = 1.0             # 实际轨迹的转向噪声 (度/步)
ACTUAL_SEED = 2

# 库位约束 (线 1003=左 x=56, 1002=右 x=60, 1014=入口 y=50.5, 1015=底 y=69)
SLOT_LEFT = ROAD_CENTER_X
SLOT_RIGHT = ROAD_CENTER_X + RIGHT_LANE_WIDTH
ROAD_LEFT = ROAD_CENTER_X - LEFT_LANE_WIDTH
SLOT_TOP = PARKING_TOP_Y
SLOT_BTM = PARKING_BTM_Y


def corner_violations(corners):
    """8 角点约束: 角点低于库位入口线(1014)后必须在库位内, 否则必须在道路内"""
    bad = []
    for i, (cx, cy) in enumerate(corners):
        if cy > SLOT_TOP:            # 已越过库位入口线
            if cx < SLOT_LEFT or cx > SLOT_RIGHT:
                bad.append((i, round(cx, 2), round(cy, 2), 'slot'))
        else:                        # 还在道路上
            if cx < ROAD_LEFT or cx > SLOT_RIGHT:
                bad.append((i, round(cx, 2), round(cy, 2), 'road'))
        if cy > SLOT_BTM + 1e-6:     # 超过库位底线
            bad.append((i, round(cx, 2), round(cy, 2), 'bottom'))
    return bad


def corners_in_slot(corners):
    """停车判定: 8 角点全部在库位 [1003,1002]x[1014,1015] 内"""
    return all(SLOT_LEFT <= cx <= SLOT_RIGHT and SLOT_TOP <= cy <= SLOT_BTM
               for cx, cy in corners)

# 三线颜色
PLAN_COLOR = (255, 70, 70)      # 红
CTRL_COLOR = (204, 153, 0)      # 黄 (加深以适配白色背景)
ACTUAL_COLOR = (80, 150, 255)   # 蓝


def plan_path():
    """规划线: 挂车后轴中心两段路径 (盯尾不盯头)

    段1 斜线接近: 从起点横向平移到库位 x 上方 (库口线前留直行段)
    段2 垂直入库: 与库位 1003/1002 平行直倒 (挂车摆直后直着进库)
    """
    tc_start = np.array([START_X, START_Y + TRAILER_LEN / 2])
    dock_x = (SLOT_LEFT + SLOT_RIGHT) / 2      # 对准库位中心, 左右边距对称
    tc_goal = np.array([dock_x, GOAL_Y + TRAILER_LEN / 2 - 0.45])  # 留出车尾余量
    join_y = SLOT_TOP - 5.5                     # 库口线前 5.5m 摆直
    join = np.array([dock_x, join_y])

    n1, n2 = 140, 80
    seg1 = np.linspace(tc_start, join, n1, endpoint=False)
    seg2 = np.linspace(join, tc_goal, n2)
    return np.vstack([seg1, seg2])


def local_psi_ref(path, y):
    """路径局部行进方向对应的挂车参考朝向 (行进方向 - 180°)"""
    ys = path[:, 1]
    j = int(np.clip(np.searchsorted(ys, y), 2, len(path) - 3))
    d = path[j + 2] - path[j - 2]
    return np.arctan2(d[1], d[0]) - np.pi


def x_plan_at(path, y):
    """规划线按 y 取 x (路径沿 y 单调)"""
    ys = path[:, 1]
    return np.interp(y, ys, path[:, 0])


class ParkingController:
    """盯尾串级控制: 挂车航向误差+横向误差 -> 目标铰接角 -> 前轮转角

    控制律对应老司机技巧 (少量多次左右微调):
      ψ = 挂车朝向与库线方向的偏差  -> 决定目标铰接角的符号(让挂车摆向库线)
      e = 挂车后轴中心横向偏差      -> 决定需要多大的持续铰接角横移
      倒车时保持 g<0 挂车右移, g>0 左移
    """

    def __init__(self, path, k_psi=6.0, k_lateral=0.01, k_steer=6.0, g_des_max_deg=5.0):
        self.path = path
        self.prev_swa = 0.0
        # 增益
        self.k_psi = k_psi              # 挂车航向误差 -> 目标铰接角 (rad/rad)
        self.k_lateral = k_lateral      # 横向误差 -> 目标铰接角 (rad/m)
        self.k_steer = k_steer          # 铰接角误差 -> 转向角 (rad/rad)
        self.g_des_max = np.deg2rad(g_des_max_deg)

    def reset(self):
        self.prev_swa = 0.0

    def trailer_center(self, truck):
        """挂车后轴中心: 铰点(牵引车参考点)沿挂车朝向 h+g 反方向 Lt/2 处"""
        h, g = truck.h_rad, truck.g_rad
        th = h + g
        return (truck.x_m - TRAILER_LEN / 2 * np.cos(th),
                truck.y_m - TRAILER_LEN / 2 * np.sin(th))

    def compute(self, truck):
        tc_x, tc_y = self.trailer_center(truck)
        e = tc_x - x_plan_at(self.path, np.clip(tc_y, self.path[0, 1], self.path[-1, 1]))

        th1 = truck.h_rad + truck.g_rad                     # 挂车朝向
        psi_ref = local_psi_ref(self.path, tc_y)            # 局部参考朝向(垂直入库段 ψ_ref=0)
        psi = (th1 - psi_ref + np.pi) % (2 * np.pi) - np.pi

        # 倒车时保持 g<0 使挂车右移, g>0 左移; ψ>0(挂车朝向偏左) 需 g<0 摆回
        g_des = np.clip(-self.k_psi * psi + self.k_lateral * e,
                        -self.g_des_max, self.g_des_max)
        swa = self.k_steer * (g_des - truck.g_rad)

        # 防折叠安全约束 (稳定域约 +45°/-41.7°, 这里收紧到 ±40°)
        if truck.g_rad > np.deg2rad(40):
            swa = -np.deg2rad(35)
        elif truck.g_rad < -np.deg2rad(40):
            swa = np.deg2rad(35)
        swa = np.clip(swa, -np.deg2rad(35), np.deg2rad(35))

        # 转向速率限制
        swa = np.clip(swa, self.prev_swa - STEER_RATE, self.prev_swa + STEER_RATE)
        self.prev_swa = swa
        return np.rad2deg(swa), e, psi


def run_once(path, noise_std=0.0, seed=0, **ckwargs):
    """跑一次倒车入库. noise_std>0 时加入初始扰动与转向噪声 (实际轨迹)"""
    rng = np.random.default_rng(seed)
    truck = TruckTrailerOnAxle()
    x0 = START_X + rng.normal(0, 0.15) if noise_std else START_X
    h0 = INIT_HEADING_RAD + rng.normal(0, np.deg2rad(0.5)) if noise_std else INIT_HEADING_RAD
    truck.set_state([x0, START_Y, h0, 0.0])
    truck.v_mps = V_REVERSE

    controller = ParkingController(path, **ckwargs)
    tc_goal_y = GOAL_Y + TRAILER_LEN / 2 - 0.45          # 与 plan_path 的 tc_goal 一致

    states = []       # 每步 (x, y, h, g, s_deg)
    tc_states = []    # 每步挂车后轴中心
    errors, psis = [], []
    violations = []   # (step, 角点编号, x, y, 类型)
    success = False
    for step in range(MAX_STEPS):
        swa_deg, e, psi = controller.compute(truck)
        if noise_std:
            swa_deg += rng.normal(0, noise_std)
            swa_deg = float(np.clip(swa_deg, -35, 35))
        truck.set_steering(swa_deg)
        truck.drive(DT)

        states.append((truck.x_m, truck.y_m, truck.h_rad, truck.g_rad, truck.s_deg))
        tc_states.append(controller.trailer_center(truck))
        errors.append(e)
        psis.append(psi)

        bad = corner_violations(truck.get_corner_positions())
        if bad:
            violations.append((step, *bad[0]))

        # 停车: 挂车后轴中心到达库内目标
        if tc_states[-1][1] >= tc_goal_y:
            success = (not violations) and corners_in_slot(truck.get_corner_positions())
            break

    return {
        'states': states,
        'tc_states': np.array(tc_states),
        'errors': np.array(errors),
        'psis': np.array(psis),
        'violations': violations,
        'truck': truck,
        'controller': controller,
        'success': success,
        'steps': step + 1,
    }


def load_runs():
    """离线计算 规划线 / 控制线(理想) / 实际轨迹(带噪) — 均为挂车后轴中心轨迹"""
    path = plan_path()
    ideal = run_once(path, noise_std=0.0)
    actual = run_once(path, noise_std=NOISE_STD, seed=ACTUAL_SEED)
    return {
        'plan': path[:, :2],
        'control': ideal['tc_states'],
        'actual': actual,
        'ideal': ideal,
    }


# ---------------- 可视化 ----------------

def load_font(size):
    for fp in [os.path.expanduser('~/.fonts/msyh.ttc'),
               '/mnt/c/Windows/Fonts/msyh.ttc']:
        if os.path.exists(fp):
            return pygame.font.Font(fp, size)
    return pygame.font.SysFont(None, size)


def draw_dotted(surface, pts_m, color, radius=3, every=2):
    for i in range(0, len(pts_m), every):
        pygame.draw.circle(surface, color,
                           (int(pts_m[i][0] * SCALE), int(pts_m[i][1] * SCALE)), radius)


def draw_solid(surface, pts_m, color, width=3):
    px = [(int(p[0] * SCALE), int(p[1] * SCALE)) for p in pts_m]
    if len(px) >= 2:
        pygame.draw.lines(surface, color, False, px, width)


def compute_camera(truck_y, view_w, view_h):
    cam_x = ROAD_CENTER_X - 0.62 * view_w / SCALE
    view_h_m = view_h / SCALE
    if view_h_m >= ROAD_Y_END:
        cam_y = (ROAD_Y_END - view_h_m) / 2
    else:
        cam_y = float(np.clip(truck_y - 0.4 * view_h_m, 0, ROAD_Y_END - view_h_m))
    return cam_x, cam_y


WORLD_W_PX = int(80 * SCALE)    # 世界画布: x 0..80m
WORLD_H_PX = int(80 * SCALE)    # y 0..80m


def render_frame(screen, runs, idx, road, font_small, font_big):
    from render.render_utils import draw_truck, WHITE, GREEN

    actual = runs['actual']
    states = actual['states']
    idx = max(0, min(idx, len(states) - 1))
    x, y, h, g, s_deg = states[idx]

    view_w, view_h = screen.get_size()
    cam_x, cam_y = compute_camera(y, view_w, view_h)

    # 世界画布 (像素 = 米 * SCALE, 无相机偏移), 再按视口裁剪贴到屏幕
    world = pygame.Surface((WORLD_W_PX, WORLD_H_PX))
    world.fill((255, 255, 255))
    road.draw(world)

    draw_dotted(world, runs['plan'], PLAN_COLOR, radius=3, every=3)
    act = runs['actual']['tc_states'][:idx + 1]
    draw_solid(world, act, ACTUAL_COLOR, width=3)
    ctrl = runs['control'][:idx + 1]
    draw_dotted(world, ctrl, CTRL_COLOR, radius=3, every=12)

    # 库位目标点
    pygame.draw.circle(world, (0, 160, 0), (int(GOAL_X * SCALE), int(GOAL_Y * SCALE)),
                       int(0.5 * SCALE), 2)

    # 车辆
    truck = TruckTrailerOnAxle()
    truck.set_state([x, y, h, g])
    truck.s_deg = s_deg
    draw_truck(world, truck, SCALE)

    screen.fill((255, 255, 255))
    area = pygame.Rect(int(cam_x * SCALE), int(cam_y * SCALE), view_w, view_h)
    screen.blit(world, (0, 0), area=area)

    # 图例
    legend = [
        (PLAN_COLOR, "红色点状虚线 = 规划线"),
        (CTRL_COLOR, "黄色点状虚线 = 控制线 (理想控制)"),
        (ACTUAL_COLOR, "蓝色实线 = 实际轨迹 (带扰动)"),
    ]
    for i, (color, text) in enumerate(legend):
        ly = 12 + i * 30
        pygame.draw.line(screen, color, (14, ly + 12), (56, ly + 12), 3)
        for dx in range(16, 56, 10):
            pygame.draw.circle(screen, color, (dx, ly + 12), 3)
        surf = font_small.render(text, True, (30, 30, 30))
        screen.blit(surf, (66, ly))

    # 信息面板
    tc_x, tc_y = runs['actual']['tc_states'][idx]
    done = idx >= len(states) - 1
    cur_bad = corner_violations(truck.get_corner_positions())
    status = "入库成功" if (done and runs['actual']['success']) else (
        "未成功停入" if done else ("约束警告!" if cur_bad else "执行中"))
    lines = [
        f"步数: {idx + 1}/{len(states)}",
        f"挂车后轴中心: ({tc_x:.2f}, {tc_y:.2f}) m",
        f"航向误差 ψ: {np.rad2deg(runs['actual']['psis'][idx]):+.2f}°   "
        f"横向误差 e: {runs['actual']['errors'][idx]:+.2f} m",
        f"铰接角: {np.rad2deg(g):+.1f}°   转向角: {s_deg:+.1f}°",
        f"约束: {'全部满足' if not cur_bad else f'{len(cur_bad)}个角点越界'}",
    ]
    for i, text in enumerate(lines):
        surf = font_small.render(text, True, (30, 30, 30))
        screen.blit(surf, (14, view_h - 24 - (len(lines) - 1 - i) * 26))

    if done:
        msg = status
        color = (0, 150, 60) if (done and runs['actual']['success']) else (
            (190, 130, 0) if (done and not cur_bad) else (220, 40, 40))
        surf = font_big.render(msg, True, color)
        screen.blit(surf, (view_w // 2 - surf.get_width() // 2, 14))


def ensure_display():
    """WSLg 下 DISPLAY 常缺失, SDL 会静默退到 dummy 驱动(无窗口).
    检测 X11 套接字存在则自动补 DISPLAY, 否则明确报错."""
    if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
        return
    if os.path.exists('/tmp/.X11-unix/X0'):
        os.environ['DISPLAY'] = ':0'
        print("检测到 WSLg, 已自动设置 DISPLAY=:0")
    elif '--headless' not in sys.argv:
        sys.exit("未检测到图形显示 (DISPLAY/WAYLAND_DISPLAY 均未设置).\n"
                 "WSL2 请先执行: export DISPLAY=:0")


def main():
    headless = '--headless' in sys.argv
    if headless:
        os.environ['SDL_VIDEODRIVER'] = 'dummy'
    else:
        ensure_display()
    os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')   # WSL2 无声卡, 静音 ALSA 报错

    pygame.init()
    if headless:
        screen = pygame.display.set_mode((1280, 1460))   # 全程 70m 可见
    else:
        try:
            info = pygame.display.Info()
            screen = pygame.display.set_mode(
                (info.current_w, info.current_h),
                pygame.FULLSCREEN | pygame.HWSURFACE | pygame.DOUBLEBUF)
        except pygame.error:
            screen = pygame.display.set_mode((1280, 900))
    pygame.display.set_caption("半挂车倒车入库 — 规划/控制/实际 三线可视化")

    font_small = load_font(22)
    font_big = load_font(40)

    print("正在计算规划与控制轨迹...")
    runs = load_runs()
    total = len(runs['actual']['states'])
    print(f"规划点 {len(runs['plan'])}, 控制线 {len(runs['control'])} 点, "
          f"实际轨迹 {total} 步, 入库成功={runs['actual']['success']}")

    road = DualLaneRoad(
        center_x=ROAD_CENTER_X, y_start=0, y_end=ROAD_Y_END,
        left_lane_width=LEFT_LANE_WIDTH, right_lane_width=RIGHT_LANE_WIDTH,
        scale=SCALE, road_id=1)
    road.set_start_position(START_X, START_Y)

    clock = pygame.time.Clock()
    idx = total - 1 if headless else 0
    speed = 2
    paused = False
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    idx = 0
                elif event.key == pygame.K_UP:
                    speed = min(16, speed * 2)
                elif event.key == pygame.K_DOWN:
                    speed = max(1, speed // 2)

        if not paused and idx < total - 1:
            idx = min(idx + speed, total - 1)

        render_frame(screen, runs, idx, road, font_small, font_big)
        pygame.display.flip()

        if headless:
            out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'parking_three_lines.png')
            pygame.image.save(screen, out)
            print(f"截图已保存: {out}")
            running = False
        else:
            clock.tick(60)

    pygame.quit()

    t = runs['actual']['truck']
    tc = runs['actual']['controller'].trailer_center(t)
    print("=" * 60)
    print(f"实际执行: {total} 步, 入库{'成功' if runs['actual']['success'] else '失败'}")
    print(f"  牵引车参考点终点: ({t.x_m:.2f}, {t.y_m:.2f}), 挂车中心: ({tc[0]:.2f}, {tc[1]:.2f})")
    print(f"  末端铰接角: {np.rad2deg(t.g_rad):+.2f}°")
    print(f"  最大横向误差: {np.abs(runs['actual']['errors']).max():.2f}m")
    print("=" * 60)


if __name__ == '__main__':
    main()
