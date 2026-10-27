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

按键: SPACE 暂停/继续 | R 重置 | UP/DOWN 倍速 | L 拉直挂车 | ESC 退出
交互: 鼠标左键拖拽车辆移动位置; 拖拽/拉直后按 SPACE 从当前位姿重新规划并倒车
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
STEER_RATE = np.deg2rad(MAX_STEERING_RATE) * DT   # 每步转向增量上限 (rad/步, 3°/步)
TRAILER_LEN = 11.0          # 挂车轴距, 挂车后轴中心 = 铰点后方 L/2

NOISE_STD = 1.0             # 实际轨迹的转向噪声 (度/步)
ACTUAL_SEED = 2
REPLAN_G_DES_MAX_DEG = 15.0  # 拖拽起点重规划时放宽的目标铰接角上限 (度)
REPLAN_JOIN_GAP = 5.5        # 拖拽起点 S 曲线摆直点距库口线的默认距离 (米)
REPLAN_JOIN_GAPS = (3.0, 5.5, 7.0, 8.5)  # 重规划时依次尝试的摆直点

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


def plan_path(start_state=None, join_gap=REPLAN_JOIN_GAP):
    """规划线: 挂车后轴中心两段路径 (盯尾不盯头)

    默认起点: 斜线接近 + 垂直入库 (保持基线不变)
    拖拽起点: 三次 Hermite S 曲线, 起终点切向竖直/沿当前挂车朝向,
              平滑过渡后再垂直入库.

    start_state: 拖拽后位姿 (x, y, h, g, s); None 表示默认起点
    join_gap: 摆直点距库口线的距离 (米), 默认起点固定 5.5m
    """
    dock_x = (SLOT_LEFT + SLOT_RIGHT) / 2      # 对准库位中心, 左右边距对称
    tc_goal = np.array([dock_x, GOAL_Y + TRAILER_LEN / 2 - 0.45])  # 留出车尾余量

    if start_state is None:
        tc_start = np.array([START_X, START_Y + TRAILER_LEN / 2])
        join_y = SLOT_TOP - 5.5                 # 库口线前 5.5m 摆直
        join = np.array([dock_x, join_y])
        seg1 = np.linspace(tc_start, join, 140, endpoint=False)
        return np.vstack([seg1, np.linspace(join, tc_goal, 80)])

    th = start_state[2] + start_state[3]
    tc_start = np.array([start_state[0] - TRAILER_LEN / 2 * np.cos(th),
                         start_state[1] - TRAILER_LEN / 2 * np.sin(th)])
    join_y = SLOT_TOP - join_gap
    join = np.array([dock_x, join_y])

    dy = join_y - tc_start[1]
    if dy > 2.0:
        # 起始切向 = 当前倒车方向; 终点切向竖直 (dx/dy = 0)
        m0 = float(np.clip(np.cos(th) / np.sin(th), -0.6, 0.6)) if abs(np.sin(th)) > 0.3 else 0.0
        n1 = max(40, int(round(dy / 0.212)))
        t = np.linspace(0.0, 1.0, n1, endpoint=False)
        h00 = 2 * t**3 - 3 * t**2 + 1
        h10 = t**3 - 2 * t**2 + t
        h01 = -2 * t**3 + 3 * t**2
        xs = h00 * tc_start[0] + h10 * dy * m0 + h01 * dock_x
        seg1 = np.stack([xs, tc_start[1] + t * dy], axis=1)
    else:
        seg1 = np.empty((0, 2))                 # 起点已到库口附近, 直接垂直段
    return np.vstack([seg1, np.linspace(join, tc_goal, 80)])


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
        # 增益
        self.k_psi = k_psi              # 挂车航向误差 -> 目标铰接角 (rad/rad)
        self.k_lateral = k_lateral      # 横向误差 -> 目标铰接角 (rad/m)
        self.k_steer = k_steer          # 铰接角误差 -> 转向角 (rad/rad)
        self.g_des_max = np.deg2rad(g_des_max_deg)

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
        return np.rad2deg(swa), e, psi    # 原始指令, 速率限制在执行层统一做


def run_once(path, noise_std=0.0, seed=0, start_state=None, **ckwargs):
    """跑一次倒车入库. noise_std>0 时加入初始扰动与转向噪声 (实际轨迹)

    start_state=(x, y, h, g, s_deg) 时从该位姿直接起步 (拖拽后重新倒车),
    否则从默认起点并在 noise_std>0 时叠加初始扰动.
    """
    rng = np.random.default_rng(seed)
    truck = TruckTrailerOnAxle()
    if start_state is not None:
        s0 = float(start_state[4]) if len(start_state) > 4 else 0.0
        truck.set_state(start_state[:4])
        truck.s_deg = s0
    else:
        x0 = START_X + rng.normal(0, 0.15) if noise_std else START_X
        h0 = INIT_HEADING_RAD + rng.normal(0, np.deg2rad(0.5)) if noise_std else INIT_HEADING_RAD
        truck.set_state([x0, START_Y, h0, 0.0])
        s0 = 0.0
    truck.v_mps = V_REVERSE

    controller = ParkingController(path, **ckwargs)
    tc_goal_y = GOAL_Y + TRAILER_LEN / 2 - 0.45          # 与 plan_path 的 tc_goal 一致
    prev_applied = s0                                    # 执行器上一拍实际转角 (度)

    states = []       # 每步 (x, y, h, g, s_deg)
    tc_states = []    # 每步挂车后轴中心
    errors, psis = [], []
    violations = []   # (step, 角点编号, x, y, 类型)
    success = False
    for step in range(MAX_STEPS):
        swa_deg, e, psi = controller.compute(truck)
        if noise_std:
            swa_deg += rng.normal(0, noise_std)
        # 执行器统一限幅: 幅值 ±35°, 速率 ±STEER_RATE (度/步, 指令和噪声都受限)
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


def load_runs(start_state=None):
    """离线计算 规划线 / 控制线(理想) / 实际轨迹(带噪) — 均为挂车后轴中心轨迹

    start_state 非空时以该位姿为起点重新规划 (空格从拖拽处倒车):
    依次尝试不同摆直点, 取首个无角点违规的方案; 全部失败时取违规最少者.
    """
    if start_state is None:
        path = plan_path()
        ideal = run_once(path, noise_std=0.0)
        actual = run_once(path, noise_std=NOISE_STD, seed=ACTUAL_SEED)
    else:
        best = None
        for gap in REPLAN_JOIN_GAPS:
            cand_path = plan_path(start_state, join_gap=gap)
            cand_ideal = run_once(cand_path, noise_std=0.0, start_state=start_state,
                                  g_des_max_deg=REPLAN_G_DES_MAX_DEG)
            key = (0 if cand_ideal['success'] else 1, len(cand_ideal['violations']))
            if best is None or key < best[0]:
                best = (key, cand_path, cand_ideal)
            if key == (0, 0):
                break
        _, path, ideal = best
        actual = run_once(path, noise_std=NOISE_STD, seed=ACTUAL_SEED,
                          start_state=start_state,
                          g_des_max_deg=REPLAN_G_DES_MAX_DEG)
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


def truck_from_state(state):
    """由 (x, y, h, g, s_deg) 构造车辆对象"""
    truck = TruckTrailerOnAxle()
    truck.set_state(state[:4])
    truck.s_deg = state[4] if len(state) > 4 else 0.0
    return truck


def screen_to_world(pos, cam):
    """屏幕像素 -> 世界坐标(米); 视图是世界画布的等比例裁剪, 无缩放"""
    return pos[0] / SCALE + cam[0], pos[1] / SCALE + cam[1]


def point_on_truck(truck, wx, wy):
    """判断世界坐标点是否落在牵引车/挂车矩形内 (拖拽命中检测)"""
    def in_rect(cx, cy, th, half_len, half_wid):
        dx, dy = wx - cx, wy - cy
        c, s = np.cos(th), np.sin(th)
        lx = dx * c + dy * s
        ly = -dx * s + dy * c
        return abs(lx) <= half_len and abs(ly) <= half_wid

    t_len = truck.truckWheelbase + 2 * truck.truckMargin
    if in_rect(truck.x_m, truck.y_m, truck.h_rad,
               t_len / 2 + 0.3, truck.truckWidth / 2 + 0.3):
        return True

    # 挂车矩形中心, 与 draw_truck 保持一致
    cnx = truck.x_m + (-t_len / 2 + truck.truckMargin) * np.cos(truck.h_rad)
    cny = truck.y_m + (-t_len / 2 + truck.truckMargin) * np.sin(truck.h_rad)
    tr_len = truck.trailerWheelbase + truck.trailerMarginFront + truck.trailerMarginBack
    tr_cx = cnx + (truck.trailerMarginFront - tr_len / 2) * np.cos(truck.h_rad + truck.g_rad)
    tr_cy = cny + (truck.trailerMarginFront - tr_len / 2) * np.sin(truck.h_rad + truck.g_rad)
    return in_rect(tr_cx, tr_cy, truck.h_rad + truck.g_rad,
                   tr_len / 2 + 0.3, truck.trailerWidth / 2 + 0.3)


HELP_LINES = (
    "左键拖拽车辆 | L 拉直挂车",
    "空格: 暂停/继续; 拖拽后从此处倒车",
    "R 重置 | UP/DOWN 倍速 | ESC 退出",
)


def draw_help(screen, font_small, view_w, view_h, lines=HELP_LINES):
    """右上角常驻快捷键说明 (浅底框, 保证文字不被场景标线干扰)"""
    surfs = [font_small.render(t, True, (60, 60, 60)) for t in lines]
    pad, lh = 10, 26
    w = max(s.get_width() for s in surfs) + 2 * pad
    h = lh * len(surfs) + 2 * pad
    x, y = view_w - w - 14, 10
    box = pygame.Surface((w, h), pygame.SRCALPHA)
    box.fill((255, 255, 255, 250))
    pygame.draw.rect(box, (185, 185, 185), box.get_rect(), 1)
    screen.blit(box, (x, y))
    for i, surf in enumerate(surfs):
        screen.blit(surf, (x + pad, y + pad + i * lh))


WORLD_W_PX = int(80 * SCALE)    # 世界画布: x 0..80m
WORLD_H_PX = int(80 * SCALE)    # y 0..80m


def render_frame(screen, runs, idx, road, font_small, font_big, override=None, cam=None):
    from render.render_utils import draw_truck, WHITE, GREEN

    actual = runs['actual']
    states = actual['states']
    idx = max(0, min(idx, len(states) - 1))
    if override is not None:
        x, y, h, g, s_deg = override
    else:
        x, y, h, g, s_deg = states[idx]

    view_w, view_h = screen.get_size()
    cam_x, cam_y = cam if cam is not None else compute_camera(y, view_w, view_h)

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
    truck = truck_from_state((x, y, h, g, s_deg))
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
    manual = override is not None
    tc_x, tc_y = runs['actual']['controller'].trailer_center(truck)
    done = (not manual) and idx >= len(states) - 1
    cur_bad = corner_violations(truck.get_corner_positions())
    if manual:
        lines = [
            "手动位姿: 左键拖拽 | L 拉直 | 空格从此处倒车",
            f"牵引车参考点: ({x:.2f}, {y:.2f}) m",
            f"挂车后轴中心: ({tc_x:.2f}, {tc_y:.2f}) m",
            f"航向: {np.rad2deg(h):+.1f}°   铰接角: {np.rad2deg(g):+.1f}°   "
            f"转向角: {s_deg:+.1f}°",
            f"约束: {'全部满足' if not cur_bad else f'{len(cur_bad)}个角点越界'}",
        ]
    else:
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

    # 操作提示 (右上角)
    draw_help(screen, font_small, view_w, view_h)

    if manual:
        surf = font_small.render("手动位姿 (空格 从此处倒车 / R 重置)", True, (70, 90, 170))
        screen.blit(surf, (view_w // 2 - surf.get_width() // 2, 120))
    elif done:
        status = "入库成功" if runs['actual']['success'] else "未成功停入"
        color = (0, 150, 60) if runs['actual']['success'] else (
            (190, 130, 0) if not cur_bad else (220, 40, 40))
        surf = font_big.render(status, True, color)
        screen.blit(surf, (view_w // 2 - surf.get_width() // 2, 120))


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
    base_runs = load_runs()
    runs = base_runs
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
                    if override is not None:    # 以拖拽/拉直后的位姿为起点重新倒车
                        start = list(override)
                        runs = load_runs(start_state=start)
                        total = len(runs['actual']['states'])
                        idx = 0
                        override = None
                        cam_hold = None
                        dragging = False
                        paused = False
                        road.set_start_position(start[0], start[1])
                        print(f"从手动位姿 ({start[0]:.2f}, {start[1]:.2f}) 重新倒车: "
                              f"{total} 步, 入库成功={runs['actual']['success']}")
                    else:
                        paused = not paused
                elif event.key == pygame.K_r:
                    runs = base_runs
                    total = len(runs['actual']['states'])
                    idx = 0
                    override = None
                    cam_hold = None
                    dragging = False
                    road.set_start_position(START_X, START_Y)
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
