# -*- coding: utf-8 -*-
"""倒车入库实时 session — parking_three_lines.run_once() 的逐步骤镜像 (spec §9.1)。

直接复用已验证文件, 不搬代码、不改增益、不改停车判定。
"""
import os

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')

import numpy as np

from another_solve.parking_three_lines import (
    ACTUAL_SEED, DT, GOAL_Y, MAX_STEPS, NOISE_STD, STEER_RATE, TRAILER_LEN,
    V_REVERSE, ParkingController, corner_violations, corners_in_slot, plan_path,
)
from config.scene_config import INIT_HEADING_RAD, START_X, START_Y
from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle

PROTOCOL = 1
CONTROL_POINT_BACK = TRAILER_LEN / 2              # 5.5 m: 牵引车参考点 -> 虚拟控制点
HITCH_BACK = 1.5                                  # 牵引车参考点 -> 铰接点 (模型几何)
TRAILER_CENTER_BACK = 6.5                         # 铰接点 -> 挂车几何中心 (模型几何)


class ReverseDockSession:
    """单操作员倒车入库 session。phase: ready/running/paused/done/failed。"""

    def __init__(self, seed=ACTUAL_SEED, noise_std=NOISE_STD):
        self._seed = seed
        self._noise_std = noise_std
        self.path = plan_path()                   # 220 点, 无随机性, 只算一次
        self._tc_goal_y = GOAL_Y + TRAILER_LEN / 2 - 0.45   # 58.61, 与 plan_path/run_once 一致
        self.reset()

    # ---------------- 生命周期 ----------------

    def reset(self):
        rng = np.random.default_rng(self._seed)
        truck = TruckTrailerOnAxle()
        if self._noise_std:
            x0 = START_X + rng.normal(0, 0.15)    # 与 run_once 完全一致的取值与顺序
            h0 = INIT_HEADING_RAD + rng.normal(0, np.deg2rad(0.5))
        else:
            x0, h0 = START_X, INIT_HEADING_RAD
        truck.set_state([x0, START_Y, h0, 0.0])   # g=0
        truck.v_mps = V_REVERSE
        self._rng = rng
        self._truck = truck
        self._controller = ParkingController(self.path)
        self._prev_applied = 0.0                  # 执行器上一拍实际转角 (度)
        self._phase = 'ready'
        self._seq = 0
        self._steps = 0
        self._violation_steps = 0
        self._max_lat_err = 0.0
        self._last_e = 0.0
        self._last_psi = 0.0
        self._success = None

    @property
    def phase(self):
        return self._phase

    @property
    def seq(self):
        return self._seq

    # ---------------- 命令 (spec §8.7) ----------------

    def start(self):
        if self._phase in ('done', 'failed'):
            self.reset()
        if self._phase in ('ready', 'paused'):
            self._phase = 'running'

    def pause(self):
        if self._phase == 'running':
            self._phase = 'paused'

    def resume(self):
        if self._phase == 'paused':
            self._phase = 'running'

    # ---------------- 步进 (镜像 run_once 的一次循环) ----------------

    def step(self):
        if self._phase != 'running':
            return None
        truck = self._truck
        swa_deg, e, psi = self._controller.compute(truck)
        if self._noise_std:
            swa_deg += self._rng.normal(0, self._noise_std)
        swa_deg = float(np.clip(swa_deg, -35, 35))
        swa_deg = float(np.clip(swa_deg,
                                self._prev_applied - np.rad2deg(STEER_RATE),
                                self._prev_applied + np.rad2deg(STEER_RATE)))
        self._prev_applied = swa_deg
        truck.set_steering(swa_deg)
        truck.drive(DT)

        self._steps += 1
        self._seq += 1
        self._last_e, self._last_psi = float(e), float(psi)
        self._max_lat_err = max(self._max_lat_err, abs(float(e)))

        if corner_violations(truck.get_corner_positions()):
            self._violation_steps += 1

        cp = self._control_point()
        if cp[1] >= self._tc_goal_y:                       # 到达停车线
            self._success = (self._violation_steps == 0
                             and corners_in_slot(truck.get_corner_positions()))
            self._phase = 'done' if self._success else 'failed'
        elif self._steps >= MAX_STEPS:
            self._success = False
            self._phase = 'failed'
        return self.state_message()

    # ---------------- 消息构造 (spec §8) ----------------

    def _control_point(self):
        th = self._truck.h_rad + self._truck.g_rad
        return (self._truck.x_m - CONTROL_POINT_BACK * np.cos(th),
                self._truck.y_m - CONTROL_POINT_BACK * np.sin(th))

    def _trailer_center(self):
        h, g = self._truck.h_rad, self._truck.g_rad
        th = h + g
        return (self._truck.x_m - HITCH_BACK * np.cos(h) - TRAILER_CENTER_BACK * np.cos(th),
                self._truck.y_m - HITCH_BACK * np.sin(h) - TRAILER_CENTER_BACK * np.sin(th))

    def state_message(self):
        cp = self._control_point()
        tc = self._trailer_center()
        progress = float(np.clip((cp[1] - self.path[0, 1]) / (self._tc_goal_y - self.path[0, 1]),
                                 0.0, 1.0))
        return {
            'type': 'state', 'protocol': PROTOCOL,
            'seq': self._seq, 't': round(self._seq * DT, 6),
            'phase': self._phase,
            'done': self._phase in ('done', 'failed'),
            'success': self._success,
            'pose': {
                'x': float(self._truck.x_m), 'y': float(self._truck.y_m),
                'h': float(self._truck.h_rad), 'g': float(self._truck.g_rad),
                'steer': float(np.deg2rad(self._truck.s_deg)),
                'speed': V_REVERSE if self._phase == 'running' else 0.0,
            },
            'trailerCenter': [float(tc[0]), float(tc[1])],
            'controlPoint': [float(cp[0]), float(cp[1])],
            'progress': progress,
            'error': {'lateral': self._last_e, 'psi': self._last_psi},
            'violationSteps': self._violation_steps,
        }

    def done_message(self):
        msg = self.state_message()
        return {
            'type': 'done', 'protocol': PROTOCOL,
            'success': bool(self._success),
            'steps': self._steps,
            'simTime': round(self._steps * DT, 3),
            'violationSteps': self._violation_steps,
            'maxLateralError': round(self._max_lat_err, 4),
            'finalPose': msg['pose'],
            'finalControlPoint': msg['controlPoint'],
            'finalTrailerCenter': msg['trailerCenter'],
        }

    def hello_message(self):
        return {
            'type': 'hello', 'protocol': PROTOCOL,
            'scenario': {
                'id': 'reverse_dock', 'name': '倒车入库',
                'controller': {
                    'id': 'cascade', 'name': 'Cascade · tail-in',
                    'source': 'another_solve/parking_three_lines.py', 'real': True,
                },
                'dt': DT, 'noiseStdDeg': self._noise_std, 'seed': self._seed,
                'maxSteps': MAX_STEPS,
                'estimatedSteps': 866, 'estimatedStepsNoiseFree': 867,
                'estimatedDurationSec': 43.3,
                'start': {'x': 56.25, 'y': 9.91, 'h': -np.pi / 2, 'g': 0.0},
                'goal': {'x': 58.25, 'y': 53.56},
                'goalMeaning': 'tractor reference point (hitch) at GOAL_X/GOAL_Y',
                'controlGoal': {'x': 58.0, 'y': 58.61},
                'controlGoalMeaning': 'virtual control point stop y = GOAL_Y + 11/2 - 0.45',
                'road': {'left': 50.0, 'center': 56.0, 'right': 60.0,
                         'yStart': 0.0, 'yEnd': 70.0},
                'slot': {'left': 56.0, 'right': 60.0, 'top': 50.5, 'bottom': 69.0},
                'vehicle': {
                    'truckWheelbase': 3.0, 'trailerWheelbase': 11.0,
                    'truckLength': 5.0, 'truckWidth': 3.0,
                    'trailerLength': 15.0, 'trailerWidth': 2.8,
                    'maxSteeringDeg': 35.0, 'maxSteeringRateDegPerSec': 60.0,
                    'maxArticulationDeg': 90.0, 'jackknifeDeg': 85.0,
                    'reverseSpeedMps': -1.0,
                },
                'referencePath': [[float(px), float(py)] for px, py in self.path[:, :2]],
                'referencePathMeaning':
                    'virtual control point = root - 5.5 m along trailer heading',
            },
            'state': self.state_message(),
        }
