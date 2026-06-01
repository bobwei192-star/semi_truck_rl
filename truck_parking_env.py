import numpy as np
import pygame
import gymnasium as gym
import os
from typing import Optional

from truckTrailerOnAxle import TruckTrailerOnAxle
from dual_lane_road import DualLaneRoad
from scene_config import (
    SCALE, DT, ROAD_CENTER_X, ROAD_Y_START, ROAD_Y_END, LANE_WIDTH,
    LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH,
    GOAL_X, GOAL_Y, GOAL_HEADING_TOL_DEG, GOAL_G_TOL_DEG, GOAL_POS_TOL_M,
    START_X, START_Y, START_HEADING_OFFSET_RAD, START_G_INIT_RAD,
    INIT_HEADING_RAD, HEADING_NOISE_RAD, INIT_G_RAD, G_NOISE_RAD,
    START_POS_NOISE_M,
    REWARD_DIST_COEF, REWARD_HEADING_COEF, REWARD_SOFT_BOUNDARY, REWARD_JACKKNIFE,
    REWARD_COLLISION, REWARD_SUCCESS,
    REWARD_STEER_CONSISTENT, REWARD_STEER_INCONSISTENT, REWARD_STEER_MAG,
    REWARD_G_RATE_HIGH, REWARD_G_DECREASE,
    JACKKNIFE_DEG, G_CONSISTENCY_THRESHOLD, G_MAG_THRESHOLD, G_RATE_THRESHOLD,
    SOFT_BOUNDARY_MARGIN,
    MAX_STEPS, FIXED_SPEED, MAX_STEERING_DEG, MAX_STEERING_RATE,
    CURRICULUM_STAGES,
)
from render_utils import (
    WHITE, BLACK, RED,
    draw_truck as _draw_truck_fn,
    draw_trajectory as _draw_trajectory_fn,
    init_pygame_screen,
)


class TruckParkingEnv(gym.Env):
    metadata = {'render_modes': ['human', 'rgb_array'], 'render_fps': 20}

    def __init__(self, render_mode: Optional[str] = None,
                 fullscreen: bool = False,
                 curriculum_stage: int = 4):
        super().__init__()

        self.fullscreen = fullscreen
        self.curriculum_stage = min(curriculum_stage, len(CURRICULUM_STAGES) - 1)
        self._stage_config = CURRICULUM_STAGES[self.curriculum_stage]

        self.observation_space = gym.spaces.Dict({
            'observation': gym.spaces.Box(
                low=-np.inf, high=np.inf, shape=(11,), dtype=np.float32),
            'achieved_goal': gym.spaces.Box(
                low=-np.inf, high=np.inf, shape=(5,), dtype=np.float32),
            'desired_goal': gym.spaces.Box(
                low=-np.inf, high=np.inf, shape=(5,), dtype=np.float32),
        })

        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

        self.render_mode = render_mode
        self.screen = None
        self.clock = None
        self.font = None

        self.truck = None
        self.road = None
        self.step_count = 0
        self._prev_dist = 0.0
        self._goal_heading = 0.0
        self._prev_steering = 0.0
        self.trajectory_points = []
        self._show_trajectory = False

        self.goal_x = self._stage_config['goal_x']
        self.goal_y = self._stage_config['goal_y']

    def set_show_trajectory(self, enable: bool):
        self._show_trajectory = enable

    def set_curriculum_stage(self, stage: int):
        self.curriculum_stage = min(stage, len(CURRICULUM_STAGES) - 1)
        self._stage_config = CURRICULUM_STAGES[self.curriculum_stage]
        self.goal_x = self._stage_config['goal_x']
        self.goal_y = self._stage_config['goal_y']

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        if not pygame.get_init():
            pygame.init()

        sx = self._stage_config['start_x'] + self.np_random.uniform(-START_POS_NOISE_M, START_POS_NOISE_M)
        sy = self._stage_config['start_y'] + self.np_random.uniform(-START_POS_NOISE_M, START_POS_NOISE_M)

        self.goal_x = self._stage_config['goal_x']
        self.goal_y = self._stage_config['goal_y']

        dx_g = self.goal_x - sx
        dy_g = self.goal_y - sy
        self._goal_heading = np.arctan2(dy_g, dx_g)

        # 固定初始朝向：车头朝上（heading = -90°），与起点位置无关
        heading = INIT_HEADING_RAD + self.np_random.uniform(-HEADING_NOISE_RAD, HEADING_NOISE_RAD) + START_HEADING_OFFSET_RAD
        g_init = INIT_G_RAD + self.np_random.uniform(-G_NOISE_RAD, G_NOISE_RAD) + START_G_INIT_RAD

        self.truck = TruckTrailerOnAxle()
        self.truck.set_state([sx, sy, heading, g_init])
        self.truck.v_mps = FIXED_SPEED
        # 垂直倒车场景：初始方向盘回正（0°），保持直线倒车
        self.truck.s_deg = 0.0
        self._prev_steering = 0.0

        self.road = DualLaneRoad(
            center_x=ROAD_CENTER_X,
            y_start=ROAD_Y_START,
            y_end=ROAD_Y_END,
            left_lane_width=LEFT_LANE_WIDTH,
            right_lane_width=RIGHT_LANE_WIDTH,
            scale=SCALE,
            road_id=1,
            verbose=False,
        )
        self.road.set_start_position(sx, sy)

        self.step_count = 0
        self._prev_dist = self._dist_to_goal()
        self._prev_g = self.truck.g_rad
        self._last_reward = 0.0
        self._last_collided = False
        self._last_succeeded = False
        self._ep_reward = 0.0
        self.trajectory_points.clear()

        if self.render_mode == 'human':
            self._init_pygame()
            self._render_human()

        return self._get_obs(), {}

    def step(self, action):
        target_steering = float(np.clip(action[0], -1.0, 1.0)) * MAX_STEERING_DEG
        steering = np.clip(target_steering,
                           self._prev_steering - MAX_STEERING_RATE * DT,
                           self._prev_steering + MAX_STEERING_RATE * DT)
        self._prev_steering = steering

        self.truck.set_steering(steering)
        self.truck.v_mps = FIXED_SPEED
        self.truck.drive(DT)
        if abs(self.truck.v_mps) < 0.4:
            self.truck.v_mps = FIXED_SPEED

        if self._show_trajectory:
            self.trajectory_points.append((self.truck.x_m * SCALE, self.truck.y_m * SCALE))

        self.step_count += 1

        prev_g = self._prev_g
        self._prev_g = self.truck.g_rad

        jackknifed = abs(self.truck.g_rad) > np.deg2rad(JACKKNIFE_DEG)
        hard_collision = self._check_hard_collision()
        soft_boundary = self._check_soft_boundary()
        succeeded = self._is_success()

        terminated = succeeded or hard_collision or jackknifed
        truncated = self.step_count >= self._stage_config['max_steps']

        reward = self._compute_reward(soft_boundary, hard_collision, jackknifed, succeeded)

        self._last_reward = reward
        self._last_collided = hard_collision
        self._last_succeeded = succeeded
        self._ep_reward += reward

        obs = self._get_obs()
        self._prev_dist = self._dist_to_goal()

        if self.render_mode == 'human':
            self._render_human()

        return obs, reward, terminated, truncated, {
            'collided': hard_collision,
            'succeeded': succeeded,
            'jackknifed': jackknifed,
            'soft_boundary': soft_boundary,
        }

    def _dist_to_goal(self):
        dx = self.truck.x_m - self.goal_x
        dy = self.truck.y_m - self.goal_y
        return float(np.sqrt(dx * dx + dy * dy))

    def _compute_reward(self, soft_boundary, hard_collision, jackknifed, succeeded):
        dx = self.goal_x - self.truck.x_m
        dy = self.goal_y - self.truck.y_m
        dist = np.sqrt(dx * dx + dy * dy)

        stage = self.curriculum_stage
        max_g = self._stage_config.get('max_g_rad', 0.05)
        reward = (self._prev_dist - dist) * REWARD_DIST_COEF

        if stage <= 1:
            reward = -dist * 0.1
            if hard_collision:
                reward -= 100.0
            if succeeded:
                reward += 500.0
            return reward

        elif stage <= 3:
            reward = -dist * 0.1
            h_err = abs(self.truck.h_rad - self._goal_heading)
            h_err = min(h_err, 2 * np.pi - h_err)
            reward -= h_err * 0.3
            g_penalty = max(0, abs(self.truck.g_rad) - max_g * 0.5)
            reward -= g_penalty * 1.0
            if hard_collision:
                reward -= 100.0
            if succeeded:
                reward += 500.0
            return reward

        else:
            reward = -dist * 0.1
            h_err = abs(self.truck.h_rad - self._goal_heading)
            h_err = min(h_err, 2 * np.pi - h_err)
            reward -= h_err * 0.3
            g_diff = abs(self.truck.g_rad) - max_g
            if g_diff > 0:
                reward -= g_diff * 3.0
            elif abs(self.truck.g_rad) > 0.05:
                reward += 0.05
            if soft_boundary:
                reward -= REWARD_SOFT_BOUNDARY
            if hard_collision:
                reward -= 100.0
            if succeeded:
                reward += 500.0
            return reward

    def compute_reward(self, achieved_goal, desired_goal, info):
        ag = np.asarray(achieved_goal)
        dg = np.asarray(desired_goal)
        dx = ag[..., 0] - dg[..., 0]
        dy = ag[..., 1] - dg[..., 1]
        dist = np.sqrt(dx * dx + dy * dy)
        reward = -dist
        reward = np.asarray(reward)
        if reward.ndim == 0:
            return float(reward)
        return reward

    def _is_success(self):
        dx = abs(self.truck.x_m - self.goal_x)
        dy = abs(self.truck.y_m - self.goal_y)
        h_err = abs(self.truck.h_rad + np.pi / 2)
        h_err = min(h_err, 2 * np.pi - h_err)
        goal_tol = self._stage_config.get('goal_tol', GOAL_POS_TOL_M)
        return (dx < goal_tol and dy < goal_tol
                and h_err < GOAL_HEADING_TOL_DEG * np.pi / 180
                and abs(self.truck.g_rad) < GOAL_G_TOL_DEG * np.pi / 180)

    def _check_hard_collision(self):
        corners = self._get_all_corners()
        rl, rr = self.road.get_road_boundaries()
        for cx, cy in corners:
            if cx < rl or cx > rr:
                return True
            if cy < ROAD_Y_START or cy > ROAD_Y_END:
                return True
        return False

    def _check_soft_boundary(self):
        corners = self._get_all_corners()
        rl, rr = self.road.get_road_boundaries()
        for cx, cy in corners:
            if cx < rl + SOFT_BOUNDARY_MARGIN or cx > rr - SOFT_BOUNDARY_MARGIN:
                return True
        return False

    def _get_all_corners(self):
        t = self.truck
        cx, cy = t.x_m, t.y_m
        h = t.h_rad
        g = t.g_rad
        corners = []

        hl = (t.truckWheelbase + 2 * t.truckMargin) / 2
        hw = t.truckWidth / 2
        for dx in [-hl, hl]:
            for dy in [-hw, hw]:
                corners.append((
                    cx + dx * np.cos(h) - dy * np.sin(h),
                    cy + dx * np.sin(h) + dy * np.cos(h),
                ))

        cnx = cx + (-hl + t.truckMargin) * np.cos(h)
        cny = cy + (-hl + t.truckMargin) * np.sin(h)
        hg = h + g
        tl = (t.trailerWheelbase + t.trailerMarginFront + t.trailerMarginBack) / 2
        tw = t.trailerWidth / 2
        tcx = cnx + (t.trailerMarginFront - tl) * np.cos(hg)
        tcy = cny + (t.trailerMarginFront - tl) * np.sin(hg)
        for dx in [-tl, tl]:
            for dy in [-tw, tw]:
                corners.append((
                    tcx + dx * np.cos(hg) - dy * np.sin(hg),
                    tcy + dx * np.sin(hg) + dy * np.cos(hg),
                ))
        return corners

    def render(self):
        if self.render_mode == 'human':
            self._render_human()
        elif self.render_mode == 'rgb_array':
            return self._render_rgb_array()

    def _get_obs(self):
        rel_x = self.truck.x_m - self.goal_x
        rel_y = self.truck.y_m - self.goal_y
        rel_h = self.truck.h_rad - self._goal_heading
        return {
            'observation': np.array([
                rel_x, rel_y,
                np.cos(rel_h), np.sin(rel_h),
                np.cos(self.truck.g_rad), np.sin(self.truck.g_rad),
                self.truck.v_mps,
                self.truck.g_rad,
                self._prev_steering / MAX_STEERING_DEG,
                self.truck.x_m,
                self.truck.y_m,
            ], dtype=np.float32),
            'achieved_goal': np.array([
                self.truck.x_m, self.truck.y_m,
                np.cos(self.truck.h_rad), np.sin(self.truck.h_rad),
                self.truck.g_rad,
            ], dtype=np.float32),
            'desired_goal': np.array([
                self.goal_x, self.goal_y,
                np.cos(self._goal_heading), np.sin(self._goal_heading),
                0.0,
            ], dtype=np.float32),
        }

    def _init_pygame(self):
        if self.screen is not None:
            return
        self.screen, _, _ = init_pygame_screen(
            fullscreen=self.fullscreen,
            caption='Truck Parking DDPG+HER')
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 22)

    def _render_human(self):
        if self.screen is None:
            return
        self.screen.fill(WHITE)
        self.road.draw(self.screen)
        _draw_trajectory_fn(self.screen, self.trajectory_points)
        _draw_truck_fn(self.screen, self.truck, SCALE)
        self._draw_info(self.screen)
        pygame.display.flip()
        self.clock.tick(self.metadata['render_fps'])

    def _draw_info(self, surface):
        t = self.truck
        dist = self._dist_to_goal()
        rl, rr = self.road.get_road_boundaries()
        lines = [
            f"Step:{self.step_count}/{MAX_STEPS}  EpR:{self._ep_reward:.1f}  R:{self._last_reward:.2f}",
            f"Dist:{dist:.2f}m  ({t.x_m:.1f},{t.y_m:.1f}) -> ({GOAL_X:.1f},{GOAL_Y:.1f})",
            f"H:{np.degrees(t.h_rad):.1f}  G:{np.degrees(t.g_rad):.1f}  Steer:{t.s_deg:.1f}  V:{t.v_mps:.2f}",
            f"Lane:[{rl:.1f},{rr:.1f}]  Col:{'Y' if self._last_collided else 'N'}  "
            f"Succ:{'Y' if self._last_succeeded else 'N'}",
        ]
        y = 10
        for line in lines:
            surface.blit(self.font.render(line, True, BLACK), (10, y))
            y += 22

    def _render_rgb_array(self):
        if self.screen is None:
            self._init_pygame()
        self._render_human()
        arr = pygame.surfarray.array3d(self.screen)
        return np.transpose(arr, (1, 0, 2))

    def close(self):
        if self.screen is not None:
            pygame.quit()
            self.screen = None


from gymnasium.envs.registration import register
register(
    id='TruckParking-v0',
    entry_point='truck_parking_env:TruckParkingEnv',
    max_episode_steps=MAX_STEPS,
)
