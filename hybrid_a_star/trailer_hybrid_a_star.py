# -*- coding: utf-8 -*-
"""
Hybrid A* path planning for trailer truck - Python implementation
Based on Atsushi Sakai's HybridAStarTrailer (Julia version)
Adapted to use project's vehicle model, map, config and rendering systems
"""

import numpy as np
import heapq
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.scene_config import (
    START_X, START_Y, GOAL_X, GOAL_Y, INIT_HEADING_RAD, INIT_G_RAD,
    DT, FIXED_SPEED, MAX_STEERING_DEG, MAX_STEPS,
    ROAD_CENTER_X, LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH,
    PARKING_TOP_Y, PARKING_BTM_Y,
)
from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle


# ============ Configuration ============
XY_GRID_RESOLUTION = 2.0  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(15.0)  # [rad]
GOAL_TYAW_TH = np.deg2rad(5.0)  # [rad]
MOTION_RESOLUTION = 0.1  # [m] path interpolate resolution
N_STEER = 20.0  # number of steer command
EXTEND_AREA = 5.0  # [m] map extend length
SKIP_COLLISION_CHECK = 4  # skip number for collision check

# Cost weights
SB_COST = 100.0  # switch back penalty cost
BACK_COST = 5.0  # backward penalty cost
STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
STEER_COST = 1.0  # steer angle penalty cost
JACKKNIF_COST = 200.0  # Jackknif cost
H_COST = 5.0  # Heuristic cost

# Vehicle parameters (from project's TruckTrailerOnAxle)
WB = 3.0  # [m] wheel base
LT = 11.0  # [m] length of trailer (trailerWheelbase)
MAX_STEER = np.deg2rad(35.0)  # [rad] maximum steering angle


class Node:
    """Hybrid A* search node"""
    def __init__(self, xind, yind, yawind, direction, x, y, yaw, yaw1,
                 directions, steer, cost, pind):
        self.xind = xind  # x index
        self.yind = yind  # y index
        self.yawind = yawind  # yaw index
        self.direction = direction  # moving direction forward:True, backward:False
        self.x = x  # x position [m]
        self.y = y  # y position [m]
        self.yaw = yaw  # yaw angle [rad]
        self.yaw1 = yaw1  # trailer yaw angle [rad]
        self.directions = directions  # directions of each point
        self.steer = steer  # steer input
        self.cost = cost  # cost
        self.pind = pind  # parent index


class Config:
    """Configuration for hybrid A* search grid"""
    def __init__(self, minx, miny, minyaw, minyawt, maxx, maxy, maxyaw, maxyawt,
                 xw, yw, yaww, yawtw, xyreso, yawreso):
        self.minx = minx
        self.miny = miny
        self.minyaw = minyaw
        self.minyawt = minyawt
        self.maxx = maxx
        self.maxy = maxy
        self.maxyaw = maxyaw
        self.maxyawt = maxyawt
        self.xw = xw
        self.yw = yw
        self.yaww = yaww
        self.yawtw = yawtw
        self.xyreso = xyreso
        self.yawreso = yawreso


class Path:
    """Planned path"""
    def __init__(self, x, y, yaw, yaw1, direction, cost):
        self.x = x  # x position [m]
        self.y = y  # y position [m]
        self.yaw = yaw  # yaw angle [rad]
        self.yaw1 = yaw1  # trailer yaw angle [rad]
        self.direction = direction  # direction forward: True, backward: False
        self.cost = cost  # total cost


def pi_2_pi(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


def calc_trailer_yaw_from_xyyaw(x, y, yaw, init_tyaw, steps):
    """Calculate trailer yaw from x, y, yaw lists"""
    tyaw = [init_tyaw]
    for i in range(1, len(x)):
        tyaw.append(tyaw[i-1] + steps[i-1] / LT * np.sin(yaw[i-1] - tyaw[i-1]))
    return tyaw


def trailer_motion_model(x, y, yaw0, yaw1, D, d, L, delta):
    """Motion model for trailer"""
    x += D * np.cos(yaw0)
    y += D * np.sin(yaw0)
    yaw0 += D / L * np.tan(delta)
    yaw1 += D / d * np.sin(yaw0 - yaw1)
    return x, y, yaw0, yaw1


def check_trailer_collision(ox, oy, x, y, yaw0, yaw1):
    """Simple collision check for trailer"""
    # Simplified collision check - just check if within road boundaries
    # In a full implementation, this would check against obstacles
    left_bound = ROAD_CENTER_X - LEFT_LANE_WIDTH
    right_bound = ROAD_CENTER_X + RIGHT_LANE_WIDTH
    
    for ix, iy in zip(x, y):
        if ix < left_bound - 2.0 or ix > right_bound + 2.0:
            return False
        if iy < PARKING_TOP_Y - 5.0 or iy > PARKING_BTM_Y + 5.0:
            return False
    
    # Check jackknife condition
    for g in [y0 - y1 for y0, y1 in zip(yaw0, yaw1)]:
        if abs(pi_2_pi(g)) > np.deg2rad(85.0):
            return False
    
    return True


def calc_motion_inputs():
    """Calculate motion inputs (steering angles and directions)"""
    up = [i for i in np.arange(MAX_STEER / N_STEER, MAX_STEER + MAX_STEER / N_STEER,
                               MAX_STEER / N_STEER)]
    u = [0.0] + up + [-i for i in up]
    d = [1.0] * len(u) + [-1.0] * len(u)
    u = u + u
    return u, d


def calc_next_node(current, c_id, u, d, c):
    """Calculate next node from current node with control input"""
    arc_l = XY_GRID_RESOLUTION * 1.5
    
    nlist = int(np.floor(arc_l / MOTION_RESOLUTION)) + 1
    xlist = [0.0] * nlist
    ylist = [0.0] * nlist
    yawlist = [0.0] * nlist
    yaw1list = [0.0] * nlist
    
    xlist[0] = current.x[-1] + d * MOTION_RESOLUTION * np.cos(current.yaw[-1])
    ylist[0] = current.y[-1] + d * MOTION_RESOLUTION * np.sin(current.yaw[-1])
    yawlist[0] = pi_2_pi(current.yaw[-1] + d * MOTION_RESOLUTION / WB * np.tan(u))
    yaw1list[0] = pi_2_pi(current.yaw1[-1] + d * MOTION_RESOLUTION / LT * np.sin(current.yaw[-1] - current.yaw1[-1]))
    
    for i in range(nlist - 1):
        xlist[i+1] = xlist[i] + d * MOTION_RESOLUTION * np.cos(yawlist[i])
        ylist[i+1] = ylist[i] + d * MOTION_RESOLUTION * np.sin(yawlist[i])
        yawlist[i+1] = pi_2_pi(yawlist[i] + d * MOTION_RESOLUTION / WB * np.tan(u))
        yaw1list[i+1] = pi_2_pi(yawlist[i+1] + d * MOTION_RESOLUTION / LT * np.sin(yawlist[i] - yaw1list[i]))
    
    xind = round(xlist[-1] / c.xyreso)
    yind = round(ylist[-1] / c.xyreso)
    yawind = round(yawlist[-1] / c.yawreso)
    
    addedcost = 0.0
    if d > 0:
        direction = True
        addedcost += abs(arc_l)
    else:
        direction = False
        addedcost += abs(arc_l) * BACK_COST
    
    # Switch back penalty
    if direction != current.direction:
        addedcost += SB_COST
    
    # Steer penalty
    addedcost += STEER_COST * abs(u)
    
    # Steer change penalty
    addedcost += STEER_CHANGE_COST * abs(current.steer - u)
    
    # Jackknife cost
    addedcost += JACKKNIF_COST * sum(abs(pi_2_pi(y0 - y1)) for y0, y1 in zip(yawlist, yaw1list))
    
    cost = current.cost + addedcost
    
    directions = [direction] * len(xlist)
    node = Node(xind, yind, yawind, direction, xlist, ylist, yawlist, yaw1list,
                directions, u, cost, c_id)
    
    return node


def verify_index(node, c, ox, oy, inityaw1):
    """Verify if node is valid"""
    # Check map overflow
    if (node.xind - c.minx) >= c.xw:
        return False
    elif (node.xind - c.minx) <= 0:
        return False
    if (node.yind - c.miny) >= c.yw:
        return False
    elif (node.yind - c.miny) <= 0:
        return False
    
    # Check collision
    steps = [MOTION_RESOLUTION if d else -MOTION_RESOLUTION for d in node.directions]
    yaw1 = calc_trailer_yaw_from_xyyaw(node.x, node.y, node.yaw, inityaw1, steps)
    ind = list(range(0, len(node.x), SKIP_COLLISION_CHECK))
    if not check_trailer_collision(ox, oy, 
                                   [node.x[i] for i in ind],
                                   [node.y[i] for i in ind],
                                   [node.yaw[i] for i in ind],
                                   [yaw1[i] for i in ind]):
        return False
    
    return True


def is_same_grid(node1, node2):
    """Check if two nodes are in the same grid"""
    if node1.xind != node2.xind:
        return False
    if node1.yind != node2.yind:
        return False
    if node1.yawind != node2.yawind:
        return False
    return True


def calc_index(node, c):
    """Calculate unique index for node"""
    ind = ((node.yawind - c.minyaw) * c.xw * c.yw +
           (node.yind - c.miny) * c.xw +
           (node.xind - c.minx))
    
    # 4D grid
    yaw1ind = round(node.yaw1[-1] / c.yawreso)
    ind += (yaw1ind - c.minyawt) * c.xw * c.yw * c.yaww
    
    return int(ind)


def calc_config(ox, oy, xyreso, yawreso):
    """Calculate configuration for search grid"""
    min_x_m = min(ox) - EXTEND_AREA
    min_y_m = min(oy) - EXTEND_AREA
    max_x_m = max(ox) + EXTEND_AREA
    max_y_m = max(oy) + EXTEND_AREA
    
    ox = ox + [min_x_m, max_x_m]
    oy = oy + [min_y_m, max_y_m]
    
    minx = round(min_x_m / xyreso)
    miny = round(min_y_m / xyreso)
    maxx = round(max_x_m / xyreso)
    maxy = round(max_y_m / xyreso)
    
    xw = int(maxx - minx)
    yw = int(maxy - miny)
    
    minyaw = round(-np.pi / yawreso) - 1
    maxyaw = round(np.pi / yawreso)
    yaww = int(maxyaw - minyaw)
    
    minyawt = minyaw
    maxyawt = maxyaw
    yawtw = yaww
    
    config = Config(minx, miny, minyaw, minyawt, maxx, maxy, maxyaw, maxyawt,
                    xw, yw, yaww, yawtw, xyreso, yawreso)
    
    return config


def calc_cost(n, h_dp, ngoal, c):
    """Calculate total cost for node"""
    key = (n.xind - c.minx, n.yind - c.miny)
    if key in h_dp:
        return n.cost + H_COST * h_dp[key]
    else:
        # Fallback: use Euclidean distance if not in heuristic map
        h = np.sqrt((n.xind - ngoal.xind)**2 + (n.yind - ngoal.yind)**2) * c.xyreso
        return n.cost + H_COST * h


def calc_holonomic_with_obstacle_heuristic(gnode, ox, oy, xyreso):
    """Calculate holonomic heuristic with obstacles using grid A*"""
    # Simplified version - just use Euclidean distance
    # In full implementation, use grid_a_star
    gx = int(round(gnode.x[-1] / xyreso))
    gy = int(round(gnode.y[-1] / xyreso))
    
    h_dp = {}
    min_x = int(min(ox) / xyreso) - 10
    max_x = int(max(ox) / xyreso) + 10
    min_y = int(min(oy) / xyreso) - 10
    max_y = int(max(oy) / xyreso) + 10
    
    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            h_dp[(x, y)] = np.sqrt((x - gx) ** 2 + (y - gy) ** 2) * xyreso
    
    return h_dp


def get_final_path(closed, ngoal, nstart, c):
    """Extract final path from closed set"""
    rx = list(reversed(ngoal.x))
    ry = list(reversed(ngoal.y))
    ryaw = list(reversed(ngoal.yaw))
    ryaw1 = list(reversed(ngoal.yaw1))
    rdirection = list(reversed(ngoal.directions))
    nid = ngoal.pind
    finalcost = ngoal.cost
    
    while True:
        n = closed[nid]
        rx.extend(reversed(n.x))
        ry.extend(reversed(n.y))
        ryaw.extend(reversed(n.yaw))
        ryaw1.extend(reversed(n.yaw1))
        rdirection.extend(reversed(n.directions))
        nid = n.pind
        if is_same_grid(n, nstart):
            break
    
    rx = list(reversed(rx))
    ry = list(reversed(ry))
    ryaw = list(reversed(ryaw))
    ryaw1 = list(reversed(ryaw1))
    rdirection = list(reversed(rdirection))
    
    # Adjust first direction
    if len(rdirection) > 1:
        rdirection[0] = rdirection[1]
    
    path = Path(rx, ry, ryaw, ryaw1, rdirection, finalcost)
    
    return path


def calc_hybrid_astar_path(sx, sy, syaw, syaw1, gx, gy, gyaw, gyaw1, ox, oy,
                           xyreso=XY_GRID_RESOLUTION, yawreso=YAW_GRID_RESOLUTION):
    """
    Calculate Hybrid A* path for trailer truck
    
    sx: start x position [m]
    sy: start y position [m]
    syaw: start yaw angle [rad]
    syaw1: start trailer yaw angle [rad]
    gx: goal x position [m]
    gy: goal y position [m]
    gyaw: goal yaw angle [rad]
    gyaw1: goal trailer yaw angle [rad]
    ox: x position list of obstacles [m]
    oy: y position list of obstacles [m]
    xyreso: grid resolution [m]
    yawreso: yaw angle resolution [rad]
    """
    syaw = pi_2_pi(syaw)
    gyaw = pi_2_pi(gyaw)
    
    c = calc_config(ox, oy, xyreso, yawreso)
    
    nstart = Node(round(sx / xyreso), round(sy / xyreso), round(syaw / yawreso), True,
                  [sx], [sy], [syaw], [syaw1], [True], 0.0, 0.0, -1)
    ngoal = Node(round(gx / xyreso), round(gy / xyreso), round(gyaw / yawreso), True,
                 [gx], [gy], [gyaw], [gyaw1], [True], 0.0, 0.0, -1)
    
    h_dp = calc_holonomic_with_obstacle_heuristic(ngoal, ox, oy, xyreso)
    
    open_set = {}
    closed_set = {}
    open_set[calc_index(nstart, c)] = nstart
    
    # Priority queue: (cost, index, node)
    pq = []
    heapq.heappush(pq, (calc_cost(nstart, h_dp, ngoal, c), calc_index(nstart, c)))
    
    u, d = calc_motion_inputs()
    nmotion = len(u)
    
    while True:
        if len(open_set) == 0:
            print("Error: Cannot find path, No open set")
            return None
        
        # Get node with minimum cost
        _, c_id = heapq.heappop(pq)
        while c_id not in open_set and pq:
            _, c_id = heapq.heappop(pq)
        
        if c_id not in open_set:
            print("Error: Cannot find path, No open set")
            return None
        
        current = open_set[c_id]
        
        # Move current node from open to closed
        del open_set[c_id]
        closed_set[c_id] = current
        
        # Check if reached goal (simplified - check if close enough)
        if (abs(current.x[-1] - gx) < xyreso and
            abs(current.y[-1] - gy) < xyreso and
            abs(pi_2_pi(current.yaw[-1] - gyaw)) < yawreso):
            print("Goal reached!")
            # Create goal node
            fnode = current
            break
        
        inityaw1 = current.yaw1[0]
        
        for i in range(nmotion):
            node = calc_next_node(current, c_id, u[i], d[i], c)
            
            if not verify_index(node, c, ox, oy, inityaw1):
                continue
            
            node_ind = calc_index(node, c)
            
            # If already in closed set, skip
            if node_ind in closed_set:
                continue
            
            if node_ind not in open_set:
                open_set[node_ind] = node
                heapq.heappush(pq, (calc_cost(node, h_dp, ngoal, c), node_ind))
            else:
                if open_set[node_ind].cost > node.cost:
                    open_set[node_ind] = node
    
    print(f"Final expand node: {len(open_set) + len(closed_set)}")
    
    path = get_final_path(closed_set, fnode, nstart, c)
    
    return path


def plan_path_for_project():
    """Plan path using project configuration"""
    # Start and goal from project config
    sx = START_X
    sy = START_Y
    syaw = INIT_HEADING_RAD
    syaw1 = INIT_HEADING_RAD  # Initially aligned
    
    gx = GOAL_X
    gy = GOAL_Y
    gyaw = INIT_HEADING_RAD  # Same heading as start
    gyaw1 = INIT_HEADING_RAD  # Same trailer heading
    
    # Obstacles (road boundaries)
    ox = []
    oy = []
    
    # Left road boundary
    left_bound = ROAD_CENTER_X - LEFT_LANE_WIDTH
    for y in np.arange(PARKING_TOP_Y - 5, PARKING_BTM_Y + 5, 0.5):
        ox.append(left_bound)
        oy.append(y)
    
    # Right road boundary
    right_bound = ROAD_CENTER_X + RIGHT_LANE_WIDTH
    for y in np.arange(PARKING_TOP_Y - 5, PARKING_BTM_Y + 5, 0.5):
        ox.append(right_bound)
        oy.append(y)
    
    # Parking area boundaries
    for x in np.arange(left_bound, right_bound, 0.5):
        ox.append(x)
        oy.append(PARKING_TOP_Y)
        ox.append(x)
        oy.append(PARKING_BTM_Y)
    
    print(f"Planning path from ({sx}, {sy}) to ({gx}, {gy})")
    print(f"Start yaw: {np.rad2deg(syaw):.1f} deg, Goal yaw: {np.rad2deg(gyaw):.1f} deg")
    
    path = calc_hybrid_astar_path(sx, sy, syaw, syaw1, gx, gy, gyaw, gyaw1, ox, oy)
    
    if path is not None:
        print(f"Path found! Length: {len(path.x)} points, Cost: {path.cost:.2f}")
        print(f"Start: ({path.x[0]:.2f}, {path.y[0]:.2f})")
        print(f"End: ({path.x[-1]:.2f}, {path.y[-1]:.2f})")
    else:
        print("Failed to find path!")
    
    return path


if __name__ == "__main__":
    path = plan_path_for_project()
