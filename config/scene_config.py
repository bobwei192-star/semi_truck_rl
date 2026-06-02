# scene_config.py
# 半挂车倒车入库场景 — 全局参数配置中心
# 所有路宽、起点、目标、公差、物理参数均集中于此，
# 供 truck_parking_env / dual_lane_road / manual_test / train 等模块统一引用。

import numpy as np

SCALE = 20.0               # 渲染比例：20 像素/米
DT = 0.05                  # 仿真步长：0.05 秒/步

# 道路几何参数
ROAD_CENTER_X = 56.0       # 道路中心线 X 坐标（米）— 1003 虚线位置，固定不变
ROAD_Y_START = 0.0         # 道路起点 Y 坐标（米）
ROAD_Y_END = 70.0          # 道路终点 Y 坐标（米）
LEFT_LANE_WIDTH = 6.0      # 左车道宽度（米）：1001 到 1003 的距离
RIGHT_LANE_WIDTH = 4.0     # 右车道宽度（米）：1003 到 1002 的距离
LANE_WIDTH = 8.0           # 兼容旧接口，= LEFT_LANE_WIDTH + RIGHT_LANE_WIDTH

# # 目标位置（倒车入库终点）
# GOAL_X = 57.5             # 目标点 X 坐标（米），靠左以避免碰撞
# GOAL_Y = 53.62             # 目标点 Y 坐标（米）
# GOAL_HEADING_TOL_DEG = 10.0   # 目标航向角容差（度）
# GOAL_G_TOL_DEG = 5.0          # 目标铰接角容差（度）
# GOAL_POS_TOL_M = 0.5          # 目标位置容差（米）

# # 起点位置（每次 reset 时在此基础上加随机扰动）
# START_X = 57.5                   # 起点 X 坐标（米），与 GOAL_X 一致
# START_Y = 9.91                # 起点 Y 坐标（米）





# 起点位置（库位起点，每次 reset 时在此基础上加随机扰动）
START_X =    98.16             # 起点 X 坐标（米），与 GOAL_X 一致
START_Y =   55          # 起点 Y 坐标（米）

# 目标位置（正向出库终点 - 库前位置）
GOAL_X =  94       # 目标点 X 坐标（米），靠左以避免碰撞
GOAL_Y =   5.57    # 目标点 Y 坐标（米）
GOAL_HEADING_TOL_DEG = 10.0   # 目标航向角容差（度）
GOAL_G_TOL_DEG = 5.0          # 目标铰接角容差（度）
GOAL_POS_TOL_M = 0.5          # 目标位置容差（米）




# 初始朝向配置（车头朝上 = -90°）
INIT_HEADING_RAD = -np.pi / 2   # 初始车头朝向（弧度），-90° 朝上（朝向屏幕上方，y轴负方向）
HEADING_NOISE_RAD = 0.03        # 初始朝向随机扰动（弧度）
INIT_G_RAD = 0.0                # 初始铰接角（弧度）
G_NOISE_RAD = 0.01              # 初始铰接角随机扰动（弧度）

# 为了向后兼容，保留旧变量名
START_HEADING_OFFSET_RAD = 0.0  # 起点航向角额外偏移（弧度）
START_G_INIT_RAD = 0.0          # 起点铰接角额外偏移（弧度）

# 起点位置随机扰动（米）
START_POS_NOISE_M = 0.3         # 起点位置随机扰动范围（米）

# 库位标线几何（用于 dual_lane_road 绘制虚线）
PARKING_LINE_Y = 2.5       # 库位入口线 Y 坐标（米）
PARKING_TOP_Y = 50.5      # 库位上边界线 Y 坐标（米）
PARKING_BTM_Y = 69.0       # 库位下边界线 Y 坐标（米）

# 车辆控制参数
MAX_STEPS = 1500           # 单 episode 最大步数
FIXED_SPEED = 1.0          # 固定正向行驶速度（米/秒），正号表示前进
MAX_STEERING_DEG = 35    # 最大转向角（度）
MAX_STEERING_RATE = 60.0   # 转向速率限制（度/秒）

# 奖励函数参数
REWARD_DIST_COEF = 10.0         # 距离奖励系数
REWARD_HEADING_COEF = 0.5     # 朝向引导奖励系数
REWARD_SOFT_BOUNDARY = 1.0      # 软边界惩罚
REWARD_JACKKNIFE = 0.1        # 铰接角过大惩罚
REWARD_COLLISION = 100.0      # 碰撞惩罚
REWARD_SUCCESS = 500.0        # 成功奖励

# 人类经验奖励参数
REWARD_STEER_CONSISTENT = 0.1   # 方向盘方向一致奖励
REWARD_STEER_INCONSISTENT = 0.05 # 方向盘方向不一致惩罚
REWARD_STEER_MAG = 0.05       # 方向盘幅度合适奖励
REWARD_G_RATE_HIGH = 0.1      # 铰接角变化率过大惩罚
REWARD_G_DECREASE = 0.1       # 铰接角减小奖励

# 铰接角阈值
JACKKNIFE_DEG = 85.0          # 折叠判定阈值（度）
G_CONSISTENCY_THRESHOLD = 0.02  # 方向盘一致性铰接角阈值（弧度）
G_MAG_THRESHOLD = 0.05        # 方向盘幅度铰接角阈值（弧度）
G_RATE_THRESHOLD = 0.5        # 铰接角变化率阈值（弧度/秒）

# 软边界缓冲（米）
SOFT_BOUNDARY_MARGIN = 0.3    # 软边界缓冲距离（米）

# 训练总步数
TRAIN_TOTAL_STEPS = 500000  # DDPG+HER 总训练步数

# Curriculum Learning 阶段配置 - 正向出库版本
# 关键变化：从直线出库 → 小角度曲线出库
# 每个阶段逐步增加侧向位移（ΔX）和铰接角控制难度
#
# 半挂车正向出库运动学特点：
# - 前进时，车头转向带动挂车同向偏移
# - 相对倒车更容易控制
#
# 阶段设计策略：
# Stage 0-1: 直线出库，训练基础距离控制和 g_rad ≈ 0
# Stage 2-3: 小角度曲线出库，学习产生和控制铰接角
# Stage 4:   完整出库（从库位到库前）
CURRICULUM_STAGES = [
    # ========== Stage 0: 短距离直线出库（最简单）==========
    # 起点终点 X 相同，Y 差距 10m
    # 目标：训练基本的距离控制和碰撞避免
    # 铰接角 g_rad 应该接近 0（保持直线）
    {
        'start_x': 57.5,
        'start_y': 43.62,
        'goal_x': 57.5,
        'goal_y': 33.62,
        'max_steps': 200,
        'goal_tol': 1.0,
        'max_g_rad': 0.05,
        'switch_threshold': 50,
        'description': '短距离直线出库，ΔX=0m, ΔY=10m',
        'difficulty': '简单',
        'key_skill': '基础距离控制',
    },

    # ========== Stage 1: 中等距离直线出库 ==========
    # Y 差距增加到 20m
    # 目标：训练中等距离稳定控制
    {
        'start_x': 57.5,
        'start_y': 53.62,
        'goal_x': 57.5,
        'goal_y': 33.62,
        'max_steps': 400,
        'goal_tol': 0.8,
        'max_g_rad': 0.05,
        'switch_threshold': 100,
        'description': '中等距离直线出库，ΔX=0m, ΔY=20m',
        'difficulty': '较易',
        'key_skill': '中等距离稳定控制',
    },

    # ========== Stage 2: 长距离直线出库 ==========
    # Y 差距增加到 30m
    # 目标：训练长距离稳定控制
    {
        'start_x': 57.5,
        'start_y': 53.62,
        'goal_x': 57.5,
        'goal_y': 23.62,
        'max_steps': 500,
        'goal_tol': 0.8,
        'max_g_rad': 0.05,
        'switch_threshold': 100,
        'description': '长距离直线出库，ΔX=0m, ΔY=30m',
        'difficulty': '中等',
        'key_skill': '长距离稳定控制',
    },

    # ========== Stage 3: 小角度曲线出库 ==========
    # 起点左偏 1.5m，需要向右转向出库
    # 目标：学习产生小铰接角
    {
        'start_x': 56.0,
        'start_y': 53.62,
        'goal_x': 57.5,
        'goal_y': 19.91,
        'max_steps': 600,
        'goal_tol': 0.6,
        'max_g_rad': 0.15,
        'switch_threshold': 100,
        'description': '小角度曲线出库，ΔX=-1.5m, ΔY=33.71m',
        'difficulty': '较难',
        'key_skill': '产生小铰接角',
    },

    # ========== Stage 4: 完整出库（最终目标）==========
    # 从库位起点到库前终点，Y差距约43.71m
    {
        'start_x': 57.5,
        'start_y': 53.62,
        'goal_x': 57.5,
        'goal_y': 9.91,
        'max_steps': 800,
        'goal_tol': 0.5,
        'max_g_rad': 0.20,
        'switch_threshold': None,
        'description': '完整出库，ΔX=0m, ΔY=43.71m',
        'difficulty': '难',
        'key_skill': '完全出库控制',
    },
]
