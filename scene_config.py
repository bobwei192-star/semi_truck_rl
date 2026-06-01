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

# 目标位置（倒车入库终点）
GOAL_X = 57.5             # 目标点 X 坐标（米），靠左以避免碰撞
GOAL_Y = 53.62             # 目标点 Y 坐标（米）
GOAL_HEADING_TOL_DEG = 10.0   # 目标航向角容差（度）
GOAL_G_TOL_DEG = 5.0          # 目标铰接角容差（度）
GOAL_POS_TOL_M = 0.5          # 目标位置容差（米）

# 起点位置（每次 reset 时在此基础上加随机扰动）
START_X = 57.5                   # 起点 X 坐标（米），与 GOAL_X 一致
START_Y = 9.91                # 起点 Y 坐标（米）

# 初始朝向配置（车头朝上 = -90°）
INIT_HEADING_RAD = -np.pi / 2   # 初始车头朝向（弧度），-90° 朝上
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
FIXED_SPEED = -1.0         # 固定倒车速度（米/秒），负号表示倒车
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

# Curriculum Learning 阶段配置 - 曲线倒车入库版本
# 关键变化：从直线倒车 → 小角度曲线 → 大角度曲线
# 每个阶段逐步增加侧向位移（ΔX）和铰接角控制难度
#
# 半挂车倒车入库运动学特点：
# - 倒车时，车头转向带动挂车反向偏移
# - 要让车头向右转（goal_x > start_x），挂车会向左偏移
# - 需要主动产生并控制铰接角 g_rad
#
# 阶段设计策略：
# Stage 0-1: 直线倒车，训练基础距离控制和 g_rad ≈ 0
# Stage 2-3: 小角度曲线（ΔX=1.5~2.5m），学习产生 g_rad
# Stage 4-5: 中等角度曲线（ΔX=3~4m），学习控制 g_rad
# Stage 6:   完整入库（ΔX=4.5m），终极挑战
CURRICULUM_STAGES = [
    # ========== Stage 0: 直线倒车（最简单）==========
    # 起点终点 X 相同，Y 差距 8.62m
    # 目标：训练基本的距离控制和碰撞避免
    # 铰接角 g_rad 应该接近 0（保持直线）
    {
        'start_x': 57.5,
        'start_y': 45.0,
        'goal_x': 57.5,
        'goal_y': 53.62,
        'max_steps': 200,
        'goal_tol': 1.0,
        'max_g_rad': 0.05,
        'switch_threshold': 50,
        'description': '直线倒车，ΔX=0m, ΔY=8.62m',
        'difficulty': '简单',
        'key_skill': '基础距离控制',
    },

    # ========== Stage 1: 长距离直线倒车 ==========
    # Y 差距增加到 23.71m
    # 目标：训练长时间稳定控制
    {
        'start_x': 57.5,
        'start_y': 30.0,
        'goal_x': 57.5,
        'goal_y': 53.62,
        'max_steps': 400,
        'goal_tol': 0.8,
        'max_g_rad': 0.05,
        'switch_threshold': 100,
        'description': '长距离直线，ΔX=0m, ΔY=23.62m',
        'difficulty': '较易',
        'key_skill': '长时间稳定控制',
    },

    # ========== Stage 2: 小角度左曲线 ==========
    # 起点左偏 1.5m，需要向右转向入库
    # 目标：学习产生小铰接角
    {
        'start_x': 56.0,
        'start_y': 45.0,
        'goal_x': 57.5,
        'goal_y': 53.62,
        'max_steps': 300,
        'goal_tol': 0.8,
        'max_g_rad': 0.15,
        'switch_threshold': 80,
        'description': '小角度左曲线，ΔX=-1.5m, ΔY=8.62m',
        'difficulty': '中等',
        'key_skill': '产生小铰接角',
    },

    # ========== Stage 3: 中角度左曲线 + 长距离 ==========
    # 起点左偏 2.5m，Y 差距 23.62m
    # 目标：学习在长距离中保持曲线
    {
        'start_x': 55.0,
        'start_y': 30.0,
        'goal_x': 57.5,
        'goal_y': 53.62,
        'max_steps': 500,
        'goal_tol': 0.6,
        'max_g_rad': 0.20,
        'switch_threshold': 100,
        'description': '中角度左曲线，ΔX=-2.5m, ΔY=23.62m',
        'difficulty': '较难',
        'key_skill': '长距离曲线控制',
    },

    # ========== Stage 4: 大角度左曲线 ==========
    # 起点左偏 3.5m，需要较大铰接角
    # 目标：学习大铰接角控制
    {
        'start_x': 54.0,
        'start_y': 45.0,
        'goal_x': 57.5,
        'goal_y': 53.62,
        'max_steps': 500,
        'goal_tol': 0.5,
        'max_g_rad': 0.30,
        'switch_threshold': 80,
        'description': '大角度左曲线，ΔX=-3.5m, ΔY=8.62m',
        'difficulty': '难',
        'key_skill': '大铰接角控制',
    },

    # ========== Stage 5: 完整角度 + 长距离 ==========
    # 起点左偏 3.5m，Y 差距 43.71m
    # 目标：终极曲线控制
    {
        'start_x': 54.0,
        'start_y': 9.91,
        'goal_x': 57.5,
        'goal_y': 53.62,
        'max_steps': 800,
        'goal_tol': 0.5,
        'max_g_rad': 0.35,
        'switch_threshold': 100,
        'description': '完整曲线，ΔX=-3.5m, ΔY=43.71m',
        'difficulty': '很难',
        'key_skill': '终极曲线控制',
    },

    # ========== Stage 6: 极限入库（最终目标）==========
    # 起点左偏 4.5m（最大侧向位移）
    # 这是最难的场景
    {
        'start_x': 53.0,
        'start_y': 9.91,
        'goal_x': 57.5,
        'goal_y': 53.62,
        'max_steps': 1200,
        'goal_tol': 0.5,
        'max_g_rad': 0.40,
        'switch_threshold': None,
        'description': '极限入库，ΔX=-4.5m, ΔY=43.71m',
        'difficulty': '极难',
        'key_skill': '完全入库控制',
    },
]
