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







# 起点位置（倒车起点，每次 reset 时在此基础上加随机扰动）
START_X = 56.25              # 起点 X 坐标（米）
START_Y = 9.91               # 起点 Y 坐标（米）
#起点朝向 也在这里规定 为 -90°
STARTSTART_HEADING_RAD = -np.pi / 2   # 起点航向角（弧度），-90° 朝上（朝向屏幕上方，y轴负方向）

# 目标位置（倒车入库终点 - 库位中心）
GOAL_X = 58.25               # 目标点 X 坐标（米）
GOAL_Y = 53.56               # 目标点 Y 坐标（米）
GOAL_HEADING_TOL_DEG = 10.0  # 目标航向角容差（度）
GOAL_G_TOL_DEG = 5.0         # 目标铰接角容差（度）
GOAL_POS_TOL_M = 0.5         # 目标位置容差（米）

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
FIXED_SPEED = -1.0         # 固定倒车速度（米/秒），负号表示倒车
MAX_STEERING_DEG = 35      # 最大转向角（度）
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

# Curriculum Learning 阶段配置 - 倒车入库版本（8阶段）
# 核心思想：从简单到复杂，逐步增加难度
# 每阶段只增加一个难度维度，确保智能体能稳定学习
#
# 倒车运动学特点：
# - 倒车时铰接角有自发增大趋势（不稳定系统）
# - 需要主动控制铰接角在稳定范围内
# - 优先训练铰接角稳定，再训练位置精度
#
# 阶段设计策略：
# Stage 0-2: 短距离直线倒车，训练铰接角稳定（核心）
# Stage 3-4: 引入横向偏移，学习曲线倒车
# Stage 5-7: 逐步增加距离和偏移量，达到完整倒车入库

CURRICULUM_STAGES = [
    # ========== Stage 0: 静态对齐（最简单）==========
    # 目的：训练智能体学会将铰接角保持在0附近
    # 距离：1米（Y轴向下坐标系，从小Y向大Y倒车）
    # 关键技能：铰接角归零
    {
        'start_x': 58.25,
        'start_y': 50.56,      # Y较小（屏幕上方）
        'goal_x': 58.25,
        'goal_y': 51.56,       # Y较大（屏幕下方）
        'max_steps': 50,
        'goal_tol': 1.0,       # 放宽位置容差，重点是铰接角
        'max_g_rad': 0.05,
        'success_threshold': 0.85,
        'description': '静态对齐（铰接角归零），距离1m',
        'difficulty': '最简单',
        'key_skill': '铰接角稳定',
    },

    # ========== Stage 1: 短距离直线倒车 ==========
    # 目的：在保持铰接角稳定的前提下，学习距离控制
    # 距离：3米（Y轴向下坐标系，从小Y向大Y倒车）
    # 关键技能：距离控制
    {
        'start_x': 58.25,
        'start_y': 50.56,      # Y较小（屏幕上方，库位）
        'goal_x': 58.25,
        'goal_y': 53.56,       # Y较大（屏幕下方，库位外）
        'max_steps': 100,
        'goal_tol': 0.5,
        'max_g_rad': 0.08,
        'success_threshold': 0.80,
        'description': '短距离直线倒车，距离3m',
        'difficulty': '简单',
        'key_skill': '距离控制',
    },

    # ========== Stage 2: 中距离直线倒车 ==========
    # 目的：延长控制时间，训练稳定性
    # 距离：8米（Y轴向下坐标系，从小Y向大Y倒车）
    # 关键技能：长期稳定控制
    {
        'start_x': 58.25,
        'start_y': 50.56,      # Y较小（屏幕上方）
        'goal_x': 58.25,
        'goal_y': 58.56,       # Y较大（屏幕下方）
        'max_steps': 200,
        'goal_tol': 0.5,
        'max_g_rad': 0.10,
        'success_threshold': 0.75,
        'description': '中距离直线倒车，距离8m',
        'difficulty': '较易',
        'key_skill': '长期稳定控制',
    },

    # ========== Stage 3: 长距离直线倒车 ==========
    # 目的：训练长期稳定控制，开始引入边界约束
    # 距离：15米（Y轴向下坐标系，从小Y向大Y倒车）
    # 关键技能：边界约束
    {
        'start_x': 58.25,
        'start_y': 50.56,      # Y较小（屏幕上方）
        'goal_x': 58.25,
        'goal_y': 65.56,       # Y较大（屏幕下方）
        'max_steps': 400,
        'goal_tol': 0.6,
        'max_g_rad': 0.12,
        'success_threshold': 0.70,
        'description': '长距离直线倒车，距离15m',
        'difficulty': '中等',
        'key_skill': '边界约束',
    },

    # ========== Stage 4: 微小横向偏移（关键过渡）==========
    # 目的：开始学习横向调整（关键过渡）
    # 距离：20米（Y轴向下坐标系，从小Y向大Y倒车）
    # 关键技能：横向调整
    {
        'start_x': 58.25,
        'start_y': 50.56,      # Y较小（屏幕上方）
        'goal_x': 58.25,
        'goal_y': 70.56,       # Y较大（屏幕下方）
        'start_x_noise': 0.5,
        'max_steps': 500,
        'goal_tol': 0.7,
        'max_g_rad': 0.15,
        'success_threshold': 0.65,
        'description': '微小横向偏移，距离20m, ΔX=±0.5m',
        'difficulty': '较难',
        'key_skill': '横向调整',
    },

    # ========== Stage 5: 小幅横向调整 ==========
    # 目的：增强横向调整能力，学习曲线倒车
    # 距离：25米（Y轴向下坐标系，从小Y向大Y倒车）
    # 关键技能：曲线倒车
    {
        'start_x': 58.25,
        'start_y': 50.56,      # Y较小（屏幕上方）
        'goal_x': 58.25,
        'goal_y': 75.56,       # Y较大（屏幕下方）
        'start_x_noise': 1.5,
        'max_steps': 600,
        'goal_tol': 0.8,
        'max_g_rad': 0.20,
        'success_threshold': 0.60,
        'description': '小幅横向调整，距离25m, ΔX=±1.5m',
        'difficulty': '难',
        'key_skill': '曲线倒车',
    },

    # ========== Stage 6: 中等横向偏移 ==========
    # 目的：掌握显著曲线倒车
    # 距离：30米（Y轴向下坐标系，从小Y向大Y倒车）
    # 关键技能：显著曲线
    {
        'start_x': 58.25,
        'start_y': 50.56,      # Y较小（屏幕上方）
        'goal_x': 58.25,
        'goal_y': 80.56,       # Y较大（屏幕下方）
        'start_x_noise': 3.0,
        'max_steps': 800,
        'goal_tol': 1.0,
        'max_g_rad': 0.25,
        'success_threshold': 0.55,
        'description': '中等横向偏移，距离30m, ΔX=±3.0m',
        'difficulty': '很难',
        'key_skill': '显著曲线',
    },

    # ========== Stage 7: 完整倒车入库（最终目标）==========
    # 目的：完整任务，综合所有技能
    # 距离：约40米（Y轴向下坐标系，从小Y向大Y倒车）
    # 关键技能：完全倒车控制
    {
        'start_x': 58.25,
        'start_y': 50.56,      # Y较小（屏幕上方，库位）
        'goal_x': 58.25,
        'goal_y': 90.91,       # Y较大（屏幕下方，倒车终点）
        'start_x_noise': 2.0,
        'start_y_noise': 1.0,
        'max_steps': 1500,
        'goal_tol': 1.0,
        'max_g_rad': 0.30,
        'success_threshold': 0.50,
        'description': '完整倒车入库，距离40m, ΔX=±2.0m, ΔY=±1.0m',
        'difficulty': '最难',
        'key_skill': '完全倒车控制',
    },
]
