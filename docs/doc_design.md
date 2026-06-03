# 半挂车倒车入库 - 强化学习设计文档 (v3.0)

## 1. 项目概述

基于强化学习训练智能体，控制半挂车从起点以**倒车、匀速、不停车**的方式，精确停入目标库位。

### 1.1 项目文件结构

```
semi_truck_rl/
├── truck_parking_env.py    # Gymnasium 环境 (所有RL接口)
├── truckTrailerOnAxle.py   # 半挂车动力学模型
├── dual_lane_road.py       # 道路几何 + 库位绘制
├── geometry_line.py        # 几何线段基类
├── train.py                # DDPG+HER 训练脚本
├── inference.py            # 模型推理 + Pygame 可视化
├── manual.py               # 手动键盘控制
├── manual_test.py          # 原始手动测试 (参考)
├── utils.py                # 训练回调 + 奖励曲线
├── design.md               # 本文档
├── requirements_docking.md # 需求文档 (A2考试标准)
├── rl_ckpt/                # 检查点 + TensorBoard + 曲线
└── __init__.py
```

### 1.2 技术选型

| 选型 | 说明 |
|------|------|
| 环境接口 | Gymnasium Dict 空间 (HER 兼容) |
| RL 算法 | DDPG (off-policy) + HER (Hindsight Experience Replay) |
| 动作空间 | 连续 Box(-1, 1, (1,)) → 目标转向角 ±35° |
| 状态空间 | Dict{observation(9), achieved_goal(4), desired_goal(4)} |
| 动作噪声 | OrnsteinUhlenbeckActionNoise (σ=0.15, θ=0.15) |

---

## 2. 运动学与控制

### 2.1 转向速率限制

真实车辆不能瞬间打满方向盘。在 step() 中对目标转向角做一阶速率限制：

```python
max_steering_rate = 60  # °/s
steering = np.clip(
    target_steering_deg,
    prev_steering - max_steering_rate * dt,
    prev_steering + max_steering_rate * dt
)
```

### 2.2 倒车不稳定发散 (Jackknife)

半挂车倒车是非线性不稳定系统。铰接角超过 85° 直接终止回合（物理上已不可能恢复）：

```python
jackknifed = abs(g_rad) > np.deg2rad(85)
terminated = succeeded or hard_collision or jackknifed
```

观测中包含 `g_rad` 和铰接角变化率 `dg/dt`（通过相邻步差分），帮助智能体预判发散。

### 2.3 固定匀速倒车

速度硬编码为 `v_mps = -1.0`，动作只控制转向。若因物理模型漂移到 0 附近，强制重置：

```python
self.truck.v_mps = -1.0
if abs(self.truck.v_mps) < 0.4:
    self.truck.v_mps = -1.0
```

---

## 3. 状态空间 (9维观测 + 4维目标)

### 3.1 观测向量 (相对坐标，泛化能力强)

| 维度 | 含义 | 说明 |
|------|------|------|
| 0 | rel_x | truck_x - goal_x |
| 1 | rel_y | truck_y - goal_y |
| 2 | cos(rel_heading) | 相对航向余弦 |
| 3 | sin(rel_heading) | 相对航向正弦 |
| 4 | cos(g_rad) | 铰接角余弦 |
| 5 | sin(g_rad) | 铰接角正弦 |
| 6 | v_mps | 速度 |
| 7 | g_rad | 铰接角原始值 |
| 8 | steering_norm | 归一化转向角 |

### 3.2 目标向量 (4维)

| 维度 | 含义 |
|------|------|
| 0 | goal_x |
| 1 | goal_y |
| 2 | cos(goal_heading) |
| 3 | 0 (目标铰接角) |

`achieved_goal` 对应 `[x, y, cos(h), g_rad]`，`desired_goal` 对应 `[goal_x, goal_y, cos(goal_heading), 0]`。

---

## 4. 奖励函数

### 4.1 HER 稀疏奖励 (compute_reward)

```python
def compute_reward(self, achieved_goal, desired_goal, info):
    dist = np.linalg.norm(achieved_goal[:2] - desired_goal[:2])
    return -dist
```

满足 HER 目标重标定的一致性要求（仅依赖 achieved vs desired）。

### 4.2 密集引导奖励 (_compute_reward)

| 组件 | 公式 | 权重 |
|------|------|------|
| 进度奖励 | `(prev_dist - current_dist) * 10` | 核心驱动 |
| 铰接角惩罚 | `-0.1 * abs(g_rad)` | 抑制折刀 |
| 铰接角变化惩罚 | `-0.05 * abs(dg)` | 鼓励平滑 |
| 软边界惩罚 | `-2.0` | 警告但不终止 |
| 硬碰撞惩罚 | `-200.0` | 回合终止 |
| Jackknife 惩罚 | `-500.0` | 回合终止 |
| 成功奖励 | `+500.0` | 回合终止 |
| 步数惩罚 | `-0.1` | 鼓励尽快完成 |

---

## 5. 碰撞检测与终止条件

### 5.1 OBB 碰撞检测

检测车头4角 + 挂车4角 共8个角点，与道路边界做范围检测：

```python
def _get_all_corners(self):
    # 车头矩形 (truckWheelbase + 2*truckMargin) × truckWidth
    # 挂车矩形 (trailerWheelbase + trailerMarginFront + trailerMarginBack) × trailerWidth
    # 两矩形通过铰接点连接，各自独立旋转
```

### 5.2 分级碰撞

| 级别 | 条件 | 行为 |
|------|------|------|
| 软碰撞 | 角点距边界 < 0.3m | 惩罚 -2，不终止 |
| 硬碰撞 | 角点超出边界 | 终止，惩罚 -200 |
| Jackknife | \|g\| > 85° | 终止，惩罚 -500 |

### 5.3 成功判定

| 指标 | 容差 |
|------|------|
| 位置误差 | < 0.5m |
| 朝向误差 | < 10° |
| 铰接角 | < 5° |

---

## 6. 训练配置

| 参数 | 值 |
|------|-----|
| 算法 | DDPG + HER |
| HER 策略 | future |
| handle_timeout | True |
| 动作噪声 | OU, σ=0.15, θ=0.15 |
| 学习率 | 1e-3 |
| γ | 0.98 |
| τ | 0.01 |
| 网络结构 | [512, 512, 256] |
| Buffer | 2e6 |
| Batch | 512 |
| learning_starts | 2000 |
| train_freq | 每episode |
| 总步数 | 5e5 |
| 单回合最大步数 | 1500 |
| dt | 0.05s |
| 环境速度 | -1.0 m/s (固定) |

---

## 7. 初始状态随机化

```python
sx = start_x + uniform(-2.0, 2.0)
sy = start_y + uniform(-1.0, 1.0)
heading = base_heading + uniform(-0.1, 0.1)
g_init = uniform(-0.05, 0.05)
```

课程学习逐步扩大随机范围。

---

## 8. 道路参数

| 参数 | 值 |
|------|------|
| 道路中心 X | 56.0m |
| 车道宽度 | 8.0m (单车道) |
| 道路总宽 | 16.0m |
| 左边界 | 48.0m |
| 右边界 | 64.0m |
| 道路范围 Y | [0, 70]m |
| SCALE | 20 px/m |
| 起点 | (54, 9.91) + 小随机 |
| 目标 | (58.25, 50.56) |

---

## 9. 监控指标 (TensorBoard)

- `rollout/ep_rew_mean` - 平均回合奖励
- `rollout/ep_len_mean` - 平均回合长度
- Jackknife 发生率
- 碰撞位置分布
- 成功回合的步数分布
- 最终位置误差分布

---

**文档版本**: v3.0
**更新日期**: 2026-05-30
**变更**: 扁平化目录、转向速率限制、OBB碰撞、相对坐标9维观测、4维目标、OU噪声、分级碰撞、起始随机化、max_steps→1500
