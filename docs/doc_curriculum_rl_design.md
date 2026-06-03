# 半挂车倒车入库 - 课程学习强化学习方案设计文档 (v2.0)

## 1. 项目背景与问题分析

### 1.1 任务目标
控制 A2 半挂车从起点以**倒车、匀速、不停车**方式精确定位入目标库位。

| 约束 | 说明 |
|------|------|
| 行驶方向 | 仅倒车 (v < 0) |
| 速度 | 匀速倒车，-1.0 m/s 固定 |
| 停车限制 | 不允许中途停车 |
| 碰线限制 | 车辆任何部位不得触碰实线边界 |

### 1.2 核心难点

#### 倒车运动学特性（与正向出库本质不同）
半挂车倒车是非线性**不稳定系统**，铰接角有自发增大的趋势：

| 铰接角范围 | 系统行为 | 控制难度 |
|-----------|---------|---------|
| \|g\| < 15° | 稳定可控 | 容易 |
| 15° < \|g\| < 30° | 轻微发散 | 需主动修正 |
| 30° < \|g\| < 85° | 快速发散 | 高难度 |
| \|g\| > 85° | Jackknife | 无法恢复 |

#### 多目标耦合
需要同时控制：位置(x,y)、航向角(h)、铰接角(g)

### 1.3 现有方案问题分析

#### 问题1: 场景配置错误
- 需求文档：倒车入库
- 实际代码：正向出库
- 影响：训练的模型无法用于倒车任务

#### 问题2: 课程阶段跨度过大
现有5个阶段：
- Stage 0→1: 距离增加10m（相对合理）
- Stage 1→2: 距离再增加10m（相对合理）
- Stage 2→3: 引入横向偏移（难度跃升）
- Stage 3→4: 完整任务（步数增加33%）

问题：从直线到曲线的跨越对智能体来说难度激增

#### 问题3: 缺乏明确验收标准
- 无量化成功率阈值
- 无阶段切换条件
- 无性能评估指标

#### 问题4: 奖励信号稀疏
- 主要依赖距离奖励
- 缺少中间状态的引导
- 铰接角控制奖励不足

## 2. 重新设计的课程学习方案

### 2.1 设计原则

1. **循序渐进**：每阶段只增加一个难度维度
2. **可验证性**：每阶段有明确、可量化的通过标准
3. **稳定性优先**：先保证铰接角稳定，再追求精确到位
4. **样本效率**：早期阶段快速迭代，后期阶段精细调优

### 2.2 课程阶段设计（8阶段，更细粒度）

#### Stage 0: 静态对齐（铰接角归零）
**目的**：训练智能体学会将铰接角保持在0附近

| 参数 | 值 |
|------|-----|
| 起点 | (58.25, 49.56) |
| 目标 | (58.25, 50.56) |
| 距离 | 1.0 米 |
| 初始g | 随机 ±0.1 rad |
| 最大步数 | 50 |

**验收标准**：
- ✅ 成功率 ≥ 85%（连续3次评估，每次100 episodes）
- ✅ 平均铰接角 < 0.05 rad
- ✅ 平均位置误差 < 0.5 m
- ✅ 无碰撞、无Jackknife

**奖励函数**（密集引导）：
```python
reward = 0
# 1. 距离奖励（核心）
dist_delta = prev_dist - current_dist
reward += dist_delta * 20.0

# 2. 铰接角稳定奖励（关键）
reward -= abs(g) * 5.0           # 惩罚铰接角
reward -= abs(dg_dt) * 3.0       # 惩罚铰接角变化率

# 3. 航向奖励
heading_error = abs(h - goal_heading)
reward -= heading_error * 2.0

# 4. 成功奖励
if succeeded:
    reward += 100.0
```

**训练步数**：50,000 steps

---

#### Stage 1: 短距离直线倒车（3米）
**目的**：在保持铰接角稳定的前提下，学习距离控制

| 参数 | 值 |
|------|-----|
| 起点 | (58.25, 47.56) |
| 目标 | (58.25, 50.56) |
| 距离 | 3.0 米 |
| 初始g | 0 ± 0.03 rad |
| 最大步数 | 100 |

**验收标准**：
- ✅ 成功率 ≥ 80%
- ✅ 平均铰接角 < 0.08 rad
- ✅ 平均位置误差 < 0.5 m

**奖励函数**：同Stage 0，距离系数调整为15.0

**训练步数**：80,000 steps

---

#### Stage 2: 中距离直线倒车（8米）
**目的**：延长控制时间，训练稳定性

| 参数 | 值 |
|------|-----|
| 起点 | (58.25, 42.56) |
| 目标 | (58.25, 50.56) |
| 距离 | 8.0 米 |
| 初始g | 0 ± 0.05 rad |
| 最大步数 | 200 |

**验收标准**：
- ✅ 成功率 ≥ 75%
- ✅ 平均铰接角 < 0.10 rad
- ✅ 平均位置误差 < 0.5 m

**训练步数**：100,000 steps

---

#### Stage 3: 长距离直线倒车（15米）
**目的**：训练长期稳定控制，开始引入边界约束

| 参数 | 值 |
|------|-----|
| 起点 | (58.25, 35.56) |
| 目标 | (58.25, 50.56) |
| 距离 | 15.0 米 |
| 初始g | 0 ± 0.08 rad |
| 最大步数 | 400 |

**验收标准**：
- ✅ 成功率 ≥ 70%
- ✅ 平均铰接角 < 0.12 rad
- ✅ 平均位置误差 < 0.6 m
- ✅ 边界碰撞率 < 10%

**奖励函数调整**：
```python
# 增加边界惩罚
if soft_collision:
    reward -= 10.0
if hard_collision:
    reward -= 200.0
```

**训练步数**：120,000 steps

---

#### Stage 4: 微小横向偏移（±0.5米）
**目的**：开始学习横向调整（关键过渡）

| 参数 | 值 |
|------|-----|
| 起点 | (58.25±0.5, 30.56) |
| 目标 | (58.25, 50.56) |
| Y距离 | 20.0 米 |
| X偏移 | ±0.5 米（随机） |
| 最大步数 | 500 |

**验收标准**：
- ✅ 成功率 ≥ 65%
- ✅ 平均铰接角 < 0.15 rad
- ✅ 平均位置误差 < 0.7 m

**关键技能**：
学会用小幅转向产生横向偏移，同时保持铰接角稳定

**训练步数**：150,000 steps

---

#### Stage 5: 小幅横向调整（±1.5米）
**目的**：增强横向调整能力，学习曲线倒车

| 参数 | 值 |
|------|-----|
| 起点 | (58.25±1.5, 25.56) |
| 目标 | (58.25, 50.56) |
| Y距离 | 25.0 米 |
| X偏移 | ±1.5 米（随机） |
| 最大步数 | 600 |

**验收标准**：
- ✅ 成功率 ≥ 60%
- ✅ 平均铰接角 < 0.20 rad
- ✅ 平均位置误差 < 0.8 m

**奖励函数调整**：
```python
# 增加铰接角控制奖励
if abs(g) < 0.15:
    reward += 1.0  # 小铰接角奖励
    
# 转向一致性奖励（人类经验）
if (g > 0 and steering > 0) or (g < 0 and steering < 0):
    reward += 0.5  # 方向一致
```

**训练步数**：200,000 steps

---

#### Stage 6: 中等横向偏移（±3米）
**目的**：掌握显著曲线倒车

| 参数 | 值 |
|------|-----|
| 起点 | (58.25±3.0, 20.56) |
| 目标 | (58.25, 50.56) |
| Y距离 | 30.0 米 |
| X偏移 | ±3.0 米（随机） |
| 最大步数 | 800 |

**验收标准**：
- ✅ 成功率 ≥ 55%
- ✅ 平均铰接角 < 0.25 rad（最大可到0.3）
- ✅ 平均位置误差 < 1.0 m

**训练步数**：250,000 steps

---

#### Stage 7: 完整倒车入库（最终目标）
**目的**：完整任务，综合所有技能

| 参数 | 值 |
|------|-----|
| 起点 | (58.25±2.0, 9.91±1.0) |
| 目标 | (58.25, 50.56) |
| 距离 | 约40米 |
| X随机 | ±2.0 米 |
| Y随机 | ±1.0 米 |
| 最大步数 | 1500 |

**验收标准**：
- ✅ 成功率 ≥ 50%
- ✅ 平均铰接角 < 0.30 rad
- ✅ 平均位置误差 < 1.0 m
- ✅ 朝向误差 < 10°
- ✅ 铰接角误差 < 5°

**训练步数**：300,000 steps

---

### 2.3 阶段切换策略

#### 自动切换条件
```python
SWITCH_CRITERIA = {
    'min_episodes': 100,          # 最少评估episode数
    'success_rate_threshold': stage_threshold,  # 阶段相关阈值
    'eval_frequency': 20,         # 每20个episode评估一次
    'eval_windows': 3,            # 连续3次评估都达标
    'max_stage_steps': stage_max_steps,  # 阶段最大步数
    'early_stop_patience': 5,     # 连续5次不达标回退
}
```

#### 切换逻辑
```python
def should_switch_stage(stage_idx, eval_history):
    stage_config = CURRICULUM_STAGES[stage_idx]
    threshold = stage_config['success_threshold']
    
    # 条件1: 连续N次评估达标
    if len(eval_history) >= SWITCH_CRITERIA['eval_windows']:
        recent_rates = [e['success_rate'] for e in eval_history[-SWITCH_CRITERIA['eval_windows']:]]
        if all(rate >= threshold for rate in recent_rates):
            return True, "达标切换"
    
    # 条件2: 达到最大步数，且最近表现尚可
    if current_steps >= stage_config['max_steps']:
        recent_rate = np.mean([e['success_rate'] for e in eval_history[-5:]])
        if recent_rate >= threshold * 0.7:  # 宽松条件
            return True, "最大步数切换"
        else:
            return False, "需要更多训练"
    
    return False, "继续训练"
```

#### 回退机制（防止灾难性遗忘）
```python
def should_rollback(stage_idx, eval_history):
    # 如果新阶段表现急剧下降，回退到上一阶段
    if len(eval_history) >= 10:
        recent_rate = np.mean([e['success_rate'] for e in eval_history[-5:]])
        prev_stage_threshold = CURRICULUM_STAGES[stage_idx-1]['success_threshold']
        
        if recent_rate < prev_stage_threshold * 0.5:
            return True, "性能骤降，回退"
    
    return False, None
```

## 3. 算法与网络架构

### 3.1 推荐算法：PPO (Proximal Policy Optimization)

**选择理由**：
1. 训练稳定，不易发散（对倒车不稳定系统至关重要）
2. 支持连续动作空间
3. 支持多环境并行训练
4. 超参数相对鲁棒

### 3.2 网络架构

```python
PPO_CONFIG = {
    'policy': 'MultiInputPolicy',
    'learning_rate': 3e-4,
    'n_steps': 2048,
    'batch_size': 64,
    'n_epochs': 10,
    'gamma': 0.99,
    'gae_lambda': 0.95,
    'clip_range': 0.2,
    'ent_coef': 0.01,      # 适度探索
    'vf_coef': 0.5,
    'max_grad_norm': 0.5,
    'network_arch': {
        'pi': [256, 256, 128],   # Actor网络
        'vf': [256, 256, 128],   # Critic网络
    }
}
```

### 3.3 并行训练配置

```python
N_ENVS = 4  # 并行环境数（根据CPU核心数调整）

# 学习率调度（后期衰减）
def lr_schedule(progress):
    return 3e-4 * (1 - 0.5 * progress)
```

## 4. 奖励函数设计（分阶段演进）

### 4.1 奖励函数演进策略

| 阶段 | 奖励重点 | 特殊处理 |
|------|---------|---------|
| 0-2 | 铰接角稳定 > 距离 | 高铰接角惩罚 |
| 3-4 | 距离 + 边界 | 增加边界约束 |
| 5-7 | 全面 + 铰接角控制 | 转向一致性奖励 |

### 4.2 通用奖励函数框架

```python
def compute_reward(state, action, next_state, done, info, stage_idx):
    reward = 0.0
    
    # === 1. 距离奖励（核心驱动）===
    dist = get_distance(next_state, goal)
    prev_dist = get_distance(state, goal)
    dist_delta = prev_dist - dist
    reward += dist_delta * REWARD_CONFIG[stage_idx]['dist_coef']
    
    # === 2. 铰接角奖励（关键）===
    g = next_state.g_rad
    dg = (next_state.g_rad - state.g_rad) / DT
    
    # 铰接角大小惩罚
    reward -= abs(g) * REWARD_CONFIG[stage_idx]['g_coef']
    
    # 铰接角变化率惩罚（防止快速发散）
    reward -= abs(dg) * REWARD_CONFIG[stage_idx]['dg_coef']
    
    # 小铰接角奖励（Stage 5+）
    if stage_idx >= 5 and abs(g) < 0.15:
        reward += 1.0
    
    # === 3. 航向奖励 ===
    heading_error = abs(normalize_angle(next_state.h - goal_heading))
    reward -= heading_error * REWARD_CONFIG[stage_idx]['heading_coef']
    
    # === 4. 边界约束（Stage 3+）===
    if stage_idx >= 3:
        if info.get('soft_collision', False):
            reward -= 10.0
        if info.get('hard_collision', False):
            reward -= 200.0
    
    # === 5. Jackknife 惩罚 ===
    if info.get('jackknifed', False):
        reward -= 500.0
    
    # === 6. 成功奖励 ===
    if info.get('succeeded', False):
        reward += REWARD_CONFIG[stage_idx]['success_bonus']
    
    # === 7. 步数惩罚（鼓励效率）===
    reward -= 0.05
    
    return reward
```

### 4.3 阶段奖励配置

```python
REWARD_CONFIG = {
    0: {'dist_coef': 20.0, 'g_coef': 5.0, 'dg_coef': 3.0, 'heading_coef': 2.0, 'success_bonus': 100.0},
    1: {'dist_coef': 18.0, 'g_coef': 4.5, 'dg_coef': 2.8, 'heading_coef': 2.0, 'success_bonus': 120.0},
    2: {'dist_coef': 15.0, 'g_coef': 4.0, 'dg_coef': 2.5, 'heading_coef': 2.0, 'success_bonus': 150.0},
    3: {'dist_coef': 12.0, 'g_coef': 3.5, 'dg_coef': 2.0, 'heading_coef': 2.5, 'success_bonus': 200.0},
    4: {'dist_coef': 10.0, 'g_coef': 3.0, 'dg_coef': 2.0, 'heading_coef': 3.0, 'success_bonus': 250.0},
    5: {'dist_coef': 10.0, 'g_coef': 3.0, 'dg_coef': 2.0, 'heading_coef': 3.0, 'success_bonus': 300.0},
    6: {'dist_coef': 10.0, 'g_coef': 2.5, 'dg_coef': 1.5, 'heading_coef': 3.5, 'success_bonus': 400.0},
    7: {'dist_coef': 10.0, 'g_coef': 2.0, 'dg_coef': 1.5, 'heading_coef': 4.0, 'success_bonus': 500.0},
}
```

## 5. 状态空间设计

### 5.1 观测向量（9维）

| 维度 | 含义 | 归一化范围 |
|------|------|-----------|
| 0 | rel_x = truck_x - goal_x | [-10, 10] |
| 1 | rel_y = truck_y - goal_y | [-50, 50] |
| 2 | cos(rel_heading) | [-1, 1] |
| 3 | sin(rel_heading) | [-1, 1] |
| 4 | cos(g_rad) | [-1, 1] |
| 5 | sin(g_rad) | [-1, 1] |
| 6 | v_mps / 2.0 | [-1, 1] |
| 7 | g_rad / (π/2) | [-1, 1] |
| 8 | steering_norm | [-1, 1] |

### 5.2 目标向量（4维，HER兼容）

| 维度 | 含义 |
|------|------|
| 0 | goal_x |
| 1 | goal_y |
| 2 | cos(goal_heading) |
| 3 | 0 (目标铰接角) |

## 6. 训练监控与评估

### 6.1 TensorBoard指标

```python
LOG_METRICS = [
    'rollout/ep_rew_mean',      # 平均回合奖励
    'rollout/ep_len_mean',      # 平均回合长度
    'train/entropy_loss',       # 熵损失
    'train/policy_gradient_loss',# 策略梯度损失
    'train/value_loss',         # 价值函数损失
    'custom/success_rate',      # 成功率（自定义）
    'custom/avg_g_rad',         # 平均铰接角（自定义）
    'custom/avg_dist_error',    # 平均距离误差（自定义）
    'custom/collision_rate',    # 碰撞率（自定义）
]
```

### 6.2 阶段评估脚本

```python
def evaluate_stage(model, env, num_episodes=100):
    results = {
        'success_count': 0,
        'collision_count': 0,
        'jackknife_count': 0,
        'total_reward': 0,
        'total_steps': 0,
        'final_distances': [],
        'final_g_rads': [],
        'final_heading_errors': [],
    }
    
    for ep in range(num_episodes):
        obs, _ = env.reset()
        done = False
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            
            results['total_reward'] += reward
            results['total_steps'] += 1
        
        results['success_count'] += int(info.get('succeeded', False))
        results['collision_count'] += int(info.get('collided', False))
        results['jackknife_count'] += int(info.get('jackknifed', False))
        results['final_distances'].append(info.get('final_dist', 999))
        results['final_g_rads'].append(info.get('final_g', 0))
    
    results['success_rate'] = results['success_count'] / num_episodes
    results['collision_rate'] = results['collision_count'] / num_episodes
    results['avg_final_dist'] = np.mean(results['final_distances'])
    results['avg_final_g'] = np.mean([abs(g) for g in results['final_g_rads']])
    
    return results
```

## 7. 实施计划

### 7.1 训练时间估算

| 阶段 | 训练步数 | 预估时间（GPU） | 预估时间（CPU） |
|------|---------|----------------|----------------|
| 0 | 50k | 30分钟 | 2小时 |
| 1 | 80k | 45分钟 | 3小时 |
| 2 | 100k | 1小时 | 4小时 |
| 3 | 120k | 1.5小时 | 5小时 |
| 4 | 150k | 2小时 | 6小时 |
| 5 | 200k | 2.5小时 | 8小时 |
| 6 | 250k | 3小时 | 10小时 |
| 7 | 300k | 4小时 | 12小时 |
| **总计** | **1.25M** | **15小时** | **50小时** |

### 7.2 硬件建议

| 配置 | 最低要求 | 推荐配置 |
|------|---------|---------|
| CPU | 4核 | 8核以上 |
| 内存 | 8GB | 16GB以上 |
| GPU | 无（可CPU训练） | RTX 3060以上 |
| 存储 | 10GB | 20GB（保存检查点） |

## 8. 预期结果与成功标准

### 8.1 各阶段预期成功率

| 阶段 | 目标成功率 | 关键指标 |
|------|-----------|---------|
| 0 | ≥ 85% | 铰接角 < 0.05 rad |
| 1 | ≥ 80% | 铰接角 < 0.08 rad |
| 2 | ≥ 75% | 铰接角 < 0.10 rad |
| 3 | ≥ 70% | 边界碰撞 < 10% |
| 4 | ≥ 65% | 位置误差 < 0.7m |
| 5 | ≥ 60% | 横向调整能力 |
| 6 | ≥ 55% | 曲线倒车 |
| 7 | ≥ 50% | 完整任务 |

### 8.2 最终验收标准（Stage 7）

| 指标 | 阈值 | 说明 |
|------|------|------|
| 成功率 | ≥ 50% | 连续3次评估，每次100 episodes |
| 位置误差 | < 1.0m | Euclidean距离 |
| 朝向误差 | < 10° | 相对目标朝向 |
| 铰接角误差 | < 5° | 挂车对齐 |
| 碰撞率 | < 20% | 包括软硬碰撞 |
| 平均步数 | < 1200步 | 效率指标 |

## 9. 风险与应对

### 9.1 潜在风险

| 风险 | 概率 | 影响 | 应对措施 |
|------|------|------|---------|
| 训练不收敛 | 中 | 高 | 调整学习率、增加奖励引导 |
| 灾难性遗忘 | 中 | 中 | 使用回退机制、增加重叠训练 |
| 铰接角失控 | 高 | 高 | 增加Stage 0-2的训练时间 |
| 边界碰撞 | 中 | 中 | 增加边界奖励权重 |
| 超时（步骤耗尽） | 低 | 中 | 增加max_steps或调整奖励 |

### 9.2 调试建议

1. **Stage 0-2失败**：铰接角惩罚不足，增加`g_coef`
2. **Stage 3+边界碰撞多**：增加边界检测灵敏度
3. **成功率无法提升**：检查奖励函数稀疏性，增加引导奖励
4. **训练不稳定**：降低学习率至1e-4，增加batch_size

## 10. 后续优化方向

### 10.1 算法改进
- 尝试SAC（Soft Actor-Critic）获得更好的样本效率
- 尝试TD3（Twin Delayed DDPG）减少过估计
- 结合HER（Hindsight Experience Replay）处理稀疏奖励

### 10.2 课程优化
- 动态调整阶段切换条件（基于学习曲线）
- 引入自适应难度调整
- 多任务学习（同时训练多个起点）

### 10.3 实车迁移
- 域随机化（增加传感器噪声、模型不确定性）
- Sim2Real迁移学习
- 安全约束增强（实车碰撞代价更高）

---

**文档版本**: v2.0  
**更新日期**: 2026-06-02  
**变更说明**: 重新设计8阶段课程学习方案，增加明确验收标准，优化奖励函数，细化实施计划
