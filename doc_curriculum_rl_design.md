# 半挂车倒车入库 - 课程学习强化学习方案设计文档

## 1. 项目背景与问题分析

### 1.1 当前状态
- **场景**：半挂车垂直倒车入库
- **目标位置**：(57.5, 53.62) 米
- **起点位置**：(57.5, 9.91) 米（与目标X坐标一致）
- **算法尝试**：
  - ADP（近似动态规划）：失败，铰接角控制困难
  - RL（强化学习）：失败，收敛困难

### 1.2 失败原因分析

#### ADP 失败原因
1. **模型依赖**：ADP 需要精确的半挂车动力学模型
2. **非线性问题**：铰接角和 heading 的非线性耦合难以用线性化方法处理
3. **边界约束**：道路边界和碰撞检测难以用 ADP 的代价函数表达

#### RL 失败原因
1. **探索困难**：动作空间连续，随机探索效率低
2. **奖励稀疏**：只有成功/失败时有明显奖励，中间过程奖励信号弱
3. **状态空间复杂**：6维状态空间（x, y, h, g, v, steering）
4. ** episode 过长**：单 episode 需要 500-1200 步，信用分配困难

### 1.3 核心难点
- **铰接角控制**：半挂车的铰接角 g 容易过大导致折叠（jackknife）
- **边界约束**：道路宽度有限，车辆容易超出边界
- **长时序依赖**：当前动作的影响可能在几十步后才显现
- **多目标优化**：需要同时优化位置、朝向、铰接角

## 2. 课程学习方案设计

### 2.1 核心思想
**从简单到复杂，逐步增加难度**

### 2.2 课程阶段设计

#### Stage 0: 直线倒车（最简单）
- **起点**：(57.5, 45.0) → 距离目标仅 8.62 米
- **特点**：
  - 起点X = 目标X，无需横向调整
  - 距离短，episode 短（约 200 步）
  - 只需保持直线倒车
- **成功标准**：
  - 到达目标位置
  - 铰接角 < 5°
  - 无碰撞

#### Stage 1: 短距离倒车
- **起点**：(57.5, 35.0) → 距离目标 18.62 米
- **特点**：
  - 距离增加，需要更长时间的稳定控制
  - 开始引入轻微的位置扰动
- **成功标准**：
  - 到达目标位置
  - 铰接角 < 8°

#### Stage 2: 中距离倒车
- **起点**：(57.5, 25.0) → 距离目标 28.62 米
- **特点**：
  - 距离进一步增加
  - 需要更好的 heading 控制
- **成功标准**：
  - 到达目标位置
  - 铰接角 < 10°

#### Stage 3: 长距离倒车
- **起点**：(57.5, 15.0) → 距离目标 38.62 米
- **特点**：
  - 接近最终难度
  - 需要稳定的长期控制策略
- **成功标准**：
  - 到达目标位置
  - 铰接角 < 12°

#### Stage 4: 完整倒车（最终目标）
- **起点**：(57.5, 9.91) → 距离目标 43.71 米
- **特点**：
  - 完整场景
  - 需要处理所有挑战
- **成功标准**：
  - 到达目标位置
  - 铰接角 < 15°

### 2.3 阶段切换策略

#### 自动切换条件
```python
# 当前阶段训练成功标准
SWITCH_CRITERIA = {
    'min_episodes': 50,           # 最少训练 episode 数
    'success_rate': 0.8,          # 成功率 > 80%
    'avg_reward': -100,           # 平均奖励 > -100
    'max_stage_steps': 50000,     # 当前阶段最大训练步数
}
```

#### 切换逻辑
1. 每 10 个 episode 评估一次
2. 如果成功率 > 80% 且平均奖励 > -100，则进入下一阶段
3. 如果超过最大步数仍未达标，也进入下一阶段（防止卡死）

## 3. 强化学习算法选择

### 3.1 推荐算法：PPO + HER

#### 为什么选择 PPO？
1. **稳定性**：PPO 的 clipped surrogate objective 保证训练稳定
2. **样本效率**：相比 DDPG，PPO 对超参数不那么敏感
3. **连续动作**：支持连续动作空间
4. **并行训练**：支持多环境并行，加速训练

#### 为什么选择 HER？
1. **稀疏奖励**：倒车任务奖励稀疏，HER 可以从失败中学习
2. **目标导向**：HER 适合目标导向的任务（到达指定位置）
3. **样本效率**：通过重新标记目标，提高样本效率

### 3.2 网络架构

#### Actor 网络
```
输入：状态 (x, y, h, g, v, steering, goal_x, goal_y)
  ↓
FC(256) + ReLU
  ↓
FC(256) + ReLU
  ↓
FC(1) + tanh  # 输出 steering ∈ [-1, 1]
```

#### Critic 网络
```
输入：状态 + 动作
  ↓
FC(256) + ReLU
  ↓
FC(256) + ReLU
  ↓
FC(1)  # 输出 Q 值
```

### 3.3 超参数设计

```python
PPO_HYPERPARAMS = {
    'learning_rate': 3e-4,
    'n_steps': 2048,
    'batch_size': 64,
    'n_epochs': 10,
    'gamma': 0.99,
    'gae_lambda': 0.95,
    'clip_range': 0.2,
    'ent_coef': 0.01,
    'vf_coef': 0.5,
    'max_grad_norm': 0.5,
}

HER_HYPERPARAMS = {
    'buffer_size': 1000000,
    'learning_starts': 1000,
    'replay_batch_size': 256,
    'replay_k': 4,  # 重放比例
}
```

## 4. 奖励函数设计

### 4.1 基础奖励
```python
def compute_reward(state, action, next_state, done, info):
    reward = 0.0
    
    # 1. 距离奖励（核心）
    dist = np.sqrt((next_state.x - GOAL_X)**2 + (next_state.y - GOAL_Y)**2)
    reward -= dist * 0.1  # 每米距离惩罚
    
    # 2. 朝向奖励
    heading_error = abs(next_state.h - (-np.pi/2))  # 目标朝向朝上
    reward -= heading_error * 0.5
    
    # 3. 铰接角惩罚
    if abs(next_state.g) > np.deg2rad(5):
        reward -= abs(next_state.g) * 2.0  # 铰接角过大惩罚
    
    # 4. 边界惩罚
    if info.get('collided', False):
        reward -= 100.0  # 碰撞大惩罚
    
    # 5. 成功奖励
    if info.get('succeeded', False):
        reward += 500.0  # 成功大奖励
    
    return reward
```

### 4.2 课程学习专用奖励
```python
# Stage 0: 简单奖励（只关注距离和碰撞）
def stage0_reward(state, action, next_state, done, info):
    reward = 0.0
    
    # 距离奖励
    dist = np.sqrt((next_state.x - GOAL_X)**2 + (next_state.y - GOAL_Y)**2)
    reward -= dist * 0.1
    
    # 碰撞惩罚
    if info.get('collided', False):
        reward -= 100.0
    
    # 成功奖励
    if info.get('succeeded', False):
        reward += 500.0
    
    return reward

# Stage 4: 完整奖励（所有因素）
def stage4_reward(state, action, next_state, done, info):
    reward = 0.0
    
    # 距离奖励
    dist = np.sqrt((next_state.x - GOAL_X)**2 + (next_state.y - GOAL_Y)**2)
    reward -= dist * 0.1
    
    # 朝向奖励
    heading_error = abs(next_state.h - (-np.pi/2))
    reward -= heading_error * 0.5
    
    # 铰接角惩罚
    if abs(next_state.g) > np.deg2rad(5):
        reward -= abs(next_state.g) * 2.0
    
    # 边界惩罚
    if info.get('collided', False):
        reward -= 100.0
    
    # 成功奖励
    if info.get('succeeded', False):
        reward += 500.0
    
    return reward
```

## 5. 状态空间设计

### 5.1 状态表示
```python
STATE_DIM = 8

class TruckState:
    def __init__(self, x, y, h, g, v, steering, goal_x, goal_y):
        self.x = x  # 车头 X 坐标
        self.y = y  # 车头 Y 坐标
        self.h = h  # 车头朝向（弧度）
        self.g = g  # 铰接角（弧度）
        self.v = v  # 速度
        self.steering = steering  # 方向盘角度
        self.goal_x = goal_x  # 目标 X
        self.goal_y = goal_y  # 目标 Y
    
    def to_array(self):
        return np.array([
            self.x, self.y,
            np.cos(self.h), np.sin(self.h),
            np.cos(self.g), np.sin(self.g),
            self.v, self.steering,
            self.goal_x, self.goal_y,
        ])
```

### 5.2 状态归一化
```python
class StateNormalizer:
    def __init__(self):
        # 根据场景范围设置
        self.x_mean = 57.5
        self.x_std = 5.0
        self.y_mean = 30.0
        self.y_std = 20.0
        self.h_mean = 0.0
        self.h_std = np.pi
        self.g_mean = 0.0
        self.g_std = np.pi / 2
        
    def normalize(self, state):
        return np.array([
            (state.x - self.x_mean) / self.x_std,
            (state.y - self.y_mean) / self.y_std,
            np.cos(state.h),  # 已经在 [-1, 1]
            np.sin(state.h),  # 已经在 [-1, 1]
            np.cos(state.g),  # 已经在 [-1, 1]
            np.sin(state.g),  # 已经在 [-1, 1]
            state.v / 2.0,  # 归一化到 [-1, 1]
            state.steering / 35.0,  # 归一化到 [-1, 1]
        ])
```

## 6. 训练流程

### 6.1 训练流程图
```
开始
  ↓
初始化 Stage 0
  ↓
训练 PPO + HER
  ↓
每 10 episode 评估
  ↓
是否达标？
  ↓ 是
进入下一阶段
  ↓
是否所有阶段完成？
  ↓ 否
返回训练
  ↓ 是
训练完成
```

### 6.2 训练脚本
```python
# train_curriculum.py
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
import gymnasium as gym

def train_with_curriculum():
    # 初始化
    current_stage = 0
    stages = CURRICULUM_STAGES
    
    for stage_idx, stage_config in enumerate(stages):
        print(f"\n{'='*50}")
        print(f"开始 Stage {stage_idx}: {stage_config}")
        print(f"{'='*50}\n")
        
        # 创建环境
        env = DummyVecEnv([lambda: gym.make('TruckParking-v0', 
                                            curriculum_stage=stage_idx)])
        
        # 初始化或加载模型
        if stage_idx == 0:
            model = PPO('MlpPolicy', env, verbose=1)
        else:
            model = PPO.load(f"stage_{stage_idx-1}_model", env=env)
        
        # 训练
        model.learn(total_timesteps=50000)
        
        # 保存模型
        model.save(f"stage_{stage_idx}_model")
        
        print(f"Stage {stage_idx} 完成！")
    
    print("\n所有阶段训练完成！")
```

## 7. 可行性分析

### 7.1 技术可行性

#### 优势
1. **课程学习**：从简单到复杂，降低学习难度
2. **PPO 稳定性**：训练过程稳定，不易发散
3. **HER 样本效率**：从失败中学习，提高样本效率
4. **并行训练**：多环境并行，加速训练

#### 风险
1. **阶段切换**：切换时机不当可能导致性能下降
2. **灾难性遗忘**：进入新阶段后忘记旧阶段知识
3. **超参数敏感**：PPO 超参数需要精心调优

### 7.2 时间可行性

#### 训练时间估算
- **Stage 0**：~2 小时（50,000 步）
- **Stage 1**：~4 小时（100,000 步）
- **Stage 2**：~6 小时（150,000 步）
- **Stage 3**：~8 小时（200,000 步）
- **Stage 4**：~10 小时（250,000 步）

**总计**：约 30 小时

#### 硬件要求
- **最低**：8 核 CPU，16GB 内存
- **推荐**：GPU（RTX 3060 以上），32GB 内存

### 7.3 成功概率评估

#### 高概率成功因素
1. **课程学习**：大幅降低学习难度
2. **PPO + HER**：成熟的算法组合
3. **状态空间适中**：8维状态，可控
4. **动作空间简单**：1维连续动作（steering）

#### 潜在问题及解决方案
| 问题 | 解决方案 |
|------|----------|
| 训练不稳定 | 降低学习率，增加 batch size |
| 收敛慢 | 增加并行环境数，使用 GPU |
| 阶段切换失败 | 放宽切换条件，增加重叠训练 |
| 灾难性遗忘 | 使用渐进式神经网络 |

## 8. 评估指标

### 8.1 训练过程指标
- **Episode 奖励**：每 episode 的平均奖励
- **成功率**：成功到达目标的 episode 比例
- **平均步数**：成功 episode 的平均步数
- **铰接角分布**：铰接角的统计分布

### 8.2 最终评估指标
- **成功率**：> 90%
- **平均步数**：< 1000 步
- **平均铰接角**：< 10°
- **碰撞率**：< 5%

## 9. 下一步计划

### 9.1 立即执行
1. [ ] 实现课程学习环境
2. [ ] 实现 PPO + HER 训练脚本
3. [ ] 训练 Stage 0（验证可行性）

### 9.2 短期目标
1. [ ] 完成所有阶段训练
2. [ ] 评估最终性能
3. [ ] 可视化训练结果

### 9.3 长期目标
1. [ ] 优化算法（SAC, TD3 等）
2. [ ] 迁移到真实场景
3. [ ] 多车协同倒车

## 10. 结论

**方案可行性：高**

通过课程学习，将复杂的倒车任务分解为简单的子任务，逐步增加难度，可以显著提高强化学习的成功率。PPO + HER 是成熟的算法组合，配合合理的状态设计和奖励函数，有望在 30 小时内训练出稳定的倒车策略。

**关键成功因素**：
1. 课程阶段设计合理
2. 奖励函数设计得当
3. 超参数调优到位
4. 充分的训练时间

**预期结果**：
- Stage 0-2：高成功率（> 80%）
- Stage 3-4：中等成功率（50-80%）
- 最终模型：能够完成大部分倒车任务
