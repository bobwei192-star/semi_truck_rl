# 全局停车约束模块

## 概述

此模块定义了半挂车倒车入库任务的全局约束条件，所有算法调试时必须服从这些约束。

## 约束条件

### 1. 车道边界约束（运动过程中）

车辆运动过程中，车身所有部分（包括8个角）不得碰撞：
- **1001线**（左车道左边界）：X = 50.0m
- **1002线**（右车道右边界/车库左边界）：X = 60.0m

### 2. 第五轮位置约束（停车后）

车辆停下后，第五轮（铰接点）需与目标位置重合：
- **GOAL_X** = 58.25m
- **GOAL_Y** = 53.56m
- **容差**：0.5m

### 3. 车库边界约束（停车后）

车辆停下后，车身所有部分需在车库边界内：
- **1002线**（车库左边界）：X = 60.0m
- **1003线**（车库右边界）：X = 56.0m
- **1014线**（车库上边界）：Y = 50.5m
- **1015线**（车库下边界）：Y = 69.0m

## 车辆8个角编号

```
牵引车头（编号1-4）:
1 - 车头右前角
2 - 车头右后角
3 - 车头左后角
4 - 车头左前角

挂车（编号5-8）:
5 - 挂车右前角（靠近车头）
6 - 挂车右后角
7 - 挂车左后角
8 - 挂车左前角（靠近车头）
```

## 使用方法

### 基本用法

```python
from constraints.parking_constraints import verify_parking_success

# 在仿真结束时检查约束
success = verify_parking_success(truck)
# 返回 True/False，并打印详细报告
```

### 分步检查

```python
from constraints.parking_constraints import constraints

# 检查车道边界（运动过程中）
violations = constraints.check_lane_boundary_violation(truck, step=current_step)

# 检查第五轮位置
result = constraints.check_goal_position(truck)
# result['within_tolerance'] 为 True/False

# 检查车库边界
violations = constraints.check_garage_boundary(truck)

# 运行所有检查
results = constraints.run_all_checks(truck, step=current_step)
```

### 获取8个角位置

```python
from constraints.parking_constraints import get_corner_positions

corners = get_corner_positions(truck)
# corners[0] = (x1, y1)  # 角1位置
# corners[7] = (x8, y8)  # 角8位置
```

### 获取第五轮位置

```python
from constraints.parking_constraints import get_fifth_wheel_position

x, y = get_fifth_wheel_position(truck)
```

## 文件结构

```
constraints/
├── __init__.py
└── parking_constraints.py    # 全局约束模块
```

## 依赖

- `config.scene_config`：场景配置参数
- `numpy`：数值计算

## 注意事项

1. 所有算法在调试时必须在仿真结束时调用 `verify_parking_success()` 进行约束验证
2. 约束检查会输出详细的违规信息，包括具体哪个角越界、越界距离等
3. 轨迹违规会自动记录，可通过 `constraints.get_trajectory_summary()` 获取
4. 每次仿真前调用 `constraints.reset_trajectory_tracking()` 重置轨迹记录
