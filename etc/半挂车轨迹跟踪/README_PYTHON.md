# 半挂车轨迹跟踪系统 - Python版本

## 项目概述

本项目将原始的MATLAB半挂车轨迹跟踪系统转换为Python版本。系统使用LQR控制器和扩展卡尔曼滤波器来实现半挂车的轨迹跟踪控制。

## 转换完成情况

### ✅ 已成功转换的模块

1. **主程序** (`main.py`)
   - 系统主入口
   - 参数设置、轨迹创建、模拟运行、结果展示

2. **参数设置** (`set_parameters.py`)
   - 系统参数配置
   - 轨迹参数、模拟器参数、估计器参数、控制器参数、动力学参数

3. **轨迹示例** (`trajectory_examples.py`)
   - 8个不同的轨迹示例
   - 包含多项式拟合和函数定义两种轨迹生成方式

4. **动力学模型** (`dynamics.py`)
   - 半挂车系统动力学方程
   - 状态导数计算

5. **控制器** (`controller.py`)
   - LQR控制器实现
   - 增益平滑和转向平滑

6. **模拟器** (`simulator.py`)
   - 系统模拟主循环
   - 欧拉积分、状态估计、控制命令生成

7. **轨迹创建** (`create_trajectory.py`)
   - 轨迹生成和增益矩阵计算
   - 正向和反向行驶支持

8. **传感器** (`sensors.py`)
   - 状态测量和噪声添加

9. **估计器** (`estimator.py`)
   - 扩展卡尔曼滤波器
   - 状态估计和协方差更新

10. **目标点选择** (`select_goal_point.py`)
    - 在轨迹上选择目标点
    - 向前看距离控制

11. **执行器模型** (`actuator_model.py`)
    - 执行器限制应用
    - 速度和转向角度限制

12. **增益矩阵设置** (`set_gain_matrix.py`)
    - LQR增益计算
    - 连续时间代数Riccati方程求解

13. **绘图功能** (`plot_functions.py`)
    - 输入信号绘制
    - 估计器性能分析
    - 轨迹跟踪可视化

### 🔧 技术实现细节

- **数据结构**: 使用Python字典替代MATLAB结构体
- **矩阵运算**: 使用NumPy进行矩阵运算
- **数值计算**: 使用SciPy求解代数Riccati方程
- **可视化**: 使用Matplotlib进行数据可视化
- **索引处理**: 注意Python索引从0开始，MATLAB从1开始

### 📊 系统特性

- **状态变量**: [x, y, θ, φ] - 位置、挂车角度、驾驶室角度
- **控制输入**: [速度, 转向角度]
- **控制器**: 线性二次调节器(LQR)
- **估计器**: 扩展卡尔曼滤波器(EKF)
- **轨迹跟踪**: 基于预定义轨迹的跟踪控制

## 运行说明

### 环境要求

```bash
pip install numpy scipy matplotlib
```

### 运行完整系统

```bash
python main.py
```

### 运行简化测试

```bash
python test_simple.py
```

## 测试结果

✅ **所有基本功能测试通过**:
- 参数设置成功
- 轨迹创建成功  
- 动力学模型正常
- 控制器工作正常
- 绘图功能可用

## 文件结构

```
半挂车轨迹跟踪/
├── main.py                    # 主程序
├── set_parameters.py          # 参数设置
├── trajectory_examples.py     # 轨迹示例
├── dynamics.py               # 动力学模型
├── controller.py             # 控制器
├── simulator.py              # 模拟器
├── create_trajectory.py      # 轨迹创建
├── sensors.py                # 传感器
├── estimator.py              # 估计器
├── select_goal_point.py      # 目标点选择
├── actuator_model.py         # 执行器模型
├── set_gain_matrix.py        # 增益矩阵设置
├── plot_functions.py         # 绘图功能
├── test_simple.py            # 简化测试
├── requirements.txt          # 依赖包
└── README_PYTHON.md          # 本文档
```

## 注意事项

1. **简化实现**: 某些复杂功能（如精确的雅可比矩阵计算）使用了简化版本
2. **性能优化**: 对于大规模模拟，可能需要进一步优化性能
3. **动画功能**: 完整的动画显示功能需要进一步开发

## 原始MATLAB文件

原始的MATLAB文件仍然保留在目录中，可供参考和对比。

---

**转换完成时间**: 2025年11月30日  
**Python版本**: 3.11.5  
**主要依赖**: NumPy, SciPy, Matplotlib
