# 半挂卡车模拟器 - Python版本

这是一个将MATLAB半挂卡车模拟器转换为Python的版本，
用于模拟半挂卡车-拖车系统的非线性非完整约束动力学。

## 项目概述

这个项目包含三个Python版本：

1. **完整版本** (`truck_simulator_python.py`) - 更接近MATLAB原版的实现
2. **简化版本** (`simple_truck_simulator.py`) - 更容易运行和理解的简化版本
3. **稳定版本** (`stable_truck_simulator.py`) - 数值稳定的简化版本（推荐）

## 功能特点

- 模拟半挂卡车-拖车系统的2D平面运动
- 考虑非完整约束（轮胎不侧滑）
- 转向系统动力学建模
- 空气动力学阻力
- 实时动画显示
- 多种图表输出

## 运行要求

### 系统要求
- Python 3.7+
- 必要的Python库

### 安装依赖

```bash
pip install numpy matplotlib scipy
```

或者使用requirements.txt文件：
```bash
pip install -r requirements.txt
```

## 运行方法

### 运行简化版本（推荐）
```bash
python simple_truck_simulator.py
```

### 运行完整版本
```bash
python truck_simulator_python.py
```

## 输出内容

运行仿真后会生成：

1. **实时动画**：显示卡车和拖车的运动轨迹
2. **多个图表**：
   - 卡车轨迹图
   - 速度随时间变化
   - 车轮转向角
   - 拖车相对角度
   - 位置随时间变化

## 仿真参数

### 卡车参数
- 质量：1500 kg
- 转动惯量：10000 kg·m²
- 前轴到CG距离：1 m
- 后轴到CG距离：1 m

### 拖车参数
- 质量：3000 kg
- 转动惯量：20000 kg·m²
- 铰接点到CG距离：6 m
- 铰接点到后轴距离：12 m

### 控制参数
- 推力：100 N
- 转向扭矩：1 N·m
- 仿真时间：30秒

## 项目文件说明

- `TruckSimulator_MainScript.m` - MATLAB主脚本
- `TruckDynModel.m` - MATLAB动力学模型
- `TruckKinetics.m` - MATLAB动力学计算
- `Report.pdf` - 项目技术报告
- `truck_simulator_python.py` - Python完整版本
- `simple_truck_simulator.py` - Python简化版本
- `requirements.txt` - Python依赖列表

## 技术细节

这个模拟器基于严格的数学建模，考虑了：
- 非完整约束方程
- 轮侧向力计算
- 转向系统动力学
- 空气动力学阻力
- 滚动摩擦

## 应用领域

- 车辆动力学研究
- 控制系统开发
- 路径跟踪算法测试
- 工程教育

## 注意事项

- 完整版本包含复杂的符号计算表达式，可能需要进一步调试
- 简化版本使用简化的动力学模型，更容易运行和理解
- 动画显示需要图形界面支持

## 许可证

本项目基于原始MATLAB代码转换，遵循相应的学术使用许可。
