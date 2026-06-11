# Another Solve - MPC Parking Controller

Based on paper: j.cnki.xk.2023.2088

## Overview

This module implements a Model Predictive Control (MPC) based parking system for tractor-trailer vehicles, following the methodology described in the paper.

## Structure

```
another_solve/
├── __init__.py          # Module initialization
├── config.py            # Vehicle and parking space parameters
├── path_planner.py      # B-spline path planning with collision analysis
├── kinematics.py        # Vehicle kinematics model
├── mpc_controller.py    # MPC controller implementation
├── main.py              # Main simulation script
├── simulate.py          # Integration with project vehicle model
└── README.md            # This file
```

## Key Components

### 1. Vehicle Parameters (config.py)
- Based on TruckSim Cab-Over + European Trailer
- Key parameters: wheelbase, hitch point location, trailer dimensions
- Parking space dimensions following JGJ100-98 standard

### 2. Path Planning (path_planner.py)
- **Collision Analysis**: Determines feasible region for arc path starting point
  - Left collision check (Eq. 5)
  - Right collision check (Eq. 10)
  - Traffic lane collision check (Eq. 14)
- **B-spline Path Generation**: Smooth parking path using control points

### 3. Kinematics Model (kinematics.py)
- State vector: [x_C, y_C, phi_0, phi_1]
  - x_C, y_C: Trailer rear axle center position
  - phi_0: Tractor heading angle
  - phi_1: Trailer heading angle
- Control input: delta (front wheel steering angle)
- Linearization for MPC (Eq. 34, 35)

### 4. MPC Controller (mpc_controller.py)
- Prediction horizon: Np
- Control horizon: Nc
- QP-based optimization
- Constraints on steering angle and rate

## Usage

### Basic Test

```python
from another_solve.main import ParkingMPC

# Create controller
controller = ParkingMPC(
    start_point=(8.15, 7.2),
    arc_start=(6.15, 7.2),
    use_simple_mpc=True
)

# Run simulation
controller.plan_path()
controller.generate_reference()
result = controller.run_simulation(max_steps=2000, dt=0.1)

# Visualize
controller.visualize()
```

### Integration with Project Vehicle Model

```python
from another_solve.simulate import IntegratedParkingSimulator

simulator = IntegratedParkingSimulator()
result = simulator.run(start_point=(8.15, 7.2), arc_start=(6.15, 7.2))
simulator.visualize(result)
```

## Key Equations from Paper

### Minimum Turning Radius
```
R_min = L1 / tan(alpha)  (Eq. 1)
```

### Kinematics Model
```
dx_C = v * cos(phi_1)
dy_C = v * sin(phi_1)
dphi_1 = v/L1 * (Lb*tan(delta)/(L0*cos(phi_0-phi_1)) + tan(phi_0-phi_1))  (Eq. 31)
dphi_0 = (dphi_1 * L1 * cos(phi_0-phi_1) - v * sin(phi_0-phi_1)) / Lb  (Eq. 25)
```

### Linearized Discrete Model
```
A(k|t) = [1, 0, 0, -v*sin(phi_1_ref)*T]
         [0, 1, 0,  v*cos(phi_1_ref)*T]
         [0, 0, 1,  0                  ]
         [0, 0, P*Q*T, 1-P*Q*T        ]  (Eq. 34)
```

## Notes

1. The simplified MPC controller uses a proportional feedback law. For better performance, tune the gains in `SimpleMPCController`.

2. The full MPC controller (`MPCController`) requires solving a QP problem at each step, which is computationally more intensive.

3. The reference steering angles in the current implementation are close to zero because the path is primarily composed of straight segments. Consider adjusting the path planning parameters for curved paths.

## Dependencies

- numpy
- scipy (for B-spline interpolation and QP solving)
- matplotlib (for visualization)

## References

Paper: j.cnki.xk.2023.2088 - "牵引挂车垂直泊车路径规划与MPC跟踪控制"
