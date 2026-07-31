

![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![GTSAM](https://img.shields.io/badge/Library-GTSAM-orange.svg)
![Eigen3](https://img.shields.io/badge/Library-Eigen3-green.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)
![Build](https://img.shields.io/badge/Build-CMake-red.svg)
# Factor-Graph Visual-Inertial-Wheel Odometry (VI-WO) Fusion

**VIWO (Visual-Inertial-Wheel Odometry) Factor Graph** is a C++17 state estimation engine designed for mobile robotics. It addresses the well-known metric scale drift in monocular visual-inertial navigation by incorporating wheel encoder measurements into an incremental factor graph optimization framework.

## About The Project

Standard monocular Visual-Inertial Odometry (VIO) relies on camera frames and IMU measurements to estimate 3D motion. However, camera-based setups naturally suffer from scale drift over time—especially during constant-velocity motion or when traversing feature-sparse corridors.

This repository introduces a custom **Wheel Odometry Factor** built for the GTSAM framework. By tightly coupling relative wheel encoder measurements with visual motion and inertial priors inside a non-linear factor graph, the pipeline uses ISAM2 incremental smoothing to lock down real-world metric scale, delivering drift-resistant 3D positioning for mobile platforms.
## Benchmark Results

![Trajectory and ATE Comparison](docs/ate_trajectory_plot.png)

| Method | Absolute Trajectory Error (ATE RMSE) | Drift Reduction |
| :--- | :--- | :--- |
| **VI Baseline** | 0.246 m | Baseline |
| **VIWO Proposed** | **0.029 m** | **~88.2%** |

## Overview
Monocular Visual-Inertial Odometry (VIO) inherently suffers from metric scale drift during constant-velocity motion or visually featureless environments. This project introduces a custom **Wheel Odometry Factor** integrated into a GTSAM non-linear factor graph using ISAM2 incremental smoothing.

## Mathematical Formulation

### Custom Wheel Factor Residual
Given two consecutive $SE(3)$ keyframe poses $T_i = (R_i, \mathbf{p}_i)$ and $T_j = (R_j, \mathbf{p}_j)$, the relative body-frame displacement residual $\mathbf{r}(T_i, T_j) \in \mathbb{R}^3$ is defined as:

$$
\mathbf{r}(T_i, T_j) = R_i^T (\mathbf{p}_j - \mathbf{p}_i) - \Delta \mathbf{p}_{\text{wheel}}
$$

### Analytical Jacobians (GTSAM Local Tangent Space)

Under GTSAM local body manifold parameterisation with tangent space perturbation vector $\boldsymbol{\xi} = [\boldsymbol{\omega}^T, \mathbf{v}^T]^T \in \mathfrak{se}(3)$:

**With respect to Pose $i$** ($H_1 \in \mathbb{R}^{3 \times 6}$):

$$
H_1 = \frac{\partial \mathbf{r}}{\partial T_i} = \begin{bmatrix} (R_i^T (\mathbf{p}_j - \mathbf{p}_i))^\times & -I_{3 \times 3} \end{bmatrix}
$$

**With respect to Pose $j$** ($H_2 \in \mathbb{R}^{3 \times 6}$):

$$
H_2 = \frac{\partial \mathbf{r}}{\partial T_j} = \begin{bmatrix} \mathbf{0}_{3 \times 3} & R_i^T R_j \end{bmatrix}
$$
  $$

## Repository Structure
```text
viwo-factor-graph/
├── docs/
│   └── ate_trajectory_plot.png
├── include/
│   └── viwo/
│       ├── factor_graph_engine.hpp
│       └── wheel_odometry_factor.hpp
├── scripts/
│   └── plot_ate_trajectory.py
├── src/
│   └── main_benchmark.cpp
├── CMakeLists.txt
├── LICENSE
└── README.md
```

## Quick Start & Build

### Requirements
- C++17 Compiler
- CMake >= 3.16
- Eigen3
- GTSAM >= 4.0

```bash
mkdir build && cd build
cmake ..
make
./viwo_benchmark
python3 ../scripts/plot_ate_trajectory.py
```
