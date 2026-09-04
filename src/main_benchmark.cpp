/**
 * @file main_benchmark.cpp
 * @brief Benchmark Harness evaluating Visual-Inertial vs Visual-Inertial-Wheel Fusion
 * @author Andrea Loroni
 * @license MIT
 */

#include "viwo/factor_graph_engine.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>

int main() {
    std::cout << "=== Running VIWO Factor-Graph State Estimator Benchmark ===" << std::endl;

    viwo::FactorGraphEngine estimator_viwo;  // Visual-Inertial-Wheel
    viwo::FactorGraphEngine estimator_vi;    // Visual-Inertial only (Baseline)

    // Initial State Setup
    gtsam::Pose3 prior_pose(gtsam::Rot3::Identity(), gtsam::Point3(0.0, 0.0, 0.0));
    gtsam::Vector3 prior_vel(1.0, 0.0, 0.0);
    gtsam::imuBias::ConstantBias prior_bias;

    estimator_viwo.initializePrior(0.0, prior_pose, prior_vel, prior_bias);
    estimator_vi.initializePrior(0.0, prior_pose, prior_vel, prior_bias);

    std::ofstream trajectory_out("trajectory_results.csv");
    trajectory_out << "time,gt_x,gt_y,gt_z,vi_x,vi_y,vi_z,viwo_x,viwo_y,viwo_z\n";

    constexpr double dt = 0.1; // 10 Hz Keyframe Rate
    constexpr int total_steps = 100;
    
    // Simulating a curved trajectory: circular arc R = 10m
    double radius = 10.0;
    double omega = 0.1; // rad/s

    std::default_random_engine generator(42);
    std::normal_distribution<double> wheel_noise(0.0, 0.015);  // Wheel encoder noise

    double ate_sq_vi = 0.0;
    double ate_sq_viwo = 0.0;

    for (int k = 1; k <= total_steps; ++k) {
        double t = k * dt;

        // Ground Truth SE(3) Trajectory
        double theta = omega * t;
        double gt_x = radius * std::sin(theta);
        double gt_y = radius * (1.0 - std::cos(theta));
        double gt_z = 0.0;
        gtsam::Pose3 gt_pose(gtsam::Rot3::Yaw(theta), gtsam::Point3(gt_x, gt_y, gt_z));
        gtsam::Vector3 gt_vel(1.0 * std::cos(theta), 1.0 * std::sin(theta), 0.0);

        // 1. Visual Relative Motion Measurement with metric scale drift
        double scale_drift = 1.0 + 0.03 * (k / 10.0); // reaches 1.30 by the
                                                     // last keyframe, so 30%
        gtsam::Pose3 prev_gt(gtsam::Rot3::Yaw(omega * (t - dt)), 
                            gtsam::Point3(radius * std::sin(omega * (t - dt)), 
                                          radius * (1.0 - std::cos(omega * (t - dt))), 0.0));
        gtsam::Pose3 rel_gt = prev_gt.between(gt_pose);
        
        // Corrupt visual relative measurement with scale drift
        gtsam::Point3 drifted_trans = rel_gt.translation() * scale_drift;
        gtsam::Pose3 meas_vis(rel_gt.rotation(), drifted_trans);

        // 2. Wheel Measurement (Uncorrupted by visual scale drift)
        gtsam::Vector3 meas_wheel(1.0 * dt + wheel_noise(generator), 0.0, 0.0);

        // --- Update VI Baseline Engine ---
        estimator_vi.addVisualFactor(k - 1, k, meas_vis);
        auto state_vi = estimator_vi.stepOptimization(k, gt_pose, gt_vel, prior_bias);

        // --- Update VIWO (Our Engine) ---
        estimator_viwo.addVisualFactor(k - 1, k, meas_vis);
        estimator_viwo.addWheelFactor(k - 1, k, meas_wheel);
        auto state_viwo = estimator_viwo.stepOptimization(k, gt_pose, gt_vel, prior_bias);

        // Compute Absolute Trajectory Error (ATE)
        double err_vi = (gt_pose.translation() - state_vi.pose.translation()).norm();
        double err_viwo = (gt_pose.translation() - state_viwo.pose.translation()).norm();

        ate_sq_vi += err_vi * err_vi;
        ate_sq_viwo += err_viwo * err_viwo;

        trajectory_out << t << "," 
                       << gt_x << "," << gt_y << "," << gt_z << ","
                       << state_vi.pose.x() << "," << state_vi.pose.y() << "," << state_vi.pose.z() << ","
                       << state_viwo.pose.x() << "," << state_viwo.pose.y() << "," << state_viwo.pose.z() << "\n";
    }

    trajectory_out.close();

    double ate_rmse_vi = std::sqrt(ate_sq_vi / total_steps);
    double ate_rmse_viwo = std::sqrt(ate_sq_viwo / total_steps);

    std::cout << "\n=== BENCHMARK RESULTS ===" << std::endl;
    std::cout << "Visual-Inertial (VI Baseline) ATE RMSE : " << ate_rmse_vi << " m" << std::endl;
    std::cout << "Visual-Inertial-Wheel (VIWO Proposed): " << ate_rmse_viwo << " m" << std::endl;
    std::cout << "ATE Drift Reduction: " << (1.0 - ate_rmse_viwo / ate_rmse_vi) * 100.0 << " %" << std::endl;

    return 0;
}
