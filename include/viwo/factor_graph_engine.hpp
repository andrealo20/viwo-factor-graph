/**
 * @file factor_graph_engine.hpp
 * @brief GTSAM ISAM2 Factor Graph Estimator for Visual-Inertial-Wheel Fusion
 * @author Andrea Loroni
 * @license MIT
 */

#ifndef VIWO_FACTOR_GRAPH_ENGINE_HPP
#define VIWO_FACTOR_GRAPH_ENGINE_HPP

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>

#include "viwo/wheel_odometry_factor.hpp"

namespace viwo {

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z, r,p,y)
using gtsam::symbol_shorthand::V; // Velocity (vx,vy,vz)
using gtsam::symbol_shorthand::B; // Bias (ax,ay,az, gx,gy,gz)

struct EstimatorState {
    gtsam::Pose3 pose;
    gtsam::Vector3 velocity;
    gtsam::imuBias::ConstantBias bias;
    double timestamp;
};

class FactorGraphEngine {
public:
    FactorGraphEngine() {
        // Configure ISAM2 Smoother
        gtsam::ISAM2Params params;
        params.rerelabelThreshold = 0.1;
        params.rerelabelPartial = true;
        isam2_ = std::make_unique<gtsam::ISAM2>(params);

        graph_ = gtsam::NonlinearFactorGraph();
        initial_values_ = gtsam::Values();
        key_index_ = 0;
    }

    /**
     * @brief Initialize Graph State with Prior Factors
     */
    void initializePrior(double timestamp, const gtsam::Pose3& prior_pose,
                        const gtsam::Vector3& prior_vel,
                        const gtsam::imuBias::ConstantBias& prior_bias) 
    {
        key_index_ = 0;

        // Noise models for priors
        auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished());
        auto vel_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << 0.1, 0.1, 0.1).finished());
        auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4).finished());

        graph_.addPrior(X(0), prior_pose, pose_noise);
        graph_.addPrior(V(0), prior_vel, vel_noise);
        graph_.addPrior(B(0), prior_bias, bias_noise);

        initial_values_.insert(X(0), prior_pose);
        initial_values_.insert(V(0), prior_vel);
        initial_values_.insert(B(0), prior_bias);

        current_state_.pose = prior_pose;
        current_state_.velocity = prior_vel;
        current_state_.bias = prior_bias;
        current_state_.timestamp = timestamp;

        isam2_->update(graph_, initial_values_);
        graph_.resize(0);
        initial_values_.clear();
    }

    /**
     * @brief Add Integrated Wheel Odometry Measurement Factor
     */
    void addWheelFactor(size_t from_idx, size_t to_idx,
                       const gtsam::Vector3& delta_p,
                       double std_dev = 0.02) 
    {
        auto wheel_noise = gtsam::noiseModel::Isotropic::Sigma(3, std_dev);
        graph_.add(boost::make_shared<WheelOdometryFactor>(
            X(from_idx), X(to_idx), delta_p, wheel_noise));
    }

    /**
     * @brief Add Visual Relative Motion Factor between Keyframes
     */
    void addVisualFactor(size_t from_idx, size_t to_idx,
                         const gtsam::Pose3& relative_pose,
                         double rot_std = 0.01, double trans_std = 0.03) 
    {
        auto vis_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << rot_std, rot_std, rot_std, trans_std, trans_std, trans_std).finished());
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            X(from_idx), X(to_idx), relative_pose, vis_noise));
    }

    /**
     * @brief Process and Optimise Frame Keypoint Step
     */
    EstimatorState stepOptimization(size_t new_idx, const gtsam::Pose3& pred_pose,
                                   const gtsam::Vector3& pred_vel,
                                   const gtsam::imuBias::ConstantBias& pred_bias) 
    {
        initial_values_.insert(X(new_idx), pred_pose);
        initial_values_.insert(V(new_idx), pred_vel);
        initial_values_.insert(B(new_idx), pred_bias);

        isam2_->update(graph_, initial_values_);
        isam2_->update(); // Double update for smoother convergence

        gtsam::Values current_estimate = isam2_->calculateEstimate();

        current_state_.pose = current_estimate.at<gtsam::Pose3>(X(new_idx));
        current_state_.velocity = current_estimate.at<gtsam::Vector3>(V(new_idx));
        current_state_.bias = current_estimate.at<gtsam::imuBias::ConstantBias>(B(new_idx));

        graph_.resize(0);
        initial_values_.clear();

        return current_state_;
    }

    const EstimatorState& getCurrentState() const { return current_state_; }

private:
    std::unique_ptr<gtsam::ISAM2> isam2_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_values_;
    size_t key_index_{0};
    EstimatorState current_state_;
};

} // namespace viwo

#endif // VIWO_FACTOR_GRAPH_ENGINE_HPP
