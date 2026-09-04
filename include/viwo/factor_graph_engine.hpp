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
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>

#include <boost/shared_ptr.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

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

/**
 * @brief Continuous-time IMU noise densities and the gravity magnitude.
 *
 * The sigmas are spectral densities, not per-sample standard deviations: an
 * accelerometer sigma of 0.01 means 0.01 m/s^2 per sqrt(Hz). A simulator that
 * produces samples at rate f must therefore draw its per-sample noise with
 * standard deviation sigma / sqrt(dt), dt = 1/f. The values below are typical
 * of a mid-grade MEMS unit.
 */
struct ImuNoiseParams {
    double gravity = 9.81;           // m/s^2, magnitude only, sign set by the frame convention
    double accel_sigma = 0.01;       // m/s^2 / sqrt(Hz)
    double gyro_sigma = 1.75e-4;     // rad/s / sqrt(Hz)
    double integration_sigma = 1e-4; // m/s^2, position integration uncertainty
    double accel_bias_rw = 1.0e-3;   // m/s^3 / sqrt(Hz), accelerometer bias random walk
    double gyro_bias_rw = 1.0e-5;    // rad/s^2 / sqrt(Hz), gyroscope bias random walk
};

class FactorGraphEngine {
public:
    explicit FactorGraphEngine(const ImuNoiseParams& imu_noise = ImuNoiseParams())
        : imu_noise_(imu_noise),
          preintegrated_(makeImuParams(imu_noise))
    {
        // ISAM2 defaults. Earlier versions set "rerelabelThreshold" and
        // "rerelabelPartial", which are not members of ISAM2Params, so the
        // project did not compile against GTSAM. The tuned settings made no
        // measurable difference on this problem when tried, so the defaults
        // are kept: fewer knobs whose effect has not been measured.
        gtsam::ISAM2Params params;
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

        auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished());
        auto vel_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << 0.1, 0.1, 0.1).finished());

        // The bias prior is loose enough to admit a real accelerometer bias of
        // a few hundredths of a m/s^2 and a gyro bias of a few tenths of a
        // mrad/s, which is what the graph is now asked to estimate. The
        // earlier value of 1e-3 was tight enough to pin the bias at its
        // initial guess, harmless only because no factor touched it.
        //
        // The vertical accelerometer bias is the exception and is held tight
        // on purpose. This trajectory is level and turns only in yaw, so the
        // body z axis never tilts and a constant b_z is indistinguishable from
        // an error in the gravity magnitude: it is structurally unobservable,
        // not merely weakly observed. Left loose it would be absorbed as free
        // vertical acceleration, and 0.5 * b * t^2 with b of a hundredth of a
        // m/s^2 is tens of centimetres of drift over the ten second run, which
        // is the size of the quantity this benchmark exists to measure. On a
        // trajectory with pitch or roll excitation the prior can be relaxed.
        auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 5e-2, 5e-2, 1e-4, 5e-4, 5e-4, 5e-4).finished());

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

        // Start accumulating from the prior bias, so that the first
        // preintegrated interval is linearised about the same bias the graph
        // is initialised with.
        preintegrated_.resetIntegrationAndSetBias(prior_bias);

        isam2_->update(graph_, initial_values_);
        graph_.resize(0);
        initial_values_.clear();
    }

    /**
     * @brief Accumulate one raw IMU sample into the running preintegration.
     *
     * @param measured_acc   Specific force in the body frame [m/s^2], that is
     *                       the acceleration of the body minus gravity, both
     *                       expressed in the body frame. At rest with the body
     *                       z axis pointing up this reads (0, 0, +g).
     * @param measured_gyro  Angular rate of the body frame with respect to the
     *                       world frame, expressed in the body frame [rad/s].
     * @param dt             Sample interval [s].
     */
    void integrateImuMeasurement(const gtsam::Vector3& measured_acc,
                                 const gtsam::Vector3& measured_gyro,
                                 double dt)
    {
        preintegrated_.integrateMeasurement(measured_acc, measured_gyro, dt);
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
     * @brief Close the current keyframe interval and optimise.
     *
     * Adds the inertial factors for the interval (new_idx - 1, new_idx), takes
     * the linearisation point for the new pose and velocity from the
     * preintegrated prediction, runs ISAM2, and restarts the preintegration
     * about the freshly estimated bias.
     *
     * The linearisation point comes from predict() and never from ground
     * truth. Both estimators in the benchmark go through this same code path,
     * so the VI and VIWO comparison stays fair: the only difference between
     * them is the presence of the wheel factor.
     */
    EstimatorState stepOptimization(size_t new_idx, double timestamp)
    {
        if (new_idx != key_index_ + 1) {
            throw std::runtime_error(
                "FactorGraphEngine::stepOptimization: keyframe indices must be consecutive");
        }

        const double dt_ij = preintegrated_.deltaTij();
        if (!(dt_ij > 0.0)) {
            // Without an inertial factor V(new_idx) and B(new_idx) would enter
            // the graph untouched by any factor: ISAM2 would never eliminate
            // them and calculateEstimate() would hand back the values that
            // were inserted. Refuse instead of returning a fake estimate.
            throw std::runtime_error(
                "FactorGraphEngine::stepOptimization: no IMU measurement was integrated "
                "for this keyframe interval");
        }

        const size_t prev_idx = key_index_;

        // Linearisation point for the new state: dead reckoning from the last
        // estimate through the preintegrated measurement.
        const gtsam::NavState prev_nav(current_state_.pose, current_state_.velocity);
        const gtsam::NavState pred_nav =
            preintegrated_.predict(prev_nav, current_state_.bias);

        initial_values_.insert(X(new_idx), pred_nav.pose());
        initial_values_.insert(V(new_idx), gtsam::Vector3(pred_nav.velocity()));
        initial_values_.insert(B(new_idx), current_state_.bias);

        // ImuFactor is a 5-way factor on (X_i, V_i, X_j, V_j, B_i). It carries
        // no constraint on B_j, so the bias chain is closed by a random walk
        // between factor whose sigma grows with sqrt(dt).
        graph_.add(gtsam::ImuFactor(X(prev_idx), V(prev_idx),
                                    X(new_idx), V(new_idx),
                                    B(prev_idx), preintegrated_));

        const double sigma_ba = imu_noise_.accel_bias_rw * std::sqrt(dt_ij);
        const double sigma_bg = imu_noise_.gyro_bias_rw * std::sqrt(dt_ij);
        auto bias_walk_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << sigma_ba, sigma_ba, sigma_ba,
                                 sigma_bg, sigma_bg, sigma_bg).finished());
        graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
            B(prev_idx), B(new_idx), gtsam::imuBias::ConstantBias(), bias_walk_noise));

        isam2_->update(graph_, initial_values_);
        isam2_->update(); // Double update for smoother convergence

        // Fetched by key rather than through a full calculateEstimate(). The
        // full call is linear in the number of keyframes, so calling it once
        // per frame makes the whole run quadratic, which is the cost
        // incremental smoothing exists to avoid.
        current_state_.pose = isam2_->calculateEstimate<gtsam::Pose3>(X(new_idx));
        current_state_.velocity = isam2_->calculateEstimate<gtsam::Vector3>(V(new_idx));
        current_state_.bias =
            isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(new_idx));
        current_state_.timestamp = timestamp;

        graph_.resize(0);
        initial_values_.clear();
        key_index_ = new_idx;

        // Restart the preintegration about the bias the graph just estimated,
        // so the next interval is linearised where the solution actually is.
        preintegrated_.resetIntegrationAndSetBias(current_state_.bias);

        return current_state_;
    }

    /**
     * @brief Every pose in the graph, after the last update.
     *
     * ISAM2 is a smoother: a pose keeps moving as later keyframes constrain
     * it, so the estimate of X(k) taken at the moment k was added is not the
     * estimate the run ends with. An absolute trajectory error has to be
     * computed over the final trajectory, which is what this returns. Calling
     * the full calculateEstimate() once, at the end, costs nothing.
     */
    std::vector<gtsam::Pose3> getSmoothedTrajectory() const
    {
        const gtsam::Values values = isam2_->calculateEstimate();
        std::vector<gtsam::Pose3> poses;
        poses.reserve(key_index_ + 1);
        for (size_t k = 0; k <= key_index_; ++k) {
            poses.push_back(values.at<gtsam::Pose3>(X(k)));
        }
        return poses;
    }

    /** @brief Velocity at every keyframe, after the last update. */
    std::vector<gtsam::Vector3> getSmoothedVelocities() const
    {
        const gtsam::Values values = isam2_->calculateEstimate();
        std::vector<gtsam::Vector3> velocities;
        velocities.reserve(key_index_ + 1);
        for (size_t k = 0; k <= key_index_; ++k) {
            velocities.push_back(values.at<gtsam::Vector3>(V(k)));
        }
        return velocities;
    }

    const EstimatorState& getCurrentState() const { return current_state_; }

    const ImuNoiseParams& getImuNoiseParams() const { return imu_noise_; }

private:
    /**
     * @brief Build the preintegration parameters.
     *
     * MakeSharedU is the Z-up convention: it sets n_gravity to (0, 0, -g), so
     * the navigation frame has its z axis pointing away from the centre of the
     * Earth. That matches the benchmark trajectory, which is planar in the
     * world xy plane with attitude given by a yaw rotation about world z. The
     * synthetic accelerometer must therefore report specific force
     * f_b = R_wb^T (a_w - g_w) = R_wb^T (a_w + (0, 0, g)), which reads
     * (0, 0, +9.81) when the platform is level and not accelerating.
     * MakeSharedD would put gravity at (0, 0, +g) instead and the estimator
     * would see a spurious 2g of vertical specific force. That mistake is not
     * subtle in its effect: it produces hundreds of metres of vertical error
     * over this ten second run.
     */
    static boost::shared_ptr<gtsam::PreintegrationParams>
    makeImuParams(const ImuNoiseParams& n)
    {
        boost::shared_ptr<gtsam::PreintegrationParams> p =
            gtsam::PreintegrationParams::MakeSharedU(n.gravity);
        p->setAccelerometerCovariance(
            gtsam::Matrix33::Identity() * (n.accel_sigma * n.accel_sigma));
        p->setGyroscopeCovariance(
            gtsam::Matrix33::Identity() * (n.gyro_sigma * n.gyro_sigma));
        p->setIntegrationCovariance(
            gtsam::Matrix33::Identity() * (n.integration_sigma * n.integration_sigma));
        return p;
    }

    ImuNoiseParams imu_noise_;
    gtsam::PreintegratedImuMeasurements preintegrated_;
    std::unique_ptr<gtsam::ISAM2> isam2_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_values_;
    size_t key_index_{0};
    EstimatorState current_state_;
};

} // namespace viwo

#endif // VIWO_FACTOR_GRAPH_ENGINE_HPP
