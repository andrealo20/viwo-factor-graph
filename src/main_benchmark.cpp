/**
 * @file main_benchmark.cpp
 * @brief Benchmark Harness evaluating Visual-Inertial vs Visual-Inertial-Wheel Fusion
 * @author Andrea Loroni
 * @license MIT
 *
 * Two estimators are given identical measurements and differ in exactly one
 * respect: the second also receives the wheel factor. Both run the same code
 * path inside FactorGraphEngine, and neither is handed ground truth: every new
 * state is initialised from the preintegrated IMU prediction.
 *
 * The trajectory is a planar circular arc, radius 10 m, yaw rate 0.1 rad/s,
 * forward speed 1 m/s, which is the arc a differential-drive platform follows
 * at constant wheel speeds. Everything the estimators see is derived from it.
 */

#include "viwo/factor_graph_engine.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

/**
 * @brief Deterministic Gaussian source, reproducible across standard libraries.
 *
 * std::normal_distribution is not specified to produce the same sequence on
 * libstdc++, libc++ and MSVC, and neither is std::default_random_engine. Since
 * this benchmark prints numbers a reader is invited to reproduce, both are
 * replaced: std::mt19937 is specified exactly by the standard, and Box-Muller
 * is written out here so the mapping from its output to normal samples is
 * fixed too.
 */
class DeterministicGaussian {
public:
    explicit DeterministicGaussian(std::uint32_t seed)
        : rng_(seed), spare_(0.0), have_spare_(false) {}

    double operator()()
    {
        if (have_spare_) {
            have_spare_ = false;
            return spare_;
        }
        double u1 = uniform();
        while (u1 <= 1e-12) {  // log(0) guard
            u1 = uniform();
        }
        const double u2 = uniform();
        const double radius = std::sqrt(-2.0 * std::log(u1));
        const double angle = 2.0 * kPi * u2;
        spare_ = radius * std::sin(angle);
        have_spare_ = true;
        return radius * std::cos(angle);
    }

private:
    double uniform()
    {
        // Written out rather than taken from std::uniform_real_distribution,
        // which is also unspecified across implementations.
        return (static_cast<double>(rng_()) + 0.5) * (1.0 / 4294967296.0);
    }

    std::mt19937 rng_;
    double spare_;
    bool have_spare_;
};

// ---------------------------------------------------------------------------
// Ground truth
// ---------------------------------------------------------------------------

constexpr double kRadius = 10.0;   // m
constexpr double kOmega = 0.1;     // rad/s, yaw rate
constexpr double kSpeed = kRadius * kOmega;  // m/s, 1.0 by construction

gtsam::Pose3 truePose(double t)
{
    const double theta = kOmega * t;
    return gtsam::Pose3(gtsam::Rot3::Yaw(theta),
                        gtsam::Point3(kRadius * std::sin(theta),
                                      kRadius * (1.0 - std::cos(theta)),
                                      0.0));
}

gtsam::Vector3 trueVelocity(double t)
{
    const double theta = kOmega * t;
    return gtsam::Vector3(kSpeed * std::cos(theta), kSpeed * std::sin(theta), 0.0);
}

gtsam::Vector3 trueAcceleration(double t)
{
    // Differentiating trueVelocity: centripetal, magnitude speed * omega.
    const double theta = kOmega * t;
    const double a = kSpeed * kOmega;
    return gtsam::Vector3(-a * std::sin(theta), a * std::cos(theta), 0.0);
}

/**
 * @brief Specific force in the body frame, the quantity an accelerometer reads.
 *
 * f_b = R_wb^T (a_w - g_w). Written in that general form rather than as the
 * constant it evaluates to on this particular arc, because the constant is
 * correct only by a cancellation between the centripetal term and the yaw
 * rotation, and hardcoding it would silently produce wrong measurements if
 * anyone changed the radius, the rate or the planarity.
 *
 * The gravity vector must match the convention PreintegrationParams was built
 * with. MakeSharedU sets n_gravity to (0, 0, -g), so g_w is the acceleration
 * due to gravity, pointing down, and a level stationary platform reads
 * (0, 0, +g). On this arc the expression evaluates to (0, 0.1, 9.81).
 */
gtsam::Vector3 trueSpecificForce(double t, double gravity)
{
    const gtsam::Vector3 g_w(0.0, 0.0, -gravity);
    const gtsam::Vector3 a_w = trueAcceleration(t);
    return truePose(t).rotation().unrotate(gtsam::Point3(a_w - g_w));
}

/** @brief Body-frame angular rate. Pure yaw, so only the z component is non-zero. */
gtsam::Vector3 trueAngularRate(double)
{
    return gtsam::Vector3(0.0, 0.0, kOmega);
}

double ateRmse(const std::vector<gtsam::Pose3>& estimated,
               const std::vector<gtsam::Pose3>& truth)
{
    double sum_sq = 0.0;
    const size_t n = estimated.size();
    for (size_t i = 0; i < n; ++i) {
        const double e = (truth[i].translation() - estimated[i].translation()).norm();
        sum_sq += e * e;
    }
    return std::sqrt(sum_sq / static_cast<double>(n));
}

}  // namespace

int main()
{
    std::cout << "=== Running VIWO Factor-Graph State Estimator Benchmark ===" << std::endl;

    viwo::ImuNoiseParams imu_noise;
    viwo::FactorGraphEngine estimator_viwo(imu_noise);  // Visual-Inertial-Wheel
    viwo::FactorGraphEngine estimator_vi(imu_noise);    // Visual-Inertial only

    constexpr double dt = 0.1;            // 10 Hz keyframes
    constexpr int total_steps = 100;
    constexpr int imu_rate_hz = 200;
    constexpr int imu_per_keyframe = static_cast<int>(imu_rate_hz * dt);  // 20
    constexpr double imu_dt = 1.0 / imu_rate_hz;
    constexpr double wheel_sigma = 0.015;  // m, per keyframe interval

    // A constant sensor bias the estimators are not told about. There is no
    // vertical accelerometer bias: this trajectory is level and turns only in
    // yaw, so a constant b_z is indistinguishable from an error in the gravity
    // magnitude and cannot be estimated from these measurements at all.
    // Injecting one would add unobservable vertical drift to every run and
    // would say nothing about the fusion being measured. See the bias prior in
    // factor_graph_engine.hpp.
    const gtsam::Vector3 true_accel_bias(0.02, -0.015, 0.0);   // m/s^2
    const gtsam::Vector3 true_gyro_bias(1.0e-4, -2.0e-4, 3.0e-4);  // rad/s

    // Per-sample standard deviations from the continuous-time densities.
    const double accel_noise = imu_noise.accel_sigma / std::sqrt(imu_dt);
    const double gyro_noise = imu_noise.gyro_sigma / std::sqrt(imu_dt);

    // One generator for the whole run, so both estimators see exactly the same
    // measurements and the comparison isolates the wheel factor.
    DeterministicGaussian gauss(42u);

    const gtsam::Pose3 prior_pose = truePose(0.0);
    const gtsam::Vector3 prior_vel = trueVelocity(0.0);
    const gtsam::imuBias::ConstantBias prior_bias;  // zero, the honest initial guess

    estimator_viwo.initializePrior(0.0, prior_pose, prior_vel, prior_bias);
    estimator_vi.initializePrior(0.0, prior_pose, prior_vel, prior_bias);

    std::vector<gtsam::Pose3> truth;
    std::vector<double> stamps;
    truth.push_back(prior_pose);
    stamps.push_back(0.0);

    for (int k = 1; k <= total_steps; ++k) {
        const double t = k * dt;
        const double t_prev = (k - 1) * dt;

        // --- Inertial measurements over the interval -----------------------
        for (int s = 0; s < imu_per_keyframe; ++s) {
            // Sampled at interval midpoints, which is where a rectangle rule
            // is second-order rather than first.
            const double t_s = t_prev + (static_cast<double>(s) + 0.5) * imu_dt;

            const gtsam::Vector3 f_b = trueSpecificForce(t_s, imu_noise.gravity);
            const gtsam::Vector3 w_b = trueAngularRate(t_s);

            // Each draw goes into its own named variable before the vector is
            // built: the order in which function arguments are evaluated is
            // unspecified in C++, so three inline calls would make the noise
            // sequence depend on the compiler.
            const double na_x = gauss();
            const double na_y = gauss();
            const double na_z = gauss();
            const double ng_x = gauss();
            const double ng_y = gauss();
            const double ng_z = gauss();

            const gtsam::Vector3 acc_meas =
                f_b + true_accel_bias + accel_noise * gtsam::Vector3(na_x, na_y, na_z);
            const gtsam::Vector3 gyro_meas =
                w_b + true_gyro_bias + gyro_noise * gtsam::Vector3(ng_x, ng_y, ng_z);

            estimator_vi.integrateImuMeasurement(acc_meas, gyro_meas, imu_dt);
            estimator_viwo.integrateImuMeasurement(acc_meas, gyro_meas, imu_dt);
        }

        // --- Visual relative motion, corrupted by metric scale drift --------
        const gtsam::Pose3 rel_gt = truePose(t_prev).between(truePose(t));
        const double scale_drift = 1.0 + 0.03 * (k / 10.0);  // reaches 1.30, so 30 per cent
        const gtsam::Pose3 meas_vis(rel_gt.rotation(),
                                    gtsam::Point3(rel_gt.translation() * scale_drift));

        // --- Wheel displacement, in the body frame -------------------------
        const double wheel_noise_draw = gauss();
        const gtsam::Vector3 meas_wheel(kSpeed * dt + wheel_sigma * wheel_noise_draw,
                                        0.0, 0.0);

        estimator_vi.addVisualFactor(k - 1, k, meas_vis);
        estimator_vi.stepOptimization(k, t);

        estimator_viwo.addVisualFactor(k - 1, k, meas_vis);
        estimator_viwo.addWheelFactor(k - 1, k, meas_wheel, wheel_sigma);
        estimator_viwo.stepOptimization(k, t);

        truth.push_back(truePose(t));
        stamps.push_back(t);
    }

    // --- Absolute trajectory error, over the smoothed trajectory -----------
    //
    // ISAM2 is a smoother, so a pose keeps moving as later keyframes constrain
    // it. Accumulating the error on each pose at the instant it was added, as
    // an earlier version did, mixes estimates taken at different times and is a
    // filtered error rather than an ATE. Both trajectories are therefore read
    // back once, at the end.
    const std::vector<gtsam::Pose3> traj_vi = estimator_vi.getSmoothedTrajectory();
    const std::vector<gtsam::Pose3> traj_viwo = estimator_viwo.getSmoothedTrajectory();
    const std::vector<gtsam::Vector3> vel_vi = estimator_vi.getSmoothedVelocities();
    const std::vector<gtsam::Vector3> vel_viwo = estimator_viwo.getSmoothedVelocities();

    std::ofstream out("trajectory_results.csv");
    out << "time,gt_x,gt_y,gt_z,vi_x,vi_y,vi_z,viwo_x,viwo_y,viwo_z,"
           "gt_vx,gt_vy,vi_vx,vi_vy,viwo_vx,viwo_vy\n";
    out << std::setprecision(10);
    for (size_t i = 0; i < truth.size(); ++i) {
        const gtsam::Point3 g = truth[i].translation();
        const gtsam::Point3 a = traj_vi[i].translation();
        const gtsam::Point3 b = traj_viwo[i].translation();
        const gtsam::Vector3 gv = trueVelocity(stamps[i]);
        out << stamps[i] << ","
            << g.x() << "," << g.y() << "," << g.z() << ","
            << a.x() << "," << a.y() << "," << a.z() << ","
            << b.x() << "," << b.y() << "," << b.z() << ","
            << gv.x() << "," << gv.y() << ","
            << vel_vi[i].x() << "," << vel_vi[i].y() << ","
            << vel_viwo[i].x() << "," << vel_viwo[i].y() << "\n";
    }
    out.close();

    const double ate_vi = ateRmse(traj_vi, truth);
    const double ate_viwo = ateRmse(traj_viwo, truth);

    std::cout << "\n=== BENCHMARK RESULTS ===" << std::endl;
    std::cout << "Visual-Inertial (VI Baseline) ATE RMSE : " << ate_vi << " m" << std::endl;
    std::cout << "Visual-Inertial-Wheel (VIWO Proposed) : " << ate_viwo << " m" << std::endl;
    std::cout << "ATE Drift Reduction: "
              << (1.0 - ate_viwo / ate_vi) * 100.0 << " %" << std::endl;

    const gtsam::imuBias::ConstantBias bias_viwo = estimator_viwo.getCurrentState().bias;
    std::cout << "\nEstimated accelerometer bias: "
              << bias_viwo.accelerometer().transpose()
              << "   (true " << true_accel_bias.transpose() << ")" << std::endl;
    std::cout << "Estimated gyroscope bias:     "
              << bias_viwo.gyroscope().transpose()
              << "   (true " << true_gyro_bias.transpose() << ")" << std::endl;
    std::cout << "\nThe bias columns are the evidence that the inertial states are "
                 "constrained: they are estimated, not returned as inserted."
              << std::endl;

    return 0;
}
