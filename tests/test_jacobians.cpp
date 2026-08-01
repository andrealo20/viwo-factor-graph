/**
 * @file test_jacobians.cpp
 * @brief Verify the analytical Jacobians of WheelOdometryFactor.
 *
 * The Jacobians are the actual contribution of this repository, and they are
 * the part that fails silently: a wrong derivative does not raise an error,
 * it produces an optimiser that converges slowly, or to the wrong answer, or
 * appears to work until the problem gets harder. The only way to know they
 * are right is to compare them against numerical differentiation taken under
 * the same retraction GTSAM uses internally.
 *
 * The residual is
 *
 *     r(Ti, Tj) = Ri^T (pj - pi) - dp_wheel
 *
 * and with GTSAM's body-frame perturbation, xi = [omega, v] in se(3):
 *
 *     H1 = [ skew(Ri^T (pj - pi))   -I ]
 *     H2 = [ 0                       Ri^T Rj ]
 *
 * Build (GTSAM and Eigen must be installed):
 *
 *     g++ -std=c++17 -O2 -Iinclude tests/test_jacobians.cpp \
 *         -lgtsam -lgtsam_unstable -o test_jacobians && ./test_jacobians
 */

#include "viwo/wheel_odometry_factor.hpp"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <cstdio>
#include <random>

using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Point3;
using gtsam::Vector3;

int main() {
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> ang(-1.0, 1.0);
    std::uniform_real_distribution<double> pos(-3.0, 3.0);
    std::uniform_real_distribution<double> wheel(-1.0, 1.0);

    auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);

    double worst_h1 = 0.0;
    double worst_h2 = 0.0;
    constexpr int TRIALS = 200;

    for (int t = 0; t < TRIALS; ++t) {
        const Pose3 Ti(Rot3::Expmap(Vector3(ang(rng), ang(rng), ang(rng))),
                       Point3(pos(rng), pos(rng), pos(rng)));
        const Pose3 Tj(Rot3::Expmap(Vector3(ang(rng), ang(rng), ang(rng))),
                       Point3(pos(rng), pos(rng), pos(rng)));
        const Vector3 dp(wheel(rng), wheel(rng), wheel(rng));

        viwo::WheelOdometryFactor factor(gtsam::Symbol('x', 0),
                                         gtsam::Symbol('x', 1), dp, noise);

        gtsam::Matrix H1, H2;
        factor.evaluateError(Ti, Tj, H1, H2);

        // Numerical derivatives use GTSAM's own retraction, so this compares
        // like with like rather than against a hand-rolled perturbation.
        const gtsam::Matrix N1 = gtsam::numericalDerivative21<Vector3, Pose3, Pose3>(
            [&](const Pose3& a, const Pose3& b) {
                return factor.evaluateError(a, b);
            }, Ti, Tj);
        const gtsam::Matrix N2 = gtsam::numericalDerivative22<Vector3, Pose3, Pose3>(
            [&](const Pose3& a, const Pose3& b) {
                return factor.evaluateError(a, b);
            }, Ti, Tj);

        worst_h1 = std::max(worst_h1, (H1 - N1).cwiseAbs().maxCoeff());
        worst_h2 = std::max(worst_h2, (H2 - N2).cwiseAbs().maxCoeff());
    }

    std::printf("Wheel factor Jacobians, %d random pose pairs\n", TRIALS);
    std::printf("  H1 (w.r.t. pose i): max |analytic - numerical| = %.3e\n", worst_h1);
    std::printf("  H2 (w.r.t. pose j): max |analytic - numerical| = %.3e\n", worst_h2);

    // Numerical differentiation itself carries error of order 1e-7, so this
    // threshold checks the analytical form rather than the finite difference.
    const double tol = 1e-5;
    const bool ok = worst_h1 < tol && worst_h2 < tol;
    std::printf("  %s (tolerance %.0e)\n", ok ? "PASS" : "FAIL", tol);
    return ok ? 0 : 1;
}
