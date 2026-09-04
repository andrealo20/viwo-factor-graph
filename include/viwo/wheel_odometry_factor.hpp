/**
 * @file wheel_odometry_factor.hpp
 * @brief Custom GTSAM Factor for 3D/2D Wheel Odometry Body-Velocity Constraints
 * @author Andrea Loroni
 * @license MIT
 */

#ifndef VIWO_WHEEL_ODOMETRY_FACTOR_HPP
#define VIWO_WHEEL_ODOMETRY_FACTOR_HPP

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <iostream>
#include <string>

namespace viwo {

/**
 * @brief Custom GTSAM Factor constraining relative body translation between two SE(3) poses
 * based on integrated wheel encoder measurements.
 * 
 * Residual function:
 *   r(T_i, T_j) = R_i^T * (p_j - p_i) - delta_p_measured
 */
class WheelOdometryFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
private:
    gtsam::Vector3 measured_delta_p_; // Relative position delta in body frame i

public:
    typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> Base;
    typedef boost::shared_ptr<WheelOdometryFactor> shared_ptr;

    /**
     * @brief Default constructor, for serialisation.
     *
     * Eigen does not zero its fixed-size types, so leaving the measurement
     * uninitialised here would give a default-constructed factor a residual
     * built from whatever was on the stack, with nothing to signal it.
     */
    WheelOdometryFactor() : measured_delta_p_(gtsam::Vector3::Zero()) {}

    /**
     * @param key1 Key for Pose3 at time step i
     * @param key2 Key for Pose3 at time step j
     * @param measured_delta_p Measured body-frame displacement vector [dx, dy, dz]^T
     * @param model Noise model covariance matrix
     */
    WheelOdometryFactor(gtsam::Key key1, gtsam::Key key2,
                        const gtsam::Vector3& measured_delta_p,
                        const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key1, key2),
          measured_delta_p_(measured_delta_p) {}

    virtual ~WheelOdometryFactor() {}

    /**
     * @brief Compute error vector and exact analytical Jacobians w.r.t Pose_i and Pose_j
     */
    gtsam::Vector evaluateError(const gtsam::Pose3& Pose_i, const gtsam::Pose3& Pose_j,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const override 
    {
        const gtsam::Rot3& R_i = Pose_i.rotation();
        const gtsam::Rot3& R_j = Pose_j.rotation();
        const gtsam::Point3& p_i = Pose_i.translation();
        const gtsam::Point3& p_j = Pose_j.translation();

        // Predicted relative translation in body frame i
        gtsam::Point3 p_rel_world = p_j - p_i;
        gtsam::Point3 p_rel_body = R_i.unrotate(p_rel_world);

        // Error residual vector
        gtsam::Vector3 error = p_rel_body - measured_delta_p_;

        // Compute Exact Analytical Jacobians in GTSAM Local SE(3) Tangent Space
        if (H1) {
            // H1 (3x6): [ dError/dRot_i , dError/dTrans_i ]
            gtsam::Matrix36 H1_mat;
            H1_mat.block<3, 3>(0, 0) = gtsam::skewSymmetric(p_rel_body);
            H1_mat.block<3, 3>(0, 3) = -gtsam::Matrix33::Identity(); // Local body perturbation derivative
            *H1 = H1_mat;
        }

        if (H2) {
            // H2 (3x6): [ dError/dRot_j , dError/dTrans_j ]
            gtsam::Matrix36 H2_mat;
            H2_mat.block<3, 3>(0, 0) = gtsam::Matrix33::Zero();
            H2_mat.block<3, 3>(0, 3) = R_i.transpose() * R_j.matrix(); // Relative rotation R_i^T * R_j
            *H2 = H2_mat;
        }

        return error;
    }

    /**
     * @brief Print the factor and the measurement it carries.
     *
     * Without this, graph.print() shows the base class only, so the one piece
     * of state that distinguishes two wheel factors stays invisible exactly
     * when someone is inspecting a graph to find out why it is misbehaving.
     */
    void print(const std::string& s = "",
               const gtsam::KeyFormatter& keyFormatter =
                   gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "WheelOdometryFactor("
                  << keyFormatter(this->keys().at(0)) << ","
                  << keyFormatter(this->keys().at(1)) << ")\n"
                  << "  measured body-frame displacement: ["
                  << measured_delta_p_.transpose() << "]\n";
        if (this->noiseModel()) {
            this->noiseModel()->print("  noise model: ");
        }
    }

    /**
     * @brief Value equality, which GTSAM's testable concept expects.
     */
    bool equals(const gtsam::NonlinearFactor& expected,
                double tol = 1e-9) const override
    {
        const WheelOdometryFactor* other =
            dynamic_cast<const WheelOdometryFactor*>(&expected);
        return other != NULL
            && Base::equals(expected, tol)
            && gtsam::equal_with_abs_tol(measured_delta_p_,
                                         other->measured_delta_p_, tol);
    }

    /** @brief The measurement this factor was built with. */
    const gtsam::Vector3& measured() const { return measured_delta_p_; }

    /**
     * @brief Deep copy method for GTSAM factory
     */
    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new WheelOdometryFactor(*this)));
    }
};

} // namespace viwo

#endif // VIWO_WHEEL_ODOMETRY_FACTOR_HPP
