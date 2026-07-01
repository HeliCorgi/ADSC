#include "adsc/controller.hpp"

#include <cmath>

namespace adsc {

namespace {

// Componentwise saturation to [-1, 1] (boundary-layer softening of sign()).
Eigen::Vector3d saturate(const Eigen::Vector3d& v) {
    return v.cwiseMax(-1.0).cwiseMin(1.0);
}

double sgn(double x) { return (x > 0.0) - (x < 0.0); }

}  // namespace

Eigen::Vector3d SlidingModeController::torque(const Eigen::Matrix3d& inertia,
                                              const Eigen::Quaterniond& q,
                                              const Eigen::Vector3d& w,
                                              const Eigen::Quaterniond& q_target) const {
    // Error quaternion q_e = q_target^{-1} (x) q.
    Eigen::Quaterniond q_e = (q_target.conjugate() * q).normalized();
    double          q_e0  = q_e.w();
    Eigen::Vector3d q_ev  = q_e.vec();

    // sign(q_e0) selects the short-way rotation (avoids unwinding).
    double s_sign = (q_e0 >= 0.0) ? 1.0 : -1.0;

    // Sliding surface.
    Eigen::Vector3d s = w + g_.lambda * s_sign * q_ev;

    // d/dt of the error-quaternion vector part: q_ev_dot = 1/2 (q_e0 w + q_ev x w).
    Eigen::Vector3d q_ev_dot = 0.5 * (q_e0 * w + q_ev.cross(w));

    // Equivalent + reaching control.
    Eigen::Vector3d tau =
        w.cross(inertia * w)
        - inertia * (g_.lambda * s_sign * q_ev_dot)
        - inertia * (g_.k * saturate(s / g_.phi));

    // DACS realism: per-axis firing deadband on the surface, then clamp.
    for (int i = 0; i < 3; ++i) {
        if (std::abs(s(i)) < g_.deadband) {
            tau(i) = 0.0;                          // coast: thruster stays off
        } else {
            tau(i) = sgn(tau(i)) * std::min(std::abs(tau(i)), g_.max_torque);
        }
    }
    return tau;
}

}  // namespace adsc
