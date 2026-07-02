#include "adsc/controller.hpp"

#include <algorithm>
#include <cmath>

namespace adsc {

namespace {

// Componentwise saturation to [-1, 1] (boundary-layer softening of sign()).
Eigen::Vector3d saturate(const Eigen::Vector3d& v) {
    return v.cwiseMax(-1.0).cwiseMin(1.0);
}

double sgn(double x) { return (x > 0.0) - (x < 0.0); }

}  // namespace

// Regulation path. Kept verbatim from the pre-WP2 implementation (NOT routed
// through the tracking overload) so its compiled floating-point behavior is
// bit-identical to v2 and the quoted detumble regression numbers cannot drift;
// the w_t = 0 equivalence with the tracking law is asserted in
// tests/test_sync.cpp.
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

Eigen::Vector3d SlidingModeController::sliding_surface(
        const Eigen::Quaterniond& q,
        const Eigen::Vector3d& w,
        const Eigen::Quaterniond& q_t,
        const Eigen::Vector3d& w_t) const {
    const Eigen::Quaterniond q_e = (q_t.conjugate() * q).normalized();
    const double s_sign = (q_e.w() >= 0.0) ? 1.0 : -1.0;
    const Eigen::Vector3d w_e = w - q_e.conjugate() * w_t;
    return w_e + g_.lambda * s_sign * q_e.vec();
}

// Tracking path (WP2).
Eigen::Vector3d SlidingModeController::torque(const Eigen::Matrix3d& inertia,
                                              const Eigen::Quaterniond& q,
                                              const Eigen::Vector3d& w,
                                              const Eigen::Quaterniond& q_t,
                                              const Eigen::Vector3d& w_t,
                                              const Eigen::Vector3d& w_t_dot) const {
    // Error quaternion q_e = q_t^{-1} (x) q rotates servicer-body vectors into
    // the target body frame; q_e^* therefore brings target-frame vectors into
    // the servicer body frame.
    const Eigen::Quaterniond q_e = (q_t.conjugate() * q).normalized();
    const double          q_e0 = q_e.w();
    const Eigen::Vector3d q_ev = q_e.vec();

    // sign(q_e0) selects the short-way rotation (avoids unwinding).
    const double s_sign = (q_e0 >= 0.0) ? 1.0 : -1.0;

    // Target rate and torque-free acceleration in the servicer body frame.
    const Eigen::Vector3d w_t_b     = q_e.conjugate() * w_t;
    const Eigen::Vector3d w_t_dot_b = q_e.conjugate() * w_t_dot;

    // Relative rate and sliding surface.
    const Eigen::Vector3d w_e = w - w_t_b;
    const Eigen::Vector3d s   = w_e + g_.lambda * s_sign * q_ev;

    // Relative error kinematics: q_e_dot = 1/2 q_e (x) [0, w_e].
    const Eigen::Vector3d q_ev_dot = 0.5 * (q_e0 * w_e + q_ev.cross(w_e));

    // Equivalent control + target-motion feedforward + reaching term. The
    // body-frame derivative of the rotated target rate is
    //   d/dt[C^T w_t] = C^T w_t_dot - w_e x (C^T w_t),
    // i.e. the transport term -w_e x w_t_b must be carried. In normal closed
    // loop the reaching term masks a missing w_t_dot; the feedforward-honesty
    // test (reaching disabled) catches that. The transport term vanishes on a
    // synchronized trajectory (w_e ~ 0) and is exercised during the acceptance
    // test's convergence phase instead.
    Eigen::Vector3d tau =
        w.cross(inertia * w)
        + inertia * (w_t_dot_b - w_e.cross(w_t_b))
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
