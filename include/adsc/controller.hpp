#pragma once

#include <Eigen/Dense>

namespace adsc {

// Quaternion sliding-mode attitude controller with a boundary layer, sized for
// a Divert-and-Attitude-Control-System (DACS) with a firing deadband and a
// minimum-impulse / maximum-torque bound.
//
// Sliding surface:  s = w + lambda * sign(q_e0) * q_ev
// Control law:      tau = w x (I w)
//                       - I * lambda * sign(q_e0) * q_ev_dot
//                       - I * K * sat(s / phi)
// which drives s_dot = -K sat(s/phi), i.e. s -> 0. The saturation (boundary
// layer phi) replaces the discontinuous sign() to suppress chatter.
//
// This is a continuous-torque approximation of a discrete-impulse DACS: per
// axis, torque below the deadband is coasted (thruster stays off) and the
// commanded torque is clamped to max_torque. It is a control-law demonstration,
// not a flight thruster allocator.
class SlidingModeController {
public:
    struct Gains {
        double lambda     = 0.6;    // surface slope (attitude vs rate)
        double k          = 0.08;   // reaching gain  [rad/s^2]
        double phi        = 0.03;   // boundary-layer half-width
        double deadband   = 0.015;  // per-axis surface deadband  [~0.86 deg]
        double max_torque = 0.05;   // per-axis torque clamp  [N m]
    };

    SlidingModeController() = default;
    explicit SlidingModeController(const Gains& g) : g_(g) {}

    // Body torque command driving (q, w) toward q_target with zero rate.
    Eigen::Vector3d torque(const Eigen::Matrix3d& inertia,
                           const Eigen::Quaterniond& q,
                           const Eigen::Vector3d& w,
                           const Eigen::Quaterniond& q_target) const;

    const Gains& gains() const { return g_; }

private:
    Gains g_;
};

}  // namespace adsc
