#pragma once

#include <Eigen/Dense>

namespace adsc {

// Quaternion sliding-mode attitude controller with a boundary layer, sized for
// a Divert-and-Attitude-Control-System (DACS) with a firing deadband and a
// minimum-impulse / maximum-torque bound.
//
// WP2 extends the controller from regulation (fixed target attitude, zero
// target rate) to TRACKING of a moving target attitude (q_t, w_t):
//
//   Error quaternion:  q_e = q_t^* (x) q     (servicer body -> target body)
//   Relative rate:     w_e = w - C(q_e)^T w_t   [target rate in servicer body]
//   Sliding surface:   s   = w_e + lambda * sign(q_e0) * q_ev
//   Control law:       tau = w x (I w)
//                          + I * ( C^T w_t_dot - w_e x (C^T w_t) )   [feedforward]
//                          - I * lambda * sign(q_e0) * q_ev_dot
//                          - I * K * sat(s / phi)
//
// which drives s_dot = -K sat(s/phi), i.e. s -> 0. The feedforward carries the
// target's torque-free angular acceleration w_t_dot = I_t^{-1}(-w_t x I_t w_t)
// and the transport term -w_e x (C^T w_t) from differentiating C(q_e)^T w_t in
// the body frame; dropping either is masked by the reaching term in normal
// operation, which is why tests/test_sync.cpp certifies the feedforward with
// the reaching term disabled. The saturation (boundary layer phi) replaces the
// discontinuous sign() to suppress chatter.
//
// Regulation is mathematically the w_t = 0 special case. It is nevertheless
// kept as its own verbatim code path (identical to the pre-WP2 implementation)
// so its compiled floating-point behavior — and with it the quoted v2 detumble
// regression numbers — cannot drift through expression re-association; the
// special-case property is asserted numerically in tests/test_sync.cpp
// instead.
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

    // Regulation: body torque command driving (q, w) toward a fixed q_target
    // with zero rate. Behavior identical to the pre-WP2 controller.
    Eigen::Vector3d torque(const Eigen::Matrix3d& inertia,
                           const Eigen::Quaterniond& q,
                           const Eigen::Vector3d& w,
                           const Eigen::Quaterniond& q_target) const;

    // Tracking (WP2): body torque command driving (q, w) to follow a moving
    // target attitude q_t with target body rate w_t and torque-free target
    // acceleration w_t_dot (both expressed in the TARGET body frame).
    Eigen::Vector3d torque(const Eigen::Matrix3d& inertia,
                           const Eigen::Quaterniond& q,
                           const Eigen::Vector3d& w,
                           const Eigen::Quaterniond& q_t,
                           const Eigen::Vector3d& w_t,
                           const Eigen::Vector3d& w_t_dot) const;

    // Sliding variable s = w_e + lambda*sign(q_e0)*q_ev for the tracking
    // problem. Exposed so the feedforward-honesty test can observe whether the
    // surface stays parked near zero without any reaching-term rescue.
    Eigen::Vector3d sliding_surface(const Eigen::Quaterniond& q,
                                    const Eigen::Vector3d& w,
                                    const Eigen::Quaterniond& q_t,
                                    const Eigen::Vector3d& w_t) const;

    const Gains& gains() const { return g_; }

private:
    Gains g_;
};

}  // namespace adsc
