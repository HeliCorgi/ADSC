#pragma once

#include <Eigen/Dense>

namespace adsc {

// Rigid-body rotational dynamics in the body frame.
//   Euler:      I w_dot = tau - w x (I w)
//   Kinematics: q_dot   = 1/2 * q (x) [0, w]
// Integrated with fixed-step RK4. The inertia tensor can be updated in place
// after debris capture.
//
// Quaternion convention (R8, stated once for the whole project): Eigen
// Quaterniond, Hamilton product, storage order (x, y, z, w) — always
// construct/read via the named accessors (.w(), .vec(), .coeffs()), never via
// positional storage assumptions. q maps body-frame vectors into the parent
// (inertial) frame; the body rate w sits on the right of the kinematics above.
class RigidBody {
public:
    RigidBody(const Eigen::Matrix3d& inertia,
              const Eigen::Quaterniond& q0,
              const Eigen::Vector3d& w0);

    // Advance one step of length dt under an applied body torque.
    void step(const Eigen::Vector3d& torque, double dt);

    // Replace the inertia tensor (e.g. after mass capture). A small isotropic
    // term is added for numerical conditioning before inversion.
    void set_inertia(const Eigen::Matrix3d& inertia, double regularization = 1e-6);

    const Eigen::Quaterniond& attitude() const { return q_; }
    const Eigen::Vector3d&    rate()     const { return w_; }
    const Eigen::Matrix3d&    inertia()  const { return I_; }

private:
    // State derivative given a body rate and applied torque.
    Eigen::Vector3d angular_accel(const Eigen::Vector3d& w,
                                  const Eigen::Vector3d& torque) const;

    Eigen::Matrix3d    I_;
    Eigen::Matrix3d    I_inv_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d    w_;
};

// Quaternion time-derivative for a body rate w. Exposed for the controller.
Eigen::Quaterniond quat_derivative(const Eigen::Quaterniond& q,
                                   const Eigen::Vector3d& w);

}  // namespace adsc
