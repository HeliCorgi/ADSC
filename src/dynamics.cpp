#include "adsc/dynamics.hpp"

namespace adsc {

Eigen::Quaterniond quat_derivative(const Eigen::Quaterniond& q,
                                   const Eigen::Vector3d& w) {
    Eigen::Quaterniond omega(0.0, w.x(), w.y(), w.z());
    Eigen::Quaterniond dq = q * omega;
    dq.coeffs() *= 0.5;
    return dq;
}

RigidBody::RigidBody(const Eigen::Matrix3d& inertia,
                     const Eigen::Quaterniond& q0,
                     const Eigen::Vector3d& w0)
    : q_(q0.normalized()), w_(w0) {
    set_inertia(inertia);
}

void RigidBody::set_inertia(const Eigen::Matrix3d& inertia, double regularization) {
    I_     = inertia + Eigen::Matrix3d::Identity() * regularization;
    I_inv_ = I_.inverse();
}

Eigen::Vector3d RigidBody::angular_accel(const Eigen::Vector3d& w,
                                         const Eigen::Vector3d& torque) const {
    return I_inv_ * (torque - w.cross(I_ * w));
}

void RigidBody::step(const Eigen::Vector3d& torque, double dt) {
    // RK4 on (q, w). Torque is held constant across the step (zero-order hold).
    auto q_dot = [](const Eigen::Quaterniond& q, const Eigen::Vector3d& w) {
        return quat_derivative(q, w);
    };

    const Eigen::Quaterniond q0 = q_;
    const Eigen::Vector3d    w0 = w_;

    Eigen::Quaterniond k1q = q_dot(q0, w0);
    Eigen::Vector3d    k1w = angular_accel(w0, torque);

    Eigen::Quaterniond q1 = q0; q1.coeffs() += 0.5 * dt * k1q.coeffs();
    Eigen::Vector3d    w1 = w0 + 0.5 * dt * k1w;
    Eigen::Quaterniond k2q = q_dot(q1, w1);
    Eigen::Vector3d    k2w = angular_accel(w1, torque);

    Eigen::Quaterniond q2 = q0; q2.coeffs() += 0.5 * dt * k2q.coeffs();
    Eigen::Vector3d    w2 = w0 + 0.5 * dt * k2w;
    Eigen::Quaterniond k3q = q_dot(q2, w2);
    Eigen::Vector3d    k3w = angular_accel(w2, torque);

    Eigen::Quaterniond q3 = q0; q3.coeffs() += dt * k3q.coeffs();
    Eigen::Vector3d    w3 = w0 + dt * k3w;
    Eigen::Quaterniond k4q = q_dot(q3, w3);
    Eigen::Vector3d    k4w = angular_accel(w3, torque);

    q_.coeffs() += (dt / 6.0) * (k1q.coeffs() + 2.0 * k2q.coeffs()
                                 + 2.0 * k3q.coeffs() + k4q.coeffs());
    q_.normalize();
    w_ += (dt / 6.0) * (k1w + 2.0 * k2w + 2.0 * k3w + k4w);
}

}  // namespace adsc
