#include "adsc/estimator.hpp"

#include <cmath>

namespace adsc {

namespace {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<     0.0, -v.z(),  v.y(),
          v.z(),    0.0, -v.x(),
         -v.y(),  v.x(),    0.0;
    return m;
}

// Joseph-form measurement update on (x-error e, P) for residual z_res with
// Jacobian H and noise R. Returns the NIS and writes the state correction.
// Symmetrizes P afterwards. Templated on the (small, fixed) dimensions.
template <int N, int M>
double joseph_update(Eigen::Matrix<double, N, 1>& correction,
                     Eigen::Matrix<double, N, N>& P,
                     const Eigen::Matrix<double, M, 1>& z_res,
                     const Eigen::Matrix<double, M, N>& H,
                     const Eigen::Matrix<double, M, M>& R) {
    const Eigen::Matrix<double, M, M> S = H * P * H.transpose() + R;
    const Eigen::Matrix<double, M, M> S_inv = S.inverse();
    const Eigen::Matrix<double, N, M> K = P * H.transpose() * S_inv;

    correction = K * z_res;

    const Eigen::Matrix<double, N, N> IKH =
        Eigen::Matrix<double, N, N>::Identity() - K * H;
    P = IKH * P * IKH.transpose() + K * R * K.transpose();
    // Enforce symmetry every update. .eval() breaks the aliasing of P with
    // P.transpose() on the RHS (in-place evaluation would blend already-
    // overwritten entries and NOT produce a symmetric matrix).
    P = (0.5 * (P + P.transpose())).eval();

    return z_res.dot(S_inv * z_res);
}

}  // namespace

double GaussianSource::uniform01() {
    // (raw + 1) / 2^32 in (0, 1]: safe for log().
    return (static_cast<double>(gen_()) + 1.0) / 4294967296.0;
}

double GaussianSource::sample() {
    if (have_spare_) {
        have_spare_ = false;
        return spare_;
    }
    const double u1 = uniform01();
    const double u2 = uniform01();
    const double m  = std::sqrt(-2.0 * std::log(u1));
    spare_      = m * std::sin(2.0 * kPi * u2);
    have_spare_ = true;
    return m * std::cos(2.0 * kPi * u2);
}

Eigen::Vector3d GaussianSource::sample3(double sigma) {
    return sigma * Eigen::Vector3d(sample(), sample(), sample());
}

Eigen::Quaterniond quat_exp(const Eigen::Vector3d& v) {
    const double a = v.norm();
    if (a < 1e-12) {
        return Eigen::Quaterniond(1.0, 0.5 * v.x(), 0.5 * v.y(), 0.5 * v.z())
            .normalized();
    }
    const double s = std::sin(0.5 * a) / a;
    return Eigen::Quaterniond(std::cos(0.5 * a), s * v.x(), s * v.y(), s * v.z());
}

Eigen::Vector3d quat_residual(const Eigen::Quaterniond& q_ref,
                              const Eigen::Quaterniond& q) {
    Eigen::Quaterniond d = q_ref.conjugate() * q;
    if (d.w() < 0.0) d.coeffs() = -d.coeffs();
    return 2.0 * d.vec();
}

// ---------------------------------------------------------------------------
// TranslationEkf
// ---------------------------------------------------------------------------

TranslationEkf::TranslationEkf(const CwModel& cw, const Vector6d& x0,
                               const Eigen::Matrix<double, 6, 6>& P0)
    : cw_(cw), x_(x0), P_(P0) {}

void TranslationEkf::predict(double dt, double q_vel_sigma) {
    const Matrix6d phi = cw_.stm(dt);   // analytic CW STM (relmotion, WP1)
    x_ = phi * x_;
    Matrix6d Q = Matrix6d::Zero();
    Q.block<3, 3>(3, 3) =
        q_vel_sigma * q_vel_sigma * Eigen::Matrix3d::Identity();
    P_ = phi * P_ * phi.transpose() + Q;
    P_ = (0.5 * (P_ + P_.transpose())).eval();  // .eval(): avoid self-aliasing
}

double TranslationEkf::update(double range_meas, const Eigen::Vector3d& los_meas,
                              double sigma_range, double sigma_los) {
    const Eigen::Vector3d r = x_.head<3>();
    const double rn = r.norm();
    const Eigen::Vector3d r_hat = r / rn;

    // z = [ |r| ; r/|r| ],  H = [ r_hat^T 0 ; (I - r_hat r_hat^T)/|r| 0 ].
    Eigen::Matrix<double, 4, 1> z_res;
    z_res(0) = range_meas - rn;
    z_res.tail<3>() = los_meas - r_hat;

    Eigen::Matrix<double, 4, 6> H = Eigen::Matrix<double, 4, 6>::Zero();
    H.block<1, 3>(0, 0) = r_hat.transpose();
    H.block<3, 3>(1, 0) =
        (Eigen::Matrix3d::Identity() - r_hat * r_hat.transpose()) / rn;

    Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Zero();
    R(0, 0) = sigma_range * sigma_range;
    R.block<3, 3>(1, 1) =
        sigma_los * sigma_los * Eigen::Matrix3d::Identity();

    Vector6d dx;
    const double nis = joseph_update<6, 4>(dx, P_, z_res, H, R);
    x_ += dx;
    return nis;
}

void TranslationEkf::apply_control_delta_v(const Eigen::Vector3d& dv) {
    x_.tail<3>() += dv;
}

// ---------------------------------------------------------------------------
// AttitudeMekf
// ---------------------------------------------------------------------------

AttitudeMekf::AttitudeMekf(const Eigen::Quaterniond& q_own0,
                           const Eigen::Matrix3d& P_own0,
                           const Eigen::Quaterniond& q_rel0,
                           const Eigen::Vector3d& w_t0,
                           const Eigen::Matrix<double, 6, 6>& P_rel0,
                           const Eigen::Matrix3d& target_inertia)
    : q_own_(q_own0.normalized()),
      q_rel_(q_rel0.normalized()),
      w_t_(w_t0),
      P_own_(P_own0),
      P_rel_(P_rel0),
      I_t_(target_inertia),
      I_t_inv_(target_inertia.inverse()) {}

void AttitudeMekf::predict(const Eigen::Vector3d& w_g, double dt,
                           double gyro_sigma, double wt_accel_sigma) {
    // --- Reference propagation -------------------------------------------
    // Own attitude: gyro rotation over the step (exact exp map).
    q_own_ = (q_own_ * quat_exp(w_g * dt)).normalized();

    // Relative attitude: q_rel_dot = 1/2 q_rel (x) [0, w_e_hat],
    // w_e_hat = w_g - C(q_rel)^T w_t_hat  (WP2 kinematics on estimates).
    const Eigen::Vector3d w_t_body = q_rel_.conjugate() * w_t_;
    const Eigen::Vector3d w_e_hat = w_g - w_t_body;
    q_rel_ = (q_rel_ * quat_exp(w_e_hat * dt)).normalized();

    // Target rate: torque-free Euler with the KNOWN target inertia (RK4).
    const auto f = [this](const Eigen::Vector3d& w) -> Eigen::Vector3d {
        return I_t_inv_ * (-(w.cross(I_t_ * w)));
    };
    const Eigen::Vector3d k1 = f(w_t_);
    const Eigen::Vector3d k2 = f(w_t_ + 0.5 * dt * k1);
    const Eigen::Vector3d k3 = f(w_t_ + 0.5 * dt * k2);
    const Eigen::Vector3d k4 = f(w_t_ + dt * k3);
    w_t_ += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    // --- Covariance propagation ------------------------------------------
    // Own block: dtheta_dot = -[w_g]x dtheta - n_g.
    const Eigen::Matrix3d phi_own =
        Eigen::Matrix3d::Identity() - skew(w_g) * dt;
    const double q_ang = gyro_sigma * gyro_sigma * dt * dt;
    P_own_ = phi_own * P_own_ * phi_own.transpose() +
             q_ang * Eigen::Matrix3d::Identity();
    P_own_ = (0.5 * (P_own_ + P_own_.transpose())).eval();

    // Relative block: [dtheta_rel; dw_t] with
    //   dtheta_dot = -[w_g]x dtheta - C(q_rel)^T dw_t - n_g
    //   dw_t_dot   = A(w_t) dw_t + n_wt,  A = I_t^{-1}(-[w]x I_t + [I_t w]x).
    const Eigen::Matrix3d Ct = q_rel_.conjugate().toRotationMatrix();
    const Eigen::Matrix3d A =
        I_t_inv_ * (-skew(w_t_) * I_t_ + skew(I_t_ * w_t_));
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Zero();
    F.block<3, 3>(0, 0) = -skew(w_g);
    F.block<3, 3>(0, 3) = -Ct;
    F.block<3, 3>(3, 3) = A;
    const Eigen::Matrix<double, 6, 6> phi_rel =
        Eigen::Matrix<double, 6, 6>::Identity() + F * dt;

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q.block<3, 3>(0, 0) = q_ang * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(3, 3) = wt_accel_sigma * wt_accel_sigma * dt * dt *
                          Eigen::Matrix3d::Identity();
    P_rel_ = phi_rel * P_rel_ * phi_rel.transpose() + Q;
    P_rel_ = (0.5 * (P_rel_ + P_rel_.transpose())).eval();
}

double AttitudeMekf::update_star_tracker(const Eigen::Quaterniond& q_meas,
                                         double sigma) {
    const Eigen::Vector3d z_res = quat_residual(q_own_, q_meas);
    const Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R =
        sigma * sigma * Eigen::Matrix3d::Identity();

    Eigen::Vector3d dtheta;
    const double nis = joseph_update<3, 3>(dtheta, P_own_, z_res, H, R);
    q_own_ = (q_own_ * quat_exp(dtheta)).normalized();  // multiplicative inject
    return nis;
}

double AttitudeMekf::update_vision(const Eigen::Quaterniond& q_rel_meas,
                                   double sigma) {
    const Eigen::Vector3d z_res = quat_residual(q_rel_, q_rel_meas);
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R =
        sigma * sigma * Eigen::Matrix3d::Identity();

    Vector6d dx;
    const double nis = joseph_update<6, 3>(dx, P_rel_, z_res, H, R);
    q_rel_ = (q_rel_ * quat_exp(dx.head<3>())).normalized();  // multiplicative
    w_t_ += dx.tail<3>();
    return nis;
}

}  // namespace adsc
