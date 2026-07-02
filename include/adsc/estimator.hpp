#pragma once

#include <cstdint>
#include <random>

#include <Eigen/Dense>

#include "adsc/relmotion.hpp"

namespace adsc {

// ============================================================================
// Estimator + sensor models (WP4)
// ----------------------------------------------------------------------------
// Two decoupled filters, per the spec's recommended split. The decoupling
// assumptions are stated here once:
//
//  (1) TranslationEkf — linear Kalman filter on the 6-state LVLH relative
//      translation [r; v]. The prediction matrix is the ANALYTIC CW state
//      transition from relmotion (no new discretization). Measurements at the
//      rangefinder rate: range |r| plus the line-of-sight UNIT VECTOR r/|r|.
//      Unit-vector LOS is used instead of az/el angles deliberately: the
//      az/el Jacobian is singular at boresight, while d(r/|r|)/dr =
//      (I - r_hat r_hat^T)/|r| is well-defined everywhere.
//      Decoupling assumption: relative translation is independent of the
//      attitude states (true in this simulation: CW dynamics do not depend on
//      attitude, and the LOS sensor is modeled in the LVLH frame, i.e.
//      attitude-compensated upstream).
//
//  (2) AttitudeMekf — MULTIPLICATIVE EKF blocks for attitude. The error state
//      is a 3-component small-angle vector per attitude; covariance is carried
//      as 3x3 (own) and 6x6 (relative + target rate). An additive quaternion
//      EKF is deliberately NOT used (normalization decay is its classic
//      failure mode); the reference quaternion is always corrected by the
//      multiplicative injection q_hat <- q_hat (x) [1, dtheta/2] and the error
//      state is reset to zero.
//        Block A (own attitude, 3-state dtheta_own): gyro-propagated reference
//        with star-tracker updates.
//        Block B (relative attitude + target rate, 6-state [dtheta_rel;
//        dw_t]): propagated with the gyro and the torque-free target model
//        (target inertia assumed KNOWN — documented limitation, a real
//        mission needs inertia identification); vision pose updates at low
//        rate measure the relative quaternion directly.
//      Decoupling assumption: blocks A and B share the same gyro noise, so
//      their errors are weakly correlated through the process noise; no
//      measurement couples them, each block's marginal filter is consistent
//      on its own, and the NEES watchdog test verifies that empirically.
//
// Error dynamics used (derived for q mapping body->inertial, q_dot =
// 1/2 q (x) [0, w], right-multiplicative body-frame error q_true =
// q_hat (x) [1, dtheta/2]):
//   own:  dtheta_dot = -[w_g]x dtheta - n_g
//   rel:  dtheta_dot = -[w_g]x dtheta - C(q_rel_hat)^T dw_t - n_g
//         dw_t_dot   = A(w_t_hat) dw_t + n_wt,
//         A(w) = I_t^{-1} ( -[w]x I_t + [I_t w]x )
// (the [w_g]x in the relative block is the sum of the relative-rate and
// rotated-target-rate transport terms: w_e_hat + C^T w_t_hat = w_g).
//
// Covariance updates use the JOSEPH form plus explicit symmetrization each
// update; the tests check symmetry and positive-definiteness of every P over
// a full run.
//
// Determinism (R6): all randomness comes from ONE std::mt19937 with a fixed
// seed, converted to Gaussians by an explicit Box-Muller transform. The
// std::normal_distribution algorithm is implementation-defined (results would
// differ across standard libraries), so it is not used; the mt19937 output
// sequence itself IS pinned by the C++ standard, making every run bit-stable
// across platforms.
// ============================================================================

// Deterministic Gaussian source: std::mt19937 raw draws (the engine's output
// sequence is pinned by the C++ standard) + explicit Box-Muller.
class GaussianSource {
public:
    explicit GaussianSource(uint32_t seed) : gen_(seed) {}

    double sample();                       // ~ N(0,1)
    Eigen::Vector3d sample3(double sigma); // ~ N(0, sigma^2 I3)

private:
    double uniform01();  // in (0,1], from raw mt19937 draws

    std::mt19937 gen_;
    bool   have_spare_ = false;
    double spare_      = 0.0;
};

// The one and only bundle the control path is allowed to consume (WP4
// truth-isolation): mission code hands the controller THIS struct, never the
// simulator's truth state. Truth is read only by the sensor models and the
// error-statistics recorder.
struct EstimatedState {
    Eigen::Quaterniond q_own    = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    w_own    = Eigen::Vector3d::Zero();  // gyro-based rate
    Eigen::Quaterniond q_target = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    w_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d    w_target_dot = Eigen::Vector3d::Zero();
    Vector6d           rel_translation = Vector6d::Zero();  // [r; v] LVLH
};

// Linear Kalman filter on the LVLH relative translation (see file header).
class TranslationEkf {
public:
    TranslationEkf(const CwModel& cw, const Vector6d& x0,
                   const Eigen::Matrix<double, 6, 6>& P0);

    // Coast prediction over dt using the analytic CW STM; q_vel_sigma is the
    // per-step white velocity perturbation the truth model also carries.
    void predict(double dt, double q_vel_sigma);

    // Range + LOS unit-vector update. Returns the NIS (innovation^T S^{-1}
    // innovation, 4 dof).
    double update(double range_meas, const Eigen::Vector3d& los_meas,
                  double sigma_range, double sigma_los);

    const Vector6d& state() const { return x_; }
    const Eigen::Matrix<double, 6, 6>& covariance() const { return P_; }

private:
    CwModel cw_;
    Vector6d x_;
    Eigen::Matrix<double, 6, 6> P_;
};

// Multiplicative attitude EKF blocks (see file header).
class AttitudeMekf {
public:
    AttitudeMekf(const Eigen::Quaterniond& q_own0, const Eigen::Matrix3d& P_own0,
                 const Eigen::Quaterniond& q_rel0, const Eigen::Vector3d& w_t0,
                 const Eigen::Matrix<double, 6, 6>& P_rel0,
                 const Eigen::Matrix3d& target_inertia);

    // Propagate both blocks one control step with the gyro measurement w_g.
    // gyro_sigma is the per-step white rate noise; wt_accel_sigma the small
    // random angular acceleration the truth target also carries.
    void predict(const Eigen::Vector3d& w_g, double dt, double gyro_sigma,
                 double wt_accel_sigma);

    // Star-tracker update of the own-attitude block. Returns NIS (3 dof).
    double update_star_tracker(const Eigen::Quaterniond& q_meas, double sigma);

    // Vision relative-pose update of the relative block. Returns NIS (3 dof).
    double update_vision(const Eigen::Quaterniond& q_rel_meas, double sigma);

    const Eigen::Quaterniond& q_own() const { return q_own_; }
    const Eigen::Quaterniond& q_rel() const { return q_rel_; }
    const Eigen::Vector3d&    w_target() const { return w_t_; }
    const Eigen::Matrix3d&    P_own() const { return P_own_; }
    const Eigen::Matrix<double, 6, 6>& P_rel() const { return P_rel_; }

private:
    Eigen::Quaterniond q_own_;
    Eigen::Quaterniond q_rel_;   // servicer-body -> target-body (WP2 q_e)
    Eigen::Vector3d    w_t_;     // target rate, target body frame
    Eigen::Matrix3d    P_own_;
    Eigen::Matrix<double, 6, 6> P_rel_;
    Eigen::Matrix3d    I_t_, I_t_inv_;
};

// Small-angle quaternion exp map: v -> [cos(|v|/2), sin(|v|/2) v_hat].
Eigen::Quaterniond quat_exp(const Eigen::Vector3d& v);

// Small-angle residual 2*vec(q_ref^{-1} (x) q), with the sign fixed so the
// scalar part is positive (short-way residual).
Eigen::Vector3d quat_residual(const Eigen::Quaterniond& q_ref,
                              const Eigen::Quaterniond& q);

}  // namespace adsc
