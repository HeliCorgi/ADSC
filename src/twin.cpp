#include "adsc/twin.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "adsc/estimator.hpp"  // GaussianSource (house-canonical RNG, per this WP's recon note)

namespace adsc {

// ---------------------------------------------------------------------------
// VirtualTwinEkf
// ---------------------------------------------------------------------------

VirtualTwinEkf::VirtualTwinEkf(const VirtualTwinConfig& cfg, const EkfState& x0)
    : cfg_(cfg), st_(x0) {
    n_ = tether_mean_motion_rad_s(cfg_.altitude_km);
    mu_ = (cfg_.m_parent_kg * cfg_.m_tip_kg) / (cfg_.m_parent_kg + cfg_.m_tip_kg);
    const double delta = (cfg_.m_parent_kg - cfg_.m_tip_kg) / (cfg_.m_parent_kg + cfg_.m_tip_kg);
    const double bn = field_Bn_tesla(cfg_.altitude_km, cfg_.inclination_deg);
    a_L_ = -delta * bn / (2.0 * mu_);
}

void VirtualTwinEkf::predict(double dt_s) {
    const Eigen::Vector4d x0 = st_.x;

    // Continuous nonlinear dynamics f(x) (Deliverable 6). gamma(c_hat) =
    // c_hat/(2*mu) is a TUNABLE ASSUMED pitch-damping rate the EKF estimates
    // ONLINE -- it is NOT a faithful reduction of the truth-twin's
    // per-segment LONGITUDINAL (axial) dashpot force c*ldot*e (tether.cpp):
    // that force is purely radial (along e) by construction, so it has ZERO
    // tangential component, and in the rigid-rotation configuration this
    // reduced model represents (ldot == 0 identically), it contributes
    // EXACTLY ZERO direct pitch (theta_ddot) damping (confirmed both
    // algebraically and against an exact bead-model evaluation,
    // wp16_xcheck.py adversarial review, finding #2). gamma is instead a
    // free process parameter the filter fits from the (theta, tension)
    // measurements; its mismatch against the true axial-damping physics is
    // exactly what the q_c_hat process-noise term (VirtualTwinConfig,
    // twin.hpp) is sized to absorb, not a derived physical quantity.
    auto f = [this](const Eigen::Vector4d& x) {
        Eigen::Vector4d dx;
        const double gamma = x(3) / (2.0 * mu_);
        dx(0) = x(1);
        dx(1) = -3.0 * n_ * n_ * std::sin(x(0)) * std::cos(x(0)) + a_L_ * x(2) -
                2.0 * gamma * x(1);
        dx(2) = 0.0;
        dx(3) = 0.0;
        return dx;
    };

    // Continuous Jacobian F at the PRE-step state x0 (Deliverable 6).
    const double gamma0 = x0(3) / (2.0 * mu_);
    const double dgamma_dc = 1.0 / (2.0 * mu_);
    Eigen::Matrix4d F = Eigen::Matrix4d::Zero();
    F(0, 1) = 1.0;
    F(1, 0) = -3.0 * n_ * n_ * std::cos(2.0 * x0(0));
    F(1, 1) = -2.0 * gamma0;
    F(1, 2) = a_L_;
    F(1, 3) = -2.0 * x0(1) * dgamma_dc;

    const Eigen::Matrix4d Fdt = F * dt_s;
    const Eigen::Matrix4d Phi = Eigen::Matrix4d::Identity() + Fdt + 0.5 * Fdt * Fdt;

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0, 0) = cfg_.q_theta * dt_s;
    Q(1, 1) = cfg_.q_thetadot * dt_s;
    Q(2, 2) = cfg_.q_I_eff * dt_s;
    Q(3, 3) = cfg_.q_c_hat * dt_s;

    st_.P = Phi * st_.P * Phi.transpose() + Q;
    st_.P = (0.5 * (st_.P + st_.P.transpose())).eval();

    // Nonlinear RK4 mean propagation, same fixed dt (house style).
    const Eigen::Vector4d k1 = f(x0);
    const Eigen::Vector4d k2 = f(x0 + 0.5 * dt_s * k1);
    const Eigen::Vector4d k3 = f(x0 + 0.5 * dt_s * k2);
    const Eigen::Vector4d k4 = f(x0 + dt_s * k3);
    st_.x = x0 + (dt_s / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

double VirtualTwinEkf::update(double z_theta_rad, double z_tension_n,
                              double sigma_theta_rad, double sigma_tension_n) {
    const double th = st_.x(0);
    const double thd = st_.x(1);
    const double L = cfg_.tether_length_m;

    const double h1 = th;
    const double h2 = mu_ * L * (n_ * n_ * (1.0 + 3.0 * std::cos(th) * std::cos(th)) + thd * thd);

    Eigen::Vector2d z_res;
    z_res(0) = z_theta_rad - h1;
    z_res(1) = z_tension_n - h2;

    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 0) = -3.0 * mu_ * L * n_ * n_ * std::sin(2.0 * th);
    H(1, 1) = 2.0 * mu_ * L * thd;

    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0, 0) = sigma_theta_rad * sigma_theta_rad;
    R(1, 1) = sigma_tension_n * sigma_tension_n;

    const Eigen::Matrix2d S = H * st_.P * H.transpose() + R;
    const Eigen::Matrix2d S_inv = S.inverse();
    const Eigen::Matrix<double, 4, 2> K = st_.P * H.transpose() * S_inv;

    st_.x += K * z_res;

    const Eigen::Matrix4d IKH = Eigen::Matrix4d::Identity() - K * H;
    st_.P = IKH * st_.P * IKH.transpose() + K * R * K.transpose();
    st_.P = (0.5 * (st_.P + st_.P.transpose())).eval();

    return z_res.dot(S_inv * z_res);
}

// ---------------------------------------------------------------------------
// Twin-to-twin sync loop
// ---------------------------------------------------------------------------

SyncReport run_twin_sync(const TruthTwinConfig& truth_cfg, const VirtualTwinConfig& virt_cfg,
                         ControllerMode controller, double sim_orbits, uint64_t seed) {
    TetherConfig truth_tcfg = truth_cfg.truth_tether;
    // The truth twin's OWN internal controller decision is never consulted
    // (see file header, "virtual-to-real pushback"): every step below
    // passes an EXTERNAL current_override_a computed from the virtual
    // twin's estimated state (or, for the Constant baseline, from
    // truth_tcfg.const_current_a directly -- there is no feedback law to
    // evaluate in that case).
    TetherSim truth(truth_tcfg);

    const double n = truth.mean_motion_rad_s();
    const double t_orbit = 2.0 * kPi / n;
    const double t_end = sim_orbits * t_orbit;

    // PLACEHOLDER initial parameter guesses: the virtual twin starts from a
    // generic mid-range prior, NOT the truth twin's (perturbed, unknown to
    // it) actual EA/damping/eta_I.
    const double i_eff_guess0 = 1.0;   // [A]
    const double c_hat_guess0 = 0.05;  // [N.s/m]

    EkfState x0;
    x0.x = Eigen::Vector4d(truth_tcfg.theta0_deg * kPi / 180.0, 0.0,
                          i_eff_guess0, c_hat_guess0);
    x0.P = Eigen::Matrix4d::Zero();
    x0.P(0, 0) = (2.0 * kPi / 180.0) * (2.0 * kPi / 180.0);  // PLACEHOLDER initial angle uncertainty
    x0.P(1, 1) = 1.0e-4 * 1.0e-4;                            // PLACEHOLDER initial rate uncertainty
    x0.P(2, 2) = 1.0 * 1.0;                                  // PLACEHOLDER initial I_eff uncertainty [A]
    x0.P(3, 3) = 0.1 * 0.1;                                  // PLACEHOLDER initial c_hat uncertainty [N.s/m]
    VirtualTwinEkf ekf(virt_cfg, x0);

    // SplitMix64-derived seed truncated to the 32 bits GaussianSource's
    // mt19937 ctor takes; the input is already well-mixed (splitmix64_seed
    // / campaign.cpp idiom, applied by the caller), so the low 32 bits are
    // still well distributed (R6).
    GaussianSource noise(static_cast<uint32_t>(seed));

    bool virtual_gate_on = true;
    std::vector<double> nis_hist;
    double theta_err_sq_sum = 0.0;
    long n_theta_samples = 0;

    SyncReport rep;
    rep.n_orbits = static_cast<int>(sim_orbits);
    int consecutive_ok_orbits = 0;
    int last_orbit_marked = -1;

    const double i_true = truth_cfg.truth_tether.eta_I * truth_cfg.truth_tether.I_cap_A;
    const double c_true = truth_cfg.truth_tether.damping_c_Ns_per_m;

    while (truth.time_s() < t_end) {
        double commanded_a = 0.0;
        if (controller == ControllerMode::FixedDuty) {
            const bool on = controller_c2_gate(truth.u_rad(), truth_tcfg.duty_on,
                                              truth_tcfg.switch_phase);
            commanded_a = on ? ekf.state().x(2) : 0.0;  // apply the ESTIMATED I_eff, never the unknown truth current
        } else if (controller == ControllerMode::PhaseGated) {
            const double p_probe = ekf.a_L() * ekf.state().x(1);  // reduced-model analogue of the unit-current Lorentz-power probe (tether.hpp's probe_unit_current_power_w)
            virtual_gate_on =
                controller_c1_gate(p_probe, truth_tcfg.gate_hysteresis, virtual_gate_on);
            commanded_a = virtual_gate_on ? ekf.state().x(2) : 0.0;
        } else {
            commanded_a = truth_tcfg.const_current_a;  // Constant baseline: passive, no feedback loop
        }

        truth.step(&commanded_a);
        ekf.predict(truth_tcfg.dt_s);

        const double sigma_theta_rad = truth_cfg.sigma_theta_deg * kPi / 180.0;
        const double z_theta = truth.chord_angle_rad() + noise.sample() * sigma_theta_rad;
        const double z_tension =
            truth.root_tension_n() + noise.sample() * truth_cfg.sigma_tension_n;
        const double nis =
            ekf.update(z_theta, z_tension, sigma_theta_rad, truth_cfg.sigma_tension_n);
        nis_hist.push_back(nis);

        const double theta_err = ekf.state().x(0) - truth.chord_angle_rad();
        theta_err_sq_sum += theta_err * theta_err;
        ++n_theta_samples;

        const int orbit_now = static_cast<int>(truth.time_s() / t_orbit);
        if (orbit_now != last_orbit_marked) {
            last_orbit_marked = orbit_now;
            const double i_err = std::fabs(ekf.state().x(2) - i_true) / std::max(1e-9, std::fabs(i_true));
            const double c_err = std::fabs(ekf.state().x(3) - c_true) / std::max(1e-9, std::fabs(c_true));
            const bool nis_ok = (nis >= 0.05 && nis <= 7.38);
            const bool params_ok = (i_err < 0.10 && c_err < 0.10);
            if (params_ok && nis_ok) {
                ++consecutive_ok_orbits;
                if (consecutive_ok_orbits >= 5 && !rep.converged) {
                    rep.converged = true;
                    rep.converged_at_orbit = orbit_now;
                }
            } else {
                consecutive_ok_orbits = 0;
            }
        }

        if (truth.status() != DivergeStatus::Ok) break;  // truth diverged: sync report is meaningless past this point
    }

    rep.final_I_eff_rel_err = std::fabs(ekf.state().x(2) - i_true) / std::max(1e-9, std::fabs(i_true));
    rep.final_c_hat_rel_err = std::fabs(ekf.state().x(3) - c_true) / std::max(1e-9, std::fabs(c_true));
    rep.theta_rmse_deg =
        (n_theta_samples > 0)
            ? std::sqrt(theta_err_sq_sum / static_cast<double>(n_theta_samples)) * 180.0 / kPi
            : 0.0;
    if (!nis_hist.empty()) {
        std::vector<double> sorted = nis_hist;
        std::sort(sorted.begin(), sorted.end());
        const std::size_t mid = sorted.size() / 2;
        rep.median_nis = (sorted.size() % 2 == 1)
            ? sorted[mid]
            : 0.5 * (sorted[mid - 1] + sorted[mid]);
    }
    return rep;
}

}  // namespace adsc
