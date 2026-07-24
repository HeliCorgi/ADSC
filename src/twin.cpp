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
// [DT-v2: 3D] VirtualTwinEkf3D (Deliverable D8)
// ---------------------------------------------------------------------------

namespace {

// Nondimensional electrodynamic torques Q_theta, Q_phi (t7-libration-study.md
// Sec 3.3-3.4), evaluated at the given (theta, phi, u, I_eff). Exactly
// linear in I_eff (pref below is linear in it), so the Jacobian entries
// F(1,4)/F(3,4) reuse this same helper called with I_eff=1.0 (per-unit-
// current probe, the same pattern tether.cpp's probe_unit_current_power_w()
// uses) rather than dividing by a possibly-zero I_eff.
void nondim_torques(double delta, double mu, double n, double beta, double inc_rad,
                    double th, double ph, double u_rad, double i_eff,
                    double* q_theta, double* q_phi) {
    const double Br = -2.0 * beta * std::sin(inc_rad) * std::sin(u_rad);
    const double Bt = beta * std::sin(inc_rad) * std::cos(u_rad);
    const double Bn = beta * std::cos(inc_rad);
    const double cth = std::cos(th), sth = std::sin(th);
    const double cph = std::cos(ph), sph = std::sin(ph);
    const double ax = cth * cph, ay = sth * cph, az = sph;
    const double udotB = ax * Br + ay * Bt + az * Bn;
    const double pref = delta * i_eff / (2.0 * mu * n * n);
    const double tvx = pref * (ax * udotB - Br);
    const double tvy = pref * (ay * udotB - Bt);
    const double tvz = pref * (az * udotB - Bn);
    *q_theta = tvz;
    // Roll generalized force = projection of tv onto the roll rotation axis
    // (sth, -cth, 0): q_phi = tvx*sth - tvy*cth. (Previously sign-inverted;
    // fixed post adversarial review -- see tests/test_twin.cpp's
    // direction-of-forcing check, which is specifically designed to catch a
    // regression of this sign.)
    *q_phi = tvx * sth - tvy * cth;
}

}  // namespace

VirtualTwinEkf3D::VirtualTwinEkf3D(const VirtualTwinConfig3D& cfg, const EkfState3D& x0)
    : cfg_(cfg), st_(x0) {
    n_ = tether_mean_motion_rad_s(cfg_.altitude_km);
    mu_ = (cfg_.m_parent_kg * cfg_.m_tip_kg) / (cfg_.m_parent_kg + cfg_.m_tip_kg);
    delta_ = (cfg_.m_parent_kg - cfg_.m_tip_kg) / (cfg_.m_parent_kg + cfg_.m_tip_kg);
    beta_ = tether_dipole_beta_tesla(cfg_.altitude_km);
    inc_rad_ = cfg_.inclination_deg * kPi / 180.0;
}

void VirtualTwinEkf3D::predict(double dt_s, double u_rad) {
    const Eigen::Matrix<double, 6, 1> x0 = st_.x;

    // Continuous nonlinear dynamics f(x, u) -- physical-time form of
    // t7-libration-study.md Eqs 2.2-2.3 (x n^2), full 3D field. gamma
    // (tunable pitch damping) enters ONLY the pitch channel, same as the
    // 4-state model (no analogous roll-damping term in the design record).
    auto f = [this](const Eigen::Matrix<double, 6, 1>& x, double u) {
        Eigen::Matrix<double, 6, 1> dx;
        const double th = x(0), thd = x(1), ph = x(2), phd = x(3), i_eff = x(4), c_hat = x(5);
        const double gamma = c_hat / (2.0 * mu_);
        double q_theta = 0.0, q_phi = 0.0;
        nondim_torques(delta_, mu_, n_, beta_, inc_rad_, th, ph, u, i_eff, &q_theta, &q_phi);
        const double cph = std::cos(ph), sph = std::sin(ph);
        const double cph2 = std::max(cph * cph, 1e-6);  // guard: never divide by ~0 near a 90-deg roll (out of physical scope anyway)
        const double cth = std::cos(th), sth = std::sin(th);
        dx(0) = thd;
        dx(1) = (n_ * n_ * q_theta + 2.0 * phd * (thd + n_) * sph * cph -
                 3.0 * n_ * n_ * sth * cth * cph * cph) / cph2 -
                2.0 * gamma * thd;
        dx(2) = phd;
        dx(3) = n_ * n_ * q_phi - (thd + n_) * (thd + n_) * sph * cph -
                3.0 * n_ * n_ * cth * cth * sph * cph;
        dx(4) = 0.0;
        dx(5) = 0.0;
        return dx;
    };

    // Continuous Jacobian F at the PRE-step state x0 and u_rad (start of
    // step) -- see twin.hpp's class comment for the linearization caveat
    // (the cos^2(phi) divisor in the pitch equation is held fixed at its
    // pre-step value rather than fully differentiated). F(1,4)/F(3,4) (the
    // Lorentz->pitch/roll pumping entries) use the per-unit-current probe
    // (nondim_torques with i_eff=1.0), avoiding any division by a possibly-
    // zero I_eff estimate.
    const double th0 = x0(0), thd0 = x0(1), ph0 = x0(2), c_hat0 = x0(5);
    const double gamma0 = c_hat0 / (2.0 * mu_);
    const double cph0 = std::cos(ph0), sph0 = std::sin(ph0);
    const double cph0_2 = std::max(cph0 * cph0, 1e-6);
    const double cth0 = std::cos(th0);
    double q_theta_unit = 0.0, q_phi_unit = 0.0;
    nondim_torques(delta_, mu_, n_, beta_, inc_rad_, th0, ph0, u_rad, 1.0,
                  &q_theta_unit, &q_phi_unit);

    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Zero();
    F(0, 1) = 1.0;
    F(1, 0) = -3.0 * n_ * n_ * std::cos(2.0 * th0);
    F(1, 1) = -2.0 * gamma0;
    // Pitch<-roll-rate coupling (partial of the 2*phidot*(thetadot+n)*sinphi
    // *cosphi term in Eq 2.2 w.r.t. phidot). Derived directly from
    // t7-libration-study.md Eq 2.2's own "-2(theta'+1)phi'sinphi cosphi"
    // term (moved to the RHS, sign flips) -- vanishes at ph0=0 either way,
    // to 2nd order, matching the design record's stated limit.
    F(1, 3) = 2.0 * (thd0 + n_) * sph0 * cph0 / cph0_2;
    F(1, 4) = n_ * n_ * q_theta_unit / cph0_2;
    F(1, 5) = -thd0 / mu_;
    F(2, 3) = 1.0;
    F(3, 2) = -((thd0 + n_) * (thd0 + n_) + 3.0 * n_ * n_ * cth0 * cth0) * std::cos(2.0 * ph0);
    F(3, 4) = n_ * n_ * q_phi_unit;

    const Eigen::Matrix<double, 6, 6> Fdt = F * dt_s;
    const Eigen::Matrix<double, 6, 6> Phi =
        Eigen::Matrix<double, 6, 6>::Identity() + Fdt + 0.5 * Fdt * Fdt;

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q(0, 0) = cfg_.q_theta * dt_s;
    Q(1, 1) = cfg_.q_thetadot * dt_s;
    Q(2, 2) = cfg_.q_phi * dt_s;
    Q(3, 3) = cfg_.q_phidot * dt_s;
    Q(4, 4) = cfg_.q_I_eff * dt_s;
    Q(5, 5) = cfg_.q_c_hat * dt_s;

    st_.P = Phi * st_.P * Phi.transpose() + Q;
    st_.P = (0.5 * (st_.P + st_.P.transpose())).eval();

    // Nonlinear RK4 mean propagation, field re-evaluated at each sub-stage's
    // own argument of latitude (u_rad advances at n_ within the step, same
    // discipline as tether.cpp's TetherSim::step()).
    const Eigen::Matrix<double, 6, 1> k1 = f(x0, u_rad);
    const Eigen::Matrix<double, 6, 1> k2 = f(x0 + 0.5 * dt_s * k1, u_rad + n_ * 0.5 * dt_s);
    const Eigen::Matrix<double, 6, 1> k3 = f(x0 + 0.5 * dt_s * k2, u_rad + n_ * 0.5 * dt_s);
    const Eigen::Matrix<double, 6, 1> k4 = f(x0 + dt_s * k3, u_rad + n_ * dt_s);
    st_.x = x0 + (dt_s / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

double VirtualTwinEkf3D::update_scalar_angle(double z_ang_rad, double sigma_ang_rad) {
    const double th = st_.x(0), ph = st_.x(2);
    const double cth = std::cos(th), sth = std::sin(th);
    const double cph = std::cos(ph), sph = std::sin(ph);
    const double cos_ang = std::clamp(cth * cph, -1.0, 1.0);
    const double h1 = std::acos(cos_ang);
    const double sin_ang = std::sqrt(std::max(0.0, 1.0 - cos_ang * cos_ang));
    const double denom = std::max(sin_ang, 1e-9);  // guard: d/dx acos(x) singular at x=+-1 (th=ph=0)

    Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
    H(0, 0) = (sth * cph) / denom;
    H(0, 2) = (cth * sph) / denom;

    const double z_res = z_ang_rad - h1;
    const double R = sigma_ang_rad * sigma_ang_rad;
    const double S = (H * st_.P * H.transpose())(0, 0) + R;
    const Eigen::Matrix<double, 6, 1> K = (st_.P * H.transpose()) / S;

    st_.x += K * z_res;
    const Eigen::Matrix<double, 6, 6> IKH =
        Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    st_.P = IKH * st_.P * IKH.transpose() + (K * R) * K.transpose();
    st_.P = (0.5 * (st_.P + st_.P.transpose())).eval();

    return z_res * z_res / S;
}

double VirtualTwinEkf3D::update_chord2axis(double z_y, double z_z, double z_tension_n,
                                           double sigma_y, double sigma_z,
                                           double sigma_tension_n) {
    const double th = st_.x(0), thd = st_.x(1), ph = st_.x(2), phd = st_.x(3);
    const double L = cfg_.tether_length_m;
    const double cth = std::cos(th), sth = std::sin(th);
    const double cph = std::cos(ph), sph = std::sin(ph);

    const double h_y = sth * cph;
    const double h_z = sph;
    // NOTE (honest-comment, no math change): the centrifugal contribution
    // here is the bare thd^2 + phd^2 (angular-rate-squared) sum. A rotating
    // dumbbell's tension is more consistently expressed with an
    // (thd + n)^2-based term (i.e. rate relative to inertial space, matching
    // the (thd + n) combination used elsewhere in this file's dynamics,
    // e.g. dx(3) above and F(3,2)) -- this h_t is a simplification, not a
    // rigorously derived centrifugal tension. It is a filter-internal
    // measurement model only: it is not wired into any reported evidence
    // artifact (update_chord2axis is not called from run_twin_sync or any
    // WP16 evidence path as of this commit).
    const double h_t = mu_ * L * (n_ * n_ * (1.0 + 3.0 * cth * cth * cph * cph) + thd * thd + phd * phd);

    Eigen::Matrix<double, 3, 1> z_res;
    z_res(0) = z_y - h_y;
    z_res(1) = z_z - h_z;
    z_res(2) = z_tension_n - h_t;

    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = cth * cph;
    H(0, 2) = -sth * sph;
    H(1, 2) = cph;
    H(2, 0) = -6.0 * mu_ * L * n_ * n_ * cth * sth * cph * cph;
    H(2, 1) = 2.0 * mu_ * L * thd;
    H(2, 2) = -6.0 * mu_ * L * n_ * n_ * cth * cth * cph * sph;
    H(2, 3) = 2.0 * mu_ * L * phd;

    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0, 0) = sigma_y * sigma_y;
    R(1, 1) = sigma_z * sigma_z;
    R(2, 2) = sigma_tension_n * sigma_tension_n;

    const Eigen::Matrix3d S = H * st_.P * H.transpose() + R;
    const Eigen::Matrix3d S_inv = S.inverse();
    const Eigen::Matrix<double, 6, 3> K = st_.P * H.transpose() * S_inv;

    st_.x += K * z_res;
    const Eigen::Matrix<double, 6, 6> IKH =
        Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    st_.P = IKH * st_.P * IKH.transpose() + K * R * K.transpose();
    st_.P = (0.5 * (st_.P + st_.P.transpose())).eval();

    return z_res.dot(S_inv * z_res);
}

// ---------------------------------------------------------------------------
// Twin-to-twin sync loop
// ---------------------------------------------------------------------------

TwinSyncReport run_twin_sync(const TruthTwinConfig& truth_cfg, const VirtualTwinConfig& virt_cfg,
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

    // Weak-observability / filter-honesty diagnostics (Deliverable 6, see the
    // TwinSyncReport finding note in twin.hpp): track the MINIMUM c_hat
    // variance (P(3,3)) reached over the run -- it must not collapse to
    // spurious certainty -- and confirm the covariance stays symmetric AND
    // positive-definite at EVERY assimilation step (Joseph form should
    // guarantee this; we verify rather than assume).
    double min_c_hat_variance = x0.P(3, 3);
    bool cov_spd_all_steps = true;

    TwinSyncReport rep;
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

        // Post-update covariance diagnostics (weak-observability finding).
        const Eigen::Matrix4d& P_now = ekf.state().P;
        min_c_hat_variance = std::min(min_c_hat_variance, P_now(3, 3));
        const double sym_err = (P_now - P_now.transpose()).cwiseAbs().maxCoeff();
        const Eigen::LLT<Eigen::Matrix4d> llt(P_now);
        if (sym_err >= 1e-8 || llt.info() != Eigen::Success) cov_spd_all_steps = false;

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

    rep.final_I_eff = ekf.state().x(2);
    rep.final_c_hat = ekf.state().x(3);
    rep.min_c_hat_variance = min_c_hat_variance;
    rep.cov_spd_all_steps = cov_spd_all_steps;
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
