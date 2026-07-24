#include "adsc/tether.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace adsc {

namespace {

// SPENVIS (ESA/BIRA-IASB), centred-dipole IGRF epoch-2000 values -- IDENTICAL
// to the file-local constants in src/decay.cpp (kB0, kDipoleRefRadius_m),
// which are not exported via decay.hpp. Re-declared here rather than piped
// through a header, the same established pattern tests/test_decay.cpp
// itself uses (its CHECK 9 for the EDT integrator cross-check).
constexpr double kB0 = 3.01153e-5;               // T
constexpr double kDipoleRefRadius_m = 6371200.0;  // m

// EnergySpike divergence-guard calibration constants (Deliverable 3,
// recalibrated per the wp16_xcheck.py adversarial review -- see the guard's
// comment in TetherSim::step() for the two false-positive mechanisms this
// avoids).
constexpr double kEnergySpikeFloorFrac   = 1e-9;  // e_floor = this * e_scale
constexpr double kEnergySpikeAbsGateFrac = 1e-6;  // absolute trip gate = this * e_scale
constexpr int    kEnergySpikeWarmupSteps = 100;   // steps before the EMA starts tracking

}  // namespace

// ---------------------------------------------------------------- orbit/field

double tether_orbit_radius_m(double altitude_km) {
    return kDipoleRefRadius_m + altitude_km * 1000.0;
}

double tether_mean_motion_rad_s(double altitude_km) {
    const double a = tether_orbit_radius_m(altitude_km);
    return std::sqrt(kEarthMu / (a * a * a));
}

double tether_dipole_beta_tesla(double altitude_km) {
    const double a = tether_orbit_radius_m(altitude_km);
    return kB0 * std::pow(kDipoleRefRadius_m / a, 3.0);
}

double field_Bn_tesla(double altitude_km, double inclination_deg) {
    const double beta = tether_dipole_beta_tesla(altitude_km);
    return beta * std::cos(inclination_deg * kPi / 180.0);
}

double field_Br_tesla(double altitude_km, double inclination_deg, double u_rad) {
    const double beta = tether_dipole_beta_tesla(altitude_km);
    const double inc = inclination_deg * kPi / 180.0;
    return -2.0 * beta * std::sin(inc) * std::sin(u_rad);
}

double field_Bt_tesla(double altitude_km, double inclination_deg, double u_rad) {
    const double beta = tether_dipole_beta_tesla(altitude_km);
    const double inc = inclination_deg * kPi / 180.0;
    return beta * std::sin(inc) * std::cos(u_rad);
}

double libration_eps(double current_a, double altitude_km, double inclination_deg,
                     double m_parent_kg, double m_tip_kg, double tether_length_m) {
    (void)tether_length_m;  // eps (t7-libration-study.md Eq 3.7) does not depend on L
    const double mu = (m_parent_kg * m_tip_kg) / (m_parent_kg + m_tip_kg);
    const double delta = std::fabs((m_parent_kg - m_tip_kg) / (m_parent_kg + m_tip_kg));
    const double n = tether_mean_motion_rad_s(altitude_km);
    const double bn = field_Bn_tesla(altitude_km, inclination_deg);
    return delta * std::fabs(current_a) * std::fabs(bn) / (6.0 * mu * n * n);
}

double current_for_eps(double eps, double altitude_km, double inclination_deg,
                       double m_parent_kg, double m_tip_kg, double tether_length_m) {
    (void)tether_length_m;
    const double mu = (m_parent_kg * m_tip_kg) / (m_parent_kg + m_tip_kg);
    const double delta = std::fabs((m_parent_kg - m_tip_kg) / (m_parent_kg + m_tip_kg));
    const double n = tether_mean_motion_rad_s(altitude_km);
    const double bn = field_Bn_tesla(altitude_km, inclination_deg);
    if (delta <= 0.0 || std::fabs(bn) <= 0.0) return 0.0;
    return eps * 6.0 * mu * n * n / (delta * std::fabs(bn));
}

const char* diverge_status_label(DivergeStatus s) {
    switch (s) {
        case DivergeStatus::Ok:               return "ok";
        case DivergeStatus::DivergedAngle:    return "diverged_angle";
        case DivergeStatus::DivergedVelocity: return "diverged_velocity";
        case DivergeStatus::EnergySpike:      return "energy_spike";
        case DivergeStatus::Overstrain:       return "overstrain";
    }
    return "ok";
}

// -------------------------------------------------------------- controllers

bool controller_c1_gate(double p_probe_w, double delta, bool previous_gate_on) {
    if (p_probe_w <= -delta) return true;   // Lorentz removing libration energy: thrust ON
    if (p_probe_w >= delta) return false;   // Lorentz pumping libration energy: thrust OFF
    return previous_gate_on;                // hysteresis band: hold
}

bool controller_c2_gate(double u_rad, double duty_on, double switch_phase) {
    const double twopi = 2.0 * kPi;
    double phase = std::fmod(u_rad / twopi + switch_phase, 1.0);
    if (phase < 0.0) phase += 1.0;
    return phase < duty_on;
}

// ---------------------------------------------------------------- internals

namespace {

std::vector<double> bead_masses(const TetherConfig& cfg) {
    const int n = cfg.n_beads;
    std::vector<double> m(static_cast<std::size_t>(n), 0.0);
    m.front() += cfg.m_parent_kg;
    m.back() += cfg.m_tip_kg;
    if (n >= 2 && cfg.lambda_tether_kg_per_m > 0.0) {
        const double L0 = cfg.tether_length_m / static_cast<double>(n - 1);
        const double m_seg = cfg.lambda_tether_kg_per_m * L0;
        for (int j = 0; j + 1 < n; ++j) {
            m[static_cast<std::size_t>(j)] += 0.5 * m_seg;
            m[static_cast<std::size_t>(j + 1)] += 0.5 * m_seg;
        }
    }
    return m;
}

// Straight-line IC at tilt theta0 from local vertical (and, [DT-v2: 3D],
// roll phi0 out of plane -- T7's own theta0=phi0 dumbbell seed convention,
// t7-libration-study.md Eq 1.1: u_t = cos(phi)cos(theta) e_r +
// cos(phi)sin(theta) e_t + sin(phi) e_n), spaced L0 apart, CoM-shifted to
// the Hill-frame origin (or, if servicer_fixed, re-shifted so bead 0 sits
// exactly at the origin instead -- Deliverable 1's two endpoint-boundary
// modes). Zero relative velocity (Deliverable 4: "rates = 0").
//
// Guard (BINDING for planar bit-identity): phi0 is forced to 0.0 whenever
// out_of_plane == false, regardless of cfg.phi0_deg -- so
// cos(phi0)==1.0 and sin(phi0)==0.0 EXACTLY (IEEE cos(0)/sin(0) are exact),
// making e == (cos theta0, sin theta0, 0) bit-for-bit identical to the
// Phase-1 Vector2d formula for every existing (out_of_plane==false) caller.
BeadState initial_state(const TetherConfig& cfg, const std::vector<double>& masses) {
    const int n = cfg.n_beads;
    const double L0 = cfg.tether_length_m / static_cast<double>(n - 1);
    const double theta0 = cfg.theta0_deg * kPi / 180.0;
    const double phi0 = cfg.out_of_plane ? (cfg.phi0_deg * kPi / 180.0) : 0.0;
    const double cph0 = std::cos(phi0);
    const Eigen::Vector3d e(std::cos(theta0) * cph0, std::sin(theta0) * cph0, std::sin(phi0));

    BeadState s;
    s.pos.resize(static_cast<std::size_t>(n));
    s.vel.assign(static_cast<std::size_t>(n), Eigen::Vector3d::Zero());
    for (int i = 0; i < n; ++i) {
        s.pos[static_cast<std::size_t>(i)] = static_cast<double>(i) * L0 * e;
    }

    double total_m = 0.0;
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    for (int i = 0; i < n; ++i) {
        const std::size_t si = static_cast<std::size_t>(i);
        total_m += masses[si];
        com += masses[si] * s.pos[si];
    }
    com /= total_m;
    for (auto& p : s.pos) p -= com;

    if (cfg.servicer_fixed) {
        const Eigen::Vector3d shift = s.pos.front();
        for (auto& p : s.pos) p -= shift;
    }
    return s;
}

// [DT-v2: 3D] full 3D Hill-frame field vector at argument of latitude
// u = n*t. Guard (BINDING for planar bit-identity): when out_of_plane ==
// false this ALWAYS returns (0, 0, Bn) regardless of what
// field_Br_tesla/field_Bt_tesla would evaluate to -- B_x = B_y are forced
// to 0, matching Phase-1 exactly (the old code path never read anything
// but Bn). Only when out_of_plane == true are the orbital-frequency
// (B_r, B_t) cross-track components consumed (D2/D9 of the WP16 Phase-2
// design record).
Eigen::Vector3d field_vector_hill(const TetherConfig& cfg, double Bn, double u_rad) {
    if (!cfg.out_of_plane) return Eigen::Vector3d(0.0, 0.0, Bn);
    const double Br = field_Br_tesla(cfg.altitude_km, cfg.inclination_deg, u_rad);
    const double Bt = field_Bt_tesla(cfg.altitude_km, cfg.inclination_deg, u_rad);
    return Eigen::Vector3d(Br, Bt, Bn);
}

struct Forces {
    std::vector<Eigen::Vector3d> accel;
    double p_lorentz_w = 0.0;
    double p_damp_w = 0.0;
    double max_strain_ratio = 0.0;
    int n_slack = 0;
    double root_tension_n = 0.0;
};

// Accelerations + force-side diagnostics at state s under a SIGNED current
// I_a [A], uniform along the whole tether (Phase-1 series-circuit
// simplification -- a tapered OML current profile is out of scope, folded
// into the tip-mass/geometry uncertainty per t7-libration-study.md Sec 9).
//
// [DT-v2: 3D] Bfield is the full 3D Hill-frame field vector (B_r, B_t, B_n)
// at the CURRENT evaluation instant (see field_vector_hill above / TetherSim
// ::step below) -- already gated to (0, 0, Bn) by the caller whenever
// out_of_plane == false, so every formula below stays a pure drop-in
// Vector2d -> Vector3d generalization with no separate planar/3D branch
// inside this function itself (house pattern: single code path, R1
// no-fork). The ONLY branch in this function is the z-frame-force ADD
// below, which is skipped entirely (not computed-then-zeroed) in planar
// mode, so it can never perturb z even at the ULP level.
Forces compute_forces(const TetherConfig& cfg, const std::vector<double>& masses,
                     double n, double L0, double k_seg, const Eigen::Vector3d& Bfield,
                     const BeadState& s, double I_a) {
    const int nb = cfg.n_beads;
    Forces f;
    f.accel.assign(static_cast<std::size_t>(nb), Eigen::Vector3d::Zero());

    for (int j = 0; j + 1 < nb; ++j) {
        const std::size_t sj = static_cast<std::size_t>(j);
        const std::size_t sj1 = static_cast<std::size_t>(j + 1);
        const Eigen::Vector3d d = s.pos[sj1] - s.pos[sj];
        const double l = d.norm();
        const Eigen::Vector3d e =
            (l > 1e-9) ? Eigen::Vector3d(d / l) : Eigen::Vector3d(1.0, 0.0, 0.0);
        const double ldot = (s.vel[sj1] - s.vel[sj]).dot(e);

        const double stretch = l - L0;
        const double spring_term = k_seg * stretch + cfg.damping_c_Ns_per_m * ldot;
        const double tension = std::max(0.0, spring_term);
        if (j == 0) f.root_tension_n = tension;

        const Eigen::Vector3d tension_force = tension * e;  // on bead j (+e), bead j+1 (-e)
        f.accel[sj] += tension_force / masses[sj];
        f.accel[sj1] -= tension_force / masses[sj1];

        if (tension > 0.0) {
            f.p_damp_w -= cfg.damping_c_Ns_per_m * ldot * ldot;
        } else {
            ++f.n_slack;
        }
        const double ratio = l / L0;
        if (ratio > f.max_strain_ratio) f.max_strain_ratio = ratio;

        // Per-segment Lorentz force (Deliverable 1 / [DT-v2: 3D] D2): dF =
        // I*l*(e x B). Planar limit check (out_of_plane==false, so Bfield =
        // (0,0,Bn) exactly): e x B = (e.y()*Bn - 0*0, 0*0 - e.x()*Bn,
        // e.x()*0 - e.y()*0) = (Bn*e.y(), -Bn*e.x(), 0), bit-identical to
        // the old Phase-1 Eigen::Vector2d(Bn*e.y(), -Bn*e.x()) formula
        // (IEEE754 multiplication is exactly commutative and x*0.0==0.0/
        // -0.0 for finite x, so every extra term this cross product adds
        // over the old 2D formula is an exact zero). [DT-v2: 3D] Nonzero
        // Bfield.x()/y() (out_of_plane==true) add the genuine out-of-plane
        // roll-forcing term dF_z = I*l*(e.x()*B.y() - e.y()*B.x()) -- the
        // Pelaez energy-pumping drive (D2).
        const Eigen::Vector3d dF = I_a * l * e.cross(Bfield);
        const Eigen::Vector3d half = 0.5 * dF;
        f.accel[sj] += half / masses[sj];
        f.accel[sj1] += half / masses[sj1];
        f.p_lorentz_w += half.dot(s.vel[sj]) + half.dot(s.vel[sj1]);
    }

    // Gravity-gradient + Coriolis (Hill/CW linearization, Deliverable 2;
    // matches relmotion.hpp's CwModel sign convention), external to every
    // bead individually (not an internal action-reaction pair). The x/y
    // formulas below are BYTE-IDENTICAL to Phase 1 (never touched).
    // [DT-v2: 3D] the cross-track (z) restoring term a_z = -n^2*z (D1: the
    // frame carries no Coriolis/centrifugal term in z, since Omega is
    // parallel to z_hat) is only ever ADDED under the out_of_plane branch
    // below -- in planar mode this term is never computed at all (not
    // computed-then-discarded), so z-acceleration can never pick up even an
    // ULP-level nonzero value from this loop.
    for (int i = 0; i < nb; ++i) {
        const std::size_t si = static_cast<std::size_t>(i);
        const double x = s.pos[si].x();
        const double vx = s.vel[si].x();
        const double vy = s.vel[si].y();
        f.accel[si].x() += 3.0 * n * n * x + 2.0 * n * vy;
        f.accel[si].y() += -2.0 * n * vx;
        if (cfg.out_of_plane) {
            const double z = s.pos[si].z();
            f.accel[si].z() += -n * n * z;
        }
    }

    if (cfg.servicer_fixed) f.accel.front() = Eigen::Vector3d::Zero();
    return f;
}

// [DT-v2: 3D] U_gg gains the +0.5*n^2*z^2 cross-track potential term (D6;
// gradient gives the -n^2*z restoring force above), added via its OWN
// separate += statement (not folded into the existing x-term expression)
// -- z==0.0 EXACTLY in planar mode, so this term's contribution is 0.0
// EXACTLY (0.5*masses[si]*n*n*0.0*0.0 == 0.0), and u_gg_running + 0.0 ==
// u_gg_running bit-for-bit (IEEE754), so compute_energy_jacobi is
// bit-identical to Phase 1 whenever out_of_plane == false: the original
// x-term line below is untouched, byte for byte.
double compute_energy_jacobi(const TetherConfig& cfg, const std::vector<double>& masses,
                             double n, double L0, double k_seg, const BeadState& s) {
    double ke = 0.0, u_gg = 0.0, u_spring = 0.0;
    for (int i = 0; i < cfg.n_beads; ++i) {
        const std::size_t si = static_cast<std::size_t>(i);
        ke += 0.5 * masses[si] * s.vel[si].squaredNorm();
        u_gg += -1.5 * masses[si] * n * n * s.pos[si].x() * s.pos[si].x();  // byte-identical to Phase 1
        u_gg += 0.5 * masses[si] * n * n * s.pos[si].z() * s.pos[si].z();   // [DT-v2: 3D] adds exactly 0.0 when z==0
    }
    for (int j = 0; j + 1 < cfg.n_beads; ++j) {
        const double l = (s.pos[static_cast<std::size_t>(j + 1)] -
                         s.pos[static_cast<std::size_t>(j)]).norm();
        const double stretch = l - L0;
        if (stretch > 0.0) u_spring += 0.5 * k_seg * stretch * stretch;
    }
    return ke + u_gg + u_spring;
}

BeadState add_scaled(const BeadState& base, const BeadState& deriv, double h) {
    BeadState out;
    const std::size_t n = base.pos.size();
    out.pos.resize(n);
    out.vel.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        out.pos[i] = base.pos[i] + h * deriv.pos[i];
        out.vel[i] = base.vel[i] + h * deriv.vel[i];
    }
    return out;
}

}  // namespace

// ---------------------------------------------------------------- TetherSim

TetherSim::TetherSim(const TetherConfig& cfg)
    : cfg_(cfg), masses_(bead_masses(cfg)), state_(initial_state(cfg, masses_)) {
    n_ = tether_mean_motion_rad_s(cfg_.altitude_km);
    L0_m_ = cfg_.tether_length_m / static_cast<double>(cfg_.n_beads - 1);
    k_seg_n_per_m_ = cfg_.EA_design_N / L0_m_;
    Bn_ = field_Bn_tesla(cfg_.altitude_km, cfg_.inclination_deg);

    // Physical energy scale (Deliverable 3 orbit-drift audit AND the
    // EnergySpike guard below), computed ONCE here from the config:
    // e_scale = mu_total * L_total^2 * n_orbit^2.
    const double mu_total =
        (cfg_.m_parent_kg * cfg_.m_tip_kg) / (cfg_.m_parent_kg + cfg_.m_tip_kg);
    e_scale_ = mu_total * cfg_.tether_length_m * cfg_.tether_length_m * n_ * n_;
    e_floor_j_ = kEnergySpikeFloorFrac * e_scale_;
    residual_ema_ = e_floor_j_;  // seeded at the floor, not 0 (EnergySpike guard)

    const double e0 = compute_energy_jacobi(cfg_, masses_, n_, L0_m_, k_seg_n_per_m_, state_);
    last_diag_.energy_jacobi = e0;
    energy_prev_j_ = e0;
    energy_at_last_mark_j_ = e0;
}

double TetherSim::chord_angle_rad() const {
    const Eigen::Vector3d d = state_.pos.back() - state_.pos.front();
    return std::atan2(d.y(), d.x());
}

double TetherSim::cone_angle_rad() const {
    const Eigen::Vector3d d = state_.pos.back() - state_.pos.front();
    const double norm = d.norm();
    const double cx = (norm > 1e-12) ? std::clamp(d.x() / norm, -1.0, 1.0) : 1.0;
    return std::acos(cx);
}

double TetherSim::roll_angle_rad() const {
    const Eigen::Vector3d d = state_.pos.back() - state_.pos.front();
    const double norm = d.norm();
    const double cz = (norm > 1e-12) ? std::clamp(d.z() / norm, -1.0, 1.0) : 0.0;
    return std::asin(cz);
}

double TetherSim::root_tension_n() const { return last_diag_.root_tension_n; }

double TetherSim::u_rad() const { return n_ * t_s_; }

double TetherSim::probe_unit_current_power_w() const {
    const Eigen::Vector3d Bfield = field_vector_hill(cfg_, Bn_, u_rad());
    const Forces f = compute_forces(cfg_, masses_, n_, L0_m_, k_seg_n_per_m_, Bfield, state_, 1.0);
    return f.p_lorentz_w;
}

StepDiagnostics TetherSim::step(const double* current_override_a) {
    double I_a = 0.0;
    if (current_override_a) {
        I_a = *current_override_a;
    } else {
        switch (cfg_.controller) {
            case ControllerMode::Constant:
                I_a = cfg_.const_current_a;
                break;
            case ControllerMode::FixedDuty: {
                const bool on = controller_c2_gate(u_rad(), cfg_.duty_on, cfg_.switch_phase);
                I_a = on ? cfg_.eta_I * cfg_.I_cap_A : 0.0;
                break;
            }
            case ControllerMode::PhaseGated: {
                const double p_probe = probe_unit_current_power_w();
                gate_on_ = controller_c1_gate(p_probe, cfg_.gate_hysteresis, gate_on_);
                I_a = gate_on_ ? cfg_.eta_I * cfg_.I_cap_A : 0.0;
                break;
            }
        }
    }

    const double dt = cfg_.dt_s;
    const BeadState s0 = state_;

    // [DT-v2: 3D] the field is evaluated at each RK4 sub-stage's OWN time
    // (t, t+dt/2, t+dt/2, t+dt) via field_vector_hill/u_rad, since the
    // cross-track (B_r, B_t) components vary at the orbital frequency
    // (out_of_plane==true only; field_vector_hill returns the constant
    // (0,0,Bn) at every stage otherwise, matching Phase-1 exactly).
    auto deriv = [&](const BeadState& s, double t_local, Forces* fout) -> BeadState {
        const Eigen::Vector3d Bfield = field_vector_hill(cfg_, Bn_, n_ * t_local);
        Forces f = compute_forces(cfg_, masses_, n_, L0_m_, k_seg_n_per_m_, Bfield, s, I_a);
        if (fout) *fout = f;
        BeadState d;
        d.pos = s.vel;
        d.vel = f.accel;
        return d;
    };

    Forces f_start;
    const BeadState k1 = deriv(s0, t_s_, &f_start);
    const BeadState s2 = add_scaled(s0, k1, 0.5 * dt);
    const BeadState k2 = deriv(s2, t_s_ + 0.5 * dt, nullptr);
    const BeadState s3 = add_scaled(s0, k2, 0.5 * dt);
    const BeadState k3 = deriv(s3, t_s_ + 0.5 * dt, nullptr);
    const BeadState s4 = add_scaled(s0, k3, dt);
    const BeadState k4 = deriv(s4, t_s_ + dt, nullptr);

    const int nb = cfg_.n_beads;
    BeadState next;
    next.pos.resize(static_cast<std::size_t>(nb));
    next.vel.resize(static_cast<std::size_t>(nb));
    for (int i = 0; i < nb; ++i) {
        const std::size_t si = static_cast<std::size_t>(i);
        next.pos[si] = s0.pos[si] + (dt / 6.0) * (k1.pos[si] + 2.0 * k2.pos[si] +
                                                  2.0 * k3.pos[si] + k4.pos[si]);
        next.vel[si] = s0.vel[si] + (dt / 6.0) * (k1.vel[si] + 2.0 * k2.vel[si] +
                                                  2.0 * k3.vel[si] + k4.vel[si]);
    }
    if (cfg_.servicer_fixed) {
        next.pos.front() = s0.pos.front();
        next.vel.front() = Eigen::Vector3d::Zero();
    }

    state_ = next;
    t_s_ += dt;

    const Eigen::Vector3d Bfield_end = field_vector_hill(cfg_, Bn_, u_rad());
    const Forces f_end =
        compute_forces(cfg_, masses_, n_, L0_m_, k_seg_n_per_m_, Bfield_end, state_, I_a);
    const double e_new = compute_energy_jacobi(cfg_, masses_, n_, L0_m_, k_seg_n_per_m_, state_);

    double vmax = 0.0;
    for (const auto& v : state_.vel) vmax = std::max(vmax, v.norm());

    StepDiagnostics d;
    d.t_s = t_s_;
    d.chord_angle_deg = chord_angle_rad() * 180.0 / kPi;
    d.cone_angle_deg = cone_angle_rad() * 180.0 / kPi;  // [DT-v2: 3D]
    d.roll_angle_deg = roll_angle_rad() * 180.0 / kPi;  // [DT-v2: 3D]
    d.energy_jacobi = e_new;
    // Trapezoidal (start/end-of-step average) accounting for the energy-rate
    // terms: I_a is held constant over the step (house style), but the
    // position/velocity-dependent terms genuinely vary within the step.
    d.p_lorentz_w = 0.5 * (f_start.p_lorentz_w + f_end.p_lorentz_w);
    d.p_damp_w = 0.5 * (f_start.p_damp_w + f_end.p_damp_w);
    d.max_strain_ratio = f_end.max_strain_ratio;
    d.n_slack = f_end.n_slack;
    d.n_segments = nb - 1;
    d.max_speed_m_s = vmax;
    d.current_applied_a = I_a;
    d.root_tension_n = f_end.root_tension_n;
    last_diag_ = d;

    // --- energy-drift-per-orbit audit (Deliverable 3) ---
    integrated_pump_since_mark_j_ += (d.p_lorentz_w + d.p_damp_w) * dt;
    const double t_orbit = 2.0 * kPi / n_;
    const int orbit_now = static_cast<int>(t_s_ / t_orbit);
    if (orbit_now != last_orbit_marked_) {
        last_orbit_drift_ =
            std::fabs(e_new - energy_at_last_mark_j_ - integrated_pump_since_mark_j_) /
            std::max(1e-30, e_scale_);
        energy_at_last_mark_j_ = e_new;
        integrated_pump_since_mark_j_ = 0.0;
        last_orbit_marked_ = orbit_now;
    }

    // --- divergence guards (Deliverable 3): first trip wins, terminal ---
    const double step_residual =
        std::fabs(e_new - energy_prev_j_ - (d.p_lorentz_w + d.p_damp_w) * dt);
    ++step_count_;
    // EnergySpike guard, recalibrated per the wp16_xcheck.py adversarial
    // review (finding #1) to avoid the TWO false-positive mechanisms it
    // flagged in the original (bare "> 10x the running EMA") formulation:
    //   1. FP round-off on well-conserved runs: an undamped/uncontrolled
    //      config (e.g. test_tether.cpp CHECK 1) drives the EMA down toward
    //      pure machine-precision energy-bookkeeping noise, after which even
    //      an ordinary, tiny residual is "> 10x" that near-zero baseline.
    //      Fix: the EMA is floored at e_floor_j_ = 1e-9*e_scale_ (never seeded
    //      at/decaying to 0) via max(residual_ema_, e_floor_j_), AND the trip
    //      also requires an ABSOLUTE gate step_residual > 1e-6*e_scale_ tied
    //      to the run's own physical energy scale -- a relative-only test can
    //      never again fire on pure round-off.
    //   2. Benign sub-mm slack-kink crossings: the straight-line,
    //      zero-relative-velocity Deliverable-4 IC is not any segment's
    //      quasi-static tension equilibrium, so an un-/lightly-damped run's
    //      FIRST tension/slack transition (a first-derivative kink in the
    //      tension-only cable law, typically a sub-millimetre stretch
    //      excursion) produces one legitimately large but PHYSICALLY BENIGN
    //      residual spike near t=0 -- confirmed by wp16_xcheck.py to
    //      otherwise false-trip CHECK 1 and both CHECK 3 sub-cases within
    //      <0.05 orbit, before the physics of interest ever develops. Fix:
    //      the EMA only starts tracking step_residual after a
    //      kEnergySpikeWarmupSteps=100-step warmup (held at e_floor_j_ until
    //      then), so this one-time early transient cannot itself corrupt the
    //      baseline the guard compares later, genuine blow-ups against.
    // A true integrator blow-up (e.g. eps=3.0, CHECK 4) clears both the
    // relative and the absolute gate by orders of magnitude and still trips
    // exactly as before; DivergedAngle/DivergedVelocity/Overstrain are
    // unchanged.
    // [DT-v2: 3D] the DivergedAngle guard generalizes from the planar
    // |chord_angle_deg| (theta-only) test to the 3D solid-angle
    // cone_angle_deg test (D6/D9) ONLY when out_of_plane == true -- the
    // planar branch below is BYTE-IDENTICAL to Phase 1 (same condition,
    // same field), so this branch cannot change a single existing
    // (out_of_plane==false) committed result. cone_angle_deg is T7's own
    // divergence-event definition (Sec 5.4: arccos(cos theta cos phi) >
    // 80 deg), which is what the D9 dumbbell-limit acceptance test targets.
    const bool angle_diverged = cfg_.out_of_plane ? (d.cone_angle_deg > 80.0)
                                                  : (std::fabs(d.chord_angle_deg) > 80.0);
    if (status_ == DivergeStatus::Ok) {
        if (angle_diverged) {
            status_ = DivergeStatus::DivergedAngle;
        } else if (d.max_speed_m_s > 10.0 * n_ * cfg_.tether_length_m) {
            status_ = DivergeStatus::DivergedVelocity;
        } else if (d.max_strain_ratio > 1.5) {
            status_ = DivergeStatus::Overstrain;
        } else if (step_residual > 10.0 * std::max(residual_ema_, e_floor_j_) &&
                  step_residual > kEnergySpikeAbsGateFrac * e_scale_) {
            status_ = DivergeStatus::EnergySpike;
        }
    }
    if (step_count_ > kEnergySpikeWarmupSteps) {
        residual_ema_ = 0.98 * residual_ema_ + 0.02 * step_residual;
    }
    energy_prev_j_ = e_new;

    return d;
}

// ----------------------------------------------------------------- run loop

SimResult run_tether_sim(const TetherConfig& cfg) {
    TetherSim sim(cfg);
    const double t_orbit = 2.0 * kPi / sim.mean_motion_rad_s();
    const double t_end = cfg.sim_orbits * t_orbit;

    SimResult r;
    r.o45_orbit = std::numeric_limits<double>::infinity();
    r.divergence_orbit = std::numeric_limits<double>::infinity();
    r.o45_cone_orbit = std::numeric_limits<double>::infinity();  // [DT-v2: 3D]

    bool o45_found = false;
    bool o45_cone_found = false;  // [DT-v2: 3D]
    double sum_abs_current_a = 0.0;
    long n_slack_total = 0;

    while (sim.time_s() < t_end) {
        const StepDiagnostics d = sim.step();
        ++r.n_steps;

        // max_chord_angle_deg / o45_orbit: BYTE-IDENTICAL Phase-1 definition
        // (fabs(chord_angle_deg)), computed unconditionally in EVERY mode --
        // never branches on out_of_plane, so these two fields cannot change
        // a single existing committed planar result.
        const double abs_angle = std::fabs(d.chord_angle_deg);
        if (abs_angle > r.max_chord_angle_deg) r.max_chord_angle_deg = abs_angle;
        if (!o45_found && abs_angle >= 45.0) {
            r.o45_orbit = sim.time_s() / t_orbit;
            o45_found = true;
        }
        // [DT-v2: 3D] parallel solid-angle/roll tracking, additive fields
        // only (D6/D10): cone_angle_deg is T7's OWN o45/divergence
        // definition (Sec 5.4), tracked here regardless of mode (cheap; ==
        // abs_angle numerically, though not bit-identically, in planar mode).
        if (d.cone_angle_deg > r.max_cone_angle_deg) r.max_cone_angle_deg = d.cone_angle_deg;
        const double abs_roll = std::fabs(d.roll_angle_deg);
        if (abs_roll > r.max_roll_deg) r.max_roll_deg = abs_roll;
        if (!o45_cone_found && d.cone_angle_deg >= 45.0) {
            r.o45_cone_orbit = sim.time_s() / t_orbit;
            o45_cone_found = true;
        }
        sum_abs_current_a += std::fabs(d.current_applied_a);
        n_slack_total += d.n_slack;

        if (sim.status() != DivergeStatus::Ok) {
            r.status = sim.status();
            r.divergence_orbit = sim.time_s() / t_orbit;
            break;
        }
    }

    r.stop_time_s = sim.time_s();
    r.final_chord_angle_deg = sim.chord_angle_rad() * 180.0 / kPi;
    r.energy_drift_per_orbit = sim.last_orbit_energy_drift();
    const long n_segments = cfg.n_beads - 1;
    r.slack_fraction = (r.n_steps > 0 && n_segments > 0)
        ? static_cast<double>(n_slack_total) / static_cast<double>(r.n_steps * n_segments)
        : 0.0;
    r.eta_lib_effective = (r.n_steps > 0 && cfg.I_cap_A > 0.0)
        ? sum_abs_current_a / (static_cast<double>(r.n_steps) * cfg.I_cap_A)
        : 0.0;
    r.retained_drag_frac = r.eta_lib_effective;
    return r;
}

}  // namespace adsc
