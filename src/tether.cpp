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

// Straight-line IC at tilt theta0 from local vertical, spaced L0 apart,
// CoM-shifted to the Hill-frame origin (or, if servicer_fixed, re-shifted
// so bead 0 sits exactly at the origin instead -- Deliverable 1's two
// endpoint-boundary modes). Zero relative velocity (Deliverable 4: "rates
// = 0").
BeadState initial_state(const TetherConfig& cfg, const std::vector<double>& masses) {
    const int n = cfg.n_beads;
    const double L0 = cfg.tether_length_m / static_cast<double>(n - 1);
    const double theta0 = cfg.theta0_deg * kPi / 180.0;
    const Eigen::Vector2d e(std::cos(theta0), std::sin(theta0));

    BeadState s;
    s.pos.resize(static_cast<std::size_t>(n));
    s.vel.assign(static_cast<std::size_t>(n), Eigen::Vector2d::Zero());
    for (int i = 0; i < n; ++i) {
        s.pos[static_cast<std::size_t>(i)] = static_cast<double>(i) * L0 * e;
    }

    double total_m = 0.0;
    Eigen::Vector2d com = Eigen::Vector2d::Zero();
    for (int i = 0; i < n; ++i) {
        const std::size_t si = static_cast<std::size_t>(i);
        total_m += masses[si];
        com += masses[si] * s.pos[si];
    }
    com /= total_m;
    for (auto& p : s.pos) p -= com;

    if (cfg.servicer_fixed) {
        const Eigen::Vector2d shift = s.pos.front();
        for (auto& p : s.pos) p -= shift;
    }
    return s;
}

struct Forces {
    std::vector<Eigen::Vector2d> accel;
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
Forces compute_forces(const TetherConfig& cfg, const std::vector<double>& masses,
                     double n, double L0, double k_seg, double Bn,
                     const BeadState& s, double I_a) {
    const int nb = cfg.n_beads;
    Forces f;
    f.accel.assign(static_cast<std::size_t>(nb), Eigen::Vector2d::Zero());

    for (int j = 0; j + 1 < nb; ++j) {
        const std::size_t sj = static_cast<std::size_t>(j);
        const std::size_t sj1 = static_cast<std::size_t>(j + 1);
        const Eigen::Vector2d d = s.pos[sj1] - s.pos[sj];
        const double l = d.norm();
        const Eigen::Vector2d e = (l > 1e-9) ? Eigen::Vector2d(d / l) : Eigen::Vector2d(1.0, 0.0);
        const double ldot = (s.vel[sj1] - s.vel[sj]).dot(e);

        const double stretch = l - L0;
        const double spring_term = k_seg * stretch + cfg.damping_c_Ns_per_m * ldot;
        const double tension = std::max(0.0, spring_term);
        if (j == 0) f.root_tension_n = tension;

        const Eigen::Vector2d tension_force = tension * e;  // on bead j (+e), bead j+1 (-e)
        f.accel[sj] += tension_force / masses[sj];
        f.accel[sj1] -= tension_force / masses[sj1];

        if (tension > 0.0) {
            f.p_damp_w -= cfg.damping_c_Ns_per_m * ldot * ldot;
        } else {
            ++f.n_slack;
        }
        const double ratio = l / L0;
        if (ratio > f.max_strain_ratio) f.max_strain_ratio = ratio;

        // Per-segment in-plane (DC channel) Lorentz force (Deliverable 1):
        // dF = I*l*(e x B), B = (0,0,Bn) in Phase 1 (planar), so
        // e x B = (Bn*e.y(), -Bn*e.x(), 0); split half to each endpoint.
        const Eigen::Vector2d dF = I_a * l * Eigen::Vector2d(Bn * e.y(), -Bn * e.x());
        const Eigen::Vector2d half = 0.5 * dF;
        f.accel[sj] += half / masses[sj];
        f.accel[sj1] += half / masses[sj1];
        f.p_lorentz_w += half.dot(s.vel[sj]) + half.dot(s.vel[sj1]);
    }

    // Gravity-gradient + Coriolis (Hill/CW linearization, Deliverable 2;
    // matches relmotion.hpp's CwModel sign convention), external to every
    // bead individually (not an internal action-reaction pair).
    for (int i = 0; i < nb; ++i) {
        const std::size_t si = static_cast<std::size_t>(i);
        const double x = s.pos[si].x();
        const double vx = s.vel[si].x();
        const double vy = s.vel[si].y();
        f.accel[si].x() += 3.0 * n * n * x + 2.0 * n * vy;
        f.accel[si].y() += -2.0 * n * vx;
    }

    if (cfg.servicer_fixed) f.accel.front() = Eigen::Vector2d::Zero();
    return f;
}

double compute_energy_jacobi(const TetherConfig& cfg, const std::vector<double>& masses,
                             double n, double L0, double k_seg, const BeadState& s) {
    double ke = 0.0, u_gg = 0.0, u_spring = 0.0;
    for (int i = 0; i < cfg.n_beads; ++i) {
        const std::size_t si = static_cast<std::size_t>(i);
        ke += 0.5 * masses[si] * s.vel[si].squaredNorm();
        u_gg += -1.5 * masses[si] * n * n * s.pos[si].x() * s.pos[si].x();  // z=0 (planar)
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
    const Eigen::Vector2d d = state_.pos.back() - state_.pos.front();
    return std::atan2(d.y(), d.x());
}

double TetherSim::root_tension_n() const { return last_diag_.root_tension_n; }

double TetherSim::u_rad() const { return n_ * t_s_; }

double TetherSim::probe_unit_current_power_w() const {
    const Forces f = compute_forces(cfg_, masses_, n_, L0_m_, k_seg_n_per_m_, Bn_, state_, 1.0);
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

    auto deriv = [&](const BeadState& s, Forces* fout) -> BeadState {
        Forces f = compute_forces(cfg_, masses_, n_, L0_m_, k_seg_n_per_m_, Bn_, s, I_a);
        if (fout) *fout = f;
        BeadState d;
        d.pos = s.vel;
        d.vel = f.accel;
        return d;
    };

    Forces f_start;
    const BeadState k1 = deriv(s0, &f_start);
    const BeadState s2 = add_scaled(s0, k1, 0.5 * dt);
    const BeadState k2 = deriv(s2, nullptr);
    const BeadState s3 = add_scaled(s0, k2, 0.5 * dt);
    const BeadState k3 = deriv(s3, nullptr);
    const BeadState s4 = add_scaled(s0, k3, dt);
    const BeadState k4 = deriv(s4, nullptr);

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
        next.vel.front() = Eigen::Vector2d::Zero();
    }

    state_ = next;
    t_s_ += dt;

    const Forces f_end =
        compute_forces(cfg_, masses_, n_, L0_m_, k_seg_n_per_m_, Bn_, state_, I_a);
    const double e_new = compute_energy_jacobi(cfg_, masses_, n_, L0_m_, k_seg_n_per_m_, state_);

    double vmax = 0.0;
    for (const auto& v : state_.vel) vmax = std::max(vmax, v.norm());

    StepDiagnostics d;
    d.t_s = t_s_;
    d.chord_angle_deg = chord_angle_rad() * 180.0 / kPi;
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
    if (status_ == DivergeStatus::Ok) {
        if (std::fabs(d.chord_angle_deg) > 80.0) {
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

    bool o45_found = false;
    double sum_abs_current_a = 0.0;
    long n_slack_total = 0;

    while (sim.time_s() < t_end) {
        const StepDiagnostics d = sim.step();
        ++r.n_steps;

        const double abs_angle = std::fabs(d.chord_angle_deg);
        if (abs_angle > r.max_chord_angle_deg) r.max_chord_angle_deg = abs_angle;
        if (!o45_found && abs_angle >= 45.0) {
            r.o45_orbit = sim.time_s() / t_orbit;
            o45_found = true;
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
