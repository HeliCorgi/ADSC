#pragma once

#include <vector>

#include <Eigen/Dense>

#include "adsc/relmotion.hpp"  // kEarthMu, kPi (NOT kEarthRadius -- see the radius-convention note below)

namespace adsc {

// ============================================================================
// WP16 Digital Twin Phase 1 -- lumped-mass EDT tether dynamics
// ----------------------------------------------------------------------------
// [DT-v1: lumped-mass tether, aligned dipole, twin-to-twin]. NO real asset
// exists anywhere in this file or in twin.hpp/twin.cpp: the "truth" state
// used by the twin-to-twin sync (twin.hpp) is itself a perturbed-parameter
// instance of the SAME simulated model implemented here. T7 (EDT libration
// dynamic-stability trade, _tasks_local/t7-libration-study.md) stays OPEN;
// nothing in this file resolves it. The two controllers below (Deliverable
// 5) are in-model PROPOSALS, evaluated only against this simulated physics
// -- never a claim about a resolved or operational system.
//
// Scope (Phase 1, PLANAR only): N point-mass beads on a tension-only
// spring-dashpot tether, in the linearized Hill/LVLH rotating frame (radial
// x, along-track y -- the SAME sign convention as relmotion.hpp's CwModel),
// under gravity-gradient + Coriolis + a per-segment Lorentz force driven
// ONLY by the orbit-normal field component B_n. B_n is CONSTANT around an
// aligned-dipole orbit (wp13-edt-derivation.md Sec 2.3) -- the DC / in-plane
// channel. The cross-track field components (B_r, B_t), which drive the
// out-of-plane roll libration and the real Pelaez energy-pumping
// instability (t7-libration-study.md Sec 3.5), are a Phase-2 extension and
// are NOT integrated here: this planar Phase-1 model reproduces only the
// BOUNDED-libration / DC-tumble behaviour of T7 Table 5.1 (Deliverable 4,
// target T4a), never the faster out-of-plane pumping onset (T4b, an
// explicit Phase-2 gate) -- a stated, pre-registered model boundary, not an
// oversight.
//
// Orbit-radius convention: altitude is added to kDipoleRefRadius_m (the
// SPENVIS mean-Earth reference radius the dipole formula is defined
// against), NOT the WGS-84 kEarthRadius used elsewhere in this repo
// (decay.cpp). This deliberately matches _tasks_local/t7_libration_sim.py's
// orbit_params() convention exactly, so the Deliverable-4 dumbbell-limit
// validation reproduces T7's own reference numbers, not just their order of
// magnitude.
//
// Physical constants reused verbatim from src/decay.cpp (WP13 EDT-v1, R15
// pin): kB0 = 3.01153e-5 T, kDipoleRefRadius_m = 6371200.0 m. Those two are
// file-local (anonymous-namespace) constants in decay.cpp, not exported via
// decay.hpp, so tether.cpp re-declares them identically -- the same
// established pattern tests/test_decay.cpp itself uses (its CHECK 9).
//
// Determinism (R6): this file has NO randomness of its own. Dispersions and
// sensor noise live in twin.cpp / main_twin.cpp (SplitMix64 + the
// header-exposed adsc::GaussianSource, per the campaign.cpp / estimator.hpp
// house conventions).
// ============================================================================

// ---------------------------------------------------------------- orbit/field

// Reference orbit radius a = kDipoleRefRadius_m + altitude_km*1000 (see the
// convention note above -- deliberately NOT kEarthRadius).
double tether_orbit_radius_m(double altitude_km);

// Mean motion n = sqrt(kEarthMu / a^3) [rad/s].
double tether_mean_motion_rad_s(double altitude_km);

// Aligned-dipole field magnitude beta(a) = kB0*(kDipoleRefRadius_m/a)^3 [T].
double tether_dipole_beta_tesla(double altitude_km);

// Orbit-normal field component B_n = beta*cos(i) [T] -- CONSTANT in the
// argument of latitude u for an aligned dipole (the Phase-1 DC channel).
double field_Bn_tesla(double altitude_km, double inclination_deg);

// In-plane (cross-track-forcing) field components, exposed only for
// Phase-2 / twin.cpp diagnostic notes -- NOT consumed by the Phase-1
// planar EOM below (wp13-edt-derivation.md Sec 2.3):
//   B_r = -2*beta*sin(i)*sin(u), B_t = beta*sin(i)*cos(u).
double field_Br_tesla(double altitude_km, double inclination_deg, double u_rad);
double field_Bt_tesla(double altitude_km, double inclination_deg, double u_rad);

// T7-equivalent nondimensional ED/GG torque ratio
//   eps = |Delta| * |current_a| * |B_n| / (6 * mu * n^2)
// (t7-libration-study.md Eq 3.7). NOT used inside the bead integrator
// (which only ever sees a current in amps) -- this is a cross-check /
// test-setup helper so the Deliverable-4 dumbbell-limit validation can pick
// a bead-model current that reproduces a given T7 eps scenario.
double libration_eps(double current_a, double altitude_km, double inclination_deg,
                     double m_parent_kg, double m_tip_kg, double tether_length_m);

// Inverse of libration_eps: the (unsigned) current [A] that reproduces a
// given eps at this geometry. Test-setup helper only (see above).
double current_for_eps(double eps, double altitude_km, double inclination_deg,
                       double m_parent_kg, double m_tip_kg, double tether_length_m);

// ------------------------------------------------------------------- config

enum class ControllerMode {
    Constant,    // fixed signed current the whole run (validation / uncontrolled baseline)
    PhaseGated,  // Deliverable 5 C1: bang-bang on sign(Lorentz power into libration)
    FixedDuty,   // Deliverable 5 C2: fixed duty_on fraction of each orbit at switch_phase
};

enum class DivergeStatus {
    Ok,
    DivergedAngle,     // chord angle-from-vertical > 80 deg (T7 "diverged" event)
    DivergedVelocity,  // any |v_i| > 10*n*L
    EnergySpike,       // per-step energy residual clears BOTH a relative gate (>10x the running
                       // EMA, itself floored at e_floor=1e-9*e_scale and warmed up for 100 steps
                       // before it starts tracking) AND an absolute gate (>1e-6*e_scale), where
                       // e_scale = mu_total*L_total^2*n_orbit^2 (integrator blow-up only -- see
                       // tether.cpp's guard comment for the two false-positive mechanisms this
                       // recalibration, per the wp16_xcheck.py adversarial review, now avoids)
    Overstrain,        // any segment l_j/L0 > 1.5
};
const char* diverge_status_label(DivergeStatus s);

// All PLACEHOLDER values are marked; none is a physically validated figure
// (WP16 Phase 1, no real asset -- see file header). Precondition: n_beads
// >= 2.
struct TetherConfig {
    // --- orbit (reused WP13 catalog-A constants, decay.cpp/decay.hpp) ---
    double altitude_km     = 840.0;  // catalog_A() (SL-16/Zenit-2), decay.cpp
    double inclination_deg = 71.0;   // catalog_A()

    // --- geometry / mass (Deliverable 1) ---
    int    n_beads = 8;          // N beads; n_beads=2 selects the rigid dumbbell-limit configuration (Deliverable 4)
    double tether_length_m = 3000.0;  // reused EdtConfig::tether_length_m (decay.hpp)
    double m_parent_kg      = 9000.0;  // catalog_A parent/debris mass (bead 0)
    double m_tip_kg         = 20.0;    // PLACEHOLDER (= EdtConfig::kit_mass_kg)
    double lambda_tether_kg_per_m = 2.7e-3;  // PLACEHOLDER tether linear density [kg/m] (BETs-scale tape, ~1e-6 m^2 Al); set to 0.0 for the massless-tether dumbbell-limit configuration

    // --- segment stiffness / damping (PLACEHOLDER "design" softened
    //     values, Deliverable 3 -- NOT the physical EA=7.0e4 N tape
    //     stiffness, which is prohibitively stiff for a fixed-step RK4 MC
    //     campaign; see the accompanying rigidity-ratio guardrails) ---
    double EA_design_N        = 250.0;  // PLACEHOLDER softened axial stiffness [N]
    double damping_c_Ns_per_m = 0.05;   // PLACEHOLDER per-segment dashpot [N.s/m]

    // --- current model (Deliverable 1) ---
    double I_cap_A = 2.0;  // reused EdtConfig::avg_current_a (decay.hpp)
    double eta_I    = 0.7;  // PLACEHOLDER collection efficiency [-] in [0,1]; the effective current magnitude commanded "on" by a controller is eta_I*I_cap_A

    // --- endpoint boundary (Deliverable 1) ---
    bool servicer_fixed = false;  // false = FREE dumbbell (Phase-1 default, matches the T7 dumbbell exactly); true = bead 0 pinned at the Hill-frame origin (servicer-fixed Dirichlet mode)

    // --- controller (Deliverable 5) ---
    ControllerMode controller = ControllerMode::Constant;
    double const_current_a = 2.0;   // Constant mode: SIGNED current [A] (validation / uncontrolled-baseline hook; the sign is an arbitrary but fixed convention -- see tether.cpp)
    double duty_on          = 0.75;  // FixedDuty (C2): reuses EdtConfig::eta_libration's meaning
    double switch_phase     = 0.0;   // FixedDuty (C2): ON-window phase offset in [0,1)
    double gate_hysteresis  = 0.02;  // PLACEHOLDER PhaseGated (C1) hysteresis band applied to the unit-current Lorentz-power probe (informal units -- see controller_c1_gate)

    // --- initial condition (Deliverable 4) ---
    double theta0_deg = 3.0;  // initial tilt from local vertical, matches the T7 seed; rates = 0 (Hill-frame velocities start at zero)

    // --- integration (Deliverable 3) ---
    // PLACEHOLDER RK4 step [s] (Deliverable-3 recommended MC pair for
    // EA_design=250 N). Two segment-eigenfrequency estimates exist for this
    // n_beads=8 chain (wp16_xcheck.py review, finding #3): the isolated-pair
    // reduced-mass estimate omega_pair*dt = 0.247, omega_pair/n ~= 973, and
    // the TRUE maximum chain (zone-boundary) eigenfrequency
    // omega_chain = 2*sqrt(k_seg/m_bead) ~= 1.42 rad/s -- sqrt(2) above
    // omega_pair for n_beads>2 (the two coincide only for the N=2 dumbbell).
    // Margined against THAT true chain maximum, omega_chain*dt ~= 0.351,
    // still comfortably inside the RK4 imaginary-axis stability boundary
    // 2*sqrt(2) ~= 2.828 (margin ~8x) -- the "dt is stable" conclusion is
    // unaffected; only the single-pair label previously quoted here
    // understated the true dispersion maximum.
    double dt_s       = 0.247;
    double sim_orbits = 20.0;   // PLACEHOLDER Phase-1 simulation horizon [orbits] -- deliberately SHORTER than the multi-year deorbit timescale; defensible because T7 itself finds libration divergence/saturation within O(1)-O(10) orbits (t7-libration-study.md Sec 5.5), so this window captures the phenomena of interest at a fraction of the (already only 200-orbit) T7 study's own cost. MC drivers may override this further downward for runtime (documented at the call site).
};

// -------------------------------------------------------------- diagnostics

// Per-step diagnostics (Deliverable 3 energy audit + divergence guards).
struct StepDiagnostics {
    double t_s               = 0.0;
    double chord_angle_deg   = 0.0;  // signed, atan2(bead[N-1]-bead[0]) from local vertical (+x/radial)
    double energy_jacobi     = 0.0;  // E_J = KE_rel + U_gg + U_spring (rotating-frame Jacobi integral)
    double p_lorentz_w       = 0.0;  // sum_i FL_i . v_i (trapezoidal start/end-of-step average)
    double p_damp_w          = 0.0;  // -sum_j c*ldot_j^2 over segments IN TENSION (<=0)
    double max_strain_ratio  = 0.0;  // max_j l_j/L0 (post-step)
    int    n_slack           = 0;    // segments with l_j < L0 this step
    int    n_segments        = 0;    // n_beads - 1
    double max_speed_m_s     = 0.0;
    double current_applied_a = 0.0;  // signed current actually applied this step
    double root_tension_n    = 0.0;  // tension in the segment touching bead 0
};

// Aggregate outcome of one run_tether_sim() call (Deliverables 3-5, 7).
struct SimResult {
    DivergeStatus status         = DivergeStatus::Ok;
    double stop_time_s           = 0.0;
    double max_chord_angle_deg   = 0.0;
    double final_chord_angle_deg = 0.0;
    double o45_orbit             = 0.0;  // orbit of first |chord angle| >= 45 deg; +inf if never reached
    double divergence_orbit      = 0.0;  // orbit of the terminal divergence-guard trip; +inf if never
    double energy_drift_per_orbit = 0.0;  // last full-orbit |drift| / E_scale (Deliverable 3)
    double slack_fraction        = 0.0;  // time-average fraction of segments slack
    double eta_lib_effective     = 0.0;  // time-average |I_applied| / I_cap_A (Deliverable 5 headline)
    double retained_drag_frac    = 0.0;  // == eta_lib_effective (deorbit-thrust bookkeeping alias, Deliverable 5)
    int    n_steps               = 0;
};

// ------------------------------------------------------------------- engine

// Bead-model state: positions/velocities in the Hill/LVLH rotating frame
// [m], [m/s]. z is not carried (Phase-1 is planar, z==0 by construction).
struct BeadState {
    std::vector<Eigen::Vector2d> pos;
    std::vector<Eigen::Vector2d> vel;
};

// Stateful fixed-step RK4 stepper for one TetherConfig (Deliverables 1-3).
// Owns the C1 controller's hysteresis state internally so run_tether_sim()
// and twin.cpp's truth-twin loop both get bit-identical stepping.
class TetherSim {
public:
    explicit TetherSim(const TetherConfig& cfg);

    // Advances one fixed dt_s RK4 step. If current_override_a is non-null,
    // that SIGNED current [A] is applied directly, bypassing cfg.controller
    // entirely -- this is the twin-to-twin "virtual computes, real
    // executes" hook (twin.cpp, Deliverable 6): the virtual twin decides
    // the gate/duty command from ITS OWN reduced-model state and commands
    // the truth twin through this parameter, rather than letting the truth
    // model gate on its own (full-fidelity) internal Lorentz-power reading.
    StepDiagnostics step(const double* current_override_a = nullptr);

    double chord_angle_rad() const;  // atan2(pos.back()-pos.front()) from local vertical (+x)
    double root_tension_n() const;   // tension in the segment touching bead 0 (last computed)
    double time_s() const { return t_s_; }
    double u_rad() const;            // argument of latitude, n*t (u0 = 0)
    double mean_motion_rad_s() const { return n_; }
    double field_Bn_tesla_value() const { return Bn_; }
    double energy_jacobi() const { return last_diag_.energy_jacobi; }
    double last_orbit_energy_drift() const { return last_orbit_drift_; }
    DivergeStatus status() const { return status_; }
    const BeadState& state() const { return state_; }
    const std::vector<double>& masses() const { return masses_; }
    const TetherConfig& config() const { return cfg_; }

    // Fixed-current probe (Deliverable 5): what P_lorentz WOULD be for a
    // UNIT (+1 A) reference current at the CURRENT (pre-step) state,
    // without mutating anything. Used by the internal C1 gate and exposed
    // so twin.cpp's virtual twin can compute an analogous probe from its
    // own reduced state independently.
    double probe_unit_current_power_w() const;

private:
    TetherConfig cfg_;
    std::vector<double> masses_;
    BeadState state_;
    double t_s_ = 0.0;
    double n_ = 0.0;
    double L0_m_ = 0.0;
    double k_seg_n_per_m_ = 0.0;
    double Bn_ = 0.0;
    bool   gate_on_ = true;  // C1 hysteresis state
    DivergeStatus status_ = DivergeStatus::Ok;
    StepDiagnostics last_diag_;
    double energy_prev_j_ = 0.0;
    // EnergySpike guard state (recalibrated per the wp16_xcheck.py
    // adversarial review -- see tether.cpp's guard comment): e_scale_/
    // e_floor_j_ are the run's physical energy scale and noise floor,
    // computed once in the ctor; residual_ema_ is SEEDED at e_floor_j_ (not
    // 0, not a "not yet warmed up" sentinel) and only starts tracking the
    // per-step residual after step_count_ passes the warmup.
    double e_scale_ = 0.0;
    double e_floor_j_ = 0.0;
    double residual_ema_ = 0.0;
    long   step_count_ = 0;
    double energy_at_last_mark_j_ = 0.0;
    double integrated_pump_since_mark_j_ = 0.0;
    int    last_orbit_marked_ = -1;
    double last_orbit_drift_ = 0.0;
};

// Runs a TetherSim from t=0 to cfg.sim_orbits orbits (or until a divergence
// guard trips), aggregating the SimResult (Deliverables 3-5, 7). A pure
// function of cfg: fully deterministic, no RNG (see file header).
SimResult run_tether_sim(const TetherConfig& cfg);

// -------------------------------------------------------------- controllers
// Deliverable 5, "pure functions": each takes the current decision input
// and returns the gate state, with no hidden state of its own -- the caller
// (TetherSim, or twin.cpp's virtual-twin loop) owns whatever state persists
// across calls (e.g. C1's hysteresis latch).

// C1 phase-gated / energy-dissipative bang-bang gate: ON (thrust) if the
// unit-current Lorentz-power probe is <= -delta (Lorentz removing
// libration energy), OFF if >= +delta (pumping), else HOLD the previous
// gate state (hysteresis band, prevents chatter).
bool controller_c1_gate(double p_probe_w, double delta, bool previous_gate_on);

// C2 fixed duty schedule: ON for the first duty_on fraction of each orbit,
// starting at switch_phase (both in [0,1)); a pure function of the orbit
// clock, no persistent state.
bool controller_c2_gate(double u_rad, double duty_on, double switch_phase);

}  // namespace adsc
