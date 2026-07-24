#pragma once

#include <vector>

#include <Eigen/Dense>

#include "adsc/relmotion.hpp"  // kEarthMu, kPi (NOT kEarthRadius -- see the radius-convention note below)

namespace adsc {

// ============================================================================
// WP16 Digital Twin Phase 1/2 -- lumped-mass EDT tether dynamics
// ----------------------------------------------------------------------------
// [DT-v1: lumped-mass tether, aligned dipole, twin-to-twin]. NO real asset
// exists anywhere in this file or in twin.hpp/twin.cpp: the "truth" state
// used by the twin-to-twin sync (twin.hpp) is itself a perturbed-parameter
// instance of the SAME simulated model implemented here. T7 (EDT libration
// dynamic-stability trade, _tasks_local/t7-libration-study.md) stays OPEN;
// nothing in this file resolves it, in EITHER mode below. The two
// controllers below (Deliverable 5) are in-model PROPOSALS, evaluated only
// against this simulated physics -- never a claim about a resolved or
// operational system.
//
// Scope (Phase 1, PLANAR, default): N point-mass beads on a tension-only
// spring-dashpot tether, in the linearized Hill/LVLH rotating frame (radial
// x, along-track y -- the SAME sign convention as relmotion.hpp's CwModel),
// under gravity-gradient + Coriolis + a per-segment Lorentz force driven
// ONLY by the orbit-normal field component B_n. B_n is CONSTANT around an
// aligned-dipole orbit (wp13-edt-derivation.md Sec 2.3) -- the DC / in-plane
// channel. This planar Phase-1 model reproduces only the BOUNDED-libration
// / DC-tumble behaviour of T7 Table 5.1 (Deliverable 4, target T4a).
//
// [DT-v2: 3D] Scope (Phase 2, out-of-plane, TetherConfig::out_of_plane =
// true, a RUNTIME-SELECTABLE MODE on this SAME code path -- not a second
// integrator, house fidelity-ladder pattern, R1 no-fork): the cross-track
// field components (B_r, B_t), which are constant-around-the-orbit-normal
// in Phase 1 but vary at the orbital frequency, now drive a per-segment
// out-of-plane (z / cross-track / roll) Lorentz force -- the channel T7's
// own Sec 3.5 identifies as the real Pelaez energy-pumping mechanism, and
// which the planar model is blind to BY CONSTRUCTION. In this mode:
//   - BeadState carries a full Hill-frame z (cross-track) component always;
//     bead z accelerations get an ADDED gravity-gradient/free-particle term
//     -n^2*z (t7-libration-study.md Sec 2, roll+pitch derivation cross-
//     checked against relmotion.hpp's CwModel z_ddot + n^2 z = a_z
//     convention) and an ADDED z-component of the (now fully 3D) per-segment
//     Lorentz force I*l*(e x B), e and B both 3D.
//   - The DivergedAngle guard and o45 crossing generalize from the planar
//     signed chord angle to the 3D solid-angle "cone angle from local
//     vertical" (arccos(d_x/|d|)), which reduces to |chord_angle_deg|
//     exactly when z==0 (see tether.cpp compute_forces/step comments).
//   - Guard (BINDING for planar bit-identity): when out_of_plane == false,
//     B_x = B_y are forced to 0 and no z-forcing term (GG, Lorentz) is ever
//     ADDED to a bead's z acceleration -- every new Phase-2 code path only
//     ADDS terms under an out_of_plane branch, it never replaces or
//     reorders the existing planar x/y computation. Combined with z==0 (and
//     vz==0) at every Phase-1 IC, this makes the planar mode's committed
//     wp16_twin.csv rows VALUE-IDENTICAL (in fact bit-identical) to Phase 1.
//   - T4b (the out-of-plane pumping-onset target, deferred explicitly in
//     Phase 1) is attempted via this mode's dumbbell-limit acceptance test;
//     see tether.cpp / main_twin.cpp / tests/test_tether.cpp for the T7
//     ADVERSARIAL CORRECTION cross-check (o45 ~ 0.53 orbit, 80-deg
//     divergence by ~4.35 orbits at the nominal I=2A/71deg dumbbell
//     config). This DOES NOT resolve T7; whether the phase-gated C1
//     controller suppresses this roll-driven divergence is the real,
//     first-class open question this phase answers (see twin.hpp/twin.cpp
//     and main_twin.cpp's 3D Monte Carlo section) -- a NO is reported
//     plainly, never tuned away.
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

// In-plane-field / cross-track-forcing components (wp13-edt-derivation.md
// Sec 2.3): B_r = -2*beta*sin(i)*sin(u), B_t = beta*sin(i)*cos(u). NOT
// consumed by the Phase-1 planar EOM (out_of_plane == false): that mode
// forces B_x = B_y = 0 in compute_forces (tether.cpp), so these stay
// diagnostics-only there. [DT-v2: 3D] Phase-2 out-of-plane mode
// (TetherConfig::out_of_plane == true) DOES consume both, evaluated at the
// current argument of latitude u = n*t on each RK4 sub-stage, as the
// orbital-frequency (B_r, B_t) that drives the out-of-plane roll Lorentz
// force -- the Pelaez energy-pumping channel the planar model cannot see.
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

    // --- [DT-v2: 3D] out-of-plane mode (Phase 2) ---
    // Runtime-selectable mode on this SAME code path (house fidelity-ladder
    // pattern, R1 no-fork), NOT a second integrator. false (default) is the
    // Phase-1 planar mode, VALUE-IDENTICAL (in fact bit-identical) to every
    // committed Phase-1 result: B_x = B_y are forced to 0 and no z-forcing
    // term is ever added to a bead's acceleration (see tether.cpp). true
    // additionally integrates the cross-track field components (B_r, B_t)
    // and a full 3D per-segment tension/Lorentz force, reproducing the T7
    // out-of-plane roll-libration / Pelaez energy-pumping channel (Sec 3.5)
    // the planar mode cannot see by construction. T7 stays OPEN in EITHER
    // mode; this flag only selects which forcing terms are integrated.
    bool out_of_plane = false;
    // Initial out-of-plane (roll) tilt [deg], the phi0 companion to theta0
    // above (T7's own theta0=phi0=3deg dumbbell seed, Sec 5.4). IGNORED
    // (forced to an effective 0) whenever out_of_plane == false, so a
    // stray nonzero value can never perturb a planar-mode run.
    double phi0_deg = 0.0;

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
    double chord_angle_deg   = 0.0;  // signed, atan2(bead[N-1]-bead[0]) from local vertical (+x/radial); PLANAR-plane (theta-only) angle, computed identically regardless of mode
    double energy_jacobi     = 0.0;  // E_J = KE_rel + U_gg + U_spring (rotating-frame Jacobi integral)
    double p_lorentz_w       = 0.0;  // sum_i FL_i . v_i (trapezoidal start/end-of-step average)
    double p_damp_w          = 0.0;  // -sum_j c*ldot_j^2 over segments IN TENSION (<=0)
    double max_strain_ratio  = 0.0;  // max_j l_j/L0 (post-step)
    int    n_slack           = 0;    // segments with l_j < L0 this step
    int    n_segments        = 0;    // n_beads - 1
    double max_speed_m_s     = 0.0;
    double current_applied_a = 0.0;  // signed current actually applied this step
    double root_tension_n    = 0.0;  // tension in the segment touching bead 0
    // [DT-v2: 3D] solid-angle generalization of chord_angle_deg: angle
    // between the chord (bead[N-1]-bead[0]) and local vertical (+x/radial),
    // unsigned, in [0,180] deg -- arccos(d_x/|d|). Computed every step in
    // EITHER mode (cheap); reduces EXACTLY to |chord_angle_deg| when z==0
    // (planar mode / T7's own cone-angle divergence definition, Sec 5.4).
    double cone_angle_deg    = 0.0;
    // [DT-v2: 3D] roll angle: arcsin(d_z/|d|), the out-of-plane companion
    // to chord_angle_deg's in-plane pitch -- 0 identically in planar mode
    // (z==0 by construction).
    double roll_angle_deg    = 0.0;
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

    // [DT-v2: 3D] additive fields (ignored/near-zero in planar mode; do NOT
    // alter max_chord_angle_deg/o45_orbit/divergence_orbit's own existing
    // computation, which stays exactly the fabs(chord_angle_deg)-based
    // Phase-1 definition in EVERY mode -- see tether.cpp). These track the
    // solid-angle/roll generalization in parallel, unconditionally (cheap):
    double max_cone_angle_deg   = 0.0;  // max_j cone_angle_deg over the run
    double max_roll_deg         = 0.0;  // max_j |roll_angle_deg| over the run
    double o45_cone_orbit       = 0.0;  // orbit of first cone_angle_deg >= 45 deg; +inf if never (this is T7's OWN o45 definition, Sec 5.4, and the one the D9 dumbbell-limit acceptance test checks in out_of_plane mode)
};

// ------------------------------------------------------------------- engine

// Bead-model state: positions/velocities in the Hill/LVLH rotating frame
// [m], [m/s]. [DT-v2: 3D] z (cross-track) is now carried ALWAYS (promoted
// Vector2d -> Vector3d, house pattern: single code path, runtime-selectable
// mode, R1 no-fork) -- but z == 0 and vz == 0 IDENTICALLY (bit-for-bit) in
// planar mode (TetherConfig::out_of_plane == false), because every z-force
// term in compute_forces (tether.cpp) is only ever ADDED under an
// out_of_plane branch, never computed or replaced unconditionally. Phase-1
// planar behavior is therefore preserved exactly, not merely "z stays
// small".
struct BeadState {
    std::vector<Eigen::Vector3d> pos;
    std::vector<Eigen::Vector3d> vel;
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

    double chord_angle_rad() const;  // atan2(pos.back()-pos.front()) from local vertical (+x); PLANAR (theta-only) angle, computed identically in every mode
    // [DT-v2: 3D] solid-angle generalization: arccos(clamp(d_x/|d|,-1,1)),
    // d = pos.back()-pos.front() (3D). Reduces exactly to |chord_angle_rad()|
    // when z == 0 (planar mode). This is T7's OWN divergence-event/o45
    // definition (Sec 5.4: arccos(cos theta cos phi)).
    double cone_angle_rad() const;
    // [DT-v2: 3D] roll angle: arcsin(clamp(d_z/|d|,-1,1)) -- 0 identically
    // in planar mode (z == 0 by construction).
    double roll_angle_rad() const;
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
