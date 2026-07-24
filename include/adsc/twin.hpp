#pragma once

#include <cstdint>

#include <Eigen/Dense>

#include "adsc/tether.hpp"

namespace adsc {

// ============================================================================
// WP16 Digital Twin Phase 1 -- twin-to-twin sync + 4-state EKF
// ----------------------------------------------------------------------------
// [DT-v1: lumped-mass tether, aligned dipole, twin-to-twin]. HONESTY FRAME
// (binding, repeated here because this is the file where it matters most):
// there is NO real asset anywhere in this codebase. "TRUTH-TWIN" below is
// itself a perturbed-parameter instance of the SAME simulated bead-tether
// model (tether.hpp) playing the role of "Real"; it emits NOISY SIMULATED
// sensor measurements, nothing more. "VIRTUAL-TWIN" runs a REDUCED
// pitch-pendulum-equivalent model and assimilates those simulated
// measurements with an EKF; it never sees the truth-twin's parameters. This
// file demonstrates twin-to-twin assimilation ONLY. There is no operational
// twin, and T7 (EDT libration dynamic-stability trade,
// _tasks_local/t7-libration-study.md) is NOT resolved by any result here.
//
// Scope simplification: both twins here use the N=2 rigid-dumbbell
// TetherConfig configuration (Deliverable 4's exact geometry), so the
// virtual twin's reduced 2-body pitch-pendulum model is not just an
// approximation of the truth twin but the SAME reduced-order system the
// dumbbell-limit validation already exercises -- this sidesteps an
// otherwise-arbitrary segment-length mapping between an N=8 truth model and
// a single-segment reduced filter. The design note in the WP16 task record
// explicitly permits "the full N=8 (or N=2) bead model" for the truth twin.
//
// EKF state x = [theta, thetadot, I_eff, c_hat]^T. Process model, Jacobian,
// and measurement model follow the WP16 design record verbatim (theta =
// reduced pitch angle, tension proxy h2 = mu*L*(n^2*(1+3cos^2 theta) +
// thetadot^2)). Joseph-form covariance update + explicit symmetrization
// every step, matching the estimator.hpp/estimator.cpp house convention
// (estimator.hpp file header, lines 59-61) -- reimplemented locally here
// (not templated off estimator.cpp's private helper) per this WP's own
// recon note: a small standalone EKF class rather than subclassing
// TranslationEkf/AttitudeMekf, which are tightly coupled to CW/quaternion
// state representations this 4-state filter does not share.
//
// Determinism (R6): sensor noise uses the header-exposed
// adsc::GaussianSource (estimator.hpp) seeded via SplitMix64
// (campaign.hpp/campaign.cpp idiom) -- one canonical RNG choice for all of
// WP16, per this WP's own recon note.
// ============================================================================

// Truth-twin configuration: a perturbed TetherConfig (Deliverable 7
// dispersions applied by the caller, e.g. main_twin.cpp) plus the simulated
// sensor-noise model. All PLACEHOLDER.
struct TruthTwinConfig {
    TetherConfig truth_tether;      // perturbed EA_design_N/damping_c_Ns_per_m/eta_I etc.
    double sigma_theta_deg = 0.5;   // PLACEHOLDER angle-sensor noise (1-sigma) [deg]
    double sigma_tension_n = 0.05;  // PLACEHOLDER tension-sensor noise (1-sigma) [N]
};

// EKF state x = [theta(rad), thetadot(rad/s), I_eff(A), c_hat(N.s/m)]^T.
struct EkfState {
    Eigen::Vector4d x = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
};

// Virtual-twin reduced-model configuration: the geometry it ASSUMES (never
// the truth-twin's actual EA/c/eta_I) plus PLACEHOLDER process-noise gains.
struct VirtualTwinConfig {
    double altitude_km     = 840.0;
    double inclination_deg = 71.0;
    double m_parent_kg     = 9000.0;
    double m_tip_kg        = 20.0;
    double tether_length_m = 3000.0;

    // PLACEHOLDER process noise (random-walk white-noise scaling, per dt).
    double q_theta     = 1e-12;  // small: absorbs the UNMODELED out-of-plane pumping channel (honest reduced-model mismatch, Deliverable 6)
    double q_thetadot  = 1e-10;
    double q_I_eff     = 1e-5;   // random walk on the estimated effective current [A^2/s]
    double q_c_hat     = 1e-7;   // random walk on gamma=c_hat/(2*mu), a TUNABLE ASSUMED pitch-damping
                                  // rate the EKF fits online -- NOT a faithful reduction of the
                                  // truth-twin's axial dashpot (an axial c gives ~zero direct pitch
                                  // damping in rigid rotation; see twin.cpp predict()) -- this gain
                                  // is what absorbs that reduced-model mismatch [(N.s/m)^2/s]
};

// Standalone 4-state EKF for the reduced pitch-pendulum-equivalent model
// (Deliverable 6). Joseph-form update + explicit symmetrization every step.
class VirtualTwinEkf {
public:
    VirtualTwinEkf(const VirtualTwinConfig& cfg, const EkfState& x0);

    // Nonlinear RK4 mean propagation (same fixed dt as the truth-twin's own
    // stepper) + 2nd-order-Taylor STM covariance propagation
    // Phi = I4 + F*dt + 0.5*(F*dt)^2, F evaluated at the pre-step state.
    void predict(double dt_s);

    // Joint (angle, tension) measurement update. Returns NIS = nu^T S^-1 nu
    // (2 dof) for filter-consistency gating (Deliverable 6/7).
    double update(double z_theta_rad, double z_tension_n,
                 double sigma_theta_rad, double sigma_tension_n);

    const EkfState& state() const { return st_; }
    double a_L() const { return a_L_; }        // Lorentz coefficient [rad/s^2 per A], exposed for the C1 virtual-gate probe
    double mean_motion_rad_s() const { return n_; }
    double reduced_mass_kg() const { return mu_; }

private:
    VirtualTwinConfig cfg_;
    EkfState st_;
    double n_ = 0.0;    // mean motion [rad/s]
    double mu_ = 0.0;   // reduced mass [kg]
    double a_L_ = 0.0;  // -Delta*B_n/(2*mu) [rad/s^2 per A]
};

// ============================================================================
// [DT-v2: 3D] WP16 Phase 2 -- 6-state roll-augmented EKF (Deliverable D8)
// ----------------------------------------------------------------------------
// A wholly NEW, ADDITIVE class (the existing 4-state EkfState/VirtualTwinEkf
// above is byte-untouched, still the Phase-1 planar twin-sync engine).
// State x = [theta, thetadot, phi, phidot, I_eff, c_hat]^T -- the existing
// 4-state x = [theta, thetadot, I_eff, c_hat]^T with the two roll states
// (phi, phidot) inserted in the middle, matching the WP16 Phase-2 design
// record's own state ordering. Continuous dynamics near the operating
// point, physical-time form of t7-libration-study.md Eqs 2.2-2.3 (x n^2,
// same convention as the existing 4-state predict()):
//   thetaddot = [n^2*Q_theta(theta,phi,u,I_eff) + 2*phidot*(thetadot+n)*
//                sin(phi)*cos(phi) - 3*n^2*sin(theta)*cos(theta)*cos^2(phi)]
//               / cos^2(phi)  - 2*gamma*thetadot
//   phiddot   = n^2*Q_phi(theta,phi,u,I_eff) - (thetadot+n)^2*sin(phi)*
//               cos(phi) - 3*n^2*cos^2(theta)*sin(phi)*cos(phi)
// where (Q_theta, Q_phi) are T7's own nondimensional electrodynamic torques
// (Sec 3.3-3.4), evaluated with the FULL 3D field (B_r(u), B_t(u), B_n) --
// KEY DIFFERENCE from the 4-state model: the Lorentz coefficients are
// u-DEPENDENT here (through B_r(u), B_t(u)), not the 4-state model's
// constant a_L. gamma = c_hat/(2*mu), a TUNABLE assumed pitch-damping rate
// (same meaning/caveats as the 4-state model's gamma -- see VirtualTwinConfig
// ::q_c_hat and the weak-observability finding on TwinSyncReport, which
// applies here unchanged).
//
// The continuous Jacobian F below is evaluated at the operating point with
// the cos^2(phi) divisor in the pitch equation held at its pre-step value
// (a linearization convenience, not a claim of exactness away from small
// phi -- acceptable because F only propagates the COVARIANCE, not the mean,
// which still uses the full nonlinear f via RK4).
//
// OBSERVABILITY FINDING [DT-v2: 3D] (pre-registered, honest): a single
// SCALAR angle-only sensor (e.g. the cone angle ang=arccos(cos th cos ph))
// leaves ROLL UNOBSERVABLE near the origin (its Jacobian w.r.t. (th,ph) is
// singular there and symmetric in the two states, so it cannot partition
// pitch from roll). A TWO-AXIS chord-direction measurement
// (y=sin(th)cos(ph), z=sin(ph)) makes roll directly observable (the z
// measurement's Jacobian row is [0,0,cos(ph),0,0,0], nonsingular at ph=0).
// update_scalar_angle() and update_chord2axis() below expose BOTH
// measurement models so this finding can be demonstrated directly (see
// tests/test_twin.cpp), not just asserted in a comment.
// ============================================================================

// EKF state x = [theta, thetadot, phi, phidot, I_eff, c_hat]^T (rad, rad/s,
// rad, rad/s, A, N.s/m).
struct EkfState3D {
    Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Identity();
};

// Process-noise / geometry configuration for the 6-state filter -- mirrors
// VirtualTwinConfig with an added roll process-noise pair (q_phi, q_phidot),
// same "small, absorbs unmodeled mismatch" convention as q_theta/q_thetadot.
struct VirtualTwinConfig3D {
    double altitude_km     = 840.0;
    double inclination_deg = 71.0;
    double m_parent_kg     = 9000.0;
    double m_tip_kg        = 20.0;
    double tether_length_m = 3000.0;

    double q_theta    = 1e-12;
    double q_thetadot = 1e-10;
    double q_phi      = 1e-12;   // [DT-v2: 3D] roll-angle process noise, same order as q_theta
    double q_phidot   = 1e-10;   // [DT-v2: 3D] roll-rate process noise, same order as q_thetadot
    double q_I_eff    = 1e-5;
    double q_c_hat    = 1e-7;
};

// Standalone 6-state EKF for the roll-augmented reduced pitch+roll model
// (Deliverable D8). Joseph-form update + explicit symmetrization every
// step, same house convention as the 4-state VirtualTwinEkf.
class VirtualTwinEkf3D {
public:
    VirtualTwinEkf3D(const VirtualTwinConfig3D& cfg, const EkfState3D& x0);

    // Nonlinear RK4 mean propagation + 2nd-order-Taylor STM covariance
    // propagation, evaluated at argument of latitude u = n*t_now (the
    // caller tracks t_now; a fresh u is required every predict() call since
    // the roll-forcing field components vary with it).
    void predict(double dt_s, double u_rad);

    // [DT-v2: 3D] scalar cone-angle-only measurement: z = arccos(cos(theta)
    // cos(phi)) (T7's own divergence-angle definition). Demonstrates the
    // roll-UNOBSERVABLE finding: this update alone cannot shrink phi's
    // variance below what its own process noise re-inflates each predict().
    double update_scalar_angle(double z_ang_rad, double sigma_ang_rad);

    // [DT-v2: 3D] 2-axis chord-direction + tension measurement: z =
    // (sin(theta)cos(phi), sin(phi), tension) -- the RECOMMENDED sensor for
    // this 3D twin (D8): the z-component of the chord makes roll directly
    // observable, unlike the scalar-angle-only update above. Returns NIS
    // (3 dof) for filter-consistency gating.
    double update_chord2axis(double z_y, double z_z, double z_tension_n,
                             double sigma_y, double sigma_z, double sigma_tension_n);

    const EkfState3D& state() const { return st_; }
    double mean_motion_rad_s() const { return n_; }
    double reduced_mass_kg() const { return mu_; }

private:
    VirtualTwinConfig3D cfg_;
    EkfState3D st_;
    double n_ = 0.0;
    double mu_ = 0.0;
    double delta_ = 0.0;  // (m_parent - m_tip)/(m_parent + m_tip), signed asymmetry (T7 Sec 1)
    double beta_ = 0.0;   // aligned-dipole field magnitude at this altitude [T]
    double inc_rad_ = 0.0;
};

// One twin-to-twin sync convergence report (Deliverables 6/7).
//
// Named TwinSyncReport (not SyncReport) deliberately: mission.hpp already
// declares an unrelated adsc::SyncReport for the WP2 tumble-sync outcome
// (synced/sync_time_s/max_rate_err_deg_s/...). The two report different
// physical quantities for different subsystems; they only ever collided on
// the name, not the concept, so this file gets its own type rather than
// merging two unrelated field sets into one struct.
//
// WEAK-OBSERVABILITY FINDING [DT-v1] (a genuine digital-twin result, stated
// plainly, not a bug): I_eff is STRONGLY observable from this measurement set
// -- it enters the pitch dynamics as the Lorentz torque a_L*I_eff and so moves
// the angle innovation directly (an independent finite-difference cross-check,
// _tasks_local/wp16_xcheck.py, finds the angle measurement ~2.7e6x more
// sensitive to i_eff_true than to c_true, noise-normalized). c_hat is NOT: it
// is the EKF's TUNABLE effective pitch-damping parameter (gamma=c_hat/(2*mu)),
// which is a DIFFERENT physical quantity from the truth twin's AXIAL dashpot
// c_true. As the VirtualTwinConfig::q_c_hat comment and twin.cpp predict()
// document, an axial dashpot produces ~zero direct pitch damping in near-rigid
// rotation (the truth twin's free-decay rate is ~10 orders below gamma;
// wp16_xcheck.py part (d)), so the (angle, tension) data carry almost no
// information about c_true and the tension channel that does respond to c_true
// is not connected to c_hat by the measurement model (H(1,3)=0). There is thus
// NO data-driven reason for c_hat to converge to c_true; the estimator instead
// pins c_hat to the effective pitch-damping the data supports (near zero, or a
// small noise-driven value) with HONEST residual uncertainty. The fields below
// therefore report c_hat as bounded-with-uncertainty, not as a c_true match;
// see tests/test_twin.cpp block 2 for the restructured (observable-quantity)
// acceptance criteria.
struct TwinSyncReport {
    int    n_orbits               = 0;
    double final_I_eff_rel_err    = 0.0;
    double final_c_hat_rel_err    = 0.0;  // |c_hat - c_true|/c_true; reported for the record, NOT asserted small -- c_hat is weakly observable (see finding above)
    double final_c_hat            = 0.0;  // raw final c_hat [N.s/m]: the EKF's effective pitch-damping estimate (bounded-with-uncertainty, not a c_true match)
    double final_I_eff            = 0.0;  // raw final I_eff [A]: strongly observable via the Lorentz-torque magnitude
    double min_c_hat_variance     = 0.0;  // min P(3,3) over the run: the filter's honest uncertainty on c_hat must NOT collapse to spurious certainty (weak observability)
    bool   cov_spd_all_steps      = true; // P stayed symmetric AND positive-definite (Cholesky) at every step of the sync run
    double theta_rmse_deg         = 0.0;  // RMSE of (EKF theta - truth chord angle) over the whole run
    double median_nis             = 0.0;  // 2-dof NIS; 95% chi-square(2) band is [0.05, 7.38]
    bool   converged              = false;
    int    converged_at_orbit     = -1;   // -1 if never
};

// Runs one twin-to-twin sync demo: the TRUTH-TWIN steps the full (perturbed)
// bead model; every dt_s it emits a noisy (angle, tension) pair; the
// VIRTUAL-TWIN assimilates via the EKF above. VIRTUAL-TO-REAL PUSHBACK
// (Deliverable 6): for controller in {PhaseGated, FixedDuty}, the commanded
// current applied to the TRUTH twin is computed from the VIRTUAL twin's own
// estimated state (EkfState), never from the truth twin's own parameters --
// i.e. "Real runs the schedule Virtual computed." For controller ==
// Constant, the truth twin's own const_current_a is applied directly (no
// feedback loop to evaluate; this is the passive baseline). Deterministic
// given seed (sensor noise only; the dynamics themselves have no RNG).
TwinSyncReport run_twin_sync(const TruthTwinConfig& truth_cfg, const VirtualTwinConfig& virt_cfg,
                              ControllerMode controller, double sim_orbits, uint64_t seed);

}  // namespace adsc
