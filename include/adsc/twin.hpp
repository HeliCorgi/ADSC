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

// One twin-to-twin sync convergence report (Deliverables 6/7).
struct SyncReport {
    int    n_orbits               = 0;
    double final_I_eff_rel_err    = 0.0;
    double final_c_hat_rel_err    = 0.0;
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
SyncReport run_twin_sync(const TruthTwinConfig& truth_cfg, const VirtualTwinConfig& virt_cfg,
                         ControllerMode controller, double sim_orbits, uint64_t seed);

}  // namespace adsc
