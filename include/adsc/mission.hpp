#pragma once

#include <cstdint>

#include <Eigen/Dense>

#include "adsc/controller.hpp"
#include "adsc/dynamics.hpp"
#include "adsc/estimator.hpp"
#include "adsc/fuel_store.hpp"
#include "adsc/relmotion.hpp"
#include "adsc/thermal.hpp"

namespace adsc {

// Vehicle and mission constants for a small ADR servicer (~27 kg dry, ~34 kg
// wet as configured below). Grouped here so nothing is a bare literal buried
// in the logic.
struct Config {
    double dry_mass_kg          = 27.2;
    double base_inertia         = 0.55;   // isotropic seed [kg m^2]
    double initial_fuel_kg      = 7.2;
    double deorbit_reserve_kg   = 1.2;

    double max_v_rel            = 0.15;    // capture closing-speed cap [m/s]
    double abort_dv             = 2.0;     // safe-abort impulse cap [m/s]

    // WP1: relative orbital motion (LVLH about the target's circular orbit).
    double target_altitude_km   = 825.0;   // PLACEHOLDER: SL-16-class band (D2) [km]
    double keep_out_radius_m    = 200.0;   // approach keep-out sphere radius [m]

    // F1: post-abort coast verification sweep (numerical check parameters,
    // not physical placeholders).
    double abort_coast_check_periods = 2.0;  // horizon [target orbital periods]
    double abort_coast_check_dt_s    = 1.0;  // RK4 step for the range sweep [s]

    // WP2: tumble synchronization. The target inertia is deliberately
    // asymmetric so its torque-free rate vector precesses (an isotropic body
    // tumbles at constant rate, which would make tracking trivially easy);
    // only the ratios matter for torque-free motion, not the absolute scale.
    double sync_target_rate_deg_s = 2.0;    // PLACEHOLDER: tumble rate, spec range 0.5-5 [deg/s]
    Eigen::Vector3d target_inertia_diag{1.0, 0.6, 0.3};  // PLACEHOLDER principal moments [kg m^2]

    // WP2 acceptance thresholds (spec section 5).
    double sync_rate_tol_deg_s = 0.1;   // |w_rel| tolerance [deg/s]
    double sync_att_tol_deg    = 2.0;   // attitude-error tolerance [deg]
    double sync_hold_s         = 30.0;  // dwell the criteria must hold [s]

    // WP2 sync-phase DACS gains: holding sync on a precessing target needs a
    // finer firing deadband than the detumble phase (coasting inside the
    // deadband lets the relative rate drift at the target's angular
    // acceleration, so the deadband bounds the hold error). PLACEHOLDER values.
    SlidingModeController::Gains sync_gains{0.6, 0.08, 0.03, 3.0e-4, 0.05};

    // WP3: kit + deorbit-decay trades. PLACEHOLDER physical values (R10).
    double kit_mass_kg          = 2.4;    // installed deorbit-kit mass [kg]
    double kit_sail_area_m2     = 120.0;  // installed drag-sail area [m^2]
    double drag_cd              = 2.2;    // free-molecular drag coefficient [-]
    // Stop altitude: below ~150-200 km the orbit decays within days regardless
    // of A/m, so decay is integrated to a handoff altitude and the object is
    // handed to rapid reentry; integrating lower adds negligible time.
    double reentry_handoff_altitude_km = 180.0;
    // Solar-cycle density scaling (T4): decay figures are reported over the
    // min..max range, never a point value. PLACEHOLDER values: a single
    // altitude-independent factor is a deliberately coarse solar-activity
    // proxy — the real min-to-max density swing is strongly altitude-dependent
    // (roughly an order of magnitude near 800 km; see the discussion around
    // Vallado's exponential model, Table 8-4). Refine from MSIS/MASTER-class
    // atmosphere data when the evidence pack is filled.
    double solar_min_density_factor = 0.5;  // PLACEHOLDER solar-min scaling
    double solar_max_density_factor = 8.0;  // PLACEHOLDER solar-max scaling

    // WP3 installer-mission GNC tuning (R10; the corridor matches the WP1 demo).
    double approach_rho_far_m     = 1200.0;  // outer corridor hold amplitude [m]
    double approach_rho_near_m    = 300.0;   // inner corridor hold amplitude [m]
    int    approach_holds         = 10;      // corridor hold-ellipse count
    double depart_standoff_factor = 2.0;     // depart hold range / keep-out radius

    // WP4: sensor + estimator abstractions. All PLACEHOLDER values (R10);
    // sensors are Gaussian abstractions of real hardware. The bias fields are
    // sensitivity-study knobs and are NOT estimated (known limitation); the
    // filter-consistency tests assume the zero defaults.
    double gyro_sigma_rad_s    = 1.0e-4;   // per-step white rate noise [rad/s]
    double gyro_bias_rad_s     = 0.0;      // unestimated bias knob [rad/s]
    double st_sigma_rad        = 1.0e-4;   // star-tracker angle noise [rad]
    double vision_sigma_rad    = 2.0e-3;   // vision relative-pose noise [rad]
    double range_sigma_m       = 0.05;     // rangefinder noise [m]
    double range_bias_m        = 0.0;      // unestimated bias knob [m]
    double los_sigma           = 5.0e-4;   // LOS unit-vector noise per axis [-]
    double los_bias            = 0.0;      // unestimated LOS bias knob per axis [-]
    double wt_disturb_sigma    = 1.0e-6;   // target random ang. accel [rad/s^2]
    double trans_vel_noise_m_s = 1.0e-5;   // per-step relative vel noise [m/s]
    double st_rate_hz          = 5.0;      // star-tracker update rate [Hz]
    double vision_rate_hz      = 2.0;      // vision pose update rate [Hz]
    double ranging_rate_hz     = 10.0;     // range + LOS update rate [Hz]
    // Filter initialization 1-sigma uncertainties and the consistency-stats
    // window start (transients before it are excluded from NEES/NIS).
    double est_init_att_sigma_rad  = 5.0e-3;
    double est_init_wt_sigma_rad_s = 1.0e-3;
    double est_init_pos_sigma_m    = 5.0;
    double est_init_vel_sigma_m_s  = 0.05;
    double est_stats_start_s       = 40.0;
    uint32_t est_seed              = 20260703u;  // fixed RNG seed (R6)

    double control_dt           = 0.01;   // GNC loop step [s]
    double pcm_capacity_j       = 5000.0;
    double max_safe_time_s      = 14400.0; // 4 h
    double avionics_power_w     = 1.2;     // steady heat into PCM [W]

    double regularization_eps   = 1e-6;
};

// Outcome of a safe-abort computation (F1). The commanded impulse is capped at
// Config::abort_dv. When the cap binds, the along-track drift is not fully
// nulled and the bounded-orbit guarantee is lost — so instead of implying
// safety, the result carries the *verified* minimum range of the actual
// post-burn thrust-off coast.
struct SafeAbort {
    enum class Status {
        Clean,   // full impulse delivered: drift-free safety ellipse
        Capped   // |dv| hit Config::abort_dv: residual drift remains
    };

    Eigen::Vector3d dv = Eigen::Vector3d::Zero();  // commanded impulse [m/s]
    Status status = Status::Clean;

    // Minimum range to the target over the post-burn coast, propagated for
    // Config::abort_coast_check_periods orbital periods at
    // Config::abort_coast_check_dt_s. Reported for Clean and Capped alike. [m]
    double coast_min_range_m = 0.0;
};

// Outcome of a tumble-synchronization run (WP2).
struct SyncReport {
    bool   synced = false;        // criteria held for sync_hold_s within the run
    double sync_time_s = 0.0;     // start of the first completed hold window
    // Maximum errors observed AFTER the dwell window completed (from
    // sync_time_s + sync_hold_s to the end of the run). The dwell itself is
    // certified by the criteria staying satisfied throughout; these fields
    // track whether sync degrades after it is declared.
    double max_rate_err_deg_s = 0.0;
    double max_att_err_deg    = 0.0;
    double final_rate_err_deg_s = 0.0;
    double final_att_err_deg    = 0.0;
    double sim_time_s = 0.0;
};

// WP2: closed-loop tumble synchronization. The target tumbles torque-free
// (asymmetric inertia from cfg.target_inertia_diag, so its rate precesses);
// the servicer (isotropic inertia cfg.base_inertia) tracks it with the
// tracking SMC under cfg.sync_gains, with torque-free feedforward computed
// from the target's own (regularized) inertia. Attitude error is the
// principal rotation angle 2*acos(|q_e0|); rate error is |w - C(q_e)^T w_t|.
// Deterministic: fixed inputs, no randomness (R6).
SyncReport run_tumble_sync(const Config& cfg,
                           const Eigen::Quaterniond& q_target0,
                           const Eigen::Vector3d&    w_target0,
                           const Eigen::Quaterniond& q_servicer0,
                           const Eigen::Vector3d&    w_servicer0,
                           double max_sim_time_s);

// Outcome of an estimate-driven synchronization run (WP4). `sync` carries the
// WP2 criteria evaluated on the TRUTH state (the honest acceptance); the rest
// are estimation-error and filter-consistency statistics. Truth never enters
// the control path — the controller consumes an EstimatedState only.
struct EstimatedSyncReport {
    SyncReport sync;

    // Estimation-error statistics (truth minus estimate). RMS over the stats
    // window (t >= est_stats_start_s); finals at end of run.
    double rms_att_own_deg = 0.0;
    double rms_att_rel_deg = 0.0;
    double rms_wt_deg_s    = 0.0;
    double rms_pos_m       = 0.0;
    double rms_vel_m_s     = 0.0;
    double final_att_rel_deg = 0.0;
    double final_wt_deg_s    = 0.0;
    double final_pos_m       = 0.0;

    // Filter-consistency statistics over the stats window: time-averaged NEES
    // (state dims 6/3/6) and NIS (measurement dofs 4/3/3) with sample counts.
    double nees_trans_mean = 0.0;
    double nees_own_mean   = 0.0;
    double nees_rel_mean   = 0.0;
    double nis_trans_mean  = 0.0;
    double nis_st_mean     = 0.0;
    double nis_vis_mean    = 0.0;
    int    n_trans = 0;
    int    n_st    = 0;
    int    n_vis   = 0;

    // Covariance health over the WHOLE run, across all three filters.
    double p_min_eig  = 0.0;   // minimum eigenvalue observed (must stay > 0)
    double p_max_asym = 0.0;   // max |P - P^T| entry observed (Joseph + symm.)
};

// WP4: closed-loop tumble synchronization driven by ESTIMATES under sensor
// noise. Truth propagates the servicer/target/translation; a fixed-seed
// GaussianSource generates all sensor noise, matched truth process noise and
// initial estimate errors (R6, bit-stable across platforms); the tracking
// controller consumes only the EstimatedState. The translation EKF runs
// alongside on x_trans0 (a coasting relative orbit) — estimated and
// consistency-tested, but not used for control (no translation guidance yet).
EstimatedSyncReport run_estimated_sync(const Config& cfg,
                                       const Eigen::Quaterniond& q_target0,
                                       const Eigen::Vector3d&    w_target0,
                                       const Eigen::Quaterniond& q_servicer0,
                                       const Vector6d&           x_trans0,
                                       double max_sim_time_s);

// Outcome of a post-capture stabilization run.
struct StabilizationReport {
    bool   aborted        = false;   // closing speed exceeded the cap
    bool   captured       = false;
    bool   settled        = false;   // rate driven below the settle threshold
    bool   thermal_ok     = true;    // PCM did not saturate during the run
    double settle_time_s  = 0.0;     // time to reach the rate threshold
    double final_rate     = 0.0;     // |w| at end of run [rad/s]
    double sim_time_s      = 0.0;
    double mass_total_kg  = 0.0;
    double inertia_trace  = 0.0;
};

// WP3: the installer mission runs as a sequence of phases. Contact only happens
// after synchronization holds for the dwell; the servicer never tows (D1) — it
// installs a passive deorbit kit and departs.
enum class Phase { Approach, Sync, Attach, Depart, Complete, Aborted };

// Outcome of the attach (clamp) event (WP3).
struct AttachReport {
    bool   clamped = false;
    double contact_speed_m_s = 0.0;
    // Contact kinetic energy 0.5 * m * v^2 at the gated closing speed, with m
    // the servicer contact mass (bus dry + kit). For the multi-tonne target the
    // reduced mass ~ servicer mass, so this is a conservative upper bound. The
    // budget exists to show a compliant, geometry-keyed clamp (D4) at
    // max_v_rel does not have the energy to shed MLI/paint and manufacture new
    // debris (contact-honesty acceptance, spec v4.1 WP3). [J]
    double contact_energy_j = 0.0;
    double servicer_mass_before_kg = 0.0;  // bus dry + kit (carries the kit)
    double servicer_mass_after_kg  = 0.0;  // bus dry (kit handed over)
    double target_mass_after_kg    = 0.0;  // target + kit
    double target_area_after_m2    = 0.0;  // installed sail area
    double target_area_over_mass   = 0.0;  // A/m after attach [m^2/kg]
};

// Outcome of a full installer mission run (WP3).
struct MissionReport {
    Phase  reached = Phase::Approach;   // furthest phase reached
    bool   approach_safe = false;       // corridor holds clear the keep-out
    double approach_closest_m = 0.0;    // guaranteed corridor min range [m]
    bool   synced = false;              // tumble sync achieved (WP2 gate)
    double sync_time_s = 0.0;
    AttachReport attach;
    bool   departed = false;            // safe departure onto a bounded orbit
    double depart_coast_min_m = 0.0;    // min range of the post-departure coast [m]
    bool   success = false;             // reached Depart with every gate passed
};

class Mission {
public:
    explicit Mission(const Config& cfg = {});

    // Safe-abort impulse (WP1 + F1): a Clohessy-Wiltshire delta-v toward a
    // drift-free natural-motion relative orbit (bounded "safety ellipse")
    // through the current position. The impulse magnitude is capped at
    // Config::abort_dv; when the cap binds the result is flagged Capped and
    // the bounded-orbit property no longer holds. In both cases the returned
    // coast_min_range_m is the verified minimum range of the post-burn coast.
    SafeAbort compute_safe_abort(const Eigen::Vector3d& r_rel,
                                 const Eigen::Vector3d& v_rel) const;

    // Point-mass parallel-axis inertia contribution from a captured body at
    // r_attach, applied to the vehicle inertia (with regularization).
    void update_inertia_on_capture(const Eigen::Vector3d& r_attach,
                                   double debris_mass);

    // WP3 installer mission: approach -> sync -> attach -> depart. The servicer
    // verifies its passively-safe approach corridor, synchronizes with the
    // tumbling target (WP2 gate), clamps and installs the deorbit kit (losing
    // the kit mass; the target gains kit mass and sail area), then departs onto
    // a bounded relative orbit. Deterministic (R6); the target orbit for the
    // GNC phases is the configured reference orbit (cfg.target_altitude_km),
    // while target_mass_kg is the catalog target's mass for the attach transfer.
    MissionReport run_installer_mission(double target_mass_kg,
                                        const Eigen::Quaterniond& q_target0,
                                        const Eigen::Vector3d&    w_target0,
                                        const Eigen::Quaterniond& q_servicer0,
                                        double max_sync_time_s = 120.0);

    // Legacy v2 detumble demonstration, retained as a regression reference (its
    // settle time is a quoted number, spec section 2): closes the loop after a
    // capture and reports the settle time / final rate, or aborts if the closing
    // speed exceeds the cap. NOTE: the installer paradigm (run_installer_mission)
    // does NOT detumble the multi-tonne target — it synchronizes with the tumble
    // and installs a kit. This entry point stays for the regression pin only.
    StabilizationReport post_capture_stabilization(bool captured,
                                                   double debris_mass,
                                                   const Eigen::Vector3d& r_attach,
                                                   const Eigen::Vector3d& r_rel,
                                                   const Eigen::Vector3d& v_rel,
                                                   const Eigen::Vector3d& initial_rate,
                                                   double max_sim_time_s = 120.0);

    // Returns true if a deorbit burn is permitted. Autonomous by default;
    // human approval only gates the emergency path.
    bool deorbit_permitted(bool ground_human_approval, bool& out_autonomous);

    double fuel_kg();
    const Config& config() const { return cfg_; }

private:
    Config      cfg_;
    CwModel     cw_;    // relative-motion model for the target orbit (WP1)
    FuelStore   fuel_;
    RigidBody   body_;
    SlidingModeController ctrl_;
    double      mass_total_kg_;
};

}  // namespace adsc
