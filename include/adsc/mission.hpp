#pragma once

#include <Eigen/Dense>

#include "adsc/controller.hpp"
#include "adsc/dynamics.hpp"
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

    // Runs the closed-loop detumble/hold after a capture attempt and reports
    // what actually happened. If the closing speed is over the cap, it aborts
    // instead of capturing.
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
