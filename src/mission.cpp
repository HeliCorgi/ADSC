#include "adsc/mission.hpp"

#include <algorithm>

namespace adsc {

Mission::Mission(const Config& cfg)
    : cfg_(cfg),
      cw_(CwModel::from_orbit(kEarthRadius + cfg.target_altitude_km * 1000.0)),
      fuel_(cfg.initial_fuel_kg),
      body_(Eigen::Matrix3d::Identity() * cfg.base_inertia,
            Eigen::Quaterniond::Identity(),
            Eigen::Vector3d::Zero()),
      mass_total_kg_(cfg.dry_mass_kg) {}

double Mission::fuel_kg() {
    return fuel_.read().value;
}

SafeAbort Mission::compute_safe_abort(const Eigen::Vector3d& r_rel,
                                      const Eigen::Vector3d& v_rel) const {
    // Clohessy-Wiltshire abort impulse (WP1). We null the secular along-track
    // drift (target vy = -2 n x) and the radial/cross-track rates that would
    // otherwise inflate the relative orbit, leaving a bounded, drift-free
    // "safety ellipse" through the current position: a thrust-off coast then
    // revisits the current standoff instead of closing in.
    SafeAbort out;
    const double n = cw_.n();
    const Eigen::Vector3d v_target(0.0, -2.0 * n * r_rel.x(), 0.0);
    Eigen::Vector3d dv = v_target - v_rel;

    // F1: cap at the thruster budget. A capped impulse leaves residual drift,
    // so the bounded-orbit guarantee is lost and we say so.
    const double dv_max = cfg_.abort_dv;  // impulse-magnitude cap [m/s]
    const double mag = dv.norm();
    if (mag > dv_max && mag > 1e-12) {
        dv = (dv_max / mag) * dv;
        out.status = SafeAbort::Status::Capped;
    }
    out.dv = dv;

    // F1: verify the actual post-burn coast instead of implying safety. Clean
    // or capped, the honest product is the propagated minimum range.
    Vector6d x;
    x << r_rel, (v_rel + dv);
    double min_range = rel_range(x);
    const double horizon = cfg_.abort_coast_check_periods * cw_.period();
    const double dt = cfg_.abort_coast_check_dt_s;
    double t = 0.0;
    while (t < horizon) {
        x = cw_.propagate_rk4(x, dt, dt);
        min_range = std::min(min_range, rel_range(x));
        t += dt;
    }
    out.coast_min_range_m = min_range;
    return out;
}

void Mission::update_inertia_on_capture(const Eigen::Vector3d& r_attach,
                                        double debris_mass) {
    const double r2 = r_attach.squaredNorm();
    Eigen::Matrix3d add =
        debris_mass * (r2 * Eigen::Matrix3d::Identity()
                       - r_attach * r_attach.transpose());
    body_.set_inertia(body_.inertia() + add, cfg_.regularization_eps);
}

StabilizationReport Mission::post_capture_stabilization(
        bool captured,
        double debris_mass,
        const Eigen::Vector3d& r_attach,
        const Eigen::Vector3d& r_rel,
        const Eigen::Vector3d& v_rel,
        const Eigen::Vector3d& initial_rate,
        double max_sim_time_s) {
    StabilizationReport rep;
    rep.mass_total_kg = mass_total_kg_;
    rep.inertia_trace = body_.inertia().trace();

    // Closing-speed gate: too fast -> abort, do not capture.
    if (v_rel.norm() > cfg_.max_v_rel) {
        rep.aborted = true;
        (void)compute_safe_abort(r_rel, v_rel);  // impulse would be commanded here
        return rep;
    }
    if (!captured || debris_mass <= 0.0) return rep;

    // Commit the capture: grow mass and inertia, seed the tumble.
    mass_total_kg_ += debris_mass;
    update_inertia_on_capture(r_attach, debris_mass);
    body_ = RigidBody(body_.inertia(), Eigen::Quaterniond::Identity(), initial_rate);

    rep.captured      = true;
    rep.mass_total_kg = mass_total_kg_;
    rep.inertia_trace = body_.inertia().trace();

    ThermalPCM pcm(cfg_.pcm_capacity_j, cfg_.max_safe_time_s);
    const Eigen::Quaterniond target = Eigen::Quaterniond::Identity();
    const double settle_rate = 1e-3;  // |w| threshold for "settled" [rad/s]

    double t = 0.0;
    while (t < max_sim_time_s) {
        Eigen::Vector3d tau =
            ctrl_.torque(body_.inertia(), body_.attitude(), body_.rate(), target);
        body_.step(tau, cfg_.control_dt);

        // Heat: steady avionics load plus a crude term for thruster activity.
        double firing_power = tau.cwiseAbs().sum() * 5.0;  // illustrative scale
        if (pcm.absorb(cfg_.avionics_power_w + firing_power, cfg_.control_dt)) {
            rep.thermal_ok = false;
            break;
        }

        t += cfg_.control_dt;
        if (!rep.settled && body_.rate().norm() < settle_rate) {
            rep.settled       = true;
            rep.settle_time_s = t;
        }
    }

    rep.sim_time_s  = t;
    rep.final_rate  = body_.rate().norm();
    return rep;
}

bool Mission::deorbit_permitted(bool ground_human_approval, bool& out_autonomous) {
    if (fuel_kg() < cfg_.deorbit_reserve_kg) {
        out_autonomous = false;
        return false;  // insufficient reserve
    }
    // Autonomous unless a human is explicitly in the loop for the emergency path.
    out_autonomous = !ground_human_approval;
    return true;
}

}  // namespace adsc
