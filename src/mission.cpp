#include "adsc/mission.hpp"

#include <algorithm>
#include <cmath>

namespace adsc {

SyncReport run_tumble_sync(const Config& cfg,
                           const Eigen::Quaterniond& q_target0,
                           const Eigen::Vector3d&    w_target0,
                           const Eigen::Quaterniond& q_servicer0,
                           const Eigen::Vector3d&    w_servicer0,
                           double max_sim_time_s) {
    SyncReport rep;

    RigidBody target(cfg.target_inertia_diag.asDiagonal(), q_target0, w_target0);
    RigidBody servicer(Eigen::Matrix3d::Identity() * cfg.base_inertia,
                       q_servicer0, w_servicer0);
    const SlidingModeController ctrl(cfg.sync_gains);

    // Torque-free feedforward uses the same (regularized) inertia the target
    // actually propagates with, so the model and the plant cannot disagree.
    const Eigen::Matrix3d I_t     = target.inertia();
    const Eigen::Matrix3d I_t_inv = I_t.inverse();

    const double rad2deg = 180.0 / kPi;

    double criteria_start = -1.0;  // window start; -1 = criteria not held
    double max_rate = 0.0, max_att = 0.0;
    double rate_err_deg_s = 0.0, att_err_deg = 0.0;

    double t = 0.0;
    while (t < max_sim_time_s) {
        const Eigen::Vector3d w_t = target.rate();
        const Eigen::Vector3d w_t_dot = I_t_inv * (-(w_t.cross(I_t * w_t)));

        const Eigen::Vector3d tau = ctrl.torque(
            servicer.inertia(), servicer.attitude(), servicer.rate(),
            target.attitude(), w_t, w_t_dot);

        servicer.step(tau, cfg.control_dt);
        target.step(Eigen::Vector3d::Zero(), cfg.control_dt);
        t += cfg.control_dt;

        // Errors: principal rotation angle and relative body rate.
        const Eigen::Quaterniond q_e =
            (target.attitude().conjugate() * servicer.attitude()).normalized();
        const Eigen::Vector3d w_rel =
            servicer.rate() - q_e.conjugate() * target.rate();
        rate_err_deg_s = w_rel.norm() * rad2deg;
        att_err_deg = 2.0 * std::acos(std::min(1.0, std::abs(q_e.w()))) * rad2deg;

        const bool ok = rate_err_deg_s < cfg.sync_rate_tol_deg_s &&
                        att_err_deg   < cfg.sync_att_tol_deg;
        if (!rep.synced) {
            if (ok) {
                if (criteria_start < 0.0) criteria_start = t;
            } else {
                criteria_start = -1.0;  // window broken before the dwell
            }
        }
        if (!rep.synced && criteria_start >= 0.0 &&
            t - criteria_start >= cfg.sync_hold_s) {
            rep.synced      = true;
            rep.sync_time_s = criteria_start;
        }
        // Post-dwell hold quality: every sample inside the dwell already
        // satisfied the criteria (or the window would have reset), so the
        // informative maximum is the one AFTER sync is declared.
        if (rep.synced) {
            max_rate = std::max(max_rate, rate_err_deg_s);
            max_att  = std::max(max_att,  att_err_deg);
        }
    }

    rep.max_rate_err_deg_s   = max_rate;
    rep.max_att_err_deg      = max_att;
    rep.final_rate_err_deg_s = rate_err_deg_s;
    rep.final_att_err_deg    = att_err_deg;
    rep.sim_time_s           = t;
    return rep;
}

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
