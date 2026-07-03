#include "adsc/mission.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace adsc {

Eigen::Vector3d ActuatorError::apply(const Eigen::Vector3d& tau) const {
    // Scale each axis, then rotate by the small misalignment. The neutral
    // default (scale = 0, misalign = 0) is exactly the identity: cwiseProduct
    // with Ones() reproduces tau bit-for-bit and quat_exp(0) is the identity
    // quaternion, so the delegating 6-arg run_tumble_sync stays byte-identical.
    const Eigen::Vector3d scaled =
        tau.cwiseProduct(Eigen::Vector3d::Ones() + scale);
    return quat_exp(misalign_rad) * scaled;
}

SyncReport run_tumble_sync(const Config& cfg,
                           const Eigen::Quaterniond& q_target0,
                           const Eigen::Vector3d&    w_target0,
                           const Eigen::Quaterniond& q_servicer0,
                           const Eigen::Vector3d&    w_servicer0,
                           double max_sim_time_s) {
    return run_tumble_sync(cfg, q_target0, w_target0, q_servicer0, w_servicer0,
                           max_sim_time_s, ActuatorError{});
}

SyncReport run_tumble_sync(const Config& cfg,
                           const Eigen::Quaterniond& q_target0,
                           const Eigen::Vector3d&    w_target0,
                           const Eigen::Quaterniond& q_servicer0,
                           const Eigen::Vector3d&    w_servicer0,
                           double max_sim_time_s,
                           const ActuatorError& actuator) {
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

        // Actuator dispersion (WP5): delivered torque = actuator.apply(tau).
        // Neutral default => tau, so the pinned WP2 run is byte-identical.
        servicer.step(actuator.apply(tau), cfg.control_dt);
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

EstimatedSyncReport run_estimated_sync(const Config& cfg,
                                       const Eigen::Quaterniond& q_target0,
                                       const Eigen::Vector3d&    w_target0,
                                       const Eigen::Quaterniond& q_servicer0,
                                       const Vector6d&           x_trans0,
                                       double max_sim_time_s) {
    EstimatedSyncReport rep;
    const double dt = cfg.control_dt;
    const double rad2deg = 180.0 / kPi;

    // One fixed-seed Gaussian source drives EVERYTHING random: sensor noise,
    // matched truth process noise, and sampled initial estimate errors (R6).
    GaussianSource rng(cfg.est_seed);

    // ------------------------------------------------------------------
    // TRUTH world. Read only by the sensor models below and by the
    // error-statistics / acceptance-criteria recorder at the bottom of the
    // loop. It is never handed to the controller.
    // ------------------------------------------------------------------
    RigidBody truth_target(cfg.target_inertia_diag.asDiagonal(), q_target0,
                           w_target0);
    RigidBody truth_servicer(Eigen::Matrix3d::Identity() * cfg.base_inertia,
                             q_servicer0, Eigen::Vector3d::Zero());
    Vector6d truth_trans = x_trans0;

    const CwModel cw =
        CwModel::from_orbit(kEarthRadius + cfg.target_altitude_km * 1000.0);
    const Eigen::Matrix3d I_t = truth_target.inertia();
    const Eigen::Matrix3d I_t_inv = I_t.inverse();
    const Eigen::Matrix3d I_servicer =
        Eigen::Matrix3d::Identity() * cfg.base_inertia;

    // ------------------------------------------------------------------
    // Filters, initialized truth + sampled error ~ N(0, P0) so the
    // consistency statistics are meaningful from the first step.
    // ------------------------------------------------------------------
    const double s_att = cfg.est_init_att_sigma_rad;
    const double s_wt  = cfg.est_init_wt_sigma_rad_s;

    Eigen::Matrix<double, 6, 6> P0_trans = Eigen::Matrix<double, 6, 6>::Zero();
    P0_trans.block<3, 3>(0, 0) = cfg.est_init_pos_sigma_m *
                                 cfg.est_init_pos_sigma_m *
                                 Eigen::Matrix3d::Identity();
    P0_trans.block<3, 3>(3, 3) = cfg.est_init_vel_sigma_m_s *
                                 cfg.est_init_vel_sigma_m_s *
                                 Eigen::Matrix3d::Identity();
    Vector6d x0_est = truth_trans;
    x0_est.head<3>() += rng.sample3(cfg.est_init_pos_sigma_m);
    x0_est.tail<3>() += rng.sample3(cfg.est_init_vel_sigma_m_s);
    TranslationEkf ekf(cw, x0_est, P0_trans);

    const Eigen::Quaterniond q_rel_true0 =
        (q_target0.conjugate() * q_servicer0).normalized();
    Eigen::Matrix<double, 6, 6> P0_rel = Eigen::Matrix<double, 6, 6>::Zero();
    P0_rel.block<3, 3>(0, 0) = s_att * s_att * Eigen::Matrix3d::Identity();
    P0_rel.block<3, 3>(3, 3) = s_wt * s_wt * Eigen::Matrix3d::Identity();
    // The filter's "known" target inertia is the same (regularized) tensor the
    // truth target actually propagates with — the known-inertia assumption is
    // exact in this simulation and documented as a limitation in the README.
    // Initial-error draws are hoisted into named locals: function-argument
    // evaluation order is unspecified, and the RNG draw order must be fixed
    // for the run to be reproducible across compilers (R6).
    const Eigen::Vector3d e_own0 = rng.sample3(s_att);
    const Eigen::Vector3d e_rel0 = rng.sample3(s_att);
    const Eigen::Vector3d e_wt0  = rng.sample3(s_wt);
    AttitudeMekf mekf((q_servicer0 * quat_exp(e_own0)).normalized(),
                      s_att * s_att * Eigen::Matrix3d::Identity(),
                      (q_rel_true0 * quat_exp(e_rel0)).normalized(),
                      w_target0 + e_wt0, P0_rel, I_t);

    const SlidingModeController ctrl(cfg.sync_gains);

    // Measurement cadences in control steps.
    const int st_every  = static_cast<int>(1.0 / (cfg.st_rate_hz * dt) + 0.5);
    const int vis_every = static_cast<int>(1.0 / (cfg.vision_rate_hz * dt) + 0.5);
    const int rng_every = static_cast<int>(1.0 / (cfg.ranging_rate_hz * dt) + 0.5);
    const double dt_ranging = rng_every * dt;

    // Accumulators.
    double criteria_start = -1.0;
    double max_rate = 0.0, max_att = 0.0;
    double rate_err_deg_s = 0.0, att_err_deg = 0.0;
    double sum_own2 = 0.0, sum_rel2 = 0.0, sum_wt2 = 0.0;
    long   n_att = 0;
    double sum_pos2 = 0.0, sum_vel2 = 0.0;
    long   n_pos = 0;
    double nees_t = 0.0, nees_o = 0.0, nees_r = 0.0;
    double nis_t = 0.0, nis_s = 0.0, nis_v = 0.0;
    double p_min = std::numeric_limits<double>::max();
    double p_asym = 0.0;

    const auto track_p = [&](const Eigen::MatrixXd& P) {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
        p_min = std::min(p_min, es.eigenvalues().minCoeff());
        p_asym = std::max(p_asym,
                          (P - P.transpose()).cwiseAbs().maxCoeff());
    };

    const int steps = static_cast<int>(max_sim_time_s / dt + 0.5);
    double t = 0.0;
    for (int k = 0; k < steps; ++k) {
        const bool in_window = t >= cfg.est_stats_start_s;

        // --- Sensor sampling + measurement updates at time t --------------
        if (k % st_every == 0) {
            const Eigen::Quaterniond q_st =
                (truth_servicer.attitude() *
                 quat_exp(rng.sample3(cfg.st_sigma_rad))).normalized();
            const double nis = mekf.update_star_tracker(q_st, cfg.st_sigma_rad);
            track_p(mekf.P_own());
            if (in_window) {
                nis_s += nis;
                const Eigen::Vector3d e =
                    quat_residual(mekf.q_own(), truth_servicer.attitude());
                nees_o += e.dot(mekf.P_own().ldlt().solve(e));
                ++rep.n_st;
            }
        }
        if (k % vis_every == 0) {
            const Eigen::Quaterniond q_rel_true =
                (truth_target.attitude().conjugate() *
                 truth_servicer.attitude()).normalized();
            const Eigen::Quaterniond q_vis =
                (q_rel_true * quat_exp(rng.sample3(cfg.vision_sigma_rad)))
                    .normalized();
            const double nis = mekf.update_vision(q_vis, cfg.vision_sigma_rad);
            track_p(mekf.P_rel());
            if (in_window) {
                nis_v += nis;
                // Error in the filter's own convention (truth "minus" estimate
                // for BOTH blocks) — mixing signs across blocks would flip the
                // cross-covariance terms and corrupt the NEES.
                Vector6d e;
                e.head<3>() = quat_residual(mekf.q_rel(), q_rel_true);
                e.tail<3>() = truth_target.rate() - mekf.w_target();
                nees_r += e.dot(mekf.P_rel().ldlt().solve(e));
                ++rep.n_vis;
            }
        }
        if (k % rng_every == 0) {
            if (k > 0) {
                // Truth coast over one ranging interval with matched
                // velocity process noise, then the filter prediction.
                truth_trans = cw.propagate(truth_trans, dt_ranging);
                truth_trans.tail<3>() += rng.sample3(cfg.trans_vel_noise_m_s);
                ekf.predict(dt_ranging, cfg.trans_vel_noise_m_s);
            }
            const Eigen::Vector3d r_true = truth_trans.head<3>();
            const double z_range =
                r_true.norm() + cfg.range_bias_m + cfg.range_sigma_m * rng.sample();
            const Eigen::Vector3d z_los =
                r_true.normalized() +
                Eigen::Vector3d::Constant(cfg.los_bias) +
                rng.sample3(cfg.los_sigma);
            const double nis =
                ekf.update(z_range, z_los, cfg.range_sigma_m, cfg.los_sigma);
            track_p(ekf.covariance());
            if (in_window) {
                nis_t += nis;
                const Vector6d e = ekf.state() - truth_trans;
                nees_t += e.dot(ekf.covariance().ldlt().solve(e));
                ++rep.n_trans;
                sum_pos2 += (ekf.state().head<3>() - r_true).squaredNorm();
                sum_vel2 +=
                    (ekf.state().tail<3>() - truth_trans.tail<3>()).squaredNorm();
                ++n_pos;
            }
        }

        // --- Control from ESTIMATES ONLY -----------------------------------
        // The gyro reading is both the controller's rate input and the
        // filter's propagation input (one sample per step).
        const Eigen::Vector3d w_g =
            truth_servicer.rate() +
            Eigen::Vector3d::Constant(cfg.gyro_bias_rad_s) +
            rng.sample3(cfg.gyro_sigma_rad_s);

        EstimatedState est;
        est.q_own    = mekf.q_own();
        est.w_own    = w_g;
        est.q_target = (mekf.q_own() * mekf.q_rel().conjugate()).normalized();
        est.w_target = mekf.w_target();
        est.w_target_dot =
            I_t_inv * (-(mekf.w_target().cross(I_t * mekf.w_target())));
        est.rel_translation = ekf.state();

        const Eigen::Vector3d tau =
            ctrl.torque(I_servicer, est.q_own, est.w_own, est.q_target,
                        est.w_target, est.w_target_dot);

        // --- Truth propagation over [t, t+dt] ------------------------------
        truth_servicer.step(tau, dt);
        truth_target.step(I_t * rng.sample3(cfg.wt_disturb_sigma), dt);
        mekf.predict(w_g, dt, cfg.gyro_sigma_rad_s, cfg.wt_disturb_sigma);
        t += dt;

        // --- Truth-evaluated criteria + estimation-error statistics --------
        const Eigen::Quaterniond q_e =
            (truth_target.attitude().conjugate() * truth_servicer.attitude())
                .normalized();
        const Eigen::Vector3d w_rel =
            truth_servicer.rate() - q_e.conjugate() * truth_target.rate();
        rate_err_deg_s = w_rel.norm() * rad2deg;
        att_err_deg =
            2.0 * std::acos(std::min(1.0, std::abs(q_e.w()))) * rad2deg;

        const bool ok = rate_err_deg_s < cfg.sync_rate_tol_deg_s &&
                        att_err_deg   < cfg.sync_att_tol_deg;
        if (!rep.sync.synced) {
            if (ok) {
                if (criteria_start < 0.0) criteria_start = t;
            } else {
                criteria_start = -1.0;
            }
        }
        if (!rep.sync.synced && criteria_start >= 0.0 &&
            t - criteria_start >= cfg.sync_hold_s) {
            rep.sync.synced      = true;
            rep.sync.sync_time_s = criteria_start;
        }
        if (rep.sync.synced) {
            max_rate = std::max(max_rate, rate_err_deg_s);
            max_att  = std::max(max_att,  att_err_deg);
        }

        if (t >= cfg.est_stats_start_s) {
            const double e_own =
                quat_residual(mekf.q_own(), truth_servicer.attitude()).norm();
            const Eigen::Quaterniond q_rel_true =
                (truth_target.attitude().conjugate() *
                 truth_servicer.attitude()).normalized();
            const double e_rel =
                quat_residual(mekf.q_rel(), q_rel_true).norm();
            const double e_wt =
                (mekf.w_target() - truth_target.rate()).norm();
            sum_own2 += e_own * e_own;
            sum_rel2 += e_rel * e_rel;
            sum_wt2  += e_wt * e_wt;
            ++n_att;
            rep.final_att_rel_deg = e_rel * rad2deg;
            rep.final_wt_deg_s    = e_wt * rad2deg;
        }
    }

    rep.sync.max_rate_err_deg_s   = max_rate;
    rep.sync.max_att_err_deg      = max_att;
    rep.sync.final_rate_err_deg_s = rate_err_deg_s;
    rep.sync.final_att_err_deg    = att_err_deg;
    rep.sync.sim_time_s           = t;

    if (n_att > 0) {
        rep.rms_att_own_deg = std::sqrt(sum_own2 / n_att) * rad2deg;
        rep.rms_att_rel_deg = std::sqrt(sum_rel2 / n_att) * rad2deg;
        rep.rms_wt_deg_s    = std::sqrt(sum_wt2 / n_att) * rad2deg;
    }
    if (n_pos > 0) {
        rep.rms_pos_m   = std::sqrt(sum_pos2 / n_pos);
        rep.rms_vel_m_s = std::sqrt(sum_vel2 / n_pos);
        rep.final_pos_m = (ekf.state().head<3>() - truth_trans.head<3>()).norm();
    }
    if (rep.n_trans > 0) {
        rep.nees_trans_mean = nees_t / rep.n_trans;
        rep.nis_trans_mean  = nis_t / rep.n_trans;
    }
    if (rep.n_st > 0) {
        rep.nees_own_mean = nees_o / rep.n_st;
        rep.nis_st_mean   = nis_s / rep.n_st;
    }
    if (rep.n_vis > 0) {
        rep.nees_rel_mean = nees_r / rep.n_vis;
        rep.nis_vis_mean  = nis_v / rep.n_vis;
    }
    rep.p_min_eig  = p_min;
    rep.p_max_asym = p_asym;
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

MissionReport Mission::run_installer_mission(double target_mass_kg,
                                             const Eigen::Quaterniond& q_target0,
                                             const Eigen::Vector3d&    w_target0,
                                             const Eigen::Quaterniond& q_servicer0,
                                             double max_sync_time_s) {
    MissionReport rep;

    // --- Approach: the passively-safe corridor must clear the keep-out. The
    //     per-hold guaranteed minimum range bounds every thrust-off coast (the
    //     full coast verification is WP1's job, pinned in test_relmotion).
    const std::vector<SafetyEllipse> corridor =
        approach_corridor(cfg_.approach_rho_far_m, cfg_.approach_rho_near_m,
                          cfg_.keep_out_radius_m, cfg_.approach_holds);
    double closest = std::numeric_limits<double>::max();
    for (const SafetyEllipse& hold : corridor) {
        closest = std::min(closest, hold.min_range());
    }
    rep.approach_closest_m = corridor.empty() ? 0.0 : closest;
    rep.approach_safe = !corridor.empty() && closest > cfg_.keep_out_radius_m;
    rep.reached = Phase::Approach;
    if (!rep.approach_safe) {
        rep.reached = Phase::Aborted;
        return rep;
    }

    // --- Sync: gate on tumble synchronization (WP2). ---
    const SyncReport sync = run_tumble_sync(
        cfg_, q_target0, w_target0, q_servicer0, Eigen::Vector3d::Zero(),
        max_sync_time_s);
    rep.synced      = sync.synced;
    rep.sync_time_s = sync.sync_time_s;
    rep.reached     = Phase::Sync;
    if (!rep.synced) {
        rep.reached = Phase::Aborted;
        return rep;
    }

    // --- Attach: clamp at the gated closing speed and hand over the kit. The
    //     servicer loses the kit mass; the target gains kit mass and sail area
    //     (its A/m changes), which feeds the WP3 decay trades.
    AttachReport& at = rep.attach;
    at.clamped            = true;
    at.contact_speed_m_s  = cfg_.max_v_rel;
    const double m_contact = cfg_.dry_mass_kg + cfg_.kit_mass_kg;
    at.contact_energy_j        = 0.5 * m_contact * cfg_.max_v_rel * cfg_.max_v_rel;
    at.servicer_mass_before_kg = m_contact;
    at.servicer_mass_after_kg  = cfg_.dry_mass_kg;
    at.target_mass_after_kg    = target_mass_kg + cfg_.kit_mass_kg;
    at.target_area_after_m2    = cfg_.kit_sail_area_m2;
    at.target_area_over_mass   = cfg_.kit_sail_area_m2 / at.target_mass_after_kg;
    rep.reached = Phase::Attach;

    // --- Depart: transfer to a bounded, keep-out-clearing relative orbit
    //     (a safe standoff hold at depart_standoff_factor x the keep-out). ---
    const Eigen::Vector3d r_depart(
        0.0, -cfg_.depart_standoff_factor * cfg_.keep_out_radius_m, 0.0);
    const SafeAbort ab = compute_safe_abort(r_depart, Eigen::Vector3d::Zero());
    rep.depart_coast_min_m = ab.coast_min_range_m;
    rep.departed = ab.coast_min_range_m > cfg_.keep_out_radius_m;
    rep.reached  = rep.departed ? Phase::Complete : Phase::Depart;
    rep.success  = rep.approach_safe && rep.synced && at.clamped && rep.departed;
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
