#include "adsc/guidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "adsc/estimator.hpp"

namespace adsc {

namespace {

// Reuses the campaign's coarsened keep-out-screen step as the range-tracking
// sample resolution for the min-clearance metric (main_campaign.cpp uses the
// same 8 s figure for its own keep-out sweep).
constexpr double kRangeSampleDtS = 8.0;

// Fixed dwell at the departure standoff on top of Config::sync_hold_s: WP2's
// tracking SMC declares sync well inside sync_hold_s (nominal ~17 s demo,
// spec section 2), but the campaign's own sync convergence budget
// (CampaignConfig::sync_time_budget_s) is 75 s; this local constant mirrors
// that same allowance so the L0 guidance demo dwells at least as long as the
// real WP2 sync loop would need, without pulling in campaign.hpp for a
// translation-guidance detail. Attitude sync itself is WP2's job and is not
// re-simulated here (no attitude dynamics in this L0 demo).
constexpr double kSyncAllowanceS = 75.0;

// WP11 two-point boundary velocity solve for a CW coast of duration tau: the
// velocity immediately after r0 that arrives at r_target after tau seconds
// (Phi_rv^{-1} (r_target - Phi_rr r0)), built from the same stm() the rest of
// WP1 already uses. tau = guid_transfer_frac_T * period (quarter-period
// scale) is far from the Phi_rv singularities at multiples of T/2; the
// shorter final-approach steps stay well inside that margin too.
Eigen::Vector3d transfer_velocity(const CwModel& cw, double tau,
                                  const Eigen::Vector3d& r0,
                                  const Eigen::Vector3d& r_target) {
    const Matrix6d phi = cw.stm(tau);
    const Eigen::Matrix3d Phi_rr = phi.block<3, 3>(0, 0);
    const Eigen::Matrix3d Phi_rv = phi.block<3, 3>(0, 3);
    const Eigen::Vector3d rhs = r_target - Phi_rr * r0;
    return Phi_rv.fullPivLu().solve(rhs);
}

// WP11 reachability screen (the safe-set rule): evaluate the clearing-abort
// law on a CANDIDATE post-impulse state (r unchanged by the impulse, v_post
// the velocity right after it) and decide whether the impulse may be
// committed.
//
// OUTSIDE the keep-out sphere the post-impulse state must be able to reach a
// bounded, margin-clearing abort: Clean or BoundedClearing (single impulse),
// or RetreatHop (WP11's two-impulse hop carries a genuine TERMINAL bounded
// guarantee too -- unlike the rejected pure along-track opening-drift design
// -- so it is accepted on equal footing: bounded_min_range_m >= keep_out +
// margin). Without accepting RetreatHop here, an unsatisfiable band would
// exist for on-axis (x0 ~ 0) states with keep_out < range <= keep_out +
// margin: Stage 1 needs |y0| >= keep_out + margin exactly there, and Stage 2
// is deliberately gated off at |y0| <= keep_out + margin (mission.cpp), so
// nothing outside RetreatHop could ever clear that band.
//
// INSIDE the sphere (final approach / contact, the authorized exception to
// the keep-out gate, D5/WP11: the sphere gates UNAUTHORIZED flight) the
// screen is retreat-feasibility instead of clearance: an abort must still
// carry a bounded terminal guarantee, AND its transient coast must not come
// significantly closer than the range at which it is evaluated
// (coast_min_range_m >= guid_inside_floor_frac * range).
bool reachability_screen_ok(const Config& cfg, const CwModel& cw,
                            const Eigen::Vector3d& r,
                            const Eigen::Vector3d& v_post,
                            SafeAbort* out_ab) {
    const SafeAbort ab = clearing_abort_for(cfg, cw, r, v_post);
    if (out_ab) *out_ab = ab;
    const double R_clear = cfg.keep_out_radius_m + cfg.abort_clear_margin_m;
    const bool bounded_ok =
        (ab.status == SafeAbort::Status::Clean ||
         ab.status == SafeAbort::Status::BoundedClearing ||
         ab.status == SafeAbort::Status::RetreatHop) &&
        ab.bounded_min_range_m >= R_clear;
    const double range = r.norm();
    if (range > cfg.keep_out_radius_m) {
        return bounded_ok;
    }
    return bounded_ok && ab.coast_min_range_m >= cfg.guid_inside_floor_frac * range;
}

}  // namespace

const char* guidance_mode_label(GuidanceMode m) {
    switch (m) {
        case GuidanceMode::FarApproach:   return "far_approach";
        case GuidanceMode::Hold:          return "hold";
        case GuidanceMode::SyncHold:      return "sync_hold";
        case GuidanceMode::FinalApproach: return "final_approach";
        case GuidanceMode::Contact:       return "contact";
        case GuidanceMode::Retreat:       return "retreat";
        case GuidanceMode::Complete:      return "complete";
        case GuidanceMode::Abort:         return "abort";
    }
    return "unknown";
}

std::vector<GuidanceModeSpec> guidance_mode_table() {
    return {
        {GuidanceMode::FarApproach,
         "start of the demo, at the far V-bar hold (guid_hold_far_m)",
         "two-impulse CW transfer departs toward the mid hold",
         "either impulse's post-state fails the reachability screen",
         "execute the clearing abort from the current state and enter Abort"},
        {GuidanceMode::Hold,
         "arrival at the mid V-bar hold (guid_hold_mid_m)",
         "two-impulse CW transfer departs toward the sync-hold standoff",
         "either impulse's post-state fails the reachability screen",
         "execute the clearing abort from the current state and enter Abort"},
        {GuidanceMode::SyncHold,
         "arrival at the departure standoff (depart_standoff_factor x keep_out)",
         "sync dwell (Config::sync_hold_s + a fixed sync allowance) completes",
         "n/a (zero-dv equilibrium dwell; attitude sync is WP2's job)",
         "n/a"},
        {GuidanceMode::FinalApproach,
         "sync dwell complete",
         "range <= guid_contact_range_m",
         "a glideslope step's post-state fails the reachability screen, or the LOS cone is violated",
         "execute the clearing abort from the current state and enter Abort"},
        {GuidanceMode::Contact,
         "range <= guid_contact_range_m",
         "contact-speed matching burn commits",
         "the matching burn's post-state fails the reachability screen",
         "execute the clearing abort from the current state and enter Abort"},
        {GuidanceMode::Retreat,
         "immediately after contact",
         "the standoff equilibrium is re-established (post-hop range >= standoff, or accepted in place)",
         "n/a (the RetreatHop abort IS the escalation of last resort; not itself re-screened)",
         "n/a"},
        {GuidanceMode::Complete,
         "standoff equilibrium re-established outside keep_out + margin",
         "end of the demo",
         "n/a",
         "n/a"},
        {GuidanceMode::Abort,
         "a reachability screen or LOS check fails",
         "n/a (terminal)",
         "n/a",
         "n/a (the last feasible clearing abort has already been executed)"},
    };
}

GuidedApproach::GuidedApproach(const Config& cfg)
    : cfg_(cfg),
      cw_(CwModel::from_orbit(kEarthRadius + cfg.target_altitude_km * 1000.0)) {}

GuidedApproachReport GuidedApproach::fly() {
    // WP12 L4 dispatch (R1: extends, never replaces -- everything below this
    // guard is the ORIGINAL WP11 code, unmodified; cfg_.guid_estimate_driven
    // defaults to false, so every existing caller is unaffected byte-for-byte).
    if (cfg_.guid_estimate_driven) return fly_estimate_driven();

    GuidedApproachReport rep;

    // V-bar points are CW equilibria: with x = z = 0 and zero velocity the
    // state is stationary (derivative(): x_ddot = 3n^2*0 + 2n*0 = 0,
    // y_ddot = -2n*0 = 0, z_ddot = -n^2*0 = 0), so every "hold" below is a
    // literal rest state, not merely a slowly-drifting approximation.
    Vector6d x = Vector6d::Zero();
    x(1) = -cfg_.guid_hold_far_m;

    double t_clock = 0.0;
    double dv_total = 0.0;
    double min_clearance_outside = std::numeric_limits<double>::max();
    // The clearance metric covers PRE-AUTHORIZATION flight only (see
    // GuidedApproachReport::min_clearance_outside_m): once FinalApproach
    // begins -- the authorized keep-out penetration, D5/WP11 -- descending
    // through the sphere boundary would otherwise drive the metric to ~0 by
    // construction and it would no longer measure a safety margin.
    bool   preauth = true;
    bool   abort_feasible_every_step = true;
    bool   los_cone_ok = true;
    bool   aborted = false;
    const double period = cw_.period();
    const double tau = cfg_.guid_transfer_frac_T * period;
    const double standoff = cfg_.depart_standoff_factor * cfg_.keep_out_radius_m;
    const double R_clear = cfg_.keep_out_radius_m + cfg_.abort_clear_margin_m;

    // Sample a just-departed coast (x0's state) for leg_t seconds at
    // kRangeSampleDtS resolution, tracking the minimum range seen (into
    // phase_min) and the minimum (range - keep_out) over samples OUTSIDE the
    // keep-out sphere (into the run-wide min_clearance_outside).
    const auto sample_leg = [&](const Vector6d& x0, double leg_t, double& phase_min) {
        Vector6d xs = x0;
        phase_min = std::min(phase_min, rel_range(xs));
        double tt = 0.0;
        while (tt < leg_t - 1e-9) {
            const double step = std::min(kRangeSampleDtS, leg_t - tt);
            xs = cw_.propagate(xs, step);
            tt += step;
            const double range = rel_range(xs);
            phase_min = std::min(phase_min, range);
            if (preauth && range > cfg_.keep_out_radius_m) {
                min_clearance_outside =
                    std::min(min_clearance_outside, range - cfg_.keep_out_radius_m);
            }
        }
    };

    // Evaluate the reachability screen on a candidate post-impulse state;
    // on failure, latch `aborted` so the caller stops committing impulses.
    const auto screen = [&](const Eigen::Vector3d& r, const Eigen::Vector3d& v_post) {
        const bool ok = reachability_screen_ok(cfg_, cw_, r, v_post, nullptr);
        abort_feasible_every_step = abort_feasible_every_step && ok;
        if (!ok) aborted = true;
        return ok;
    };

    // Two-impulse CW hold-to-hold hop along -V-bar: solve the departure
    // velocity that arrives at (0, y_target, 0) after tau seconds
    // (transfer_velocity), screen + commit it, coast tau, then null the
    // arrival velocity to re-establish the target hold equilibrium. Leaves
    // `aborted` set (and does not commit the failing impulse) if either
    // impulse's screen fails.
    const auto hop = [&](double y_target, double& phase_dv, double& phase_min,
                         bool& phase_ok) {
        if (aborted) return;
        const Eigen::Vector3d r0 = x.head<3>();
        const Eigen::Vector3d v0 = x.tail<3>();
        const Eigen::Vector3d r_target(0.0, y_target, 0.0);
        const Eigen::Vector3d v_req = transfer_velocity(cw_, tau, r0, r_target);
        if (!screen(r0, v_req)) { phase_ok = false; return; }
        phase_dv += (v_req - v0).norm();
        dv_total += (v_req - v0).norm();
        x.tail<3>() = v_req;  // commit BEFORE sampling: the coast runs at v_req, not v0
        sample_leg(x, tau, phase_min);
        x = cw_.propagate(x, tau);
        t_clock += tau;
        const Eigen::Vector3d v_arr = x.tail<3>();
        if (!screen(x.head<3>(), Eigen::Vector3d::Zero())) { phase_ok = false; return; }
        phase_dv += v_arr.norm();
        dv_total += v_arr.norm();
        x.tail<3>() = Eigen::Vector3d::Zero();
    };

    // On any screen failure: do not execute the failing impulse; execute the
    // clearing abort FROM THE CURRENT (already-committed) state instead, and
    // stop. Safety-logic branch the nominal L0 demo is not expected to
    // exercise.
    const auto abort_and_stop = [&]() {
        const SafeAbort ab = clearing_abort_for(cfg_, cw_, x.head<3>(), x.tail<3>());
        x.tail<3>() += ab.dv;
        const double abort_cost = ab.dv.norm() + ab.dv_second_burn_m_s;
        dv_total += abort_cost;
        if (!rep.phases.empty()) {
            rep.phases.back().dv_m_s += abort_cost;
            rep.phases.back().abort_feasible_throughout = false;
            rep.phases.back().t_end_s = t_clock;
        }
        rep.completed  = false;
        rep.final_mode = GuidanceMode::Abort;
        rep.dv_total_m_s = dv_total;
        rep.min_clearance_outside_m = min_clearance_outside;
        rep.abort_feasible_every_step = abort_feasible_every_step;
        rep.los_cone_ok = los_cone_ok;
    };

    // ---- FarApproach: depart the far hold toward the mid hold. ----
    GuidedPhaseLog far_phase;
    far_phase.mode = GuidanceMode::FarApproach;
    far_phase.t_start_s = t_clock;
    far_phase.min_range_m = rel_range(x);
    bool far_ok = true;
    hop(-cfg_.guid_hold_mid_m, far_phase.dv_m_s, far_phase.min_range_m, far_ok);
    far_phase.t_end_s = t_clock;
    far_phase.abort_feasible_throughout = far_ok;
    rep.phases.push_back(far_phase);
    if (aborted) { abort_and_stop(); return rep; }

    // ---- Hold: depart the mid hold toward the sync-hold standoff. ----
    GuidedPhaseLog hold_phase;
    hold_phase.mode = GuidanceMode::Hold;
    hold_phase.t_start_s = t_clock;
    hold_phase.min_range_m = rel_range(x);
    bool hold_ok = true;
    hop(-standoff, hold_phase.dv_m_s, hold_phase.min_range_m, hold_ok);
    hold_phase.t_end_s = t_clock;
    hold_phase.abort_feasible_throughout = hold_ok;
    rep.phases.push_back(hold_phase);
    if (aborted) { abort_and_stop(); return rep; }

    // ---- SyncHold: dwell at the standoff equilibrium (zero dv). Attitude
    //      sync is WP2's job and is not re-simulated in this L0 demo. ----
    GuidedPhaseLog sync_phase;
    sync_phase.mode = GuidanceMode::SyncHold;
    sync_phase.t_start_s = t_clock;
    t_clock += cfg_.sync_hold_s + kSyncAllowanceS;
    sync_phase.t_end_s = t_clock;
    sync_phase.min_range_m = rel_range(x);  // constant: x is an equilibrium
    sync_phase.dv_m_s = 0.0;
    sync_phase.abort_feasible_throughout = true;
    rep.phases.push_back(sync_phase);

    // ---- FinalApproach: glideslope-with-floor descent along -V-bar. ----
    // v_des(range) = max(guid_contact_speed_m_s, guid_glideslope_k * range):
    // at the standoff (guid_hold_far_m's departure already happened; the
    // relevant crossover is at range = contact_speed / k = 400 m, exactly
    // the standoff) the floor and the k*range branch are equal, so this
    // demo's whole final approach flies at the constant floor speed -- the
    // k*range branch only matters if final approach ever starts farther out.
    // Each step re-solves the exact two-point boundary velocity to the next
    // waypoint (rather than a naive single-axis command), so the small
    // Coriolis-like radial coupling (x_ddot = 3n^2 x + 2n*vy) is corrected
    // every step instead of accumulating -- an uncorrected constant-vy
    // command diverges over the ~4000 s / 67-step descent (verified
    // numerically; not shipped).
    GuidedPhaseLog final_phase;
    final_phase.mode = GuidanceMode::FinalApproach;
    final_phase.t_start_s = t_clock;
    final_phase.min_range_m = rel_range(x);
    preauth = false;  // authorized keep-out penetration begins here (D5/WP11)
    bool final_ok = true;
    while (true) {
        const double range = rel_range(x);
        if (range <= cfg_.guid_contact_range_m) break;
        const double y_now = x(1);
        const double v_des =
            std::max(cfg_.guid_contact_speed_m_s, cfg_.guid_glideslope_k * std::abs(y_now));
        double y_next = y_now + v_des * cfg_.guid_step_s;  // y_now < 0, closing toward 0
        if (y_next > -cfg_.guid_contact_range_m) y_next = -cfg_.guid_contact_range_m;
        const Eigen::Vector3d r0 = x.head<3>();
        const Eigen::Vector3d v0 = x.tail<3>();
        const Eigen::Vector3d r_target(0.0, y_next, 0.0);
        const Eigen::Vector3d v_req = transfer_velocity(cw_, cfg_.guid_step_s, r0, r_target);
        if (!screen(r0, v_req)) { final_ok = false; break; }
        final_phase.dv_m_s += (v_req - v0).norm();
        dv_total += (v_req - v0).norm();
        x.tail<3>() = v_req;  // commit BEFORE sampling: the coast runs at v_req, not v0
        sample_leg(x, cfg_.guid_step_s, final_phase.min_range_m);
        x = cw_.propagate(x, cfg_.guid_step_s);
        t_clock += cfg_.guid_step_s;
        // LOS cone about -V-bar (only meaningful once past contact range).
        if (std::abs(x(1)) > cfg_.guid_contact_range_m) {
            const double cone_radius =
                std::tan(cfg_.guid_los_half_angle_rad) * std::abs(x(1));
            const bool los_ok = std::sqrt(x(0) * x(0) + x(2) * x(2)) <= cone_radius;
            los_cone_ok = los_cone_ok && los_ok;
        }
    }
    final_phase.t_end_s = t_clock;
    final_phase.abort_feasible_throughout = final_ok;
    rep.phases.push_back(final_phase);
    if (aborted) { abort_and_stop(); return rep; }

    // ---- Contact: matching burn to the design contact speed (contact speed
    //      is PRODUCED by this burn, not merely gated by comparing against
    //      max_v_rel afterward). Attach itself is WP3's clamp, not
    //      re-simulated here. ----
    GuidedPhaseLog contact_phase;
    contact_phase.mode = GuidanceMode::Contact;
    contact_phase.t_start_s = t_clock;
    contact_phase.min_range_m = rel_range(x);
    const Eigen::Vector3d r_contact = x.head<3>();
    const Eigen::Vector3d v_before_contact = x.tail<3>();
    const Eigen::Vector3d target_v(0.0, cfg_.guid_contact_speed_m_s, 0.0);
    bool contact_ok = screen(r_contact, target_v);
    if (!contact_ok) {
        contact_phase.t_end_s = t_clock;
        contact_phase.abort_feasible_throughout = false;
        rep.phases.push_back(contact_phase);
        abort_and_stop();
        return rep;
    }
    contact_phase.dv_m_s = (target_v - v_before_contact).norm();
    dv_total += contact_phase.dv_m_s;
    x.tail<3>() = target_v;
    contact_phase.t_end_s = t_clock;
    contact_phase.abort_feasible_throughout = true;
    rep.phases.push_back(contact_phase);
    rep.contact_speed_m_s = x.tail<3>().norm();
    rep.contact_range_m   = rel_range(x);

    // ---- Retreat: the RetreatHop abort's own two burns ARE the maneuver
    //      (burn 1, half-period coast, burn 2), then a V-bar hop/station-keep
    //      to re-establish the standoff if the post-hop range is short of
    //      it. The RetreatHop burns are the escalation of last resort
    //      itself -- not re-screened -- but the station-keep hop (if
    //      needed) is a regular hop and IS screened. ----
    GuidedPhaseLog retreat_phase;
    retreat_phase.mode = GuidanceMode::Retreat;
    retreat_phase.t_start_s = t_clock;
    retreat_phase.min_range_m = rel_range(x);

    const SafeAbort retreat_ab =
        clearing_abort_for(cfg_, cw_, x.head<3>(), x.tail<3>());
    x.tail<3>() += retreat_ab.dv;
    retreat_phase.dv_m_s += retreat_ab.dv.norm();
    dv_total += retreat_ab.dv.norm();

    const double half_period = 0.5 * period;
    sample_leg(x, half_period, retreat_phase.min_range_m);
    x = cw_.propagate(x, half_period);
    t_clock += half_period;

    // Burn 2: null vx and vz (analytically only vx is nonzero at T/2; vy is
    // already the drift-free rate for the new radial offset, so it is left
    // untouched; nulling vz absorbs numerical residue from the discretized
    // sweep -- see clearing_abort_for's RetreatHop comment, mission.cpp).
    const Eigen::Vector3d v_before_burn2 = x.tail<3>();
    x(3) = 0.0;
    x(5) = 0.0;
    const double dv2_actual = (v_before_burn2 - x.tail<3>()).norm();
    retreat_phase.dv_m_s += dv2_actual;
    dv_total += dv2_actual;
    retreat_phase.min_range_m = std::min(retreat_phase.min_range_m, rel_range(x));

    bool retreat_ok = true;
    if (rel_range(x) < standoff) {
        const double y_target = (x(1) <= 0.0) ? -standoff : standoff;
        hop(y_target, retreat_phase.dv_m_s, retreat_phase.min_range_m, retreat_ok);
    } else if (!screen(x.head<3>(), Eigen::Vector3d::Zero())) {
        retreat_ok = false;
    } else {
        // Accept the post-hop station: null any residual velocity in place.
        const Eigen::Vector3d dv_station = -x.tail<3>();
        retreat_phase.dv_m_s += dv_station.norm();
        dv_total += dv_station.norm();
        x.tail<3>() = Eigen::Vector3d::Zero();
    }
    retreat_phase.t_end_s = t_clock;
    retreat_phase.abort_feasible_throughout = retreat_ok;
    rep.phases.push_back(retreat_phase);
    if (aborted) { abort_and_stop(); return rep; }

    // ---- Complete: standoff equilibrium re-established outside keep_out +
    //      margin. ----
    GuidedPhaseLog complete_phase;
    complete_phase.mode = GuidanceMode::Complete;
    complete_phase.t_start_s = t_clock;
    complete_phase.t_end_s   = t_clock;
    complete_phase.min_range_m = rel_range(x);
    complete_phase.dv_m_s = 0.0;
    complete_phase.abort_feasible_throughout = true;
    rep.phases.push_back(complete_phase);

    rep.completed  = rel_range(x) > R_clear;
    rep.final_mode = rep.completed ? GuidanceMode::Complete : GuidanceMode::Abort;
    rep.dv_total_m_s = dv_total;
    rep.min_clearance_outside_m = min_clearance_outside;
    rep.abort_feasible_every_step = abort_feasible_every_step;
    rep.los_cone_ok = los_cone_ok;
    return rep;
}

// ============================================================================
// WP12 L4: estimate-driven translation guidance
// ----------------------------------------------------------------------------
// Closes the WP11/README "estimate-driven translation guidance remains open"
// known limit. Truth propagates the SAME deterministic CW dynamics as fly()
// above and receives MATCHED velocity process noise at each ranging interval
// (mirrors run_estimated_sync, estimator.hpp/WP4); range + LOS-unit-vector
// measurements are drawn from truth with the existing WP4 sigmas, gated by a
// Bernoulli sensor-dropout draw (a missed detection: the update is skipped,
// but the filter still predicts), and the range measurement carries an
// UNESTIMATED random-walk bias -- a documented estimation gap (the filter has
// no bias state), so a large enough walk will inflate the reported NIS; that
// is reported honestly, not hidden.
//
// Every guidance decision (hop transfer-velocity solves, the glideslope
// target, the reachability screen, the contact matching burn, and the
// post-contact retreat) reads the CURRENT TranslationEkf estimate, never
// x_truth; the SAME resulting impulse is then applied to both x_truth (the
// physically actuated vehicle) and the filter's own velocity estimate
// (TranslationEkf::apply_control_delta_v -- a known control input adds no
// uncertainty). Truth is read ONLY for the matched process noise, the
// error-statistics accumulators, and the final honest contact-speed /
// contact-range report (D5/WP11's truth-isolation discipline, extended to
// translation the way WP4 already does for attitude).
// ============================================================================
GuidedApproachReport GuidedApproach::fly_estimate_driven() {
    GuidedApproachReport rep;

    Vector6d x_truth = Vector6d::Zero();
    x_truth(1) = -cfg_.guid_hold_far_m;

    // One fixed-seed GaussianSource drives every sensor draw (measurement
    // noise, the dropout gate, the bias-walk increment, and the sampled
    // initial estimate error) -- deterministic (R6).
    GaussianSource rng(cfg_.est_seed);
    Vector6d x0_est = x_truth;
    x0_est.head<3>() += rng.sample3(cfg_.est_init_pos_sigma_m);
    x0_est.tail<3>() += rng.sample3(cfg_.est_init_vel_sigma_m_s);
    Eigen::Matrix<double, 6, 6> P0 = Eigen::Matrix<double, 6, 6>::Zero();
    P0.block<3, 3>(0, 0) = cfg_.est_init_pos_sigma_m * cfg_.est_init_pos_sigma_m *
                           Eigen::Matrix3d::Identity();
    P0.block<3, 3>(3, 3) = cfg_.est_init_vel_sigma_m_s * cfg_.est_init_vel_sigma_m_s *
                           Eigen::Matrix3d::Identity();
    TranslationEkf ekf(cw_, x0_est, P0);

    double range_bias_m = 0.0;  // unestimated random-walk range bias (documented gap)
    double nis_sum = 0.0;  int nis_n  = 0;
    double nees_sum = 0.0; int nees_n = 0;
    double dv_total = 0.0;
    bool   aborted = false;
    bool   abort_feasible_every_step = true;
    bool   los_cone_ok = true;
    bool   preauth = true;  // see fly()'s identical field for the rationale
    double min_clearance_outside = std::numeric_limits<double>::max();
    const double dt_r = 1.0 / cfg_.ranging_rate_hz;

    // Coast BOTH truth and the filter over `duration` seconds at the ranging
    // cadence: truth carries matched velocity process noise (mirrors WP4's
    // run_estimated_sync), the filter predicts with the analytic CW STM
    // (exact for any dt -- sub-stepping here is for the SENSOR cadence, not
    // numerical accuracy), and a range+LOS update is taken at every interval
    // EXCEPT where the Bernoulli dropout draw hits. NIS is tallied only on
    // executed updates; NEES on every sub-step (it needs only the current
    // estimate/truth pair, no measurement).
    const auto coast_estimated = [&](double duration) {
        double remaining = duration;
        while (remaining > 1e-9) {
            const double h = std::min(dt_r, remaining);
            x_truth = cw_.propagate(x_truth, h);
            x_truth.tail<3>() += rng.sample3(cfg_.trans_vel_noise_m_s);
            ekf.predict(h, cfg_.trans_vel_noise_m_s);

            range_bias_m +=
                rng.sample() * cfg_.range_bias_walk_m_per_sqrt_s * std::sqrt(h);

            if (rng.uniform01() >= cfg_.sensor_dropout_prob) {
                const Eigen::Vector3d r_true = x_truth.head<3>();
                const double z_range =
                    r_true.norm() + range_bias_m + cfg_.range_sigma_m * rng.sample();
                const Eigen::Vector3d z_los =
                    r_true.normalized() + rng.sample3(cfg_.los_sigma);
                nis_sum += ekf.update(z_range, z_los, cfg_.range_sigma_m, cfg_.los_sigma);
                ++nis_n;
            }
            const Vector6d e = ekf.state() - x_truth;
            nees_sum += e.dot(ekf.covariance().ldlt().solve(e));
            ++nees_n;

            const double range = rel_range(x_truth);
            if (preauth && range > cfg_.keep_out_radius_m) {
                min_clearance_outside =
                    std::min(min_clearance_outside, range - cfg_.keep_out_radius_m);
            }
            remaining -= h;
        }
    };

    // Screen the candidate ESTIMATED post-impulse state (the vehicle has no
    // other observable), then apply the SAME delta-v to truth and to the
    // filter's velocity estimate.
    const auto commit_impulse = [&](const Eigen::Vector3d& v_req) -> bool {
        const Eigen::Vector3d r_est = ekf.state().head<3>();
        const bool ok = reachability_screen_ok(cfg_, cw_, r_est, v_req, nullptr);
        abort_feasible_every_step = abort_feasible_every_step && ok;
        if (!ok) { aborted = true; return false; }
        const Eigen::Vector3d dv = v_req - ekf.state().tail<3>();
        dv_total += dv.norm();
        x_truth.tail<3>() += dv;
        ekf.apply_control_delta_v(dv);
        return true;
    };

    const double period = cw_.period();
    const double tau = cfg_.guid_transfer_frac_T * period;
    const double standoff = cfg_.depart_standoff_factor * cfg_.keep_out_radius_m;
    const double R_clear = cfg_.keep_out_radius_m + cfg_.abort_clear_margin_m;

    // Last-resort clearing abort, executed from TRUTH (the physically real
    // state) -- same principle as fly()'s abort_and_stop. Also finalizes
    // every trailing report field, so every early-exit site below is just
    // `if (aborted) { abort_and_stop(); return rep; }`, matching fly()'s idiom.
    const auto abort_and_stop = [&]() {
        const SafeAbort ab =
            clearing_abort_for(cfg_, cw_, x_truth.head<3>(), x_truth.tail<3>());
        x_truth.tail<3>() += ab.dv;
        dv_total += ab.dv.norm() + ab.dv_second_burn_m_s;
        rep.completed  = false;
        rep.final_mode = GuidanceMode::Abort;
        rep.dv_total_m_s = dv_total;
        rep.min_clearance_outside_m = min_clearance_outside;
        rep.abort_feasible_every_step = abort_feasible_every_step;
        rep.los_cone_ok = los_cone_ok;
        rep.est_final_pos_err_m = (ekf.state().head<3>() - x_truth.head<3>()).norm();
        rep.est_nis_trans_mean  = (nis_n  > 0) ? nis_sum  / nis_n  : 0.0;
        rep.est_nees_trans_mean = (nees_n > 0) ? nees_sum / nees_n : 0.0;
        rep.est_nis_n  = nis_n;
        rep.est_nees_n = nees_n;
    };

    // Two-impulse hold-to-hold hop along -V-bar, transfer velocity solved
    // from the ESTIMATE (WP12 L4; compare fly()'s truth-fed hop).
    const auto hop = [&](double y_target) {
        if (aborted) return;
        const Eigen::Vector3d r0 = ekf.state().head<3>();
        const Eigen::Vector3d r_target(0.0, y_target, 0.0);
        const Eigen::Vector3d v_req = transfer_velocity(cw_, tau, r0, r_target);
        if (!commit_impulse(v_req)) return;
        coast_estimated(tau);
        if (aborted) return;
        commit_impulse(Eigen::Vector3d::Zero());  // null arrival velocity
    };

    // ---- FarApproach -> Hold: depart the far hold, then the mid hold. ----
    hop(-cfg_.guid_hold_mid_m);
    if (aborted) { abort_and_stop(); return rep; }
    hop(-standoff);
    if (aborted) { abort_and_stop(); return rep; }

    // ---- SyncHold: zero-dv dwell at the standoff (attitude sync is WP2's
    //      job, not re-simulated here); the filter keeps converging through
    //      the dwell via coast_estimated's ranging updates. ----
    coast_estimated(cfg_.sync_hold_s + kSyncAllowanceS);

    // ---- FinalApproach: glideslope-with-floor descent, ESTIMATE-driven. ----
    preauth = false;  // authorized keep-out penetration begins here (D5/WP11)
    // WP12 defensive cap: fly()'s truth-fed loop provably terminates (the
    // commanded closing rate is always positive), but the ESTIMATE could in
    // principle stall short of contact range under noise/dropout; this bound
    // is ~7x the nominal ~67-step descent.
    const int kMaxFinalApproachSteps = 500;
    bool reached_contact_range = false;
    for (int step = 0; step < kMaxFinalApproachSteps && !aborted; ++step) {
        const double range_est = ekf.state().head<3>().norm();
        if (range_est <= cfg_.guid_contact_range_m) { reached_contact_range = true; break; }
        const double y_now = ekf.state()(1);
        const double v_des =
            std::max(cfg_.guid_contact_speed_m_s, cfg_.guid_glideslope_k * std::abs(y_now));
        double y_next = y_now + v_des * cfg_.guid_step_s;  // y_now < 0, closing toward 0
        if (y_next > -cfg_.guid_contact_range_m) y_next = -cfg_.guid_contact_range_m;
        const Eigen::Vector3d r0 = ekf.state().head<3>();
        const Eigen::Vector3d r_target(0.0, y_next, 0.0);
        const Eigen::Vector3d v_req = transfer_velocity(cw_, cfg_.guid_step_s, r0, r_target);
        if (!commit_impulse(v_req)) break;
        coast_estimated(cfg_.guid_step_s);
        // LOS cone about -V-bar, evaluated on the ESTIMATE.
        if (std::abs(ekf.state()(1)) > cfg_.guid_contact_range_m) {
            const double cone_radius =
                std::tan(cfg_.guid_los_half_angle_rad) * std::abs(ekf.state()(1));
            const bool los_ok =
                std::sqrt(ekf.state()(0) * ekf.state()(0) + ekf.state()(2) * ekf.state()(2)) <=
                cone_radius;
            los_cone_ok = los_cone_ok && los_ok;
        }
    }
    if (aborted) { abort_and_stop(); return rep; }
    if (!reached_contact_range) { aborted = true; abort_and_stop(); return rep; }

    // ---- Contact: matching burn to the design contact speed, ESTIMATE-
    //      driven; contact speed/range REPORTED from TRUTH (honest reporting,
    //      D5/WP11 truth-isolation). ----
    const Eigen::Vector3d target_v(0.0, cfg_.guid_contact_speed_m_s, 0.0);
    if (!commit_impulse(target_v)) { abort_and_stop(); return rep; }
    rep.contact_speed_m_s = x_truth.tail<3>().norm();
    rep.contact_range_m   = x_truth.head<3>().norm();

    // ---- Retreat: RetreatHop-style two-burn departure computed from the
    //      ESTIMATE (compare fly()'s truth-fed retreat), re-establishing the
    //      standoff (or station-keeping in place if the post-hop range
    //      already clears it). ----
    {
        const SafeAbort retreat_ab =
            clearing_abort_for(cfg_, cw_, ekf.state().head<3>(), ekf.state().tail<3>());
        dv_total += retreat_ab.dv.norm();
        x_truth.tail<3>() += retreat_ab.dv;
        ekf.apply_control_delta_v(retreat_ab.dv);

        coast_estimated(0.5 * period);

        // Burn 2: null vx and vz on the ESTIMATE (only those are analytically
        // nonzero at T/2 for the drift-free family -- see fly()'s RetreatHop
        // burn 2 comment); apply the SAME delta to truth.
        const Eigen::Vector3d v_before_burn2 = ekf.state().tail<3>();
        const Eigen::Vector3d v_after_burn2(0.0, v_before_burn2.y(), 0.0);
        const Eigen::Vector3d dv2 = v_after_burn2 - v_before_burn2;
        dv_total += dv2.norm();
        x_truth.tail<3>() += dv2;
        ekf.apply_control_delta_v(dv2);

        if (ekf.state().head<3>().norm() < standoff) {
            const double y_target = (ekf.state()(1) <= 0.0) ? -standoff : standoff;
            hop(y_target);
        } else {
            commit_impulse(Eigen::Vector3d::Zero());  // station-keep: null residual velocity
        }
    }
    if (aborted) { abort_and_stop(); return rep; }

    // ---- Complete: standoff equilibrium re-established outside keep_out +
    //      margin (TRUTH-evaluated: the honest completion criterion). ----
    rep.completed  = x_truth.head<3>().norm() > R_clear;
    rep.final_mode = rep.completed ? GuidanceMode::Complete : GuidanceMode::Abort;
    rep.dv_total_m_s = dv_total;
    rep.min_clearance_outside_m = min_clearance_outside;
    rep.abort_feasible_every_step = abort_feasible_every_step;
    rep.los_cone_ok = los_cone_ok;
    rep.est_final_pos_err_m = (ekf.state().head<3>() - x_truth.head<3>()).norm();
    rep.est_nis_trans_mean  = (nis_n  > 0) ? nis_sum  / nis_n  : 0.0;
    rep.est_nees_trans_mean = (nees_n > 0) ? nees_sum / nees_n : 0.0;
    rep.est_nis_n  = nis_n;
    rep.est_nees_n = nees_n;
    return rep;
}

}  // namespace adsc
