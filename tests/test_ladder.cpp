// WP12 fidelity-ladder tests. Framework-free (see tests/test_relmotion.cpp
// for the assertion-macro style); explicit return-1 checks (R4).
//
// MANDATORY cross-validation (stated eps = 2.0 m; see kCrossValEpsM below
// for the measured linearization-error justification): L1 with J2 forced off
// must reproduce the ORIGINAL WP1 Clohessy-Wiltshire closed form
// for (a) the WP1 424.3 m worst-coast scenario (re-derived here from the
// SAME corridor sweep main_metrics.cpp uses, rather than a hand-copied
// state, so this is also an independent regression check on that pinned
// number), (b) all 14 forensic-14 post-abort coast minima (pinned states,
// R15 -- see include/adsc/forensic14_states.hpp for the shared-header
// provenance note), and (c) the guidance-relevant departure-standoff case.
// L2 with drag forced off must reproduce L1 bit-for-bit (same code path).
// A J2-ON sanity check verifies the erosion is nonzero but bounded. L4/L5
// demo assertions close out the fidelity ladder (estimate-driven guidance,
// actuator realization error).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "adsc/forensic14_states.hpp"
#include "adsc/guidance.hpp"
#include "adsc/mission.hpp"
#include "adsc/propagation.hpp"
#include "adsc/relmotion.hpp"

using namespace adsc;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

namespace {
// Cross-validation tolerance [m]. The L1 path with J2 forced off is the FULL
// NONLINEAR two-body problem, so it differs from the linear CW closed form by
// the CW linearization error, not by zero: the second-order along-track term
// scales like ~3*pi*rho^2/r per orbit (~1 m at the forensic-14 ellipse
// amplitudes, rho ~ 400-700 m at r ~ 7.1e6 m). Measured worst case across all
// 14 pinned states: 0.52 m (SL-16/340, the largest x0 > 0 excursion). 2.0 m
// bounds that with margin while still catching any real frame/initialization
// bug, which produces tens-to-hundreds of meters of error immediately.
constexpr double kCrossValEpsM = 2.0;
}  // namespace

int main() {
    // ------------------------------------------------------------------
    // 1(a). WP1 424.3 m worst-coast scenario: re-derive the SAME corridor
    //       sweep main_metrics.cpp's WP1 section runs (identical corridor,
    //       12 phases/hold, 2 orbital periods, dt = 1 s), locate the
    //       actual worst (minimum-range) state, then cross-validate L1
    //       (J2 forced off) against the ORIGINAL CW RK4 sweep from that
    //       SAME t=0 state.
    // ------------------------------------------------------------------
    {
        const Config cfg;  // default target_altitude_km = 825.0 km (WP1 generic reference orbit)
        const double a = kEarthRadius + cfg.target_altitude_km * 1000.0;
        const CwModel cw = CwModel::from_orbit(a);
        const std::vector<SafetyEllipse> corridor = approach_corridor(
            cfg.approach_rho_far_m, cfg.approach_rho_near_m, cfg.keep_out_radius_m,
            cfg.approach_holds);
        CHECK(!corridor.empty());

        const double horizon = 2.0 * cw.period();
        const double dt = 1.0;
        const int phases = 12;
        double worst = std::numeric_limits<double>::max();
        Vector6d worst_x0 = Vector6d::Zero();

        for (const SafetyEllipse& hold : corridor) {
            for (int p = 0; p < phases; ++p) {
                const double theta = 2.0 * kPi * static_cast<double>(p) / phases;
                const Vector6d x0 = cw.ellipse_state(hold, theta);
                Vector6d x = x0;
                double local_min = rel_range(x);
                double t = 0.0;
                while (t < horizon) {
                    x = cw.propagate_rk4(x, dt, dt);
                    local_min = std::min(local_min, rel_range(x));
                    t += dt;
                }
                if (local_min < worst) {
                    worst = local_min;
                    worst_x0 = x0;
                }
            }
        }
        // Independent regression check on the pinned WP1 number itself
        // (generated/reference_metrics.csv: wp1_worst_coast_min_range_m,
        // 19.15/424.3/16.87/17.07 s reference family, R15).
        CHECK(std::abs(worst - 424.264069) < 1e-2);

        // Two-body cross-validation: inclination is immaterial to a pure
        // two-body (J2-off) relative coast (spherical-potential symmetry),
        // so any fixed value is a valid cross-validation target.
        const double l1_j2off = fidelity_coast_min_range_terms(
            /*enable_j2=*/false, /*enable_drag=*/false, worst_x0,
            cfg.target_altitude_km, /*incl_deg=*/51.6, horizon, dt,
            /*solar_factor=*/1.0, /*bc_chaser=*/0.0, /*bc_target=*/0.0);
        CHECK(std::abs(l1_j2off - worst) < kCrossValEpsM);
    }

    // ------------------------------------------------------------------
    // 1(b). All 14 forensic-14 post-abort coast minima: SAME commanded dv
    //       from the WP11 clearing-abort law (unchanged), coast-propagated
    //       one target orbital period at dt = 1 s.
    // ------------------------------------------------------------------
    for (int i = 0; i < kForensic14Count; ++i) {
        const Forensic14State& st = kForensic14States[i];
        const double alt_km   = (st.catalog == 'A') ? 840.0 : 750.0;
        const double incl_deg = (st.catalog == 'A') ? 71.0 : 78.0;

        Config cfg;
        cfg.target_altitude_km = alt_km;
        const CwModel cw = CwModel::from_orbit(kEarthRadius + alt_km * 1000.0);
        const Eigen::Vector3d r(st.x0, st.y0, st.z0);
        const Eigen::Vector3d v(st.vx0, st.vy0, st.vz0);
        const SafeAbort ab = clearing_abort_for(cfg, cw, r, v);

        Vector6d x0;
        x0 << r, (v + ab.dv);
        const double horizon = cw.period();
        const double dt = 1.0;

        Vector6d x = x0;
        double cw_min = rel_range(x);
        double t = 0.0;
        while (t < horizon) {
            x = cw.propagate_rk4(x, dt, dt);
            cw_min = std::min(cw_min, rel_range(x));
            t += dt;
        }

        const double l1_j2off = fidelity_coast_min_range_terms(
            false, false, x0, alt_km, incl_deg, horizon, dt, 1.0, 0.0, 0.0);
        CHECK(std::abs(l1_j2off - cw_min) < kCrossValEpsM);
    }
    std::printf("ladder: L1(J2 off) reproduces the CW closed form for the WP1 424.3 "
               "scenario and all %d forensic-14 post-abort coasts (eps %.1f m)\n",
               kForensic14Count, kCrossValEpsM);

    // ------------------------------------------------------------------
    // 1(c). Guidance-relevant standoff case: the departure-standoff V-bar
    //       equilibrium (r = (0, -depart_standoff_factor*keep_out, 0),
    //       v = 0 -- a CW drift-free rest state, guidance.cpp/campaign.cpp's
    //       own departure-standoff geometry).
    // ------------------------------------------------------------------
    {
        const Config cfg;
        const CwModel cw = CwModel::from_orbit(kEarthRadius + cfg.target_altitude_km * 1000.0);
        Vector6d x0 = Vector6d::Zero();
        x0(1) = -cfg.depart_standoff_factor * cfg.keep_out_radius_m;
        const double horizon = cw.period();
        const double dt = 1.0;

        Vector6d x = x0;
        double cw_min = rel_range(x);
        double t = 0.0;
        while (t < horizon) {
            x = cw.propagate_rk4(x, dt, dt);
            cw_min = std::min(cw_min, rel_range(x));
            t += dt;
        }
        // A true CW equilibrium never moves: sanity-check the closed-form
        // side is (to numerical precision) exactly the standoff range.
        CHECK(std::abs(cw_min - cfg.depart_standoff_factor * cfg.keep_out_radius_m) < 1e-6);

        const double l1_j2off = fidelity_coast_min_range_terms(
            false, false, x0, cfg.target_altitude_km, 51.6, horizon, dt, 1.0, 0.0, 0.0);
        CHECK(std::abs(l1_j2off - cw_min) < kCrossValEpsM);
    }
    std::printf("ladder: L1(J2 off) reproduces the CW closed form for the "
               "departure-standoff case (eps %.1f m)\n", kCrossValEpsM);

    // ------------------------------------------------------------------
    // 2. L2 with drag forced off reproduces L1 bit-for-bit (same code path,
    //    same arguments) -- verifies the L2 dispatch doesn't silently alter
    //    the J2-only physics.
    // ------------------------------------------------------------------
    {
        const Config cfg;
        Vector6d x0 = Vector6d::Zero();
        x0(1) = -400.0;
        x0(0) = 40.0;
        const CwModel cw = CwModel::from_orbit(kEarthRadius + cfg.target_altitude_km * 1000.0);
        const double horizon = cw.period();
        const double dt = 8.0;

        const double l1 = fidelity_coast_min_range(
            FidelityLevel::L1_J2, x0, cfg.target_altitude_km, 71.0, horizon, dt,
            1.0, 1.0e-3, 1.0e-3);
        const double l2_drag_off = fidelity_coast_min_range_terms(
            true, false, x0, cfg.target_altitude_km, 71.0, horizon, dt, 1.0, 1.0e-3, 1.0e-3);
        CHECK(std::abs(l1 - l2_drag_off) < 1e-9);
    }
    std::printf("ladder: L2 with drag forced off reproduces L1 bit-for-bit\n");

    // ------------------------------------------------------------------
    // 3. J2-ON sanity check: a 400 m standoff ellipse's min range over one
    //    orbital period changes by a small but NONZERO amount at L1 vs L0
    //    (differential J2 erosion is a real, bounded effect -- neither
    //    "no effect at all" nor "orders of magnitude too large").
    // ------------------------------------------------------------------
    {
        const double alt_km = 840.0, incl_deg = 71.0;  // SL-16-class reference orbit
        const CwModel cw = CwModel::from_orbit(kEarthRadius + alt_km * 1000.0);
        const SafetyEllipse ellipse{400.0, 400.0, 0.0};
        const Vector6d x0 = cw.ellipse_state(ellipse, 0.0);
        const double horizon = cw.period();
        const double dt = 8.0;

        const double l0 = fidelity_coast_min_range(
            FidelityLevel::L0_CW, x0, alt_km, incl_deg, horizon, dt, 1.0, 0.0, 0.0);
        const double l1 = fidelity_coast_min_range(
            FidelityLevel::L1_J2, x0, alt_km, incl_deg, horizon, dt, 1.0, 0.0, 0.0);
        const double erosion = std::abs(l0 - l1);
        CHECK(erosion > 0.05);
        CHECK(erosion < 50.0);
        std::printf("ladder: J2-on standoff-ellipse erosion over one period = %.4f m "
                   "(bounded, nonzero)\n", erosion);
    }

    // ------------------------------------------------------------------
    // 4. L4 (estimate-driven guidance) demo assertions.
    // ------------------------------------------------------------------
    {
        Config cfg;
        cfg.guid_estimate_driven = true;
        GuidedApproach approach(cfg);
        const GuidedApproachReport rep = approach.fly();
        CHECK(rep.completed);
        CHECK(rep.contact_speed_m_s <= cfg.max_v_rel);
        CHECK(std::isfinite(rep.est_nis_trans_mean));
        CHECK(std::isfinite(rep.est_nees_trans_mean));
        CHECK(rep.est_nis_n > 0);
        CHECK(rep.est_nees_n > 0);
        std::printf("ladder: L4 estimate-driven guidance demo completed, contact speed "
                   "%.4f m/s (gate %.4f m/s), NIS %.3f (n=%d), NEES %.3f (n=%d)\n",
                   rep.contact_speed_m_s, cfg.max_v_rel, rep.est_nis_trans_mean,
                   rep.est_nis_n, rep.est_nees_trans_mean, rep.est_nees_n);
    }

    // ------------------------------------------------------------------
    // 5. L5 (actuator realization error) assertions.
    // ------------------------------------------------------------------
    {
        const Config cfg;
        const double deg2rad = kPi / 180.0;
        const Eigen::Quaterniond q_t0 = Eigen::Quaterniond::Identity();
        const Eigen::Vector3d w_t0 = cfg.sync_target_rate_deg_s * deg2rad *
                                     Eigen::Vector3d(0.4, 0.7, -0.59).normalized();
        const Eigen::Quaterniond q_c0(Eigen::AngleAxisd(
            40.0 * deg2rad, Eigen::Vector3d(1.0, 0.5, -0.2).normalized()));
        const double short_run_s = 40.0;

        // Neutral-default byte-identity: a short 6-arg run vs an explicit,
        // fully-neutral 7-arg ActuatorError{} run (WP12's new MIB/delay/
        // fault fields default-neutral too).
        const SyncReport a = run_tumble_sync(
            cfg, q_t0, w_t0, q_c0, Eigen::Vector3d::Zero(), short_run_s);
        const SyncReport b = run_tumble_sync(
            cfg, q_t0, w_t0, q_c0, Eigen::Vector3d::Zero(), short_run_s, ActuatorError{});
        CHECK(a.synced == b.synced);
        CHECK(a.sync_time_s == b.sync_time_s);
        CHECK(a.max_rate_err_deg_s == b.max_rate_err_deg_s);
        CHECK(a.max_att_err_deg == b.max_att_err_deg);
        CHECK(a.final_rate_err_deg_s == b.final_rate_err_deg_s);
        CHECK(a.final_att_err_deg == b.final_att_err_deg);

        // MIB + one-step delay + partial single-axis fault (SAME parameters
        // as the wp12_l5_* main_metrics.cpp demo): still synchronizes.
        ActuatorError l5_act;
        l5_act.min_impulse_bit_nms = 2.0e-4;
        l5_act.delay_steps         = 1;
        l5_act.fault_axis          = 0;
        l5_act.fault_axis_scale    = 0.5;
        const SyncReport l5 = run_tumble_sync(
            cfg, q_t0, w_t0, q_c0, Eigen::Vector3d::Zero(), 120.0, l5_act);
        // Diagnostic BEFORE the assert so a failure documents its numbers in
        // the CI log (synced flag, declare time, terminal errors).
        std::printf("ladder: L5 MIB/delay/fault run: synced=%d sync_time=%.2f s "
                   "final rate err %.4f deg/s, final att err %.4f deg\n",
                   l5.synced ? 1 : 0, l5.sync_time_s,
                   l5.final_rate_err_deg_s, l5.final_att_err_deg);
        CHECK(l5.synced);
        std::printf("ladder: L5 neutral-default byte-identity verified; MIB/delay/fault "
                   "run synced at %.2f s\n", l5.sync_time_s);
    }

    std::printf("ladder: all WP12 fidelity-ladder checks passed\n");
    return 0;
}
