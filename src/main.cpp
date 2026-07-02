#include <algorithm>
#include <cstdio>
#include <limits>
#include <vector>

#include "adsc/mission.hpp"

int main() {
    using namespace adsc;

    std::printf("=== ADSC v4 (WP2) — Active Debris Self-Cleanup ===\n\n");

    Mission mission;
    std::printf("Dry mass        : %.2f kg\n", mission.config().dry_mass_kg);
    std::printf("Initial fuel    : %.2f kg\n", mission.fuel_kg());
    std::printf("Control step dt : %.3f s\n\n", mission.config().control_dt);

    // --- Scenario 1: closing too fast -> safe abort ------------------------
    {
        Eigen::Vector3d r_rel(0.08, 0.04, 0.11);
        Eigen::Vector3d v_fast(0.20, 0.10, 0.10);   // |v| ~ 0.245 m/s > cap
        auto rep = mission.post_capture_stabilization(
            true, 2.4, Eigen::Vector3d(0, 0, 0.15), r_rel, v_fast,
            Eigen::Vector3d(0.05, -0.04, 0.03));
        std::printf("[Scenario 1] closing speed %.3f m/s -> %s\n",
                    v_fast.norm(), rep.aborted ? "ABORT (safe maneuver)" : "captured");

        // F1 honesty: report what the abort impulse actually buys. At contact
        // range a clean CW abort only bounds the relative orbit through the
        // current position — it does not create separation.
        const SafeAbort ab = mission.compute_safe_abort(r_rel, v_fast);
        std::printf("  abort impulse |dv|   : %.3f m/s (%s)\n", ab.dv.norm(),
                    ab.status == SafeAbort::Status::Capped ? "CAPPED" : "clean");
        std::printf("  post-burn coast min  : %.3f m over %.0f periods\n\n",
                    ab.coast_min_range_m,
                    mission.config().abort_coast_check_periods);
    }

    // --- Scenario 2: valid capture -> closed-loop detumble -----------------
    {
        Eigen::Vector3d r_rel(0.08, 0.04, 0.11);
        Eigen::Vector3d v_ok(0.12, 0.05, 0.04);      // |v| ~ 0.14 m/s < cap
        Eigen::Vector3d tumble(0.15, -0.12, 0.09);   // post-capture body rate
        auto rep = mission.post_capture_stabilization(
            true, 2.4, Eigen::Vector3d(0, 0, 0.15), r_rel, v_ok, tumble);

        std::printf("[Scenario 2] captured=%s\n", rep.captured ? "yes" : "no");
        std::printf("  mass after capture : %.2f kg\n", rep.mass_total_kg);
        std::printf("  inertia trace      : %.4f kg m^2\n", rep.inertia_trace);
        std::printf("  initial rate |w|   : %.4f rad/s\n", tumble.norm());
        std::printf("  final   rate |w|   : %.6f rad/s\n", rep.final_rate);
        std::printf("  settled            : %s\n", rep.settled ? "yes" : "no");
        if (rep.settled)
            std::printf("  settle time        : %.2f s\n", rep.settle_time_s);
        std::printf("  thermal ok         : %s\n\n", rep.thermal_ok ? "yes" : "SAFE MODE");
    }

    // --- Deorbit gating ----------------------------------------------------
    {
        bool autonomous = false;
        bool ok = mission.deorbit_permitted(/*ground_human_approval=*/false, autonomous);
        std::printf("[Deorbit] permitted=%s mode=%s\n",
                    ok ? "yes" : "no", autonomous ? "autonomous" : "human-in-the-loop");
    }

    // --- Scenario 4 (WP1): passively-safe relative motion ------------------
    // Sample points around a nominal approach corridor of drift-free safety
    // ellipses, cut thrust at each, and coast for two orbital periods. Report
    // the closest any coast comes to the target (D5 passive-safety property).
    {
        const Config cfg = mission.config();
        const double a = kEarthRadius + cfg.target_altitude_km * 1000.0;
        const CwModel cw = CwModel::from_orbit(a);
        const double keep_out = cfg.keep_out_radius_m;

        std::printf("\n[WP1] LVLH relative motion about target @ %.0f km\n",
                    cfg.target_altitude_km);
        std::printf("  mean motion n      : %.3e rad/s\n", cw.n());
        std::printf("  orbital period     : %.1f s\n", cw.period());
        std::printf("  keep-out radius    : %.1f m\n", keep_out);

        const std::vector<SafetyEllipse> corridor =
            approach_corridor(/*rho_far=*/1200.0, /*rho_near=*/300.0, keep_out,
                              /*n_holds=*/10);

        const double horizon = 2.0 * cw.period();
        const double dt = 1.0;
        const int phases = 12;

        double worst = std::numeric_limits<double>::max();
        int samples = 0;
        for (const SafetyEllipse& hold : corridor) {
            for (int p = 0; p < phases; ++p) {
                const double theta = 2.0 * kPi * static_cast<double>(p) / phases;
                Vector6d x = cw.ellipse_state(hold, theta);
                double local_min = rel_range(x);
                double t = 0.0;
                while (t < horizon) {
                    x = cw.propagate_rk4(x, dt, dt);
                    local_min = std::min(local_min, rel_range(x));
                    t += dt;
                }
                worst = std::min(worst, local_min);
                ++samples;
            }
        }
        std::printf("  corridor holds     : %zu\n", corridor.size());
        std::printf("  thrust-off coasts  : %d samples over 2 periods\n", samples);
        std::printf("  closest approach   : %.1f m -> %s (keep-out %.1f m)\n",
                    worst, worst > keep_out ? "PASSIVELY SAFE" : "VIOLATION",
                    keep_out);
    }

    // --- Scenario 5 (WP2): tumble synchronization ---------------------------
    // A torque-free tumbling target (asymmetric inertia, so the rate vector
    // precesses) tracked by the servicer with the tracking SMC and torque-free
    // feedforward. Fixed initial conditions, same scenario as the acceptance
    // test in tests/test_sync.cpp (R6).
    {
        const Config cfg = mission.config();
        const double deg2rad = kPi / 180.0;

        const Eigen::Quaterniond q_t0 = Eigen::Quaterniond::Identity();
        const Eigen::Vector3d w_t0 =
            cfg.sync_target_rate_deg_s * deg2rad *
            Eigen::Vector3d(0.4, 0.7, -0.59).normalized();
        const Eigen::Quaterniond q_c0(
            Eigen::AngleAxisd(40.0 * deg2rad,
                              Eigen::Vector3d(1.0, 0.5, -0.2).normalized()));

        const SyncReport rep = run_tumble_sync(
            cfg, q_t0, w_t0, q_c0, Eigen::Vector3d::Zero(), 120.0);

        std::printf("\n[WP2] tumble synchronization (target %.1f deg/s, precessing)\n",
                    cfg.sync_target_rate_deg_s);
        std::printf("  target inertia diag: %.2f / %.2f / %.2f kg m^2 (PLACEHOLDER ratios)\n",
                    cfg.target_inertia_diag.x(), cfg.target_inertia_diag.y(),
                    cfg.target_inertia_diag.z());
        std::printf("  initial offset     : 40.0 deg attitude, zero rate\n");
        std::printf("  synced             : %s\n", rep.synced ? "yes" : "NO");
        if (rep.synced) {
            std::printf("  sync achieved at   : %.2f s (criteria then held %.0f s)\n",
                        rep.sync_time_s, cfg.sync_hold_s);
            std::printf("  after dwell        : max |w_rel| %.4f deg/s (tol %.1f), "
                        "max att err %.4f deg (tol %.1f)\n",
                        rep.max_rate_err_deg_s, cfg.sync_rate_tol_deg_s,
                        rep.max_att_err_deg, cfg.sync_att_tol_deg);
        }
        std::printf("  final |w_rel|      : %.5f deg/s\n", rep.final_rate_err_deg_s);
        std::printf("  final att error    : %.5f deg\n", rep.final_att_err_deg);
    }

    std::printf("\n=== simulation complete ===\n");
    return 0;
}
