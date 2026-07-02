#include <algorithm>
#include <cstdio>
#include <limits>
#include <vector>

#include "adsc/decay.hpp"
#include "adsc/mission.hpp"

int main() {
    using namespace adsc;

    std::printf("=== ADSC v4 (WP4) — Active Debris Self-Cleanup ===\n\n");

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

    // --- Scenario 6 (WP3): installer mission + kit decay trades -------------
    // The full approach->sync->attach->depart flow for a catalog-A-mass target
    // (GNC phases at the configured reference orbit), then the sail-only decay
    // trade table and the EDT parametric study. All numbers regenerate here.
    {
        const Config cfg = mission.config();
        const double deg2rad = kPi / 180.0;

        const Eigen::Quaterniond q_t0 = Eigen::Quaterniond::Identity();
        const Eigen::Vector3d w_t0 = cfg.sync_target_rate_deg_s * deg2rad *
                                     Eigen::Vector3d(0.4, 0.7, -0.59).normalized();
        const Eigen::Quaterniond q_c0(Eigen::AngleAxisd(
            40.0 * deg2rad, Eigen::Vector3d(1.0, 0.5, -0.2).normalized()));

        const DebrisCatalog A = catalog_A();
        const MissionReport m =
            mission.run_installer_mission(A.mass_kg, q_t0, w_t0, q_c0, 120.0);

        std::printf("\n[WP3] installer mission (target: %s, %.0f kg)\n",
                    A.name, A.mass_kg);
        std::printf("  approach : corridor closest %.1f m %s\n",
                    m.approach_closest_m, m.approach_safe ? "(safe)" : "(UNSAFE)");
        std::printf("  sync     : %s at %.2f s\n",
                    m.synced ? "achieved" : "FAILED", m.sync_time_s);
        std::printf("  attach   : clamp %.2f m/s, contact energy %.3f J; servicer "
                    "%.1f->%.1f kg, target A/m %.4f m^2/kg\n",
                    m.attach.contact_speed_m_s, m.attach.contact_energy_j,
                    m.attach.servicer_mass_before_kg, m.attach.servicer_mass_after_kg,
                    m.attach.target_area_over_mass);
        std::printf("  depart   : bounded coast min %.1f m %s\n",
                    m.depart_coast_min_m, m.departed ? "(safe)" : "(UNSAFE)");
        std::printf("  mission  : %s\n", m.success ? "COMPLETE" : "incomplete");

        // Sail-only decay trade table. T4: report the solar min..max RANGE,
        // never a point value. Years grow with solar-min (thin) atmosphere.
        std::printf("\n[WP3] sail-only decay trade (years to %.0f km; range = "
                    "solar max .. solar min)\n", cfg.reentry_handoff_altitude_km);
        const double areas[] = {25.0, 50.0, 100.0, 200.0, 500.0, 1000.0};
        const DebrisCatalog cats[] = {catalog_A(), catalog_B()};
        for (const DebrisCatalog& c : cats) {
            std::printf("  %s (%.0f kg, %.0f km):\n", c.name, c.mass_kg, c.altitude_km);
            for (double area : areas) {
                const double y_fast = sail_decay_years(
                    c, area, cfg.kit_mass_kg, cfg.drag_cd,
                    cfg.reentry_handoff_altitude_km, cfg.solar_max_density_factor);
                const double y_slow = sail_decay_years(
                    c, area, cfg.kit_mass_kg, cfg.drag_cd,
                    cfg.reentry_handoff_altitude_km, cfg.solar_min_density_factor);
                std::printf("    sail %6.0f m^2 : %9.1f .. %9.1f yr%s\n", area,
                            y_fast, y_slow, y_slow > 25.0 ? "  (>25 at solar min)" : "");
            }
            const double a25_fast = area_for_target_years(
                c, 25.0, cfg.kit_mass_kg, cfg.drag_cd,
                cfg.reentry_handoff_altitude_km, cfg.solar_max_density_factor);
            const double a25_slow = area_for_target_years(
                c, 25.0, cfg.kit_mass_kg, cfg.drag_cd,
                cfg.reentry_handoff_altitude_km, cfg.solar_min_density_factor);
            std::printf("    area for 25-yr guideline: %.0f .. %.0f m^2 "
                        "(solar max .. solar min)\n", a25_fast, a25_slow);
        }
        std::printf("  -> sail-only closes for the lighter/lower class but not for "
                    "the heavy high stage: open trade T1 (sail vs EDT).\n");

        // EDT parametric study (R10 placeholder; a knob, not a claim).
        std::printf("\n[WP3] EDT parametric study for %s (deorbit time vs a-decay knob)\n",
                    A.name);
        const double a0 = kEarthRadius + A.altitude_km * 1000.0;
        const double a_stop = kEarthRadius + cfg.reentry_handoff_altitude_km * 1000.0;
        const double rates[] = {50.0, 150.0, 500.0, 1500.0};  // [m/day]
        for (double rate : rates) {
            const double days = edt_deorbit_days(a0, a_stop, rate);
            std::printf("    delta-a %6.0f m/day : %8.1f days (%.2f yr)\n",
                        rate, days, days / 365.25);
        }
    }

    // --- Scenario 7 (WP4): estimate-driven sync under sensor noise ----------
    // The tracking controller consumes ONLY the EstimatedState produced by the
    // translation EKF + attitude MEKF from noisy sensors (fixed seed, R6);
    // truth is used solely for the acceptance criteria and error statistics.
    // Same constants as tests/test_estimator.cpp.
    {
        const Config cfg = mission.config();
        const double deg2rad = kPi / 180.0;

        const Eigen::Quaterniond q_t0 = Eigen::Quaterniond::Identity();
        const Eigen::Vector3d w_t0 = cfg.sync_target_rate_deg_s * deg2rad *
                                     Eigen::Vector3d(0.4, 0.7, -0.59).normalized();
        const Eigen::Quaterniond q_c0(Eigen::AngleAxisd(
            40.0 * deg2rad, Eigen::Vector3d(1.0, 0.5, -0.2).normalized()));
        const CwModel cw =
            CwModel::from_orbit(kEarthRadius + cfg.target_altitude_km * 1000.0);
        const Vector6d x_trans0 =
            cw.ellipse_state(SafetyEllipse{400.0, 400.0, 0.0}, 0.7);

        const EstimatedSyncReport rep =
            run_estimated_sync(cfg, q_t0, w_t0, q_c0, x_trans0, 120.0);

        std::printf("\n[WP4] estimate-driven tumble sync under sensor noise "
                    "(seed %u)\n", static_cast<unsigned>(cfg.est_seed));
        std::printf("  sensors            : gyro %.0e rad/s, ST %.0e rad @%.0f Hz, "
                    "vision %.0e rad @%.0f Hz, range %.2f m + LOS %.0e @%.0f Hz\n",
                    cfg.gyro_sigma_rad_s, cfg.st_sigma_rad, cfg.st_rate_hz,
                    cfg.vision_sigma_rad, cfg.vision_rate_hz,
                    cfg.range_sigma_m, cfg.los_sigma, cfg.ranging_rate_hz);
        std::printf("  synced (truth)     : %s at %.2f s\n",
                    rep.sync.synced ? "yes" : "NO", rep.sync.sync_time_s);
        if (rep.sync.synced) {
            std::printf("  after dwell        : max |w_rel| %.4f deg/s (tol %.1f), "
                        "max att err %.4f deg (tol %.1f)\n",
                        rep.sync.max_rate_err_deg_s, cfg.sync_rate_tol_deg_s,
                        rep.sync.max_att_err_deg, cfg.sync_att_tol_deg);
        }
        std::printf("  estimation RMS     : own att %.4f deg, rel att %.4f deg, "
                    "w_t %.5f deg/s\n", rep.rms_att_own_deg, rep.rms_att_rel_deg,
                    rep.rms_wt_deg_s);
        std::printf("  translation RMS    : pos %.3f m, vel %.5f m/s "
                    "(final pos err %.3f m)\n",
                    rep.rms_pos_m, rep.rms_vel_m_s, rep.final_pos_m);
        std::printf("  final errors       : rel att %.4f deg, w_t %.5f deg/s\n",
                    rep.final_att_rel_deg, rep.final_wt_deg_s);
        std::printf("  NIS  (consistency) : trans %.3f/4 (N=%d), ST %.3f/3 (N=%d), "
                    "vision %.3f/3 (N=%d)\n",
                    rep.nis_trans_mean, rep.n_trans, rep.nis_st_mean, rep.n_st,
                    rep.nis_vis_mean, rep.n_vis);
        std::printf("  NEES (consistency) : trans %.3f/6, own %.3f/3, rel %.3f/6\n",
                    rep.nees_trans_mean, rep.nees_own_mean, rep.nees_rel_mean);
        std::printf("  P health           : min eig %.3e, max asymmetry %.3e\n",
                    rep.p_min_eig, rep.p_max_asym);
    }

    std::printf("\n=== simulation complete ===\n");
    return 0;
}
