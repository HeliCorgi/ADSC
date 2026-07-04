// WP7 reference-metrics emitter: re-computes the pinned WP1/F1/WP2/WP3/WP4
// reference numbers (already demonstrated by adsc_sim and pinned in the test
// suite) and writes them as machine-readable generated/reference_metrics.csv,
// so the evidence pack can QUOTE them from a committed artifact instead of
// hand-writing any number (R6/D10). Reuses the existing mission/relmotion/
// estimator code paths verbatim -- no physics change, no new behavior. The
// scenario constants are identical to src/main.cpp and the pinned tests,
// EXCEPT the F1 capped-abort case, which is a new demonstration scenario
// defined here (large along-track drift so the impulse cap binds); its numbers
// flow only through this CSV. Deterministic; no wall-clock timestamp (v4.2 R6).
//
//   sim_metrics [out_dir]     (default out_dir = "generated")
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <string>
#include <system_error>
#include <vector>

#include "adsc/campaign.hpp"  // CampaignConfig (plan constants for the pack)
#include "adsc/decay.hpp"
#include "adsc/guidance.hpp"
#include "adsc/mission.hpp"

using namespace adsc;

namespace {
const char* kSchema = "1.0";

struct Row {
    const char* metric;
    double      value;
    const char* units;
    const char* source;
};

// WP11: generated/wp11_guidance_modes.md -- the documented mode machine
// (guidance_mode_table(), the single source of truth also used by
// guidance.cpp) plus this run's flown profile. ASCII, LF, no timestamps
// (R6); ditto fprintf style as write_schema_md (campaign.cpp).
void write_guidance_modes_md(const std::string& path, const Config& cfg,
                             const std::vector<GuidanceModeSpec>& table,
                             const GuidedApproachReport& rep) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "# WP11 closed-loop translation guidance -- mode machine\n\n");
    std::fprintf(f,
        "Deterministic, truth-fed L0 demonstration of the WP11 closed-loop "
        "translation-guidance mode machine (D13/D5): it extends (never "
        "replaces) the WP3 installer-mission phase structure with two-impulse "
        "V-bar hops, a synchronization dwell, a glideslope-with-floor final "
        "approach, contact, and a retreat back to the standoff, all screened "
        "by the WP11 clearing-abort law's reachability rule at every "
        "committed impulse. [L0: linear CW, truth-fed guidance, deterministic]\n\n");

    std::fprintf(f, "## Mode table\n\n");
    std::fprintf(f, "| mode | entry | exit | abort condition | abort action |\n");
    std::fprintf(f, "|---|---|---|---|---|\n");
    for (const GuidanceModeSpec& row : table) {
        std::fprintf(f, "| %s | %s | %s | %s | %s |\n",
                     guidance_mode_label(row.mode), row.entry, row.exit,
                     row.abort_condition, row.abort_action);
    }
    std::fprintf(f, "\n");

    std::fprintf(f, "## Flown profile (this run)\n\n");
    std::fprintf(f,
        "| mode | t_start_s | t_end_s | dv_m_per_s | min_range_m | abort_feasible_throughout |\n");
    std::fprintf(f, "|---|---:|---:|---:|---:|---|\n");
    for (const GuidedPhaseLog& ph : rep.phases) {
        std::fprintf(f, "| %s | %.3f | %.3f | %.5f | %.3f | %s |\n",
                     guidance_mode_label(ph.mode), ph.t_start_s, ph.t_end_s,
                     ph.dv_m_s, ph.min_range_m,
                     ph.abort_feasible_throughout ? "true" : "false");
    }
    std::fprintf(f, "\n");

    std::fprintf(f,
        "Completed: %s. Final mode: %s. Contact speed %.4f m/s (design "
        "%.4f m/s, gate %.4f m/s). Total Delta-v %.4f m/s. Min clearance "
        "outside keep-out %.4f m. Reachability screen held every step: %s. "
        "LOS cone held throughout final approach: %s.\n\n",
        rep.completed ? "true" : "false", guidance_mode_label(rep.final_mode),
        rep.contact_speed_m_s, cfg.guid_contact_speed_m_s, cfg.max_v_rel,
        rep.dv_total_m_s, rep.min_clearance_outside_m,
        rep.abort_feasible_every_step ? "true" : "false",
        rep.los_cone_ok ? "true" : "false");

    std::fprintf(f,
        "Contact speed is produced by the guidance profile "
        "(v = max(floor, k*range)), not merely gated; the WP1-era known "
        "limit is closed at L0.\n\n");

    std::fprintf(f, "## Escalation design note\n\n");
    std::fprintf(f,
        "A pure along-track opening-drift burn (spend the full abort_dv "
        "budget as a single along-track impulse) was evaluated and "
        "REJECTED for the WP11 clearing-abort law's third escalation stage: "
        "its oscillation amplitude (~4*abort_dv/n, km-scale at LEO) swamps "
        "the ~100 m keep-out geometry, so the resulting coast swings back "
        "through near-zero range before the secular drift ever opens the "
        "gap -- measured coast minima of a few meters against a required "
        ">= 0.8*range, for either sign of the escalation. The two-impulse "
        "radial retreat hop (SafeAbort::Status::RetreatHop, mission.hpp) "
        "was adopted instead: it swaps the radial-offset sign via a "
        "half-period hop, landing on a drift-free ellipse whose analytic "
        "minimum clears keep-out + margin, while its own transient leg "
        "opens monotonically for the geometry where it is reached. This is "
        "the negative result required by spec v5 section 10: the rejected "
        "design is documented, not merely discarded.\n\n"
        "[L0: linear CW, truth-fed guidance, deterministic]\n");
    std::fclose(f);
}
}  // namespace

int main(int argc, char** argv) {
    const std::string out_dir = (argc > 1) ? argv[1] : "generated";
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);

    const Config cfg;
    const double deg2rad = kPi / 180.0;
    std::vector<Row> rows;

    // --- WP1: passively-safe corridor thrust-off coast sweep (== adsc_sim
    //     scenario 4: cfg corridor, 12 phases per hold, 2 orbital periods).
    {
        const double a = kEarthRadius + cfg.target_altitude_km * 1000.0;
        const CwModel cw = CwModel::from_orbit(a);
        const std::vector<SafetyEllipse> corridor = approach_corridor(
            cfg.approach_rho_far_m, cfg.approach_rho_near_m,
            cfg.keep_out_radius_m, cfg.approach_holds);
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
        rows.push_back({"wp1_coast_samples", static_cast<double>(samples),
                        "count", "corridor thrust-off sweep, 2 orbital periods"});
        rows.push_back({"wp1_worst_coast_min_range_m", worst, "m",
                        "worst closest approach across all coasts"});
        rows.push_back({"wp1_keep_out_radius_m", cfg.keep_out_radius_m, "m",
                        "Config keep-out sphere"});
    }

    // --- F1: safe-abort coverage, clean and capped cases (same clean case as
    //     adsc_sim scenario 1; capped case = large along-track drift).
    {
        Mission m(cfg);
        const SafeAbort clean = m.compute_safe_abort(
            Eigen::Vector3d(0.08, 0.04, 0.11), Eigen::Vector3d(0.20, 0.10, 0.10));
        rows.push_back({"f1_clean_abort_dv_m_s", clean.dv.norm(), "m_per_s",
                        "contact-range abort impulse (uncapped)"});
        rows.push_back({"f1_clean_coast_min_m", clean.coast_min_range_m, "m",
                        "verified post-burn coast minimum, clean case"});
        rows.push_back({"f1_clean_capped", clean.status == SafeAbort::Status::Capped
                        ? 1.0 : 0.0, "bool", "0 = full impulse delivered"});
        const SafeAbort capped = m.compute_safe_abort(
            Eigen::Vector3d(0.0, -4000.0, 0.0), Eigen::Vector3d(0.0, 3.0, 0.0));
        rows.push_back({"f1_capped_abort_dv_m_s", capped.dv.norm(), "m_per_s",
                        "capped at Config abort_dv"});
        rows.push_back({"f1_capped_coast_min_m", capped.coast_min_range_m, "m",
                        "verified post-burn coast minimum, capped case"});
        rows.push_back({"f1_capped_capped", capped.status == SafeAbort::Status::Capped
                        ? 1.0 : 0.0, "bool", "1 = impulse cap bound, drift remains"});
    }

    // Shared WP2/WP3/WP4 scenario constants (identical to adsc_sim / tests).
    const Eigen::Quaterniond q_t0 = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d w_t0 = cfg.sync_target_rate_deg_s * deg2rad *
                                 Eigen::Vector3d(0.4, 0.7, -0.59).normalized();
    const Eigen::Quaterniond q_c0(Eigen::AngleAxisd(
        40.0 * deg2rad, Eigen::Vector3d(1.0, 0.5, -0.2).normalized()));

    // --- WP2: truth-driven tumble synchronization (pinned demo numbers).
    {
        const SyncReport s = run_tumble_sync(
            cfg, q_t0, w_t0, q_c0, Eigen::Vector3d::Zero(), 120.0);
        rows.push_back({"wp2_sync_time_s", s.sync_time_s, "s",
                        "criteria first held, then dwelled 30 s"});
        rows.push_back({"wp2_max_rate_after_dwell_deg_s", s.max_rate_err_deg_s,
                        "deg_per_s", "max |w_rel| after dwell (tol 0.1)"});
        rows.push_back({"wp2_max_att_after_dwell_deg", s.max_att_err_deg,
                        "deg", "max attitude error after dwell (tol 2.0)"});
    }

    // --- v2 detumble regression (kept as the quoted-number regression pin).
    {
        Mission m(cfg);
        const StabilizationReport r = m.post_capture_stabilization(
            true, 2.4, Eigen::Vector3d(0, 0, 0.15),
            Eigen::Vector3d(0.08, 0.04, 0.11), Eigen::Vector3d(0.12, 0.05, 0.04),
            Eigen::Vector3d(0.15, -0.12, 0.09));
        rows.push_back({"v2_detumble_settle_time_s", r.settle_time_s, "s",
                        "post-capture detumble regression reference"});
    }

    // --- WP3: installer mission for the catalog-A-mass target.
    {
        Mission m(cfg);
        const DebrisCatalog A = catalog_A();
        const MissionReport r =
            m.run_installer_mission(A.mass_kg, q_t0, w_t0, q_c0, 120.0);
        rows.push_back({"wp3_contact_energy_j", r.attach.contact_energy_j, "J",
                        "0.5*m*v^2 at the gated closing speed"});
        rows.push_back({"wp3_contact_speed_m_s", r.attach.contact_speed_m_s,
                        "m_per_s", "Config max_v_rel closing-speed gate"});
        rows.push_back({"wp3_contact_mass_kg", r.attach.servicer_mass_before_kg,
                        "kg", "servicer bus dry + kit at contact"});
        rows.push_back({"wp3_kit_mass_kg", cfg.kit_mass_kg, "kg",
                        "installed deorbit-kit mass (Config)"});
        rows.push_back({"wp3_target_inclination_deg", A.inclination_deg, "deg",
                        "catalog_A class inclination (D2)"});
        rows.push_back({"wp3_approach_closest_m", r.approach_closest_m, "m",
                        "corridor per-hold minimum range"});
        rows.push_back({"wp3_depart_coast_min_m", r.depart_coast_min_m, "m",
                        "post-departure bounded-coast minimum"});
    }

    // --- WP4: estimate-driven synchronization under sensor noise (fixed seed).
    {
        const CwModel cw =
            CwModel::from_orbit(kEarthRadius + cfg.target_altitude_km * 1000.0);
        const Vector6d x_trans0 =
            cw.ellipse_state(SafetyEllipse{400.0, 400.0, 0.0}, 0.7);
        const EstimatedSyncReport r =
            run_estimated_sync(cfg, q_t0, w_t0, q_c0, x_trans0, 120.0);
        rows.push_back({"wp4_sync_time_s", r.sync.sync_time_s, "s",
                        "truth-evaluated sync under estimate-only control"});
        rows.push_back({"wp4_rms_att_rel_deg", r.rms_att_rel_deg, "deg",
                        "relative-attitude estimation RMS"});
        rows.push_back({"wp4_rms_pos_m", r.rms_pos_m, "m",
                        "relative-position estimation RMS"});
        rows.push_back({"wp4_nis_trans", r.nis_trans_mean, "chi2_per_4dof",
                        "translation NIS mean (expect ~4)"});
        rows.push_back({"wp4_nees_trans", r.nees_trans_mean, "chi2_per_6dof",
                        "translation NEES mean (expect ~6, coarse)"});
        rows.push_back({"wp4_seed", static_cast<double>(cfg.est_seed), "seed",
                        "fixed RNG seed (R6)"});
    }

    // --- WP5 campaign plan constants (so the pack never hand-writes them).
    {
        const CampaignConfig ccfg;
        rows.push_back({"campaign_targets_per_mission",
                        static_cast<double>(ccfg.targets_per_mission), "count",
                        "planned targets per mission (CampaignConfig)"});
    }

    // --- WP11: closed-loop translation guidance (D13/D5 extension). ---
    GuidedApproachReport wp11_rep;
    {
        GuidedApproach approach(cfg);
        wp11_rep = approach.fly();
        rows.push_back({"wp11_guided_completed", wp11_rep.completed ? 1.0 : 0.0,
                        "bool", "L0 guided-approach demo, deterministic"});
        rows.push_back({"wp11_guided_contact_speed_m_s", wp11_rep.contact_speed_m_s,
                        "m_per_s",
                        "produced by the glideslope-with-floor profile, not merely gated"});
        rows.push_back({"wp11_guided_dv_total_m_s", wp11_rep.dv_total_m_s,
                        "m_per_s", "sum of |impulse| across the whole demo"});
        rows.push_back({"wp11_guided_min_clearance_outside_m",
                        wp11_rep.min_clearance_outside_m, "m",
                        "min (range - keep_out) over pre-authorization (outside-sphere) flight"});
        rows.push_back({"wp11_abort_feasible_every_step",
                        wp11_rep.abort_feasible_every_step ? 1.0 : 0.0, "bool",
                        "WP11 reachability screen held at every guidance step and hold"});
        rows.push_back({"wp11_los_cone_ok", wp11_rep.los_cone_ok ? 1.0 : 0.0,
                        "bool", "LOS cone about -V-bar held throughout final approach"});
        double retreat_dv = 0.0;
        for (const GuidedPhaseLog& ph : wp11_rep.phases) {
            if (ph.mode == GuidanceMode::Retreat) retreat_dv = ph.dv_m_s;
        }
        rows.push_back({"wp11_retreat_dv_total_m_s", retreat_dv, "m_per_s",
                        "RetreatHop burn1+burn2 (+ station-keep hop if the post-hop "
                        "range fell short of standoff)"});
    }

    std::FILE* f = std::fopen((out_dir + "/reference_metrics.csv").c_str(), "w");
    if (!f) return 1;
    std::fprintf(f, "schema_version,metric,value,units,source\n");
    for (const Row& r : rows) {
        std::fprintf(f, "%s,%s,%.6f,%s,\"%s\"\n", kSchema, r.metric, r.value,
                     r.units, r.source);
    }
    std::fclose(f);
    std::printf("[WP7] wrote %s/reference_metrics.csv (%zu metrics)\n",
                out_dir.c_str(), rows.size());

    write_guidance_modes_md(out_dir + "/wp11_guidance_modes.md", cfg,
                            guidance_mode_table(), wp11_rep);
    std::printf("[WP11] wrote %s/wp11_guidance_modes.md\n", out_dir.c_str());
    return 0;
}
