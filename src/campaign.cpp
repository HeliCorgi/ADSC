#include "adsc/campaign.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

namespace adsc {

// ---------------------------------------------------------------- pure stats
uint64_t splitmix64_seed(uint64_t master_seed, uint64_t index) {
    // Standard SplitMix64 finalizer applied to master_seed advanced by index
    // through the golden-ratio increment. Deterministic and well-mixed even for
    // sequential indices, so per-run streams are independent (R6).
    uint64_t z = master_seed + (index + 1) * 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

Interval wilson_interval(long k, long n, double z) {
    if (n <= 0) return {0.0, 0.0};
    const double nn = static_cast<double>(n);
    const double phat = static_cast<double>(k) / nn;
    const double z2 = z * z;
    const double denom = 1.0 + z2 / nn;
    const double center = (phat + z2 / (2.0 * nn)) / denom;
    const double margin =
        (z / denom) *
        std::sqrt(phat * (1.0 - phat) / nn + z2 / (4.0 * nn * nn));
    double lo = center - margin;
    double hi = center + margin;
    if (lo < 0.0) lo = 0.0;
    if (hi > 1.0) hi = 1.0;
    return {lo, hi};
}

double percentile_sorted(const std::vector<double>& v, double p) {
    if (v.empty()) return 0.0;
    if (v.size() == 1) return v.front();
    const double rank = (p / 100.0) * static_cast<double>(v.size() - 1);
    double lo_f;
    const double frac = std::modf(rank, &lo_f);
    const std::size_t lo = static_cast<std::size_t>(lo_f);
    if (lo + 1 >= v.size()) return v.back();
    return v[lo] + frac * (v[lo + 1] - v[lo]);
}

const char* outcome_label(Outcome o) {
    switch (o) {
        case Outcome::Completed:        return "completed";
        case Outcome::GateAbort:        return "gate_abort";
        case Outcome::SyncTimeout:      return "sync_timeout";
        case Outcome::DvExhausted:      return "dv_exhausted";
        case Outcome::KitExhausted:     return "kit_exhausted";
        case Outcome::KeepOutViolation: return "keep_out_violation";
        case Outcome::Other:            return "other";
    }
    return "other";
}

namespace {

// Deterministic Gaussian source for the campaign dispersions: std::mt19937_64
// (integer sequence pinned by the standard) + explicit Box-Muller. Mirrors the
// estimator's GaussianSource so the same cross-toolchain caveat applies; the
// draw order is fixed by the mission code below (R6).
class CampRng {
public:
    explicit CampRng(uint64_t seed) : gen_(seed) {}

    double sample() {
        if (have_spare_) { have_spare_ = false; return spare_; }
        const double u1 = uniform01();
        const double u2 = uniform01();
        const double r = std::sqrt(-2.0 * std::log(u1));
        const double a = 2.0 * kPi * u2;
        spare_ = r * std::sin(a);
        have_spare_ = true;
        return r * std::cos(a);
    }
    Eigen::Vector3d sample3(double sigma) {
        // Draw in a fixed order: constructor-argument evaluation order is
        // unspecified in C++, so the three samples must be sequenced by hand
        // for the stream to be reproducible across compilers (R6).
        const double x = sample();
        const double y = sample();
        const double z = sample();
        return Eigen::Vector3d(x, y, z) * sigma;
    }

private:
    double uniform01() {
        // 53-bit mantissa in (0, 1]; the +1 avoids log(0) in Box-Muller.
        const uint64_t bits = gen_() >> 11;
        return (static_cast<double>(bits) + 1.0) * (1.0 / 9007199254740992.0);
    }
    std::mt19937_64 gen_;
    bool   have_spare_ = false;
    double spare_      = 0.0;
};

// Dispersed safe-abort coast clearance: command a CW safe abort from the outer
// departure standoff perturbed by the initial-relative-state dispersion, and
// return the propagated minimum range. A capped abort in a large dispersion
// tail is what could pierce the keep-out sphere; with nominal dispersions the
// abort is clean and clears, so the keep-out-violation rate is ~0 (reported
// with the spec's "dispersion may be too narrow" caveat).
double dispersed_abort_coast(const Config& base_cfg, CampRng& rng,
                             double disp_rel_pos_m, double disp_rel_vel_m_s) {
    Mission m(base_cfg);
    const Eigen::Vector3d r0(
        0.0, -base_cfg.depart_standoff_factor * base_cfg.keep_out_radius_m, 0.0);
    const Eigen::Vector3d r = r0 + rng.sample3(disp_rel_pos_m);
    const Eigen::Vector3d v = rng.sample3(disp_rel_vel_m_s);
    return m.compute_safe_abort(r, v).coast_min_range_m;
}

}  // namespace

// -------------------------------------------------------------- campaign core
RunResult run_one_mission(const DebrisCatalog& catalog, const CampaignConfig& ccfg,
                          const Config& base_cfg, int run_index) {
    const double deg2rad = kPi / 180.0;
    const double rad2deg = 180.0 / kPi;

    RunResult r;
    r.catalog     = catalog.name;
    r.master_seed = ccfg.master_seed;
    r.run_index   = run_index;
    r.run_seed    = splitmix64_seed(ccfg.master_seed, static_cast<uint64_t>(run_index));
    r.dv_budget_m_s = ccfg.dv_budget_m_s;
    r.kits_initial  = ccfg.kits_initial;

    CampRng rng(r.run_seed);

    // Run-global metadata dispersions (drawn first so the per-target stream is
    // unaffected by whether they are propagated). solar_factor is recorded for
    // downstream WP3/WP6 decay cost; the sensor-suite realization is drawn to
    // advance the deterministic stream but is not propagated through the
    // truth-driven sync (documented WP5 simplification).
    r.solar_factor = std::max(
        0.05, ccfg.nominal_solar_factor * (1.0 + ccfg.disp_solar_factor_frac * rng.sample()));
    const double sensor_noise_scale =
        std::max(0.0, 1.0 + ccfg.disp_sensor_noise_frac * rng.sample());
    const Eigen::Vector3d sensor_bias = rng.sample3(ccfg.disp_sensor_bias_rad);
    (void)sensor_noise_scale;  // drawn + recorded in schema note; not propagated
    (void)sensor_bias;

    double dv_remaining = ccfg.dv_budget_m_s;
    int    kits         = ccfg.kits_initial;
    double dv_used = 0.0, mission_time = 0.0;
    int    removals = 0, attempted = 0, gate_aborts = 0, sync_timeouts = 0;
    bool   any_abort = false;
    double first_sync_time = -1.0;
    bool   tumble_recorded = false;
    Outcome outcome = Outcome::Completed;  // reached iff the loop runs to the end

    const Eigen::Vector3d nominal_axis =
        Eigen::Vector3d(0.4, 0.7, -0.59).normalized();
    const Eigen::Quaterniond q_c0_nominal(Eigen::AngleAxisd(
        40.0 * deg2rad, Eigen::Vector3d(1.0, 0.5, -0.2).normalized()));

    for (int i = 0; i < ccfg.targets_per_mission; ++i) {
        // Terminal resource checks before committing to target i.
        if (kits == 0) { outcome = Outcome::KitExhausted; break; }
        if (dv_remaining < ccfg.dv_approach_m_s) { outcome = Outcome::DvExhausted; break; }

        ++attempted;
        dv_remaining -= ccfg.dv_approach_m_s; dv_used += ccfg.dv_approach_m_s;

        // --- closing-speed gate (abort exposure) ---
        const double v_close = std::abs(
            ccfg.nominal_closing_m_s + ccfg.disp_closing_sigma_m_s * rng.sample());
        if (i == 0) r.first_closing_speed_m_s = v_close;

        if (v_close > base_cfg.max_v_rel) {
            ++gate_aborts; any_abort = true;
            if (dv_remaining < ccfg.dv_abort_m_s) { outcome = Outcome::DvExhausted; break; }
            dv_remaining -= ccfg.dv_abort_m_s; dv_used += ccfg.dv_abort_m_s;
            const double coast_min = dispersed_abort_coast(
                base_cfg, rng, ccfg.disp_rel_pos_m, ccfg.disp_rel_vel_m_s);
            if (coast_min < base_cfg.keep_out_radius_m) {
                outcome = Outcome::KeepOutViolation; break;
            }
            // Safe abort: move on to the next target (no kit consumed).
        } else {
            if (dv_remaining < ccfg.dv_sync_m_s) { outcome = Outcome::DvExhausted; break; }
            dv_remaining -= ccfg.dv_sync_m_s; dv_used += ccfg.dv_sync_m_s;

            // Dispersed tumble + servicer initial attitude + actuator error.
            const double rate_rad = std::max(0.05 * deg2rad,
                base_cfg.sync_target_rate_deg_s * deg2rad *
                (1.0 + ccfg.disp_tumble_rate_frac * rng.sample()));
            const Eigen::Vector3d axis =
                (nominal_axis + rng.sample3(ccfg.disp_tumble_axis_rad)).normalized();
            const Eigen::Vector3d w_t0 = rate_rad * axis;
            const Eigen::Quaterniond q_c0 =
                (q_c0_nominal * quat_exp(rng.sample3(ccfg.disp_init_att_rad))).normalized();
            ActuatorError act;
            act.scale        = rng.sample3(ccfg.disp_actuator_scale);
            act.misalign_rad = rng.sample3(ccfg.disp_actuator_misalign_rad);

            if (!tumble_recorded) {
                r.tumble_rate_deg_s = rate_rad * rad2deg;
                tumble_recorded = true;
            }

            const SyncReport sr = run_tumble_sync(
                base_cfg, Eigen::Quaterniond::Identity(), w_t0, q_c0,
                Eigen::Vector3d::Zero(), ccfg.sync_time_budget_s, act);

            if (!sr.synced) {
                ++sync_timeouts;
                mission_time += ccfg.sync_time_budget_s;
                // Failed sync: abort this target, move on (no kit consumed).
            } else {
                if (dv_remaining < ccfg.dv_depart_m_s) { outcome = Outcome::DvExhausted; break; }
                dv_remaining -= ccfg.dv_depart_m_s; dv_used += ccfg.dv_depart_m_s;
                --kits; ++removals;
                if (first_sync_time < 0.0) first_sync_time = sr.sync_time_s;
                mission_time += sr.sync_time_s + base_cfg.sync_hold_s +
                                ccfg.t_attach_s + ccfg.t_depart_s;
            }
        }

        // Inter-target phasing to the next target (PLACEHOLDER flat cost).
        if (i + 1 < ccfg.targets_per_mission) {
            if (dv_remaining < ccfg.dv_phasing_m_s) { outcome = Outcome::DvExhausted; break; }
            dv_remaining -= ccfg.dv_phasing_m_s; dv_used += ccfg.dv_phasing_m_s;
            mission_time += ccfg.t_phasing_s;
        }
    }

    r.outcome            = outcome;
    r.removals           = removals;
    r.targets_attempted  = attempted;
    r.dv_used_m_s        = dv_used;
    r.dv_remaining_m_s   = dv_remaining;
    r.kits_used          = ccfg.kits_initial - kits;
    r.kits_remaining     = kits;
    r.sync_time_s        = first_sync_time;  // <0 => none
    r.mission_time_s     = mission_time;
    r.gate_abort_events  = gate_aborts;
    r.sync_timeout_events = sync_timeouts;
    r.success            = (outcome == Outcome::Completed);
    r.abort              = any_abort;
    r.keep_out_violation = (outcome == Outcome::KeepOutViolation);
    r.failure_reason     = r.success ? Outcome::Completed : outcome;
    return r;
}

std::vector<RunResult> run_campaign(const DebrisCatalog& catalog,
                                    const CampaignConfig& ccfg,
                                    const Config& base_cfg) {
    std::vector<RunResult> runs;
    runs.reserve(static_cast<std::size_t>(std::max(0, ccfg.n_runs)));
    for (int i = 0; i < ccfg.n_runs; ++i) {
        runs.push_back(run_one_mission(catalog, ccfg, base_cfg, i));
    }
    return runs;
}

// ------------------------------------------------------------------- summary
std::vector<SummaryRow> summarize(const std::vector<RunResult>& runs) {
    std::vector<SummaryRow> out;
    const long n = static_cast<long>(runs.size());

    auto rate_row = [&](const char* metric, long k) {
        const Interval ci = wilson_interval(k, n, kWilsonZ95);
        SummaryRow row;
        row.metric = metric;
        row.estimate = (n > 0) ? static_cast<double>(k) / static_cast<double>(n) : 0.0;
        row.wilson_low = ci.low;
        row.wilson_high = ci.high;
        row.units = "fraction";
        row.notes = "Wilson 95% CI (z=1.959963984540054)";
        out.push_back(row);
    };
    auto dist_row = [&](const char* metric, std::vector<double> vals,
                        const char* units, const char* notes) {
        std::sort(vals.begin(), vals.end());
        SummaryRow row;
        row.metric = metric;
        double mean = 0.0;
        for (double v : vals) mean += v;
        row.estimate = vals.empty() ? 0.0 : mean / static_cast<double>(vals.size());
        row.p05 = percentile_sorted(vals, 5.0);
        row.p50 = percentile_sorted(vals, 50.0);
        row.p95 = percentile_sorted(vals, 95.0);
        row.units = units;
        row.notes = notes;
        out.push_back(row);
    };
    auto count_row = [&](const char* metric, long count, const char* units,
                         const char* notes) {
        SummaryRow row;
        row.metric = metric;
        row.estimate = static_cast<double>(count);
        row.units = units;
        row.notes = notes;
        out.push_back(row);
    };

    // Rates (with Wilson CIs).
    long n_success = 0, n_abort = 0, n_keepout = 0;
    long c_completed = 0, c_dv = 0, c_kit = 0, c_keepout = 0, c_other = 0;
    long ev_gate = 0, ev_sync = 0;
    std::vector<double> dv_used, kits_used, removals, sync_times, mission_times;
    for (const RunResult& r : runs) {
        if (r.success) ++n_success;
        if (r.abort) ++n_abort;
        if (r.keep_out_violation) ++n_keepout;
        switch (r.outcome) {
            case Outcome::Completed:        ++c_completed; break;
            case Outcome::DvExhausted:      ++c_dv; break;
            case Outcome::KitExhausted:     ++c_kit; break;
            case Outcome::KeepOutViolation: ++c_keepout; break;
            default:                        ++c_other; break;
        }
        ev_gate += r.gate_abort_events;
        ev_sync += r.sync_timeout_events;
        dv_used.push_back(r.dv_used_m_s);
        kits_used.push_back(static_cast<double>(r.kits_used));
        removals.push_back(static_cast<double>(r.removals));
        mission_times.push_back(r.mission_time_s);
        if (r.sync_time_s >= 0.0) sync_times.push_back(r.sync_time_s);
    }

    rate_row("success_rate", n_success);
    rate_row("abort_rate", n_abort);
    rate_row("keep_out_violation_rate", n_keepout);

    dist_row("dv_used_m_per_s", dv_used, "m_per_s", "per mission");
    dist_row("kits_used", kits_used, "count", "per mission");
    dist_row("removals_per_mission", removals, "count", "per mission");
    dist_row("sync_arrival_time_s", sync_times, "s",
             "first successful sync per mission; missions with no sync excluded");
    dist_row("mission_time_s", mission_times, "s",
             "includes PLACEHOLDER phasing/attach/depart time");

    count_row("completed", c_completed, "runs", "terminal outcome");
    count_row("dv_exhausted", c_dv, "runs", "terminal outcome");
    count_row("kit_exhausted", c_kit, "runs", "terminal outcome");
    count_row("keep_out_violation", c_keepout, "runs", "terminal outcome");
    count_row("gate_abort", ev_gate, "events", "per-target safe-abort events");
    count_row("sync_timeout", ev_sync, "events", "per-target failed-sync events");
    count_row("other", c_other, "runs", "unexpected outcome (should be 0)");

    return out;
}

// ------------------------------------------------------------------- output IO
namespace {

const char* bool_str(bool b) { return b ? "true" : "false"; }

// failure_reason column: "none" for a completed success, else the outcome.
std::string failure_reason_str(const RunResult& r) {
    return r.success ? std::string("none") : std::string(outcome_label(r.outcome));
}

}  // namespace

void write_runs_csv(const std::string& path,
                    const std::vector<CatalogCampaign>& campaigns) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f,
        "schema_version,catalog,master_seed,run_index,run_seed,outcome,removals,"
        "targets_attempted,dv_budget_m_per_s,dv_used_m_per_s,dv_remaining_m_per_s,"
        "kits_initial,kits_used,kits_remaining,sync_time_s,mission_time_s,"
        "failure_reason,keep_out_violation,abort,success,gate_abort_events,"
        "sync_timeout_events,first_closing_speed_m_per_s,tumble_rate_deg_per_s,"
        "solar_factor,research_only,class_level_preset,uses_live_tle,"
        "generates_target_specific_operations,requires_owner_consent_assumption,"
        "owner_consent_assumed,proximity_operations_mode,"
        "unconsented_approach_blocked_by_policy,controlled_reentry_mode,"
        "rf_transmitter_modelled,remote_sensing_modelled,compliance_notes\n");
    for (const CatalogCampaign& cc : campaigns) {
        for (const RunResult& r : cc.runs) {
            char sync_buf[32];
            if (r.sync_time_s >= 0.0) std::snprintf(sync_buf, sizeof(sync_buf), "%.4f", r.sync_time_s);
            else std::snprintf(sync_buf, sizeof(sync_buf), "%s", "");  // blank = no sync
            std::fprintf(f,
                "%s,%s,%llu,%d,%llu,%s,%d,%d,%.4f,%.4f,%.4f,%d,%d,%d,%s,%.3f,%s,"
                "%s,%s,%s,%d,%d,%.5f,%.5f,%.5f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,\"%s\"\n",
                wp5_schema_version(), r.catalog.c_str(),
                static_cast<unsigned long long>(r.master_seed), r.run_index,
                static_cast<unsigned long long>(r.run_seed), outcome_label(r.outcome),
                r.removals, r.targets_attempted, r.dv_budget_m_s, r.dv_used_m_s,
                r.dv_remaining_m_s, r.kits_initial, r.kits_used, r.kits_remaining,
                sync_buf, r.mission_time_s, failure_reason_str(r).c_str(),
                bool_str(r.keep_out_violation), bool_str(r.abort), bool_str(r.success),
                r.gate_abort_events, r.sync_timeout_events, r.first_closing_speed_m_s,
                r.tumble_rate_deg_s, r.solar_factor, bool_str(r.research_only),
                bool_str(r.class_level_preset), bool_str(r.uses_live_tle),
                bool_str(r.generates_target_specific_operations),
                bool_str(r.requires_owner_consent_assumption),
                bool_str(r.owner_consent_assumed), r.proximity_operations_mode.c_str(),
                bool_str(r.unconsented_approach_blocked_by_policy),
                bool_str(r.controlled_reentry_mode), bool_str(r.rf_transmitter_modelled),
                bool_str(r.remote_sensing_modelled), r.compliance_notes.c_str());
        }
    }
    std::fclose(f);
}

void write_summary_csv(const std::string& path, const CampaignConfig& ccfg,
                       const std::vector<CatalogCampaign>& campaigns) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f,
        "schema_version,catalog,master_seed,n_runs,metric,estimate,wilson_low,"
        "wilson_high,p05,p50,p95,units,source_runs_csv,notes\n");
    for (const CatalogCampaign& cc : campaigns) {
        const std::vector<SummaryRow> rows = summarize(cc.runs);
        for (const SummaryRow& row : rows) {
            std::fprintf(f,
                "%s,%s,%llu,%d,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%s,\"%s\"\n",
                wp5_schema_version(), cc.catalog.name,
                static_cast<unsigned long long>(ccfg.master_seed),
                static_cast<int>(cc.runs.size()), row.metric.c_str(), row.estimate,
                row.wilson_low, row.wilson_high, row.p05, row.p50, row.p95,
                row.units.c_str(), "wp5_campaign_runs.csv", row.notes.c_str());
        }
    }
    std::fclose(f);
}

void write_summary_md(const std::string& path, const CampaignConfig& ccfg,
                      const std::vector<CatalogCampaign>& campaigns) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "# WP5 Campaign Monte Carlo — summary\n\n");
    std::fprintf(f,
        "Master seed `0x%llX`, %d runs per catalog preset. All rates carry a "
        "Wilson 95%% confidence interval (z = %.15g). Distributions are reported "
        "as 5th / 50th / 95th percentiles. Regenerate with `adsc_campaign`.\n\n"
        "WP5 performs **no** legal or regulatory determination and produces **no** "
        "visualization; the compliance columns are passive research-profile "
        "metadata for future WP7/WP8 tooling.\n\n",
        static_cast<unsigned long long>(ccfg.master_seed), ccfg.n_runs, kWilsonZ95);

    for (const CatalogCampaign& cc : campaigns) {
        const std::vector<SummaryRow> rows = summarize(cc.runs);
        std::fprintf(f, "## %s\n\n", cc.catalog.name);
        std::fprintf(f, "| metric | estimate | 95%% CI / p05..p95 | units | notes |\n");
        std::fprintf(f, "|---|---:|---|---|---|\n");
        for (const SummaryRow& row : rows) {
            char detail[128];
            if (row.units == std::string("fraction")) {
                std::snprintf(detail, sizeof(detail), "[%.4f, %.4f]",
                              row.wilson_low, row.wilson_high);
            } else if (row.units == std::string("runs") ||
                       row.units == std::string("events")) {
                std::snprintf(detail, sizeof(detail), "%s", "-");
            } else {
                std::snprintf(detail, sizeof(detail), "%.3f .. %.3f .. %.3f",
                              row.p05, row.p50, row.p95);
            }
            std::fprintf(f, "| %s | %.4f | %s | %s | %s |\n", row.metric.c_str(),
                         row.estimate, detail, row.units.c_str(), row.notes.c_str());
        }
        std::fprintf(f, "\n");
    }
    std::fprintf(f,
        "Note: `gate_abort` / `sync_timeout` are per-target *event* counts (a "
        "safe abort or failed sync lets the servicer move to the next target); "
        "`completed` / `dv_exhausted` / `kit_exhausted` / `keep_out_violation` "
        "are per-*run* terminal outcomes. If `abort_rate` is 0 the closing-speed "
        "dispersion may be too narrow or the 0.15 m/s gate is not exercised; if "
        "`keep_out_violation_rate` is 0 the abort maneuvers are clearing the "
        "keep-out sphere (expected under nominal dispersions).\n");
    std::fclose(f);
}

void write_schema_md(const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f,
"# WP5 Campaign CSV schema (version 1.0)\n"
"\n"
"Stable machine-readable outputs for downstream WP6 (cost/FoM), WP7\n"
"(visualization/evidence), and WP8 (compliance matrix). The `schema_version`\n"
"column is `1.0`; do not change column meanings without bumping it. WP5 itself\n"
"performs **no** legal/regulatory determination and produces **no** charts.\n"
"\n"
"## Files\n"
"\n"
"- `wp5_campaign_runs.csv` — one row per mission run (WP5-native raw records).\n"
"- `wp5_campaign_summary.csv` — one row per metric per catalog preset.\n"
"- `wp5_campaign_summary.md` — human-readable rendering of the summary.\n"
"- `wp5_campaign_schema.md` — this document.\n"
"\n"
"## `wp5_campaign_runs.csv`\n"
"\n"
"| column | units | kind | notes |\n"
"|---|---|---|---|\n"
"| schema_version | - | WP5-native | schema id (`1.0`) |\n"
"| catalog | - | WP5-native | class-level preset name (never an object id) |\n"
"| master_seed | - | WP5-native | fixed campaign master seed |\n"
"| run_index | - | WP5-native | 0-based mission index |\n"
"| run_seed | - | WP5-native | SplitMix64(master_seed, run_index) |\n"
"| outcome | - | WP5-native | terminal: completed/dv_exhausted/kit_exhausted/keep_out_violation/other |\n"
"| removals | count | WP5-native | targets removed this mission |\n"
"| targets_attempted | count | WP5-native | targets processed before termination |\n"
"| dv_budget_m_per_s | m/s | WP5-native, PLACEHOLDER | mission Delta-v budget |\n"
"| dv_used_m_per_s | m/s | WP5-native, PLACEHOLDER-derived | sum of leg costs |\n"
"| dv_remaining_m_per_s | m/s | WP5-native, PLACEHOLDER-derived | budget minus used |\n"
"| kits_initial | count | WP5-native | starting kit inventory |\n"
"| kits_used | count | WP5-native | kits installed (= removals) |\n"
"| kits_remaining | count | WP5-native | inventory at termination |\n"
"| sync_time_s | s | WP5-native | first successful sync declare time; blank if none |\n"
"| mission_time_s | s | WP5-native, PLACEHOLDER-derived | elapsed incl. placeholder phasing |\n"
"| failure_reason | - | WP5-native | `none` if success, else the terminal outcome |\n"
"| keep_out_violation | bool | WP5-native | terminal keep-out breach occurred |\n"
"| abort | bool | WP5-native | >=1 safe-abort (closing-speed gate) event occurred |\n"
"| success | bool | WP5-native | outcome == completed |\n"
"| gate_abort_events | count | WP5-native | per-target safe-abort events this run |\n"
"| sync_timeout_events | count | WP5-native | per-target failed-sync events this run |\n"
"| first_closing_speed_m_per_s | m/s | WP5-native, PLACEHOLDER-derived | target-0 capture closing speed vs 0.15 gate |\n"
"| tumble_rate_deg_per_s | deg/s | WP5-native, PLACEHOLDER-derived | realized |w_t| of first synced attempt |\n"
"| solar_factor | - | WP5-native, PLACEHOLDER-derived | realized atmospheric-density factor (for WP3/WP6) |\n"
"| research_only | bool | future-facing WP8 | passive; always true for the research profile |\n"
"| class_level_preset | bool | future-facing WP8 | passive; targets are class parameters, not object ids |\n"
"| uses_live_tle | bool | future-facing WP8 | passive; always false (no live ephemeris, R13) |\n"
"| generates_target_specific_operations | bool | future-facing WP8 | passive; always false |\n"
"| requires_owner_consent_assumption | bool | future-facing WP8 | passive; always true |\n"
"| owner_consent_assumed | bool | future-facing WP8 | passive; research-scenario ASSUMPTION only, not a legal fact |\n"
"| proximity_operations_mode | - | future-facing WP8 | passive descriptor string |\n"
"| unconsented_approach_blocked_by_policy | bool | future-facing WP8 | passive; always true |\n"
"| controlled_reentry_mode | bool | future-facing WP8 | passive; false = passive drag-sail baseline |\n"
"| rf_transmitter_modelled | bool | future-facing WP8 | passive; always false |\n"
"| remote_sensing_modelled | bool | future-facing WP8 | passive; always false |\n"
"| compliance_notes | - | future-facing WP8 | passive; states WP5 makes no legal/regulatory determination |\n"
"\n"
"**Legal-interpretation caveat.** The compliance columns are deterministic\n"
"research-profile metadata. `owner_consent_assumed = true` is a *research\n"
"scenario assumption* (D9), never a statement that consent legally exists. No\n"
"column may be read as regulatory approval; WP5 performs no such analysis.\n"
"\n"
"## `wp5_campaign_summary.csv`\n"
"\n"
"| column | units | notes |\n"
"|---|---|---|\n"
"| schema_version | - | schema id (`1.0`) |\n"
"| catalog | - | class-level preset name |\n"
"| master_seed | - | fixed campaign master seed |\n"
"| n_runs | count | runs aggregated for this catalog |\n"
"| metric | - | metric name (see rows below) |\n"
"| estimate | mixed | rate fraction, distribution mean, or classification count |\n"
"| wilson_low | fraction | rate rows only: Wilson 95%% lower bound |\n"
"| wilson_high | fraction | rate rows only: Wilson 95%% upper bound |\n"
"| p05 | mixed | distribution rows only: 5th percentile |\n"
"| p50 | mixed | distribution rows only: 50th percentile |\n"
"| p95 | mixed | distribution rows only: 95th percentile |\n"
"| units | - | fraction / count / m_per_s / s / runs / events |\n"
"| source_runs_csv | - | provenance: wp5_campaign_runs.csv |\n"
"| notes | - | per-metric note |\n"
"\n"
"### Summary metric rows (per catalog)\n"
"\n"
"Rates (with Wilson 95%% CI): `success_rate`, `abort_rate`,\n"
"`keep_out_violation_rate`. Distributions (p05/p50/p95): `dv_used_m_per_s`,\n"
"`kits_used`, `removals_per_mission`, `sync_arrival_time_s`, `mission_time_s`.\n"
"Failure classification counts: `completed`, `dv_exhausted`, `kit_exhausted`,\n"
"`keep_out_violation` (runs) and `gate_abort`, `sync_timeout` (per-target\n"
"events), plus `other` (should be 0).\n"
"\n"
"### Future visualization (WP7) — not implemented here\n"
"\n"
"The schema supports, without change: outcome proportions, Wilson intervals,\n"
"Delta-v / removals / sync-time percentiles, keep-out-violation rate, and\n"
"failure-classification bars. WP5 emits none of these charts.\n");
    std::fclose(f);
}

}  // namespace adsc
