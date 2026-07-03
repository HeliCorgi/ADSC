// WP5 campaign tests: deterministic seed derivation + run reproducibility,
// Wilson interval edge cases, percentile correctness, campaign termination
// (kit / Delta-v exhaustion), presence of the keep-out-violation rate even when
// zero, CSV schema stability, deterministic future-facing compliance columns,
// and the guardrail that WP5 emits no legal-approval language. Explicit
// return-1 checks (R4); all randomness from fixed seeds (R6).
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "adsc/campaign.hpp"

using namespace adsc;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            return 1;                                                       \
        }                                                                   \
    } while (0)

static std::string slurp(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

static const SummaryRow* find_row(const std::vector<SummaryRow>& rows,
                                  const char* metric) {
    for (const SummaryRow& r : rows) {
        if (r.metric == metric) return &r;
    }
    return nullptr;
}

int main() {
    // 1. Seed derivation: deterministic, index-sensitive, master-sensitive.
    {
        CHECK(splitmix64_seed(123, 0) == splitmix64_seed(123, 0));
        CHECK(splitmix64_seed(123, 0) != splitmix64_seed(123, 1));
        CHECK(splitmix64_seed(123, 5) != splitmix64_seed(124, 5));
        // Non-degenerate spread across the first indices (no collisions).
        bool distinct = true;
        for (int i = 0; i < 32 && distinct; ++i)
            for (int j = i + 1; j < 32; ++j)
                if (splitmix64_seed(999, i) == splitmix64_seed(999, j)) distinct = false;
        CHECK(distinct);
    }

    // 2. Wilson interval edge cases (z = spec constant).
    {
        const double z = kWilsonZ95;
        const Interval none = wilson_interval(0, 100, z);
        CHECK(none.low >= 0.0 && none.low < 1.0e-3);   // clamped near 0, not exactly 0
        CHECK(none.high > 0.02 && none.high < 0.06);   // asymmetric, non-zero upper
        const Interval all = wilson_interval(100, 100, z);
        CHECK(all.high > 0.99 && all.high <= 1.0);
        CHECK(all.low > 0.90 && all.low < 0.97);       // never collapses to [1,1]
        const Interval empty = wilson_interval(0, 0, z);
        CHECK(empty.low == 0.0 && empty.high == 0.0);
        // Interval brackets the point estimate for a mid case.
        const Interval mid = wilson_interval(8, 10, z);
        CHECK(mid.low < 0.8 && mid.high > 0.8);
    }

    // 3. Percentiles (NumPy linear-interpolation convention).
    {
        std::vector<double> v = {1, 2, 3, 4, 5};
        CHECK(std::abs(percentile_sorted(v, 50.0) - 3.0) < 1e-12);
        CHECK(std::abs(percentile_sorted(v, 5.0) - 1.2) < 1e-12);
        CHECK(std::abs(percentile_sorted(v, 95.0) - 4.8) < 1e-12);
        std::vector<double> one = {7.0};
        CHECK(percentile_sorted(one, 5.0) == 7.0 && percentile_sorted(one, 95.0) == 7.0);
        std::vector<double> none;
        CHECK(percentile_sorted(none, 50.0) == 0.0);
    }

    // 4. Neutral actuator is the exact identity, and the 6-arg run_tumble_sync
    //    equals the 7-arg neutral overload bit-for-bit (guards the WP2 pins).
    {
        const Eigen::Vector3d tau(0.013, -0.021, 0.007);
        const Eigen::Vector3d out = ActuatorError{}.apply(tau);
        CHECK(out == tau);   // exact, not approximate

        const Config cfg;
        const double deg2rad = kPi / 180.0;
        const Eigen::Quaterniond q_t0 = Eigen::Quaterniond::Identity();
        const Eigen::Vector3d w_t0 = cfg.sync_target_rate_deg_s * deg2rad *
                                     Eigen::Vector3d(0.4, 0.7, -0.59).normalized();
        const Eigen::Quaterniond q_c0(Eigen::AngleAxisd(
            40.0 * deg2rad, Eigen::Vector3d(1.0, 0.5, -0.2).normalized()));
        const SyncReport a =
            run_tumble_sync(cfg, q_t0, w_t0, q_c0, Eigen::Vector3d::Zero(), 120.0);
        const SyncReport b = run_tumble_sync(cfg, q_t0, w_t0, q_c0,
                                             Eigen::Vector3d::Zero(), 120.0, ActuatorError{});
        CHECK(a.synced == b.synced);
        CHECK(a.sync_time_s == b.sync_time_s);
        CHECK(a.max_rate_err_deg_s == b.max_rate_err_deg_s);
        CHECK(a.max_att_err_deg == b.max_att_err_deg);
        // The quoted WP2 number (README 16.87 s) must not drift through the
        // ActuatorError refactor of run_tumble_sync.
        CHECK(a.synced && std::abs(a.sync_time_s - 16.87) < 0.02);
    }

    const Config base_cfg;

    // 5. Single-mission reproducibility (same catalog + index -> identical run).
    {
        CampaignConfig ccfg;
        const RunResult a = run_one_mission(catalog_A(), ccfg, base_cfg, 7);
        const RunResult b = run_one_mission(catalog_A(), ccfg, base_cfg, 7);
        CHECK(a.run_seed == b.run_seed);
        CHECK(a.outcome == b.outcome);
        CHECK(a.removals == b.removals);
        CHECK(a.dv_used_m_s == b.dv_used_m_s);
        CHECK(a.mission_time_s == b.mission_time_s);
        CHECK(std::string(outcome_label(a.outcome)) == outcome_label(b.outcome));
    }

    // 6. Termination on kit exhaustion: plenty of Delta-v, no aborts, easy sync,
    //    fewer kits than targets -> the mission must end kit_exhausted.
    {
        CampaignConfig ccfg;
        ccfg.n_runs = 1;
        ccfg.targets_per_mission = 6;
        ccfg.kits_initial = 2;
        ccfg.dv_budget_m_s = 1.0e6;              // effectively unlimited
        ccfg.disp_closing_sigma_m_s = 0.0;       // never crosses the abort gate
        ccfg.nominal_closing_m_s = 0.05;         // < max_v_rel
        ccfg.disp_tumble_rate_frac = 0.0;        // nominal, easy-to-sync tumble
        ccfg.disp_tumble_axis_rad = 0.0;
        ccfg.disp_init_att_rad = 0.0;
        ccfg.disp_actuator_scale = 0.0;
        ccfg.disp_actuator_misalign_rad = 0.0;
        const RunResult r = run_one_mission(catalog_A(), ccfg, base_cfg, 0);
        CHECK(r.outcome == Outcome::KitExhausted);
        CHECK(r.removals == 2);
        CHECK(r.kits_remaining == 0);
        CHECK(!r.abort);
    }

    // 7. Termination on Delta-v exhaustion: tiny budget -> dv_exhausted early.
    {
        CampaignConfig ccfg;
        ccfg.n_runs = 1;
        ccfg.targets_per_mission = 6;
        ccfg.kits_initial = 6;
        ccfg.dv_budget_m_s = 5.0;                // below one approach+sync+depart
        ccfg.disp_closing_sigma_m_s = 0.0;
        ccfg.nominal_closing_m_s = 0.05;
        ccfg.disp_tumble_rate_frac = 0.0;
        ccfg.disp_tumble_axis_rad = 0.0;
        ccfg.disp_init_att_rad = 0.0;
        const RunResult r = run_one_mission(catalog_A(), ccfg, base_cfg, 0);
        CHECK(r.outcome == Outcome::DvExhausted);
        CHECK(!r.success);
    }

    // 8. Summary always reports the keep-out-violation rate (even at zero) with
    //    a Wilson CI, and the abort rate is exercised by the closing dispersion.
    {
        CampaignConfig ccfg;
        ccfg.n_runs = 80;
        const std::vector<RunResult> runs = run_campaign(catalog_A(), ccfg, base_cfg);
        const std::vector<SummaryRow> rows = summarize(runs);
        const SummaryRow* ko = find_row(rows, "keep_out_violation_rate");
        const SummaryRow* ab = find_row(rows, "gate_abort_run_rate");
        const SummaryRow* np = find_row(rows, "nonproductive_termination_rate");
        const SummaryRow* sr = find_row(rows, "success_rate");
        CHECK(ko != nullptr && ab != nullptr && np != nullptr && sr != nullptr);
        CHECK(ko->units == "fraction");
        CHECK(ko->estimate >= 0.0);                 // reported even if zero
        CHECK(ko->wilson_high >= ko->estimate);     // CI present
        CHECK(ab->estimate > 0.0);                  // abort-path exposure exercised
        CHECK(ab->wilson_high >= ab->estimate);     // gate_abort_run_rate has a CI
        // nonproductive_termination_rate is the exact complement of success.
        CHECK(std::abs(np->estimate - (1.0 - sr->estimate)) < 1e-12);
        // All seven failure classifications must appear.
        const char* labels[] = {"completed", "dv_exhausted", "kit_exhausted",
                                "keep_out_violation", "gate_abort", "sync_timeout",
                                "other"};
        for (const char* m : labels) CHECK(find_row(rows, m) != nullptr);
    }

    // 9. Campaign-summary reproducibility for a fixed master seed.
    {
        CampaignConfig ccfg;
        ccfg.n_runs = 50;
        const std::vector<SummaryRow> a = summarize(run_campaign(catalog_B(), ccfg, base_cfg));
        const std::vector<SummaryRow> b = summarize(run_campaign(catalog_B(), ccfg, base_cfg));
        CHECK(a.size() == b.size());
        for (std::size_t i = 0; i < a.size(); ++i) {
            CHECK(a[i].metric == b[i].metric);
            CHECK(a[i].estimate == b[i].estimate);
            CHECK(a[i].p50 == b[i].p50);
        }
    }

    // 10. CSV schema stability + deterministic compliance columns + no legal
    //     approval language in any emitted artifact.
    {
        CampaignConfig ccfg;
        ccfg.n_runs = 20;
        CatalogCampaign cc;
        cc.catalog = catalog_A();
        cc.runs = run_campaign(catalog_A(), ccfg, base_cfg);
        std::vector<CatalogCampaign> camps = {cc};

        const std::string runs_p = "test_campaign_runs.csv";
        const std::string sum_p  = "test_campaign_summary.csv";
        const std::string md_p   = "test_campaign_summary.md";
        const std::string sch_p  = "test_campaign_schema.md";
        write_runs_csv(runs_p, camps);
        write_summary_csv(sum_p, ccfg, camps);
        write_summary_md(md_p, ccfg, camps);
        write_schema_md(sch_p);

        const std::string runs_txt = slurp(runs_p);
        const std::string header = runs_txt.substr(0, runs_txt.find('\n'));
        const char* required[] = {
            "schema_version", "catalog", "master_seed", "run_index", "run_seed",
            "outcome", "removals", "targets_attempted", "dv_budget_m_per_s",
            "dv_used_m_per_s", "dv_remaining_m_per_s", "kits_initial", "kits_used",
            "kits_remaining", "sync_time_s", "mission_time_s", "failure_reason",
            "keep_out_violation", "abort", "success", "research_only",
            "class_level_preset", "uses_live_tle",
            "generates_target_specific_operations",
            "requires_owner_consent_assumption", "owner_consent_assumed",
            "proximity_operations_mode", "unconsented_approach_blocked_by_policy",
            "controlled_reentry_mode", "rf_transmitter_modelled",
            "remote_sensing_modelled", "compliance_notes"};
        for (const char* col : required) CHECK(header.find(col) != std::string::npos);

        // Future-facing compliance columns are deterministic (research profile).
        for (const RunResult& r : cc.runs) {
            CHECK(r.research_only && r.class_level_preset && !r.uses_live_tle);
            CHECK(!r.generates_target_specific_operations);
            CHECK(r.requires_owner_consent_assumption && r.owner_consent_assumed);
            CHECK(r.unconsented_approach_blocked_by_policy);
        }

        // Guardrail: no artifact may present a final legal approval conclusion.
        const std::string forbidden[] = {"approved", "licensed", "compliant"};
        const std::string blobs[] = {runs_txt, slurp(sum_p), slurp(md_p), slurp(sch_p)};
        for (const std::string& blob : blobs)
            for (const std::string& word : forbidden)
                CHECK(blob.find(word) == std::string::npos);
    }

    std::printf("campaign: all tests passed\n");
    return 0;
}
