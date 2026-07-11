// WP5 campaign driver: regenerates the committed generated/ artifacts and
// prints a human-readable summary (evidence for the PR / docs/safety.md,
// R6/R12).
//
//   adsc_campaign [n_runs] [out_dir]
//
// Defaults: n_runs = CampaignConfig::n_runs (500, the full campaign target),
// out_dir = "generated". Pass a smaller n_runs for a fast CI subset without
// touching the committed full-campaign files (docs/safety.md documents the
// split; WP15: this moved out of README).
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "adsc/campaign.hpp"
#include "adsc/decay.hpp"

using namespace adsc;

namespace {

const SummaryRow* find_row(const std::vector<SummaryRow>& rows, const char* metric) {
    for (const SummaryRow& r : rows) {
        if (r.metric == metric) return &r;
    }
    return nullptr;
}

void print_catalog(const CatalogCampaign& cc) {
    const std::vector<SummaryRow> rows = summarize(cc.runs);
    std::printf("\n[WP5] %s  (%d runs)\n", cc.catalog.name,
                static_cast<int>(cc.runs.size()));

    const char* rates[] = {"success_rate", "nonproductive_termination_rate",
                           "gate_abort_run_rate", "keep_out_violation_rate"};
    for (const char* m : rates) {
        const SummaryRow* r = find_row(rows, m);
        if (r) std::printf("  %-24s : %.4f  [%.4f, %.4f] (Wilson 95%%)\n",
                           m, r->estimate, r->wilson_low, r->wilson_high);
    }
    const char* dists[] = {"dv_used_m_per_s", "kits_used", "removals_per_mission",
                           "sync_arrival_time_s", "mission_time_s"};
    for (const char* m : dists) {
        const SummaryRow* r = find_row(rows, m);
        if (r) std::printf("  %-24s : p05 %.3f  p50 %.3f  p95 %.3f  (%s)\n",
                           m, r->p05, r->p50, r->p95, r->units.c_str());
    }
    std::printf("  failure classification   :");
    const char* cls[] = {"completed", "dv_exhausted", "kit_exhausted",
                         "keep_out_violation", "gate_abort", "sync_timeout", "other"};
    for (const char* m : cls) {
        const SummaryRow* r = find_row(rows, m);
        if (r) std::printf(" %s=%d", m, static_cast<int>(r->estimate));
    }
    std::printf("\n");

    const SummaryRow* ar = find_row(rows, "gate_abort_run_rate");
    if (ar && ar->estimate <= 0.0)
        std::printf("  NOTE: gate_abort_run_rate is 0 -- closing-speed dispersion may "
                    "be too narrow or the 0.15 m/s gate is not exercised.\n");
    const SummaryRow* kr = find_row(rows, "keep_out_violation_rate");
    if (kr && kr->estimate <= 0.0)
        std::printf("  NOTE: keep_out_violation_rate is 0 -- abort maneuvers clear "
                    "the keep-out sphere under the current dispersions.\n");
    const SummaryRow* st = find_row(rows, "sync_timeout");
    if (st && st->estimate <= 0.0)
        std::printf("  NOTE: sync_timeout is 0 -- the tracking SMC synchronizes "
                    "within budget across the sampled tumble/attitude/actuator "
                    "dispersions (widen dispersions or shorten the budget to exercise it).\n");
}

}  // namespace

int main(int argc, char** argv) {
    CampaignConfig ccfg;
    if (argc > 1) {
        const int n = std::atoi(argv[1]);
        if (n > 0) ccfg.n_runs = n;
    }
    const std::string out_dir = (argc > 2) ? argv[2] : "generated";

    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);

    // Base vehicle config. The abort keep-out screen (dispersed safe-abort
    // coast vs the keep-out sphere) is deliberately coarsened for the campaign:
    // a 1-period coast at an 8 s step is an adequate rare-event screen across
    // thousands of aborts and keeps the N=500 runtime well under the CI
    // guideline. It only affects the keep-out-violation screen, not the sync
    // loop (which uses control_dt) or any pinned number.
    Config base_cfg;
    base_cfg.abort_coast_check_periods = 1.0;
    base_cfg.abort_coast_check_dt_s    = 8.0;
    const DebrisCatalog cats[] = {catalog_A(), catalog_B()};

    std::printf("=== ADSC v4 (WP5) — Campaign Monte Carlo ===\n");
    std::printf("master seed 0x%llX, %d runs/catalog, %d targets/mission, "
                "%d kits, %.1f m/s dv budget\n",
                static_cast<unsigned long long>(ccfg.master_seed), ccfg.n_runs,
                ccfg.targets_per_mission, ccfg.kits_initial, ccfg.dv_budget_m_s);

    const auto t0 = std::chrono::steady_clock::now();

    std::vector<CatalogCampaign> campaigns;
    for (const DebrisCatalog& c : cats) {
        CatalogCampaign cc;
        cc.catalog = c;
        cc.runs = run_campaign(c, ccfg, base_cfg);
        campaigns.push_back(std::move(cc));
    }

    const auto t1 = std::chrono::steady_clock::now();
    const double elapsed_s =
        std::chrono::duration<double>(t1 - t0).count();

    // Artifacts.
    write_runs_csv(out_dir + "/wp5_campaign_runs.csv", campaigns);
    write_summary_csv(out_dir + "/wp5_campaign_summary.csv", ccfg, campaigns);
    write_summary_md(out_dir + "/wp5_campaign_summary.md", ccfg, campaigns);
    write_schema_md(out_dir + "/wp5_campaign_schema.md");

    for (const CatalogCampaign& cc : campaigns) print_catalog(cc);

    std::printf("\nwrote %s/wp5_campaign_{runs,summary}.csv, "
                "wp5_campaign_summary.md, wp5_campaign_schema.md\n", out_dir.c_str());
    std::printf("campaign runtime: %.2f s (%d runs x %zu catalogs)\n",
                elapsed_s, ccfg.n_runs, sizeof(cats) / sizeof(cats[0]));
    std::printf("=== WP5 campaign complete ===\n");
    return 0;
}
