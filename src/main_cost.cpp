// WP6 cost/FoM driver: regenerates generated/wp6_cost_* and prints the
// amortization curves, FoM table and tornado ranking (evidence for the PR /
// docs/cost_model.md, R6/R12; WP15: this moved out of README).
//
//   adsc_cost [n_runs] [out_dir]
//
// Defaults: n_runs = CampaignConfig::n_runs (500), out_dir = "generated".
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "adsc/campaign.hpp"
#include "adsc/cost.hpp"
#include "adsc/decay.hpp"

using namespace adsc;

int main(int argc, char** argv) {
    CampaignConfig ccfg;
    if (argc > 1) {
        const int n = std::atoi(argv[1]);
        if (n > 0) ccfg.n_runs = n;
    }
    const std::string out_dir = (argc > 2) ? argv[2] : "generated";
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);

    // Base vehicle config MUST match the WP5 driver (main_campaign.cpp) so the
    // baseline kit count reproduces the committed WP5 campaign exactly: same
    // coarsened abort keep-out screen.
    Config base_cfg;
    base_cfg.abort_coast_check_periods = 1.0;
    base_cfg.abort_coast_check_dt_s    = 8.0;

    const CostConfig cc;
    const std::vector<DebrisCatalog> cats = {catalog_A(), catalog_B()};

    std::printf("=== ADSC v4 (WP6) — parametric cost model + FoM ===\n");
    std::printf("relative cost units (CU); NO point-value dollar figure claimed. "
                "%d runs/catalog, kit sweep %d..%d, baseline N=%d\n",
                ccfg.n_runs, cc.sweep_kit_min, cc.sweep_kit_max, ccfg.kits_initial);

    const auto t0 = std::chrono::steady_clock::now();
    CostReport report = compute_cost_report(cats, ccfg, base_cfg, cc);
    const auto t1 = std::chrono::steady_clock::now();
    report.elapsed_s = std::chrono::duration<double>(t1 - t0).count();

    // Consume the committed WP5 summary (schema 1.0): cross-check the baseline.
    const std::string wp5 = out_dir + "/wp5_campaign_summary.csv";
    for (const CatalogCost& cq : report.catalogs) {
        double base_rem = 0.0;
        for (const AmortPoint& a : cq.amortization)
            if (a.n_kits == cq.baseline_n_kits) base_rem = a.removals_p50;
        const double wp5_rem =
            read_wp5_summary_value(wp5, cq.catalog.name, "removals_per_mission", "p50");
        if (wp5_rem < 0.0)
            std::printf("[WP6] %s: WP5 summary not found -- skipping baseline cross-check\n",
                        cq.catalog.name);
        else
            std::printf("[WP6] %s: baseline removals p50 %.1f vs WP5 CSV %.1f -> %s "
                        "(schema 1.0 consumed)\n", cq.catalog.name, base_rem, wp5_rem,
                        (base_rem == wp5_rem) ? "MATCH" : "DIFFER");
    }

    write_cost_summary_csv(out_dir + "/wp6_cost_summary.csv", cc, report);
    write_cost_summary_md(out_dir + "/wp6_cost_summary.md", cc, report);
    write_cost_schema_md(out_dir + "/wp6_cost_schema.md");

    for (const CatalogCost& cq : report.catalogs) {
        std::printf("\n[WP6] %s  (amortization: cost/removal vs kits carried N)\n",
                    cq.catalog.name);
        for (const AmortPoint& a : cq.amortization)
            std::printf("  N=%d : cost/removal p50 %.2f CU  (p05..p95 %.2f..%.2f)  "
                        "ratio-to-N%d %.3f  removals p50 %.1f\n",
                        a.n_kits, a.cost_per_removal_p50, a.cost_per_removal_p05,
                        a.cost_per_removal_p95, cc.sweep_kit_min, a.ratio_to_n1_p50,
                        a.removals_p50);
        std::printf("  FoM at N=%d:\n", cq.baseline_n_kits);
        for (const FoMResult& fr : cq.fom)
            std::printf("    %-16s w(h)=%.3f  removed mass %.0f kg  "
                        "FoM p50 %.4f kg/CU  (p05..p95 %.4f..%.4f)\n",
                        weighting_label(fr.weighting), fr.band_weight,
                        fr.removed_mass_kg_p50, fr.fom_p50, fr.fom_p05, fr.fom_p95);
    }

    std::printf("\n[WP6] tornado (%s, baseline N, +/-%.0f%%), ranked by cost/removal swing:\n",
                report.tornado_catalog.c_str(), cc.tornado_delta_frac * 100.0);
    int rank = 1;
    for (const TornadoRow& tr : report.tornado)
        std::printf("  %d. %-20s cost/removal swing %.2f CU  |  spatial FoM swing %.5f kg/CU\n",
                    rank++, tr.param.c_str(), tr.cost_per_removal_swing, tr.fom_swing);

    std::printf("\nwrote %s/wp6_cost_summary.{csv,md} + wp6_cost_schema.md\n",
                out_dir.c_str());
    std::printf("cost-report runtime: %.2f s (%d runs x %zu catalogs x %d kit-sweep)\n",
                report.elapsed_s, ccfg.n_runs, cats.size(),
                cc.sweep_kit_max - cc.sweep_kit_min + 1);
    std::printf("=== WP6 cost model complete ===\n");
    return 0;
}
