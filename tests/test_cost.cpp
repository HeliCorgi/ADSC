// WP6 tests: cost-model determinism, congestion-weight interpolation + the T5
// band-ranking flip, FoM formula, amortization monotonicity (until the Δv
// budget bounds removals), per-run quantile propagation, CSV schema stability,
// the WP5-summary consumer, report determinism, and the guardrail that WP6
// emits no absolute-currency figure. Explicit return-1 checks (R4); fixed seeds
// (R6). Existing regression tests are unaffected (WP1-WP5 code is untouched).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "adsc/cost.hpp"

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

static const AmortPoint* at_n(const CatalogCost& cq, int n) {
    for (const AmortPoint& a : cq.amortization)
        if (a.n_kits == n) return &a;
    return nullptr;
}

int main() {
    const Config base;
    const CostConfig cc;

    // 1. Congestion-weight table: exact at anchors, linear between, and the T5
    //    flip (spatial favors the lower band, criticality the higher band).
    {
        CHECK(std::abs(congestion_weight(Weighting::SpatialDensity, 750.0, cc) - 1.00) < 1e-9);
        CHECK(std::abs(congestion_weight(Weighting::SpatialDensity, 840.0, cc) - 0.78) < 1e-9);
        CHECK(std::abs(congestion_weight(Weighting::Criticality, 750.0, cc) - 0.68) < 1e-9);
        CHECK(std::abs(congestion_weight(Weighting::Criticality, 840.0, cc) - 1.00) < 1e-9);
        // linear midpoint between 800 (0.92) and 840 (0.78) spatial -> 0.85.
        CHECK(std::abs(congestion_weight(Weighting::SpatialDensity, 820.0, cc) - 0.85) < 1e-9);
        // clamp below/above the table.
        CHECK(congestion_weight(Weighting::SpatialDensity, 100.0, cc) ==
              congestion_weight(Weighting::SpatialDensity, 600.0, cc));
        // T5: the two weightings rank the two catalog bands oppositely.
        const double sp750 = congestion_weight(Weighting::SpatialDensity, 750.0, cc);
        const double sp840 = congestion_weight(Weighting::SpatialDensity, 840.0, cc);
        const double cr750 = congestion_weight(Weighting::Criticality, 750.0, cc);
        const double cr840 = congestion_weight(Weighting::Criticality, 840.0, cc);
        CHECK(sp750 > sp840);   // spatial: lower band wins
        CHECK(cr840 > cr750);   // criticality: higher band wins
    }

    // 2. Cost-model determinism + component sum + FoM formula.
    {
        const DebrisCatalog A = catalog_A();
        const double cost0 = mission_campaign_cost(cc, base, A, 4, 0.0);
        // Sum of the five components (mission_time 0 => C_ops 0).
        const double m_dry = base.dry_mass_kg;
        const double m_wet = base.dry_mass_kg + base.initial_fuel_kg + 4 * base.kit_mass_kg;
        const double expect =
            cc.c_dev_cu + cc.c_bus_cu * std::pow(m_dry, cc.c_bus_exponent) +
            4 * cc.c_kit_cu +
            cc.c_launch_cu_per_kg * m_wet *
                launch_band_factor(cc, A.altitude_km, A.inclination_deg);
        CHECK(std::abs(cost0 - expect) < 1e-9);
        CHECK(cost0 == mission_campaign_cost(cc, base, A, 4, 0.0));  // deterministic
        // C_ops grows with mission time.
        CHECK(mission_campaign_cost(cc, base, A, 4, 2.0 * 86400.0) >
              cost0 + 1e-9);
        // FoM formula: removed_mass * w / cost.
        const double w = congestion_weight(Weighting::SpatialDensity, A.altitude_km, cc);
        const double removed_mass = 4.0 * A.mass_kg;
        const double fom = removed_mass * w / cost0;
        CHECK(fom > 0.0);
        CHECK(std::abs(fom - (removed_mass * w / cost0)) < 1e-9);
    }

    // Shared small-but-representative campaign config for the report tests.
    Config rbase = base;
    rbase.abort_coast_check_periods = 1.0;   // coarse keep-out screen (fast)
    rbase.abort_coast_check_dt_s    = 8.0;
    CampaignConfig ccfg;
    ccfg.n_runs = 80;
    CostConfig rcc = cc;
    rcc.sweep_kit_max = 5;                    // 1..5 keeps the test quick

    const CostReport report =
        compute_cost_report({catalog_A(), catalog_B()}, ccfg, rbase, rcc);

    // 3. Amortization monotonicity while removals still grow with N (N=1..3),
    //    and batch amortization strictly beats the single-target baseline.
    {
        CHECK(report.catalogs.size() == 2);
        for (const CatalogCost& cq : report.catalogs) {
            const AmortPoint* a1 = at_n(cq, 1);
            const AmortPoint* a2 = at_n(cq, 2);
            const AmortPoint* a3 = at_n(cq, 3);
            CHECK(a1 && a2 && a3);
            CHECK(a1->cost_per_removal_p50 > a2->cost_per_removal_p50);
            CHECK(a2->cost_per_removal_p50 > a3->cost_per_removal_p50);
            CHECK(std::abs(a1->ratio_to_n1_p50 - 1.0) < 1e-12);  // N=1 baseline
            // the minimum cost/removal over the sweep beats the single-target one.
            double best = a1->cost_per_removal_p50;
            for (const AmortPoint& a : cq.amortization)
                best = std::min(best, a.cost_per_removal_p50);
            CHECK(best < a1->cost_per_removal_p50);
            CHECK(a2->ratio_to_n1_p50 < 1.0);
        }
    }

    // 4. Distribution propagation: percentiles are ordered and carry real
    //    spread (not a mean-only collapse), for cost/removal and FoM.
    {
        const CatalogCost& cq = report.catalogs.front();
        bool any_spread = false;
        for (const AmortPoint& a : cq.amortization) {
            CHECK(a.cost_per_removal_p05 <= a.cost_per_removal_p50 + 1e-9);
            CHECK(a.cost_per_removal_p50 <= a.cost_per_removal_p95 + 1e-9);
            if (a.cost_per_removal_p95 - a.cost_per_removal_p05 > 1e-6) any_spread = true;
        }
        CHECK(any_spread);
        for (const FoMResult& fr : cq.fom) {
            CHECK(fr.fom_p05 <= fr.fom_p50 + 1e-9);
            CHECK(fr.fom_p50 <= fr.fom_p95 + 1e-9);
        }
        // Two weightings reported; band weights differ (T5 surfaced in FoM).
        CHECK(cq.fom.size() == 2);
        CHECK(std::abs(cq.fom[0].band_weight - cq.fom[1].band_weight) > 1e-9);
    }

    // 5. Report determinism (fixed master seed).
    {
        const CostReport r2 =
            compute_cost_report({catalog_A(), catalog_B()}, ccfg, rbase, rcc);
        CHECK(r2.catalogs.size() == report.catalogs.size());
        for (std::size_t i = 0; i < report.catalogs.size(); ++i) {
            const auto& a = report.catalogs[i].amortization;
            const auto& b = r2.catalogs[i].amortization;
            CHECK(a.size() == b.size());
            for (std::size_t k = 0; k < a.size(); ++k) {
                CHECK(a[k].cost_per_removal_p50 == b[k].cost_per_removal_p50);
                CHECK(a[k].removals_p50 == b[k].removals_p50);
            }
            CHECK(report.catalogs[i].fom[0].fom_p50 == r2.catalogs[i].fom[0].fom_p50);
        }
        CHECK(report.tornado.size() == r2.tornado.size());
    }

    // 6. Tornado: five parameters, sorted by cost/removal swing (descending).
    {
        CHECK(report.tornado.size() == 5);
        for (std::size_t i = 1; i < report.tornado.size(); ++i)
            CHECK(report.tornado[i - 1].cost_per_removal_swing >=
                  report.tornado[i].cost_per_removal_swing - 1e-12);
    }

    // 7. CSV schema stability + record types + no absolute-currency figure.
    {
        const std::string csv = "test_wp6_summary.csv";
        const std::string md  = "test_wp6_summary.md";
        const std::string sch = "test_wp6_schema.md";
        write_cost_summary_csv(csv, rcc, report);
        write_cost_summary_md(md, rcc, report);
        write_cost_schema_md(sch);

        const std::string txt = slurp(csv);
        const std::string header = txt.substr(0, txt.find('\n'));
        const char* cols[] = {"schema_version", "catalog", "record_type", "n_kits",
                              "param", "weighting", "metric", "estimate", "p05",
                              "p50", "p95", "units", "notes"};
        for (const char* c : cols) CHECK(header.find(c) != std::string::npos);
        const char* rts[] = {"amortization", "fom", "tornado", "cost_component",
                             "currency_anchor"};
        for (const char* rt : rts) CHECK(txt.find(rt) != std::string::npos);

        // No point-value currency: '$' must never appear in any WP6 artifact,
        // and the CU->currency anchor is left unfilled (low = high = 0).
        CHECK(txt.find('$') == std::string::npos);
        CHECK(slurp(md).find('$') == std::string::npos);
        CHECK(slurp(sch).find('$') == std::string::npos);
        CHECK(rcc.cu_to_musd_low == 0.0 && rcc.cu_to_musd_high == 0.0);
        // No final legal/approval language either (carried WP5 guardrail).
        const std::string forbidden[] = {"approved", "licensed", "compliant"};
        for (const std::string& wd : forbidden) {
            CHECK(txt.find(wd) == std::string::npos);
            CHECK(slurp(md).find(wd) == std::string::npos);
        }
    }

    // 8. WP5-summary consumer (schema 1.0): parse a synthetic summary row.
    {
        const std::string p = "test_wp5_like_summary.csv";
        std::ofstream out(p);
        out << "schema_version,catalog,master_seed,n_runs,metric,estimate,"
               "wilson_low,wilson_high,p05,p50,p95,units,source_runs_csv,notes\n";
        out << "1.0,SL-16 / Zenit-2 second stage,123,500,removals_per_mission,"
               "3.830000,0,0,3.0,4.0,4.0,count,wp5_campaign_runs.csv,\"per mission\"\n";
        out.close();
        CHECK(std::abs(read_wp5_summary_value(p, "SL-16 / Zenit-2 second stage",
                                              "removals_per_mission", "estimate") - 3.83) < 1e-9);
        CHECK(std::abs(read_wp5_summary_value(p, "SL-16 / Zenit-2 second stage",
                                              "removals_per_mission", "p50") - 4.0) < 1e-9);
        CHECK(read_wp5_summary_value(p, "nope", "removals_per_mission", "p50") == -1.0);
        CHECK(read_wp5_summary_value("no_such_file.csv", "x", "y", "p50") == -1.0);
    }

    std::printf("cost: all tests passed\n");
    return 0;
}
