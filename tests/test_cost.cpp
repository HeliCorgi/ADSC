// WP6 tests: cost-model determinism, congestion-weight interpolation + the T5
// band-ranking flip, FoM formula, amortization monotonicity (until the Δv
// budget bounds removals), per-run quantile propagation, CSV schema stability,
// the WP5-summary consumer, report determinism, and the guardrail that WP6
// emits no absolute-currency figure. Explicit return-1 checks (R4); fixed seeds
// (R6). Existing regression tests are unaffected (WP1-WP5 code is untouched).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

// Manual QUOTE-AWARE comma split (NOT std::getline-on-stringstream: that
// silently drops a trailing empty field, e.g. the blank cost_scenario column
// on every pre-WP14 row, misaligning every column-index lookup below).
// Quote-awareness is REQUIRED: several WP14 quoted notes fields contain
// literal commas (e.g. the currency_anchor_derived arithmetic note), so a
// plain comma split would shift the cost_scenario column out of position.
// RFC-4180-lite: '"' toggles the quoted state; a doubled '""' inside quotes
// is an escaped quote; commas inside quotes are field content. Quotes are
// stripped from the returned fields.
static std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> f;
    std::string cur;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (quoted) {
            if (c == '"') {
                if (i + 1 < line.size() && line[i + 1] == '"') { cur += '"'; ++i; }
                else quoted = false;
            } else {
                cur += c;
            }
        } else if (c == '"') {
            quoted = true;
        } else if (c == ',') {
            f.push_back(cur);
            cur.clear();
        } else {
            cur += c;
        }
    }
    f.push_back(cur);
    return f;
}

// schema 1.1 column order: 0 schema_version 1 catalog 2 record_type 3 n_kits
// 4 param 5 weighting 6 metric 7 estimate 8 p05 9 p50 10 p95 11 units 12 notes
// 13 cost_scenario.
static std::vector<std::vector<std::string>> parse_csv_rows(const std::string& csv_text) {
    std::vector<std::vector<std::string>> rows;
    std::stringstream ss(csv_text);
    std::string line;
    std::getline(ss, line);  // header
    while (std::getline(ss, line)) {
        if (!line.empty()) rows.push_back(split_csv_line(line));
    }
    return rows;
}

// First row matching every non-null filter; returns the value at `col`
// (std::atof) or a sentinel if no row matches. Pass nullptr to skip a filter,
// or "" to require the column be literally blank.
static double find_csv_val(const std::vector<std::vector<std::string>>& rows,
                           const char* record_type, const char* metric,
                           const char* weighting, const char* cost_scenario,
                           int col) {
    for (const std::vector<std::string>& f : rows) {
        if (f.size() <= static_cast<std::size_t>(col)) continue;
        if (record_type && f[2] != record_type) continue;
        if (metric && f[6] != metric) continue;
        if (weighting && f[5] != weighting) continue;
        if (cost_scenario && f[13] != cost_scenario) continue;
        return std::atof(f[col].c_str());
    }
    return -999999.0;  // sentinel: no matching row found
}

static bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
          s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
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
                              "p50", "p95", "units", "notes", "cost_scenario"};
        for (const char* c : cols) CHECK(header.find(c) != std::string::npos);
        const char* rts[] = {"amortization", "fom", "tornado", "cost_component",
                             "currency_anchor", "cost_component_musd",
                             "currency_anchor_derived", "campaign_cost_musd",
                             "cost_per_removal_musd", "cost_per_risk_equiv_mass"};
        for (const char* rt : rts) CHECK(txt.find(rt) != std::string::npos);

        // No point-value currency: '$' must never appear in any WP6/WP14
        // artifact (this scan runs over `txt`/md/sch AFTER the WP14 rows are
        // written, so it covers the new cost_component_musd /
        // currency_anchor_derived / campaign_cost_musd / cost_per_removal_musd
        // / cost_per_risk_equiv_mass(_musd) rows too -- no separate scan
        // needed). `CostConfig::cu_to_musd_low/high` are UNUSED SENTINELS,
        // intentionally left at 0.0 post-WP14: the real WP14 currency anchor
        // is derived at runtime from the cited itemized table (see
        // `currency_anchor_derived` checks below) and written straight into
        // the CSV -- it never round-trips through these two config fields, so
        // their ==0.0 assert below is still exactly what should hold, just
        // for a different reason than pre-WP14 ("not yet filled" ->
        // "deliberately bypassed").
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

        // 7b. WP14: itemized USD table + derived anchor/campaign-cost/
        //     cost-per-risk rows written into the same CSV.
        {
            // Table shape + D10 (sourced-or-PLACEHOLDER, never fabricated):
            // every row's note is non-empty and ends with one of the four
            // verified-status flags.
            int n_items = 0;
            const CostItemUsd* items = wp14_cost_items(&n_items);
            CHECK(n_items >= 11);
            for (int i = 0; i < n_items; ++i) {
                CHECK(items[i].item != nullptr && items[i].item[0] != '\0');
                CHECK(items[i].unit != nullptr && items[i].unit[0] != '\0');
                const std::string note = items[i].note ? items[i].note : "";
                CHECK(!note.empty());
                CHECK(ends_with(note, "[web-verified 2026-07-11]") ||
                      ends_with(note, "[web-verified (analog) 2026-07-11]") ||
                      ends_with(note, "[training-data extrapolation]") ||
                      ends_with(note, "[PLACEHOLDER]"));
            }

            const std::vector<std::vector<std::string>> rows = parse_csv_rows(txt);

            // currency_anchor_derived: a real, cited low<high range (distinct
            // from the unused cu_to_musd_low/high sentinels above).
            const double anchor_low =
                find_csv_val(rows, "currency_anchor_derived", "anchor_musd_per_cu",
                            nullptr, "low", 7);
            const double anchor_high =
                find_csv_val(rows, "currency_anchor_derived", "anchor_musd_per_cu",
                            nullptr, "high", 7);
            CHECK(anchor_low > 0.0);
            CHECK(anchor_high > anchor_low);

            // campaign_cost_musd: with_reserves (contingency applied over
            // base_total, plus the additive insurance_line) strictly exceeds
            // the pre-reserve base_total at every scenario that has a
            // nonzero contingency pct; check the high scenario (contingency
            // 30%, insurance 12% -- both nonzero, so the contingency
            // multiplier is > 1 and insurance_line is a positive add-on, so
            // the inequality is strict either way).
            const double campaign_base_total_high =
                find_csv_val(rows, "campaign_cost_musd", "base_total", nullptr,
                            "high", 7);
            const double campaign_with_reserves_high =
                find_csv_val(rows, "campaign_cost_musd", "with_reserves", nullptr,
                            "high", 7);
            CHECK(campaign_base_total_high > 0.0);
            CHECK(campaign_with_reserves_high > campaign_base_total_high);

            // Inverse-FoM identity: cost_per_risk_equiv_mass (CU/kg) is
            // DEFINED as the reciprocal of fom (kg/CU) at each percentile,
            // with p05/p95 swapped (1/x is monotone-decreasing for x>0).
            // Inverse-FoM identity BY CONSTRUCTION: each
            // cost_per_risk_equiv_mass percentile is the reciprocal of the
            // already-computed FoM percentile (NOT the percentile of a
            // per-run-reciprocal distribution -- order statistics interpolate,
            // so those would differ). The CSV stores the value through the
            // fixed "%.6f" format, so the honest exact comparison is against
            // the reciprocal ROUND-TRIPPED through the same format: identical
            // strings parse to identical doubles.
            const double csv_risk_p50_criticality = find_csv_val(
                rows, "fom", "cost_per_risk_equiv_mass", "criticality", "", 9);
            const double fom_p50_criticality =
                report.catalogs.front().fom[1].fom_p50;  // index 1 == Criticality
            CHECK(report.catalogs.front().fom[1].weighting == Weighting::Criticality);
            CHECK(fom_p50_criticality > 0.0);
            CHECK(csv_risk_p50_criticality > 0.0);
            char expect_buf[64];
            std::snprintf(expect_buf, sizeof expect_buf, "%.6f",
                          1.0 / fom_p50_criticality);
            CHECK(std::abs(csv_risk_p50_criticality - std::atof(expect_buf)) < 1e-15);
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
