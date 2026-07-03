#include "adsc/cost.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

namespace adsc {

const char* weighting_label(Weighting w) {
    switch (w) {
        case Weighting::SpatialDensity: return "spatial_density";
        case Weighting::Criticality:    return "criticality";
    }
    return "spatial_density";
}

double congestion_weight(Weighting w, double altitude_km, const CostConfig& cc) {
    const std::vector<WeightAnchor>& t = cc.weight_table;
    if (t.empty()) return 0.0;
    const auto col = [w](const WeightAnchor& a) {
        return w == Weighting::SpatialDensity ? a.spatial : a.criticality;
    };
    if (altitude_km <= t.front().altitude_km) return col(t.front());
    if (altitude_km >= t.back().altitude_km)  return col(t.back());
    for (std::size_t i = 1; i < t.size(); ++i) {
        if (altitude_km <= t[i].altitude_km) {
            const double a0 = t[i - 1].altitude_km, a1 = t[i].altitude_km;
            const double f = (altitude_km - a0) / (a1 - a0);
            return col(t[i - 1]) + f * (col(t[i]) - col(t[i - 1]));
        }
    }
    return col(t.back());
}

double launch_band_factor(const CostConfig& cc, double altitude_km,
                          double inclination_deg) {
    const double alt_term =
        cc.launch_band_per_100km * (altitude_km - cc.launch_band_ref_km) / 100.0;
    const double inc_term =
        cc.launch_band_per_deg * (inclination_deg - cc.launch_band_ref_incl_deg);
    return std::max(0.5, 1.0 + alt_term + inc_term);
}

double mission_campaign_cost(const CostConfig& cc, const Config& base,
                             const DebrisCatalog& cat, int n_kits,
                             double mission_time_s) {
    const double m_dry = base.dry_mass_kg;
    const double m_wet =
        base.dry_mass_kg + base.initial_fuel_kg + n_kits * base.kit_mass_kg;
    const double C_dev    = cc.c_dev_cu;
    const double C_bus    = cc.c_bus_cu * std::pow(m_dry, cc.c_bus_exponent);
    const double C_kit    = n_kits * cc.c_kit_cu;
    const double band     = launch_band_factor(cc, cat.altitude_km, cat.inclination_deg);
    const double C_launch = cc.c_launch_cu_per_kg * m_wet * band;
    const double C_ops    = cc.c_ops_cu_per_day * (mission_time_s / 86400.0);
    return C_dev + C_bus + C_kit + C_launch + C_ops;
}

namespace {

// Per-run cost/removal over a set of runs (removals == 0 excluded, counted).
std::vector<double> cost_per_removal(const std::vector<RunResult>& runs,
                                     const CostConfig& cc, const Config& base,
                                     const DebrisCatalog& cat, int n_kits,
                                     long* zero_out) {
    std::vector<double> cpr;
    cpr.reserve(runs.size());
    long zero = 0;
    for (const RunResult& r : runs) {
        const double cost =
            mission_campaign_cost(cc, base, cat, n_kits, r.mission_time_s);
        if (r.removals > 0) cpr.push_back(cost / static_cast<double>(r.removals));
        else ++zero;
    }
    if (zero_out) *zero_out = zero;
    return cpr;
}

double p(std::vector<double> v, double q) {  // sort-and-percentile helper
    std::sort(v.begin(), v.end());
    return percentile_sorted(v, q);
}

// CostConfig with one tornado parameter scaled by `factor` (index order matches
// the tornado table below).
CostConfig scaled(const CostConfig& base, int idx, double factor) {
    CostConfig c = base;
    switch (idx) {
        case 0: c.c_dev_cu           *= factor; break;
        case 1: c.c_bus_cu           *= factor; break;
        case 2: c.c_kit_cu           *= factor; break;
        case 3: c.c_launch_cu_per_kg *= factor; break;
        case 4: c.c_ops_cu_per_day   *= factor; break;
        default: break;
    }
    return c;
}
const char* kTornadoParam[] = {"c_dev_cu", "c_bus_cu", "c_kit_cu",
                               "c_launch_cu_per_kg", "c_ops_cu_per_day"};
constexpr int kNumTornadoParams = 5;

}  // namespace

CostReport compute_cost_report(const std::vector<DebrisCatalog>& catalogs,
                               const CampaignConfig& ccfg, const Config& base_cfg,
                               const CostConfig& cc) {
    CostReport report;
    const int baseline_n = ccfg.kits_initial;

    std::vector<RunResult> tornado_baseline_runs;  // first catalog, baseline N
    DebrisCatalog          tornado_cat = catalogs.empty() ? DebrisCatalog{} : catalogs.front();

    for (std::size_t ci = 0; ci < catalogs.size(); ++ci) {
        const DebrisCatalog& cat = catalogs[ci];
        CatalogCost cq;
        cq.catalog = cat;
        cq.baseline_n_kits = baseline_n;

        std::vector<RunResult> baseline_runs;
        double n1_cpr_p50 = 0.0;

        for (int n = cc.sweep_kit_min; n <= cc.sweep_kit_max; ++n) {
            CampaignConfig c = ccfg;
            c.kits_initial = n;
            const std::vector<RunResult> runs = run_campaign(cat, c, base_cfg);

            long zero = 0;
            std::vector<double> cpr =
                cost_per_removal(runs, cc, base_cfg, cat, n, &zero);
            std::vector<double> rem, campcost;
            rem.reserve(runs.size());
            campcost.reserve(runs.size());
            for (const RunResult& r : runs) {
                rem.push_back(static_cast<double>(r.removals));
                campcost.push_back(
                    mission_campaign_cost(cc, base_cfg, cat, n, r.mission_time_s));
            }

            AmortPoint ap;
            ap.n_kits = n;
            ap.cost_per_removal_p05 = p(cpr, 5.0);
            ap.cost_per_removal_p50 = p(cpr, 50.0);
            ap.cost_per_removal_p95 = p(cpr, 95.0);
            ap.removals_p50      = p(rem, 50.0);
            ap.campaign_cost_p50 = p(campcost, 50.0);
            ap.zero_removal_runs = zero;
            if (n == cc.sweep_kit_min) n1_cpr_p50 = ap.cost_per_removal_p50;
            ap.ratio_to_n1_p50 =
                (n1_cpr_p50 > 0.0) ? ap.cost_per_removal_p50 / n1_cpr_p50 : 0.0;
            cq.amortization.push_back(ap);

            if (n == baseline_n) baseline_runs = runs;
        }

        // FoM at the baseline kit count, per weighting.
        const Weighting weightings[] = {Weighting::SpatialDensity,
                                        Weighting::Criticality};
        std::vector<double> mass;
        mass.reserve(baseline_runs.size());
        for (const RunResult& r : baseline_runs)
            mass.push_back(static_cast<double>(r.removals) * cat.mass_kg);
        for (Weighting w : weightings) {
            const double wt = congestion_weight(w, cat.altitude_km, cc);
            std::vector<double> fom;
            fom.reserve(baseline_runs.size());
            for (const RunResult& r : baseline_runs) {
                const double cost = mission_campaign_cost(
                    cc, base_cfg, cat, baseline_n, r.mission_time_s);
                const double removed_mass =
                    static_cast<double>(r.removals) * cat.mass_kg;
                fom.push_back(cost > 0.0 ? removed_mass * wt / cost : 0.0);
            }
            FoMResult fr;
            fr.weighting = w;
            fr.band_weight = wt;
            fr.removed_mass_kg_p50 = p(mass, 50.0);
            fr.fom_p05 = p(fom, 5.0);
            fr.fom_p50 = p(fom, 50.0);
            fr.fom_p95 = p(fom, 95.0);
            cq.fom.push_back(fr);
        }

        // Baseline cost breakdown at the median mission time.
        const double t50 = [&] {
            std::vector<double> t;
            t.reserve(baseline_runs.size());
            for (const RunResult& r : baseline_runs) t.push_back(r.mission_time_s);
            return p(t, 50.0);
        }();
        const double m_dry = base_cfg.dry_mass_kg;
        const double m_wet = base_cfg.dry_mass_kg + base_cfg.initial_fuel_kg +
                             baseline_n * base_cfg.kit_mass_kg;
        cq.baseline_components = {
            {"C_dev", cc.c_dev_cu},
            {"C_bus", cc.c_bus_cu * std::pow(m_dry, cc.c_bus_exponent)},
            {"C_kit_total", baseline_n * cc.c_kit_cu},
            {"C_launch", cc.c_launch_cu_per_kg * m_wet *
                         launch_band_factor(cc, cat.altitude_km, cat.inclination_deg)},
            {"C_ops", cc.c_ops_cu_per_day * (t50 / 86400.0)},
        };

        if (ci == 0) {
            tornado_baseline_runs = baseline_runs;
            tornado_cat = cat;
        }
        report.catalogs.push_back(std::move(cq));
    }

    // Tornado (representative: first catalog, baseline N, spatial FoM).
    report.tornado_catalog = tornado_cat.name ? tornado_cat.name : "";
    const double d = cc.tornado_delta_frac;
    for (int i = 0; i < kNumTornadoParams; ++i) {
        const CostConfig lo = scaled(cc, i, 1.0 - d);
        const CostConfig hi = scaled(cc, i, 1.0 + d);
        auto cpr_p50 = [&](const CostConfig& c) {
            long z = 0;
            return p(cost_per_removal(tornado_baseline_runs, c, base_cfg,
                                      tornado_cat, baseline_n, &z), 50.0);
        };
        auto fom_p50 = [&](const CostConfig& c) {
            const double wt = congestion_weight(Weighting::SpatialDensity,
                                                tornado_cat.altitude_km, c);
            std::vector<double> fom;
            fom.reserve(tornado_baseline_runs.size());
            for (const RunResult& r : tornado_baseline_runs) {
                const double cost = mission_campaign_cost(
                    c, base_cfg, tornado_cat, baseline_n, r.mission_time_s);
                fom.push_back(cost > 0.0
                    ? static_cast<double>(r.removals) * tornado_cat.mass_kg * wt / cost
                    : 0.0);
            }
            return p(fom, 50.0);
        };
        TornadoRow tr;
        tr.param = kTornadoParam[i];
        tr.cost_per_removal_low  = cpr_p50(lo);
        tr.cost_per_removal_high = cpr_p50(hi);
        tr.cost_per_removal_swing =
            std::abs(tr.cost_per_removal_high - tr.cost_per_removal_low);
        tr.fom_low  = fom_p50(lo);
        tr.fom_high = fom_p50(hi);
        tr.fom_swing = std::abs(tr.fom_high - tr.fom_low);
        report.tornado.push_back(tr);
    }
    std::sort(report.tornado.begin(), report.tornado.end(),
              [](const TornadoRow& a, const TornadoRow& b) {
                  return a.cost_per_removal_swing > b.cost_per_removal_swing;
              });
    return report;
}

double read_wp5_summary_value(const std::string& path, const std::string& catalog,
                              const std::string& metric, const std::string& column) {
    std::ifstream in(path);
    if (!in) return -1.0;
    const int col = (column == "estimate") ? 5 : (column == "p50") ? 9 : -1;
    if (col < 0) return -1.0;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        // Fields 0..10 are numeric/plain (no embedded commas); the quoted notes
        // field is last, so a plain comma split is safe for the columns we read.
        std::vector<std::string> f;
        std::string cur;
        std::stringstream ss(line);
        while (std::getline(ss, cur, ',')) {
            f.push_back(cur);
            if (static_cast<int>(f.size()) > col) break;
        }
        if (static_cast<int>(f.size()) > col && f.size() > 4 &&
            f[1] == catalog && f[4] == metric) {
            return std::atof(f[static_cast<std::size_t>(col)].c_str());
        }
    }
    return -1.0;
}

// ------------------------------------------------------------------- output IO
void write_cost_summary_csv(const std::string& path, const CostConfig& cc,
                            const CostReport& report) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f,
        "schema_version,catalog,record_type,n_kits,param,weighting,metric,"
        "estimate,p05,p50,p95,units,notes\n");
    const auto emit = [&](const char* cat, const char* rt, const std::string& nk,
                          const char* param, const char* weighting,
                          const char* metric, double est, double p05, double p50,
                          double p95, const char* units, const char* notes) {
        std::fprintf(f, "%s,%s,%s,%s,%s,%s,%s,%.6f,%.6f,%.6f,%.6f,%s,\"%s\"\n",
                     wp6_schema_version(), cat, rt, nk.c_str(), param, weighting,
                     metric, est, p05, p50, p95, units, notes);
    };

    for (const CatalogCost& cq : report.catalogs) {
        for (const AmortPoint& a : cq.amortization) {
            const std::string nk = std::to_string(a.n_kits);
            emit(cq.catalog.name, "amortization", nk, "", "", "cost_per_removal",
                 a.cost_per_removal_p50, a.cost_per_removal_p05,
                 a.cost_per_removal_p50, a.cost_per_removal_p95, "CU_per_removal", "");
            emit(cq.catalog.name, "amortization", nk, "", "",
                 "cost_per_removal_ratio_to_n1", a.ratio_to_n1_p50, 0.0,
                 a.ratio_to_n1_p50, 0.0, "ratio", "p50 relative to sweep min N");
            emit(cq.catalog.name, "amortization", nk, "", "", "removals",
                 a.removals_p50, 0.0, a.removals_p50, 0.0, "count", "");
            emit(cq.catalog.name, "amortization", nk, "", "", "campaign_cost",
                 a.campaign_cost_p50, 0.0, a.campaign_cost_p50, 0.0, "CU", "");
        }
        emit(cq.catalog.name, "fom", "", "", "", "removed_mass_per_mission",
             cq.fom.empty() ? 0.0 : cq.fom.front().removed_mass_kg_p50, 0.0,
             cq.fom.empty() ? 0.0 : cq.fom.front().removed_mass_kg_p50, 0.0, "kg",
             "per mission at baseline N");
        for (const FoMResult& fr : cq.fom) {
            emit(cq.catalog.name, "fom", "", "", weighting_label(fr.weighting),
                 "fom", fr.fom_p50, fr.fom_p05, fr.fom_p50, fr.fom_p95, "kg_per_CU",
                 "debris-risk-reduction per cost; weighting is PLACEHOLDER (T5)");
            emit(cq.catalog.name, "fom", "", "", weighting_label(fr.weighting),
                 "band_weight", fr.band_weight, fr.band_weight, fr.band_weight,
                 fr.band_weight, "normalized", "w(h) PLACEHOLDER; cite on fill");
        }
        const std::string bnk = std::to_string(cq.baseline_n_kits);
        for (const CostComponent& comp : cq.baseline_components)
            emit(cq.catalog.name, "cost_component", bnk, comp.name.c_str(), "",
                 "component_cost", comp.cu, 0.0, comp.cu, 0.0, "CU",
                 "baseline breakdown at median mission time");
    }

    for (const TornadoRow& tr : report.tornado) {
        char note[128];
        std::snprintf(note, sizeof(note), "low=%.4f;high=%.4f (+/-%.0f%% one-at-a-time)",
                      tr.cost_per_removal_low, tr.cost_per_removal_high,
                      cc.tornado_delta_frac * 100.0);
        emit(report.tornado_catalog.c_str(), "tornado", "", tr.param.c_str(), "",
             "cost_per_removal_swing", tr.cost_per_removal_swing, 0.0,
             tr.cost_per_removal_swing, 0.0, "CU_per_removal", note);
        char note2[128];
        std::snprintf(note2, sizeof(note2), "low=%.5f;high=%.5f (spatial FoM)",
                      tr.fom_low, tr.fom_high);
        emit(report.tornado_catalog.c_str(), "tornado", "", tr.param.c_str(), "",
             "fom_swing", tr.fom_swing, 0.0, tr.fom_swing, 0.0, "kg_per_CU", note2);
    }

    emit("", "currency_anchor", "", "", "", "cu_to_musd_range", cc.cu_to_musd_low,
         cc.cu_to_musd_low, 0.0, cc.cu_to_musd_high, "musd_per_cu",
         "PLACEHOLDER: fill a CITED range in WP7; no point-value dollar figure is claimed");
    std::fclose(f);
}

void write_cost_summary_md(const std::string& path, const CostConfig& cc,
                           const CostReport& report) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "# WP6 Parametric cost model + FoM — summary\n\n");
    std::fprintf(f,
        "Primary output is in **relative cost units (CU)**; **no point-value "
        "dollar figure is claimed** (R6/D10) -- the CU->currency anchor is a "
        "PLACEHOLDER cited range, filled in WP7. All cost/FoM parameters are "
        "PLACEHOLDER (see `CostConfig`). Cost, cost/removal and FoM are "
        "propagated **per run** from the WP5 campaign (schema 1.0) and reported "
        "as p05/p50/p95. Regenerate with `adsc_cost`. Campaign re-run time: "
        "%.2f s.\n\n", report.elapsed_s);

    for (const CatalogCost& cq : report.catalogs) {
        std::fprintf(f, "## %s\n\n", cq.catalog.name);
        std::fprintf(f, "### Amortization curve (cost/removal vs kits carried N)\n\n");
        std::fprintf(f, "| N | cost/removal p50 [CU] | p05..p95 | ratio to N=%d | "
                        "removals p50 | campaign cost p50 [CU] |\n",
                     cc.sweep_kit_min);
        std::fprintf(f, "|---:|---:|---|---:|---:|---:|\n");
        for (const AmortPoint& a : cq.amortization) {
            std::fprintf(f, "| %d | %.2f | %.2f .. %.2f | %.3f | %.2f | %.2f |\n",
                         a.n_kits, a.cost_per_removal_p50, a.cost_per_removal_p05,
                         a.cost_per_removal_p95, a.ratio_to_n1_p50, a.removals_p50,
                         a.campaign_cost_p50);
        }
        std::fprintf(f, "\n### FoM (debris-risk reduction per cost) at N=%d\n\n",
                     cq.baseline_n_kits);
        std::fprintf(f, "| weighting | w(h) | removed mass p50 [kg] | "
                        "FoM p50 [kg/CU] | FoM p05..p95 |\n");
        std::fprintf(f, "|---|---:|---:|---:|---|\n");
        for (const FoMResult& fr : cq.fom)
            std::fprintf(f, "| %s | %.3f | %.0f | %.4f | %.4f .. %.4f |\n",
                         weighting_label(fr.weighting), fr.band_weight,
                         fr.removed_mass_kg_p50, fr.fom_p50, fr.fom_p05, fr.fom_p95);
        std::fprintf(f, "\n");
    }

    std::fprintf(f, "## Tornado sensitivity (%s, baseline N, +/-%.0f%%)\n\n",
                 report.tornado_catalog.c_str(), cc.tornado_delta_frac * 100.0);
    std::fprintf(f, "| rank | parameter | cost/removal swing [CU] | "
                    "spatial FoM swing [kg/CU] |\n");
    std::fprintf(f, "|---:|---|---:|---:|\n");
    int rank = 1;
    for (const TornadoRow& tr : report.tornado)
        std::fprintf(f, "| %d | %s | %.2f | %.5f |\n", rank++, tr.param.c_str(),
                     tr.cost_per_removal_swing, tr.fom_swing);

    std::fprintf(f,
        "\n**T5 (open trade — weighting disagreement).** The two normalized "
        "congestion weightings peak at different altitudes (spatial ~750 km, "
        "criticality ~840 km), so they rank target *bands* differently: the "
        "spatial-density weighting values the lower (SL-8, ~750 km) band more, "
        "the criticality weighting values the higher (SL-16, ~840 km) band more. "
        "Absolute per-catalog FoM is dominated by removed mass (the ~9 t class "
        "outranks the ~1.4 t class under both weightings), but the band-priority "
        "flip is a real metric-choice disagreement tracked as T5. Both weighting "
        "tables are PLACEHOLDER and must be filled with citations.\n\n"
        "**No absolute-cost prediction.** Every figure above is in relative CU; "
        "WP6 does not predict absolute program cost. A cited CU->currency range "
        "is a WP7 deliverable.\n");
    std::fclose(f);
}

void write_cost_schema_md(const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f,
"# WP6 cost/FoM CSV schema (version 1.0)\n"
"\n"
"`generated/wp6_cost_summary.csv` is a long-format table consumed by WP7. It\n"
"consumes the WP5 campaign (schema 1.0): the amortization sweep re-runs the WP5\n"
"engine across the kit-inventory sweep at the fixed master seed, and the\n"
"baseline row (n_kits = CampaignConfig::kits_initial) reproduces\n"
"`generated/wp5_campaign_runs.csv` exactly. Primary units are RELATIVE cost\n"
"units (CU); **no point-value currency is emitted** (R6/D10).\n"
"\n"
"## Columns\n"
"\n"
"| column | meaning |\n"
"|---|---|\n"
"| schema_version | WP6 schema id (`1.0`) |\n"
"| catalog | class-level preset name (blank for global rows) |\n"
"| record_type | amortization / fom / tornado / cost_component / currency_anchor |\n"
"| n_kits | kits carried (amortization/cost_component rows); blank otherwise |\n"
"| param | cost parameter (tornado / cost_component rows); blank otherwise |\n"
"| weighting | spatial_density / criticality (fom rows); blank otherwise |\n"
"| metric | the quantity named in the row |\n"
"| estimate | point value (p50 for distributions) |\n"
"| p05 / p50 / p95 | percentiles (distribution rows; 0 when not applicable) |\n"
"| units | CU_per_removal / CU / count / ratio / kg / kg_per_CU / normalized / musd_per_cu |\n"
"| notes | provenance / PLACEHOLDER caveats |\n"
"\n"
"## record_type rows\n"
"\n"
"- **amortization** (per catalog, per swept N): `cost_per_removal` [CU/removal,\n"
"  p05/p50/p95], `cost_per_removal_ratio_to_n1` [ratio], `removals` [count p50],\n"
"  `campaign_cost` [CU p50]. The cost/removal vs N curve is the core installer\n"
"  amortization argument: it falls until the Δv budget (not the kit count)\n"
"  bounds removals, then turns up as extra carried kits add cost without\n"
"  removals.\n"
"- **fom** (per catalog): `removed_mass_per_mission` [kg p50], and per weighting\n"
"  `fom` [kg/CU, p05/p50/p95] and `band_weight` [normalized w(h)]. FoM =\n"
"  sum m_i*w(h_i)/C_campaign (spec §4). The two weightings are PLACEHOLDER and\n"
"  disagree on band priority (T5).\n"
"- **cost_component** (per catalog, baseline N): C_dev / C_bus / C_kit_total /\n"
"  C_launch / C_ops [CU] at the median mission time.\n"
"- **tornado** (representative catalog, baseline N): per cost parameter,\n"
"  `cost_per_removal_swing` [CU] and `fom_swing` [kg/CU] under a +/- one-at-a-\n"
"  time perturbation; rows are sorted by cost/removal swing.\n"
"- **currency_anchor** (global): `cu_to_musd_range` -- a PLACEHOLDER cited\n"
"  range (estimate/p05 = low, p95 = high). WP6 never emits a point-value dollar\n"
"  figure; a filled cited range is a WP7 deliverable.\n"
"\n"
"## Future visualization / evidence (WP7) — not implemented here\n"
"\n"
"The schema supports, without change: amortization curves, FoM-under-weightings\n"
"bars with percentile whiskers, tornado bars, and the cost-component breakdown.\n"
"WP6 emits none of these charts and predicts no absolute cost.\n");
    std::fclose(f);
}

}  // namespace adsc
