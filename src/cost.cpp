#include "adsc/cost.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

namespace adsc {

// ============================================================================
// WP14 itemized absolute-cost table -- transcribed EXACTLY from
// `_tasks_local/wp14-cost-sources.md` (retrieval date 2026-07-11). Every note
// ends with its verified-status flag; D10 (sourced-or-PLACEHOLDER, never
// fabricated). Units vary by row: MUSD, MUSD/yr, MUSD/unit, or a percentage
// applied elsewhere (never as a bare point-dollar figure -- R6/D10).
// ============================================================================
namespace {

const CostItemUsd kWp14CostItems[] = {
    {"development", 8.0, 20.0, 50.0, "MUSD",
     "Smallsat bus + RPO/GNC dev NRE allocated to the campaign. Anchors: govt "
     "CubeSat/smallsat missions ~3-30 MUSD (New Space Economy survey); "
     "Aerospace Corp SSCM19 parametric CER methodology basis "
     "(aerospace.org/sscm); RPO-servicer ceiling analog Astroscale ELSA-d "
     "~100 MUSD total two-spacecraft mission (184 kg servicer + 16 kg "
     "client). Low = lean COTS bus + heritage GNC; mid = smallsat band with "
     "RPO uplift; high = RPO-heavy half of the ELSA-d ceiling. "
     "[web-verified (analog) 2026-07-11]"},
    {"manufacturing_bus_unit", 1.5, 3.0, 6.0, "MUSD",
     "One ~25 kg-dry RPO-capable recurring flight bus (NRE lives in the "
     "development row). Anchors: high-performance smallsat bus > 1 MUSD (New "
     "Space Economy survey); recurring-unit learning rates 95pct (units "
     "1-10) / 90pct (11-50) / 85pct (>50) (ICEAA Learning Rate Sensitivity "
     "Model). [web-verified (analog) 2026-07-11]"},
    {"launch_rideshare_sso", 0.35, 0.5, 0.7, "MUSD",
     "SpaceX rideshare SSO price card Feb 2026: base 350000 USD for up to "
     "50 kg to SSO plus 7000 USD/kg above 50 kg; a ~44 kg servicer fits the "
     "50 kg block. Mid/high add mid-inclination or dedicated-deployer/"
     "scheduling premiums (SpaceX directs non-SSO orbits to inquiry). "
     "[web-verified 2026-07-11]"},
    {"launch_dedicated_anchor", 7.1, 7.5, 7.5, "MUSD",
     "Context row NOT additive with launch_rideshare_sso -- alternative "
     "anchor if a dedicated orbit/schedule is required instead of "
     "rideshare; feeds the launch-HIGH bound (7.5 MUSD) of the anchor "
     "core's C_launch term (Sec.10 Method / adjudicated B3 -- width "
     "preserved, not narrowed). Rocket Lab Electron: ~300 kg to LEO "
     "dedicated mission; Q1-2025 realized avg revenue/launch 7.1 MUSD (SEC "
     "10-Q); ~7.5 MUSD nominal dedicated-mission price (SpaceNexus "
     "explainer). [web-verified 2026-07-11]"},
    {"ground_segment_3yr", 0.06, 0.12, 0.18, "MUSD",
     "Derived: AWS Ground Station published per-minute rates 3 USD/min "
     "(narrowband reserved) to 22 USD/min (wideband on-demand) at 4-6 "
     "passes/day imply ~20000-60000 USD/yr all-in ground segment (AWS "
     "billing docs + ElasticScale / New Space Economy secondary "
     "confirmation); x3 yr campaign span gives 0.06-0.18 MUSD. "
     "[web-verified 2026-07-11]"},
    {"operations_3yr", 1.0, 2.0, 4.0, "MUSD",
     "3-yr campaign routine ops: 1-2 FTE at 150000-200000 USD/yr "
     "fully-burdened plus mission-ops software 50000-200000 USD/yr "
     "(SpaceNexus mission-cost guide); ops roughly 8pct of total mission "
     "cost / 20pct of bus cost. C_ops (CU) tracks ~4 sim-days not the real "
     "multi-year ops span -- this row is the real-dollar multi-year "
     "figure. [web-verified (analog) 2026-07-11]"},
    {"ssa_tracking", 0.0, 0.0, 0.0, "MUSD/yr",
     "LeoLabs declines to publicly quote; subscription is customized by "
     "fleet size and monitoring cadence (SpaceNews; CNBC). Zero-cost floor "
     "pointer: USSF/18th SDS free public conjunction data messages (CDMs). "
     "0/0/0 is an UNFILLED placeholder not a claim the service is free -- "
     "do not invent a subscription figure. [PLACEHOLDER]"},
    {"insurance_launch_plus_1yr_pct", 5.0, 8.0, 12.0, "% insured value",
     "Launch-plus-first-year-in-orbit premium as pct of insured value; "
     "range rises after major-loss years (2023-24 hardened the market) or "
     "for unproven vehicles (Orbital Radar; Taylor Wessing; SpaceNews). "
     "Many smallsat missions self-insure -- applied only in the "
     "campaign_cost_musd_with_reserves row never in the base cost. "
     "[web-verified 2026-07-11]"},
    {"licensing_fcc_application", 0.030, 0.0375, 0.045, "MUSD",
     "FCC Part 25 application fee one-time (not annual): streamlined "
     "smallsat 30000 USD (6-yr license term) is low; standard 45000 USD "
     "(15-yr term) is high; mid is the arithmetic midpoint (no published "
     "intermediate tier). [web-verified 2026-07-11]"},
    {"licensing_fcc_annual", 0.012215, 0.012215, 0.012215, "MUSD/yr",
     "FCC annual regulatory fee NGSO Small Satellite category FY2024: "
     "12215 USD per license/call sign (Federal Register FY2024 fee "
     "assessment; FCC 25-31); category explicitly includes On-Orbit "
     "Servicing (OOS) and Rendezvous and Proximity Operations (RPO) -- the "
     "ADSC servicer class. Single published figure flat across low/mid/"
     "high. [web-verified 2026-07-11]"},
    {"kit_unit_scaled_9t", 0.1, 0.5, 2.0, "MUSD/unit",
     "9-t-class installer deorbit kit; no citable product exists at this "
     "scale. Extrapolated from Terminator Tape (50000-75000 USD/unit "
     "CubeSat-scale lower-bound analog flight-proven) and dragNET (2.8 kg "
     "module mass/design cited no public unit price) up to a custom "
     "EDT/sail plus attach interface for a multi-tonne derelict; upper "
     "bound is engineering judgment. [training-data extrapolation]"},
    {"contingency_reserve_pct", 20.0, 25.0, 30.0, "% of cost",
     "Standard NASA development-phase cost-reserve guideline for Class "
     "C/D smallsat missions (NASA Cost Estimating Handbook v4.0; NASA "
     "reserves lesson 1780; NASA TMC Class C/D expectations); applied to "
     "the pre-reserve campaign cost in the campaign_cost_musd_with_"
     "reserves row. [web-verified 2026-07-11]"},
};
constexpr int kNumWp14CostItems =
    static_cast<int>(sizeof(kWp14CostItems) / sizeof(kWp14CostItems[0]));

const CostItemUsd* find_wp14_item(const char* name) {
    for (int i = 0; i < kNumWp14CostItems; ++i)
        if (std::strcmp(kWp14CostItems[i].item, name) == 0) return &kWp14CostItems[i];
    return nullptr;
}

}  // namespace

const CostItemUsd* wp14_cost_items(int* count) {
    if (count) *count = kNumWp14CostItems;
    return kWp14CostItems;
}

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
        "estimate,p05,p50,p95,units,notes,cost_scenario\n");
    // `cost_scenario` (WP14, schema 1.1) is an ADDITIVE trailing column: every
    // pre-WP14 call site below omits it and gets the default "" (blank), so
    // every existing row's VALUES are unchanged -- only a new blank field is
    // appended. Only the new WP14 rows further down pass a real scenario.
    const auto emit = [&](const char* cat, const char* rt, const std::string& nk,
                          const char* param, const char* weighting,
                          const char* metric, double est, double p05, double p50,
                          double p95, const char* units, const char* notes,
                          const char* cost_scenario = "") {
        std::fprintf(f, "%s,%s,%s,%s,%s,%s,%s,%.6f,%.6f,%.6f,%.6f,%s,\"%s\",%s\n",
                     wp6_schema_version(), cat, rt, nk.c_str(), param, weighting,
                     metric, est, p05, p50, p95, units, notes, cost_scenario);
    };

    // ------------------------------------------------------------- WP14 setup
    // Itemized USD table + the campaign-level anchor derived from it. The
    // anchor's denominator (C_campaign_CU_p50 at the baseline N) is read from
    // the SAME sweep already computed above for catalog A -- nothing here
    // re-runs the campaign or touches a CU coefficient.
    int wp14_n_items = 0;
    const CostItemUsd* wp14_items = wp14_cost_items(&wp14_n_items);
    const char* kScenario[3] = {"low", "mid", "high"};

    const DebrisCatalog catA = catalog_A();
    const CatalogCost* cqA = nullptr;
    for (const CatalogCost& cq : report.catalogs)
        if (std::strcmp(cq.catalog.name, catA.name) == 0) { cqA = &cq; break; }
    const AmortPoint* baseAp = nullptr;
    if (cqA)
        for (const AmortPoint& a : cqA->amortization)
            if (a.n_kits == cqA->baseline_n_kits) { baseAp = &a; break; }
    const double   denom_cu   = baseAp ? baseAp->campaign_cost_p50 : 0.0;
    const int      baseline_n = cqA ? cqA->baseline_n_kits : 0;
    const std::string baseline_nk = std::to_string(baseline_n);

    const CostItemUsd* it_dev        = find_wp14_item("development");
    const CostItemUsd* it_bus        = find_wp14_item("manufacturing_bus_unit");
    const CostItemUsd* it_launch     = find_wp14_item("launch_rideshare_sso");
    const CostItemUsd* it_launch_ded = find_wp14_item("launch_dedicated_anchor");
    const CostItemUsd* it_ground     = find_wp14_item("ground_segment_3yr");
    const CostItemUsd* it_ops        = find_wp14_item("operations_3yr");
    const CostItemUsd* it_fcc_app    = find_wp14_item("licensing_fcc_application");
    const CostItemUsd* it_fcc_annual = find_wp14_item("licensing_fcc_annual");
    const CostItemUsd* it_kit        = find_wp14_item("kit_unit_scaled_9t");
    const CostItemUsd* it_ssa        = find_wp14_item("ssa_tracking");
    const CostItemUsd* it_insurance  = find_wp14_item("insurance_launch_plus_1yr_pct");
    const CostItemUsd* it_contingency = find_wp14_item("contingency_reserve_pct");
    const auto scenario_val = [](const CostItemUsd* it, int s) {
        if (!it) return 0.0;
        return s == 0 ? it->low_musd : (s == 1 ? it->mid_musd : it->high_musd);
    };

    // C_launch term for the anchor CORE (Sec.10 Method, adjudicated B3):
    // width preserved on purpose, NOT narrowed artificially -- low = the
    // rideshare card low (launch_rideshare_sso.low), mid = the rideshare
    // upper / heavier-rideshare-buy figure (launch_rideshare_sso.high), high
    // = the dedicated Electron anchor (launch_dedicated_anchor.high, which
    // equals .mid, both 7.5 MUSD).
    const double c_launch_core[3] = {
        it_launch ? it_launch->low_musd : 0.0,
        it_launch ? it_launch->high_musd : 0.0,
        it_launch_ded ? it_launch_ded->high_musd : 0.0,
    };

    // Anchor CORE = exactly the 5 CU-mapped components ONLY (Sec.10 Method,
    // adjudicated B2): development + manufacturing_bus_unit +
    // kit_unit_scaled_9t x baseline_n kits + C_launch [c_launch_core above]
    // + operations_3yr. ADDITIVES sit on top of the core and are OUTSIDE the
    // anchor numerator: ground_segment_3yr, licensing_fcc_application,
    // licensing_fcc_annual x3 yr, and the PLACEHOLDER ssa_tracking row
    // (value 0 either way). base_total = core + additives (B5-structure).
    double core_musd[3];
    double additives_musd[3];
    double base_total_musd[3];
    double anchor_musd_per_cu[3];
    double insured_value_musd[3];
    double insurance_line_musd[3];
    double with_reserves_musd[3];
    for (int s = 0; s < 3; ++s) {
        core_musd[s] = scenario_val(it_dev, s) + scenario_val(it_bus, s) +
                       scenario_val(it_kit, s) * static_cast<double>(baseline_n) +
                       c_launch_core[s] + scenario_val(it_ops, s);
        additives_musd[s] = scenario_val(it_ground, s) + scenario_val(it_fcc_app, s) +
                            scenario_val(it_fcc_annual, s) * 3.0 + scenario_val(it_ssa, s);
        base_total_musd[s] = core_musd[s] + additives_musd[s];
        anchor_musd_per_cu[s] = (denom_cu > 0.0) ? core_musd[s] / denom_cu : 0.0;

        // N5-insurance: insurance is an ADDITIVE line scaled to the
        // asset+launch insured value (matching the source's illustration),
        // NOT a multiplicative factor over base_total -- base_total also
        // carries the multi-year ops/ground/licensing opex, which is not
        // part of what gets insured.
        insured_value_musd[s] = scenario_val(it_bus, s) +
                                scenario_val(it_kit, s) * static_cast<double>(baseline_n) +
                                c_launch_core[s];
        const double insurance_pct = scenario_val(it_insurance, s);
        insurance_line_musd[s] = (insurance_pct / 100.0) * insured_value_musd[s];
        const double contingency_pct = scenario_val(it_contingency, s);
        with_reserves_musd[s] =
            base_total_musd[s] * (1.0 + contingency_pct / 100.0) + insurance_line_musd[s];
    }

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

            // WP14 (e): inverse-FoM orientation, reusing the fom_p05/p50/p95
            // already computed above (same congestion_weight() call, same
            // per-run mass/cost -- no weight is recomputed here). 1/x is
            // strictly decreasing for x>0, so the percentile order reverses:
            // each cost_per_risk_equiv_mass percentile is the reciprocal of
            // the FoM percentile BY CONSTRUCTION -- it is NOT the percentile
            // of an independent per-run-reciprocal distribution (order
            // statistics interpolate, so those two are not generally equal);
            // p05/p95 swap to the reciprocal of fom's p95/p05.
            const double risk_p50 = (fr.fom_p50 > 0.0) ? 1.0 / fr.fom_p50 : 0.0;
            const double risk_p05 = (fr.fom_p95 > 0.0) ? 1.0 / fr.fom_p95 : 0.0;
            const double risk_p95 = (fr.fom_p05 > 0.0) ? 1.0 / fr.fom_p05 : 0.0;
            emit(cq.catalog.name, "fom", "", "", weighting_label(fr.weighting),
                 "cost_per_risk_equiv_mass", risk_p50, risk_p05, risk_p50, risk_p95,
                 "CU_per_kg",
                 "= C_campaign/sum(m_i*w(h_i)) = 1/fom per the same per-run FoM "
                 "machinery (weights not recomputed); p50 is the reciprocal of "
                 "the fom_p50 percentile BY CONSTRUCTION, NOT the percentile of "
                 "per-run reciprocals (order statistics interpolate); p05/p95 "
                 "are reciprocals of fom_p95/fom_p05 (order reverses for a "
                 "decreasing transform)");
            emit(cq.catalog.name, "fom", "", "", weighting_label(fr.weighting),
                 "cost_per_risk_equiv_mass_musd",
                 risk_p50 * anchor_musd_per_cu[1], risk_p05 * anchor_musd_per_cu[1],
                 risk_p50 * anchor_musd_per_cu[1], risk_p95 * anchor_musd_per_cu[1],
                 "MUSD_per_kg",
                 "= cost_per_risk_equiv_mass[CU_per_kg] x anchor_musd_per_cu"
                 "[cost_scenario=mid]; mid-scenario anchor only to bound row count",
                 "mid");
        }
        const std::string bnk = std::to_string(cq.baseline_n_kits);
        for (const CostComponent& comp : cq.baseline_components)
            emit(cq.catalog.name, "cost_component", bnk, comp.name.c_str(), "",
                 "component_cost", comp.cu, 0.0, comp.cu, 0.0, "CU",
                 "baseline breakdown at median mission time");
    }

    // ------------------------------------------------------- WP14 (a): items
    for (int i = 0; i < wp14_n_items; ++i) {
        const CostItemUsd& it = wp14_items[i];
        for (int s = 0; s < 3; ++s) {
            const double v = scenario_val(&it, s);
            emit("", "cost_component_musd", "", it.item, "", "cost", v, 0.0, v, 0.0,
                 it.unit, it.note, kScenario[s]);
        }
    }

    // ------------------------------------------- WP14 (b): derived CU anchor
    for (int s = 0; s < 3; ++s) {
        char note[1024];
        std::snprintf(note, sizeof(note),
            "anchor_musd_per_cu[%s] = core_5_component / C_campaign_CU_p50; "
            "core = development(%.4f) + manufacturing_bus_unit(%.4f) + "
            "kit_unit_scaled_9t*%dkits(%.4f) + C_launch_core(%.4f) + "
            "operations_3yr(%.4f) = %.6f MUSD; C_campaign_CU_p50(catalog=%s;"
            "baseline N=%d)=%.6f CU; anchor=%.6f MUSD/CU. C_launch_core[%s]="
            "%.4f MUSD (low=rideshare card low, mid=rideshare upper/heavier "
            "buy, high=dedicated Electron -- launch_dedicated_anchor feeds "
            "this high bound, width preserved not narrowed, B3). ONLY these "
            "5 CU-mapped components are in the anchor core (Sec.10 Method, "
            "B2): ground_segment_3yr, licensing_fcc_application, "
            "licensing_fcc_annual*3yr and ssa_tracking (PLACEHOLDER) are "
            "ADDITIVE, outside the core (see campaign_cost_musd additives/"
            "base_total rows); insurance_launch_plus_1yr_pct and "
            "contingency_reserve_pct apply only in campaign_cost_musd"
            "[with_reserves]",
            kScenario[s], scenario_val(it_dev, s), scenario_val(it_bus, s),
            baseline_n, scenario_val(it_kit, s) * static_cast<double>(baseline_n),
            c_launch_core[s], scenario_val(it_ops, s), core_musd[s], catA.name,
            baseline_n, denom_cu, anchor_musd_per_cu[s], kScenario[s],
            c_launch_core[s]);
        emit("", "currency_anchor_derived", baseline_nk, "", "", "anchor_musd_per_cu",
             anchor_musd_per_cu[s], 0.0, anchor_musd_per_cu[s], 0.0, "musd_per_cu",
             note, kScenario[s]);
    }

    // --------------------------------------------- WP14 (c): campaign cost
    // Four rows per scenario, each separately visible (B5-structure) so the
    // anchor's basis is auditable straight from the CSV: core_5_component
    // (== currency_anchor_derived numerator), additives (outside the CU
    // model, outside the anchor), base_total = core + additives, and
    // with_reserves (N5-insurance: contingency multiplies base_total,
    // insurance is an ADDITIVE line on insured_value, not a multiplier over
    // the whole base_total -- see the with_reserves note below).
    for (int s = 0; s < 3; ++s) {
        char note_core[320];
        std::snprintf(note_core, sizeof(note_core),
            "campaign_cost_musd[core_5_component;%s] = the 5 CU-mapped "
            "components (development+manufacturing_bus_unit+"
            "kit_unit_scaled_9t*%dkits+C_launch_core+operations_3yr); "
            "identical numerator to currency_anchor_derived = %.6f MUSD",
            kScenario[s], baseline_n, core_musd[s]);
        emit("", "campaign_cost_musd", baseline_nk, "", "", "core_5_component",
             core_musd[s], 0.0, core_musd[s], 0.0, "MUSD", note_core, kScenario[s]);

        char note_add[320];
        std::snprintf(note_add, sizeof(note_add),
            "campaign_cost_musd[additives;%s] = ground_segment_3yr + "
            "licensing_fcc_application + licensing_fcc_annual*3yr + "
            "ssa_tracking (PLACEHOLDER, 0); outside the CU model and outside "
            "the anchor core (Sec.10 Method) = %.6f MUSD",
            kScenario[s], additives_musd[s]);
        emit("", "campaign_cost_musd", baseline_nk, "", "", "additives",
             additives_musd[s], 0.0, additives_musd[s], 0.0, "MUSD", note_add,
             kScenario[s]);

        char note_base[320];
        std::snprintf(note_base, sizeof(note_base),
            "campaign_cost_musd[base_total;%s] = core_5_component(%.6f) + "
            "additives(%.6f) = %.6f MUSD",
            kScenario[s], core_musd[s], additives_musd[s], base_total_musd[s]);
        emit("", "campaign_cost_musd", baseline_nk, "", "", "base_total",
             base_total_musd[s], 0.0, base_total_musd[s], 0.0, "MUSD", note_base,
             kScenario[s]);

        char note2[640];
        std::snprintf(note2, sizeof(note2),
            "campaign_cost_musd[with_reserves;%s] = base_total(%.6f MUSD) * "
            "(1+contingency_reserve_pct[%s]/100=%.4f) + insurance_line"
            "(%.6f MUSD); insurance_line = insurance_launch_plus_1yr_pct[%s]"
            "/100=%.4f * insured_value(manufacturing_bus_unit+"
            "kit_unit_scaled_9t*%dkits+C_launch_core=%.6f MUSD) = %.6f MUSD "
            "-- insurance scales with asset+launch value (N5), NOT "
            "multiplied over the multi-year opex in base_total",
            kScenario[s], base_total_musd[s], kScenario[s],
            1.0 + scenario_val(it_contingency, s) / 100.0, insurance_line_musd[s],
            kScenario[s], scenario_val(it_insurance, s) / 100.0, baseline_n,
            insured_value_musd[s], insurance_line_musd[s]);
        emit("", "campaign_cost_musd", baseline_nk, "", "", "with_reserves",
             with_reserves_musd[s], 0.0, with_reserves_musd[s], 0.0, "MUSD", note2,
             kScenario[s]);
    }

    // --------------------------------------- WP14 (d): cost/removal in MUSD
    for (int s = 0; s < 3; ++s) {
        const double p05 = baseAp ? baseAp->cost_per_removal_p05 * anchor_musd_per_cu[s] : 0.0;
        const double p50 = baseAp ? baseAp->cost_per_removal_p50 * anchor_musd_per_cu[s] : 0.0;
        const double p95 = baseAp ? baseAp->cost_per_removal_p95 * anchor_musd_per_cu[s] : 0.0;
        char note[384];
        std::snprintf(note, sizeof(note),
            "cost_per_removal_musd[%s] = cost_per_removal_CU(catalog=%s;"
            "baseline N=%d; p05=%.4f p50=%.4f p95=%.4f) x "
            "anchor_musd_per_cu[%s]=%.6f MUSD/CU",
            kScenario[s], catA.name, baseline_n,
            baseAp ? baseAp->cost_per_removal_p05 : 0.0,
            baseAp ? baseAp->cost_per_removal_p50 : 0.0,
            baseAp ? baseAp->cost_per_removal_p95 : 0.0, kScenario[s],
            anchor_musd_per_cu[s]);
        emit(catA.name, "cost_per_removal_musd", baseline_nk, "", "",
             "cost_per_removal", p50, p05, p50, p95, "MUSD_per_removal", note,
             kScenario[s]);
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
         "SUPERSEDED sentinel (CostConfig::cu_to_musd_low/high stay 0.0 for "
         "schema continuity); the anchor is now DERIVED AT RUNTIME from the "
         "WP14 itemized cost table (see currency_anchor_derived rows) -- no "
         "point-value dollar figure is claimed");
    std::fclose(f);
}

void write_cost_summary_md(const std::string& path, const CostConfig& cc,
                           const CostReport& report) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "# WP6 Parametric cost model + FoM — summary\n\n");
    // NB: the wall-clock re-run time is reported to stdout by the driver, NOT
    // written here -- an embedded runtime would make this committed artifact
    // fail the "CI regeneration == committed" reproducibility check (R6).
    std::fprintf(f,
        "Primary output is in **relative cost units (CU)**; **no point-value "
        "dollar figure is claimed** (R6/D10) -- the CU->currency anchor is now "
        "DERIVED AT RUNTIME from the WP14 cited itemized cost table (see the "
        "`currency_anchor_derived` rows in the CSV), per cost_scenario "
        "(low/mid/high); `CostConfig::cu_to_musd_low/high` are superseded "
        "sentinels left at 0.0. All cost/FoM parameters are PLACEHOLDER (see "
        "`CostConfig`). Cost, cost/removal and FoM are propagated **per run** "
        "from the WP5 campaign (schema 1.0) and reported as p05/p50/p95. "
        "Regenerate with `adsc_cost`.\n\n");

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
"# WP6/WP14 cost/FoM CSV schema (version 1.1)\n"
"\n"
"`generated/wp6_cost_summary.csv` is a long-format table consumed downstream.\n"
"It consumes the WP5 campaign (schema 1.0): the amortization sweep re-runs the\n"
"WP5 engine across the kit-inventory sweep at the fixed master seed, and the\n"
"baseline row (n_kits = CampaignConfig::kits_initial) reproduces\n"
"`generated/wp5_campaign_runs.csv` exactly. Primary units are RELATIVE cost\n"
"units (CU); **no point-value currency is emitted** (R6/D10) -- absolute costs\n"
"appear ONLY as cited low/mid/high ranges (WP14).\n"
"\n"
"Schema 1.1 (WP14) is an ADDITIVE bump over 1.0 (WP6): one new trailing\n"
"column (`cost_scenario`), four new `record_type` values (cost_component_musd\n"
"/ currency_anchor_derived / campaign_cost_musd / cost_per_removal_musd), and\n"
"two new `metric` values under the existing `fom` record_type\n"
"(cost_per_risk_equiv_mass / cost_per_risk_equiv_mass_musd). Every row that\n"
"existed under schema 1.0 is byte-identical in VALUE under 1.1; the only\n"
"change to those rows is the appended blank `cost_scenario` field.\n"
"\n"
"## Columns\n"
"\n"
"| column | meaning |\n"
"|---|---|\n"
"| schema_version | schema id (`1.1`) |\n"
"| catalog | class-level preset name (blank for global rows) |\n"
"| record_type | amortization / fom / tornado / cost_component / "
"currency_anchor / cost_component_musd / currency_anchor_derived / "
"campaign_cost_musd / cost_per_removal_musd |\n"
"| n_kits | kits carried (amortization/cost_component rows) or the baseline N\n"
"  the WP14 derived rows were computed at; blank otherwise |\n"
"| param | cost parameter (tornado / cost_component rows) or the WP14 item id\n"
"  (cost_component_musd rows); blank otherwise |\n"
"| weighting | spatial_density / criticality (fom rows); blank otherwise |\n"
"| metric | the quantity named in the row |\n"
"| estimate | point value (p50 for distributions) |\n"
"| p05 / p50 / p95 | percentiles (distribution rows; 0 when not applicable) |\n"
"| units | CU_per_removal / CU / count / ratio / kg / kg_per_CU / CU_per_kg /\n"
"  normalized / musd_per_cu / MUSD / MUSD_per_removal / MUSD_per_kg / MUSD_per_yr\n"
"  / MUSD_per_unit / percent-valued units carried in the row's own `units`\n"
"  string (e.g. \"pct of cost\") |\n"
"| notes | provenance / PLACEHOLDER caveats / WP14 arithmetic strings |\n"
"| cost_scenario | low / mid / high (WP14 rows only); blank for pre-WP14 rows\n"
"  and for rows (e.g. fom / cost_per_risk_equiv_mass in CU) that are not\n"
"  currency-scenario-dependent |\n"
"\n"
"## record_type rows (schema 1.0, unchanged)\n"
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
"  disagree on band priority (T5). WP14 adds, per weighting: `cost_per_"
"  risk_equiv_mass` [CU_per_kg, p05/p50/p95] = C_campaign/sum(m_i*w(h_i)),\n"
"  the exact inverse orientation: each percentile is the reciprocal of the\n"
"  FoM percentile BY CONSTRUCTION (NOT the percentile of per-run\n"
"  reciprocals -- order statistics interpolate, so the two are not the same\n"
"  computation), with p05/p95 swapped since 1/x is decreasing; and\n"
"  `cost_per_risk_equiv_mass_musd` [MUSD_per_kg, cost_scenario=mid only, to\n"
"  bound row count] = the same quantity converted via the mid-scenario anchor.\n"
"- **cost_component** (per catalog, baseline N): C_dev / C_bus / C_kit_total /\n"
"  C_launch / C_ops [CU] at the median mission time.\n"
"- **tornado** (representative catalog, baseline N): per cost parameter,\n"
"  `cost_per_removal_swing` [CU] and `fom_swing` [kg/CU] under a +/- one-at-a-\n"
"  time perturbation; rows are sorted by cost/removal swing.\n"
"- **currency_anchor** (global): `cu_to_musd_range` -- unfilled pre-WP14\n"
"  sentinel (CostConfig::cu_to_musd_low/high, both 0.0; SUPERSEDED, see\n"
"  currency_anchor_derived below). Retained byte-identical for schema\n"
"  continuity; it never carries a point-value dollar figure.\n"
"\n"
"## record_type rows (schema 1.1, new -- WP14)\n"
"\n"
"- **cost_component_musd** (global, one row per WP14 itemized cost row x\n"
"  cost_scenario): `metric=cost`, `estimate`/`p50` = the row's value at that\n"
"  scenario, `units` = the item's own unit (MUSD / MUSD/yr / MUSD/unit / a\n"
"  percentage unit), `param` = the item id, `notes` = the full cited source\n"
"  note (D10: every row is sourced-or-PLACEHOLDER; a PLACEHOLDER row's note\n"
"  ends `[PLACEHOLDER]` and its value is 0, never fabricated).\n"
"- **currency_anchor_derived** (global, one row per cost_scenario):\n"
"  `metric=anchor_musd_per_cu` [musd_per_cu] = core_5_component / \n"
"  C_campaign_CU_p50 at the baseline N (catalog A). The core is EXACTLY the\n"
"  5 CU-mapped components (Sec.10 Method): development +\n"
"  manufacturing_bus_unit + kit_unit_scaled_9t x baseline-N-kits + C_launch\n"
"  (low=rideshare card low, mid=rideshare upper, high=dedicated Electron --\n"
"  width preserved, not narrowed) + operations_3yr. ground_segment_3yr,\n"
"  licensing_fcc_application, licensing_fcc_annual, and the PLACEHOLDER\n"
"  ssa_tracking row are ADDITIVE and excluded from this core (see\n"
"  campaign_cost_musd below); so are the insurance/contingency percentage\n"
"  rows. The `notes` field prints the full arithmetic string, numerator AND\n"
"  denominator, so the figure is independently auditable from the row alone.\n"
"- **campaign_cost_musd** (global, per cost_scenario), four separately\n"
"  visible rows so the anchor's basis is auditable: `metric=core_5_component`\n"
"  = the identical numerator as currency_anchor_derived, in MUSD;\n"
"  `metric=additives` = ground_segment_3yr + licensing_fcc_application +\n"
"  licensing_fcc_annual x3 yr + ssa_tracking (PLACEHOLDER, 0), outside the CU\n"
"  model and outside the anchor core; `metric=base_total` = core_5_component\n"
"  + additives; `metric=with_reserves` = base_total x (1 +\n"
"  contingency_reserve_pct/100) + insurance_line, where insurance_line =\n"
"  (insurance_launch_plus_1yr_pct/100) x insured_value and insured_value =\n"
"  manufacturing_bus_unit + kit_unit_scaled_9t x baseline-N-kits + C_launch\n"
"  (asset+launch scale only -- insurance is an ADDITIVE line, not a\n"
"  multiplier over the multi-year opex folded into base_total); `notes`\n"
"  carries the formula for each row.\n"
"- **cost_per_removal_musd** (catalog A, baseline N, per cost_scenario):\n"
"  `metric=cost_per_removal` [MUSD_per_removal, p05/p50/p95] = the existing\n"
"  cost_per_removal CU percentiles at the baseline N times that scenario's\n"
"  anchor_musd_per_cu.\n"
"\n"
"## Honesty framing (D10, carried into every WP14 currency row)\n"
"\n"
"External absolute-cost estimation by an unaffiliated author is inherently\n"
"low-standing; agencies re-run with their own parametric models; the relative\n"
"CU results remain primary. No point-value currency figure is emitted:\n"
"absolute costs appear ONLY as low/mid/high cited ranges.\n"
"\n"
"## Future visualization / evidence -- not implemented here\n"
"\n"
"The schema supports, without change: amortization curves, FoM-under-weightings\n"
"bars with percentile whiskers, tornado bars, the cost-component breakdown, and\n"
"the WP14 itemized/anchor/campaign-cost tables. This module emits none of\n"
"these charts.\n");
    std::fclose(f);
}

}  // namespace adsc
