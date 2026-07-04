// WP12 fidelity-ladder driver: regenerates generated/wp12_ladder.{csv,md}.
//
// F2 ("Passive-safety claims are model-scoped") states the WP1 keep-out and
// safety-ellipse guarantees are exact only in the linear Clohessy-Wiltshire
// model. This emitter RE-VERIFIES the already-committed WP5 campaign's
// closing-speed-gate abort events at two higher fidelity levels (L1: +J2,
// L2: +J2+drag) -- SAME commanded dv from the recorded WP11 clearing-abort
// law (the law is UNCHANGED; this is re-verification of an existing decision
// under a higher-fidelity coast, not a new law) -- plus a forensic-14
// per-level clearance table and a margin-decay measurement (the erosion F2
// only PROMISED to characterize, now MEASURED). R1: the WP5 campaign
// artifacts (wp5_campaign_*.csv/.md) are untouched byte-for-byte; every
// number here is a NEW, separately-tagged claim (R14: [L0]/[L1]/[L2] tags).
//
//   adsc_ladder [n_runs] [out_dir]
//
// Defaults: n_runs = CampaignConfig::n_runs (500), out_dir = "generated".
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "adsc/campaign.hpp"
#include "adsc/decay.hpp"
#include "adsc/estimator.hpp"  // GaussianSource (L2 independent BC-dispersion stream, ds-v2)
#include "adsc/forensic14_states.hpp"
#include "adsc/mission.hpp"
#include "adsc/propagation.hpp"
#include "adsc/relmotion.hpp"

using namespace adsc;

namespace {

// ---------------------------------------------------------------- labeling
const char* level_tag(FidelityLevel level) {
    switch (level) {
        case FidelityLevel::L0_CW:      return "L0";
        case FidelityLevel::L1_J2:      return "L1";
        case FidelityLevel::L2_J2_DRAG: return "L2";
    }
    return "L0";
}
// R14 dispersion-set tag: L0/L1 re-verify the SAME committed campaign draws
// (ds-v1); L2 additionally layers the independent BC-uncertainty stream
// (design doc: "ds-v2 = ds-v1 + independent BC stream").
const char* level_dispersion_set(FidelityLevel level) {
    return (level == FidelityLevel::L2_J2_DRAG) ? "ds-v2" : "ds-v1";
}
const char* level_note(FidelityLevel level) {
    switch (level) {
        case FidelityLevel::L0_CW:      return "[L0: two-body linear CW, ds-v1]";
        case FidelityLevel::L1_J2:      return "[L1: two-body+J2, ds-v1]";
        case FidelityLevel::L2_J2_DRAG: return "[L2: +drag, ds-v2]";
    }
    return "";
}

// FNV-1a 64-bit (mirrors campaign.cpp's file-local helper, duplicated here
// since that one is anonymous-namespace-scoped in a different translation
// unit -- same standard algorithm/constants, R6).
uint64_t fnv1a64(const char* s) {
    uint64_t h = 1469598103934665603ULL;
    for (; *s; ++s) {
        h ^= static_cast<unsigned char>(*s);
        h *= 1099511628211ULL;
    }
    return h;
}

// PLACEHOLDER bare-stage drag cross-sections (WP12 L2). Kept local to this
// emitter rather than added to DebrisCatalog (R10): they are not a class-level
// literature figure the way mass/altitude/inclination are.
constexpr double kSL16AreaM2 = 33.0;  // PLACEHOLDER SL-16 / Zenit-2 bare-stage cross-section [m^2]
constexpr double kSL8AreaM2  = 7.5;   // PLACEHOLDER SL-8 / Kosmos-3M bare-stage cross-section [m^2]

// R10: nothing in the L2 BC-dispersion logic below is a bare literal --
// PLACEHOLDER statistical/numerical-safety constants are named here.
constexpr double kBcDispersionFrac = 0.30;  // PLACEHOLDER +/-30% 1-sigma per-craft ballistic-coefficient dispersion (design doc)
constexpr double kBcMultiplierFloor = 0.1;  // numerical-safety floor on the sampled BC multiplier (prevents non-physical <=0 drag)

double catalog_bare_area_m2(const DebrisCatalog& cat) {
    // The two committed presets split cleanly at 800 km (840 vs 750 km); a
    // third catalog would need a real lookup, not this two-way split.
    return (cat.altitude_km > 800.0) ? kSL16AreaM2 : kSL8AreaM2;
}

// Nominal (undispersed) per-craft ballistic coefficient Cd*(A/m) [m^2/kg].
// Chaser mass is the servicer's pre-install bus+kit mass (the same figure
// main_metrics.cpp reports as wp3_contact_mass_kg): the closing-speed gate
// (and hence every abort event) fires BEFORE kit installation.
double nominal_bc_chaser(const Config& cfg) {
    return cfg.drag_cd * cfg.servicer_drag_area_m2 / (cfg.dry_mass_kg + cfg.kit_mass_kg);
}
double nominal_bc_target(const Config& cfg, const DebrisCatalog& cat) {
    return cfg.drag_cd * catalog_bare_area_m2(cat) / cat.mass_kg;
}

// -------------------------------------------------------------- row schema
// Generic long-format row (matches the repo's reference_metrics.csv /
// campaign summary idiom): one row per metric, a `section` discriminator
// distinguishes the three ladder tables, `item` names the specific
// case/geometry within a section.
struct LadderRow {
    std::string section;         // "abort_reverify" | "forensic14" | "margin_decay"
    std::string level;           // "L0" | "L1" | "L2"
    std::string dispersion_set;  // "ds-v1" | "ds-v2"
    std::string catalog;
    std::string item;
    std::string metric;
    double      value;
    std::string units;
    std::string notes;
};

bool is_rate_metric(const std::string& m) {
    return m == "violation_rate" || m == "wilson_low" || m == "wilson_high";
}

// -------------------------------------------------------- abort-event replay
// Re-verify ONE catalog's captured closing-speed-gate abort events at ONE
// fidelity level: SAME commanded dv (event.ab.dv, the WP11 clearing-abort
// law's output -- the law is UNCHANGED, R1), coast-propagated at `level` for
// one campaign screen horizon (1 orbital period @ 8 s, matching
// main_campaign.cpp's own coarsened keep-out screen).
//
// KNOWN GAP (documented, not silently dropped): for a RetreatHop event, the
// campaign's own screen already coast-verifies BOTH burns (mission.cpp,
// clearing_abort_for); this ladder re-verification applies ONLY the FIRST
// burn's dv here, since fidelity_coast_min_range has no mid-course-burn
// hook. RetreatHop is rare (the escalation of last resort) and this is
// reported as an explicit limitation in the emitted markdown, not hidden.
void reverify_abort_events(FidelityLevel level, const DebrisCatalog& cat,
                           const CampaignConfig& ccfg, const Config& base_cfg,
                           const std::vector<CampaignAbortEvent>& events,
                           double horizon_s, std::vector<LadderRow>* out) {
    const double bc_chaser0 = nominal_bc_chaser(base_cfg);
    const double bc_target0 = nominal_bc_target(base_cfg, cat);
    const uint64_t bc_base_seed = splitmix64_seed(
        ccfg.master_seed ^ fnv1a64(cat.name) ^ fnv1a64("wp12-bc-v1"), 0);

    int n_violations = 0;
    std::vector<double> clearances;
    clearances.reserve(events.size());

    for (std::size_t idx = 0; idx < events.size(); ++idx) {
        const CampaignAbortEvent& ev = events[idx];
        Vector6d x0;
        x0 << ev.r, (ev.v + ev.ab.dv);

        double bc_chaser = bc_chaser0;
        double bc_target = bc_target0;
        const double solar_factor = ccfg.nominal_solar_factor;
        if (level == FidelityLevel::L2_J2_DRAG) {
            // L2 BC-dispersion: an INDEPENDENT stream (ds-v2), seeded per
            // catalog and event index; the main campaign ds-v1 stream
            // (CampRng inside run_one_mission) is never touched.
            const uint64_t seed = splitmix64_seed(bc_base_seed, static_cast<uint64_t>(idx));
            GaussianSource bc_rng(static_cast<uint32_t>(seed));
            const double z_c = bc_rng.sample();
            const double z_t = bc_rng.sample();
            bc_chaser = bc_chaser0 * std::max(kBcMultiplierFloor, 1.0 + kBcDispersionFrac * z_c);
            bc_target = bc_target0 * std::max(kBcMultiplierFloor, 1.0 + kBcDispersionFrac * z_t);
        }

        const double min_range = fidelity_coast_min_range(
            level, x0, cat.altitude_km, cat.inclination_deg, horizon_s,
            base_cfg.abort_coast_check_dt_s, solar_factor, bc_chaser, bc_target);
        if (min_range < base_cfg.keep_out_radius_m) ++n_violations;
        clearances.push_back(min_range - base_cfg.keep_out_radius_m);
    }

    std::sort(clearances.begin(), clearances.end());
    const int n = static_cast<int>(events.size());
    const Interval ci = wilson_interval(n_violations, n, kWilsonZ95);

    const auto row = [&](const char* metric, double value, const char* units) {
        out->push_back({"abort_reverify", level_tag(level), level_dispersion_set(level),
                        cat.name, "", metric, value, units, level_note(level)});
    };
    row("n_events", static_cast<double>(n), "count");
    row("n_violations", static_cast<double>(n_violations), "count");
    row("violation_rate", (n > 0) ? static_cast<double>(n_violations) / n : 0.0, "fraction");
    row("wilson_low", ci.low, "fraction");
    row("wilson_high", ci.high, "fraction");
    row("clearance_floor_m", clearances.empty() ? 0.0 : clearances.front(), "m");
    row("clearance_p05_m", percentile_sorted(clearances, 5.0), "m");
    row("clearance_p50_m", percentile_sorted(clearances, 50.0), "m");
}

// ------------------------------------------------------------ forensic-14
// Per-level clearance of the 14 pinned forensic states (SAME commanded dv
// from clearing_abort_for -- the law is unchanged). A case that clears at L0
// but dips below keep-out at L1/L2 is FLAGGED PROMINENTLY (a first-class
// finding, not an error): honest reporting under a higher-fidelity coast,
// not a regression of the L0 result (which stays byte-identical).
void forensic14_per_level(FidelityLevel level, const Config& base_cfg,
                          std::vector<LadderRow>* out) {
    for (int i = 0; i < kForensic14Count; ++i) {
        const Forensic14State& st = kForensic14States[i];
        const double alt_km  = (st.catalog == 'A') ? 840.0 : 750.0;
        const double incl_deg = (st.catalog == 'A') ? 71.0 : 78.0;

        Config cfg = base_cfg;
        cfg.target_altitude_km = alt_km;
        const CwModel cw = CwModel::from_orbit(kEarthRadius + alt_km * 1000.0);
        const Eigen::Vector3d r(st.x0, st.y0, st.z0);
        const Eigen::Vector3d v(st.vx0, st.vy0, st.vz0);
        const SafeAbort ab = clearing_abort_for(cfg, cw, r, v);

        Vector6d x0;
        x0 << r, (v + ab.dv);
        const DebrisCatalog cat = (st.catalog == 'A') ? catalog_A() : catalog_B();
        const double bc_chaser = nominal_bc_chaser(base_cfg);
        const double bc_target = nominal_bc_target(base_cfg, cat);
        const double horizon_s = base_cfg.abort_coast_check_periods * cw.period();
        const double min_range = fidelity_coast_min_range(
            level, x0, alt_km, incl_deg, horizon_s, base_cfg.abort_coast_check_dt_s,
            /*solar_factor=*/1.0, bc_chaser, bc_target);
        const bool clears = min_range >= base_cfg.keep_out_radius_m;

        const std::string item =
            std::string(1, st.catalog) + std::to_string(st.run_index);
        out->push_back({"forensic14", level_tag(level), level_dispersion_set(level),
                        cat.name, item, "coast_min_range_m", min_range, "m",
                        level_note(level)});
        out->push_back({"forensic14", level_tag(level), level_dispersion_set(level),
                        cat.name, item, "clears", clears ? 1.0 : 0.0, "bool",
                        level_note(level)});
    }
}

// ------------------------------------------------------------ margin decay
// F2 promised the linear-model erosion would be characterized; this MEASURES
// it: min-range erosion (relative to the orbit=1 baseline, AT L1) over 1..5
// orbits, for a 400 m standoff ellipse and a rho=300 m corridor hold, both
// catalog orbits.
void margin_decay(const Config& base_cfg, const DebrisCatalog& cat,
                  std::vector<LadderRow>* out) {
    const CwModel cw = CwModel::from_orbit(kEarthRadius + cat.altitude_km * 1000.0);
    const double period = cw.period();
    const double bc_chaser = nominal_bc_chaser(base_cfg);
    const double bc_target = nominal_bc_target(base_cfg, cat);

    struct Geometry { const char* name; double rho; };
    const Geometry geoms[] = {{"standoff_400m", 400.0}, {"corridor_300m", 300.0}};

    for (const Geometry& g : geoms) {
        const SafetyEllipse ellipse{g.rho, g.rho, 0.0};
        const Vector6d x0 = cw.ellipse_state(ellipse, /*theta=*/0.0);

        double baseline_min_range = 0.0;
        for (int orbits = 1; orbits <= 5; ++orbits) {
            const double horizon_s = static_cast<double>(orbits) * period;
            const double min_range = fidelity_coast_min_range(
                FidelityLevel::L1_J2, x0, cat.altitude_km, cat.inclination_deg,
                horizon_s, base_cfg.abort_coast_check_dt_s, /*solar_factor=*/1.0,
                bc_chaser, bc_target);
            if (orbits == 1) baseline_min_range = min_range;
            const double erosion_m = baseline_min_range - min_range;

            const std::string item = std::string(g.name) + "/orbit=" + std::to_string(orbits);
            out->push_back({"margin_decay", "L1", "ds-v1", cat.name, item,
                            "min_range_m", min_range, "m", level_note(FidelityLevel::L1_J2)});
            out->push_back({"margin_decay", "L1", "ds-v1", cat.name, item,
                            "erosion_m", erosion_m, "m",
                            "erosion relative to the orbit=1 baseline at L1; "
                            "positive = closer approach after repeated orbits"});
        }
    }
}

// ------------------------------------------------------------------- output
void write_csv(const std::string& path, const std::vector<LadderRow>& rows) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "schema_version,section,level,dispersion_set,catalog,item,"
                    "metric,value,units,notes\n");
    for (const LadderRow& r : rows) {
        const char* fmt = is_rate_metric(r.metric) ? "%.6f" : "%.4f";
        char valbuf[32];
        std::snprintf(valbuf, sizeof(valbuf), fmt, r.value);
        std::fprintf(f, "1.0,%s,%s,%s,%s,%s,%s,%s,%s,\"%s\"\n",
                    r.section.c_str(), r.level.c_str(), r.dispersion_set.c_str(),
                    r.catalog.c_str(), r.item.c_str(), r.metric.c_str(), valbuf,
                    r.units.c_str(), r.notes.c_str());
    }
    std::fclose(f);
}

void write_md(const std::string& path, const std::vector<LadderRow>& rows,
             const CampaignConfig& ccfg, const DebrisCatalog (&cats)[2],
             int n_events_a, int n_events_b) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    std::fprintf(f, "# WP12 fidelity ladder -- what each level verifies, and what it does not\n\n");
    std::fprintf(f,
        "Runtime-selectable fidelity levels (single code path, R1): "
        "**L0** (`[L0: two-body linear CW, ds-v1]`) is the ORIGINAL WP1 "
        "Clohessy-Wiltshire linearization -- every committed WP5 campaign "
        "number stays byte-identical; **L1** (`[L1: two-body+J2, ds-v1]`) "
        "differences a full inertial two-body+J2 RK4 propagation of both "
        "craft into the target's instantaneous LVLH frame; **L2** "
        "(`[L2: +drag, ds-v2]`) adds a per-craft free-molecular drag term "
        "under an INDEPENDENT ballistic-coefficient dispersion stream "
        "(ds-v2 = ds-v1 + independent BC stream; the committed campaign's "
        "own ds-v1 stream is untouched). L0 numbers here reproduce the "
        "committed wp5_campaign_runs.csv abort events exactly (same "
        "commanded dv, same coast); L1/L2 numbers are NEW claims, R14-tagged, "
        "and are re-VERIFICATIONS of the existing WP11 clearing-abort LAW "
        "(unchanged) under a higher-fidelity coast -- not a new abort law.\n\n"
        "Master seed `0x%llX`, %d runs/catalog (SAME campaign draws as "
        "wp5_campaign_runs.csv -- ds-v1). Regenerate with `adsc_ladder`.\n\n",
        static_cast<unsigned long long>(ccfg.master_seed), ccfg.n_runs);

    std::fprintf(f, "## Abort-event re-verification (%d catalog-A + %d catalog-B "
                    "closing-speed-gate events from the committed campaign)\n\n",
                n_events_a, n_events_b);
    std::fprintf(f, "| level | dispersion set | catalog | n_events | n_violations | "
                    "violation_rate | wilson 95%% CI | clearance floor (m) | "
                    "p05 (m) | p50 (m) |\n");
    std::fprintf(f, "|---|---|---|---:|---:|---:|---|---:|---:|---:|\n");
    for (const DebrisCatalog& cat : cats) {
        const std::string cat_name = cat.name;
        for (const char* lvl : {"L0", "L1", "L2"}) {
            auto find = [&](const char* metric) -> double {
                for (const LadderRow& r : rows) {
                    if (r.section == "abort_reverify" && r.level == lvl &&
                        r.catalog == cat_name && r.metric == metric) return r.value;
                }
                return 0.0;
            };
            std::string disp = (std::string(lvl) == "L2") ? "ds-v2" : "ds-v1";
            std::fprintf(f, "| %s | %s | %s | %.0f | %.0f | %.6f | [%.6f, %.6f] | "
                            "%.4f | %.4f | %.4f |\n",
                        lvl, disp.c_str(), cat_name.c_str(), find("n_events"),
                        find("n_violations"), find("violation_rate"),
                        find("wilson_low"), find("wilson_high"),
                        find("clearance_floor_m"), find("clearance_p05_m"),
                        find("clearance_p50_m"));
        }
    }
    std::fprintf(f, "\nKNOWN GAP: RetreatHop abort events (rare -- the escalation "
                    "of last resort) are re-verified here using ONLY their FIRST "
                    "burn's dv; the campaign's own screen coast-verifies both "
                    "burns (mission.cpp). fidelity_coast_min_range has no "
                    "mid-course-burn hook, so this is a documented limitation of "
                    "the ladder re-verification, not of the campaign itself.\n\n");

    std::fprintf(f, "## Forensic-14 per-level clearance\n\n");
    std::fprintf(f, "The 14 WP10c/WP11 pinned states (R15) all clear at L0 by "
                    "construction (the WP11 clearing law). A case that clears at "
                    "L0 but dips below keep-out at a higher level is FLAGGED "
                    "PROMINENTLY below as a first-class finding -- an honest "
                    "result of a higher-fidelity coast, not a regression of the "
                    "committed L0 numbers.\n\n");
    std::fprintf(f, "| case | L0 clears | L0 min range (m) | L1 clears | L1 min range (m) | "
                    "L2 clears | L2 min range (m) | flag |\n");
    std::fprintf(f, "|---|---|---:|---|---:|---|---:|---|\n");
    for (int i = 0; i < kForensic14Count; ++i) {
        const Forensic14State& st = kForensic14States[i];
        const std::string item = std::string(1, st.catalog) + std::to_string(st.run_index);
        double clr[3] = {0, 0, 0}, minr[3] = {0, 0, 0};
        const char* lvls[3] = {"L0", "L1", "L2"};
        for (int lv = 0; lv < 3; ++lv) {
            for (const LadderRow& r : rows) {
                if (r.section != "forensic14" || r.item != item || r.level != lvls[lv]) continue;
                if (r.metric == "clears") clr[lv] = r.value;
                if (r.metric == "coast_min_range_m") minr[lv] = r.value;
            }
        }
        const bool flag = (clr[0] > 0.5) && (clr[1] < 0.5 || clr[2] < 0.5);
        std::fprintf(f, "| %s | %s | %.4f | %s | %.4f | %s | %.4f | %s |\n",
                    item.c_str(), clr[0] > 0.5 ? "yes" : "no", minr[0],
                    clr[1] > 0.5 ? "yes" : "no", minr[1],
                    clr[2] > 0.5 ? "yes" : "no", minr[2],
                    flag ? "**CLEARS AT L0, DIPS BELOW KEEP-OUT AT L1/L2**" : "-");
    }
    std::fprintf(f, "\n");

    std::fprintf(f, "## Margin-decay measurement (F2: promised, now measured)\n\n");
    std::fprintf(f, "Min-range erosion over repeated orbits at L1, relative to the "
                    "orbit=1 baseline, for a 400 m standoff ellipse and the "
                    "rho=300 m innermost corridor hold, both catalog orbits.\n\n");
    std::fprintf(f, "| catalog | geometry | orbits | min range (m) | erosion vs orbit=1 (m) |\n");
    std::fprintf(f, "|---|---|---:|---:|---:|\n");
    for (const LadderRow& r : rows) {
        if (r.section != "margin_decay" || r.metric != "min_range_m") continue;
        double erosion = 0.0;
        for (const LadderRow& r2 : rows) {
            if (r2.section == "margin_decay" && r2.metric == "erosion_m" &&
                r2.catalog == r.catalog && r2.item == r.item) erosion = r2.value;
        }
        const std::size_t slash = r.item.find('/');
        const std::string geom = (slash == std::string::npos) ? r.item : r.item.substr(0, slash);
        const std::string orbit_part = (slash == std::string::npos) ? "" : r.item.substr(slash + 1);
        std::fprintf(f, "| %s | %s | %s | %.4f | %.4f |\n", r.catalog.c_str(),
                    geom.c_str(), orbit_part.c_str(), r.value, erosion);
    }
    std::fprintf(f, "\n");

    std::fprintf(f,
        "Coverage statement: this ladder verifies keep-out-abort geometry "
        "re-verification, forensic-14 regression clearance, and min-range "
        "erosion -- each tagged by the fidelity level ([L0]/[L1]/[L2]) that "
        "produced it. It does NOT verify: attitude dynamics (WP2, untouched "
        "translation-only extension), the estimate-driven guidance loop "
        "(WP12 L4, see wp12_est_* in reference_metrics.csv), or actuator "
        "realization error (WP12 L5, see wp12_l5_* in reference_metrics.csv). "
        "No result on this page is a legal, regulatory, or safety conclusion "
        "beyond its stated model scope; each row states exactly which model "
        "produced it.\n");
    std::fclose(f);
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

    // SAME coarsened keep-out screen as main_campaign.cpp (R1: this replays
    // that EXACT committed campaign, so the base config must match it).
    Config base_cfg;
    base_cfg.abort_coast_check_periods = 1.0;
    base_cfg.abort_coast_check_dt_s    = 8.0;
    const DebrisCatalog cats[] = {catalog_A(), catalog_B()};

    std::printf("=== ADSC v5 (WP12) -- fidelity ladder ===\n");

    std::vector<LadderRow> rows;
    std::vector<std::vector<CampaignAbortEvent>> events_by_catalog;
    for (const DebrisCatalog& cat : cats) {
        std::vector<CampaignAbortEvent> events;
        for (int i = 0; i < ccfg.n_runs; ++i) {
            (void)run_one_mission(cat, ccfg, base_cfg, i, &events);
        }
        events_by_catalog.push_back(events);
        std::printf("[WP12] %s: %zu closing-speed-gate abort events captured for re-verification\n",
                    cat.name, events.size());
    }

    for (const FidelityLevel level : {FidelityLevel::L0_CW, FidelityLevel::L1_J2,
                                      FidelityLevel::L2_J2_DRAG}) {
        for (std::size_t c = 0; c < 2; ++c) {
            const CwModel cw = CwModel::from_orbit(kEarthRadius + cats[c].altitude_km * 1000.0);
            const double horizon_s = base_cfg.abort_coast_check_periods * cw.period();
            reverify_abort_events(level, cats[c], ccfg, base_cfg, events_by_catalog[c],
                                 horizon_s, &rows);
        }
        forensic14_per_level(level, base_cfg, &rows);
    }
    for (const DebrisCatalog& cat : cats) {
        margin_decay(base_cfg, cat, &rows);
    }

    write_csv(out_dir + "/wp12_ladder.csv", rows);
    write_md(out_dir + "/wp12_ladder.md", rows, ccfg, cats,
            static_cast<int>(events_by_catalog[0].size()),
            static_cast<int>(events_by_catalog[1].size()));
    std::printf("wrote %s/wp12_ladder.csv, %s/wp12_ladder.md\n", out_dir.c_str(), out_dir.c_str());
    std::printf("=== WP12 fidelity ladder complete ===\n");
    return 0;
}
