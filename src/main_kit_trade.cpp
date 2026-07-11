// WP13 kit-class trade CSV emitter: writes generated/wp13_kit_trade.{csv,md}
// combining the EXISTING WP3 sail-decay model (src/decay.cpp:
// sail_decay_years, area_for_target_years) with the NEW WP13 EDT-v1
// aligned-dipole physics core (src/decay.cpp: edt_deorbit_years) into one
// per-class (A/B/C) recommended-kit trade table. Also writes the WP13
// class-C controlled-reentry mission-class comparison, generated/
// wp13_classC.{csv,md} (Zenit-2 stage / catalog_A vs Envisat-class /
// catalog_C -- same-target-set rule, spec:237/244-246). This changes no
// existing physics and no existing pinned number: it is a pure consumer of
// catalog_A/B/C and the pre-existing decay.cpp functions. Deterministic and
// timestamp-free (R6).
//
//   kit_trade [out_dir]
//
// Default out_dir = "generated".
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <system_error>

#include "adsc/decay.hpp"
#include "adsc/mission.hpp"  // Config

using namespace adsc;

namespace {

// Stable schema id for wp13_kit_trade.csv and wp13_classC.csv (independent
// of wp3's schema id; both new files start at 1.0).
const char* kSchema = "1.0";

// R14 model-scope tag, carried verbatim in every edt_years row's notes: the
// aligned-dipole force model, the mandatory eta(i) band, and the T7
// libration caveat, stated in-line with the number it qualifies.
const char* kEdtModelTag =
    "[model: EDT-v1 aligned-dipole, capped-current; eta band "
    "|cos i|..cos^2 i; libration T7 OPEN - PLACEHOLDER duty factor]";

// decay.hpp's own documented print for the exactly-polar (|cos i| ~= 0)
// null-coupling case: years_optimistic/years_conservative are +infinity, a
// physically correct null result, not a numerical artifact (see the
// edt_deorbit_years doc comment). Retrograde catalogs (i > 90 deg) get
// FINITE numbers: the drag magnitude scales with |cos i| (Lenz).
const char* kEdtPolarNote = "n/a (polar: aligned-dipole avg force -> 0)";

// Writes the 7 record_type rows shared by every catalog (sail_area_25yr,
// edt_years, edt_eta, edt_emf_power, kit_mass x2, deploy_risk). The 8th row
// (recommended_kit) is catalog-specific prose and is written by the caller.
void write_common_rows(std::FILE* f, const DebrisCatalog& c, const Config& cfg,
                       const EdtConfig& ecfg, double a25_max, double a25_min,
                       const EdtResult& edt) {
    // sail_area_25yr: EXISTING WP3 model, unchanged. value_lo/value_hi =
    // solar-max/solar-min band (mirrors wp3_decay_trade.csv's area_for_25yr).
    std::fprintf(f,
        "%s,%s,%.1f,%.1f,%.1f,sail_area_25yr,sail,0.0,%.4f,%.4f,m2,"
        "\"sail area for the 25-yr IADC guideline (solar max .. solar min); "
        "existing area_for_target_years model, see wp3_decay_trade.csv "
        "area_for_25yr\"\n",
        kSchema, c.name, c.mass_kg, c.altitude_km, c.inclination_deg,
        a25_max, a25_min);

    // edt_years: WP13 EDT-v1 band. value_lo = years_optimistic (eta_hi =
    // |cos i|), value_hi = years_conservative (eta_lo = cos(i)^2) -- never a
    // point value. The exactly-polar null case prints text, not a number.
    if (std::isinf(edt.years_optimistic)) {
        std::fprintf(f,
            "%s,%s,%.1f,%.1f,%.1f,edt_years,edt,0.0,%s,%s,years,"
            "\"%s inclination %.1f deg gives |cos i| ~= 0 (exactly polar), "
            "so the aligned-dipole model reports null orbit-normal "
            "coupling %s\"\n",
            kSchema, c.name, c.mass_kg, c.altitude_km, c.inclination_deg,
            kEdtPolarNote, kEdtPolarNote, kEdtModelTag, c.inclination_deg,
            kEdtPolarNote);
    } else {
        std::fprintf(f,
            "%s,%s,%.1f,%.1f,%.1f,edt_years,edt,0.0,%.4f,%.4f,years,\"%s\"\n",
            kSchema, c.name, c.mass_kg, c.altitude_km, c.inclination_deg,
            edt.years_optimistic, edt.years_conservative, kEdtModelTag);
    }

    // edt_eta: the mandatory inclination-dependent v x B efficiency band.
    // value_lo = eta_lo = cos(i)^2, value_hi = eta_hi = |cos i|.
    const char* sign_note =
        (c.inclination_deg > 90.0)
            ? "; retrograde (i > 90 deg): the EMF polarity reverses but the "
              "induced current always drags (Lenz), so the magnitude "
              "|cos i| is used"
            : "";
    std::fprintf(f,
        "%s,%s,%.1f,%.1f,%.1f,edt_eta,edt,0.0,%.6f,%.6f,[-],"
        "\"eta_lo = cos^2(i) (EMF/collection-limited current), eta_hi = "
        "|cos i| (power/current-capped current); "
        "wp13-edt-derivation.md Section 5%s\"\n",
        kSchema, c.name, c.mass_kg, c.altitude_km, c.inclination_deg,
        edt.eta_lo, edt.eta_hi, sign_note);

    // edt_emf_power: value = open-circuit motional EMF [V]; value_hi =
    // diagnostic power = emf_v * avg_current_a [W]; value_lo unused (0.0).
    std::fprintf(f,
        "%s,%s,%.1f,%.1f,%.1f,edt_emf_power,edt,%.4f,0.0,%.4f,"
        "\"V (value); W (value_hi)\","
        "\"value = open-circuit motional EMF at the catalog altitude [V]; "
        "value_hi = diagnostic power = emf_v * EdtConfig::avg_current_a "
        "[W]; value_lo unused (0.0); not a deorbit-time metric\"\n",
        kSchema, c.name, c.mass_kg, c.altitude_km, c.inclination_deg,
        edt.emf_v, edt.power_w);

    // kit_mass (sail): EXISTING Config::kit_mass_kg. Point value: value =
    // value_lo = value_hi.
    std::fprintf(f,
        "%s,%s,%.1f,%.1f,%.1f,kit_mass,sail,%.4f,%.4f,%.4f,kg,"
        "\"installed drag-sail kit mass (Config::kit_mass_kg)\"\n",
        kSchema, c.name, c.mass_kg, c.altitude_km, c.inclination_deg,
        cfg.kit_mass_kg, cfg.kit_mass_kg, cfg.kit_mass_kg);

    // kit_mass (edt): NEW EdtConfig::kit_mass_kg, PLACEHOLDER.
    std::fprintf(f,
        "%s,%s,%.1f,%.1f,%.1f,kit_mass,edt,%.4f,%.4f,%.4f,kg,"
        "\"installed EDT kit mass, PLACEHOLDER (EdtConfig::kit_mass_kg)\"\n",
        kSchema, c.name, c.mass_kg, c.altitude_km, c.inclination_deg,
        ecfg.kit_mass_kg, ecfg.kit_mass_kg, ecfg.kit_mass_kg);

    // deploy_risk: EdtConfig::deploy_failure_prob, PLACEHOLDER. Reported
    // only -- NOT applied to the edt_years band above.
    std::fprintf(f,
        "%s,%s,%.1f,%.1f,%.1f,deploy_risk,edt,%.4f,%.4f,%.4f,[-],"
        "\"PLACEHOLDER EDT deployment-failure probability "
        "(EdtConfig::deploy_failure_prob); reported only, not applied to "
        "the edt_years band above\"\n",
        kSchema, c.name, c.mass_kg, c.altitude_km, c.inclination_deg,
        ecfg.deploy_failure_prob, ecfg.deploy_failure_prob,
        ecfg.deploy_failure_prob);
}

// Writes generated/wp13_kit_trade_schema.md documenting the wp13_kit_trade.csv
// columns. A single fputs of one string literal -- no printf format
// specifiers, so there is zero formatting risk (mirrors main_decay_trade.cpp's
// write_schema_md).
void write_kit_trade_schema_md(const std::string& out_dir) {
    std::FILE* f = std::fopen((out_dir + "/wp13_kit_trade_schema.md").c_str(), "w");
    if (!f) return;
    std::fputs(
"# WP13 kit-class trade CSV schema (version 1.0)\n"
"\n"
"`generated/wp13_kit_trade.csv` is the committed per-class (A/B/C) kit-trade\n"
"table emitted by `kit_trade` (src/main_kit_trade.cpp). It combines the\n"
"EXISTING WP3 sail-decay model (src/decay.cpp: sail_decay_years,\n"
"area_for_target_years) with the NEW WP13 EDT-v1 aligned-dipole physics core\n"
"(src/decay.cpp: edt_deorbit_years) into one deterministic, timestamp-free\n"
"table (R6): sail area, EDT deorbit-time band, EDT diagnostics, kit mass\n"
"(sail and EDT), EDT deployment risk, and a generated (never hand-written)\n"
"recommended-kit row with a one-line rationale per class. Class-C\n"
"controlled-reentry mission-class detail lives in the separate\n"
"`wp13_classC.{csv,md}` pair, not here (see that file's own column notes).\n"
"\n"
"## Columns\n"
"\n"
"| column | meaning |\n"
"|---|---|\n"
"| schema_version | WP13 kit-trade schema id (`1.0`) |\n"
"| catalog | class-level preset name (`DebrisCatalog::name`) |\n"
"| mass_kg | catalog stage mass [kg] |\n"
"| altitude_km | catalog altitude [km] |\n"
"| inclination_deg | catalog inclination [deg] (drives the mandatory EDT eta(i) band) |\n"
"| record_type | see record_type rows below |\n"
"| kit_option | sail / edt, or the recommendation label (recommended_kit only) |\n"
"| value | single-value metric, or 0.0 for band-only / not-applicable rows |\n"
"| value_lo | band lower edge (see per-row meaning below) |\n"
"| value_hi | band upper edge (see per-row meaning below) |\n"
"| units | value / value_lo / value_hi units |\n"
"| notes | provenance, model-scope tag, and caveats |\n"
"\n"
"## record_type rows\n"
"\n"
"- `sail_area_25yr`: sail area for the 25-yr IADC guideline from the\n"
"  EXISTING WP3 model (`area_for_target_years`, unchanged). value_lo/hi are\n"
"  the solar-max/solar-min band (mirrors `wp3_decay_trade.csv`'s\n"
"  `area_for_25yr` row; this does not repeat the full area sweep -- see\n"
"  that file for the sweep, now including the 1000 m^2 row).\n"
"- `edt_years`: EDT-v1 deorbit-time band from `edt_deorbit_years`.\n"
"  value_lo = years_optimistic (eta_hi = |cos i|, power/current-capped\n"
"  edge); value_hi = years_conservative (eta_lo = cos(i)^2,\n"
"  EMF/collection-limited edge) -- NEVER a point value. notes carries the\n"
"  EDT-v1 model-scope tag (below). Exactly-polar catalogs (|cos i| ~= 0:\n"
"  the aligned-dipole orbit-normal field vanishes) print the literal\n"
"  text `n/a (polar: aligned-dipole avg force -> 0)` in value_lo/value_hi\n"
"  instead of a number: this is the honest, physically-correct null result\n"
"  for that idealization (see the `edt_deorbit_years` doc comment in\n"
"  decay.hpp), not a missing-data placeholder. Retrograde catalogs\n"
"  (i > 90 deg) get FINITE bands: the passive tether's induced current\n"
"  always drags (Lenz), so the force magnitude scales with |cos i|.\n"
"- `edt_eta`: the mandatory inclination-dependent v x B efficiency band\n"
"  (spec:240-243, never omitted). value_lo = eta_lo = cos(i)^2, value_hi =\n"
"  eta_hi = |cos i| (see `wp13-edt-derivation.md` Section 5 for\n"
"  both current-limit regimes; for i > 90 deg the EMF polarity reverses\n"
"  and the magnitude is reported, with a note in the row).\n"
"- `edt_emf_power`: value = open-circuit motional EMF at the catalog\n"
"  altitude [V]; value_hi = diagnostic power = emf_v * avg_current_a [W];\n"
"  value_lo is unused (0.0). Diagnostic only, not a deorbit-time metric.\n"
"- `kit_mass`: installed kit mass. kit_option = sail uses the EXISTING\n"
"  `Config::kit_mass_kg`; kit_option = edt uses the NEW\n"
"  `EdtConfig::kit_mass_kg` (PLACEHOLDER). value = value_lo = value_hi (a\n"
"  point value, not a band).\n"
"- `deploy_risk`: `EdtConfig::deploy_failure_prob`, PLACEHOLDER. Reported\n"
"  only -- NOT applied to the edt_years band above (see the\n"
"  `edt_deorbit_years` doc comment in decay.hpp).\n"
"- `recommended_kit`: value = 0.0 (a label row, not a numeric metric);\n"
"  kit_option carries the recommendation label; notes carries the\n"
"  one-line, generated (never hand-written) rationale, quoting the actual\n"
"  computed sail-area and/or EDT-years numbers for that catalog. Class C's\n"
"  recommendation is `controlled-reentry-mission-class`, pointing at\n"
"  `wp13_classC.md` rather than a sail/EDT kit choice.\n"
"\n"
"## EDT-v1 model scope (R14 fidelity tag)\n"
"\n"
"Every `edt_years` row's notes column carries the literal tag\n"
"`[model: EDT-v1 aligned-dipole, capped-current; eta band cos(i)..cos^2(i);\n"
"libration T7 OPEN - PLACEHOLDER duty factor]`. This states the model scope\n"
"in-line with the number it qualifies (R14): aligned (untilted) dipole\n"
"geomagnetic field (SPENVIS centred dipole, IGRF epoch 2000); force capped\n"
"by either a fixed/power-limited current (optimistic edge) or an\n"
"EMF/collection-limited current (conservative edge); the along-track\n"
"efficiency factor is the orbit-averaged eta(i) in [cos(i)^2, cos(i)],\n"
"MANDATORY per spec WP13 (never a point value, never omitted for a\n"
"high-inclination catalog). Libration/dynamic stability (T7, Pelaez et al.\n"
"2000) is explicitly UNRESOLVED and is folded in only as a flat\n"
"PLACEHOLDER duty-cycle knob (`EdtConfig::eta_libration`) -- never claimed\n"
"solved. Plasma electron density is a cited PLACEHOLDER parameter, not an\n"
"IRI implementation (spec non-goal); see\n"
"`wp13-literature.md` Topic 3 for the citation and the open\n"
"solar-min/max pair gap.\n"
"\n"
"## Schema changes (R15)\n"
"\n"
"Additive columns to this file bump the minor version (1.0 -> 1.1) with\n"
"this file and all consumers updated in the same PR, the same rule as\n"
"`wp3_decay_trade_schema.md`'s own closing paragraph. Additive rows (a new\n"
"catalog, a new record_type) do NOT need a version bump under this\n"
"project's own rule.\n"
        , f);
    std::fclose(f);
}

// ----------------------------------------------------------------------------
// WP13 class-C controlled-reentry mission-class comparison.
// ----------------------------------------------------------------------------

// Controlled-deorbit target perigee: 6371 + 40 km, a mean-Earth-radius
// reentry-perigee convention (per WP13 task spec) -- deliberately distinct
// from the WGS-84 kEarthRadius used below for the starting circular orbit
// radius r_a. This is a stated choice, not a silent inconsistency: r_a
// matches every other altitude-to-radius conversion in this codebase
// (kEarthRadius + altitude_km*1000), while r_p uses the literal 6371+40 km
// perigee target exactly as specified.
constexpr double kReentryPerigeeRadius_m = (6371.0 + 40.0) * 1000.0;

struct ClassCCandidate {
    const char* label;
    DebrisCatalog cat;
};

// Single impulsive perigee-lowering burn from the catalog's circular orbit
// (radius r_a) to the target perigee r_p: dv = v_c*(1 - sqrt(2*r_p/(r_a+r_p))),
// v_c = sqrt(mu/r_a). Writes the circular speed to *v_c_out for reuse by the
// caller (both the CSV and the md table report v_c alongside dv).
double controlled_deorbit_dv_ms(const DebrisCatalog& cat, double* v_c_out) {
    const double r_a = kEarthRadius + cat.altitude_km * 1000.0;
    const double r_p = kReentryPerigeeRadius_m;
    const double v_c = std::sqrt(kEarthMu / r_a);
    const double dv = v_c * (1.0 - std::sqrt(2.0 * r_p / (r_a + r_p)));
    if (v_c_out) *v_c_out = v_c;
    return dv;
}

void write_classc_csv(const std::string& out_dir, const DebrisCatalog& zenit,
                      const DebrisCatalog& envisat) {
    std::FILE* f = std::fopen((out_dir + "/wp13_classC.csv").c_str(), "w");
    if (!f) return;
    std::fprintf(f,
        "schema_version,candidate,mass_kg,altitude_km,inclination_deg,"
        "metric,value,units,notes\n");

    const ClassCCandidate cands[] = {
        {"Zenit-2 stage (catalog_A)", zenit},
        {"Envisat-class (catalog_C)", envisat},
    };
    for (const ClassCCandidate& cd : cands) {
        double v_c = 0.0;
        const double dv = controlled_deorbit_dv_ms(cd.cat, &v_c);
        std::fprintf(f,
            "%s,%s,%.1f,%.1f,%.1f,v_circular_initial,%.4f,m/s,"
            "\"circular-orbit speed at the catalog altitude, "
            "v_c = sqrt(mu/r_a)\"\n",
            kSchema, cd.label, cd.cat.mass_kg, cd.cat.altitude_km,
            cd.cat.inclination_deg, v_c);
        std::fprintf(f,
            "%s,%s,%.1f,%.1f,%.1f,target_perigee_radius,%.1f,km,"
            "\"controlled-deorbit target perigee, mean-Earth-radius "
            "convention 6371+40 km (distinct from the WGS-84 kEarthRadius "
            "used for r_a)\"\n",
            kSchema, cd.label, cd.cat.mass_kg, cd.cat.altitude_km,
            cd.cat.inclination_deg, kReentryPerigeeRadius_m / 1000.0);
        std::fprintf(f,
            "%s,%s,%.1f,%.1f,%.1f,controlled_deorbit_dv,%.4f,m/s,"
            "\"impulsive perigee-lowering burn from circular (RE+alt) to "
            "perigee 6371+40 km: dv = v_c*(1-sqrt(2*r_p/(r_a+r_p)))\"\n",
            kSchema, cd.label, cd.cat.mass_kg, cd.cat.altitude_km,
            cd.cat.inclination_deg, dv);
    }
    std::fclose(f);
}

void write_classc_md(const std::string& out_dir, const DebrisCatalog& zenit,
                     const DebrisCatalog& envisat) {
    std::FILE* f = std::fopen((out_dir + "/wp13_classC.md").c_str(), "w");
    if (!f) return;

    double v_c_z = 0.0, v_c_e = 0.0;
    const double dv_z = controlled_deorbit_dv_ms(zenit, &v_c_z);
    const double dv_e = controlled_deorbit_dv_ms(envisat, &v_c_e);

    std::fprintf(f, "# WP13 Class-C controlled-reentry mission-class comparison\n\n");
    std::fprintf(f,
        "Class C (spec WP13) is a **separate mission class**, not a sail/EDT "
        "kit choice: massive/high-risk derelicts where the deliverable is a "
        "controlled reentry, evaluated on casualty risk, ground footprint, "
        "delta-v, consent, and cost. The comparison against tug architectures "
        "is allowed **here only**, and only on the **same target set** (spec "
        "WP13/WP14, instr Sec 0.4). Data source: `generated/wp13_classC.csv` "
        "(schema %s), regenerated by `kit_trade` -- no hand-written numbers.\n\n",
        kSchema);

    std::fprintf(f, "## Candidates (same target set)\n\n");
    std::fprintf(f, "| candidate | mass [kg] | altitude [km] | inclination [deg] | source |\n");
    std::fprintf(f, "|---|---:|---:|---:|---|\n");
    std::fprintf(f,
        "| Zenit-2 / SL-16 second stage (`catalog_A`) | %.0f | %.0f | %.1f | "
        "web-verified ~8300(-9000) kg, ~845x857 km, 71.01 deg (Tselina-2 "
        "family), `wp13-literature.md` Topic 5a; also class A's "
        "sail/EDT anchor -- massive by the same McKnight et al. 2021 top-50 "
        "ranking that motivates class C |\n",
        zenit.mass_kg, zenit.altitude_km, zenit.inclination_deg);
    std::fprintf(f,
        "| Envisat-class massive SSO payload (`catalog_C`) | %.0f | %.0f | %.1f | "
        "web-verified 8211 kg, ~765-800 km sun-synchronous, ~98.4-98.55 deg, "
        "eoPortal / ESA Earth Online, `wp13-literature.md` "
        "Topic 6 item 1 |\n\n",
        envisat.mass_kg, envisat.altitude_km, envisat.inclination_deg);

    std::fprintf(f, "## Uncontrolled-reentry casualty framing (cited, not computed)\n\n");
    std::fprintf(f,
        "NASA-STD-8719.14A (with Change 1), Requirement 4.7-1: the expected "
        "worldwide human-casualty risk from reentering debris shall not "
        "exceed **Ec < 1e-4** (1 in 10,000); casualty is assumed for any "
        "surviving fragment with impact kinetic energy **> 15 J**; a "
        "controlled reentry must additionally ensure no surviving >15 J "
        "fragment lands within **370 km of foreign landmasses** "
        "(soma.larc.nasa.gov/SIMPLEx/pdf_files/871914.pdf; ESA applies the "
        "same 1e-4 limit via DRAMA/SARA, technology.esa.int/page/"
        "re-entry-safety -- both web-verified, "
        "`wp13-literature.md` Topic 7). This repo does **not** "
        "compute a per-object Ec for either candidate: that requires a "
        "survivability / ground-footprint tool (NASA DAS/ORSAT or ESA "
        "SARA class), out of scope here: "
        "**[CITATION NEEDED - PLACEHOLDER: per-object Ec analysis]**. Both "
        "candidates are multi-tonne, so an UNCONTROLLED reentry is the "
        "presumptive risk driver the 370 km / Ec<1e-4 rule exists for; that "
        "presumption is not itself a computed Ec value.\n\n");

    std::fprintf(f, "## Controlled-deorbit delta-v (COMPUTED)\n\n");
    std::fprintf(f,
        "Single impulsive perigee-lowering burn from the catalog's circular "
        "orbit (radius r_a = R_E + altitude) to a target perigee r_p = "
        "6371 + 40 km (mean-Earth-radius convention for the reentry perigee, "
        "distinct from the WGS-84 kEarthRadius used for r_a elsewhere in "
        "this repo -- a deliberate, documented choice, not a silent "
        "inconsistency): dv = v_c * (1 - sqrt(2 r_p / (r_a + r_p))), v_c = "
        "sqrt(mu / r_a). This is the minimum single-burn delta-v to commit "
        "the stage to reentry; it excludes targeting/footprint-control burns "
        "and any margin. Full precision in `generated/wp13_classC.csv`.\n\n");
    std::fprintf(f, "| candidate | v_c [m/s] | dv [m/s] |\n");
    std::fprintf(f, "|---|---:|---:|\n");
    std::fprintf(f, "| Zenit-2 / SL-16 (catalog_A) | %.1f | **%.1f** |\n", v_c_z, dv_z);
    std::fprintf(f, "| Envisat-class (catalog_C) | %.1f | **%.1f** |\n\n", v_c_e, dv_e);

    std::fprintf(f, "## Consent gate\n\n");
    std::fprintf(f,
        "Legal accessibility enters as a **gate and metadata flags, never a "
        "multiplier** (D12): the WP8 compliance engine BLOCKs unconsented "
        "active debris removal, and target prioritization consumes its "
        "PASS/BLOCK output and flags rather than folding a subjective legal "
        "weight into any score (`tools/compliance/check_compliance.py`, "
        "policy `ADSC-POL-01`, consent-missing -> BLOCK). Any controlled-"
        "reentry mission on either candidate above is subject to that same "
        "gate: no consent, no operation, regardless of the casualty-risk or "
        "delta-v numbers on this page.\n\n");

    std::fprintf(f, "## Kit-installer vs tug (same target set)\n\n");
    std::fprintf(f,
        "Steelmanned: a tug architecture also amortizes fixed costs (launch, "
        "ground segment, ops) across a batch of targets visited in one plane, "
        "exactly like the kit-installer's own batch amortization (WP6 "
        "amortization curve). The comparison is fair only because it is the "
        "SAME target set (spec WP14 rule) -- comparing a tug against a "
        "different, easier target set would be meaningless. What does NOT "
        "amortize for a tug, and is the installer's structural edge (D1: "
        "installer-not-tug): (1) a tug must carry propellant to change EACH "
        "target's orbit itself, so its per-target delta-v cost scales with "
        "target count, while an installer's kit-carrying stages do their own "
        "deorbit after release -- the installer pays no per-target deorbit "
        "delta-v; (2) a tug must detumble, dock/grapple, and physically tow "
        "each multi-tonne stage through its own controlled-reentry burn, "
        "repeating the highest-risk contact event per target, while an "
        "installer's own contact event is bounded by the WP3 low-energy "
        "capture approach (0.333 J contact-energy budget at 0.15 m/s, "
        "README/evidence pack Section on WP3) and does not itself carry a "
        "target through reentry. Both architectures still need SOME per-"
        "target consent/compliance gate (D12) and SOME reentry delta-v paid "
        "by someone (either pre-installed as a kit, or delivered by the tug) "
        "-- the installer's edge is where that delta-v and that repeated "
        "high-risk contact event are paid, not that they vanish.\n\n");

    std::fprintf(f, "## Cost\n\n");
    std::fprintf(f, "| candidate | cost | source |\n");
    std::fprintf(f, "|---|---|---|\n");
    std::fprintf(f, "| Zenit-2 / SL-16 (catalog_A) | itemized low/mid/high ranges | wp6_cost_summary.csv cost_component_musd + campaign_cost_musd rows (WP14) |\n");
    std::fprintf(f, "| Envisat-class (catalog_C) | not separately itemized | WP14 ranges are servicer-campaign-scale; a controlled-reentry mission needs its own estimate (open) |\n\n");

    std::fprintf(f, "## `wp13_classC.csv` columns\n\n");
    std::fprintf(f, "| column | meaning |\n");
    std::fprintf(f, "|---|---|\n");
    std::fprintf(f, "| schema_version | class-C schema id (`%s`) |\n", kSchema);
    std::fprintf(f, "| candidate | candidate label (Zenit-2 stage / Envisat-class) |\n");
    std::fprintf(f, "| mass_kg, altitude_km, inclination_deg | catalog parameters (constant per candidate) |\n");
    std::fprintf(f, "| metric | v_circular_initial / target_perigee_radius / controlled_deorbit_dv |\n");
    std::fprintf(f, "| value | metric value at full precision (the md table above rounds to 0.1) |\n");
    std::fprintf(f, "| units | m/s or km |\n");
    std::fprintf(f, "| notes | formula / provenance |\n");

    std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string out_dir = (argc > 1) ? argv[1] : "generated";
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);

    const Config cfg;
    const EdtConfig ecfg;
    const double stop_km = cfg.reentry_handoff_altitude_km;

    const DebrisCatalog A = catalog_A();
    const DebrisCatalog B = catalog_B();
    const DebrisCatalog C = catalog_C();

    std::FILE* f = std::fopen((out_dir + "/wp13_kit_trade.csv").c_str(), "w");
    if (!f) return 1;
    std::fprintf(f,
        "schema_version,catalog,mass_kg,altitude_km,inclination_deg,record_type,"
        "kit_option,value,value_lo,value_hi,units,notes\n");

    // --- Catalog A: SL-16/Zenit-2 -- sail infeasible, EDT candidate with
    //     open risks (T7 libration, plasma Ne PLACEHOLDER). -------------------
    {
        const double a25_max = area_for_target_years(A, 25.0, cfg.kit_mass_kg,
            cfg.drag_cd, stop_km, cfg.solar_max_density_factor);
        const double a25_min = area_for_target_years(A, 25.0, cfg.kit_mass_kg,
            cfg.drag_cd, stop_km, cfg.solar_min_density_factor);
        const EdtResult edt = edt_deorbit_years(A, ecfg, stop_km);
        write_common_rows(f, A, cfg, ecfg, a25_max, a25_min, edt);

        char notes[512];
        std::snprintf(notes, sizeof(notes),
            "sail infeasible (%.0f..%.0f m2 for the 25-yr guideline, solar "
            "max..min); EDT (band %.1f..%.1f yr) subject to T7; libration "
            "(T7, Pelaez et al. 2000) and plasma Ne (PLACEHOLDER solar-min/"
            "max pair, wp13-literature.md Topic 3) remain open -- an honest "
            "EDT candidate, not a closed recommendation",
            a25_max, a25_min, edt.years_optimistic, edt.years_conservative);
        std::fprintf(f,
            "%s,%s,%.1f,%.1f,%.1f,recommended_kit,edt-candidate-open-risks,"
            "0.0,0.0,0.0,[-],\"%s\"\n",
            kSchema, A.name, A.mass_kg, A.altitude_km, A.inclination_deg, notes);
    }

    // --- Catalog B: SL-8/Kosmos-3M -- sail closes, no EDT needed. -----------
    {
        const double a25_max = area_for_target_years(B, 25.0, cfg.kit_mass_kg,
            cfg.drag_cd, stop_km, cfg.solar_max_density_factor);
        const double a25_min = area_for_target_years(B, 25.0, cfg.kit_mass_kg,
            cfg.drag_cd, stop_km, cfg.solar_min_density_factor);
        const EdtResult edt = edt_deorbit_years(B, ecfg, stop_km);
        write_common_rows(f, B, cfg, ecfg, a25_max, a25_min, edt);

        char notes[256];
        std::snprintf(notes, sizeof(notes),
            "sail closes (%.0f..%.0f m2 for the 25-yr guideline, solar "
            "max..min); no EDT kit needed for this class",
            a25_max, a25_min);
        std::fprintf(f,
            "%s,%s,%.1f,%.1f,%.1f,recommended_kit,sail,0.0,0.0,0.0,[-],\"%s\"\n",
            kSchema, B.name, B.mass_kg, B.altitude_km, B.inclination_deg, notes);
    }

    // --- Catalog C: Envisat-class -- controlled-reentry mission class,
    //     not a sail/EDT kit recommendation (see wp13_classC.md). -----------
    {
        const double a25_max = area_for_target_years(C, 25.0, cfg.kit_mass_kg,
            cfg.drag_cd, stop_km, cfg.solar_max_density_factor);
        const double a25_min = area_for_target_years(C, 25.0, cfg.kit_mass_kg,
            cfg.drag_cd, stop_km, cfg.solar_min_density_factor);
        const EdtResult edt = edt_deorbit_years(C, ecfg, stop_km);
        write_common_rows(f, C, cfg, ecfg, a25_max, a25_min, edt);

        std::fprintf(f,
            "%s,%s,%.1f,%.1f,%.1f,recommended_kit,controlled-reentry-mission-class,"
            "0.0,0.0,0.0,[-],\"massive (%.0f kg) high-inclination payload -- "
            "sail/EDT kit-only deorbit is not the recommended mission class; "
            "see generated/wp13_classC.md for the controlled-reentry "
            "comparison\"\n",
            kSchema, C.name, C.mass_kg, C.altitude_km, C.inclination_deg,
            C.mass_kg);
    }

    std::fclose(f);

    write_kit_trade_schema_md(out_dir);
    write_classc_csv(out_dir, A, C);
    write_classc_md(out_dir, A, C);

    std::printf("[WP13] wrote %s/wp13_kit_trade.csv, wp13_kit_trade_schema.md, "
                "wp13_classC.csv and wp13_classC.md (3 catalogs, stop %.0f km)\n",
                out_dir.c_str(), stop_km);
    return 0;
}
