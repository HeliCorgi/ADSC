#!/usr/bin/env python3
"""WP14 Prioritization table -> generated/wp14_prioritization.{csv,md}

A pure derived/joined view over three already-committed, already-regenerated
CSVs -- NO new simulation, NO new statistics. Every number below is either
read directly from a committed row, or is a single elementwise arithmetic
combination of two already-committed numbers (always noted where it occurs,
e.g. cost_per_removal_musd for a catalog that has no dedicated WP6 anchor row
is computed as CU_percentile x anchor_musd_per_cu -- exactly the same
multiplication WP6/WP14 already performs for the anchor catalog itself).
Zero hand-written numbers. Python 3 standard library only (spec v5 R9).

Inputs (schema versions read from the files themselves, never hardcoded):
  generated/wp6_cost_summary.csv   (schema 1.1: cost/FoM/currency-anchor rows)
  generated/wp13_kit_trade.csv     (schema 1.0: per-class kit/EDT/sail trade)
  generated/wp5_campaign_summary.csv (schema 1.1: campaign Monte Carlo rates)

Discipline (spec v5 R6/D10/R11):
  * No wall-clock timestamp or run time is embedded in any output.
  * All numeric fields use fixed-decimal formatting so the byte output is
    stable across platforms and runs (the CI "regeneration == committed"
    gate).
  * No point-value currency figure is ever printed -- absolute-cost numbers
    appear ONLY as low/mid/high cited ranges (the same D10 discipline as
    wp6_cost_summary.csv).
  * D12: legal accessibility enters as a gate + metadata flags via the WP8
    compliance engine, never as a multiplier in the FoM or in any rank score
    computed here.
  * ASCII only, LF-only line endings.

Usage:  make_prioritization.py <repo_root> [out_dir]
        out_dir defaults to <repo_root>/generated
"""
import csv
import os
import sys

CAT_A = "SL-16 / Zenit-2 second stage"
CAT_B = "SL-8 / Kosmos-3M second stage"
CAT_C = "Envisat-class massive SSO payload"

WEIGHTINGS = ("spatial_density", "criticality")
SCENARIOS = ("low", "mid", "high")

CSV_HEADER = [
    "schema_version", "catalog", "record_type", "metric", "weighting",
    "cost_scenario", "kit_option", "estimate", "p05", "p50", "p95",
    "value_lo", "value_hi", "units", "notes",
]
SCHEMA_VERSION = "1.0"


# ---- fixed formatting (platform-stable) ------------------------------------
def f0(v):
    return "%.0f" % float(v)


def f1(v):
    return "%.1f" % float(v)


def f2(v):
    return "%.2f" % float(v)


def f3(v):
    return "%.3f" % float(v)


def f4(v):
    return "%.4f" % float(v)


def f6(v):
    return "%.6f" % float(v)


def band_str(lo, hi, fmt):
    """'lo..hi' honoring the wp13_kit_trade.csv convention that a non-numeric
    band field (e.g. the exact-polar 'n/a (...)' text) is the honest physical
    null result, not missing data -- printed verbatim rather than reformatted."""
    try:
        return "%s..%s" % (fmt(float(lo)), fmt(float(hi)))
    except ValueError:
        return "%s .. %s" % (lo, hi)


def load_rows(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def write_text(path, s):
    # newline="\n" forces LF on every platform so committed bytes are stable.
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(s)


# ---- literature citations (D2/D12; hand-written prose, not data -- the same
# ---- convention tools/evidence/make_evidence.py already uses for its own
# ---- Kessler/Liou/McKnight citations) --------------------------------------
MCKNIGHT_CITATION = (
    "McKnight, D., Witner, R., Letizia, F., Lemmens, S., Anselmo, L., "
    "Pardini, C., Rossi, A., Kunstadter, C., et al. (2021), \"Identifying "
    "the 50 statistically-most-concerning derelict objects in LEO,\" Acta "
    "Astronautica 181, 282-291, doi:10.1016/j.actaastro.2021.01.021 "
    "(wp13-literature.md Topic 6): 11 expert teams' consensus top-50 most-"
    "concerning LEO objects is dominated by rocket bodies, and the top ~20 "
    "are ALL SL-16 (Zenit-2) second stages (average mass of the top-50 "
    "objects ~5,295 kg); removing the top-50 would roughly halve LEO "
    "collision risk (top-10 ~30% reduction)."
)
SL8_POPULATION_CITATION = (
    "wp13-literature.md Topic 5b: SL-8 (Kosmos-3M) second-stage clusters "
    "span three altitude boxes -- Box 3 ~700-800 km (50 R/Bs), Box 2 "
    "~900-1000 km (143 R/Bs), Box 1 ~1500-1600 km (44 R/Bs), inclinations "
    "~74/83 deg -- a combined population an order of magnitude larger than "
    "the SL-16 top-20 concern list; the class appears prominently in SL-8-"
    "specific ADR mission studies (e.g. ADReS-A, Springer "
    "10.1007/978-3-319-15982-9_3)."
)
ENVISAT_CITATION = (
    "wp13-literature.md Topic 6 item 1: Envisat (ESA), mass 8,211 kg "
    "(Service Module 2,673 + PEB 1,021 + Payload Carrier 2,078 + Fuel 319 "
    "+ Instruments 2,118 kg), sun-synchronous orbit ~765-800 km, "
    "inclination ~98.4-98.55 deg, defunct since April 2012; the archetypal "
    "'too massive / high-casualty-risk-on-uncontrolled-reentry' object "
    "cited across the debris-mitigation literature (eoPortal; ESA Earth "
    "Online)."
)
D12_SENTENCE = (
    "Legal accessibility enters as a gate and metadata flags, never a "
    "multiplier (D12, adsc-specification-v5.md section 3): the WP8 "
    "compliance engine (`tools/compliance/check_compliance.py`, policy "
    "`ADSC-POL-01`) BLOCKs unconsented active debris removal, and this "
    "prioritization table consumes its PASS/BLOCK output and metadata "
    "flags rather than folding a subjective legal weight into the FoM or "
    "into any rank below. No catalog in this document is ranked, up or "
    "down, on legal/consent grounds -- consent is a gate, not a scoring "
    "input."
)
HONESTY_NOTE = (
    "The criticality weighting's citation basis is McKnight-anchored at "
    "the ~840-850 km peak (McKnight et al. 2021: the top ~20 most-"
    "concerning LEO objects are all SL-16/Zenit-2 second stages, ~9 t "
    "class, ~850 km -- exactly catalog A's band), which is a real, cited "
    "anchor point for catalog A's criticality weight w(h=840)=1.00. But "
    "the REST of the criticality weighting table "
    "(`CostConfig::weight_table`, the non-peak altitude/weight pairs away "
    "from 840-850 km) remains PLACEHOLDER: the McKnight ranking gives a "
    "strong, cited reason to weight the ~840-850 km peak high, but it "
    "does not by itself supply a citable functional form for how the "
    "weight falls off away from that peak, so the shape between anchor "
    "points is still an uncited interpolation (T5 open). The dressed-up-"
    "mass-sort critique from the WP14 review is acknowledged here, not "
    "dismissed: because both weighting tables today have limited dynamic "
    "range and removed mass dominates the FoM ratio, a skeptical reader "
    "is right to ask whether the 'criticality' weighting is doing real "
    "discriminating work beyond re-deriving a mass ranking; strengthening "
    "the non-peak citation basis (or accepting the narrower dynamic range "
    "as the honest state of the evidence) is the open item, not resolved "
    "by this document."
)


class Data:
    """Reads the three committed CSVs; exposes only values that appear in
    them (no hand-written numbers)."""

    def __init__(self, gen):
        self.wp6 = load_rows(os.path.join(gen, "wp6_cost_summary.csv"))
        self.wp13 = load_rows(os.path.join(gen, "wp13_kit_trade.csv"))
        self.wp5 = load_rows(os.path.join(gen, "wp5_campaign_summary.csv"))
        self.wp6_schema = self.wp6[0]["schema_version"]
        self.wp13_schema = self.wp13[0]["schema_version"]
        self.wp5_schema = self.wp5[0]["schema_version"]
        self.catalogs13 = []
        for r in self.wp13:
            if r["catalog"] not in self.catalogs13:
                self.catalogs13.append(r["catalog"])
        self.campaign_catalogs = []
        for r in self.wp5:
            if r["catalog"] not in self.campaign_catalogs:
                self.campaign_catalogs.append(r["catalog"])

    def wp6q(self, catalog, record_type, metric, n_kits=None, weighting=None,
             cost_scenario=None):
        out = []
        for r in self.wp6:
            if (r["catalog"] == catalog and r["record_type"] == record_type
                    and r["metric"] == metric
                    and (n_kits is None or r["n_kits"] == str(n_kits))
                    and (weighting is None or r["weighting"] == weighting)
                    and (cost_scenario is None
                         or r["cost_scenario"] == cost_scenario)):
                out.append(r)
        if len(out) != 1:
            raise KeyError((catalog, record_type, metric, n_kits, weighting,
                            cost_scenario, "rows_found=%d" % len(out)))
        return out[0]

    def wp6_anchor(self, cost_scenario):
        """currency_anchor_derived row (global; catalog is blank in the CSV)."""
        return self.wp6q("", "currency_anchor_derived", "anchor_musd_per_cu",
                          cost_scenario=cost_scenario)

    def wp6_baseline_n(self, catalog):
        """Baseline kit count N, read from the cost_component rows' own
        n_kits column (never hardcoded)."""
        ns = {r["n_kits"] for r in self.wp6
              if r["catalog"] == catalog and r["record_type"] == "cost_component"}
        if len(ns) != 1:
            raise KeyError(("wp6_baseline_n", catalog, ns))
        return int(next(iter(ns)))

    def wp13v(self, catalog, record_type, kit_option=None):
        out = []
        for r in self.wp13:
            if (r["catalog"] == catalog and r["record_type"] == record_type
                    and (kit_option is None or r["kit_option"] == kit_option)):
                out.append(r)
        if len(out) != 1:
            raise KeyError((catalog, record_type, kit_option,
                            "rows_found=%d" % len(out)))
        return out[0]


def short(catalog):
    return catalog.split("/")[0].strip()


# ---- row builder ------------------------------------------------------------
def row(catalog="", record_type="", metric="", weighting="", cost_scenario="",
        kit_option="", estimate="", p05="", p50="", p95="", value_lo="",
        value_hi="", units="", notes=""):
    return {
        "schema_version": SCHEMA_VERSION, "catalog": catalog,
        "record_type": record_type, "metric": metric, "weighting": weighting,
        "cost_scenario": cost_scenario, "kit_option": kit_option,
        "estimate": estimate, "p05": p05, "p50": p50, "p95": p95,
        "value_lo": value_lo, "value_hi": value_hi, "units": units,
        "notes": notes,
    }


def cu_to_musd(cu_row, anchor_row):
    """(p05, p50, p95) in MUSD/removal, elementwise CU_percentile x
    anchor_musd_per_cu -- the identical multiplication WP6/WP14 already
    performs for the anchor catalog's own cost_per_removal_musd rows."""
    a = float(anchor_row["estimate"])
    return (float(cu_row["p05"]) * a, float(cu_row["p50"]) * a,
            float(cu_row["p95"]) * a)


def build_rows(d):
    rows = []
    anchors = {s: d.wp6_anchor(s) for s in SCENARIOS}
    baseline_n = {}
    cu_baseline = {}

    for cat in (CAT_A, CAT_B):
        baseline_n[cat] = d.wp6_baseline_n(cat)
        n = baseline_n[cat]

        # FoM under both weightings, p05/p50/p95 [kg/CU] -- machine-read.
        for w in WEIGHTINGS:
            fr = d.wp6q(cat, "fom", "fom", weighting=w)
            bw = d.wp6q(cat, "fom", "band_weight", weighting=w)
            rows.append(row(catalog=cat, record_type="fom", metric="fom",
                             weighting=w, estimate=fr["estimate"], p05=fr["p05"],
                             p50=fr["p50"], p95=fr["p95"], units=fr["units"],
                             notes=fr["notes"]))
            rows.append(row(catalog=cat, record_type="fom", metric="band_weight",
                             weighting=w, estimate=bw["estimate"],
                             units=bw["units"], notes=bw["notes"]))

        rmass = d.wp6q(cat, "fom", "removed_mass_per_mission")
        rows.append(row(catalog=cat, record_type="fom",
                         metric="removed_mass_per_mission",
                         estimate=rmass["estimate"], units=rmass["units"],
                         notes=rmass["notes"]))

        # Cost per removal, CU, at the baseline (Delta-v-limited) N.
        cu = d.wp6q(cat, "amortization", "cost_per_removal", n_kits=n)
        cu_baseline[cat] = cu
        rows.append(row(catalog=cat, record_type="cost_per_removal_cu",
                         metric="cost_per_removal", p05=cu["p05"], p50=cu["p50"],
                         p95=cu["p95"], units=cu["units"],
                         notes="baseline N=%d (the Delta-v-limited amortization "
                               "minimum, generated/wp6_cost_summary.csv "
                               "record_type=amortization)" % n))

        # Cost per removal, MUSD, per cost_scenario -- via the anchor rows.
        for s in SCENARIOS:
            p05, p50, p95 = cu_to_musd(cu, anchors[s])
            if cat == CAT_A:
                # CAT_A already has a dedicated WP6 cost_per_removal_musd row
                # (the anchor was itself derived from CAT_A's own baseline
                # cost). Cross-check rather than trust: this catches any
                # future WP6 change silently drifting from this tool's own
                # arithmetic.
                wp6row = d.wp6q(cat, "cost_per_removal_musd", "cost_per_removal",
                                 n_kits=n, cost_scenario=s)
                for label, mine, theirs in (("p05", p05, wp6row["p05"]),
                                            ("p50", p50, wp6row["p50"]),
                                            ("p95", p95, wp6row["p95"])):
                    # Both operands are read back from 6-decimal CSV text, so
                    # each carries up to 5e-7 of independent rounding error;
                    # at the ~45 CU / ~0.06 MUSD/CU magnitudes here that
                    # compounds to ~3e-5 in the product -- a real drift bug
                    # would be orders of magnitude larger than this floor.
                    if abs(mine - float(theirs)) > 5e-5:
                        raise AssertionError(
                            "cost_per_removal_musd cross-check failed for "
                            "%s/%s/%s: computed %.6f != committed WP6 %.6f"
                            % (cat, s, label, mine, float(theirs)))
                note = ("= cost_per_removal_CU(catalog=%s;baseline N=%d) x "
                        "anchor_musd_per_cu[%s]=%s MUSD/CU "
                        "(generated/wp6_cost_summary.csv "
                        "record_type=currency_anchor_derived); cross-checked "
                        "against that same file's own "
                        "record_type=cost_per_removal_musd row for this "
                        "catalog/scenario -- identical by construction, since "
                        "the anchor was itself derived from this catalog's "
                        "baseline cost" % (cat, n, s, f6(anchors[s]["estimate"])))
            else:
                note = ("= cost_per_removal_CU(catalog=%s;baseline N=%d) x "
                        "anchor_musd_per_cu[%s]=%s MUSD/CU (generated/"
                        "wp6_cost_summary.csv record_type="
                        "currency_anchor_derived); NOTE: the anchor itself "
                        "was derived from catalog A's (%s) baseline campaign "
                        "cost (Sec.10 Method), not this catalog's own -- "
                        "applying it here assumes the CU->MUSD conversion "
                        "factor is campaign-structure-invariant across "
                        "catalogs rather than catalog-specific; a stated "
                        "simplifying assumption, not a hidden one"
                        % (cat, n, s, f6(anchors[s]["estimate"]), short(CAT_A)))
            rows.append(row(catalog=cat, record_type="cost_per_removal_musd",
                             metric="cost_per_removal", cost_scenario=s,
                             estimate=f6(p50), p05=f6(p05), p50=f6(p50),
                             p95=f6(p95), units="MUSD_per_removal", notes=note))

    # Envisat-class: no WP5/WP6 campaign rows -- say so explicitly.
    rows.append(row(catalog=CAT_C, record_type="mission_class",
                     metric="not_kit_ranked",
                     notes="Envisat-class massive SSO payload has no WP5/WP6 "
                           "campaign rows (it is not part of the kit-installer "
                           "Monte Carlo sweep); FoM and cost_per_removal are "
                           "undefined for this class in this joined view. It "
                           "is evaluated instead as a separate controlled-"
                           "reentry mission class (spec WP13 Class C; see "
                           "generated/wp13_classC.md), not kit-ranked against "
                           "SL-16/SL-8."))

    # Recommended kit + sail/EDT band, all three catalogs (wp13 only).
    for cat in d.catalogs13:
        rk = d.wp13v(cat, "recommended_kit")
        rows.append(row(catalog=cat, record_type="recommended_kit",
                         kit_option=rk["kit_option"], notes=rk["notes"]))
        sail = d.wp13v(cat, "sail_area_25yr")
        rows.append(row(catalog=cat, record_type="sail_area_25yr",
                         value_lo=sail["value_lo"], value_hi=sail["value_hi"],
                         units=sail["units"], notes=sail["notes"]))
        edt = d.wp13v(cat, "edt_years")
        rows.append(row(catalog=cat, record_type="edt_years",
                         value_lo=edt["value_lo"], value_hi=edt["value_hi"],
                         units=edt["units"], notes=edt["notes"]))

    # Priority rank -- the deliverable conclusion of this document, not a
    # measured quantity; the supporting FoM/cost/citation rows above and the
    # one-paragraph defenses in the .md are the evidence for it.
    rows.append(row(catalog=CAT_A, record_type="priority_rank", metric="rank",
                     estimate="1",
                     notes="mass-dominant FoM under BOTH weightings + "
                           "McKnight top-20-all-SL-16 citation; see the .md "
                           "defense paragraph"))
    rows.append(row(catalog=CAT_B, record_type="priority_rank", metric="rank",
                     estimate="2",
                     notes="cheap closing sail kit, high per-class population; "
                           "see the .md defense paragraph"))
    rows.append(row(catalog=CAT_C, record_type="priority_rank", metric="rank",
                     kit_option="not-kit-ranked",
                     notes="separate controlled-reentry mission class (spec "
                           "WP13 Class C), not ranked against the two "
                           "kit-installer classes above; see the .md defense "
                           "paragraph"))

    return rows, anchors, baseline_n, cu_baseline


def write_csv(path, rows):
    lines = [",".join(CSV_HEADER)]
    for r in rows:
        cells = []
        for col in CSV_HEADER:
            v = str(r[col])
            if any(c in v for c in (",", "\"", "\n")):
                v = "\"%s\"" % v.replace("\"", "\"\"")
            cells.append(v)
        lines.append(",".join(cells))
    write_text(path, "\n".join(lines) + "\n")


def build_md(d, anchors, baseline_n, cu_baseline):
    L = []
    w = L.append

    fomA_sp = d.wp6q(CAT_A, "fom", "fom", weighting="spatial_density")
    fomA_cr = d.wp6q(CAT_A, "fom", "fom", weighting="criticality")
    fomB_sp = d.wp6q(CAT_B, "fom", "fom", weighting="spatial_density")
    fomB_cr = d.wp6q(CAT_B, "fom", "fom", weighting="criticality")
    bwA_sp = d.wp6q(CAT_A, "fom", "band_weight", weighting="spatial_density")
    bwA_cr = d.wp6q(CAT_A, "fom", "band_weight", weighting="criticality")
    bwB_sp = d.wp6q(CAT_B, "fom", "band_weight", weighting="spatial_density")
    bwB_cr = d.wp6q(CAT_B, "fom", "band_weight", weighting="criticality")

    w("# WP14 Prioritization: target-class ranking (cost + FoM + kit trade)")
    w("")
    w("A pure derived/joined view over three already-committed, already-")
    w("regenerated CSVs -- `generated/wp6_cost_summary.csv` (schema %s), "
      "`generated/wp13_kit_trade.csv` (schema %s), and "
      "`generated/wp5_campaign_summary.csv` (schema %s). This tool runs no "
      "new simulation and adds no new statistics; every number below is read "
      "directly from those files, or is a single elementwise arithmetic "
      "combination of two already-committed numbers (noted inline wherever "
      "that occurs). Regenerated by "
      "`tools/prioritization/make_prioritization.py` (Python 3 stdlib only, "
      "R9)." % (d.wp6_schema, d.wp13_schema, d.wp5_schema))
    w("")
    w("## Ranking (spec:265-266)")
    w("")
    w("1. **%s** -- mass-dominant FoM under both weightings; the McKnight "
      "top-of-ranking class." % CAT_A)
    w("2. **%s** -- cheap closing sail kit, high per-class population."
      % CAT_B)
    w("3. **%s** -- a separate controlled-reentry mission class, "
      "NOT kit-ranked against the two classes above." % CAT_C)
    w("")
    w("## Legal/consent gate (D12)")
    w("")
    w(D12_SENTENCE)
    w("")

    # ---- Catalog A -----------------------------------------------------
    w("## 1. %s (priority 1)" % CAT_A)
    w("")
    w("### Figure of merit (debris-risk reduction per cost), p50 [kg/CU]")
    w("")
    w("| weighting | FoM p50 | FoM p05..p95 | band weight w(h) |")
    w("|---|---:|---|---:|")
    w("| spatial_density | %s | %s..%s | %s |" % (
        f1(fomA_sp["p50"]), f1(fomA_sp["p05"]), f1(fomA_sp["p95"]),
        f2(bwA_sp["estimate"])))
    w("| criticality | %s | %s..%s | %s |" % (
        f1(fomA_cr["p50"]), f1(fomA_cr["p05"]), f1(fomA_cr["p95"]),
        f2(bwA_cr["estimate"])))
    w("")
    nA = baseline_n[CAT_A]
    cuA = cu_baseline[CAT_A]
    w("### Cost per removal (baseline N=%d, the Delta-v-limited amortization "
      "minimum)" % nA)
    w("")
    w("- CU: p05 %s / p50 %s / p95 %s CU/removal (relative cost units, "
      "primary metric)." % (f2(cuA["p05"]), f2(cuA["p50"]), f2(cuA["p95"])))
    w("- MUSD (external, low-standing -- see the honesty framing quoted in "
      "the WP14 evidence-pack cost section), p50 per cost_scenario:")
    for s in SCENARIOS:
        p05, p50, p95 = cu_to_musd(cuA, anchors[s])
        w("  - %s: p05 %s / p50 %s / p95 %s MUSD/removal" % (
            s, f3(p05), f3(p50), f3(p95)))
    w("")
    w("### Recommended kit")
    w("")
    rkA = d.wp13v(CAT_A, "recommended_kit")
    sailA = d.wp13v(CAT_A, "sail_area_25yr")
    edtA = d.wp13v(CAT_A, "edt_years")
    w("- kit_option: `%s`" % rkA["kit_option"])
    w("- sail area, 25-yr guideline (solar max..min): %s m^2 -- "
      "impractical at the upper end." % band_str(sailA["value_lo"],
                                                  sailA["value_hi"], f0))
    w("- EDT deorbit-time band: %s yr %s" % (
        band_str(edtA["value_lo"], edtA["value_hi"], f1), edtA["notes"]))
    w("- verbatim rationale, quoted from `generated/wp13_kit_trade.csv` (never "
      "hand-transcribed): \"%s\"" % rkA["notes"])
    w("")
    w("### Prioritization defense (one paragraph)")
    w("")
    w("The %s class ranks first because its FoM dominates the %s class "
      "under BOTH weightings (spatial-density %s vs %s kg/CU p50; "
      "criticality %s vs %s kg/CU p50 -- computed from "
      "generated/wp6_cost_summary.csv), which is the mass-dominance result "
      "the WP6/WP14 cost model already establishes; that dominance is "
      "corroborated, not merely asserted, by the published debris-ranking "
      "literature: %s The class is the ADSC catalog-A anchor for exactly "
      "this reason. Its recommended kit is an honest EDT candidate, not a "
      "closed recommendation: sail-only does not close the 25-year "
      "guideline for this class (%s m^2, impractical), so the "
      "electrodynamic-tether branch is carried instead, and that branch "
      "remains open on the libration/dynamic-tether-stability question "
      "(T7, Pelaez et al. 2000, cited in wp13-literature.md) -- the EDT "
      "years band above folds libration in only as a flat PLACEHOLDER "
      "duty-cycle knob, never claimed solved. Legal accessibility is a "
      "gate, not a factor in this ranking (D12, above)." % (
          short(CAT_A), short(CAT_B), f1(fomA_sp["p50"]), f1(fomB_sp["p50"]),
          f1(fomA_cr["p50"]), f1(fomB_cr["p50"]), MCKNIGHT_CITATION,
          band_str(sailA["value_lo"], sailA["value_hi"], f0)))
    w("")

    # ---- Catalog B -----------------------------------------------------
    w("## 2. %s (priority 2)" % CAT_B)
    w("")
    w("### Figure of merit (debris-risk reduction per cost), p50 [kg/CU]")
    w("")
    w("| weighting | FoM p50 | FoM p05..p95 | band weight w(h) |")
    w("|---|---:|---|---:|")
    w("| spatial_density | %s | %s..%s | %s |" % (
        f1(fomB_sp["p50"]), f1(fomB_sp["p05"]), f1(fomB_sp["p95"]),
        f2(bwB_sp["estimate"])))
    w("| criticality | %s | %s..%s | %s |" % (
        f1(fomB_cr["p50"]), f1(fomB_cr["p05"]), f1(fomB_cr["p95"]),
        f2(bwB_cr["estimate"])))
    w("")
    nB = baseline_n[CAT_B]
    cuB = cu_baseline[CAT_B]
    w("### Cost per removal (baseline N=%d, the Delta-v-limited amortization "
      "minimum)" % nB)
    w("")
    w("- CU: p05 %s / p50 %s / p95 %s CU/removal (relative cost units, "
      "primary metric)." % (f2(cuB["p05"]), f2(cuB["p50"]), f2(cuB["p95"])))
    w("- MUSD (external, low-standing; see the honesty framing quoted in "
      "the WP14 evidence-pack cost section), p50 per cost_scenario -- note "
      "this catalog reuses catalog A's currency anchor (see the CSV notes "
      "column for the stated simplifying assumption):")
    for s in SCENARIOS:
        p05, p50, p95 = cu_to_musd(cuB, anchors[s])
        w("  - %s: p05 %s / p50 %s / p95 %s MUSD/removal" % (
            s, f3(p05), f3(p50), f3(p95)))
    w("")
    w("### Recommended kit")
    w("")
    rkB = d.wp13v(CAT_B, "recommended_kit")
    sailB = d.wp13v(CAT_B, "sail_area_25yr")
    w("- kit_option: `%s`" % rkB["kit_option"])
    w("- sail area, 25-yr guideline (solar max..min): %s m^2 -- tens of "
      "square meters, a practical kit; no EDT kit needed for this class."
      % band_str(sailB["value_lo"], sailB["value_hi"], f0))
    w("- verbatim rationale, quoted from `generated/wp13_kit_trade.csv` (never "
      "hand-transcribed): \"%s\"" % rkB["notes"])
    w("")
    w("### Prioritization defense (one paragraph)")
    w("")
    w("The %s class ranks second: its FoM is smaller than %s's under both "
      "weightings (mass-dominated, as above), but it closes today with a "
      "cheap, practical sail kit (%s m^2 for the 25-year guideline, tens of "
      "square meters -- no EDT branch and no open T7 caveat needed), and it "
      "represents a much larger per-class population than the SL-16 top-20 "
      "concern list: %s Cost per removal at its own baseline N is "
      "essentially the same as %s's in CU terms (%s vs %s CU/removal p50), "
      "so the batch-amortization economics are not a differentiator between "
      "the two classes -- kit simplicity and closure certainty are. Legal "
      "accessibility is a gate, not a factor in this ranking (D12, above)."
      % (short(CAT_B), short(CAT_A),
         band_str(sailB["value_lo"], sailB["value_hi"], f0),
         SL8_POPULATION_CITATION, short(CAT_A), f2(cuB["p50"]),
         f2(cuA["p50"])))
    w("")

    # ---- Catalog C (Envisat) -------------------------------------------
    w("## 3. %s (separate mission class, not kit-ranked)" % CAT_C)
    w("")
    w("No WP5/WP6 campaign rows exist for this catalog (it is not part of "
      "the kit-installer Monte Carlo sweep), so FoM and cost_per_removal "
      "are undefined for it here -- stated rather than silently omitted.")
    w("")
    rkC = d.wp13v(CAT_C, "recommended_kit")
    sailC = d.wp13v(CAT_C, "sail_area_25yr")
    edtC = d.wp13v(CAT_C, "edt_years")
    w("### Why it is not a kit choice")
    w("")
    w("- sail area, 25-yr guideline (solar max..min): %s m^2 (context "
      "only -- not the recommendation)." % band_str(
          sailC["value_lo"], sailC["value_hi"], f0))
    w("- EDT deorbit-time band: %s yr (context only -- not the "
      "recommendation)." % band_str(edtC["value_lo"], edtC["value_hi"], f1))
    w("- kit_option: `%s`" % rkC["kit_option"])
    w("- verbatim rationale, quoted from `generated/wp13_kit_trade.csv` (never "
      "hand-transcribed): \"%s\"" % rkC["notes"])
    w("")
    w("### Prioritization defense (one paragraph)")
    w("")
    w("The %s is evaluated as a separate controlled-reentry mission class "
      "(spec WP13 Class C), not ranked against %s or %s: %s Controlled-"
      "deorbit delta-v, casualty-risk framing and cost detail for this "
      "candidate live in `generated/wp13_classC.{csv,md}`, out of scope for "
      "this cost/FoM joined view; what belongs here is the negative result "
      "itself, stated plainly -- a sail or EDT kit-only deorbit is not the "
      "recommended path for an 8,211 kg, ~98.4 deg payload, so it is not "
      "assigned a kit-ranked priority number alongside the SL-16/SL-8 "
      "classes above. Legal accessibility is a gate, not a factor here "
      "either (D12, above)." % (short(CAT_C), short(CAT_A), short(CAT_B),
                                ENVISAT_CITATION))
    w("")

    w("## Honesty note (T5, criticality weighting citation basis)")
    w("")
    w(HONESTY_NOTE)
    w("")
    w("---")
    w("*Generated by `tools/prioritization/make_prioritization.py` from "
      "committed artifacts only. This is not legal advice.*")
    w("")
    return "\n".join(L)


def main():
    if len(sys.argv) < 2:
        print("usage: make_prioritization.py <repo_root> [out_dir]",
              file=sys.stderr)
        return 2
    repo = sys.argv[1]
    gen = os.path.join(repo, "generated")
    out_dir = sys.argv[2] if len(sys.argv) > 2 else gen
    os.makedirs(out_dir, exist_ok=True)

    d = Data(gen)
    rows, anchors, baseline_n, cu_baseline = build_rows(d)

    csv_path = os.path.join(out_dir, "wp14_prioritization.csv")
    md_path = os.path.join(out_dir, "wp14_prioritization.md")
    write_csv(csv_path, rows)
    md_text = build_md(d, anchors, baseline_n, cu_baseline)
    md_text.encode("ascii")  # hard guarantee: ASCII only
    write_text(md_path, md_text)
    print("[WP14] wrote wp14_prioritization.csv (%d rows) + "
          "wp14_prioritization.md (%d lines)"
          % (len(rows), md_text.count("\n") + 1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
