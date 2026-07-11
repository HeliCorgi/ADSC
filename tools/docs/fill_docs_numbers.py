#!/usr/bin/env python3
"""docs/ marker-block filler (WP15, R16 mechanism).

  fill_docs_numbers.py <repo_root> [--check]

R16 (`adsc-specification-v5.md` section 5): files under docs/ may state
numbers only via generated includes or explicit references to evidence
artifacts. This tool is the generated-include mechanism: it regenerates ONLY
the text strictly between each committed `<!-- DOCS-<NAME>-START ... -->` /
`<!-- DOCS-<NAME>-END -->` marker-comment pair in every `docs/*.md` file, from
the committed CSVs those markers cite. The marker comments themselves (and
everything outside them) are literal, hand-maintained prose and are never
touched by this tool -- exactly the same discipline
`tools/readme/fill_readme_numbers.py` already established for README.md's
WP5-NUMBERS/WP6-NUMBERS blocks (this tool's WP5/WP6 *full-table* builders are
the direct descendants of that script's original build_wp5_block()/
build_wp6_block(), moved here because the full tables now live in docs/ while
README keeps only a compact digest).

Blocks filled (source CSV in parentheses):
  docs/safety.md            DOCS-WP12-LADDER  (generated/wp12_ladder.csv)
  docs/safety.md            DOCS-WP5-FULL     (generated/wp5_campaign_summary.csv)
  docs/cost_model.md        DOCS-WP6-FULL     (generated/wp6_cost_summary.csv)
  docs/gnc.md               DOCS-GNC-WP2      (generated/reference_metrics.csv)
  docs/target_selection.md  DOCS-WP3-DECAY    (generated/wp3_decay_trade.csv)
  docs/target_selection.md  DOCS-WP13-KIT     (generated/wp13_kit_trade.csv)
  docs/target_selection.md  DOCS-WP13-CLASSC  (generated/wp13_classC.csv)
  docs/technical_summary_5p.md DOCS-5P-KEY    (wp5/wp6/reference_metrics)

Default mode rewrites each docs/*.md file in place. `--check` instead exits
non-zero if any regenerated block differs from the committed file (for CI:
a stale doc fails the build instead of silently drifting from the CSVs it
claims to summarize -- see ci.yml).

Python 3 standard library only (spec R9). This tool's own SOURCE stays pure
ASCII (spec R11): the docs' existing Unicode punctuation/math glyphs are
produced via explicit \\uXXXX escapes in ordinary string literals, never as
literal source bytes. No wall-clock timestamps; output is UTF-8 with "\\n"
line endings so the CI byte-reproducibility gate can compare it (matching
every other generator in this repository).
"""
import csv
import os
import re
import sys

# ---- Unicode glyphs used inside the docs/ blocks, named so the source file
# ---- itself stays ASCII-only (spec R11) while the generated text is not --
# ---- each is an explicit \uXXXX escape, never a literal byte.
EM = "\u2014"     # em dash
MINUS = "\u2212"  # minus sign (distinct from ASCII hyphen)
DELTA = "\u0394"  # Greek capital delta (used for "Dv")
GE = "\u2265"     # >=
LE = "\u2264"     # <=
TIMES = "\u00d7"  # multiplication sign
SUBI = "\u1d62"   # Latin subscript small letter i
SIGMA = "\u03a3"  # Greek capital sigma
DOT = "\u00b7"    # middle dot
PM = "\u00b1"     # plus-minus sign
GG = "\u226b"     # much-greater-than
SUP2 = "\u00b2"   # superscript 2 (m^2)
APPROX = "\u2248" # almost-equal
DEG = "\u00b0"    # degree sign
ETA = "\u03b7"    # Greek small letter eta
SQRT = "\u221a"   # square root
MU = "\u03bc"     # Greek small letter mu
ARROW = "\u2192"  # rightwards arrow
SEC = "\u00a7"    # section sign

CATALOG_A = "SL-16 / Zenit-2 second stage"
CATALOG_B = "SL-8 / Kosmos-3M second stage"
CATALOG_C = "Envisat-class massive SSO payload"

CATALOG_SHORT = {CATALOG_A: "SL-16", CATALOG_B: "SL-8"}
CATALOG_FOM_LABEL = {CATALOG_A: "SL-16 (~9 t, 840 km)", CATALOG_B: "SL-8 (~1.4 t, 750 km)"}
CATALOG_DECAY_LABEL = {CATALOG_A: "SL-16 / Zenit-2", CATALOG_B: "SL-8 / Kosmos-3M"}
CATALOG_LETTER = {CATALOG_A: "A", CATALOG_B: "B", CATALOG_C: "C"}

CANDIDATE_LABEL = {
    "Zenit-2 stage (catalog_A)": "Zenit-2 / SL-16 (catalog A)",
    "Envisat-class (catalog_C)": "Envisat-class (catalog C)",
}
CANDIDATE_ORDER = ["Zenit-2 stage (catalog_A)", "Envisat-class (catalog_C)"]


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

def fcomma0(v):
    return "{:,.0f}".format(float(v))


def load_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


# ---------------------------------------------------------------------------
# Data loaders (one per committed CSV this tool reads).
# ---------------------------------------------------------------------------

class Wp5Data:
    """generated/wp5_campaign_summary.csv (schema 1.1)."""

    def __init__(self, root):
        self.rows = load_csv(os.path.join(root, "generated", "wp5_campaign_summary.csv"))
        self.catalogs = []
        for r in self.rows:
            if r["catalog"] not in self.catalogs:
                self.catalogs.append(r["catalog"])
        self.n_runs = int(float(self.rows[0]["n_runs"]))
        self.master_seed_hex = "0x%X" % int(self.rows[0]["master_seed"])

    def m(self, catalog, metric):
        for r in self.rows:
            if r["catalog"] == catalog and r["metric"] == metric:
                return r
        raise KeyError((catalog, metric))


class Wp6Data:
    """generated/wp6_cost_summary.csv (schema 1.1, WP14)."""

    def __init__(self, root):
        self.rows = load_csv(os.path.join(root, "generated", "wp6_cost_summary.csv"))
        self.schema_version = self.rows[0]["schema_version"]

    def q(self, catalog, record_type, metric, n_kits=None, weighting=None):
        for r in self.rows:
            if (r["catalog"] == catalog and r["record_type"] == record_type
                    and r["metric"] == metric
                    and (n_kits is None or r["n_kits"] == str(n_kits))
                    and (weighting is None or r["weighting"] == weighting)):
                return r
        raise KeyError((catalog, record_type, metric, n_kits, weighting))

    def first_row(self, catalog, record_type):
        for r in self.rows:
            if r["catalog"] == catalog and r["record_type"] == record_type:
                return r
        raise KeyError((catalog, record_type))

    def anchor(self, scenario):
        for r in self.rows:
            if (r["record_type"] == "currency_anchor_derived"
                    and r["metric"] == "anchor_musd_per_cu"
                    and r["cost_scenario"] == scenario):
                return r
        raise KeyError(scenario)


class RefMetrics:
    """generated/reference_metrics.csv (schema 1.0)."""

    def __init__(self, root):
        rows = load_csv(os.path.join(root, "generated", "reference_metrics.csv"))
        self.d = {r["metric"]: r["value"] for r in rows}

    def v(self, metric):
        return float(self.d[metric])


class Wp3Data:
    """generated/wp3_decay_trade.csv (schema 1.0)."""

    def __init__(self, root):
        self.rows = load_csv(os.path.join(root, "generated", "wp3_decay_trade.csv"))

    def decay_rows(self, catalog):
        return [r for r in self.rows
                if r["catalog"] == catalog and r["record_type"] == "decay_years"]

    def area25(self, catalog):
        for r in self.rows:
            if r["catalog"] == catalog and r["record_type"] == "area_for_25yr":
                return r
        raise KeyError(catalog)

    def meta(self, catalog):
        for r in self.rows:
            if r["catalog"] == catalog:
                return r
        raise KeyError(catalog)


class Wp13KitData:
    """generated/wp13_kit_trade.csv (schema 1.0)."""

    def __init__(self, root):
        self.rows = load_csv(os.path.join(root, "generated", "wp13_kit_trade.csv"))

    def row(self, catalog, record_type, kit_option=None):
        matches = [r for r in self.rows
                   if r["catalog"] == catalog and r["record_type"] == record_type
                   and (kit_option is None or r["kit_option"] == kit_option)]
        if len(matches) != 1:
            raise RuntimeError(
                "expected exactly 1 wp13_kit_trade.csv row for %r, got %d"
                % ((catalog, record_type, kit_option), len(matches)))
        return matches[0]


class Wp13ClassCData:
    """generated/wp13_classC.csv (schema 1.0)."""

    def __init__(self, root):
        self.rows = load_csv(os.path.join(root, "generated", "wp13_classC.csv"))

    def row(self, candidate, metric):
        for r in self.rows:
            if r["candidate"] == candidate and r["metric"] == metric:
                return r
        raise KeyError((candidate, metric))


class Wp12LadderData:
    """generated/wp12_ladder.csv (schema 1.0)."""

    def __init__(self, root):
        self.rows = load_csv(os.path.join(root, "generated", "wp12_ladder.csv"))

    def ar(self, level, catalog, metric):
        for r in self.rows:
            if (r["section"] == "abort_reverify" and r["level"] == level
                    and r["catalog"] == catalog and r["metric"] == metric):
                return r
        raise KeyError((level, catalog, metric))

    def dispersion_set(self, level, catalog):
        return self.ar(level, catalog, "n_events")["dispersion_set"]

    def forensic_items(self):
        return sorted(set((r["catalog"], r["item"]) for r in self.rows
                           if r["section"] == "forensic14"))

    def forensic_all_clear(self):
        return all(float(r["value"]) == 1.0 for r in self.rows
                   if r["section"] == "forensic14" and r["metric"] == "clears")

    def margin_decay_erosion(self, item):
        vals = {}
        for r in self.rows:
            if (r["section"] == "margin_decay" and r["metric"] == "erosion_m"
                    and r["item"] == item):
                vals[r["catalog"]] = float(r["value"])
        return vals


class Data:
    def __init__(self, root):
        self.d5 = Wp5Data(root)
        self.d6 = Wp6Data(root)
        self.ref = RefMetrics(root)
        self.wp3 = Wp3Data(root)
        self.wp13 = Wp13KitData(root)
        self.classc = Wp13ClassCData(root)
        self.ladder = Wp12LadderData(root)


# ---------------------------------------------------------------------------
# Shared cell formatters (mirrors tools/readme/fill_readme_numbers.py).
# ---------------------------------------------------------------------------

def rate_cell(row):
    return "**%s** [%s, %s]" % (f3(row["estimate"]), f3(row["wilson_low"]),
                                 f3(row["wilson_high"]))


def rate_plain(row):
    return "%s [%s, %s]" % (f3(row["estimate"]), f3(row["wilson_low"]),
                             f3(row["wilson_high"]))


def ko_cell(row):
    m = re.search(r"level tag (\S+), dispersion set (\S+)", row["notes"])
    return "%s [%s, %s]" % (rate_cell(row), m.group(1), m.group(2))


def pctl(row, fmt):
    return "%s / %s / %s" % (fmt(row["p05"]), fmt(row["p50"]), fmt(row["p95"]))


def count(row):
    return int(round(float(row["estimate"])))


def amortization_min(d6, catalog):
    """Returns (nmin, cost_per_removal_p50_at_nmin, ratio_to_n1_p50_at_nmin)."""
    ns = sorted({int(r["n_kits"]) for r in d6.rows
                 if r["record_type"] == "amortization" and r["metric"] == "cost_per_removal"})
    cpr = {n: float(d6.q(catalog, "amortization", "cost_per_removal", n)["p50"]) for n in ns}
    nmin = min(ns, key=lambda n: cpr[n])
    ratio = float(d6.q(catalog, "amortization", "cost_per_removal_ratio_to_n1", nmin)["p50"])
    return nmin, cpr[nmin], ratio


# ---------------------------------------------------------------------------
# Block builders.
# ---------------------------------------------------------------------------

def build_wp5_full(data):
    """docs/safety.md DOCS-WP5-FULL: the full WP5 campaign table (the
    original tools/readme/fill_readme_numbers.py build_wp5_block(), moved
    here verbatim -- README now carries only a compact digest of the same
    CSV, built by tools/readme/fill_readme_numbers.py's own, smaller
    template)."""
    d5 = data.d5
    A, B = d5.catalogs[0], d5.catalogs[1]
    n_runs = d5.n_runs

    succA, succB = d5.m(A, "success_rate"), d5.m(B, "success_rate")
    nptA, nptB = (d5.m(A, "nonproductive_termination_rate"),
                  d5.m(B, "nonproductive_termination_rate"))
    gaA, gaB = d5.m(A, "gate_abort_run_rate"), d5.m(B, "gate_abort_run_rate")
    koA, koB = (d5.m(A, "keep_out_violation_rate"),
                d5.m(B, "keep_out_violation_rate"))
    dvA, dvB = d5.m(A, "dv_used_m_per_s"), d5.m(B, "dv_used_m_per_s")
    kuA, kuB = d5.m(A, "kits_used"), d5.m(B, "kits_used")
    rmA, rmB = (d5.m(A, "removals_per_mission"), d5.m(B, "removals_per_mission"))
    syA, syB = d5.m(A, "sync_arrival_time_s"), d5.m(B, "sync_arrival_time_s")

    dvexA, dvexB = count(d5.m(A, "dv_exhausted")), count(d5.m(B, "dv_exhausted"))
    kitexA, kitexB = (count(d5.m(A, "kit_exhausted")), count(d5.m(B, "kit_exhausted")))
    koCountA, koCountB = (count(d5.m(A, "keep_out_violation")),
                          count(d5.m(B, "keep_out_violation")))
    compA, compB = count(d5.m(A, "completed")), count(d5.m(B, "completed"))
    gaCountA, gaCountB = (count(d5.m(A, "gate_abort")), count(d5.m(B, "gate_abort")))
    stCountA, stCountB = (count(d5.m(A, "sync_timeout")), count(d5.m(B, "sync_timeout")))

    m = re.search(r"level tag (\S+), dispersion set (\S+)", koA["notes"])
    l0tag, dstag = m.group(1), m.group(2)
    dv_limited_pct = int(round(100.0 * dvexA / n_runs))

    L = []
    L.append("The full N = %d campaign (both catalog presets) runs in a few tens of seconds"
              % n_runs)
    L.append("on a CI runner (see the Actions log of the current run for the actual figure).")
    L.append("`success` here means a **productive end** %s the mission installed its"
              % EM)
    L.append("full 4-kit complement (`kit_exhausted`) or cleared all 6 targets (`completed`) %s"
              % EM)
    L.append("not one cut short by %sv exhaustion or a keep-out violation." % DELTA)
    L.append("")
    L.append("| metric | SL-16 / Zenit-2 class | SL-8 / Kosmos-3M class |")
    L.append("|---|---|---|")
    L.append("| success rate | %s | %s |" % (rate_cell(succA), rate_cell(succB)))
    L.append("| nonproductive-termination rate (= 1 %s success) | %s | %s |"
              % (MINUS, rate_cell(nptA), rate_cell(nptB)))
    L.append("| gate-abort-run rate (abort-path exposure) | %s | %s |"
              % (rate_cell(gaA), rate_cell(gaB)))
    L.append("| keep-out-violation rate | %s | %s |" % (ko_cell(koA), ko_cell(koB)))
    L.append("| %sv used p05/p50/p95 [m/s] | %s | %s |"
              % (DELTA, pctl(dvA, f0), pctl(dvB, f0)))
    L.append("| kits used p05/p50/p95 | %s | %s |" % (pctl(kuA, f0), pctl(kuB, f0)))
    L.append("| removals/mission p05/p50/p95 | %s | %s |" % (pctl(rmA, f0), pctl(rmB, f0)))
    L.append("| sync arrival p05/p50/p95 [s] | %s | %s |" % (pctl(syA, f2), pctl(syB, f2)))
    L.append("| failure counts (runs) | dv_exhausted %d, kit_exhausted %d, "
              "keep_out %d, completed %d | dv_exhausted %d, kit_exhausted "
              "%d, keep_out %d, completed %d |"
              % (dvexA, kitexA, koCountA, compA, dvexB, kitexB, koCountB, compB))
    L.append("| per-target events | gate_abort %d, sync_timeout %d | "
              "gate_abort %d, sync_timeout %d |"
              % (gaCountA, stCountA, gaCountB, stCountB))
    L.append("")
    L.append("Two abort-related rates are reported and are deliberately distinct:")
    L.append("**`gate-abort-run rate`** is the abort-path exposure (fraction of runs with %s 1"
              % GE)
    L.append("closing-speed gate abort %s what the spec calls the \"abort rate\"), while" % EM)
    L.append("**`nonproductive-termination rate`** is 1 %s success. Under the current *flat"
              % MINUS)
    L.append("PLACEHOLDER* %sv cost the two coincide numerically %s every aborting mission needs"
              % (DELTA, EM))
    L.append("an extra target-slot to still install its kits and so exhausts the 140 m/s")
    L.append("budget %s but they are separate concepts and will diverge once the cost model"
              % EM)
    L.append("gains structure. The honest campaign finding: the servicer is **%sv-limited about"
              % DELTA)
    L.append("%d%% of the time** (`dv_exhausted` = %d/%d) and installs its full kit"
              % (dv_limited_pct, dvexA, n_runs))
    L.append("complement the rest; the attitude sync **never** times out across the sampled")
    L.append("tumble/attitude/actuator dispersions, and keep-out violations are **%d of %d**"
              % (koCountA, n_runs))
    L.append("per catalog [%s: linear CW, dispersion set %s, Wilson 95%% upper bound"
              % (l0tag, dstag))
    L.append("%s] %s the WP11 clearing-abort law accepts an abort only when the analytic"
              % (f4(koA["wilson_high"]), EM))
    L.append("post-burn ellipse clears keep-out plus a design margin (mechanism forensics:")
    L.append("`generated/wp10_violation_forensics.md`; audit:")
    L.append("`generated/wp11_abort_audit.md`). `completed` (all 6 targets) is 0 by")
    L.append("construction %s 4 kits cannot service 6 targets. The `%sv used` / `kits used`"
              % (EM, DELTA))
    L.append("percentiles matching across the two presets is expected (a flat PLACEHOLDER cost")
    L.append("takes quantized, catalog-independent values %s not a copy-paste bug). Full per-run"
              % EM)
    L.append("records and the column schema are in [generated/](../generated/).")
    return "\n".join(L)


def build_wp6_full(data):
    """docs/cost_model.md DOCS-WP6-FULL: the full WP6 amortization/FoM/tornado
    table (the original build_wp6_block(), moved here; the schema label now
    reads the CSV's own schema_version instead of the template's former
    hardcoded, stale "1.0" -- the WP15 content-mapping pass found and fixed
    that drift, per R16's whole point)."""
    d5, d6 = data.d5, data.d6
    A, B = d5.catalogs[0], d5.catalogs[1]
    Ashort, Bshort = CATALOG_SHORT[A], CATALOG_SHORT[B]
    n_runs = d5.n_runs

    ns = sorted({int(r["n_kits"]) for r in d6.rows
                 if r["record_type"] == "amortization" and r["metric"] == "cost_per_removal"})
    cprA = {n: float(d6.q(A, "amortization", "cost_per_removal", n)["p50"]) for n in ns}
    cprB = {n: float(d6.q(B, "amortization", "cost_per_removal", n)["p50"]) for n in ns}
    nmin = min(ns, key=lambda n: cprA[n])
    nmax = max(ns)
    display_ns = sorted(set(n for n in (1, 2, 3, nmin, nmin + 1, nmax) if n in ns))

    baseline_n = int(d6.first_row(A, "cost_component")["n_kits"])

    ratio_min = float(d6.q(A, "amortization", "cost_per_removal_ratio_to_n1", nmin)["p50"])
    pct_min = ratio_min * 100.0
    saving_x = 1.0 / ratio_min
    catalog_gap_pct = abs(cprA[nmin] - cprB[nmin]) / cprA[nmin] * 100.0

    fomA_sp = d6.q(A, "fom", "fom", weighting="spatial_density")
    fomA_cr = d6.q(A, "fom", "fom", weighting="criticality")
    fomB_sp = d6.q(B, "fom", "fom", weighting="spatial_density")
    fomB_cr = d6.q(B, "fom", "fom", weighting="criticality")
    wA_sp = d6.q(A, "fom", "band_weight", weighting="spatial_density")
    wA_cr = d6.q(A, "fom", "band_weight", weighting="criticality")
    wB_sp = d6.q(B, "fom", "band_weight", weighting="spatial_density")
    wB_cr = d6.q(B, "fom", "band_weight", weighting="criticality")

    tornado = [r for r in d6.rows if r["catalog"] == A
               and r["record_type"] == "tornado" and r["metric"] == "cost_per_removal_swing"]
    if len(tornado) != 5:
        raise RuntimeError(
            "expected exactly 5 tornado cost_per_removal_swing rows for %r, "
            "got %d -- the DOCS-WP6-FULL template is hardcoded for a "
            "5-parameter, 2-line wrap and needs a human rewrite if the "
            "parameter count changes" % (A, len(tornado)))
    items = [(r["param"], float(r["p50"])) for r in tornado]

    L = []
    L.append("The full sweep (%d runs %s 2 catalogs %s N = %d..%d kits) runs in a few tens of"
              % (n_runs, TIMES, TIMES, ns[0], nmax))
    L.append("seconds on a CI runner (see the Actions log of the current run for the actual")
    L.append("figure), and the baseline (N = %d) removals reproduce the committed WP5 CSV"
              % baseline_n)
    L.append("exactly (`MATCH (schema %s consumed)`)." % d6.schema_version)
    L.append("")
    L.append("**Amortization curve %s cost/removal vs kits carried N (%s class):**"
              % (EM, Ashort))
    L.append("")
    L.append("| N | cost/removal p50 [CU] | ratio to N=1 | removals p50 |")
    L.append("|---:|---:|---:|---:|")
    for n in display_ns:
        cpr = float(d6.q(A, "amortization", "cost_per_removal", n)["p50"])
        ratio = float(d6.q(A, "amortization", "cost_per_removal_ratio_to_n1", n)["p50"])
        rem = float(d6.q(A, "amortization", "removals", n)["p50"])
        if n == nmin:
            L.append("| **%d** | **%s** | **%s** | %s |" % (n, f2(cpr), f3(ratio), f0(rem)))
        else:
            L.append("| %d | %s | %s | %s |" % (n, f2(cpr), f3(ratio), f0(rem)))
    L.append("")
    L.append("Batch amortization drives cost/removal down to **%s %% of the single-target"
              % f1(pct_min))
    L.append("(N = %d) baseline** %s a **%s%s per-removal saving** %s bottoming at N = %d where the"
              % (ns[0], EM, f1(saving_x), TIMES, EM, nmin))
    L.append("**%sv budget (not the kit count)** caps removals at %d; carrying more kits then"
              % (DELTA, nmin))
    L.append("adds cost without removals and the curve turns back up. That shape *is* the")
    L.append("quantitative installer/batch argument (%s class is within ~%s %%)."
              % (Bshort, f1(catalog_gap_pct)))
    L.append("")
    L.append("**FoM = %s m%s%sw(h%s)/C_campaign at N = %d (p50, kg/CU):**"
              % (SIGMA, SUBI, DOT, SUBI, baseline_n))
    L.append("")
    L.append("| catalog | spatial-density | criticality |")
    L.append("|---|---:|---:|")
    L.append("| %s | %s (w = %s) | %s (w = %s) |"
              % (CATALOG_FOM_LABEL[A], f2(fomA_sp["p50"]), f2(wA_sp["estimate"]),
                 f2(fomA_cr["p50"]), f2(wA_cr["estimate"])))
    L.append("| %s | %s (w = %s) | %s (w = %s) |"
              % (CATALOG_FOM_LABEL[B], f2(fomB_sp["p50"]), f2(wB_sp["estimate"]),
                 f2(fomB_cr["p50"]), f2(wB_cr["estimate"])))
    L.append("")
    L.append("The ~9 t class outranks the ~1.4 t class under **both** weightings (FoM is")
    L.append("mass-dominated), but the **band weight flips** %s spatial density values the"
              % EM)
    L.append("750 km band more, the criticality index values the 840 km band more %s a genuine"
              % EM)
    L.append("metric-choice disagreement tracked as **open trade T5**. **Tornado** (%s,"
              % Ashort)
    L.append("%s30 %%, ranked by cost/removal swing): `%s` (%s CU) %s `%s` (%s) >"
              % (PM, items[0][0], f1(items[0][1]), GG, items[1][0], f1(items[1][1])))
    L.append("`%s` (%s) > `%s` (%s) > `%s` (%s). All"
              % (items[2][0], f1(items[2][1]), items[3][0], f1(items[3][1]),
                 items[4][0], f1(items[4][1])))
    L.append("values are relative CU; **no absolute-dollar figure is claimed**.")
    return "\n".join(L)


def build_wp12_ladder(data):
    """docs/safety.md DOCS-WP12-LADDER: abort-event re-verification table +
    forensic-14/margin-decay summary from generated/wp12_ladder.csv."""
    ladder = data.ladder
    catalogs = [CATALOG_A, CATALOG_B]
    levels = ["L0", "L1", "L2"]

    n_events = {c: int(round(float(ladder.ar("L0", c, "n_events")["value"])))
                for c in catalogs}
    seed = data.d5.master_seed_hex

    L = []
    L.append("Abort-event re-verification (%d catalog-A + %d catalog-B closing-speed-gate"
              % (n_events[CATALOG_A], n_events[CATALOG_B]))
    L.append("events from the committed campaign; master seed `%s`, same" % seed)
    L.append("draws as `wp5_campaign_runs.csv` for `ds-v1`):")
    L.append("")
    L.append("| level | dispersion set | catalog | n_events | n_violations | "
              "violation_rate | wilson 95% CI | clearance floor (m) | p05 (m) | p50 (m) |")
    L.append("|---|---|---|---:|---:|---:|---|---:|---:|---:|")
    floor = {}
    for catalog in catalogs:
        for level in levels:
            nev = ladder.ar(level, catalog, "n_events")
            nvi = ladder.ar(level, catalog, "n_violations")
            vr = ladder.ar(level, catalog, "violation_rate")
            wlo = ladder.ar(level, catalog, "wilson_low")
            whi = ladder.ar(level, catalog, "wilson_high")
            cf = ladder.ar(level, catalog, "clearance_floor_m")
            cp05 = ladder.ar(level, catalog, "clearance_p05_m")
            cp50 = ladder.ar(level, catalog, "clearance_p50_m")
            floor[(catalog, level)] = float(cf["value"])
            L.append("| %s | %s | %s | %s | %s | %s | [%s, %s] | %s | %s | %s |"
                      % (level, ladder.dispersion_set(level, catalog), catalog,
                         f0(nev["value"]), f0(nvi["value"]), f6(vr["value"]),
                         f6(wlo["value"]), f6(whi["value"]), f4(cf["value"]),
                         f4(cp05["value"]), f4(cp50["value"])))
    L.append("")

    items = ladder.forensic_items()
    n_cases = len(items)
    if not ladder.forensic_all_clear():
        raise RuntimeError(
            "DOCS-WP12-LADDER template is hardcoded for an all-clear "
            "forensic-14 result and needs a human rewrite -- a violation "
            "was found in generated/wp12_ladder.csv")

    erosion = {c: floor[(c, "L0")] - floor[(c, "L2")] for c in catalogs}
    tighter = min(catalogs, key=lambda c: floor[(c, "L2")])
    erosion_lo, erosion_hi = sorted(erosion.values())

    L.append("Forensic-14 per-level clearance: all %d of %d pinned cases clear keep-out at"
              % (n_cases, n_cases))
    L.append("**L0, L1 and L2** %s the anticipated CW-safe-but-higher-level-unsafe case does"
              % EM)
    L.append("**not** materialize for this dispersion set: the WP11 clearance margin")
    L.append("absorbs the measured ~%s-%s m of J2+drag coast erosion (clearance floor"
              % (f1(erosion_lo), f1(erosion_hi)))
    L.append("%s m at L0 %s %s m at L2, %s class, the tighter of the two). A"
              % (f1(floor[(tighter, "L0")]), ARROW, f1(floor[(tighter, "L2")]),
                 CATALOG_SHORT[tighter]))
    L.append("negative-negative result, reported with the same discipline as a positive")
    L.append("one. Full 14-row per-case table: `generated/wp12_ladder.md`.")
    L.append("")

    worst_erosion = max(ladder.margin_decay_erosion("standoff_400m/orbit=5").values())
    L.append("Safety-ellipse margin decay under J2 (the F2 caveat, promised qualitatively")
    L.append("since WP1, now measured): worst 5-orbit min-range erosion **%s m** on the"
              % f2(worst_erosion))
    L.append("400 m standoff ellipse [L1: two-body+J2]. Full per-orbit table for both")
    L.append("geometries and both catalogs: `generated/wp12_ladder.md`.")
    return "\n".join(L)


def build_gnc_wp2(data):
    """docs/gnc.md DOCS-GNC-WP2: the WP2 sync-time/dwell-error pins."""
    ref = data.ref
    L = []
    L.append("| metric | value | tolerance |")
    L.append("|---|---:|---:|")
    L.append("| sync time | **%s s** | criteria first held, then dwelled 30 s |"
              % f2(ref.v("wp2_sync_time_s")))
    L.append("| max \\|w_rel\\| after dwell | **%s deg/s** | tol 0.1 deg/s |"
              % f6(ref.v("wp2_max_rate_after_dwell_deg_s")))
    L.append("| max attitude error after dwell | **%s%s** | tol 2.0%s |"
              % (f6(ref.v("wp2_max_att_after_dwell_deg")), DEG, DEG))
    return "\n".join(L)


def build_wp3_decay(data):
    """docs/target_selection.md DOCS-WP3-DECAY: the sail-area x solar-activity
    decay sweep for catalogs A and B, plus the 25-yr guideline areas."""
    wp3 = data.wp3
    L = []
    for catalog, comment in ((CATALOG_A, ("impractical at the upper end.",)),
                             (CATALOG_B, ("tens of", "square meters, a practical kit."))):
        meta = wp3.meta(catalog)
        L.append("**Catalog %s %s %s, %s%s kg, %s km:**"
                  % (CATALOG_LETTER[catalog], EM, CATALOG_DECAY_LABEL[catalog],
                     APPROX, fcomma0(meta["mass_kg"]), f0(meta["altitude_km"])))
        L.append("")
        L.append("| sail area [m%s] | decay time, solar max [yr] | decay time, solar min [yr] |"
                  % SUP2)
        L.append("|---:|---:|---:|")
        for r in wp3.decay_rows(catalog):
            L.append("| %s | %s | %s |" % (f0(r["sail_area_m2"]), f1(r["value_solar_max"]),
                                            f1(r["value_solar_min"])))
        L.append("")
        area = wp3.area25(catalog)
        if len(comment) == 1:
            L.append("Sail area for the 25-year guideline (solar max..min): **%s..%s m%s** %s"
                      % (f0(area["value_solar_max"]), f0(area["value_solar_min"]), SUP2, EM))
            L.append(comment[0])
        else:
            L.append("Sail area for the 25-year guideline (solar max..min): **%s..%s m%s** %s %s"
                      % (f0(area["value_solar_max"]), f0(area["value_solar_min"]), SUP2, EM,
                         comment[0]))
            L.append(comment[1])
        L.append("")
    L.append("25-year figure is the **IADC guideline** (IADC-02-01, Space Debris Mitigation")
    L.append("Guidelines, guideline 5.3.2). Under a US FCC license the current rule (FCC")
    L.append("22-74, adopted 2022) requires **5-year** post-mission disposal for LEO space")
    L.append("stations instead %s the class-A sail-only negative gets strictly harder under"
              % EM)
    L.append("that standard. Both rules are carried separately in the WP8 compliance")
    L.append("rulepacks (see [docs/legal_regulatory.md](legal_regulatory.md)); this trade")
    L.append("keeps the IADC 25-year reference line.")
    return "\n".join(L)


def build_wp13_kit(data):
    """docs/target_selection.md DOCS-WP13-KIT: per-class kit-trade summary +
    mass/deploy-risk/EMF table."""
    wp13 = data.wp13
    catalogs = [CATALOG_A, CATALOG_B, CATALOG_C]

    L = []
    L.append("| catalog | recommended kit | sail area, 25-yr [m%s] (max..min) | "
              "EDT deorbit time [yr] | EDT %s(i) band |" % (SUP2, ETA))
    L.append("|---|---|---:|---:|---:|")
    for c in catalogs:
        sail = wp13.row(c, "sail_area_25yr")
        edt_years = wp13.row(c, "edt_years")
        edt_eta = wp13.row(c, "edt_eta")
        rec = wp13.row(c, "recommended_kit")
        L.append("| %s %s %s | %s | %s..%s | %s..%s | %s..%s |"
                  % (CATALOG_LETTER[c], EM, c, rec["kit_option"],
                     f0(sail["value_lo"]), f0(sail["value_hi"]),
                     f1(edt_years["value_lo"]), f1(edt_years["value_hi"]),
                     f3(edt_eta["value_lo"]), f3(edt_eta["value_hi"])))
    L.append("")
    L.append("Kit mass, deploy risk, and diagnostic EMF/power (same source CSV):")
    L.append("")
    L.append("| catalog | sail kit mass [kg] | EDT kit mass [kg] (PLACEHOLDER) | "
              "EDT deploy-failure risk (PLACEHOLDER) | EDT open-circuit EMF [V] |")
    L.append("|---|---:|---:|---:|---:|")
    for c in catalogs:
        kit_sail = wp13.row(c, "kit_mass", "sail")
        kit_edt = wp13.row(c, "kit_mass", "edt")
        deploy = wp13.row(c, "deploy_risk", "edt")
        emf = wp13.row(c, "edt_emf_power", "edt")
        L.append("| %s | %s | %s | %s | %s |"
                  % (CATALOG_LETTER[c], f1(kit_sail["value"]), f1(kit_edt["value"]),
                     f2(deploy["value"]), f1(emf["value"])))
    L.append("")
    L.append("Deploy risk is quoted for the record and is **not** folded into the")
    L.append("EDT-years band above.")
    return "\n".join(L)


def build_wp13_classc(data):
    """docs/target_selection.md DOCS-WP13-CLASSC: controlled-deorbit delta-v
    comparison for the Class-C candidates."""
    classc = data.classc
    L = []
    L.append("Controlled-deorbit %sv (COMPUTED): a single impulsive perigee-lowering burn"
              % DELTA)
    L.append("from the catalog's circular orbit (radius r_a) to a target perigee r_p =")
    L.append("6371+40 km (mean-Earth-radius convention), dv = v_c%s(1 %s %s(2%sr_p/(r_a+r_p))),"
              % (DOT, MINUS, SQRT, DOT))
    L.append("v_c = %s(%s/r_a). This is the minimum single-burn %sv to commit the stage to"
              % (SQRT, MU, DELTA))
    L.append("reentry; it excludes targeting/footprint-control burns and any margin.")
    L.append("")
    L.append("| candidate | v_c [m/s] | %sv [m/s] |" % DELTA)
    L.append("|---|---:|---:|")
    for candidate in CANDIDATE_ORDER:
        vc = classc.row(candidate, "v_circular_initial")
        dv = classc.row(candidate, "controlled_deorbit_dv")
        L.append("| %s | %s | **%s** |"
                  % (CANDIDATE_LABEL[candidate], f1(vc["value"]), f1(dv["value"])))
    return "\n".join(L)


def build_5p_key(data):
    """docs/technical_summary_5p.md DOCS-5P-KEY: the pinned key-numbers table,
    the same headline figures as README's WP5-NUMBERS/WP6-NUMBERS digest plus
    a few GNC pins, in plain (non-bulleted) prose-table form."""
    d5, d6, ref = data.d5, data.d6, data.ref
    A, B = d5.catalogs[0], d5.catalogs[1]

    succA, succB = d5.m(A, "success_rate"), d5.m(B, "success_rate")
    koA = d5.m(A, "keep_out_violation_rate")
    koCountA = count(d5.m(A, "keep_out_violation"))

    nmin, cpr_min, ratio_min = amortization_min(d6, A)
    fomA_sp = d6.q(A, "fom", "fom", weighting="spatial_density")
    fomA_cr = d6.q(A, "fom", "fom", weighting="criticality")
    anchor_low = float(d6.anchor("low")["p50"])
    anchor_mid = float(d6.anchor("mid")["p50"])
    anchor_high = float(d6.anchor("high")["p50"])
    cost_low = cpr_min * anchor_low
    cost_mid = cpr_min * anchor_mid
    cost_high = cpr_min * anchor_high

    L = []
    L.append("| metric | value | fidelity / notes | source |")
    L.append("|---|---|---|---|")
    L.append("| Campaign success (productive end), SL-16 / SL-8 | %s / %s (Wilson 95%%) | "
              "L0, ds-v1 | `generated/wp5_campaign_summary.csv` |"
              % (rate_plain(succA), rate_plain(succB)))
    L.append("| Keep-out violations, both classes | %s / %s, Wilson UB %s %s | L0, ds-v1; "
              "zero re-verified over every re-verified abort event at L1/L2 | "
              "`generated/wp5_campaign_summary.csv`, `generated/wp12_ladder.csv` |"
              % (f0(koCountA), f0(d5.n_runs), LE, f4(koA["wilson_high"])))
    L.append("| Amortization minimum (N = %d) | %s CU/removal = %s%s the N = 1 baseline | "
              "%sv-limited, measured | `generated/wp6_cost_summary.csv` |"
              % (nmin, f2(cpr_min), f3(ratio_min), TIMES, DELTA))
    L.append("| FoM p50, SL-16 class (spatial / criticality) | %s / %s kg/CU | two "
              "independent weightings, disagreement tracked (T5) | "
              "`generated/wp6_cost_summary.csv` |"
              % (f2(fomA_sp["p50"]), f2(fomA_cr["p50"])))
    L.append("| Cost/removal p50, SL-16 class (low / mid / high) | %s / %s / %s MUSD | "
              "external anchor, cited range only (D10) | `generated/wp6_cost_summary.csv` "
              "(WP14 rows) |"
              % (f2(cost_low), f2(cost_mid), f2(cost_high)))
    L.append("| Attitude sync (truth-driven pin) | %s s | L0 | `reference_metrics.csv` |"
              % f2(ref.v("wp2_sync_time_s")))
    L.append("| Attitude sync, estimate-driven under sensor noise | %s s | L0, "
              "estimator-in-the-loop | `reference_metrics.csv` |"
              % f2(ref.v("wp4_sync_time_s")))
    L.append("| Guided-approach demo: contact speed / total %sv | %s m/s (gate %s m/s) / "
              "%s m/s | L0, truth-fed guidance | evidence pack %s3, `reference_metrics.csv` |"
              % (DELTA, f2(ref.v("wp11_guided_contact_speed_m_s")),
                 f2(ref.v("wp3_contact_speed_m_s")), f2(ref.v("wp11_guided_dv_total_m_s")),
                 SEC))
    L.append("")
    L.append("*This block is generated by `tools/docs/fill_docs_numbers.py` from the same")
    L.append("committed CSVs as README's WP5-NUMBERS/WP6-NUMBERS blocks and")
    L.append("`reference_metrics.csv`; run with `--check` to verify, in-place to refresh.*")
    return "\n".join(L)


# ---------------------------------------------------------------------------
# Marker registry: (doc-relative-path, start marker, end marker, builder).
# ---------------------------------------------------------------------------

REGISTRY = [
    ("docs/safety.md",
     "<!-- DOCS-WP12-LADDER-START (source: generated/wp12_ladder.{csv,md}; "
     "copied verbatim from the committed table) -->",
     "<!-- DOCS-WP12-LADDER-END -->",
     build_wp12_ladder),
    ("docs/safety.md",
     "<!-- DOCS-WP5-FULL-START (source: generated/wp5_campaign_summary.csv, "
     "schema 1.1; verbatim from tools/docs/fill_docs_numbers.py "
     "build_wp5_full(), CI regenerates this from adsc_campaign, seed "
     "0x5AD5C0DECAFE2026) -->",
     "<!-- DOCS-WP5-FULL-END -->",
     build_wp5_full),
    ("docs/cost_model.md",
     "<!-- DOCS-WP6-FULL-START (source: generated/wp6_cost_summary.csv, "
     "schema 1.1; verbatim from tools/docs/fill_docs_numbers.py "
     "build_wp6_full(), CI regenerates this from adsc_cost) -->",
     "<!-- DOCS-WP6-FULL-END -->",
     build_wp6_full),
    ("docs/gnc.md",
     "<!-- DOCS-GNC-WP2-START (copied from generated/reference_metrics.csv: "
     "wp2_sync_time_s, wp2_max_rate_after_dwell_deg_s, "
     "wp2_max_att_after_dwell_deg) -->",
     "<!-- DOCS-GNC-WP2-END -->",
     build_gnc_wp2),
    ("docs/target_selection.md",
     "<!-- DOCS-WP3-DECAY-START (source: generated/wp3_decay_trade.csv, "
     "schema 1.0) -->",
     "<!-- DOCS-WP3-DECAY-END -->",
     build_wp3_decay),
    ("docs/target_selection.md",
     "<!-- DOCS-WP13-KIT-START (source: generated/wp13_kit_trade.csv, "
     "schema 1.0; copied from evidence pack section 4) -->",
     "<!-- DOCS-WP13-KIT-END -->",
     build_wp13_kit),
    ("docs/target_selection.md",
     "<!-- DOCS-WP13-CLASSC-START (source: generated/wp13_classC.{csv,md}, "
     "schema 1.0; copied verbatim) -->",
     "<!-- DOCS-WP13-CLASSC-END -->",
     build_wp13_classc),
    ("docs/technical_summary_5p.md",
     "<!-- DOCS-5P-KEY-START -->",
     "<!-- DOCS-5P-KEY-END -->",
     build_5p_key),
]


def replace_block(text, start_marker, end_marker, new_body):
    start_idx = text.index(start_marker)
    end_idx = text.index(end_marker, start_idx)
    before = text[:start_idx + len(start_marker)]
    after = text[end_idx:]
    return before + "\n" + new_body + "\n" + after


def _first_diff_line(a, b):
    la, lb = a.split("\n"), b.split("\n")
    for i, (x, y) in enumerate(zip(la, lb)):
        if x != y:
            return i + 1, x, y
    if len(la) != len(lb):
        return min(len(la), len(lb)) + 1, "<end>", "<more lines>"
    return None


def main():
    argv = sys.argv[1:]
    check_mode = "--check" in argv
    positional = [a for a in argv if a != "--check"]
    if not positional:
        print("usage: fill_docs_numbers.py <repo_root> [--check]", file=sys.stderr)
        return 2
    root = os.path.abspath(positional[0])

    data = Data(root)

    by_file = []
    for relpath, start, end, builder in REGISTRY:
        if not by_file or by_file[-1][0] != relpath:
            by_file.append((relpath, []))
        by_file[-1][1].append((start, end, builder))

    any_stale = False
    for relpath, blocks in by_file:
        path = os.path.join(root, *relpath.split("/"))
        with open(path, encoding="utf-8") as f:
            original = f.read()
        new_text = original
        for start, end, builder in blocks:
            new_text = replace_block(new_text, start, end, builder(data))

        if check_mode:
            if new_text == original:
                print("[docs_numbers] %s marker blocks match the committed CSVs" % relpath)
            else:
                any_stale = True
                print("[docs_numbers] %s marker blocks are STALE vs the committed CSVs"
                      % relpath, file=sys.stderr)
                diff = _first_diff_line(original, new_text)
                if diff:
                    lineno, old, new = diff
                    print("  first difference at line %d:" % lineno, file=sys.stderr)
                    print("    committed : %r" % old, file=sys.stderr)
                    print("    from CSVs : %r" % new, file=sys.stderr)
        else:
            with open(path, "w", encoding="utf-8", newline="\n") as f:
                f.write(new_text)
            if new_text == original:
                print("[docs_numbers] %s marker blocks already up to date" % relpath)
            else:
                print("[docs_numbers] %s marker blocks rewritten" % relpath)

    if check_mode:
        return 1 if any_stale else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
