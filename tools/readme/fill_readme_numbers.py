#!/usr/bin/env python3
"""README marker-block filler (WP14 review item 0.4b / C4).

  fill_readme_numbers.py <repo_root> [--check]

Regenerates ONLY the text strictly between the two committed marker-comment
pairs in README.md --

  <!-- WP5-NUMBERS-START (filled from CI adsc_campaign, seed 0x5AD5C0DECAFE2026) -->
  ...
  <!-- WP5-NUMBERS-END -->

  <!-- WP6-NUMBERS-START (filled from CI adsc_cost) -->
  ...
  <!-- WP6-NUMBERS-END -->

-- from the two committed CSVs (generated/wp5_campaign_summary.csv,
generated/wp6_cost_summary.csv). Every NUMBER quoted inside a block (table
cell or inline prose) is read mechanically from those CSVs with a fixed
rounding; the surrounding prose/table skeleton is literal template text (it
does not reflow). This closes the review's C4 finding: before this tool,
README.md was hand-updated and could silently drift from the committed CSVs
it claims to summarize.

Default mode rewrites README.md in place. `--check` instead exits non-zero
if the regenerated blocks differ from the committed file (for CI: a stale
README fails the build instead of silently drifting -- see ci.yml).

Python 3 standard library only (spec v4.2 R9). This tool's own SOURCE stays
pure ASCII (spec R11): the README's existing Unicode punctuation/math glyphs
(em dash, Delta, times, Sigma, subscript i, ...) are produced via explicit
\\uXXXX escapes in ordinary string literals, never as literal source bytes.
No wall-clock timestamps; output is UTF-8 with "\\n" line endings so the CI
byte-reproducibility gate can compare it (matching every other generator in
this repository).
"""
import csv
import os
import re
import sys

# ---- Unicode glyphs used inside the README blocks, named so the source
# ---- file itself stays ASCII-only (spec R11) while the generated text
# ---- is not -- each is an explicit \uXXXX escape, never a literal byte.
EM = "\u2014"     # em dash
MINUS = "\u2212"  # minus sign (distinct from ASCII hyphen)
DELTA = "\u0394"  # Greek capital delta (used for "Dv")
GE = "\u2265"     # >=
TIMES = "\u00d7"  # multiplication sign
SUBI = "\u1d62"   # Latin subscript small letter i
SIGMA = "\u03a3"  # Greek capital sigma
DOT = "\u00b7"    # middle dot
PM = "\u00b1"     # plus-minus sign
GG = "\u226b"     # much-greater-than

WP5_START = ("<!-- WP5-NUMBERS-START (filled from CI adsc_campaign, seed "
             "0x5AD5C0DECAFE2026) -->")
WP5_END = "<!-- WP5-NUMBERS-END -->"
WP6_START = "<!-- WP6-NUMBERS-START (filled from CI adsc_cost) -->"
WP6_END = "<!-- WP6-NUMBERS-END -->"

# Fixed catalog display labels (literal template text, D2/D13: catalog
# identity/altitude is out of WP14's scope -- see the WP14 integration map's
# trap list on CATALOG_ALT_KM-style hardcoded mirrors).
CATALOG_SHORT = {
    "SL-16 / Zenit-2 second stage": "SL-16",
    "SL-8 / Kosmos-3M second stage": "SL-8",
}
CATALOG_FOM_LABEL = {
    "SL-16 / Zenit-2 second stage": "SL-16 (~9 t, 840 km)",
    "SL-8 / Kosmos-3M second stage": "SL-8 (~1.4 t, 750 km)",
}


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


def load_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


class Wp5Data:
    """generated/wp5_campaign_summary.csv (schema 1.1, WP5's own additive
    dispersion_set_id/worst_abort_clearance_m bump -- unrelated to WP14 and
    unread here)."""

    def __init__(self, root):
        self.rows = load_csv(os.path.join(root, "generated",
                                           "wp5_campaign_summary.csv"))
        self.catalogs = []
        for r in self.rows:
            if r["catalog"] not in self.catalogs:
                self.catalogs.append(r["catalog"])
        self.n_runs = int(float(self.rows[0]["n_runs"]))

    def m(self, catalog, metric):
        for r in self.rows:
            if r["catalog"] == catalog and r["metric"] == metric:
                return r
        raise KeyError((catalog, metric))


class Wp6Data:
    """generated/wp6_cost_summary.csv (schema 1.1, WP14)."""

    def __init__(self, root):
        self.rows = load_csv(os.path.join(root, "generated",
                                           "wp6_cost_summary.csv"))

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

    def rows_of(self, catalog, record_type, metric):
        return [r for r in self.rows
                if r["catalog"] == catalog and r["record_type"] == record_type
                and r["metric"] == metric]


def rate_cell(row):
    """'**0.556** [0.512, 0.599]' from a WP5 rate row (estimate/wilson_low/
    wilson_high), 3dp."""
    return "**%s** [%s, %s]" % (f3(row["estimate"]), f3(row["wilson_low"]),
                                 f3(row["wilson_high"]))


def ko_cell(row):
    """rate_cell() plus the '[L0, ds-v1]' fidelity/dispersion tag, both
    parsed out of the row's own notes field (never hand-typed) so a future
    dispersion-set or fidelity-level change is reflected automatically."""
    m = re.search(r"level tag (\S+), dispersion set (\S+)", row["notes"])
    return "%s [%s, %s]" % (rate_cell(row), m.group(1), m.group(2))


def pctl(row, fmt):
    return "%s / %s / %s" % (fmt(row["p05"]), fmt(row["p50"]), fmt(row["p95"]))


def count(row):
    return int(round(float(row["estimate"])))


def build_wp5_block(d5):
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
    rmA, rmB = (d5.m(A, "removals_per_mission"),
                d5.m(B, "removals_per_mission"))
    syA, syB = d5.m(A, "sync_arrival_time_s"), d5.m(B, "sync_arrival_time_s")

    dvexA, dvexB = count(d5.m(A, "dv_exhausted")), count(d5.m(B, "dv_exhausted"))
    kitexA, kitexB = (count(d5.m(A, "kit_exhausted")),
                      count(d5.m(B, "kit_exhausted")))
    koCountA, koCountB = (count(d5.m(A, "keep_out_violation")),
                          count(d5.m(B, "keep_out_violation")))
    compA, compB = count(d5.m(A, "completed")), count(d5.m(B, "completed"))
    gaCountA, gaCountB = (count(d5.m(A, "gate_abort")),
                          count(d5.m(B, "gate_abort")))
    stCountA, stCountB = (count(d5.m(A, "sync_timeout")),
                          count(d5.m(B, "sync_timeout")))

    m = re.search(r"level tag (\S+), dispersion set (\S+)", koA["notes"])
    l0tag, dstag = m.group(1), m.group(2)
    dv_limited_pct = int(round(100.0 * dvexA / n_runs))

    L = []
    L.append("The full N = %d campaign (both catalog presets) runs in a few "
              "tens of seconds" % n_runs)
    L.append("on a CI runner (see the Actions log of the current run for the "
              "actual figure).")
    L.append("`success` here means a **productive end** %s the mission "
              "installed its" % EM)
    L.append("full 4-kit complement (`kit_exhausted`) or cleared all 6 "
              "targets (`completed`) %s" % EM)
    L.append("not one cut short by %sv exhaustion or a keep-out violation."
              % DELTA)
    L.append("")
    L.append("| metric | SL-16 / Zenit-2 class | SL-8 / Kosmos-3M class |")
    L.append("|---|---|---|")
    L.append("| success rate | %s | %s |" % (rate_cell(succA), rate_cell(succB)))
    L.append("| nonproductive-termination rate (= 1 %s success) | %s | %s |"
              % (MINUS, rate_cell(nptA), rate_cell(nptB)))
    L.append("| gate-abort-run rate (abort-path exposure) | %s | %s |"
              % (rate_cell(gaA), rate_cell(gaB)))
    L.append("| keep-out-violation rate | %s | %s |"
              % (ko_cell(koA), ko_cell(koB)))
    L.append("| %sv used p05/p50/p95 [m/s] | %s | %s |"
              % (DELTA, pctl(dvA, f0), pctl(dvB, f0)))
    L.append("| kits used p05/p50/p95 | %s | %s |"
              % (pctl(kuA, f0), pctl(kuB, f0)))
    L.append("| removals/mission p05/p50/p95 | %s | %s |"
              % (pctl(rmA, f0), pctl(rmB, f0)))
    L.append("| sync arrival p05/p50/p95 [s] | %s | %s |"
              % (pctl(syA, f2), pctl(syB, f2)))
    L.append("| failure counts (runs) | dv_exhausted %d, kit_exhausted %d, "
              "keep_out %d, completed %d | dv_exhausted %d, kit_exhausted "
              "%d, keep_out %d, completed %d |"
              % (dvexA, kitexA, koCountA, compA, dvexB, kitexB, koCountB, compB))
    L.append("| per-target events | gate_abort %d, sync_timeout %d | "
              "gate_abort %d, sync_timeout %d |"
              % (gaCountA, stCountA, gaCountB, stCountB))
    L.append("")
    L.append("Two abort-related rates are reported and are deliberately "
              "distinct:")
    L.append("**`gate-abort-run rate`** is the abort-path exposure (fraction "
              "of runs with %s 1" % GE)
    L.append("closing-speed gate abort %s what the spec calls the \"abort "
              "rate\"), while" % EM)
    L.append("**`nonproductive-termination rate`** is 1 %s success. Under "
              "the current *flat" % MINUS)
    L.append("PLACEHOLDER* %sv cost the two coincide numerically %s every "
              "aborting mission needs" % (DELTA, EM))
    L.append("an extra target-slot to still install its kits and so "
              "exhausts the 140 m/s")
    L.append("budget %s but they are separate concepts and will diverge "
              "once the cost model" % EM)
    L.append("gains structure. The honest campaign finding: the servicer is "
              "**%sv-limited about" % DELTA)
    L.append("%d%% of the time** (`dv_exhausted` = %d/%d) and installs its "
              "full kit" % (dv_limited_pct, dvexA, n_runs))
    L.append("complement the rest; the attitude sync **never** times out "
              "across the sampled")
    L.append("tumble/attitude/actuator dispersions, and keep-out violations "
              "are **%d of %d**" % (koCountA, n_runs))
    L.append("per catalog [%s: linear CW, dispersion set %s, Wilson 95%% "
              "upper bound" % (l0tag, dstag))
    L.append("%s] %s the WP11 clearing-abort law accepts an abort only when "
              "the analytic" % (f4(koA["wilson_high"]), EM))
    L.append("post-burn ellipse clears keep-out plus a design margin "
              "(mechanism forensics:")
    L.append("`generated/wp10_violation_forensics.md`; audit:")
    L.append("`generated/wp11_abort_audit.md`). `completed` (all 6 targets) "
              "is 0 by")
    L.append("construction %s 4 kits cannot service 6 targets. The `%sv "
              "used` / `kits used`" % (EM, DELTA))
    L.append("percentiles matching across the two presets is expected (a "
              "flat PLACEHOLDER cost")
    L.append("takes quantized, catalog-independent values %s not a "
              "copy-paste bug). Full per-run" % EM)
    L.append("records and the column schema are in "
              "[generated/](generated/).")
    return "\n".join(L)


def build_wp6_block(d5, d6):
    A, B = d5.catalogs[0], d5.catalogs[1]
    Ashort, Bshort = CATALOG_SHORT[A], CATALOG_SHORT[B]
    n_runs = d5.n_runs

    ns = sorted({int(r["n_kits"]) for r in d6.rows
                 if r["record_type"] == "amortization"
                 and r["metric"] == "cost_per_removal"})
    cprA = {n: float(d6.q(A, "amortization", "cost_per_removal", n)["p50"])
            for n in ns}
    cprB = {n: float(d6.q(B, "amortization", "cost_per_removal", n)["p50"])
            for n in ns}
    nmin = min(ns, key=lambda n: cprA[n])
    nmax = max(ns)
    # Curated display subset (first 3 points, the curve minimum, one past
    # it, and the sweep max) -- matches the committed table exactly and
    # keeps it short; the full sweep is in the committed CSV.
    display_ns = sorted(set(n for n in (1, 2, 3, nmin, nmin + 1, nmax)
                             if n in ns))

    baseline_n = int(d6.first_row(A, "cost_component")["n_kits"])

    ratio_min = float(d6.q(A, "amortization", "cost_per_removal_ratio_to_n1",
                           nmin)["p50"])
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
               and r["record_type"] == "tornado"
               and r["metric"] == "cost_per_removal_swing"]
    if len(tornado) != 5:
        raise RuntimeError(
            "expected exactly 5 tornado cost_per_removal_swing rows for %r, "
            "got %d -- the WP6-NUMBERS tornado prose template is hardcoded "
            "for a 5-parameter, 2-line wrap and needs a human rewrite if the "
            "parameter count changes" % (A, len(tornado)))
    items = [(r["param"], float(r["p50"])) for r in tornado]

    L = []
    L.append("The full sweep (%d runs %s 2 catalogs %s N = %d..%d kits) runs "
              "in a few tens of" % (n_runs, TIMES, TIMES, ns[0], nmax))
    L.append("seconds on a CI runner (see the Actions log of the current run "
              "for the actual")
    L.append("figure), and the baseline (N = %d) removals reproduce the "
              "committed WP5 CSV" % baseline_n)
    L.append("exactly (`MATCH (schema 1.0 consumed)`).")
    L.append("")
    L.append("**Amortization curve %s cost/removal vs kits carried N (%s "
              "class):**" % (EM, Ashort))
    L.append("")
    L.append("| N | cost/removal p50 [CU] | ratio to N=1 | removals p50 |")
    L.append("|---:|---:|---:|---:|")
    for n in display_ns:
        cpr = float(d6.q(A, "amortization", "cost_per_removal", n)["p50"])
        ratio = float(d6.q(A, "amortization", "cost_per_removal_ratio_to_n1",
                           n)["p50"])
        rem = float(d6.q(A, "amortization", "removals", n)["p50"])
        if n == nmin:
            L.append("| **%d** | **%s** | **%s** | %s |"
                      % (n, f2(cpr), f3(ratio), f0(rem)))
        else:
            L.append("| %d | %s | %s | %s |" % (n, f2(cpr), f3(ratio), f0(rem)))
    L.append("")
    L.append("Batch amortization drives cost/removal down to **%s %% of the "
              "single-target" % f1(pct_min))
    L.append("(N = %d) baseline** %s a **%s%s per-removal saving** %s "
              "bottoming at N = %d where the"
              % (ns[0], EM, f1(saving_x), TIMES, EM, nmin))
    L.append("**%sv budget (not the kit count)** caps removals at %d; "
              "carrying more kits then" % (DELTA, nmin))
    L.append("adds cost without removals and the curve turns back up. That "
              "shape *is* the")
    L.append("quantitative installer/batch argument (%s class is within "
              "~%s %%)." % (Bshort, f1(catalog_gap_pct)))
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
    L.append("The ~9 t class outranks the ~1.4 t class under **both** "
              "weightings (FoM is")
    L.append("mass-dominated), but the **band weight flips** %s spatial "
              "density values the" % EM)
    L.append("750 km band more, the criticality index values the 840 km "
              "band more %s a genuine" % EM)
    L.append("metric-choice disagreement tracked as **open trade T5**. "
              "**Tornado** (%s," % Ashort)
    L.append("%s30 %%, ranked by cost/removal swing): `%s` (%s CU) %s `%s` "
              "(%s) >"
              % (PM, items[0][0], f1(items[0][1]), GG, items[1][0],
                 f1(items[1][1])))
    L.append("`%s` (%s) > `%s` (%s) > `%s` (%s). All"
              % (items[2][0], f1(items[2][1]), items[3][0], f1(items[3][1]),
                 items[4][0], f1(items[4][1])))
    L.append("values are relative CU; **no absolute-dollar figure is "
              "claimed**.")
    return "\n".join(L)


def replace_block(text, start_marker, end_marker, new_body):
    start_idx = text.index(start_marker)
    end_idx = text.index(end_marker, start_idx)
    before = text[:start_idx + len(start_marker)]
    after = text[end_idx:]
    return before + "\n" + new_body + "\n" + after


def rewrite(text, d5, d6):
    text = replace_block(text, WP5_START, WP5_END, build_wp5_block(d5))
    text = replace_block(text, WP6_START, WP6_END, build_wp6_block(d5, d6))
    return text


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
        print("usage: fill_readme_numbers.py <repo_root> [--check]",
              file=sys.stderr)
        return 2
    root = os.path.abspath(positional[0])
    readme_path = os.path.join(root, "README.md")

    with open(readme_path, encoding="utf-8") as f:
        original = f.read()

    d5 = Wp5Data(root)
    d6 = Wp6Data(root)
    new_text = rewrite(original, d5, d6)

    if check_mode:
        if new_text == original:
            print("[readme_numbers] README.md marker blocks match the "
                  "committed CSVs")
            return 0
        print("[readme_numbers] README.md marker blocks are STALE vs the "
              "committed CSVs", file=sys.stderr)
        diff = _first_diff_line(original, new_text)
        if diff:
            lineno, old, new = diff
            print("  first difference at line %d:" % lineno, file=sys.stderr)
            print("    committed : %r" % old, file=sys.stderr)
            print("    from CSVs : %r" % new, file=sys.stderr)
        return 1

    with open(readme_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(new_text)
    if new_text == original:
        print("[readme_numbers] README.md marker blocks already up to date")
    else:
        print("[readme_numbers] README.md marker blocks rewritten")
    return 0


if __name__ == "__main__":
    sys.exit(main())
