#!/usr/bin/env python3
"""README marker-block filler (WP14 review item 0.4b / C4; WP15 R16 digest).

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

WP15 (proposal package, R16) shrank README's own blocks to a compact
"Key results" digest (a handful of headline rows, ~4-6 lines each) --
the full WP5/WP6 tables these blocks used to carry now live in
docs/safety.md's DOCS-WP5-FULL block and docs/cost_model.md's DOCS-WP6-FULL
block respectively, filled by the sibling tool `tools/docs/fill_docs_numbers.py`
(which reuses this file's original, full-table template functions verbatim,
renamed build_wp5_full()/build_wp6_full()). README's digest and docs/'s full
tables are two independently-generated views of the SAME committed CSVs, so
neither can drift from the other without a --check failure somewhere.

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
# (The full-table glyph set -- GE/SUBI/SIGMA/DOT/PM/GG/MINUS -- moved with
# the full-table builders to tools/docs/fill_docs_numbers.py in WP15; only
# the two glyphs the compact digest still uses remain here.)
DELTA = "\u0394"  # Greek capital delta (used for "Dv")
TIMES = "\u00d7"  # multiplication sign

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

    def anchor(self, scenario):
        """currency_anchor_derived row for cost_scenario in {low, mid, high}
        (WP14; catalog-blank, always at the baseline N)."""
        for r in self.rows:
            if (r["record_type"] == "currency_anchor_derived"
                    and r["metric"] == "anchor_musd_per_cu"
                    and r["cost_scenario"] == scenario):
                return r
        raise KeyError(scenario)


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


def amortization_min(d6, catalog):
    """(nmin, cost_per_removal p50 at nmin, ratio_to_n1 p50 at nmin) -- nmin
    is the kit count that minimizes cost/removal on the amortization sweep
    (currently 4, Dv-budget-limited; computed, never hardcoded, so a future
    model change that shifts the curve's minimum is picked up automatically)."""
    ns = sorted({int(r["n_kits"]) for r in d6.rows
                 if r["record_type"] == "amortization"
                 and r["metric"] == "cost_per_removal"})
    cpr = {n: float(d6.q(catalog, "amortization", "cost_per_removal", n)["p50"])
           for n in ns}
    nmin = min(ns, key=lambda n: cpr[n])
    ratio = float(d6.q(catalog, "amortization", "cost_per_removal_ratio_to_n1",
                       nmin)["p50"])
    return nmin, cpr[nmin], ratio


def build_wp5_digest(d5):
    """README's compact WP5 'Key results' digest (WP15): 4 headline rows out
    of the full WP5 campaign table -- the full table moved to
    docs/safety.md's DOCS-WP5-FULL block (tools/docs/fill_docs_numbers.py's
    build_wp5_full(), the direct descendant of this function's pre-WP15
    form)."""
    A, B = d5.catalogs[0], d5.catalogs[1]
    succA, succB = d5.m(A, "success_rate"), d5.m(B, "success_rate")
    koA, koB = (d5.m(A, "keep_out_violation_rate"),
                d5.m(B, "keep_out_violation_rate"))
    dvA, dvB = d5.m(A, "dv_used_m_per_s"), d5.m(B, "dv_used_m_per_s")
    rmA, rmB = (d5.m(A, "removals_per_mission"),
                d5.m(B, "removals_per_mission"))

    L = []
    L.append("| campaign (N = %d/catalog) | %s class | %s class |"
              % (d5.n_runs, CATALOG_SHORT[A], CATALOG_SHORT[B]))
    L.append("|---|---|---|")
    L.append("| productive-end rate | %s | %s |"
              % (rate_cell(succA), rate_cell(succB)))
    L.append("| keep-out violations | %s | %s |" % (ko_cell(koA), ko_cell(koB)))
    L.append("| %sv used p50 [m/s] | %s | %s |"
              % (DELTA, f0(dvA["p50"]), f0(dvB["p50"])))
    L.append("| removals/mission p50 | %s | %s |"
              % (f0(rmA["p50"]), f0(rmB["p50"])))
    L.append("")
    L.append("Full tables + failure taxonomy: [docs/safety.md](docs/safety.md).")
    return "\n".join(L)


def build_wp6_digest(d5, d6):
    """README's compact WP6 'Key results' digest (WP15): 4 headline rows out
    of the full amortization/FoM/tornado table -- the full table moved to
    docs/cost_model.md's DOCS-WP6-FULL block (tools/docs/fill_docs_numbers.py's
    build_wp6_full(), the direct descendant of this function's pre-WP15
    form). The cost/removal-at-anchor row is computed here from the
    amortization curve's own p50 (not from the CSV's separately-rounded
    cost_per_removal_musd rows) so this digest and the anchor derivation
    agree to the same %.2f -- the two are mathematically identical but were
    rounded through slightly different intermediate values upstream."""
    A = d5.catalogs[0]
    Ashort = CATALOG_SHORT[A]

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
    L.append("| cost/FoM (relative CU; WP14 MUSD ranges) | value |")
    L.append("|---|---|")
    L.append("| amortization minimum (N = %d) | **%s CU/removal = %s%s the "
              "N = 1 baseline** |" % (nmin, f2(cpr_min), f3(ratio_min), TIMES))
    L.append("| FoM p50, %s (spatial / criticality) | %s / %s kg/CU |"
              % (Ashort, f2(fomA_sp["p50"]), f2(fomA_cr["p50"])))
    L.append("| derived anchor (low / mid / high) | %s / %s / %s MUSD/CU |"
              % (f4(anchor_low), f4(anchor_mid), f4(anchor_high)))
    L.append("| cost/removal p50 at the anchor | %s / %s / %s MUSD |"
              % (f2(cost_low), f2(cost_mid), f2(cost_high)))
    L.append("")
    L.append("Full model + sources-or-PLACEHOLDER itemization:")
    L.append("[docs/cost_model.md](docs/cost_model.md).")
    return "\n".join(L)


def replace_block(text, start_marker, end_marker, new_body):
    start_idx = text.index(start_marker)
    end_idx = text.index(end_marker, start_idx)
    before = text[:start_idx + len(start_marker)]
    after = text[end_idx:]
    return before + "\n" + new_body + "\n" + after


def rewrite(text, d5, d6):
    text = replace_block(text, WP5_START, WP5_END, build_wp5_digest(d5))
    text = replace_block(text, WP6_START, WP6_END, build_wp6_digest(d5, d6))
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
