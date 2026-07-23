#!/usr/bin/env python3
"""WP7 evidence-pack claim audit (registered with ctest as `evidence`);
extended in WP15 (R16) to also audit docs/*.md.

  test_evidence.py <repo_root>

Checks (non-zero exit on any failure, R4):
  1. Forbidden overclaims absent: flight-ready / flight-proven / guaranteed /
     TRL 5|6 (regex), and approved / licensed / compliant as legal conclusions.
  2. Required honesty sections present: Limitations, "This is not legal
     advice.", PLACEHOLDER inventory, the TRL 4 element-scoped statement,
     [CITATION NEEDED] discipline (no fabricated citations - the marker is the
     honest alternative and MUST be used where sources are unconfirmed).
  3. Number-transcription audit: the headline numbers quoted in the pack are
     recomputed from the committed CSVs with the same fixed formats and must
     match exactly (anti-transcription-error).
  4. Determinism: two generator runs are byte-identical (SHA-256), and the
     regenerated pack matches the committed evidence/adsc_evidence_pack.md.
  5. Hygiene: ASCII only, no timestamp markers, referenced SVG figures exist.
  6. (WP15/R16) docs/*.md: the same banned-word scan as (1) extends to every
     docs/*.md file; every spec-mandated docs/ file exists; the R15 pin list
     (the WP15 content map's headline pinned numbers) never appears as bare,
     hand-typed prose outside a DOCS-*-START/END generated-include block --
     see scan_docs_r16()'s own docstring for the exact enforcement scope.
"""
import csv
import hashlib
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

fails = []
def check(cond, msg):
    if not cond:
        fails.append(msg)


def sha(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def load_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


# ---------------------------------------------------------------------------
# WP15 (R16): docs/ claims-audit extension.
# ---------------------------------------------------------------------------

# Every docs/*.md file the spec mandates for WP15 (spec:270-274 core list +
# the WP15 instructions' task 4 deliverables). Listed explicitly so a file
# that goes missing (or gets renamed without updating this list) fails loudly
# instead of being silently skipped.
REQUIRED_DOCS = [
    "concept.md", "technical_architecture.md", "gnc.md", "safety.md",
    "target_selection.md", "cost_model.md", "legal_regulatory.md",
    "limitations.md", "roadmap.md", "pitch_agencies.md",
    "one_pager.md", "technical_summary_5p.md", "pitch_deck.md",
    "release_engineering.md",
]

# R15 pin list (spec R15's named legacy GNC pins, plus the WP15 content map's
# other headline pinned numbers -- amortization/FoM/anchor/sail-area figures)
# that must never appear as bare, hand-typed prose in docs/*.md outside a
# DOCS-*-START/END generated-include block (R16). This is an explicit,
# growing literal-string list checked as exact substrings -- it mirrors the
# R14 pattern-list precedent in check 2 below (an enumerated, hand-maintained
# list that must grow with new headline numbers, not a general "any digit"
# rule, which would false-positive on citation years/DOIs/page ranges -- see
# the WP15 content map section 5, trap 4).
R15_DOCS_PINS = [
    "19.15", "424.3", "16.87", "17.07", "16.28", "0.333", "44.80", "157.07",
    "0.285", "156.70", "200.90", "31.35", "21.32", "135..2155", "7..115",
    "0.0076", "0.0130",
]

# Enforcement scope for the pin scan below (documented per the R14
# pattern-list precedent, per WP15 instructions task 3):
#   - A pin occurrence is EXEMPT if the line falls inside a committed
#     DOCS-*-START/END marker block (fill_docs_numbers.py's generated-include
#     mechanism already keeps that text byte-checked against its source CSV).
#   - Otherwise a pin occurrence is EXEMPT if its own markdown paragraph (a
#     run of consecutive non-blank lines -- the natural "logical line" of a
#     wrapped markdown sentence, since these docs hand-wrap at ~78-80 cols),
#     OR the very next paragraph (the "citation trailed after the prose"
#     convention used throughout docs/), contains one of the four explicit
#     reference phrases: "evidence pack", "generated/", "reference_metrics",
#     "regenerate".
#   - A pin occurrence is ALSO exempt if its paragraph contains the spec's
#     fixed anchor phrase "clearance-verified aborts" -- the mandatory,
#     verbatim approach-safety wording formula (spec section 1) pins 0.0076
#     inline BY DESIGN (R15: "any further upgrade of this wording is itself
#     an R15-documented change"); it is a fixed, board-approved SENTENCE, not
#     a place a generated include belongs. This mirrors the pack's own
#     "minimum cost (?!\\s+claim)" carve-out for its own reference sentence.
#   - Anything else is a genuine violation: a number that should be sourced
#     via the include mechanism or an explicit reference, hand-typed instead.
_EXEMPT_PHRASES = ("evidence pack", "generated/", "reference_metrics",
                    "regenerate")
_FORMULA_ANCHOR = "clearance-verified aborts"
_DOCS_MARKER_RE = re.compile(
    r"<!-- DOCS-[A-Za-z0-9-]+-START.*?-->.*?<!-- DOCS-[A-Za-z0-9-]+-END -->",
    re.DOTALL)


def _marker_block_lines(text):
    """Set of 0-based line indices that fall inside any committed
    DOCS-*-START/END marker block (inclusive of the marker-comment lines
    themselves)."""
    lines_in_block = set()
    for m in _DOCS_MARKER_RE.finditer(text):
        start_line = text.count("\n", 0, m.start())
        end_line = text.count("\n", 0, m.end())
        lines_in_block.update(range(start_line, end_line + 1))
    return lines_in_block


def _paragraphs(lines):
    """List of paragraphs, each a list of 0-based line indices, splitting on
    blank lines (a markdown "paragraph" -- a tight bullet list with no blank
    lines between items counts as one paragraph, which is deliberate: list
    items commonly share a single citation for the whole list)."""
    paras = []
    current = []
    for i, line in enumerate(lines):
        if line.strip() == "":
            if current:
                paras.append(current)
                current = []
        else:
            current.append(i)
    if current:
        paras.append(current)
    return paras


# Banned-word families (same concepts as the evidence-pack scan, check 1
# above). Unlike the evidence pack -- which is written to avoid these words
# ENTIRELY, so a bare re.search over the whole file works -- docs/ is more
# discursive prose that explicitly DISCUSSES these very topics ("the only
# path to TRL 5", "guarantees are model-scoped", "'minimum cost' ... is
# banned"): an honest disclaiming/hedging sentence, not an overclaim. So each
# match is checked against its own paragraph (a run of consecutive non-blank
# lines) for one of these documented hedge/negation markers before being
# treated as a genuine violation -- an explicit, growing allowlist, same
# discipline as the R14 pattern list and the pack's own single
# "minimum cost(?!\\s+claim)" carve-out, just generalized to docs/ prose:
#   not / never / no       -- plain negation ("is not claimed", "no claim")
#   banned                 -- self-referential ban statement
#   reserved                -- the WP9-reserved framing (TRL/roadmap prose)
#   requires                -- "reaching TRL 5 ... requires ..." framing
#   only in / only path     -- explicit model/track scoping (tightened, not
#                              bare "only", to avoid exempting a genuine
#                              overclaim that happens to contain "only")
#   model-scoped            -- the R14 explicit scoping tag
#   lost                    -- "the ... guarantee is lost" (explicit failure
#                              admission, the opposite of a claim)
#   overclaims              -- meta-description of the audit's own banned-
#                              word list (technical_architecture.md
#                              describing what ctest `evidence` rejects)
_DOCS_BANNED_PATTERNS = (
    (r"flight[\s-]*(ready|proven)", "flight-ready/-proven"),
    (r"proven\s+in\s+flight", "proven in flight"),
    (r"guarantee", "guarantee(d/s)"),
    (r"approved", "approved"),
    (r"licensed", "licensed"),
    (r"compliant\b", "compliant"),
    (r"minimum[\s-]*cost(?!\s+claim)", "minimum cost (absolute claim)"),
    (r"\bTRL\s*[56]\b", "TRL 5/6 claim"),
)
_HEDGE_RE = re.compile(
    r"\b(not|never|no|banned|reserved|requires|lost|overclaims)\b"
    r"|only (in|path)"
    r"|model-scoped",
    re.IGNORECASE)


def scan_docs_r16(repo):
    """Returns a list of human-readable failure strings for the WP15/R16
    docs/ claims-audit extension: banned words (same families as the pack
    scan above, hedge-context-aware -- see _DOCS_BANNED_PATTERNS), required
    docs/ files present, and the R15 pin list never appearing as bare prose
    outside a marker block or an exempt citation (see the enforcement-scope
    comment above _EXEMPT_PHRASES)."""
    docs_dir = os.path.join(repo, "docs")
    problems = []

    for name in REQUIRED_DOCS:
        if not os.path.exists(os.path.join(docs_dir, name)):
            problems.append("required docs/%s is missing" % name)

    if not os.path.isdir(docs_dir):
        return problems

    for name in sorted(os.listdir(docs_dir)):
        if not name.endswith(".md"):
            continue
        path = os.path.join(docs_dir, name)
        with open(path, encoding="utf-8") as f:
            text = f.read()

        lines = text.split("\n")
        block_lines = _marker_block_lines(text)
        paras = _paragraphs(lines)
        para_of_line = {}
        for pi, para in enumerate(paras):
            for li in para:
                para_of_line[li] = pi

        def context_for(li, lookahead=1):
            pi = para_of_line.get(li)
            covering = list(paras[pi]) if pi is not None else []
            for k in range(1, lookahead + 1):
                if pi is not None and pi + k < len(paras):
                    covering = covering + list(paras[pi + k])
            return "\n".join(lines[i] for i in covering)

        for pat, why in _DOCS_BANNED_PATTERNS:
            for m in re.finditer(pat, text, re.IGNORECASE):
                li = text.count("\n", 0, m.start())
                if li in block_lines:
                    continue
                if _HEDGE_RE.search(context_for(li)) is not None:
                    continue
                problems.append(
                    "docs/%s:%d contains forbidden claim language with no "
                    "nearby hedge/negation: %s" % (name, li + 1, why))

        for pin in R15_DOCS_PINS:
            for li, line in enumerate(lines):
                if pin not in line:
                    continue
                if li in block_lines:
                    continue
                context = context_for(li)
                if _FORMULA_ANCHOR in context:
                    continue
                if any(p in context for p in _EXEMPT_PHRASES):
                    continue
                problems.append(
                    "docs/%s:%d has pin '%s' outside a DOCS-* marker block "
                    "with no nearby evidence-pack/generated//"
                    "reference_metrics/regenerate citation (R16)"
                    % (name, li + 1, pin))

    return problems


def main():
    if len(sys.argv) < 2:
        print("usage: test_evidence.py <repo_root>", file=sys.stderr)
        return 2
    repo = os.path.abspath(sys.argv[1])
    g = os.path.join(repo, "generated")
    committed = os.path.join(repo, "evidence", "adsc_evidence_pack.md")
    check(os.path.exists(committed), "committed evidence pack missing")
    if not os.path.exists(committed):
        print("FAIL:", fails[0])
        return 1
    with open(committed, encoding="utf-8") as f:
        pack = f.read()
    low = pack.lower()

    # 1. Forbidden overclaims / conclusion words (regex families so spaced or
    #    inflected variants cannot slip through: "flight ready", "guarantees",
    #    "proven in flight", "TRL5").
    for pat, why in ((r"flight[\s-]*(ready|proven)", "flight-ready/-proven"),
                     (r"proven\s+in\s+flight", "proven in flight"),
                     (r"guarantee", "guarantee(d/s)"),
                     (r"approved", "approved"),
                     (r"licensed", "licensed"),
                     (r"compliant\b", "compliant"),
                     # spec v5 section 1: "minimum cost" as an absolute claim
                     # is banned (cost-effectiveness is the defensible claim).
                     # The D10 reference sentence spells out the ban itself, so
                     # exclude the quoted-ban phrasing via a negative check on
                     # the exact allowed sentence rather than a lookbehind:
                     (r"minimum[\s-]*cost(?!\s+claim)", "minimum cost (absolute claim)")):
        check(re.search(pat, low) is None,
              "pack contains forbidden claim language: %s" % why)
    check(re.search(r"\bTRL\s*[56]\b", pack) is None,
          "pack claims TRL 5/6")
    for bad in ("generated at", "generated on"):
        check(bad not in low, "pack contains timestamp marker '%s'" % bad)
    check(re.search(r"\b(utc|gmt)\b", low) is None,
          "pack contains a clock-zone marker")
    try:
        pack.encode("ascii")
    except UnicodeEncodeError:
        check(False, "pack is not ASCII")

    # 2. Required honesty sections.
    for req in ("## 6. Limitations", "This is not legal advice.",
                "PLACEHOLDER inventory", "TRL 4",
                "## 11. Changelog - R15 pin supersessions",
                "### Fidelity ladder (WP12)"):
        check(req in pack, "pack lacks required element: %s" % req)
    # R14 (WP11): the keep-out claim carries an explicit fidelity-level tag
    # with the dispersion-set id and the Wilson upper bound -- a zero never
    # stands bare.
    check(re.search(r"keep-out violations \*\*0 of \d+\*\* \[L0: linear CW, "
                    r"dispersion set ds-v1, Wilson 95% upper bound 0\.\d+\]",
                    pack) is not None,
          "keep-out claim lacks the R14 level tag / Wilson upper bound")
    # R14 pattern list: headline fidelity-scoped claims keep their level tags.
    # This list must grow with new claim types (adding a headline claim
    # without adding its audit pattern is an R14 violation - spec v5 R14).
    for tag, why in ((r"\[L0: linear CW", "L0 keep-out/guidance claims"),
                     (r"\[L1: two-body\+J2", "L1 ladder re-verification"),
                     (r"ds-v2", "L2 ladder re-verification (ds-v2 stream)"),
                     (r"\[L4: L0 dynamics \+ dropout",
                      "L4 estimate-driven guidance"),
                     (r"\[L5: MIB/delay/fault", "L5 actuator realization"),
                     (r"\[model: EDT-v1 aligned-dipole",
                      "EDT-v1 model-scope tag"),
                     (r"\[DT-v1", "DT-v1 model-scope tag")):
        check(re.search(tag, pack) is not None,
              "pack lacks R14-tagged headline claim: %s" % why)
    check("element" in pack[pack.find("TRL 4"):pack.find("TRL 4") + 200],
          "TRL 4 statement is not element-scoped")
    # T7 honesty check (WP13): libration/dynamic tether stability must be
    # flagged as an open, unresolved risk -- never claimed solved.
    check("libration" in low, "pack lacks 'libration' (T7 honesty check)")
    check(re.search(r"\bT7\b", pack) is not None,
          "pack lacks a 'T7' reference (T7 honesty check)")
    check(re.search(r"UNRESOLVED", pack) is not None or
          re.search(r"\bOPEN\b", pack) is not None,
          "pack lacks 'UNRESOLVED' or 'OPEN' for the T7 libration caveat")
    # citation discipline: the pack either cites fully or marks the gap -
    # the marker must exist for the knowingly-unconfirmed external sources.
    check("[CITATION NEEDED - PLACEHOLDER" in pack,
          "unconfirmed external sources must carry the [CITATION NEEDED] marker")

    # referenced figures exist
    for m in re.finditer(r"\]\(\.\./(generated/viz/[a-z0-9_]+\.svg)\)", pack):
        check(os.path.exists(os.path.join(repo, m.group(1))),
              "referenced figure missing: %s" % m.group(1))

    # 3. Number-transcription audit (same fixed formats as the generator).
    ref = {r["metric"]: float(r["value"])
           for r in load_csv(os.path.join(g, "reference_metrics.csv"))}
    wp5 = load_csv(os.path.join(g, "wp5_campaign_summary.csv"))
    wp6 = load_csv(os.path.join(g, "wp6_cost_summary.csv"))
    wp3 = load_csv(os.path.join(g, "wp3_decay_trade.csv"))
    catA = wp5[0]["catalog"]

    def wp5m(metric):
        for r in wp5:
            if r["catalog"] == catA and r["metric"] == metric:
                return r
        raise KeyError(metric)

    def must_contain(s, why):
        check(s in pack, "pack lacks '%s' (%s)" % (s, why))

    must_contain("%.1f m" % ref["wp1_worst_coast_min_range_m"],
                 "WP1 worst coast minimum from reference_metrics.csv")
    must_contain("%.0f thrust-off coasts" % ref["wp1_coast_samples"],
                 "WP1 coast sample count")
    must_contain("%.2f s" % ref["wp2_sync_time_s"], "WP2 sync time")
    must_contain("%.2f s" % ref["wp4_sync_time_s"], "WP4 est-driven sync time")
    must_contain("%.3f J" % ref["wp3_contact_energy_j"], "contact energy")
    must_contain("%.2f s" % ref["v2_detumble_settle_time_s"], "detumble pin")
    must_contain("%.0f targets-per-mission" % ref["campaign_targets_per_mission"],
                 "campaign plan size from reference_metrics.csv")
    must_contain("~%.0f deg" % ref["wp3_target_inclination_deg"],
                 "catalog inclination from reference_metrics.csv")
    # amortization ratio at the curve minimum (catalog A)
    cpr = {int(r["n_kits"]): float(r["p50"]) for r in wp6
           if r["catalog"] == catA and r["record_type"] == "amortization"
           and r["metric"] == "cost_per_removal"}
    nmin = min(cpr, key=lambda n: cpr[n])
    for r in wp6:
        if (r["catalog"] == catA and r["record_type"] == "amortization"
                and r["metric"] == "cost_per_removal_ratio_to_n1"
                and r["n_kits"] == str(nmin)):
            must_contain("%.3f x" % float(r["p50"]), "amortization min ratio")
    # 25-yr sail areas for both catalogs
    for r in wp3:
        if r["record_type"] == "area_for_25yr":
            must_contain("%.0f..%.0f m^2" % (float(r["value_solar_max"]),
                                             float(r["value_solar_min"])),
                         "25-yr sail area range for %s" % r["catalog"])
    # WP13 EDT-v1 deorbit-time band for the SL-16 / Zenit-2 catalog, formatted
    # exactly as make_evidence.py's edt_band_str(row, f1) formats it.
    wp13 = load_csv(os.path.join(g, "wp13_kit_trade.csv"))
    for r in wp13:
        if (r["catalog"] == "SL-16 / Zenit-2 second stage"
                and r["record_type"] == "edt_years"):
            must_contain("%.1f..%.1f" % (float(r["value_lo"]),
                                         float(r["value_hi"])),
                         "SL-16 EDT-v1 deorbit-time band (WP13)")
            break
    else:
        check(False, "SL-16 edt_years row not found in wp13_kit_trade.csv")
    # rates with Wilson interval formatting
    for metric in ("success_rate", "keep_out_violation_rate",
                   "gate_abort_run_rate"):
        row = wp5m(metric)
        must_contain("%.3f [%.3f, %.3f]" % (float(row["estimate"]),
                                            float(row["wilson_low"]),
                                            float(row["wilson_high"])),
                     "%s with Wilson CI" % metric)
    # compliance summary counts
    with open(os.path.join(g, "compliance_findings.json"), encoding="utf-8") as f:
        cs = json.load(f)["summary"]
    must_contain("PASS=%d, INFO=%d, WARN=%d, BLOCK=%d" %
                 (cs["PASS"], cs["INFO"], cs["WARN"], cs["BLOCK"]),
                 "compliance summary counts")
    # PLACEHOLDER inventory is non-empty, counted, and importance-classified
    # (spec:260-264): "Total marks: **N** (decision-critical: **X**,
    # moderate: **Y**, cosmetic: **Z**)".
    m = re.search(
        r"Total marks: \*\*(\d+)\*\* \(decision-critical: \*\*(\d+)\*\*, "
        r"moderate: \*\*(\d+)\*\*, cosmetic: \*\*(\d+)\*\*\)", pack)
    check(m is not None,
          "PLACEHOLDER inventory lacks the importance-classified total-count line")
    if m:
        n_total, n_crit, n_mod, n_cosmetic = (int(g) for g in m.groups())
        n_rows = len(re.findall(r"\n\| `[^`]+:\d+` \|", pack))
        check(n_total == n_rows,
              "inventory count %d != table rows %d" % (n_total, n_rows))
        check(n_rows > 20, "inventory suspiciously small (%d)" % n_rows)
        check(n_crit + n_mod + n_cosmetic == n_total,
              "decision-critical(%d) + moderate(%d) + cosmetic(%d) != total(%d)"
              % (n_crit, n_mod, n_cosmetic, n_total))
        # Sanity floor, not a pin: as of this writing the actual count is 56;
        # 20 is chosen comfortably below that so incidental PLACEHOLDER-line
        # rewording elsewhere doesn't flip this test, while still catching a
        # real regression (e.g. the classifier silently stops matching).
        check(n_crit >= 20,
              "decision-critical count (%d) is below the 20-item sanity floor"
              % n_crit)
        # The importance column itself is present and every value recognized.
        check("| location | importance | line |" in pack,
              "inventory table lacks the importance column header")
        for label in ("decision-critical", "moderate", "cosmetic"):
            check(("| %s |" % label) in pack,
                  "inventory table has no %s-classified row" % label)

    # 4. Determinism + committed == regenerated. The committed pack content was
    # captured into `pack` above; the generator then intentionally rewrites the
    # committed path twice and must come back byte-identical both times. (On a
    # failure the working tree is left with the regenerated bytes - exactly
    # what the CI git-diff gate flags; `git checkout` restores locally.)
    shas = []
    for _ in (1, 2):
        r = subprocess.run([sys.executable,
                            os.path.join(HERE, "make_evidence.py"), repo],
                           capture_output=True, text=True)
        check(r.returncode == 0,
              "make_evidence failed: %s" % r.stderr.strip())
        if r.returncode == 0:
            shas.append(sha(committed))
    check(len(shas) == 2 and shas[0] == shas[1],
          "evidence generator is not deterministic")
    with open(committed, "rb") as f:
        regen = f.read()
    check(regen.replace(b"\r\n", b"\n") == pack.replace("\r\n", "\n").encode("utf-8"),
          "regenerated pack differs from the committed one")

    # 6. (WP15/R16) docs/ claims-audit extension: banned words, required
    # files present, R15 pin list never bare-typed outside a marker block.
    for problem in scan_docs_r16(repo):
        check(False, problem)

    if fails:
        for m2 in fails:
            print("FAIL:", m2)
        print("evidence: %d failure(s)" % len(fails))
        return 1
    print("evidence: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
