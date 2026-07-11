#!/usr/bin/env python3
"""WP14 prioritization-table tests (registered with ctest, run by CMake's
python3).

  test_prioritization.py <repo_root>

Checks (returns non-zero on any failure, R4), mirroring tools/viz/test_viz.py:
  1. Regenerating from the committed CSVs (wp6_cost_summary.csv,
     wp13_kit_trade.csv, wp5_campaign_summary.csv) produces both expected
     files and the generator itself exits 0 (its own internal cross-check
     assertion between this tool's arithmetic and the committed WP6 anchor
     row is part of that exit code).
  2. ASCII-only, LF-only, no stub markers (TODO/FIXME/XXX/TBD), no wall-clock
     timestamp markers (R6), and no banned overclaim language.
  3. D10/R6 currency discipline: no point-dollar glyph anywhere, and every
     currency-bearing row (MUSD units) is a genuine range (p05 < p95).
  4. Content: all three catalogs present (SL-16, SL-8, Envisat-class); the
     Envisat-class row states it has no campaign rows; the D12 sentence,
     the McKnight citation and the T5/honesty note are present in the .md.
  5. Determinism: two generator runs are byte-identical, and both match the
     committed generated/wp14_prioritization.{csv,md} exactly (reproducibility
     -- the same contract CI enforces with git diff).
"""
import csv
import hashlib
import os
import re
import subprocess
import sys
import tempfile

STUB_MARKERS = ("TODO", "FIXME", "XXX", "TBD")
TIMESTAMP_MARKERS = ("2026-", "2027-", "UTC", "GMT", "generated at", "generated on")
BANNED_CLAIM_PATTERNS = (
    (r"flight[\s-]*(ready|proven)", "flight-ready/-proven"),
    (r"guarantee", "guarantee(d/s)"),
    (r"approved", "approved"),
    (r"licensed", "licensed"),
    (r"compliant\b", "compliant"),
)
EXPECTED = ("wp14_prioritization.csv", "wp14_prioritization.md")

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


def main():
    if len(sys.argv) < 2:
        print("usage: test_prioritization.py <repo_root>", file=sys.stderr)
        return 2
    repo = os.path.abspath(sys.argv[1])
    gen = os.path.join(repo, "generated")
    make = os.path.join(repo, "tools", "prioritization", "make_prioritization.py")
    committed = {name: os.path.join(gen, name) for name in EXPECTED}

    for name, path in committed.items():
        if not os.path.exists(path):
            print("FAIL: committed generated/%s is missing" % name)
            return 1

    # 1. Regenerate into a scratch dir; the generator's own exit code carries
    #    its internal cross-check (this tool's own CU x anchor arithmetic vs
    #    the committed WP6 cost_per_removal_musd row for catalog A).
    tmp = tempfile.mkdtemp(prefix="wp14_prioritization_")
    r = subprocess.run([sys.executable, make, repo, tmp],
                       capture_output=True, text=True)
    check(r.returncode == 0,
          "make_prioritization failed: %s" % (r.stderr.strip() or r.stdout.strip()))

    for name in EXPECTED:
        path = os.path.join(tmp, name)
        check(os.path.exists(path), "missing regenerated %s" % name)
    if fails:
        for f in fails:
            print("FAIL:", f)
        print("prioritization: %d failure(s)" % len(fails))
        return 1

    csv_txt = open(os.path.join(tmp, "wp14_prioritization.csv"), encoding="utf-8").read()
    md_txt = open(os.path.join(tmp, "wp14_prioritization.md"), encoding="utf-8").read()

    # 2. Hygiene: ASCII, LF-only, no stub markers / timestamps / banned claims.
    for name, txt in (("wp14_prioritization.csv", csv_txt),
                      ("wp14_prioritization.md", md_txt)):
        try:
            txt.encode("ascii")
        except UnicodeEncodeError:
            check(False, "%s: non-ASCII content" % name)
        with open(os.path.join(tmp, name), "rb") as f:
            check(b"\r" not in f.read(), "%s: contains CR bytes (must be LF)" % name)
        for m in STUB_MARKERS + TIMESTAMP_MARKERS:
            check(m not in txt, "%s: contains forbidden marker '%s'" % (name, m))
        low = txt.lower()
        for pat, why in BANNED_CLAIM_PATTERNS:
            check(re.search(pat, low) is None,
                  "%s: contains forbidden claim language: %s" % (name, why))
        check(re.search(r"\bTRL\s*[56]\b", txt) is None,
              "%s: claims TRL 5/6" % name)

    # 3. D10/R6 currency discipline: never a point-dollar glyph; every
    #    currency-bearing (MUSD) row is a genuine range (p05 strictly < p95).
    check("$" not in csv_txt, "csv: contains a point-dollar glyph")
    check("$" not in md_txt, "md: contains a point-dollar glyph")
    rows = load_csv(os.path.join(tmp, "wp14_prioritization.csv"))
    musd_rows = [r for r in rows if "MUSD" in r["units"]]
    check(len(musd_rows) > 0, "csv: no MUSD-denominated rows found at all")
    for r in musd_rows:
        p05, p95 = float(r["p05"]), float(r["p95"])
        check(p05 < p95,
              "csv: MUSD row (%s/%s/%s) is not a genuine range (p05 %.6f !< p95 %.6f)"
              % (r["catalog"], r["record_type"], r["cost_scenario"], p05, p95))

    # 4. Content checks.
    catalogs = {r["catalog"] for r in rows if r["catalog"]}
    for cat in ("SL-16 / Zenit-2 second stage", "SL-8 / Kosmos-3M second stage",
               "Envisat-class massive SSO payload"):
        check(cat in catalogs, "csv: missing catalog %r" % cat)
    check(any(r["record_type"] == "mission_class"
             and r["catalog"] == "Envisat-class massive SSO payload"
             for r in rows),
          "csv: Envisat-class row does not state it has no campaign rows")
    check("no wp5/wp6 campaign rows" in md_txt.lower(),
          "md: Envisat-class section does not state it has no campaign rows")
    check("D12" in md_txt, "md: missing the D12 legal-gate reference")
    check("gate and metadata flags, never a multiplier" in md_txt,
          "md: missing the D12 gate-not-multiplier sentence")
    check("McKnight" in md_txt, "md: missing the McKnight citation")
    check("doi:10.1016/j.actaastro.2021.01.021" in md_txt,
          "md: McKnight citation lacks its DOI")
    check("Honesty note" in md_txt, "md: missing the honesty-note section")
    check("T5" in md_txt, "md: missing a T5 reference in the honesty note")
    check("PLACEHOLDER" in md_txt,
          "md: honesty note must acknowledge the non-peak weight table is "
          "still PLACEHOLDER")

    # every non-mission_class/priority_rank row must carry a source-bearing
    # notes field (D10-style discipline: never a bare, unexplained number).
    for r in rows:
        if r["record_type"] in ("fom", "cost_per_removal_cu",
                                "cost_per_removal_musd", "recommended_kit",
                                "sail_area_25yr", "edt_years"):
            check(r["notes"].strip() != "",
                  "csv row %s/%s/%s has an empty notes field"
                  % (r["catalog"], r["record_type"], r.get("weighting", "")))

    # 5. Determinism + committed == regenerated.
    tmp2 = tempfile.mkdtemp(prefix="wp14_prioritization_2_")
    r2 = subprocess.run([sys.executable, make, repo, tmp2],
                        capture_output=True, text=True)
    check(r2.returncode == 0, "second make_prioritization run failed")
    if r2.returncode == 0:
        for name in EXPECTED:
            h1 = sha(os.path.join(tmp, name))
            h2 = sha(os.path.join(tmp2, name))
            check(h1 == h2, "%s: generator is not deterministic across runs" % name)

    for name, cpath in committed.items():
        tpath = os.path.join(tmp, name)
        if not os.path.exists(tpath):
            continue
        with open(cpath, "rb") as a, open(tpath, "rb") as b:
            cb, tb = a.read(), b.read()
        # normalize CRLF->LF so a Windows checkout of the committed file still
        # matches the LF the generator writes (CI enforces raw bytes via git).
        check(cb.replace(b"\r\n", b"\n") == tb,
              "%s: committed bytes != regenerated (reproducibility)" % name)

    if fails:
        for f in fails:
            print("FAIL:", f)
        print("prioritization: %d failure(s)" % len(fails))
        return 1
    print("prioritization: all checks passed (%d artifacts, %d rows)"
          % (len(EXPECTED), len(rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
