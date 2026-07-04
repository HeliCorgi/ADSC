#!/usr/bin/env python3
"""WP10c/WP11 forensics + abort-audit claim audit (ctest `forensics`).

  test_forensics.py <repo_root>

Checks (non-zero exit on any failure, R4):
  1. The generator's own hard gates ran clean: make_forensics.py exits 0,
     which already asserts (a) the WP11-audit clearing-law replay reproduces
     every replay-reachable column of the committed wp5_campaign_runs.csv for
     all runs with ZERO committed keep-out violations (D13), and (b) the
     WP10-archive legacy-law replay reproduces exactly the pinned forensic-14
     violation set from the same seeds.
  2. Determinism: two generator runs are byte-identical (SHA-256) across all
     FOUR artifacts, and the regenerated files match the committed
     generated/wp10_violation_forensics.{csv,md} and
     generated/wp11_abort_audit.{csv,md}.
  3. Content audit: the WP10 archive has one row per pinned forensic-14 case
     with the defined taxonomy; the WP11 audit has one row per gate-abort
     event in the committed campaign, every status from the clearing-law
     ladder, and every clearance non-negative (zero violations, D13).
  4. Claims discipline on both mds: same banned-language families as the
     evidence-pack audit, no timestamp/clock-zone markers, ASCII + LF only,
     required sections and R14 level tags present.
"""
import csv
import hashlib
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


FORENSIC14 = {
    ("SL-16 / Zenit-2 second stage", "65"),
    ("SL-16 / Zenit-2 second stage", "81"),
    ("SL-16 / Zenit-2 second stage", "86"),
    ("SL-16 / Zenit-2 second stage", "112"),
    ("SL-16 / Zenit-2 second stage", "340"),
    ("SL-16 / Zenit-2 second stage", "460"),
    ("SL-16 / Zenit-2 second stage", "490"),
    ("SL-8 / Kosmos-3M second stage", "88"),
    ("SL-8 / Kosmos-3M second stage", "99"),
    ("SL-8 / Kosmos-3M second stage", "160"),
    ("SL-8 / Kosmos-3M second stage", "254"),
    ("SL-8 / Kosmos-3M second stage", "362"),
    ("SL-8 / Kosmos-3M second stage", "383"),
    ("SL-8 / Kosmos-3M second stage", "419"),
}


def audit_md(path, required, label):
    with open(path, encoding="utf-8") as f:
        md = f.read()
    low = md.lower()
    for pat, why in ((r"flight[\s-]*(ready|proven)", "flight-ready/-proven"),
                     (r"proven\s+in\s+flight", "proven in flight"),
                     (r"guarantee", "guarantee(d/s)"),
                     (r"approved", "approved"),
                     (r"licensed", "licensed"),
                     (r"compliant\b", "compliant")):
        check(re.search(pat, low) is None,
              "%s contains forbidden claim language: %s" % (label, why))
    check(re.search(r"\bTRL\s*[56]\b", md) is None, "%s claims TRL 5/6" % label)
    for bad in ("generated at", "generated on"):
        check(bad not in low, "%s contains timestamp marker '%s'" % (label, bad))
    check(re.search(r"\b(utc|gmt)\b", low) is None,
          "%s contains a clock-zone marker" % label)
    try:
        md.encode("ascii")
    except UnicodeEncodeError:
        check(False, "%s is not ASCII" % label)
    with open(path, "rb") as f:
        check(b"\r" not in f.read(), "%s contains CR bytes (must be LF)" % label)
    for req in required:
        check(req in md, "%s lacks required element: %s" % (label, req))
    return md


def main():
    if len(sys.argv) < 2:
        print("usage: test_forensics.py <repo_root>", file=sys.stderr)
        return 2
    repo = os.path.abspath(sys.argv[1])
    gen = os.path.join(repo, "generated")
    wp10_csv = os.path.join(gen, "wp10_violation_forensics.csv")
    wp10_md = os.path.join(gen, "wp10_violation_forensics.md")
    wp11_csv = os.path.join(gen, "wp11_abort_audit.csv")
    wp11_md = os.path.join(gen, "wp11_abort_audit.md")
    runs_csv = os.path.join(gen, "wp5_campaign_runs.csv")

    outputs = (wp10_csv, wp10_md, wp11_csv, wp11_md)
    for p in outputs + (runs_csv,):
        if not os.path.exists(p):
            print("FAIL: missing %s" % p)
            return 1
    committed = {p: sha(p) for p in outputs}

    # 1 + 2. Generator runs clean twice; byte-identical; matches committed.
    tool = os.path.join(HERE, "make_forensics.py")
    hashes = []
    for _ in range(2):
        r = subprocess.run([sys.executable, tool, repo],
                           capture_output=True, text=True)
        check(r.returncode == 0,
              "make_forensics.py failed (its internal replay gates): %s%s"
              % (r.stdout.strip(), r.stderr.strip()))
        if r.returncode != 0:
            break
        hashes.append(tuple(sha(p) for p in outputs))
    if len(hashes) == 2:
        check(hashes[0] == hashes[1], "generator is not deterministic")
        check(hashes[1] == tuple(committed[p] for p in outputs),
              "regenerated forensics/audit differ from committed bytes (R6/R12)")

    # 3a. WP10 archive content.
    rows10 = load_csv(wp10_csv)
    check({(r["catalog"], r["run_index"]) for r in rows10} == FORENSIC14,
          "WP10 archive rows do not match the pinned forensic-14 set")
    taxonomy = {"capped_abort_residual_drift",
                "clean_abort_safety_ellipse_intersects_keep_out",
                "starts_inside_keep_out"}
    for r in rows10:
        check(r["classification"] in taxonomy,
              "unknown WP10 classification %r" % r["classification"])
        check(float(r["coast_min_range_m"]) < float(r["keep_out_radius_m"]),
              "WP10 archive row %s does not violate keep-out"
              % ((r["catalog"], r["run_index"]),))

    # 3b. WP11 audit content.
    runs = load_csv(runs_csv)
    total_aborts = sum(int(r["gate_abort_events"]) for r in runs)
    check(sum(1 for r in runs if r["keep_out_violation"] == "true") == 0,
          "committed campaign has keep-out violations (D13 regression)")
    rows11 = load_csv(wp11_csv)
    check(len(rows11) == total_aborts,
          "WP11 audit rows (%d) != committed gate-abort events (%d)"
          % (len(rows11), total_aborts))
    statuses = {"Clean", "BoundedClearing", "RetreatHop", "Capped"}
    for r in rows11:
        check(r["status"] in statuses, "unknown audit status %r" % r["status"])
        check(float(r["clearance_m"]) >= 0.0,
              "audit row %s has negative clearance (violation)"
              % ((r["catalog"], r["run_index"], r["target_index"]),))

    # 4. Claims discipline + required sections on both mds.
    audit_md(wp10_md,
             ("ARCHIVE", "Supersession note (WP11)", "Replay recipe",
              "Replay cross-validation", "[L0:", "Honest limits"),
             "wp10 archive md")
    md11 = audit_md(wp11_md,
                    ("Replay cross-validation", "Keep-out result", "[L0:",
                     "Wilson 95% upper bound", "abort_dv as a design variable",
                     "honest limits"),
                    "wp11 audit md")
    check(re.search(r"\*\*0 of \d+\*\* \[L0: linear CW, dispersion\s+set "
                    r"ds-v1, Wilson 95% upper bound 0\.\d+\]", md11) is not None,
          "wp11 audit md lacks the R14-tagged zero-violation claim")

    if fails:
        for m in fails:
            print("FAIL:", m)
        return 1
    print("forensics: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
