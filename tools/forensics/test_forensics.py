#!/usr/bin/env python3
"""WP10c forensics claim audit (registered with ctest as `forensics`).

  test_forensics.py <repo_root>

Checks (non-zero exit on any failure, R4):
  1. The generator's own hard gate ran clean: make_forensics.py exits 0, which
     already asserts the 1000-run replay reproduces every replay-reachable
     column of the committed wp5_campaign_runs.csv and exactly the committed
     violation set (the generator refuses to write otherwise).
  2. Determinism: two generator runs are byte-identical (SHA-256), and the
     regenerated files match the committed generated/wp10_violation_forensics.*.
  3. Content audit: one forensic row per keep_out_violation=true run in
     wp5_campaign_runs.csv with matching run_seed; every classification is
     from the defined taxonomy; every coast_min_range_m is below the keep-out
     radius; capped rows are labeled capped_abort_residual_drift and clean
     rows are not.
  4. Claims discipline on the md: same banned-language families as the
     evidence-pack audit (no flight-ready/proven, guarantee, approved,
     licensed, compliant, TRL 5/6), no timestamp/clock-zone markers, ASCII
     only, LF only, required honesty sections present.
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


def main():
    if len(sys.argv) < 2:
        print("usage: test_forensics.py <repo_root>", file=sys.stderr)
        return 2
    repo = os.path.abspath(sys.argv[1])
    gen = os.path.join(repo, "generated")
    out_csv = os.path.join(gen, "wp10_violation_forensics.csv")
    out_md = os.path.join(gen, "wp10_violation_forensics.md")
    runs_csv = os.path.join(gen, "wp5_campaign_runs.csv")

    for p in (out_csv, out_md, runs_csv):
        if not os.path.exists(p):
            print("FAIL: missing %s" % p)
            return 1
    committed = {p: sha(p) for p in (out_csv, out_md)}

    # 1 + 2. Generator runs clean twice; byte-identical; matches committed.
    tool = os.path.join(HERE, "make_forensics.py")
    hashes = []
    for _ in range(2):
        r = subprocess.run([sys.executable, tool, repo],
                           capture_output=True, text=True)
        check(r.returncode == 0,
              "make_forensics.py failed (its internal replay cross-validation "
              "gate): %s%s" % (r.stdout.strip(), r.stderr.strip()))
        if r.returncode != 0:
            break
        hashes.append((sha(out_csv), sha(out_md)))
    if len(hashes) == 2:
        check(hashes[0] == hashes[1], "generator is not deterministic")
        check(hashes[1] == (committed[out_csv], committed[out_md]),
              "regenerated forensics differ from committed bytes (R6/R12)")

    # 3. Content audit against the campaign CSV.
    runs = load_csv(runs_csv)
    viol = {(r["catalog"], r["run_index"]): r for r in runs
            if r["keep_out_violation"] == "true"}
    rows = load_csv(out_csv)
    check(len(rows) == len(viol),
          "forensics rows (%d) != violating runs (%d)" % (len(rows), len(viol)))
    taxonomy = {"capped_abort_residual_drift",
                "clean_abort_safety_ellipse_intersects_keep_out",
                "starts_inside_keep_out"}
    seen = set()
    for r in rows:
        key = (r["catalog"], r["run_index"])
        seen.add(key)
        check(key in viol, "forensics row for non-violating run %s" % (key,))
        if key in viol:
            check(r["run_seed"] == viol[key]["run_seed"],
                  "run_seed mismatch for %s" % (key,))
        check(r["classification"] in taxonomy,
              "unknown classification %r" % r["classification"])
        check(float(r["coast_min_range_m"]) < float(r["keep_out_radius_m"]),
              "row %s coast_min does not violate the keep-out radius" % (key,))
        if r["capped"] == "true":
            check(r["classification"] == "capped_abort_residual_drift",
                  "capped row %s not classified as capped mechanism" % (key,))
        else:
            check(r["classification"] != "capped_abort_residual_drift",
                  "clean row %s classified as capped mechanism" % (key,))
    check(seen == set(viol),
          "forensics rows do not cover every violating run")

    # 4. Claims discipline on the md (same families as the evidence audit).
    with open(out_md, encoding="utf-8") as f:
        md = f.read()
    low = md.lower()
    for pat, why in ((r"flight[\s-]*(ready|proven)", "flight-ready/-proven"),
                     (r"proven\s+in\s+flight", "proven in flight"),
                     (r"guarantee", "guarantee(d/s)"),
                     (r"approved", "approved"),
                     (r"licensed", "licensed"),
                     (r"compliant\b", "compliant")):
        check(re.search(pat, low) is None,
              "forensics md contains forbidden claim language: %s" % why)
    check(re.search(r"\bTRL\s*[56]\b", md) is None, "forensics md claims TRL 5/6")
    for bad in ("generated at", "generated on"):
        check(bad not in low, "forensics md contains timestamp marker '%s'" % bad)
    check(re.search(r"\b(utc|gmt)\b", low) is None,
          "forensics md contains a clock-zone marker")
    try:
        md.encode("ascii")
    except UnicodeEncodeError:
        check(False, "forensics md is not ASCII")
    with open(out_md, "rb") as f:
        check(b"\r" not in f.read(), "forensics md contains CR bytes (must be LF)")
    for req in ("Replay recipe", "Replay cross-validation", "Honest limits",
                "[L0:"):
        check(req in md, "forensics md lacks required element: %s" % req)

    if fails:
        for m in fails:
            print("FAIL:", m)
        return 1
    print("forensics: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
