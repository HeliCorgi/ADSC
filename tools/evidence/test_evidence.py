#!/usr/bin/env python3
"""WP7 evidence-pack claim audit (registered with ctest as `evidence`).

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
                      "EDT-v1 model-scope tag")):
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
    # PLACEHOLDER inventory is non-empty and counted
    m = re.search(r"Total marks: \*\*(\d+)\*\*", pack)
    check(m is not None, "PLACEHOLDER inventory lacks a total count")
    if m:
        n_rows = len(re.findall(r"\n\| `[^`]+:\d+` \|", pack))
        check(int(m.group(1)) == n_rows,
              "inventory count %s != table rows %d" % (m.group(1), n_rows))
        check(n_rows > 20, "inventory suspiciously small (%d)" % n_rows)

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

    if fails:
        for m2 in fails:
            print("FAIL:", m2)
        print("evidence: %d failure(s)" % len(fails))
        return 1
    print("evidence: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
