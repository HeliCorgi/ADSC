#!/usr/bin/env python3
"""WP8 Compliance Matrix Generator tests (registered with ctest as `compliance`).

  test_compliance.py <repo_root>

Checks (non-zero exit on any failure, R4):
  1.  ADR profile with target_owner_consent missing -> BLOCK.
  2.  uses_live_tle && generates_target_specific_operations -> BLOCK (policy).
  3.  Transmitter declared without RF filing evidence -> WARN (ITU placeholder).
  4.  The research-only, class-level, no-live-TLE profile is NOT blocked merely
      for being an ADR concept (BLOCK == 0).
  5.  Unknown/missing fields never PASS (strict ops -> UNKNOWN).
  6.  WP5 metadata consistency: real CSV + research profile -> INFO; a mutated
      synthetic CSV -> WARN; missing CSV -> UNKNOWN.
  7.  Output determinism: two runs -> byte-identical (SHA-256).
  8.  Reproducibility: regenerated outputs == committed generated/ + evidence/.
  9.  No 'approved'/'licensed'/'compliant' as a final conclusion; disclaimers,
      coverage-honesty and stale-law warnings present; ASCII; no timestamps.
  10. Every rule carries all mandatory fields with valid enum values.
"""
import copy
import csv
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_compliance as cc            # noqa: E402
import generate_matrix as gm             # noqa: E402

fails = []
def check(cond, msg):
    if not cond:
        fails.append(msg)

def by_id(doc, rule_id):
    for f in doc["findings"]:
        if f["rule_id"] == rule_id:
            return f
    return None

def sha(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def main():
    if len(sys.argv) < 2:
        print("usage: test_compliance.py <repo_root>", file=sys.stderr)
        return 2
    repo = os.path.abspath(sys.argv[1])
    packs_dir = os.path.join(repo, "tools", "compliance", "rulepacks")
    profile_path = os.path.join(repo, "tools", "compliance", "examples",
                                "adsc_research_profile.json")
    wp5_csv = os.path.join(repo, "generated", "wp5_campaign_runs.csv")

    packs = cc.load_rulepacks(packs_dir)
    with open(profile_path, encoding="utf-8") as f:
        base = json.load(f)

    # 10. Rulepack integrity: mandatory fields + enums on every rule.
    REQ = ["rule_id", "title", "jurisdiction", "authority", "source_reference",
           "source_date_or_version", "binding_type", "severity",
           "applies_when", "pass_when", "finding_if_fail",
           "required_evidence", "limitations"]
    ids = set()
    for p in packs:
        for r in p["rules"]:
            for k in REQ:
                check(k in r, "%s: missing rule field %s" % (r.get("rule_id", "?"), k))
            check(r["binding_type"] in ("binding", "guideline", "agency_guidance",
                                        "adsc_policy", "placeholder"),
                  "%s: bad binding_type" % r["rule_id"])
            check(r["severity"] in ("info", "warn", "blocking"),
                  "%s: bad severity" % r["rule_id"])
            check(r["rule_id"] not in ids, "duplicate rule_id %s" % r["rule_id"])
            ids.add(r["rule_id"])

    # 1. ADR with consent missing entirely -> BLOCK (absence is not consent).
    p = copy.deepcopy(base)
    del p["target_owner_consent"]
    doc = cc.evaluate(p, packs, wp5_csv, "test")
    check(by_id(doc, "ADSC-POL-01")["status"] == "BLOCK",
          "consent missing must BLOCK ADSC-POL-01, got %s"
          % by_id(doc, "ADSC-POL-01")["status"])
    check(by_id(doc, "INTL-OST-08-01")["status"] == "BLOCK",
          "consent missing must BLOCK INTL-OST-08-01")
    check(doc["summary"]["BLOCK"] >= 2, "expected >=2 BLOCK on consent-missing")
    # explicit false -> also BLOCK
    p = copy.deepcopy(base)
    p["target_owner_consent"] = False
    doc = cc.evaluate(p, packs, wp5_csv, "test")
    check(by_id(doc, "ADSC-POL-01")["status"] == "BLOCK",
          "consent=false must BLOCK")

    # 2. live TLE + target-specific products -> BLOCK (ADSC policy).
    p = copy.deepcopy(base)
    p["uses_live_tle"] = True
    p["generates_target_specific_operations"] = True
    doc = cc.evaluate(p, packs, wp5_csv, "test")
    check(by_id(doc, "ADSC-POL-02")["status"] == "BLOCK",
          "live TLE + targeting products must BLOCK ADSC-POL-02")
    # either flag alone also violates repo policy (D11) -> BLOCK
    p = copy.deepcopy(base)
    p["uses_live_tle"] = True
    doc = cc.evaluate(p, packs, wp5_csv, "test")
    check(by_id(doc, "ADSC-POL-02")["status"] == "BLOCK",
          "live TLE alone must BLOCK ADSC-POL-02")

    # 3. Transmitter without RF evidence -> WARN (ITU placeholder severity).
    p = copy.deepcopy(base)
    p["payload_has_transmitter"] = True
    doc = cc.evaluate(p, packs, wp5_csv, "test")
    check(by_id(doc, "ITU-RF-01")["status"] == "WARN",
          "transmitter w/o RF evidence must WARN ITU-RF-01, got %s"
          % by_id(doc, "ITU-RF-01")["status"])
    # with evidence declared -> PASS
    p["evidence_documents"] = list(p["evidence_documents"]) + \
        ["itu_or_national_frequency_filing_evidence"]
    doc = cc.evaluate(p, packs, wp5_csv, "test")
    check(by_id(doc, "ITU-RF-01")["status"] == "PASS",
          "transmitter with RF evidence should PASS ITU-RF-01")

    # 4. Research profile: never blocked merely for being an ADR concept.
    doc0 = cc.evaluate(copy.deepcopy(base), packs, wp5_csv, "test")
    check(doc0["summary"]["BLOCK"] == 0,
          "research profile must have BLOCK == 0, got %d" % doc0["summary"]["BLOCK"])
    check(by_id(doc0, "ADSC-META-01")["status"] == "INFO",
          "META-01 should be INFO on the committed CSV")
    check("consistent" in by_id(doc0, "ADSC-META-01")["detail"],
          "META-01 detail should state consistency")

    # 5. Unknown never PASS: strip strict-op fields -> UNKNOWN, not PASS.
    p = copy.deepcopy(base)
    del p["post_mission_disposal_lifetime_years"]
    doc = cc.evaluate(p, packs, wp5_csv, "test")
    check(by_id(doc, "IADC-ODM-25YR")["status"] == "UNKNOWN",
          "missing disposal lifetime must be UNKNOWN, got %s"
          % by_id(doc, "IADC-ODM-25YR")["status"])
    p = copy.deepcopy(base)
    del p["uses_live_tle"]
    doc = cc.evaluate(p, packs, wp5_csv, "test")
    check(by_id(doc, "ADSC-POL-02")["status"] == "UNKNOWN",
          "missing uses_live_tle must be UNKNOWN on ADSC-POL-02")
    # schema validation flags the missing field
    check(by_id(doc, "SCHEMA-VAL-01")["status"] == "WARN",
          "schema validation must WARN on missing required field")
    # unknown extra field never passes anything: identical statuses everywhere
    p = copy.deepcopy(base)
    p["mystery_field"] = True
    doc = cc.evaluate(p, packs, wp5_csv, "test")
    for f0, f1 in zip(doc0["findings"], doc["findings"]):
        if f0["rule_id"] == "SCHEMA-VAL-01":
            continue  # validation INFO-notes the unknown field
        check(f0["status"] == f1["status"],
              "unknown field changed %s status" % f0["rule_id"])

    # 6. WP5 consistency negative + missing directions.
    with tempfile.TemporaryDirectory(prefix="wp8_") as tmp:
        mutated = os.path.join(tmp, "runs.csv")
        with open(wp5_csv, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
            fieldnames = list(rows[0].keys())
        for r in rows:
            r["owner_consent_assumed"] = "false"  # mutate one policy column
        with open(mutated, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
            w.writeheader()
            w.writerows(rows)
        doc = cc.evaluate(copy.deepcopy(base), packs, mutated, "test")
        check(by_id(doc, "ADSC-META-01")["status"] == "WARN",
              "mutated WP5 CSV must WARN ADSC-META-01, got %s"
              % by_id(doc, "ADSC-META-01")["status"])
        doc = cc.evaluate(copy.deepcopy(base), packs,
                          os.path.join(tmp, "nope.csv"), "test")
        check(by_id(doc, "ADSC-META-01")["status"] == "UNKNOWN",
              "absent WP5 CSV must be UNKNOWN, never PASS")

    # 7+8. End-to-end determinism + reproducibility vs committed outputs.
    with tempfile.TemporaryDirectory(prefix="wp8_out_") as tmp:
        outs = []
        for i in (1, 2):
            oj = os.path.join(tmp, "f%d.json" % i)
            om = os.path.join(tmp, "m%d.md" % i)
            oc = os.path.join(tmp, "m%d.csv" % i)
            r1 = subprocess.run(
                [sys.executable, os.path.join(HERE, "check_compliance.py"),
                 "--out", oj], capture_output=True, text=True)
            check(r1.returncode == 0, "check_compliance failed: " + r1.stderr.strip())
            r2 = subprocess.run(
                [sys.executable, os.path.join(HERE, "generate_matrix.py"),
                 "--findings", oj, "--out-md", om, "--out-csv", oc],
                capture_output=True, text=True)
            check(r2.returncode == 0, "generate_matrix failed: " + r2.stderr.strip())
            outs.append((oj, om, oc))
        for a, b in zip(outs[0], outs[1]):
            check(sha(a) == sha(b), "non-deterministic output: %s vs %s"
                  % (os.path.basename(a), os.path.basename(b)))

        committed = [
            (outs[0][0], os.path.join(repo, "generated", "compliance_findings.json")),
            (outs[0][1], os.path.join(repo, "evidence", "compliance_matrix.md")),
            (outs[0][2], os.path.join(repo, "evidence", "compliance_matrix.csv")),
        ]
        for regen, comm in committed:
            check(os.path.exists(comm), "committed output missing: %s" % comm)
            if not os.path.exists(comm):
                continue
            with open(regen, "rb") as f:
                rb = f.read()
            with open(comm, "rb") as f:
                cb = f.read()
            # normalize CRLF for Windows checkouts; CI compares raw via git.
            check(cb.replace(b"\r\n", b"\n") == rb,
                  "committed != regenerated: %s" % os.path.basename(comm))

        # 9. Language / honesty guardrails on all three regenerated outputs.
        for path in outs[0]:
            with open(path, encoding="utf-8") as f:
                txt = f.read()
            low = txt.lower()
            for bad in ("approved", "licensed", "compliant"):
                check(bad not in low,
                      "%s contains forbidden conclusion word '%s'"
                      % (os.path.basename(path), bad))
            for bad in ("generated at", "generated on", "utc", "gmt"):
                check(bad not in low,
                      "%s contains timestamp marker '%s'"
                      % (os.path.basename(path), bad))
            try:
                txt.encode("ascii")
            except UnicodeEncodeError:
                check(False, "%s is not ASCII" % os.path.basename(path))
            check("This is not legal advice." in txt,
                  "%s lacks the not-legal-advice disclaimer" % os.path.basename(path))
        with open(outs[0][1], encoding="utf-8") as f:
            md = f.read()
        check("Russia, China and Japan" in md,
              "matrix md lacks the honest jurisdiction-coverage note")
        check("stale" in md, "matrix md lacks the stale-law warning")
        for s in ("PASS", "WARN", "BLOCK", "UNKNOWN"):
            check(s in md, "matrix md lacks summary status %s" % s)

    if fails:
        for m in fails:
            print("FAIL:", m)
        print("compliance: %d failure(s)" % len(fails))
        return 1
    print("compliance: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
