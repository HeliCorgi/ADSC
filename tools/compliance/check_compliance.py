#!/usr/bin/env python3
"""ADSC Compliance Matrix Generator / Regulatory Precheck - checker engine.

THIS IS NOT LEGAL ADVICE. This is a research-grade compliance PRECHECK and
evidence-matrix generator: it checks a declared mission profile against
versioned rulepacks and reports what evidence is present, missing or unknown.
It cannot and does not determine legal conformity, and it never claims to.

Python 3 standard library only (spec v4.2 R9 tooling exception; pip packages
such as jsonschema are forbidden). Profile validation is therefore implemented
manually below; tools/compliance/mission_profile.schema.json is the human- and
machine-readable DOCUMENTATION of the same contract, and the two are
dual-maintained by hand (keep them in sync).

Core policy (spec WP8):
  * Unknown is safer than a false PASS: a rule whose referenced profile field
    is missing evaluates UNKNOWN (strict ops) or fails outright (affirmative
    ops such as consent declarations) - it NEVER passes.
  * Missing mandatory evidence yields WARN or BLOCK, never PASS.
  * Unconsented active interference with a registered space object is BLOCKed
    as ADSC policy regardless of any legal argument.
  * Every finding carries binding_type so legally-binding requirements,
    non-binding guidelines, agency guidance, ADSC internal policy, and
    placeholder research assumptions are never conflated.

Determinism (spec v4.2 R6): output carries no wall-clock timestamp or run
time; identical inputs produce byte-identical output (fixed rulepack order,
fixed key order, LF line endings, ASCII).

Usage:
  check_compliance.py [--profile P] [--rulepacks-dir D] [--wp5-csv C] [--out O]
Defaults are repo-relative (run from anywhere).
"""
import argparse
import csv
import json
import os
import sys

SCHEMA_VERSION = "1.0"
DISCLAIMER = "This is not legal advice."
COVERAGE_NOTE = (
    "Jurisdiction coverage is currently limited to the UN space treaties, US "
    "agency rules (FCC/FAA/NOAA), ITU references and ESA reference guidelines. "
    "National law of other launching states relevant to ADSC's adopter framing "
    "- notably Russia, China and Japan - is NOT yet covered and is future work."
)
STALE_NOTE = (
    "Rulepacks are versioned research artifacts (see source_date_or_version "
    "per rule). Regulations change; rulepacks can go stale and must be "
    "re-verified against current law before any real-world use."
)

# Fixed load order (determinism, R6).
RULEPACK_ORDER = ["international", "us_fcc", "us_faa", "us_noaa", "itu",
                  "esa_reference", "adsc_policy"]

# ---------------------------------------------------------------------------
# Profile validation (manual, stdlib-only; mirrors mission_profile.schema.json)
# ---------------------------------------------------------------------------
REQUIRED_FIELDS = {
    "mission_name": str,
    "mission_type": str,
    "research_only": bool,
    "launching_state": str,
    "operating_state": str,
    "registry_state": str,
    "target_owner_state": str,
    "target_registered_state": str,
    "target_owner_consent": bool,
    "active_debris_removal": bool,
    "proximity_operations": bool,
    "payload_has_transmitter": bool,
    "payload_has_remote_sensing": bool,
    "controlled_reentry": bool,
    "post_mission_disposal_lifetime_years": (int, float),
    "uses_live_tle": bool,
    "generates_target_specific_operations": bool,
    "insurance_or_indemnification_evidence": str,
    "export_control_review_status": str,
    "evidence_documents": list,
}
OPTIONAL_FIELDS = {"notes": str}
MISSION_TYPES = ["research_simulation", "flight"]


def validate_profile(profile):
    """Return (issues_warn, issues_info): human-readable validation issues."""
    warns, infos = [], []
    for name in sorted(REQUIRED_FIELDS):
        typ = REQUIRED_FIELDS[name]
        if name not in profile:
            warns.append("required field missing: %s" % name)
        elif not isinstance(profile[name], typ) or (
                typ is bool and not isinstance(profile[name], bool)):
            warns.append("field has wrong type: %s" % name)
    if isinstance(profile.get("evidence_documents"), list):
        for i, e in enumerate(profile["evidence_documents"]):
            if not isinstance(e, str):
                warns.append("evidence_documents[%d] is not a string" % i)
    mt = profile.get("mission_type")
    if isinstance(mt, str) and mt not in MISSION_TYPES:
        warns.append("mission_type '%s' not in documented enum %s" %
                     (mt, MISSION_TYPES))
    for name in sorted(profile):
        if name not in REQUIRED_FIELDS and name not in OPTIONAL_FIELDS:
            infos.append("unknown field ignored by all rules (never a PASS "
                         "source): %s" % name)
    return warns, infos


# ---------------------------------------------------------------------------
# Tri-state condition evaluation: returns ("PASS"|"FAIL"|"UNKNOWN", notes)
# ---------------------------------------------------------------------------
# Strict ops (eq/ne/lte/gte/in): a missing field -> UNKNOWN (never PASS).
# Affirmative ops (affirmed_true/affirmed_false/contains/exists): the safe
# direction is failure - a missing field FAILS (e.g. absent consent is treated
# as no consent), it never passes and never silently disappears as UNKNOWN.
def eval_cond(cond, profile, ctx):
    if "all" in cond:
        results = [eval_cond(c, profile, ctx) for c in cond["all"]]
        notes = [n for _, ns in results for n in ns]
        states = [s for s, _ in results]
        if "FAIL" in states:
            return "FAIL", notes
        if "UNKNOWN" in states:
            return "UNKNOWN", notes
        return "PASS", notes
    if "any" in cond:
        results = [eval_cond(c, profile, ctx) for c in cond["any"]]
        notes = [n for _, ns in results for n in ns]
        states = [s for s, _ in results]
        if "PASS" in states:
            return "PASS", notes
        if "UNKNOWN" in states:
            return "UNKNOWN", notes
        return "FAIL", notes
    if "not" in cond:
        s, notes = eval_cond(cond["not"], profile, ctx)
        if s == "PASS":
            return "FAIL", notes
        if s == "FAIL":
            return "PASS", notes
        return "UNKNOWN", notes  # not(UNKNOWN) stays UNKNOWN (never PASS)

    op = cond.get("op")
    if op == "always":
        return "PASS", []
    if op == "wp5_metadata_consistent":
        return ctx["wp5_result"], list(ctx["wp5_notes"])

    field = cond.get("field", "")
    present = field in profile
    value = profile.get(field)
    want = cond.get("value")

    if op in ("eq", "ne", "lte", "gte", "in"):
        if not present:
            return "UNKNOWN", ["%s: field missing -> UNKNOWN (never PASS)" % field]
        try:
            if op == "eq":
                ok = value == want
            elif op == "ne":
                ok = value != want
            elif op == "lte":
                ok = float(value) <= float(want)
            elif op == "gte":
                ok = float(value) >= float(want)
            else:  # in
                ok = value in want
        except (TypeError, ValueError):
            return "UNKNOWN", ["%s: not comparable -> UNKNOWN" % field]
        return ("PASS" if ok else "FAIL"), []

    if op == "affirmed_true":
        if present and value is True:
            return "PASS", []
        note = ("%s: missing - affirmative declaration required, absence "
                "treated as false (safe direction)" % field) if not present else []
        return "FAIL", [note] if note else []
    if op == "affirmed_false":
        if present and value is False:
            return "PASS", []
        note = ("%s: missing - affirmative declaration required" % field) \
            if not present else None
        return "FAIL", [note] if note else []
    if op == "exists":
        return ("PASS" if present else "FAIL"), []
    if op == "contains":
        if present and isinstance(value, list) and want in value:
            return "PASS", []
        return "FAIL", ["%s: does not contain '%s'" % (field, want)]

    return "UNKNOWN", ["unrecognized op '%s' -> UNKNOWN (never PASS)" % str(op)]


# ---------------------------------------------------------------------------
# WP5 campaign metadata consistency (consumes the WP5 runs CSV, schema 1.0)
# ---------------------------------------------------------------------------
# profile field -> WP5 runs.csv compliance column
WP5_COLUMN_MAP = [
    ("research_only", "research_only"),
    ("uses_live_tle", "uses_live_tle"),
    ("generates_target_specific_operations", "generates_target_specific_operations"),
    ("target_owner_consent", "owner_consent_assumed"),
    ("controlled_reentry", "controlled_reentry_mode"),
    ("payload_has_transmitter", "rf_transmitter_modelled"),
    ("payload_has_remote_sensing", "remote_sensing_modelled"),
]
# columns that must be uniformly 'true' for the ADSC research policy to hold
WP5_POLICY_TRUE_COLUMNS = ["class_level_preset", "unconsented_approach_blocked_by_policy"]


def check_wp5_consistency(profile, wp5_csv_path):
    """Return ("PASS"|"FAIL"|"UNKNOWN", notes)."""
    if not os.path.exists(wp5_csv_path):
        return "UNKNOWN", ["WP5 runs CSV not found -> UNKNOWN (never PASS)"]
    try:
        with open(wp5_csv_path, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    except (OSError, csv.Error) as e:
        return "UNKNOWN", ["WP5 runs CSV unreadable: %s" % e]
    if not rows:
        return "UNKNOWN", ["WP5 runs CSV has no rows -> UNKNOWN"]

    notes, mismatches = [], []
    needed = [c for _, c in WP5_COLUMN_MAP] + WP5_POLICY_TRUE_COLUMNS
    for col in needed:
        if col not in rows[0]:
            return "UNKNOWN", ["WP5 runs CSV lacks column '%s' -> UNKNOWN" % col]
        vals = sorted(set(r[col] for r in rows))
        if len(vals) != 1:
            mismatches.append("column %s is not uniform across runs: %s" %
                              (col, vals))
    if mismatches:
        return "FAIL", mismatches

    for pf, col in WP5_COLUMN_MAP:
        if pf not in profile or not isinstance(profile[pf], bool):
            return "UNKNOWN", ["profile field %s missing/non-bool -> UNKNOWN" % pf]
        want = "true" if profile[pf] else "false"
        got = rows[0][col]
        if got != want:
            mismatches.append("profile %s=%s but WP5 column %s=%s" %
                              (pf, want, col, got))
        else:
            notes.append("%s == %s (%s)" % (pf, col, got))
    for col in WP5_POLICY_TRUE_COLUMNS:
        if rows[0][col] != "true":
            mismatches.append("WP5 column %s=%s but ADSC policy requires true" %
                              (col, rows[0][col]))
    if mismatches:
        return "FAIL", mismatches
    return "PASS", ["all %d mapped columns + %d policy columns consistent "
                    "across %d runs" % (len(WP5_COLUMN_MAP),
                                        len(WP5_POLICY_TRUE_COLUMNS), len(rows))]


# ---------------------------------------------------------------------------
# Rule evaluation
# ---------------------------------------------------------------------------
SEVERITY_TO_FAIL_STATUS = {"info": "INFO", "warn": "WARN", "blocking": "BLOCK"}
STATUS_ORDER = ["PASS", "INFO", "WARN", "BLOCK", "UNKNOWN", "NOT_APPLICABLE"]


def evaluate_rule(rule, profile, ctx):
    finding = {
        "rule_id": rule["rule_id"],
        "title": rule["title"],
        "jurisdiction": rule["jurisdiction"],
        "authority": rule["authority"],
        "source_reference": rule["source_reference"],
        "source_date_or_version": rule["source_date_or_version"],
        "binding_type": rule["binding_type"],
        "severity": rule["severity"],
        "status": "UNKNOWN",
        "detail": "",
        "required_evidence": list(rule.get("required_evidence", [])),
        "missing_evidence": [],
        "limitations": rule["limitations"],
    }
    notes = []

    app, app_notes = eval_cond(rule["applies_when"], profile, ctx)
    notes.extend(app_notes)
    if app == "FAIL":
        finding["status"] = "NOT_APPLICABLE"
        finding["detail"] = "rule does not apply to this profile"
        return finding
    if app == "UNKNOWN":
        notes.append("applicability UNKNOWN - treated as applicable "
                     "(unknown is safer than a silent skip)")

    res, res_notes = eval_cond(rule["pass_when"], profile, ctx)
    notes.extend(res_notes)
    if res == "UNKNOWN":
        finding["status"] = "UNKNOWN"
        finding["detail"] = "; ".join(
            ["cannot evaluate - missing/unknown input (never PASS)"] + notes)
        return finding
    if res == "FAIL":
        finding["status"] = SEVERITY_TO_FAIL_STATUS[rule["severity"]]
        finding["detail"] = "; ".join([rule["finding_if_fail"]] + notes)
        return finding

    # pass_when passed -> check declared evidence.
    evid = profile.get("evidence_documents")
    evid = evid if isinstance(evid, list) else []
    missing = [t for t in finding["required_evidence"] if t not in evid]
    if missing:
        finding["status"] = "WARN"
        finding["missing_evidence"] = missing
        finding["detail"] = "; ".join(
            ["condition satisfied but required evidence not declared: %s"
             % ", ".join(missing)] + notes)
        return finding

    finding["status"] = rule.get("status_on_pass", "PASS")
    finding["detail"] = "; ".join(
        [rule.get("detail_on_pass", "condition satisfied")] + notes)
    return finding


def load_rulepacks(rulepacks_dir):
    packs = []
    for name in RULEPACK_ORDER:  # fixed order, not directory order (R6)
        path = os.path.join(rulepacks_dir, name + ".json")
        with open(path, encoding="utf-8") as f:
            packs.append(json.load(f))
    return packs


def evaluate(profile, packs, wp5_csv_path, profile_ref):
    wp5_result, wp5_notes = check_wp5_consistency(profile, wp5_csv_path)
    ctx = {"wp5_result": wp5_result, "wp5_notes": wp5_notes}

    findings = []
    warns, infos = validate_profile(profile)
    findings.append({
        "rule_id": "SCHEMA-VAL-01",
        "title": "Mission profile schema validation (stdlib manual validation)",
        "jurisdiction": "ADSC internal",
        "authority": "tools/compliance/mission_profile.schema.json (documentation twin)",
        "source_reference": "ADSC mission_profile.schema.json (adsc-mission-profile-1.0)",
        "source_date_or_version": "schema 1.0",
        "binding_type": "adsc_policy",
        "severity": "warn",
        "status": "WARN" if warns else "PASS",
        "detail": "; ".join(warns + infos) if (warns or infos)
                  else "all required fields present with expected types",
        "required_evidence": [],
        "missing_evidence": [],
        "limitations": "Manual validation dual-maintained with the schema "
                       "document; unknown fields are ignored by every rule and "
                       "can never produce a PASS.",
    })
    for pack in packs:
        for rule in pack["rules"]:
            findings.append(evaluate_rule(rule, profile, ctx))

    summary = {s: 0 for s in STATUS_ORDER}
    for f in findings:
        summary[f["status"]] += 1

    return {
        "schema_version": SCHEMA_VERSION,
        "tool": "ADSC Compliance Matrix Generator / Regulatory Precheck",
        "disclaimer": DISCLAIMER,
        "coverage_note": COVERAGE_NOTE,
        "stale_note": STALE_NOTE,
        "profile_name": profile.get("mission_name", "(unnamed profile)"),
        "profile_ref": profile_ref,
        "rulepacks": [{"rulepack": p["rulepack"],
                       "rulepack_version": p["rulepack_version"],
                       "rules": len(p["rules"])} for p in packs],
        "summary": summary,
        "findings": findings,
    }


def write_findings(doc, out_path):
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(doc, indent=2, ensure_ascii=True) + "\n")


def repo_root():
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "..", ".."))


def main(argv=None):
    root = repo_root()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--profile",
                    default=os.path.join(root, "tools", "compliance", "examples",
                                         "adsc_research_profile.json"))
    ap.add_argument("--rulepacks-dir",
                    default=os.path.join(root, "tools", "compliance", "rulepacks"))
    ap.add_argument("--wp5-csv",
                    default=os.path.join(root, "generated", "wp5_campaign_runs.csv"))
    ap.add_argument("--out",
                    default=os.path.join(root, "generated", "compliance_findings.json"))
    args = ap.parse_args(argv)

    with open(args.profile, encoding="utf-8") as f:
        profile = json.load(f)
    packs = load_rulepacks(args.rulepacks_dir)

    # Record a platform-independent, repo-relative profile reference (forward
    # slashes) so the committed output is byte-identical on every OS (R6).
    try:
        profile_ref = os.path.relpath(args.profile, root).replace(os.sep, "/")
    except ValueError:
        profile_ref = os.path.basename(args.profile)

    doc = evaluate(profile, packs, args.wp5_csv, profile_ref)
    write_findings(doc, args.out)

    print("compliance precheck: %s" % DISCLAIMER)
    print("profile : %s" % doc["profile_name"])
    print("summary : " + "  ".join("%s=%d" % (s, doc["summary"][s])
                                   for s in STATUS_ORDER))
    for f in doc["findings"]:
        if f["status"] in ("BLOCK", "WARN", "UNKNOWN"):
            print("  [%s] %s - %s" % (f["status"], f["rule_id"], f["title"]))
    print("wrote %s" % os.path.relpath(args.out, os.getcwd()).replace(os.sep, "/"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
