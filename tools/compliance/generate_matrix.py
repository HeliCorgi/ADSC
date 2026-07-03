#!/usr/bin/env python3
"""ADSC Compliance Matrix Generator - renders findings JSON to Markdown + CSV.

THIS IS NOT LEGAL ADVICE (the outputs repeat this). Python 3 standard library
only (spec v4.2 R9). Deterministic: no wall-clock timestamps; identical input
produces byte-identical output (LF, ASCII, fixed column order) so the CI
reproducibility gate can compare bytes (R6).

Usage:
  generate_matrix.py [--findings F] [--out-md M] [--out-csv C]
Defaults are repo-relative.
"""
import argparse
import csv
import json
import os
import sys

CSV_COLUMNS = [
    "schema_version", "rule_id", "title", "jurisdiction", "authority",
    "binding_type", "severity", "status", "detail", "source_reference",
    "source_date_or_version", "required_evidence", "missing_evidence",
    "limitations",
]

BINDING_LEGEND = [
    ("binding", "legally binding requirement (treaty/statute/regulation) in its jurisdiction"),
    ("guideline", "non-binding guideline or reference standard"),
    ("agency_guidance", "agency guidance/reference process, not a general legal requirement"),
    ("adsc_policy", "ADSC internal project policy - stricter than law by design, not law"),
    ("placeholder", "simplified research-grade rule; content not confident - requires legal review"),
]
STATUS_ORDER = ["PASS", "INFO", "WARN", "BLOCK", "UNKNOWN", "NOT_APPLICABLE"]


def render_md(doc):
    L = []
    L.append("# ADSC Compliance Matrix (Regulatory Precheck)")
    L.append("")
    L.append("**%s** This matrix is a research-grade PRECHECK of a declared" % doc["disclaimer"])
    L.append("mission profile against versioned rulepacks: it reports which prechecks pass,")
    L.append("which evidence is missing and what is unknown. It is not a legal conformity")
    L.append("determination and must not be used as one.")
    L.append("")
    L.append("- Profile: **%s** (`%s`)" % (doc["profile_name"], doc["profile_ref"]))
    L.append("- Findings schema_version: `%s`" % doc["schema_version"])
    L.append("- Coverage (honest): %s" % doc["coverage_note"])
    L.append("- Staleness: %s" % doc["stale_note"])
    L.append("- Policy: unknown or missing inputs are never treated as PASS; missing")
    L.append("  mandatory evidence yields WARN or BLOCK; unconsented active interference")
    L.append("  with a registered space object is BLOCKed as ADSC policy.")
    L.append("")
    L.append("## Summary")
    L.append("")
    L.append("| " + " | ".join(STATUS_ORDER) + " |")
    L.append("|" + "---:|" * len(STATUS_ORDER))
    L.append("| " + " | ".join(str(doc["summary"][s]) for s in STATUS_ORDER) + " |")
    L.append("")
    L.append("Rulepacks: " + ", ".join(
        "%s %s (%d rules)" % (p["rulepack"], p["rulepack_version"], p["rules"])
        for p in doc["rulepacks"]))
    L.append("")
    L.append("## Binding-type legend (never conflated)")
    L.append("")
    for k, v in BINDING_LEGEND:
        L.append("- `%s` - %s" % (k, v))
    L.append("")
    L.append("## Findings")
    L.append("")
    L.append("| status | rule_id | title | jurisdiction | binding_type | severity | detail |")
    L.append("|---|---|---|---|---|---|---|")
    for f in doc["findings"]:
        L.append("| %s | %s | %s | %s | %s | %s | %s |" % (
            f["status"], f["rule_id"], f["title"].replace("|", "/"),
            f["jurisdiction"], f["binding_type"], f["severity"],
            f["detail"].replace("|", "/")))
    L.append("")
    L.append("## Sources and limitations per rule")
    L.append("")
    for f in doc["findings"]:
        L.append("### %s - %s" % (f["rule_id"], f["title"]))
        L.append("")
        L.append("- source: %s (%s)" % (f["source_reference"], f["source_date_or_version"]))
        L.append("- binding_type: `%s`; severity: `%s`; status: **%s**" %
                 (f["binding_type"], f["severity"], f["status"]))
        if f["required_evidence"]:
            L.append("- required evidence: %s" % ", ".join(f["required_evidence"]))
        if f["missing_evidence"]:
            L.append("- missing evidence: %s" % ", ".join(f["missing_evidence"]))
        L.append("- limitations: %s" % f["limitations"])
        L.append("")
    L.append("## Output schema (version %s) - for downstream evidence tooling" % doc["schema_version"])
    L.append("")
    L.append("`generated/compliance_findings.json`: schema_version, tool, disclaimer,")
    L.append("coverage_note, stale_note, profile_name, profile_ref, rulepacks[]")
    L.append("(rulepack, rulepack_version, rules), summary{" + ", ".join(STATUS_ORDER) + "},")
    L.append("findings[] with per-rule fields (rule_id, title, jurisdiction, authority,")
    L.append("source_reference, source_date_or_version, binding_type, severity, status,")
    L.append("detail, required_evidence, missing_evidence, limitations).")
    L.append("`evidence/compliance_matrix.csv` columns: " + ", ".join(CSV_COLUMNS) + ".")
    L.append("The CSV begins with one `#` comment line (the disclaimer); parsers skip it.")
    L.append("Stable like the WP5/WP6 schemas: do not change column meanings without")
    L.append("bumping schema_version.")
    L.append("")
    L.append("*%s*" % doc["disclaimer"])
    L.append("")
    return "\n".join(L)


def write_csv(doc, path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        # One leading comment line (downstream parsers skip lines starting
        # with '#'; documented in the matrix md schema section).
        f.write("# %s Research-grade precheck output (schema %s); see "
                "compliance_matrix.md for context.\n"
                % (doc["disclaimer"], doc["schema_version"]))
        w = csv.writer(f, lineterminator="\n")
        w.writerow(CSV_COLUMNS)
        for fd in doc["findings"]:
            w.writerow([
                doc["schema_version"], fd["rule_id"], fd["title"],
                fd["jurisdiction"], fd["authority"], fd["binding_type"],
                fd["severity"], fd["status"], fd["detail"],
                fd["source_reference"], fd["source_date_or_version"],
                ";".join(fd["required_evidence"]),
                ";".join(fd["missing_evidence"]), fd["limitations"],
            ])


def repo_root():
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "..", ".."))


def main(argv=None):
    root = repo_root()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--findings",
                    default=os.path.join(root, "generated", "compliance_findings.json"))
    ap.add_argument("--out-md",
                    default=os.path.join(root, "evidence", "compliance_matrix.md"))
    ap.add_argument("--out-csv",
                    default=os.path.join(root, "evidence", "compliance_matrix.csv"))
    args = ap.parse_args(argv)

    with open(args.findings, encoding="utf-8") as f:
        doc = json.load(f)

    os.makedirs(os.path.dirname(args.out_md) or ".", exist_ok=True)
    with open(args.out_md, "w", encoding="utf-8", newline="\n") as f:
        f.write(render_md(doc))
    write_csv(doc, args.out_csv)

    print("compliance matrix: %s" % doc["disclaimer"])
    print("wrote %s and %s" % (
        os.path.relpath(args.out_md, os.getcwd()).replace(os.sep, "/"),
        os.path.relpath(args.out_csv, os.getcwd()).replace(os.sep, "/")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
