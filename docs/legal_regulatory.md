# Legal and regulatory precheck

**This is not legal advice.** Nothing in this file, in the WP8 Compliance
Matrix Generator, or anywhere else in this repository is a legal conformity
determination. It is the full detail behind the README's compliance-related
lines: the WP8 precheck's scope and mechanism, the D9/D12 framing decisions,
the rulepack staleness caveat, and the honest jurisdiction-coverage gaps.
Consent-gate application to a specific mission class (the Class-C
controlled-reentry comparison) is [docs/target_selection.md](target_selection.md);
the peaceful-use disclaimer is the README's License section.

## Framing: D9 (operator-legality) and D12 (gate, not multiplier)

- **D9 — Operator-legality framing.** The package assumes the operator is,
  or is contracted/consented by, the launching state of the target. Nothing
  in this repository assumes or enables unconsented approach to another
  state's object; this framing is the targeting rationale, not merely a
  constraint bolted on afterward.
- **D12 — Legal accessibility is a gate and metadata flags, never a
  multiplier in the FoM.** The compliance engine (WP8) already blocks
  unconsented ADR; target prioritization ([docs/target_selection.md](target_selection.md))
  consumes its PASS/BLOCK output and flags. Rationale: open trade T5 already
  shows metric-choice sensitivity in the cost/FoM model
  ([docs/cost_model.md](cost_model.md)); folding a subjective legal weight
  into a product score would multiply arbitrariness and damage credibility.

## WP8 precheck: scope and mechanism

The Compliance Matrix Generator (`tools/compliance/`) is a Python 3
standard-library-only checker (no `jsonschema` — validation is hand-written
and dual-maintained with the documented `mission_profile.schema.json`) that
evaluates a declared mission profile against **versioned rulepacks**: UN
treaties, US FCC/FAA/NOAA, ITU, ESA reference, and ADSC internal policy —
**primary sources only**. It emits `generated/compliance_findings.json` and
`evidence/compliance_matrix.{md,csv}` (schema 1.0, deterministic, no
timestamps). Every finding separates **binding law / non-binding guideline /
agency guidance / ADSC policy / placeholder research assumption** (see the
binding-type legend in `evidence/compliance_matrix.md`) — these categories
are never conflated. Unit-tested (`tools/compliance/test_compliance.py`,
ctest `compliance`).

**Policy, both directions:** *unknown is safer than a false PASS* — missing
inputs are UNKNOWN or fail, never PASS; missing mandatory evidence yields
WARN/BLOCK. Unconsented active interference with a registered object is
**BLOCKed** as ADSC policy (`ADSC-POL-01`), test-enforced — and any
live-ephemeris or target-specific-product profile is also BLOCKed (D11
guardrail, `ADSC-POL-02`). The gate genuinely works both ways: it is not only
a permission check, it is also a scope guardrail that blocks the package's
own dual-use risk.

The precheck also cross-checks the WP5 campaign's compliance metadata
columns against the declared profile (`ADSC-META-01`) — a consistency check
between two research artifacts, making no statement about the real world.

## Compliance summary (current committed profile)

For the committed research profile
(`tools/compliance/examples/adsc_research_profile.json`): **PASS=6, INFO=2,
WARN=1, BLOCK=0, UNKNOWN=0, NOT_APPLICABLE=9** — see
[evidence/adsc_evidence_pack.md](../evidence/adsc_evidence_pack.md) section 7
and [evidence/compliance_matrix.md](../evidence/compliance_matrix.md) for the
full per-rule findings table (never re-derived here, R16). The research-only,
class-level profile is not blocked merely for being an ADR concept; the one
WARN is the honest export-control-review-not-started flag (`ADSC-EXP-01`).
Consent in the research profile is a **declared scenario assumption** (D9),
never a legal fact — the finding notes must be read that way.

## Rulepack staleness caveat

Rulepacks are **versioned research artifacts that can go stale** as
regulations change. Every rule carries a `source_date_or_version` and a
`limitations` field; low-confidence rules are downgraded to
`binding_type: placeholder`. Regenerate and re-verify against current law
before any real-world use — this is stated in the compliance matrix itself
and repeated here because it is easy to forget once a precheck has PASSed
once.

Rule dates currently carried (citations, not simulation results — not
subject to the numeric-literal-outside-include-block scan): the **IADC
25-year** post-mission-disposal guideline (IADC-02-01 Rev. 2, March 2020;
later revisions exist — confirm the current one before citing); the **US FCC
5-year** rule for LEO space stations (FCC 22-74, adopted 29 September 2022);
the **ESA** debris-mitigation reference (ESSB-ST-U-007 Issue 1, 30 October
2023, contractual on ESA-procured missions, cited as a reference here, not a
legal requirement on third parties). The 25-year IADC line is the one this
project's own decay trades report ([docs/target_selection.md](target_selection.md));
under an FCC-jurisdiction scenario the operative standard is stricter (5
years), which makes the sail-only negative for catalog A strictly harder,
not easier.

## Jurisdiction coverage gaps (honest, partial)

Coverage today is **UN treaties + US agency rules (FCC/FAA/NOAA) + ITU
references + ESA reference guidelines only**. **Russian, Chinese, and
Japanese national law — all adopter targets for this package — are not yet
covered.** This is future work, stated plainly rather than implied to be
complete. Every rule's own `limitations` field additionally flags where the
precheck only verifies that evidence was *declared*, not that it is valid or
sufficient (e.g. Outer Space Treaty Art. VI: "verifies that authorization
evidence is declared, not that a valid authorization exists or covers this
activity").

## What this precheck does not claim

- **Never a legal conformity determination.** It reports evidence gaps
  against versioned research-grade rulepacks; the compliance matrix says so
  in its own header, and this document repeats it because it is the single
  most important honesty line in the package.
- No jurisdiction beyond UN + US + ITU + ESA reference today (above).
- No export-control clearance — `ADSC-EXP-01` is an open WARN, not a PASS,
  and open-literature methods (D11) are the current basis for research
  distribution, not a substitute for a completed review.
- No claim that the WP5 campaign's compliance metadata says anything about a
  real mission — it is a consistency check between two research artifacts
  (`ADSC-META-01`).
