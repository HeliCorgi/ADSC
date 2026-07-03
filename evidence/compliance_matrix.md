# ADSC Compliance Matrix (Regulatory Precheck)

**This is not legal advice.** This matrix is a research-grade PRECHECK of a declared
mission profile against versioned rulepacks: it reports which prechecks pass,
which evidence is missing and what is unknown. It is not a legal conformity
determination and must not be used as one.

- Profile: **ADSC reference research campaign (numerical simulation only)** (`tools/compliance/examples/adsc_research_profile.json`)
- Findings schema_version: `1.0`
- Coverage (honest): Jurisdiction coverage is currently limited to the UN space treaties, US agency rules (FCC/FAA/NOAA), ITU references and ESA reference guidelines. National law of other launching states relevant to ADSC's adopter framing - notably Russia, China and Japan - is NOT yet covered and is future work.
- Staleness: Rulepacks are versioned research artifacts (see source_date_or_version per rule). Regulations change; rulepacks can go stale and must be re-verified against current law before any real-world use.
- Policy: unknown or missing inputs are never treated as PASS; missing
  mandatory evidence yields WARN or BLOCK; unconsented active interference
  with a registered space object is BLOCKed as ADSC policy.

## Summary

| PASS | INFO | WARN | BLOCK | UNKNOWN | NOT_APPLICABLE |
|---:|---:|---:|---:|---:|---:|
| 6 | 2 | 1 | 0 | 0 | 9 |

Rulepacks: international 1.0 (5 rules), us_fcc 1.0 (2 rules), us_faa 1.0 (1 rules), us_noaa 1.0 (1 rules), itu 1.0 (1 rules), esa_reference 1.0 (2 rules), adsc_policy 1.0 (5 rules)

## Binding-type legend (never conflated)

- `binding` - legally binding requirement (treaty/statute/regulation) in its jurisdiction
- `guideline` - non-binding guideline or reference standard
- `agency_guidance` - agency guidance/reference process, not a general legal requirement
- `adsc_policy` - ADSC internal project policy - stricter than law by design, not law
- `placeholder` - simplified research-grade rule; content not confident - requires legal review

## Findings

| status | rule_id | title | jurisdiction | binding_type | severity | detail |
|---|---|---|---|---|---|---|
| PASS | SCHEMA-VAL-01 | Mission profile schema validation (stdlib manual validation) | ADSC internal | adsc_policy | warn | all required fields present with expected types |
| NOT_APPLICABLE | INTL-OST-06-01 | National authorization and continuing supervision of the mission | international | binding | blocking | rule does not apply to this profile |
| PASS | INTL-OST-08-01 | Registry-state jurisdiction and control: owner consent for active interference | international | binding | blocking | condition satisfied |
| NOT_APPLICABLE | INTL-REG-01 | Registration of the servicer space object | international | binding | warn | rule does not apply to this profile |
| NOT_APPLICABLE | INTL-LIAB-01 | Third-party liability risk allocation evidence | international | placeholder | warn | rule does not apply to this profile |
| PASS | IADC-ODM-25YR | Post-mission disposal lifetime within 25 years (IADC guideline) | international | guideline | warn | condition satisfied |
| NOT_APPLICABLE | FCC-ODM-5YR | Post-mission disposal within 5 years for LEO space stations (FCC-jurisdiction scenario) | US | binding | blocking | rule does not apply to this profile |
| NOT_APPLICABLE | FCC-ODM-DISCLOSE | Orbital debris mitigation disclosure with the FCC application | US | binding | warn | rule does not apply to this profile |
| NOT_APPLICABLE | FAA-LRL-01 | FAA launch and reentry authorization evidence (US commercial scenario) | US | binding | blocking | rule does not apply to this profile |
| NOT_APPLICABLE | NOAA-RS-01 | NOAA private remote-sensing system authorization evidence | US | placeholder | warn | rule does not apply to this profile |
| NOT_APPLICABLE | ITU-RF-01 | Frequency coordination / notification filing evidence for the RF payload | international | placeholder | warn | rule does not apply to this profile |
| NOT_APPLICABLE | ESA-SDM-01 | ESA space debris mitigation: LEO clearance reference (5-year class) | ESA (reference) | guideline | warn | rule does not apply to this profile |
| INFO | ESA-ZD-01 | Zero Debris alignment statement (voluntary charter) | ESA (reference) | guideline | info | No Zero Debris alignment statement is declared (informational: the charter is voluntary; an ADR evidence package benefits from stating its position).; evidence_documents: does not contain 'zero_debris_alignment_statement' |
| PASS | ADSC-POL-01 | No unconsented active interference with a registered space object (ADSC policy) | ADSC internal | adsc_policy | blocking | condition satisfied |
| PASS | ADSC-POL-02 | No live-ephemeris ingestion and no target-specific operational products (ADSC policy) | ADSC internal | adsc_policy | blocking | condition satisfied |
| PASS | ADSC-POL-03 | Non-weaponization: research-only or consented, class-level use (ADSC policy) | ADSC internal | adsc_policy | blocking | condition satisfied |
| WARN | ADSC-EXP-01 | Export-control review status (ITAR/EAR precheck placeholder) | ADSC internal | placeholder | warn | Export-control review is not complete. Open-literature methods (spec D11) are the current basis for research distribution, but a documented review is required before sharing controlled technical data or starting hardware/flight work. |
| INFO | ADSC-META-01 | WP5 campaign compliance metadata consistent with this mission profile | ADSC internal | adsc_policy | warn | WP5 campaign metadata consistent with research profile; all 7 mapped columns + 2 policy columns consistent across 1000 runs |

## Sources and limitations per rule

### SCHEMA-VAL-01 - Mission profile schema validation (stdlib manual validation)

- source: ADSC mission_profile.schema.json (adsc-mission-profile-1.0) (schema 1.0)
- binding_type: `adsc_policy`; severity: `warn`; status: **PASS**
- limitations: Manual validation dual-maintained with the schema document; unknown fields are ignored by every rule and can never produce a PASS.

### INTL-OST-06-01 - National authorization and continuing supervision of the mission

- source: Outer Space Treaty, Article VI (610 UNTS 205) (1967 (in force 10 October 1967))
- binding_type: `binding`; severity: `blocking`; status: **NOT_APPLICABLE**
- required evidence: national_authorization_evidence
- limitations: Precheck only: verifies that authorization evidence is DECLARED, not that a valid authorization exists or covers this activity. Which state is the 'appropriate State Party' for a multi-state mission requires legal review.

### INTL-OST-08-01 - Registry-state jurisdiction and control: owner consent for active interference

- source: Outer Space Treaty, Article VIII (610 UNTS 205) (1967 (in force 10 October 1967))
- binding_type: `binding`; severity: `blocking`; status: **PASS**
- limitations: The consent requirement is an INTERPRETATION derived from Art. VIII's allocation of jurisdiction and control - it is not express treaty text, and its precise legal basis requires legal review. For research profiles, consent 'true' is a declared scenario assumption (ADSC spec D9), never a legal fact, and the finding notes must be read that way. The consent instrument (agreement, contract, note verbale) required in a real case also needs legal review.

### INTL-REG-01 - Registration of the servicer space object

- source: Convention on Registration of Objects Launched into Outer Space, Article II (1023 UNTS 15) (1975 (in force 15 September 1976))
- binding_type: `binding`; severity: `warn`; status: **NOT_APPLICABLE**
- required evidence: registration_evidence
- limitations: Precheck of declared evidence only. Registration timing ('as soon as practicable') and the choice of registry state among joint launching states require legal review.

### INTL-LIAB-01 - Third-party liability risk allocation evidence

- source: Convention on International Liability for Damage Caused by Space Objects, Articles II-III (961 UNTS 187) (1972 (in force 1 September 1972))
- binding_type: `placeholder`; severity: `warn`; status: **NOT_APPLICABLE**
- limitations: Placeholder: the convention binds STATES, not operators - the operator-level insurance/indemnification check performed here is NOT imposed by the cited instrument itself but by national law, which varies by jurisdiction. binding_type is therefore 'placeholder' (conservatism policy); amounts and scope require legal review.

### IADC-ODM-25YR - Post-mission disposal lifetime within 25 years (IADC guideline)

- source: IADC Space Debris Mitigation Guidelines, IADC-02-01, guideline 5.3.2 (objects passing through the LEO protected region) (IADC-02-01 Rev. 2 (March 2020); later revisions exist - confirm the current one)
- binding_type: `guideline`; severity: `warn`; status: **PASS**
- limitations: Non-binding guideline. Stricter national rules supersede it where they apply - notably the US FCC 5-year rule (see FCC-ODM-5YR); under a US FCC-jurisdiction scenario the 25-year figure is NOT the operative standard. Confirm the current IADC revision before citing.

### FCC-ODM-5YR - Post-mission disposal within 5 years for LEO space stations (FCC-jurisdiction scenario)

- source: FCC, Second Report and Order, 'Mitigation of Orbital Debris in the New Space Age', FCC 22-74 (adopted 29 September 2022); 47 CFR Part 25 space-station disposal rules (FCC 22-74 (2022))
- binding_type: `binding`; severity: `blocking`; status: **NOT_APPLICABLE**
- required evidence: orbital_debris_mitigation_plan
- limitations: Simplified jurisdiction test (operating_state/launching_state == 'US'); the real trigger is an FCC space-station authorization, including US market access for non-US systems. Exact codified CFR subsections, transition periods and waiver practice require legal review.

### FCC-ODM-DISCLOSE - Orbital debris mitigation disclosure with the FCC application

- source: 47 CFR 25.114(d)(14) (orbital debris mitigation disclosure for space station applications) (as amended through FCC 22-74 (2022))
- binding_type: `binding`; severity: `warn`; status: **NOT_APPLICABLE**
- required evidence: orbital_debris_mitigation_plan
- limitations: Precheck of declared evidence only; the disclosure's technical content (collision risk, casualty risk, maneuverability, trackability) is not evaluated here. Requires legal review.

### FAA-LRL-01 - FAA launch and reentry authorization evidence (US commercial scenario)

- source: 51 U.S.C. chapter 509 (Commercial Space Launch Activities); 14 CFR Part 450 (Launch and Reentry License Requirements) (14 CFR Part 450 (streamlined rule, 2020))
- binding_type: `binding`; severity: `blocking`; status: **NOT_APPLICABLE**
- required evidence: faa_part450_application_evidence
- limitations: Precheck of declared evidence only. Whether the deorbit-kit-driven target reentry itself needs a Part 450 reentry authorization (vs. only the servicer launch) is an open legal question for ADR concepts - requires legal review.

### NOAA-RS-01 - NOAA private remote-sensing system authorization evidence

- source: 51 U.S.C. chapter 601 (Land Remote Sensing Policy); 15 CFR Part 960 (Licensing of Private Remote Sensing Space Systems) (15 CFR Part 960 (2020 revision))
- binding_type: `placeholder`; severity: `warn`; status: **NOT_APPLICABLE**
- required evidence: noaa_remote_sensing_authorization_evidence
- limitations: Placeholder rule: the US-nexus test (US person / platform control) and the 2020 tiering system are NOT modelled - the rule fires on any remote-sensing payload. Rendezvous inspection cameras may or may not fall under Part 960. Requires legal review.

### ITU-RF-01 - Frequency coordination / notification filing evidence for the RF payload

- source: ITU Radio Regulations, Article 9 (coordination) and Article 11 (notification and recording) (Radio Regulations, Edition of 2020; later editions exist - confirm the current one)
- binding_type: `placeholder`; severity: `warn`; status: **NOT_APPLICABLE**
- required evidence: itu_or_national_frequency_filing_evidence
- limitations: Placeholder rule: ITU obligations run through the responsible national administration, and band-specific coordination requirements are NOT modelled. The Radio Regulations bind administrations, not operators directly. Requires review by the responsible administration / legal counsel.

### ESA-SDM-01 - ESA space debris mitigation: LEO clearance reference (5-year class)

- source: ESA Space Debris Mitigation Requirements, ESSB-ST-U-007, Issue 1 (30 October 2023) (ESSB-ST-U-007 Issue 1 (2023))
- binding_type: `guideline`; severity: `warn`; status: **NOT_APPLICABLE**
- limitations: ESSB-ST-U-007 applies contractually to ESA-procured missions; it is cited here as a reference for where European practice is heading, not as a legal requirement on third parties. Clause-level applicability requires review.

### ESA-ZD-01 - Zero Debris alignment statement (voluntary charter)

- source: ESA Zero Debris Charter (2023) (Zero Debris Charter (2023))
- binding_type: `guideline`; severity: `info`; status: **INFO**
- limitations: Voluntary charter, not a legal requirement. Informational finding only.

### ADSC-POL-01 - No unconsented active interference with a registered space object (ADSC policy)

- source: ADSC specification v4.2, decisions D9 and D11 (adsc-specification-v4.md) (spec v4.2)
- binding_type: `adsc_policy`; severity: `blocking`; status: **PASS**
- limitations: Internal policy, stricter than a legal minimum by design. For research profiles the consent flag is a declared scenario assumption (D9).

### ADSC-POL-02 - No live-ephemeris ingestion and no target-specific operational products (ADSC policy)

- source: ADSC specification v4.2, decision D11 and hard rule R13 (adsc-specification-v4.md) (spec v4.2)
- binding_type: `adsc_policy`; severity: `blocking`; status: **PASS**
- limitations: Internal policy. Either flag alone fails the rule; a missing flag evaluates UNKNOWN, never PASS.

### ADSC-POL-03 - Non-weaponization: research-only or consented, class-level use (ADSC policy)

- source: ADSC specification v4.2, section 1 and decision D11; README peaceful-use disclaimer (spec v4.2)
- binding_type: `adsc_policy`; severity: `blocking`; status: **PASS**
- limitations: Proxy check: 'non-weaponization' is not directly observable from profile fields; the rule uses research-only / consent / no-targeting-products as the observable proxy. Internal policy, not law.

### ADSC-EXP-01 - Export-control review status (ITAR/EAR precheck placeholder)

- source: 22 CFR Parts 120-130 (ITAR); 15 CFR Parts 730-774 (EAR) (snapshot 2024-era regime structure; regimes change - re-verify)
- binding_type: `placeholder`; severity: `warn`; status: **WARN**
- limitations: Placeholder: jurisdiction-specific export regimes (US ITAR/EAR cited as the typical case) are not modelled in detail. Requires export-control counsel review.

### ADSC-META-01 - WP5 campaign compliance metadata consistent with this mission profile

- source: generated/wp5_campaign_runs.csv compliance columns (schema 1.0); ADSC specification v4.2, D9/D11 (WP5 runs CSV schema 1.0)
- binding_type: `adsc_policy`; severity: `warn`; status: **INFO**
- limitations: Consistency check between two research artifacts; it makes no statement about the real world. If the WP5 CSV is absent the result is UNKNOWN, never PASS.

## Output schema (version 1.0) - for downstream evidence tooling

`generated/compliance_findings.json`: schema_version, tool, disclaimer,
coverage_note, stale_note, profile_name, profile_ref, rulepacks[]
(rulepack, rulepack_version, rules), summary{PASS, INFO, WARN, BLOCK, UNKNOWN, NOT_APPLICABLE},
findings[] with per-rule fields (rule_id, title, jurisdiction, authority,
source_reference, source_date_or_version, binding_type, severity, status,
detail, required_evidence, missing_evidence, limitations).
`evidence/compliance_matrix.csv` columns: schema_version, rule_id, title, jurisdiction, authority, binding_type, severity, status, detail, source_reference, source_date_or_version, required_evidence, missing_evidence, limitations.
The CSV begins with one `#` comment line (the disclaimer); parsers skip it.
Stable like the WP5/WP6 schemas: do not change column meanings without
bumping schema_version.

*This is not legal advice.*
