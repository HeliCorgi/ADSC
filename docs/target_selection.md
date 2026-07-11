# Target selection: catalogs, kit-class trade, and prioritization

This is the full detail behind the README's kit-class-trade line. It covers
the three class-level target catalogs (A/B/C) and their citations, the WP13
sail/EDT kit-class trade with the EDT-v1 model scope and the T7 open-risk
honesty, the Class-C controlled-reentry comparison, and a pointer to the WP14
target-prioritization table. Cost figures referenced here are owned by
[docs/cost_model.md](cost_model.md); safety/campaign figures are owned by
[docs/safety.md](safety.md); regulatory-consent mechanics are
[docs/legal_regulatory.md](legal_regulatory.md).

Targets remain **class-level parameters** throughout (D2, D11): no
mission-planning product is built against a specific catalog ID, and no live
ephemeris/TLE is ingested anywhere in this repository.

## Catalogs A/B/C

| catalog | class | mass | altitude | inclination | role |
|---|---|---:|---:|---:|---|
| A — SL-16 / Zenit-2 second stage | massive rocket body | ≈9,000 kg (8,300–9,000 kg, source spread) | ≈840–857 km | 71.0° | sail/EDT kit-trade anchor; also McKnight top-of-ranking |
| B — SL-8 / Kosmos-3M second stage | medium rocket body | ≈1,400 kg (1,435 kg empty) | ≈700–1,000 km (three-box cluster) | 74°/83° | sail closes; high per-class population |
| C — Envisat-class massive SSO payload | massive defunct payload | 8,211 kg | ≈765–800 km, sun-synchronous | 98.4–98.55° | separate controlled-reentry mission class, not kit-ranked |

Citations (`_tasks_local/wp13-literature.md`, web-verified 2026-07-11):

- **Catalog A (Zenit-2 / SL-16):** empty mass ≈8,300 kg (one source lists
  ≈9,000 kg — carry both, the 8,300 kg figure is the more common one), length
  11.50 m, diameter 3.90 m, 1× RD-120 engine. Typical Tselina-2 payload orbit
  ≈845×857 km, inclination 71.01° (also a 65° family; the wider SL-16
  rocket-body cluster spans ≈750–850 km, inclinations including 71° and 81°).
  Sources: braeunig.us/space/specs/zenit.htm, astronautix.com/z/zenit-2.html,
  Wikipedia "Zenit-2". Cross-check: McKnight et al. 2021 (below) — the top 20
  most-concerning LEO objects are all SL-16 second stages, ≈9 t class,
  ≈850 km.
- **Catalog B (Kosmos-3M / SL-8):** empty mass ≈1,435 kg, gross mass
  ≈20,135 kg, 1× 11D49 engine, length ≈6 m, diameter 2.4 m. Cluster survey
  (as of 2014): Box 3 ≈700–800 km (50 rocket bodies), Box 2 ≈900–1,000 km
  (143 rocket bodies), Box 1 ≈1,500–1,600 km (44 rocket bodies), inclinations
  ≈74° and ≈83° — an order of magnitude larger combined population than the
  SL-16 top-20 concern list. Sources: astronautix.com/k/kosmos11k65m.html,
  space.skyrocket.de/doc_lau_det/kosmos-3m.htm. Appears in SL-8-specific ADR
  mission studies (e.g. ADReS-A, Springer 10.1007/978-3-319-15982-9_3).
- **Catalog C (Envisat-class):** mass 8,211 kg (Service Module 2,673 + PEB
  1,021 + Payload Carrier 2,078 + Fuel 319 + Instruments 2,118 kg),
  sun-synchronous orbit ≈765–800 km, inclination ≈98.4–98.55°, defunct since
  April 2012 — the archetypal "too massive / high-casualty-risk-on-
  uncontrolled-reentry" object cited across the debris-mitigation literature.
  Sources: eoPortal (eoportal.org/satellite-missions/envisat), ESA Earth
  Online, Wikipedia "Envisat".

**McKnight basis (criticality anchor):** McKnight, D., Witner, R., Letizia,
F., Lemmens, S., Anselmo, L., Pardini, C., Rossi, A., Kunstadter, C., et al.
(2021), "Identifying the 50 statistically-most-concerning derelict objects in
LEO," *Acta Astronautica* 181, 282-291, DOI: 10.1016/j.actaastro.2021.01.021.
Eleven expert teams' consensus top-50 most-concerning LEO objects is
dominated by rocket bodies; the top ≈20 are **all** SL-16 (Zenit-2) second
stages (average mass of the top-50 objects ≈5,295 kg); removing the top-50
would roughly halve LEO collision risk (top-10 ≈30% reduction). This is the
cited anchor for catalog A's criticality weight in the WP6/WP14 FoM — see
[docs/cost_model.md](cost_model.md). The rest of the criticality weighting
table (the non-peak altitude/weight pairs) remains an uncited interpolation
(**open trade T5**, [docs/cost_model.md](cost_model.md)).

## Sail-decay trade (WP3)

A quasi-circular drag-decay integrator over a Vallado exponential atmosphere
(Vallado, *Fundamentals of Astrodynamics and Applications*, 4th ed., Table
8-4; cross-checked against its constant-density closed form) produces the
sail-area × altitude × solar-activity → decay-years trade, always as a
solar-min..max **range** (open trade T4 — solar-activity dispersion, never a
point value).

<!-- DOCS-WP3-DECAY-START (source: generated/wp3_decay_trade.csv, schema 1.0) -->
**Catalog A — SL-16 / Zenit-2, ≈9,000 kg, 840 km:**

| sail area [m²] | decay time, solar max [yr] | decay time, solar min [yr] |
|---:|---:|---:|
| 5 | 673.5 | 10775.2 |
| 10 | 336.7 | 5387.6 |
| 25 | 134.7 | 2155.0 |
| 50 | 67.3 | 1077.5 |
| 100 | 33.7 | 538.8 |
| 200 | 16.8 | 269.4 |
| 400 | 8.4 | 134.7 |
| 800 | 4.2 | 67.3 |
| 1000 | 3.4 | 53.9 |
| 1600 | 2.1 | 33.7 |
| 3200 | 1.1 | 16.8 |

Sail area for the 25-year guideline (solar max..min): **135..2155 m²** —
impractical at the upper end.

**Catalog B — SL-8 / Kosmos-3M, ≈1,400 kg, 750 km:**

| sail area [m²] | decay time, solar max [yr] | decay time, solar min [yr] |
|---:|---:|---:|
| 5 | 36.0 | 576.4 |
| 10 | 18.0 | 288.2 |
| 25 | 7.2 | 115.3 |
| 50 | 3.6 | 57.6 |
| 100 | 1.8 | 28.8 |
| 200 | 0.9 | 14.4 |
| 400 | 0.5 | 7.2 |
| 800 | 0.2 | 3.6 |
| 1000 | 0.2 | 2.9 |
| 1600 | 0.1 | 1.8 |
| 3200 | 0.1 | 0.9 |

Sail area for the 25-year guideline (solar max..min): **7..115 m²** — tens of
square meters, a practical kit.

25-year figure is the **IADC guideline** (IADC-02-01, Space Debris Mitigation
Guidelines, guideline 5.3.2). Under a US FCC license the current rule (FCC
22-74, adopted 2022) requires **5-year** post-mission disposal for LEO space
stations instead — the class-A sail-only negative gets strictly harder under
that standard. Both rules are carried separately in the WP8 compliance
rulepacks (see [docs/legal_regulatory.md](legal_regulatory.md)); this trade
keeps the IADC 25-year reference line.
<!-- DOCS-WP3-DECAY-END -->

Sail-only closes for the lighter/lower catalog B (tens of m²) but **not** for
the heavier, higher catalog A (hundreds-to-thousands of m² — impractical):
the honest negative that brackets **open trade T1** (sail vs EDT crossover
altitude) and motivates the WP13 electrodynamic-tether branch below.

## Kit-class trade + EDT physics (WP13)

Data source: `generated/wp13_kit_trade.csv` (schema 1.0, column definitions
in `generated/wp13_kit_trade_schema.md`), combining the existing WP3
sail-decay model with the WP13 EDT-v1 aligned-dipole physics core
(`src/decay.cpp: edt_deorbit_years`) — no hand-written numbers. The sail-area
25-year figures below are the **same numbers** as the WP3 sweep above (both
CSVs derive from the same `area_for_target_years` model); this file cites
`wp13_kit_trade.csv` as the canonical per-class source to avoid an ambiguous
"which CSV" question (per the WP15 content map).

<!-- DOCS-WP13-KIT-START (source: generated/wp13_kit_trade.csv, schema 1.0; copied from evidence pack section 4) -->
| catalog | recommended kit | sail area, 25-yr [m²] (max..min) | EDT deorbit time [yr] | EDT η(i) band |
|---|---|---:|---:|---:|
| A — SL-16 / Zenit-2 second stage | edt-candidate-open-risks | 135..2155 | 3.0..9.2 | 0.106..0.326 |
| B — SL-8 / Kosmos-3M second stage | sail | 7..115 | 0.6..3.0 | 0.043..0.208 |
| C — Envisat-class massive SSO payload | controlled-reentry-mission-class | 61..983 | 5.5..37.5 | 0.021..0.146 |

Kit mass, deploy risk, and diagnostic EMF/power (same source CSV):

| catalog | sail kit mass [kg] | EDT kit mass [kg] (PLACEHOLDER) | EDT deploy-failure risk (PLACEHOLDER) | EDT open-circuit EMF [V] |
|---|---:|---:|---:|---:|
| A | 2.4 | 20.0 | 0.05 | 150.3 |
| B | 2.4 | 20.0 | 0.05 | 100.3 |
| C | 2.4 | 20.0 | 0.05 | 69.4 |

Deploy risk is quoted for the record and is **not** folded into the
EDT-years band above.
<!-- DOCS-WP13-KIT-END -->

**EDT-v1 model scope (R14 fidelity tag), quoted verbatim from the committed
CSV — never hand-transcribed, so this document and the data cannot drift
apart:**

> [model: EDT-v1 aligned-dipole, capped-current; eta band |cos i|..cos^2 i; libration T7 OPEN - PLACEHOLDER duty factor]

This states the model scope in-line with the number it qualifies (R14):
aligned (untilted) dipole geomagnetic field (SPENVIS centred dipole, IGRF
epoch 2000); force capped by either a fixed/power-limited current
(optimistic edge) or an EMF/collection-limited current (conservative edge);
the along-track efficiency factor η(i) is mandatory and never a point value
(spec WP13) — at catalog A's 71° inclination the efficiency loss is
first-order, and omitting it would overstate EDT performance.

**T7-open honesty (never claim closed).** Libration / dynamic tether
stability is explicitly **unresolved** (Peláez, J., et al., 2000, cited in
`_tasks_local/wp13-literature.md`); the EDT-years band above folds it in only
as a flat PLACEHOLDER duty-cycle knob (`EdtConfig::eta_libration`), never
claimed solved. Plasma electron density is a cited PLACEHOLDER solar-min/max
pair, not an IRI implementation (spec non-goal). Bottom line, quoted verbatim
from `generated/wp13_kit_trade.csv`'s own recommended-kit rationale for
catalog A: "sail infeasible (135..2155 m2 for the 25-yr guideline, solar
max..min); EDT (band
3.0..9.2 yr) subject to T7; libration (T7, Pelaez et al. 2000) and plasma Ne
(PLACEHOLDER solar-min/max pair) remain open — an honest EDT candidate, not a
closed recommendation."

## Class C: controlled reentry (a separate mission class)

Class C (spec WP13) is **not** a sail/EDT kit choice: massive/high-risk
derelicts where the deliverable is a controlled reentry, evaluated on
casualty risk, ground footprint, Δv, consent, and cost. Comparison against
tug architectures is allowed **here only**, and only on the **same target
set** (D1, spec WP13/WP14 rule — comparing different targets is meaningless
and banned). Data source: `generated/wp13_classC.csv` (schema 1.0),
regenerated by `kit_trade` — no hand-written numbers. This is **open trade
T2** (controlled vs uncontrolled reentry, driven by demisability).

**Uncontrolled-reentry casualty framing (cited, not computed):**
NASA-STD-8719.14A (with Change 1), Requirement 4.7-1: the expected
worldwide human-casualty risk from reentering debris shall not exceed
**Ec < 1×10⁻⁴** (1 in 10,000); casualty is assumed for any surviving fragment
with impact kinetic energy **> 15 J**; a controlled reentry must additionally
ensure no surviving >15 J fragment lands within **370 km** of foreign
landmasses (soma.larc.nasa.gov/SIMPLEx/pdf_files/871914.pdf; ESA applies the
same 1×10⁻⁴ limit via DRAMA/SARA, technology.esa.int/page/re-entry-safety).
This repository does **not** compute a per-object Ec for either candidate —
that needs a survivability/ground-footprint tool (NASA DAS/ORSAT or ESA SARA
class), out of scope here (`[CITATION NEEDED — PLACEHOLDER: per-object Ec
analysis]`). Both candidates are multi-tonne, so an uncontrolled reentry is
the presumptive risk driver the 370 km / Ec<1e-4 rule exists for — that
presumption is not itself a computed Ec value.

<!-- DOCS-WP13-CLASSC-START (source: generated/wp13_classC.{csv,md}, schema 1.0; copied verbatim) -->
Controlled-deorbit Δv (COMPUTED): a single impulsive perigee-lowering burn
from the catalog's circular orbit (radius r_a) to a target perigee r_p =
6371+40 km (mean-Earth-radius convention), dv = v_c·(1 − √(2·r_p/(r_a+r_p))),
v_c = √(μ/r_a). This is the minimum single-burn Δv to commit the stage to
reentry; it excludes targeting/footprint-control burns and any margin.

| candidate | v_c [m/s] | Δv [m/s] |
|---|---:|---:|
| Zenit-2 / SL-16 (catalog A) | 7431.2 | **223.4** |
| Envisat-class (catalog C) | 7462.2 | **208.3** |
<!-- DOCS-WP13-CLASSC-END -->

**Consent gate (D12).** Legal accessibility enters as a **gate and metadata
flags, never a multiplier**: the WP8 compliance engine BLOCKs unconsented
active debris removal, and target prioritization consumes its PASS/BLOCK
output and flags rather than folding a subjective legal weight into any
score. Any controlled-reentry mission on either candidate above is subject
to that same gate, regardless of the casualty-risk or Δv numbers above. See
[docs/legal_regulatory.md](legal_regulatory.md).

**Kit-installer vs tug, same target set.** A tug architecture also amortizes
fixed costs across a batch of targets in one plane, exactly like the
kit-installer's own batch amortization ([docs/cost_model.md](cost_model.md)).
What does *not* amortize for a tug, and is the installer's structural edge
(D1: installer-not-tug): (1) a tug must carry propellant to change **each**
target's orbit itself, so its per-target Δv scales with target count, while
an installer's kit-carrying stages deorbit themselves after release; (2) a
tug must detumble, dock/grapple, and physically tow each multi-tonne stage
through its own controlled-reentry burn, repeating the highest-risk contact
event per target, while the installer's own contact event is bounded by the
WP3 low-energy capture approach (0.333 J budget at 0.15 m/s, from
`generated/reference_metrics.csv`, owned by [docs/safety.md](safety.md)) and
does not itself carry a target through
reentry. Both architectures still need some per-target consent/compliance
gate and some reentry Δv paid by someone — the installer's edge is *where*
that Δv and that repeated high-risk contact event are paid, not that they
vanish.

**Cost:** catalog A has itemized low/mid/high ranges via the WP14 cost model
([docs/cost_model.md](cost_model.md)); catalog C's controlled-reentry mission
is not separately itemized (the WP14 ranges are servicer-campaign-scale) —
an open item, stated rather than silently assumed.

## Target prioritization (WP14)

The generated, joined ranking over cost + FoM + kit trade — no new
simulation, no new statistics, every number read directly from
`generated/wp6_cost_summary.csv`, `generated/wp13_kit_trade.csv`, and
`generated/wp5_campaign_summary.csv` — is
[generated/wp14_prioritization.md](../generated/wp14_prioritization.md).
Summary of its ranking:

1. **Catalog A (SL-16)** — mass-dominant FoM under both weightings; the
   McKnight top-of-ranking class; recommended kit is the honest
   EDT-candidate above, not a closed recommendation.
2. **Catalog B (SL-8)** — closes today with a cheap, practical sail kit; a
   much larger per-class population than catalog A's top-20 concern list;
   not a differentiator on cost per removal (within ≈0.3% of catalog A in
   relative CU).
3. **Catalog C (Envisat-class)** — a separate controlled-reentry mission
   class, **not kit-ranked** against A or B (no WP5/WP6 campaign rows exist
   for it).

Legal/consent gate (D12): no catalog above is ranked up or down on
legal/consent grounds anywhere in this document — consent is a gate, not a
scoring input. Full per-class defenses, one paragraph each, with the
McKnight/ADReS-A/Envisat citations quoted in full:
[generated/wp14_prioritization.md](../generated/wp14_prioritization.md).

## What this trade does not claim

- No claim that EDT libration risk (T7) is resolved, or that the plasma
  density used is a validated model (it is a cited PLACEHOLDER pair).
- No per-object casualty-risk (Ec) computation for Class C — that is a
  documented gap, not a silent omission.
- No absolute cost figure — cost figures are relative CU with cited
  low/mid/high MUSD ranges only, detailed in
  [docs/cost_model.md](cost_model.md).
- No legal or regulatory determination — see
  [docs/legal_regulatory.md](legal_regulatory.md).
