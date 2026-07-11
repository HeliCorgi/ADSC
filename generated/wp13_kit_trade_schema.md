# WP13 kit-class trade CSV schema (version 1.0)

`generated/wp13_kit_trade.csv` is the committed per-class (A/B/C) kit-trade
table emitted by `kit_trade` (src/main_kit_trade.cpp). It combines the
EXISTING WP3 sail-decay model (src/decay.cpp: sail_decay_years,
area_for_target_years) with the NEW WP13 EDT-v1 aligned-dipole physics core
(src/decay.cpp: edt_deorbit_years) into one deterministic, timestamp-free
table (R6): sail area, EDT deorbit-time band, EDT diagnostics, kit mass
(sail and EDT), EDT deployment risk, and a generated (never hand-written)
recommended-kit row with a one-line rationale per class. Class-C
controlled-reentry mission-class detail lives in the separate
`wp13_classC.{csv,md}` pair, not here (see that file's own column notes).

## Columns

| column | meaning |
|---|---|
| schema_version | WP13 kit-trade schema id (`1.0`) |
| catalog | class-level preset name (`DebrisCatalog::name`) |
| mass_kg | catalog stage mass [kg] |
| altitude_km | catalog altitude [km] |
| inclination_deg | catalog inclination [deg] (drives the mandatory EDT eta(i) band) |
| record_type | see record_type rows below |
| kit_option | sail / edt, or the recommendation label (recommended_kit only) |
| value | single-value metric, or 0.0 for band-only / not-applicable rows |
| value_lo | band lower edge (see per-row meaning below) |
| value_hi | band upper edge (see per-row meaning below) |
| units | value / value_lo / value_hi units |
| notes | provenance, model-scope tag, and caveats |

## record_type rows

- `sail_area_25yr`: sail area for the 25-yr IADC guideline from the
  EXISTING WP3 model (`area_for_target_years`, unchanged). value_lo/hi are
  the solar-max/solar-min band (mirrors `wp3_decay_trade.csv`'s
  `area_for_25yr` row; this does not repeat the full area sweep -- see
  that file for the sweep, now including the 1000 m^2 row).
- `edt_years`: EDT-v1 deorbit-time band from `edt_deorbit_years`.
  value_lo = years_optimistic (eta_hi = |cos i|, power/current-capped
  edge); value_hi = years_conservative (eta_lo = cos(i)^2,
  EMF/collection-limited edge) -- NEVER a point value. notes carries the
  EDT-v1 model-scope tag (below). Exactly-polar catalogs (|cos i| ~= 0:
  the aligned-dipole orbit-normal field vanishes) print the literal
  text `n/a (polar: aligned-dipole avg force -> 0)` in value_lo/value_hi
  instead of a number: this is the honest, physically-correct null result
  for that idealization (see the `edt_deorbit_years` doc comment in
  decay.hpp), not a missing-data placeholder. Retrograde catalogs
  (i > 90 deg) get FINITE bands: the passive tether's induced current
  always drags (Lenz), so the force magnitude scales with |cos i|.
- `edt_eta`: the mandatory inclination-dependent v x B efficiency band
  (spec:240-243, never omitted). value_lo = eta_lo = cos(i)^2, value_hi =
  eta_hi = |cos i| (see `wp13-edt-derivation.md` Section 5 for
  both current-limit regimes; for i > 90 deg the EMF polarity reverses
  and the magnitude is reported, with a note in the row).
- `edt_emf_power`: value = open-circuit motional EMF at the catalog
  altitude [V]; value_hi = diagnostic power = emf_v * avg_current_a [W];
  value_lo is unused (0.0). Diagnostic only, not a deorbit-time metric.
- `kit_mass`: installed kit mass. kit_option = sail uses the EXISTING
  `Config::kit_mass_kg`; kit_option = edt uses the NEW
  `EdtConfig::kit_mass_kg` (PLACEHOLDER). value = value_lo = value_hi (a
  point value, not a band).
- `deploy_risk`: `EdtConfig::deploy_failure_prob`, PLACEHOLDER. Reported
  only -- NOT applied to the edt_years band above (see the
  `edt_deorbit_years` doc comment in decay.hpp).
- `recommended_kit`: value = 0.0 (a label row, not a numeric metric);
  kit_option carries the recommendation label; notes carries the
  one-line, generated (never hand-written) rationale, quoting the actual
  computed sail-area and/or EDT-years numbers for that catalog. Class C's
  recommendation is `controlled-reentry-mission-class`, pointing at
  `wp13_classC.md` rather than a sail/EDT kit choice.

## EDT-v1 model scope (R14 fidelity tag)

Every `edt_years` row's notes column carries the literal tag
`[model: EDT-v1 aligned-dipole, capped-current; eta band cos(i)..cos^2(i);
libration T7 OPEN - PLACEHOLDER duty factor]`. This states the model scope
in-line with the number it qualifies (R14): aligned (untilted) dipole
geomagnetic field (SPENVIS centred dipole, IGRF epoch 2000); force capped
by either a fixed/power-limited current (optimistic edge) or an
EMF/collection-limited current (conservative edge); the along-track
efficiency factor is the orbit-averaged eta(i) in [cos(i)^2, cos(i)],
MANDATORY per spec WP13 (never a point value, never omitted for a
high-inclination catalog). Libration/dynamic stability (T7, Pelaez et al.
2000) is explicitly UNRESOLVED and is folded in only as a flat
PLACEHOLDER duty-cycle knob (`EdtConfig::eta_libration`) -- never claimed
solved. Plasma electron density is a cited PLACEHOLDER parameter, not an
IRI implementation (spec non-goal); see
`wp13-literature.md` Topic 3 for the citation and the open
solar-min/max pair gap.

## Schema changes (R15)

Additive columns to this file bump the minor version (1.0 -> 1.1) with
this file and all consumers updated in the same PR, the same rule as
`wp3_decay_trade_schema.md`'s own closing paragraph. Additive rows (a new
catalog, a new record_type) do NOT need a version bump under this
project's own rule.
