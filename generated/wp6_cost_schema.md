# WP6/WP14 cost/FoM CSV schema (version 1.1)

`generated/wp6_cost_summary.csv` is a long-format table consumed downstream.
It consumes the WP5 campaign (schema 1.0): the amortization sweep re-runs the
WP5 engine across the kit-inventory sweep at the fixed master seed, and the
baseline row (n_kits = CampaignConfig::kits_initial) reproduces
`generated/wp5_campaign_runs.csv` exactly. Primary units are RELATIVE cost
units (CU); **no point-value currency is emitted** (R6/D10) -- absolute costs
appear ONLY as cited low/mid/high ranges (WP14).

Schema 1.1 (WP14) is an ADDITIVE bump over 1.0 (WP6): one new trailing
column (`cost_scenario`), four new `record_type` values (cost_component_musd
/ currency_anchor_derived / campaign_cost_musd / cost_per_removal_musd), and
two new `metric` values under the existing `fom` record_type
(cost_per_risk_equiv_mass / cost_per_risk_equiv_mass_musd). Every row that
existed under schema 1.0 is byte-identical in VALUE under 1.1; the only
change to those rows is the appended blank `cost_scenario` field.

## Columns

| column | meaning |
|---|---|
| schema_version | schema id (`1.1`) |
| catalog | class-level preset name (blank for global rows) |
| record_type | amortization / fom / tornado / cost_component / currency_anchor / cost_component_musd / currency_anchor_derived / campaign_cost_musd / cost_per_removal_musd |
| n_kits | kits carried (amortization/cost_component rows) or the baseline N
  the WP14 derived rows were computed at; blank otherwise |
| param | cost parameter (tornado / cost_component rows) or the WP14 item id
  (cost_component_musd rows); blank otherwise |
| weighting | spatial_density / criticality (fom rows); blank otherwise |
| metric | the quantity named in the row |
| estimate | point value (p50 for distributions) |
| p05 / p50 / p95 | percentiles (distribution rows; 0 when not applicable) |
| units | CU_per_removal / CU / count / ratio / kg / kg_per_CU / CU_per_kg /
  normalized / musd_per_cu / MUSD / MUSD_per_removal / MUSD_per_kg / MUSD_per_yr
  / MUSD_per_unit / percent-valued units carried in the row's own `units`
  string (e.g. "pct of cost") |
| notes | provenance / PLACEHOLDER caveats / WP14 arithmetic strings |
| cost_scenario | low / mid / high (WP14 rows only); blank for pre-WP14 rows
  and for rows (e.g. fom / cost_per_risk_equiv_mass in CU) that are not
  currency-scenario-dependent |

## record_type rows (schema 1.0, unchanged)

- **amortization** (per catalog, per swept N): `cost_per_removal` [CU/removal,
  p05/p50/p95], `cost_per_removal_ratio_to_n1` [ratio], `removals` [count p50],
  `campaign_cost` [CU p50]. The cost/removal vs N curve is the core installer
  amortization argument: it falls until the Δv budget (not the kit count)
  bounds removals, then turns up as extra carried kits add cost without
  removals.
- **fom** (per catalog): `removed_mass_per_mission` [kg p50], and per weighting
  `fom` [kg/CU, p05/p50/p95] and `band_weight` [normalized w(h)]. FoM =
  sum m_i*w(h_i)/C_campaign (spec §4). The two weightings are PLACEHOLDER and
  disagree on band priority (T5). WP14 adds, per weighting: `cost_per_  risk_equiv_mass` [CU_per_kg, p05/p50/p95] = C_campaign/sum(m_i*w(h_i)),
  the exact inverse orientation: each percentile is the reciprocal of the
  FoM percentile BY CONSTRUCTION (NOT the percentile of per-run
  reciprocals -- order statistics interpolate, so the two are not the same
  computation), with p05/p95 swapped since 1/x is decreasing; and
  `cost_per_risk_equiv_mass_musd` [MUSD_per_kg, cost_scenario=mid only, to
  bound row count] = the same quantity converted via the mid-scenario anchor.
- **cost_component** (per catalog, baseline N): C_dev / C_bus / C_kit_total /
  C_launch / C_ops [CU] at the median mission time.
- **tornado** (representative catalog, baseline N): per cost parameter,
  `cost_per_removal_swing` [CU] and `fom_swing` [kg/CU] under a +/- one-at-a-
  time perturbation; rows are sorted by cost/removal swing.
- **currency_anchor** (global): `cu_to_musd_range` -- unfilled pre-WP14
  sentinel (CostConfig::cu_to_musd_low/high, both 0.0; SUPERSEDED, see
  currency_anchor_derived below). Retained byte-identical for schema
  continuity; it never carries a point-value dollar figure.

## record_type rows (schema 1.1, new -- WP14)

- **cost_component_musd** (global, one row per WP14 itemized cost row x
  cost_scenario): `metric=cost`, `estimate`/`p50` = the row's value at that
  scenario, `units` = the item's own unit (MUSD / MUSD/yr / MUSD/unit / a
  percentage unit), `param` = the item id, `notes` = the full cited source
  note (D10: every row is sourced-or-PLACEHOLDER; a PLACEHOLDER row's note
  ends `[PLACEHOLDER]` and its value is 0, never fabricated).
- **currency_anchor_derived** (global, one row per cost_scenario):
  `metric=anchor_musd_per_cu` [musd_per_cu] = core_5_component / 
  C_campaign_CU_p50 at the baseline N (catalog A). The core is EXACTLY the
  5 CU-mapped components (Sec.10 Method): development +
  manufacturing_bus_unit + kit_unit_scaled_9t x baseline-N-kits + C_launch
  (low=rideshare card low, mid=rideshare upper, high=dedicated Electron --
  width preserved, not narrowed) + operations_3yr. ground_segment_3yr,
  licensing_fcc_application, licensing_fcc_annual, and the PLACEHOLDER
  ssa_tracking row are ADDITIVE and excluded from this core (see
  campaign_cost_musd below); so are the insurance/contingency percentage
  rows. The `notes` field prints the full arithmetic string, numerator AND
  denominator, so the figure is independently auditable from the row alone.
- **campaign_cost_musd** (global, per cost_scenario), four separately
  visible rows so the anchor's basis is auditable: `metric=core_5_component`
  = the identical numerator as currency_anchor_derived, in MUSD;
  `metric=additives` = ground_segment_3yr + licensing_fcc_application +
  licensing_fcc_annual x3 yr + ssa_tracking (PLACEHOLDER, 0), outside the CU
  model and outside the anchor core; `metric=base_total` = core_5_component
  + additives; `metric=with_reserves` = base_total x (1 +
  contingency_reserve_pct/100) + insurance_line, where insurance_line =
  (insurance_launch_plus_1yr_pct/100) x insured_value and insured_value =
  manufacturing_bus_unit + kit_unit_scaled_9t x baseline-N-kits + C_launch
  (asset+launch scale only -- insurance is an ADDITIVE line, not a
  multiplier over the multi-year opex folded into base_total); `notes`
  carries the formula for each row.
- **cost_per_removal_musd** (catalog A, baseline N, per cost_scenario):
  `metric=cost_per_removal` [MUSD_per_removal, p05/p50/p95] = the existing
  cost_per_removal CU percentiles at the baseline N times that scenario's
  anchor_musd_per_cu.

## Honesty framing (D10, carried into every WP14 currency row)

External absolute-cost estimation by an unaffiliated author is inherently
low-standing; agencies re-run with their own parametric models; the relative
CU results remain primary. No point-value currency figure is emitted:
absolute costs appear ONLY as low/mid/high cited ranges.

## Future visualization / evidence -- not implemented here

The schema supports, without change: amortization curves, FoM-under-weightings
bars with percentile whiskers, tornado bars, the cost-component breakdown, and
the WP14 itemized/anchor/campaign-cost tables. This module emits none of
these charts.
