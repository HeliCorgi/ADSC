# WP6 cost/FoM CSV schema (version 1.0)

`generated/wp6_cost_summary.csv` is a long-format table consumed by WP7. It
consumes the WP5 campaign (schema 1.0): the amortization sweep re-runs the WP5
engine across the kit-inventory sweep at the fixed master seed, and the
baseline row (n_kits = CampaignConfig::kits_initial) reproduces
`generated/wp5_campaign_runs.csv` exactly. Primary units are RELATIVE cost
units (CU); **no point-value currency is emitted** (R6/D10).

## Columns

| column | meaning |
|---|---|
| schema_version | WP6 schema id (`1.0`) |
| catalog | class-level preset name (blank for global rows) |
| record_type | amortization / fom / tornado / cost_component / currency_anchor |
| n_kits | kits carried (amortization/cost_component rows); blank otherwise |
| param | cost parameter (tornado / cost_component rows); blank otherwise |
| weighting | spatial_density / criticality (fom rows); blank otherwise |
| metric | the quantity named in the row |
| estimate | point value (p50 for distributions) |
| p05 / p50 / p95 | percentiles (distribution rows; 0 when not applicable) |
| units | CU_per_removal / CU / count / ratio / kg / kg_per_CU / normalized / musd_per_cu |
| notes | provenance / PLACEHOLDER caveats |

## record_type rows

- **amortization** (per catalog, per swept N): `cost_per_removal` [CU/removal,
  p05/p50/p95], `cost_per_removal_ratio_to_n1` [ratio], `removals` [count p50],
  `campaign_cost` [CU p50]. The cost/removal vs N curve is the core installer
  amortization argument: it falls until the Δv budget (not the kit count)
  bounds removals, then turns up as extra carried kits add cost without
  removals.
- **fom** (per catalog): `removed_mass_per_mission` [kg p50], and per weighting
  `fom` [kg/CU, p05/p50/p95] and `band_weight` [normalized w(h)]. FoM =
  sum m_i*w(h_i)/C_campaign (spec §4). The two weightings are PLACEHOLDER and
  disagree on band priority (T5).
- **cost_component** (per catalog, baseline N): C_dev / C_bus / C_kit_total /
  C_launch / C_ops [CU] at the median mission time.
- **tornado** (representative catalog, baseline N): per cost parameter,
  `cost_per_removal_swing` [CU] and `fom_swing` [kg/CU] under a +/- one-at-a-
  time perturbation; rows are sorted by cost/removal swing.
- **currency_anchor** (global): `cu_to_musd_range` -- a PLACEHOLDER cited
  range (estimate/p05 = low, p95 = high). WP6 never emits a point-value dollar
  figure; a filled cited range is a WP7 deliverable.

## Future visualization / evidence (WP7) — not implemented here

The schema supports, without change: amortization curves, FoM-under-weightings
bars with percentile whiskers, tornado bars, and the cost-component breakdown.
WP6 emits none of these charts and predicts no absolute cost.
