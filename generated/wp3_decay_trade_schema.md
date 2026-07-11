# WP3 decay-trade CSV schema (version 1.0)

`generated/wp3_decay_trade.csv` is the committed sail-decay trade table
emitted by `decay_trade` (src/main_decay_trade.cpp) from the WP3 decay model
(src/decay.cpp). It re-emits the same sail-area x solar-activity ->
decay-years trade the adsc_sim scenario-6 demo prints, as machine-readable
CSV (R6: deterministic, timestamp-free). Consumers: tools/viz/make_viz.py
(WP7a figures) and tools/evidence/make_evidence.py (WP7 number audit).

## Columns

| column | meaning |
|---|---|
| schema_version | WP3 schema id (`1.0`) |
| catalog | class-level preset name (blank for the guideline row) |
| mass_kg | catalog stage mass [kg] (0.0 for the guideline row) |
| altitude_km | catalog altitude [km] (0.0 for the guideline row) |
| record_type | decay_years / area_for_25yr / guideline |
| sail_area_m2 | swept sail area [m^2] (0.0 when not applicable) |
| value_solar_max | value under the solar-max density factor |
| value_solar_min | value under the solar-min density factor |
| units | years / m2 |
| notes | provenance / caveats |

## record_type rows

- `decay_years`: decay time to the reentry-handoff altitude for the swept
  sail area, as a solar max .. solar min band (T4 discipline: never a point).
- `area_for_25yr`: sail area required to meet the IADC 25-year guideline
  (solar max .. min). The FCC 5-year rule is tracked in the WP8 rulepacks;
  the trade keeps the IADC reference line (see README).
- `guideline`: the 25-year reference line itself.

Schema changes follow R15: additive columns (e.g. the WP13 EDT/hybrid trade)
bump the minor version (1.0 -> 1.1) with this file and all consumers updated
in the same PR, so the reproducibility gate stays green.
