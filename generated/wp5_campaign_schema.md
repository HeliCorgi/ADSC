# WP5 Campaign CSV schema (version 1.0)

Stable machine-readable outputs for downstream WP6 (cost/FoM), WP7
(visualization/evidence), and WP8 (compliance matrix). The `schema_version`
column is `1.0`; do not change column meanings without bumping it. WP5 itself
performs **no** legal/regulatory determination and produces **no** charts.

## Files

- `wp5_campaign_runs.csv` — one row per mission run (WP5-native raw records).
- `wp5_campaign_summary.csv` — one row per metric per catalog preset.
- `wp5_campaign_summary.md` — human-readable rendering of the summary.
- `wp5_campaign_schema.md` — this document.

## `wp5_campaign_runs.csv`

| column | units | kind | notes |
|---|---|---|---|
| schema_version | - | WP5-native | schema id (`1.0`) |
| catalog | - | WP5-native | class-level preset name (never an object id) |
| master_seed | - | WP5-native | fixed campaign master seed |
| run_index | - | WP5-native | 0-based mission index |
| run_seed | - | WP5-native | SplitMix64(master_seed XOR FNV1a(catalog), run_index) |
| outcome | - | WP5-native | terminal: completed/dv_exhausted/kit_exhausted/keep_out_violation/other |
| removals | count | WP5-native | targets removed this mission |
| targets_attempted | count | WP5-native | targets processed before termination |
| dv_budget_m_per_s | m/s | WP5-native, PLACEHOLDER | mission Delta-v budget |
| dv_used_m_per_s | m/s | WP5-native, PLACEHOLDER-derived | sum of leg costs |
| dv_remaining_m_per_s | m/s | WP5-native, PLACEHOLDER-derived | budget minus used |
| kits_initial | count | WP5-native | starting kit inventory |
| kits_used | count | WP5-native | kits installed (= removals) |
| kits_remaining | count | WP5-native | inventory at termination |
| sync_time_s | s | WP5-native | first successful sync declare time; blank if none |
| mission_time_s | s | WP5-native, PLACEHOLDER-derived | elapsed incl. placeholder phasing |
| failure_reason | - | WP5-native | `none` if success, else the terminal outcome |
| keep_out_violation | bool | WP5-native | terminal keep-out breach occurred |
| abort | bool | WP5-native | >=1 safe-abort (closing-speed gate) event occurred |
| success | bool | WP5-native | productive end: completed OR kit_exhausted (not dv/keep-out terminated) |
| gate_abort_events | count | WP5-native | per-target safe-abort events this run |
| sync_timeout_events | count | WP5-native | per-target failed-sync events this run |
| first_closing_speed_m_per_s | m/s | WP5-native, PLACEHOLDER-derived | target-0 capture closing speed vs 0.15 gate |
| tumble_rate_deg_per_s | deg/s | WP5-native, PLACEHOLDER-derived | realized |w_t| of first synced attempt |
| solar_factor | - | WP5-native, PLACEHOLDER-derived | realized atmospheric-density factor (for WP3/WP6) |
| research_only | bool | future-facing WP8 | passive; always true for the research profile |
| class_level_preset | bool | future-facing WP8 | passive; targets are class parameters, not object ids |
| uses_live_tle | bool | future-facing WP8 | passive; always false (no live ephemeris, R13) |
| generates_target_specific_operations | bool | future-facing WP8 | passive; always false |
| requires_owner_consent_assumption | bool | future-facing WP8 | passive; always true |
| owner_consent_assumed | bool | future-facing WP8 | passive; research-scenario ASSUMPTION only, not a legal fact |
| proximity_operations_mode | - | future-facing WP8 | passive descriptor string |
| unconsented_approach_blocked_by_policy | bool | future-facing WP8 | passive; always true |
| controlled_reentry_mode | bool | future-facing WP8 | passive; false = passive drag-sail baseline |
| rf_transmitter_modelled | bool | future-facing WP8 | passive; always false |
| remote_sensing_modelled | bool | future-facing WP8 | passive; always false |
| compliance_notes | - | future-facing WP8 | passive; states WP5 makes no legal/regulatory determination |

**Legal-interpretation caveat.** The compliance columns are deterministic
research-profile metadata. `owner_consent_assumed = true` is a *research
scenario assumption* (D9), never a statement that consent legally exists. No
column may be read as regulatory approval; WP5 performs no such analysis.

## `wp5_campaign_summary.csv`

| column | units | notes |
|---|---|---|
| schema_version | - | schema id (`1.0`) |
| catalog | - | class-level preset name |
| master_seed | - | fixed campaign master seed |
| n_runs | count | runs aggregated for this catalog |
| metric | - | metric name (see rows below) |
| estimate | mixed | rate fraction, distribution mean, or classification count |
| wilson_low | fraction | rate rows only: Wilson 95% lower bound |
| wilson_high | fraction | rate rows only: Wilson 95% upper bound |
| p05 | mixed | distribution rows only: 5th percentile |
| p50 | mixed | distribution rows only: 50th percentile |
| p95 | mixed | distribution rows only: 95th percentile |
| units | - | fraction / count / m_per_s / s / runs / events |
| source_runs_csv | - | provenance: wp5_campaign_runs.csv |
| notes | - | per-metric note |

### Summary metric rows (per catalog)

Rates (with Wilson 95% CI): `success_rate`, `abort_rate`,
`keep_out_violation_rate`. Distributions (p05/p50/p95): `dv_used_m_per_s`,
`kits_used`, `removals_per_mission`, `sync_arrival_time_s`, `mission_time_s`.
Failure classification counts: `completed`, `dv_exhausted`, `kit_exhausted`,
`keep_out_violation` (runs) and `gate_abort`, `sync_timeout` (per-target
events), plus `other` (should be 0).

### Future visualization (WP7) — not implemented here

The schema supports, without change: outcome proportions, Wilson intervals,
Delta-v / removals / sync-time percentiles, keep-out-violation rate, and
failure-classification bars. WP5 emits none of these charts.
