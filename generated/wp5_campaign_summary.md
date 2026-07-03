# WP5 Campaign Monte Carlo — summary

Master seed `0x5AD5C0DECAFE2026`, 500 runs per catalog preset. All rates carry a Wilson 95% confidence interval (z = 1.95996398454005). Distributions are reported as 5th / 50th / 95th percentiles. Regenerate with `adsc_campaign`.

WP5 performs **no** legal or regulatory determination and produces **no** visualization; the compliance columns are passive research-profile metadata for future WP7/WP8 tooling.

## SL-16 / Zenit-2 second stage

| metric | estimate | 95% CI / p05..p95 | units | notes |
|---|---:|---|---|---|
| success_rate | 0.5560 | [0.5122, 0.5990] | fraction | productive end (completed OR kit_exhausted); Wilson 95% CI (z=1.959963984540054) |
| nonproductive_termination_rate | 0.4440 | [0.4010, 0.4878] | fraction | 1 - success_rate: runs ended by dv_exhausted OR keep_out_violation; Wilson 95% CI (z=1.959963984540054) |
| gate_abort_run_rate | 0.4440 | [0.4010, 0.4878] | fraction | runs with >=1 closing-speed gate abort = abort-path exposure (spec 'abort rate'); Wilson 95% CI (z=1.959963984540054) |
| keep_out_violation_rate | 0.0140 | [0.0068, 0.0286] | fraction | Wilson 95% CI (z=1.959963984540054) |
| dv_used_m_per_s | 127.6220 | 124.000 .. 124.000 .. 136.000 | m_per_s | per mission |
| kits_used | 3.8300 | 3.000 .. 4.000 .. 4.000 | count | per mission |
| removals_per_mission | 3.8300 | 3.000 .. 4.000 .. 4.000 | count | per mission |
| sync_arrival_time_s | 17.5436 | 14.466 .. 17.680 .. 20.094 | s | first successful sync per mission; missions with no sync excluded |
| mission_time_s | 345946.1017 | 348441.079 .. 349389.800 .. 349395.864 | s | includes PLACEHOLDER phasing/attach/depart time |
| completed | 0.0000 | - | runs | terminal outcome |
| dv_exhausted | 215.0000 | - | runs | terminal outcome |
| kit_exhausted | 278.0000 | - | runs | terminal outcome |
| keep_out_violation | 7.0000 | - | runs | terminal outcome |
| gate_abort | 288.0000 | - | events | per-target safe-abort events |
| sync_timeout | 0.0000 | - | events | per-target failed-sync events |
| other | 0.0000 | - | runs | unexpected outcome (should be 0) |

## SL-8 / Kosmos-3M second stage

| metric | estimate | 95% CI / p05..p95 | units | notes |
|---|---:|---|---|---|
| success_rate | 0.5420 | [0.4982, 0.5852] | fraction | productive end (completed OR kit_exhausted); Wilson 95% CI (z=1.959963984540054) |
| nonproductive_termination_rate | 0.4580 | [0.4148, 0.5018] | fraction | 1 - success_rate: runs ended by dv_exhausted OR keep_out_violation; Wilson 95% CI (z=1.959963984540054) |
| gate_abort_run_rate | 0.4580 | [0.4148, 0.5018] | fraction | runs with >=1 closing-speed gate abort = abort-path exposure (spec 'abort rate'); Wilson 95% CI (z=1.959963984540054) |
| keep_out_violation_rate | 0.0140 | [0.0068, 0.0286] | fraction | Wilson 95% CI (z=1.959963984540054) |
| dv_used_m_per_s | 127.7220 | 124.000 .. 124.000 .. 136.000 | m_per_s | per mission |
| kits_used | 3.8400 | 3.000 .. 4.000 .. 4.000 | count | per mission |
| removals_per_mission | 3.8400 | 3.000 .. 4.000 .. 4.000 | count | per mission |
| sync_arrival_time_s | 17.5170 | 14.125 .. 17.710 .. 20.163 | s | first successful sync per mission; missions with no sync excluded |
| mission_time_s | 345610.3039 | 348441.369 .. 349390.165 .. 349396.141 | s | includes PLACEHOLDER phasing/attach/depart time |
| completed | 0.0000 | - | runs | terminal outcome |
| dv_exhausted | 222.0000 | - | runs | terminal outcome |
| kit_exhausted | 271.0000 | - | runs | terminal outcome |
| keep_out_violation | 7.0000 | - | runs | terminal outcome |
| gate_abort | 288.0000 | - | events | per-target safe-abort events |
| sync_timeout | 0.0000 | - | events | per-target failed-sync events |
| other | 0.0000 | - | runs | unexpected outcome (should be 0) |

Notes. `success` = a productive end (all targets processed OR the full kit complement installed), not a mission cut short by propellant exhaustion or a keep-out violation; a run may be both a success and contain safe-abort events.

Two distinct abort-related rates are reported: `nonproductive_termination_rate` = 1 - success_rate (runs ended by dv_exhausted or keep_out_violation), and `gate_abort_run_rate` = the fraction of runs with >=1 closing-speed gate abort (the abort-path exposure the spec calls the 'abort rate'). Under the current flat PLACEHOLDER Delta-v cost these two coincide numerically -- every aborting mission needs an extra target-slot to still install its kits and so exhausts the 140 m/s budget -- but they are separate concepts and will diverge if the cost model changes. If `gate_abort_run_rate` is 0 the closing-speed dispersion may be too narrow or the 0.15 m/s gate is not exercised; if `keep_out_violation_rate` is 0 the abort maneuvers are clearing the keep-out sphere (expected under nominal dispersions).

`gate_abort` / `sync_timeout` are per-target *event* counts (a safe abort or failed sync lets the servicer move to the next target); `completed` / `dv_exhausted` / `kit_exhausted` / `keep_out_violation` are per-*run* terminal outcomes.

The `dv_used` and `kits_used` percentiles matching across the two catalog presets is expected: the Delta-v/kit cost model is a flat PLACEHOLDER, so those quantities take a small set of quantized values independent of catalog (not a copy-paste bug). The attitude-sync leg and that cost model are class-independent in the WP5 model, so the two presets differ mainly by independent sampling (per-catalog seed salt) plus the altitude-dependent keep-out abort screen. Class-specific physics lives in WP3 decay and future WP6 cost.
