# WP6 Parametric cost model + FoM — summary

Primary output is in **relative cost units (CU)**; **no point-value dollar figure is claimed** (R6/D10) -- the CU->currency anchor is a PLACEHOLDER cited range, filled in WP7. All cost/FoM parameters are PLACEHOLDER (see `CostConfig`). Cost, cost/removal and FoM are propagated **per run** from the WP5 campaign (schema 1.0) and reported as p05/p50/p95. Regenerate with `adsc_cost`.

## SL-16 / Zenit-2 second stage

### Amortization curve (cost/removal vs kits carried N)

| N | cost/removal p50 [CU] | p05..p95 | ratio to N=1 | removals p50 | campaign cost p50 [CU] |
|---:|---:|---|---:|---:|---:|
| 1 | 157.07 | 157.07 .. 159.07 | 1.000 | 1.00 | 157.07 |
| 2 | 82.22 | 82.22 .. 83.42 | 0.523 | 2.00 | 164.44 |
| 3 | 57.27 | 57.27 .. 57.94 | 0.365 | 3.00 | 171.82 |
| 4 | 44.80 | 44.80 .. 59.72 | 0.285 | 4.00 | 179.20 |
| 5 | 46.14 | 36.91 .. 61.51 | 0.294 | 4.00 | 184.55 |
| 6 | 47.48 | 37.98 .. 63.29 | 0.302 | 4.00 | 189.90 |
| 7 | 48.81 | 39.06 .. 65.08 | 0.311 | 4.00 | 195.26 |
| 8 | 50.15 | 40.13 .. 66.86 | 0.319 | 4.00 | 200.61 |

### FoM (debris-risk reduction per cost) at N=4

| weighting | w(h) | removed mass p50 [kg] | FoM p50 [kg/CU] | FoM p05..p95 |
|---|---:|---:|---:|---|
| spatial_density | 0.780 | 36000 | 156.7007 | 117.5399 .. 156.7008 |
| criticality | 1.000 | 36000 | 200.8983 | 150.6921 .. 200.8984 |

## SL-8 / Kosmos-3M second stage

### Amortization curve (cost/removal vs kits carried N)

| N | cost/removal p50 [CU] | p05..p95 | ratio to N=1 | removals p50 | campaign cost p50 [CU] |
|---:|---:|---|---:|---:|---:|
| 1 | 156.59 | 156.59 .. 158.59 | 1.000 | 1.00 | 156.59 |
| 2 | 81.97 | 81.97 .. 82.97 | 0.523 | 2.00 | 163.93 |
| 3 | 57.09 | 57.09 .. 57.76 | 0.365 | 3.00 | 171.28 |
| 4 | 44.66 | 44.66 .. 59.53 | 0.285 | 4.00 | 178.62 |
| 5 | 45.99 | 36.79 .. 61.31 | 0.294 | 4.00 | 183.95 |
| 6 | 47.32 | 37.86 .. 63.08 | 0.302 | 4.00 | 189.27 |
| 7 | 48.65 | 38.92 .. 64.86 | 0.311 | 4.00 | 194.59 |
| 8 | 49.98 | 39.99 .. 66.63 | 0.319 | 4.00 | 199.91 |

### FoM (debris-risk reduction per cost) at N=4

| weighting | w(h) | removed mass p50 [kg] | FoM p50 [kg/CU] | FoM p05..p95 |
|---|---:|---:|---:|---|
| spatial_density | 1.000 | 5600 | 31.3509 | 23.5161 .. 31.3509 |
| criticality | 0.680 | 5600 | 21.3186 | 15.9909 .. 21.3186 |

## Tornado sensitivity (SL-16 / Zenit-2 second stage, baseline N, +/-30%)

| rank | parameter | cost/removal swing [CU] | spatial FoM swing [kg/CU] |
|---:|---|---:|---:|
| 1 | c_dev_cu | 15.00 | 53.98113 |
| 2 | c_bus_cu | 4.54 | 15.93432 |
| 3 | c_launch_cu_per_kg | 3.72 | 13.04301 |
| 4 | c_kit_cu | 2.40 | 8.40093 |
| 5 | c_ops_cu_per_day | 1.21 | 4.24428 |

**T5 (open trade — weighting disagreement).** The two normalized congestion weightings peak at different altitudes (spatial ~750 km, criticality ~840 km), so they rank target *bands* differently: the spatial-density weighting values the lower (SL-8, ~750 km) band more, the criticality weighting values the higher (SL-16, ~840 km) band more. Absolute per-catalog FoM is dominated by removed mass (the ~9 t class outranks the ~1.4 t class under both weightings), but the band-priority flip is a real metric-choice disagreement tracked as T5. Both weighting tables are PLACEHOLDER and must be filled with citations.

**No absolute-cost prediction.** Every figure above is in relative CU; WP6 does not predict absolute program cost. A cited CU->currency range is a WP7 deliverable.
