# WP12 fidelity ladder -- what each level verifies, and what it does not

Runtime-selectable fidelity levels (single code path, R1): **L0** (`[L0: two-body linear CW, ds-v1]`) is the ORIGINAL WP1 Clohessy-Wiltshire linearization -- every committed WP5 campaign number stays byte-identical; **L1** (`[L1: two-body+J2, ds-v1]`) differences a full inertial two-body+J2 RK4 propagation of both craft into the target's instantaneous LVLH frame; **L2** (`[L2: +drag, ds-v2]`) adds a per-craft free-molecular drag term under an INDEPENDENT ballistic-coefficient dispersion stream (ds-v2 = ds-v1 + independent BC stream; the committed campaign's own ds-v1 stream is untouched). L0 numbers here reproduce the committed wp5_campaign_runs.csv abort events exactly (same commanded dv, same coast); L1/L2 numbers are NEW claims, R14-tagged, and are re-VERIFICATIONS of the existing WP11 clearing-abort LAW (unchanged) under a higher-fidelity coast -- not a new abort law.

Master seed `0x5AD5C0DECAFE2026`, 500 runs/catalog (SAME campaign draws as wp5_campaign_runs.csv -- ds-v1). Regenerate with `adsc_ladder`.

## Abort-event re-verification (292 catalog-A + 291 catalog-B closing-speed-gate events from the committed campaign)

| level | dispersion set | catalog | n_events | n_violations | violation_rate | wilson 95% CI | clearance floor (m) | p05 (m) | p50 (m) |
|---|---|---|---:|---:|---:|---|---:|---:|---:|
| L0 | ds-v1 | SL-16 / Zenit-2 second stage | 292 | 0 | 0.000000 | [0.000000, 0.012985] | 20.0053 | 36.3253 | 139.7579 |
| L1 | ds-v1 | SL-16 / Zenit-2 second stage | 292 | 0 | 0.000000 | [0.000000, 0.012985] | 19.0070 | 35.8083 | 139.2909 |
| L2 | ds-v2 | SL-16 / Zenit-2 second stage | 292 | 0 | 0.000000 | [0.000000, 0.012985] | 19.0086 | 35.7539 | 139.2424 |
| L0 | ds-v1 | SL-8 / Kosmos-3M second stage | 291 | 0 | 0.000000 | [0.000000, 0.013029] | 20.0002 | 26.5835 | 143.9558 |
| L1 | ds-v1 | SL-8 / Kosmos-3M second stage | 291 | 0 | 0.000000 | [0.000000, 0.013029] | 18.8096 | 26.2484 | 143.0785 |
| L2 | ds-v2 | SL-8 / Kosmos-3M second stage | 291 | 0 | 0.000000 | [0.000000, 0.013029] | 18.8129 | 26.0505 | 142.9937 |

KNOWN GAP: RetreatHop abort events (rare -- the escalation of last resort) are re-verified here using ONLY their FIRST burn's dv; the campaign's own screen coast-verifies both burns (mission.cpp). fidelity_coast_min_range has no mid-course-burn hook, so this is a documented limitation of the ladder re-verification, not of the campaign itself.

## Forensic-14 per-level clearance

The 14 WP10c/WP11 pinned states (R15) all clear at L0 by construction (the WP11 clearing law). A case that clears at L0 but dips below keep-out at a higher level is FLAGGED PROMINENTLY below as a first-class finding -- an honest result of a higher-fidelity coast, not a regression of the committed L0 numbers.

| case | L0 clears | L0 min range (m) | L1 clears | L1 min range (m) | L2 clears | L2 min range (m) | flag |
|---|---|---:|---|---:|---|---:|---|
| A65 | yes | 220.2879 | yes | 219.4876 | yes | 219.4891 | - |
| A81 | yes | 220.0053 | yes | 219.5300 | yes | 219.5314 | - |
| A86 | yes | 220.0429 | yes | 221.5558 | yes | 221.4340 | - |
| A112 | yes | 220.0102 | yes | 221.8714 | yes | 221.7495 | - |
| A340 | yes | 220.1953 | yes | 223.1716 | yes | 223.0415 | - |
| A460 | yes | 220.1762 | yes | 219.4546 | yes | 219.4562 | - |
| A490 | yes | 220.3237 | yes | 219.6226 | yes | 219.6240 | - |
| B88 | yes | 220.3699 | yes | 219.8577 | yes | 219.8602 | - |
| B99 | yes | 220.0544 | yes | 219.7985 | yes | 219.8001 | - |
| B160 | yes | 220.1758 | yes | 219.5121 | yes | 219.5149 | - |
| B254 | yes | 225.4615 | yes | 225.1436 | yes | 225.1453 | - |
| B362 | yes | 220.4168 | yes | 219.5278 | yes | 219.5303 | - |
| B383 | yes | 220.2168 | yes | 219.4631 | yes | 219.4660 | - |
| B419 | yes | 220.3780 | yes | 222.4611 | yes | 222.2242 | - |

## Margin-decay measurement (F2: promised, now measured)

Min-range erosion over repeated orbits at L1, relative to the orbit=1 baseline, for a 400 m standoff ellipse and the rho=300 m innermost corridor hold, both catalog orbits.

| catalog | geometry | orbits | min range (m) | erosion vs orbit=1 (m) |
|---|---|---:|---:|---:|
| SL-16 / Zenit-2 second stage | standoff_400m | orbit=1 | 563.9347 | 0.0000 |
| SL-16 / Zenit-2 second stage | standoff_400m | orbit=2 | 563.7906 | 0.1441 |
| SL-16 / Zenit-2 second stage | standoff_400m | orbit=3 | 563.5108 | 0.4239 |
| SL-16 / Zenit-2 second stage | standoff_400m | orbit=4 | 563.0947 | 0.8400 |
| SL-16 / Zenit-2 second stage | standoff_400m | orbit=5 | 562.5347 | 1.4000 |
| SL-16 / Zenit-2 second stage | corridor_300m | orbit=1 | 422.9539 | 0.0000 |
| SL-16 / Zenit-2 second stage | corridor_300m | orbit=2 | 422.8459 | 0.1081 |
| SL-16 / Zenit-2 second stage | corridor_300m | orbit=3 | 422.6360 | 0.3180 |
| SL-16 / Zenit-2 second stage | corridor_300m | orbit=4 | 422.3239 | 0.6301 |
| SL-16 / Zenit-2 second stage | corridor_300m | orbit=5 | 421.9038 | 1.0501 |
| SL-8 / Kosmos-3M second stage | standoff_400m | orbit=1 | 563.9213 | 0.0000 |
| SL-8 / Kosmos-3M second stage | standoff_400m | orbit=2 | 563.7897 | 0.1316 |
| SL-8 / Kosmos-3M second stage | standoff_400m | orbit=3 | 563.5249 | 0.3964 |
| SL-8 / Kosmos-3M second stage | standoff_400m | orbit=4 | 563.1343 | 0.7869 |
| SL-8 / Kosmos-3M second stage | standoff_400m | orbit=5 | 562.6045 | 1.3168 |
| SL-8 / Kosmos-3M second stage | corridor_300m | orbit=1 | 422.9439 | 0.0000 |
| SL-8 / Kosmos-3M second stage | corridor_300m | orbit=2 | 422.8452 | 0.0987 |
| SL-8 / Kosmos-3M second stage | corridor_300m | orbit=3 | 422.6466 | 0.2973 |
| SL-8 / Kosmos-3M second stage | corridor_300m | orbit=4 | 422.3536 | 0.5903 |
| SL-8 / Kosmos-3M second stage | corridor_300m | orbit=5 | 421.9562 | 0.9877 |

Coverage statement: this ladder verifies keep-out-abort geometry re-verification, forensic-14 regression clearance, and min-range erosion -- each tagged by the fidelity level ([L0]/[L1]/[L2]) that produced it. It does NOT verify: attitude dynamics (WP2, untouched translation-only extension), the estimate-driven guidance loop (WP12 L4, see wp12_est_* in reference_metrics.csv), or actuator realization error (WP12 L5, see wp12_l5_* in reference_metrics.csv). No result on this page is a legal, regulatory, or safety conclusion beyond its stated model scope; each row states exactly which model produced it.
