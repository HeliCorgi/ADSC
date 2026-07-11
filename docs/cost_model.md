# Cost model

This is the full detail behind the README's cost/FoM "Key results" digest. It
covers the parametric relative-cost model, the amortization curve, the
two-weighting figure of merit (FoM) with open trade T5, the WP14
itemized-range/currency-anchor upgrade, and the honesty framing that governs
every absolute-dollar figure this project ever states. Campaign inputs are
[docs/safety.md](safety.md) (the full WP5 table); kit-class/target context is
[docs/target_selection.md](target_selection.md).

**An absolute minimum-cost claim is banned** (spec section 1) — the
defensible claim throughout this file is **cost-effectiveness**: cost per
removal and cost per unit risk reduction.

## The parametric model (WP6)

`cost` implements the spec's

```
C_campaign = C_dev + C_bus(m_dry) + N·C_kit + C_launch(m_wet, band) + C_ops(T)
```

in **relative cost units (CU)** — no point-value dollar figure is ever
emitted from this core model (R6/D10); the CU→currency anchor is a cited
range, derived at runtime from the WP14 itemized table (see below), never a
hand-typed constant. It consumes the WP5 campaign (**schema 1.1** —
`generated/wp5_campaign_schema.md`; the amortization sweep re-runs the WP5
engine across a kit-inventory sweep at the fixed master seed, and the
baseline (N = 4) reproduces `wp5_campaign_runs.csv` exactly). Cost,
cost/removal, and

```
FoM = Σᵢ mᵢ·w(hᵢ) / C_campaign
```

are propagated **per run** (p05/p50/p95, never mean-only). FoM is reported
under **two** normalized congestion weightings (spatial-density +
criticality-style, both PLACEHOLDER tables); they disagree on band priority
— **open trade T5**, discussed below. A machine-readable tornado ranks each
cost parameter's ± impact. Outputs `generated/wp6_cost_summary.{md,csv}` +
`wp6_cost_schema.md`, **schema 1.1** (the schema bumped from 1.0 to 1.1 for
WP14's absolute-cost rows; this file states the current version rather than
carrying the "1.0" label forward — a drift the WP15 content-mapping pass
found and fixed in this document, per R16's whole point). Unit-tested
(`tests/test_cost.cpp`). Regenerate with `./build/adsc_cost`.

## Amortization curve, FoM, and tornado — the full table

<!-- DOCS-WP6-FULL-START (source: generated/wp6_cost_summary.csv, schema 1.1; verbatim from tools/docs/fill_docs_numbers.py build_wp6_full(), CI regenerates this from adsc_cost) -->
The full sweep (500 runs × 2 catalogs × N = 1..8 kits) runs in a few tens of
seconds on a CI runner (see the Actions log of the current run for the actual
figure), and the baseline (N = 4) removals reproduce the committed WP5 CSV
exactly (`MATCH (schema 1.1 consumed)`).

**Amortization curve — cost/removal vs kits carried N (SL-16 class):**

| N | cost/removal p50 [CU] | ratio to N=1 | removals p50 |
|---:|---:|---:|---:|
| 1 | 157.07 | 1.000 | 1 |
| 2 | 82.22 | 0.523 | 2 |
| 3 | 57.27 | 0.365 | 3 |
| **4** | **44.80** | **0.285** | 4 |
| 5 | 46.14 | 0.294 | 4 |
| 8 | 50.15 | 0.319 | 4 |

Batch amortization drives cost/removal down to **28.5 % of the single-target
(N = 1) baseline** — a **3.5× per-removal saving** — bottoming at N = 4 where the
**Δv budget (not the kit count)** caps removals at 4; carrying more kits then
adds cost without removals and the curve turns back up. That shape *is* the
quantitative installer/batch argument (SL-8 class is within ~0.3 %).

**FoM = Σ mᵢ·w(hᵢ)/C_campaign at N = 4 (p50, kg/CU):**

| catalog | spatial-density | criticality |
|---|---:|---:|
| SL-16 (~9 t, 840 km) | 156.70 (w = 0.78) | 200.90 (w = 1.00) |
| SL-8 (~1.4 t, 750 km) | 31.35 (w = 1.00) | 21.32 (w = 0.68) |

The ~9 t class outranks the ~1.4 t class under **both** weightings (FoM is
mass-dominated), but the **band weight flips** — spatial density values the
750 km band more, the criticality index values the 840 km band more — a genuine
metric-choice disagreement tracked as **open trade T5**. **Tornado** (SL-16,
±30 %, ranked by cost/removal swing): `c_dev_cu` (15.0 CU) ≫ `c_bus_cu` (4.5) >
`c_launch_cu_per_kg` (3.7) > `c_kit_cu` (2.4) > `c_ops_cu_per_day` (1.2). All
values are relative CU; **no absolute-dollar figure is claimed**.
<!-- DOCS-WP6-FULL-END -->

### Open trade T5 (metric choice changes band priority)

The two normalized congestion weightings peak at different altitudes
(spatial ≈750 km, criticality ≈840 km), so they rank target *bands*
differently: the spatial-density weighting values the lower (catalog B,
≈750 km) band more, the criticality weighting values the higher (catalog A,
≈840 km) band more. Absolute per-catalog FoM stays mass-dominated under both
(catalog A outranks catalog B either way), but the band-priority flip is real
and is kept visible rather than resolved by fiat. Both weighting tables are
PLACEHOLDER and must be filled with citations; the criticality weighting's
peak is McKnight-anchored (catalog A, ≈840-850 km — see
[docs/target_selection.md](target_selection.md)), but the rest of the table
(the falloff away from that peak) remains an uncited interpolation.

## Absolute cost ranges (WP14)

Spec D10 rule, quoted verbatim: "Every row carries a source or stays
PLACEHOLDER — this is D10 applied, not relaxed. Relative CU results remain
primary." Absolute cost is itemized as **low/mid/high ranges only** across
development, manufacturing, launch/rideshare, ground segment, ops,
SSA/tracking, insurance/licensing, kit unit, and failure contingency, at both
per-target and per-campaign scale. Every row in the committed
`generated/wp6_cost_summary.csv` (schema 1.1, WP14 rows) carries a source
citation or is marked PLACEHOLDER — never silently filled.

The **CU→currency anchor** is derived at runtime from a 5-component core of
that itemized table (development + manufacturing_bus_unit +
kit_unit_scaled_9t × baseline-N-kits + launch + operations_3yr), reported as
a low/mid/high range — never a point-value dollar figure. Two flagged rows
in the current itemized table are honestly non-authoritative: an SSA/tracking
subscription cost that vendors decline to quote publicly (marked
PLACEHOLDER) and a 9-t-class installer-kit unit cost for which no citable
product exists at this scale (marked as a training-data extrapolation).

**Current computed anchor values, campaign totals, and cost-per-removal
ranges are not repeated here** — they already have one home (R16): see
[evidence/adsc_evidence_pack.md](../evidence/adsc_evidence_pack.md) section 5
("Absolute cost ranges (WP14)") and the WP14 rows of
`generated/wp6_cost_summary.csv` directly, or the README's own Key-results
digest for the current headline figures.

**Honesty framing**, quoted verbatim from `generated/wp6_cost_schema.md`
(never hand-retyped, so this document and the schema cannot drift apart):

> External absolute-cost estimation by an unaffiliated author is inherently low-standing; agencies re-run with their own parametric models; the relative CU results remain primary. No point-value currency figure is emitted: absolute costs appear ONLY as low/mid/high cited ranges.

Tug-architecture cost comparison is only performed on the **same target
set** (comparing different targets is meaningless and banned) — see the
Class C discussion in [docs/target_selection.md](target_selection.md).

## What this cost model does not claim

- No absolute-dollar point value is ever emitted by the CU core model;
  currency figures appear only as cited low/mid/high ranges (D10).
- No absolute minimum-cost claim — cost-effectiveness (cost per removal, cost
  per unit risk reduction) is the defensible framing throughout.
- The FoM band-priority disagreement (T5) is not resolved by this model; it
  is a metric-choice fact, reported rather than adjudicated.
- Legal accessibility is a gate, never a scoring multiplier, in any figure
  here (D12) — see [docs/legal_regulatory.md](legal_regulatory.md).
