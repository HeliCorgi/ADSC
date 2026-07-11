# Safety case

This is the full detail behind the README's **Safety status** line. It covers
passive-safety-by-design (D5), the keep-out discipline (D13), the F1/F2
honesty follow-ups, the WP11 clearing-abort law and guided approach, the WP12
fidelity-ladder re-verification, and the full WP5 campaign statistics. The
underlying control-law and estimator math is [docs/gnc.md](gnc.md); the
one-code-path fidelity-ladder architecture is
[docs/technical_architecture.md](technical_architecture.md); everything this
case does **not** cover is [docs/limitations.md](limitations.md).

Every number below regenerates from `generated/reference_metrics.csv`, the
WP5 campaign CSVs, or `generated/wp12_ladder.{csv,md}` — see
[evidence/adsc_evidence_pack.md](../evidence/adsc_evidence_pack.md) section 3
for the same figures with the same citations (R16: this file and the evidence
pack are not allowed to drift apart).

## Design principle: passively-safe trajectories (D5) and the keep-out discipline (D13)

- **D5 — Passively-safe relative trajectories.** From any nominal-approach
  point, a thrust-off coast must not enter the keep-out sphere. Autonomous
  abort is available in every phase.
- **D13 — Keep-out violation is a design-unacceptable failure condition**,
  not merely a reported statistic. The known primary mechanism (capped-abort
  residual drift re-entering the keep-out sphere under dispersed initial
  conditions) was a hypothesis confirmed by WP10 forensics and eliminated by
  WP11 design, not assumed away.

The approach-safety wording is fixed and must not be paraphrased upward or
downward: approach safety is a **passively-safe approach design with
clearance-verified aborts (keep-out violations 0/500 per catalog [L0, ds-v1,
Wilson ≤ 0.0076])** — never "strict approach safety" or "safe satellite". The
history clause: the pre-WP11 baseline showed a nonzero keep-out-violation
rate (0.014 [0.007, 0.029], archived at the `v0.10-phase0-baseline` release);
WP11 closed it to 0/500 (below); WP12 re-verified that result at L1 (+J2) and
L2 (+drag) (below). Any further upgrade of this wording is itself an
R15-documented change.

**Passive corridor result (D5/WP1) [L0: linear CW]:** across **120
thrust-off coasts** sampled along the approach corridor and propagated 2
orbital periods, the worst closest approach is **424.264069 m** (≈424.3 m),
comfortably outside the **200 m** keep-out sphere
(`reference_metrics.csv: wp1_worst_coast_min_range_m, wp1_keep_out_radius_m`).
This holds in the linear CW model only — see the F2 caveat below and
[docs/limitations.md](limitations.md) for the full model-scope discussion.

## F1: capped-abort honesty

`compute_safe_abort` returns a Clohessy-Wiltshire impulse toward a drift-free
relative orbit (a bounded safety ellipse) through the current position,
capped at the thruster budget. The result carries a `Clean`/`Capped` status
plus the **verified** minimum range of the propagated post-burn coast — when
the cap binds, the bounded-orbit guarantee is lost and the code reports the
actual coast range instead of implying safety:

- **Clean case** (contact-range abort): impulse **0.245016 m/s**, full
  drift-null delivered, verified post-burn coast minimum **0.119712 m**.
- **Capped case** (large along-track drift): impulse saturates at the
  **2.0 m/s** thruster budget, the bounded-orbit property is lost, and the
  verified post-burn coast minimum is **3565.717926 m** — reported plainly,
  not implied to be safe.

(`reference_metrics.csv: f1_clean_abort_dv_m_s, f1_clean_coast_min_m,
f1_capped_abort_dv_m_s, f1_capped_coast_min_m`.)

## F2: model-scope caveat (linear CW)

Every passive-safety number in this file is **exact only in the linearized
Clohessy-Wiltshire model**: circular target orbit, no J2, no differential
drag, small separations. J2 and differential drag erode drift-free safety
ellipses over time, and the CW linearization error grows with separation —
so a real mission would re-verify every coast against a higher-fidelity
propagator before flight. The WP12 fidelity ladder (below) is the first
concrete step in that direction, not a substitute for it. Full discussion:
[docs/limitations.md](limitations.md).

## WP11: clearing-abort law + guided approach

`compute_clearing_abort` / `clearing_abort_for` implement a 3-stage
escalation ladder that accepts an abort only when the analytic post-burn
ellipse (`bounded_coast_min_range`, closed form) clears keep-out plus a
design margin: **drift-null baseline** → **bounded radial reshape** →
**two-impulse retreat hop**, with the legacy `Capped` honesty (F1, above)
preserved past the Δv budget. The campaign keep-out screen runs this law
(D13).

**Campaign-level result:** keep-out violations **0 of 500** per catalog
**[L0: linear CW, dispersion set ds-v1, Wilson 95% upper bound 0.0076]** —
the full campaign table is below. The independent replay audit is
`generated/wp11_abort_audit.md`; the archived pre-WP11 forensics (the
legacy-law violation mechanism) are `generated/wp10_violation_forensics.md`.

**Guided-approach demo (L0, truth-fed, deterministic)** — far approach → hold
→ sync → final approach → contact → retreat, with a reachability screen
evaluated before every committed impulse:

| metric | value |
|---|---:|
| reachability screen held at every step | 1/1 (yes) |
| LOS cone held throughout final approach | 1/1 (yes) |
| contact speed (produced by glideslope-with-floor, not merely gated) | 0.100 m/s |
| total demo Δv | 2.533611 m/s |
| retreat Δv (burn1+burn2, +station-keep hop if needed) | 0.403983 m/s |
| min clearance outside the keep-out sphere (pre-authorization flight) | 200.0 m |

(`reference_metrics.csv: wp11_guided_*, wp11_abort_feasible_every_step,
wp11_los_cone_ok, wp11_retreat_dv_total_m_s`; mode machine and per-mode abort
conditions: `generated/wp11_guidance_modes.md`.) This closes the
long-standing "gate, not guidance" limit at L0 — the 0.15 m/s `max_v_rel`
gate is retained as an independent check, not removed.

**Forensic-14 story.** The 14 WP10c/WP11 pinned violation states (R15) are a
regression fixture, not an anecdote: the legacy (pre-WP11) law reproduces all
14 of them analytically as keep-out violations; the WP11 clearing law clears
every one of the same 14 states (`tests/test_forensic14.cpp`). The WP12
fidelity ladder (below) re-checks the same 14 states at L1/L2.

## WP12: fidelity ladder re-verification

Runtime-selectable **L0/L1/L2** propagation on **one code path** (no fork,
R1): **L0** is the original WP1 Clohessy-Wiltshire linearization (every
committed WP5 number stays byte-identical); **L1** differences a full
inertial two-body+J2 RK4 propagation of both craft into the target's
instantaneous LVLH frame; **L2** adds a per-craft free-molecular drag term
under an **independent** ballistic-coefficient dispersion stream (`ds-v2`;
the committed campaign's own `ds-v1` stream is untouched). Cross-validation
(tested, `tests/test_ladder.cpp`): with J2 forced off, L1 reproduces the CW
closed form within a stated **2.0 m** linearization bound (the CW model's own
error, measured at worst 0.52 m across the forensic-14 states); with drag
forced off, L2 reproduces L1 bit-for-bit. L1/L2 numbers are **new claims**
(R14-tagged) that **re-verify** the existing WP11 clearing-abort law under a
higher-fidelity coast — they are not a new abort law.

<!-- DOCS-WP12-LADDER-START (source: generated/wp12_ladder.{csv,md}; copied verbatim from the committed table) -->
Abort-event re-verification (292 catalog-A + 291 catalog-B closing-speed-gate
events from the committed campaign; master seed `0x5AD5C0DECAFE2026`, same
draws as `wp5_campaign_runs.csv` for `ds-v1`):

| level | dispersion set | catalog | n_events | n_violations | violation_rate | wilson 95% CI | clearance floor (m) | p05 (m) | p50 (m) |
|---|---|---|---:|---:|---:|---|---:|---:|---:|
| L0 | ds-v1 | SL-16 / Zenit-2 second stage | 292 | 0 | 0.000000 | [0.000000, 0.012985] | 20.0053 | 36.3253 | 139.7579 |
| L1 | ds-v1 | SL-16 / Zenit-2 second stage | 292 | 0 | 0.000000 | [0.000000, 0.012985] | 19.0070 | 35.8083 | 139.2909 |
| L2 | ds-v2 | SL-16 / Zenit-2 second stage | 292 | 0 | 0.000000 | [0.000000, 0.012985] | 19.0086 | 35.7539 | 139.2424 |
| L0 | ds-v1 | SL-8 / Kosmos-3M second stage | 291 | 0 | 0.000000 | [0.000000, 0.013029] | 20.0002 | 26.5835 | 143.9558 |
| L1 | ds-v1 | SL-8 / Kosmos-3M second stage | 291 | 0 | 0.000000 | [0.000000, 0.013029] | 18.8096 | 26.2484 | 143.0785 |
| L2 | ds-v2 | SL-8 / Kosmos-3M second stage | 291 | 0 | 0.000000 | [0.000000, 0.013029] | 18.8129 | 26.0505 | 142.9937 |

Forensic-14 per-level clearance: all 14 of 14 pinned cases clear keep-out at
**L0, L1 and L2** — the anticipated CW-safe-but-higher-level-unsafe case does
**not** materialize for this dispersion set: the WP11 clearance margin
absorbs the measured ~1.0-1.2 m of J2+drag coast erosion (clearance floor
20.0 m at L0 → 18.8 m at L2, SL-8 class, the tighter of the two). A
negative-negative result, reported with the same discipline as a positive
one. Full 14-row per-case table: `generated/wp12_ladder.md`.

Safety-ellipse margin decay under J2 (the F2 caveat, promised qualitatively
since WP1, now measured): worst 5-orbit min-range erosion **1.40 m** on the
400 m standoff ellipse [L1: two-body+J2]. Full per-orbit table for both
geometries and both catalogs: `generated/wp12_ladder.md`.
<!-- DOCS-WP12-LADDER-END -->

**L4 (estimate-driven guidance) and L5 (actuator realization), honestly
reported:**

- **L4** [L4: L0 dynamics + dropout + bias walk, deterministic seed]: the
  guided approach flies on the translation EKF's estimate (truth is
  error-recording only) under measurement dropout and an **unestimated**
  range-bias random walk. Truth-evaluated contact speed **0.099759 m/s**
  (gate 0.15 m/s), final position error **0.023540 m**. Consistency,
  reported honestly: NIS **3.994325** (ideal ~4, consistent) but NEES
  **310.437825** against an ideal ~6 — the filter is *optimistic* about its
  own accuracy under the unestimated bias walk; adding bias states is the
  identified fix, not implemented here (see
  [docs/limitations.md](limitations.md)).
- **L5** [L5: MIB delta-sigma, 1-step delay, single-axis fault]: tumble sync
  re-demonstrated under a delta-sigma minimum-impulse-bit actuator (MIB
  2×10⁻⁴ N·m·s PLACEHOLDER, 1-step command delay, one axis at 50%
  authority): sync at **16.28 s** vs **16.87 s** under the continuous-torque
  idealization — the earlier crossing is a metric artifact of the
  first-crossing-then-dwell criterion under quantization, not a physical
  speed-up; the claim is only that sync still completes under MIB + delay +
  fault. Guidance contact velocity, quantized at a 1×10⁻³ m/s translation MIB
  PLACEHOLDER, still meets the 0.10 m/s gate.

(`reference_metrics.csv: wp12_est_*, wp12_l5_*`.) Coverage statement (from
`generated/wp12_ladder.md`): this ladder verifies keep-out-abort geometry
re-verification, forensic-14 regression clearance, and min-range erosion —
each tagged by the fidelity level that produced it. It does **not** verify
attitude dynamics beyond WP2's translation-only extension, or the full
closed-loop combination of L4 guidance with L5 actuation. No result here is a
legal, regulatory, or safety conclusion beyond its stated model scope.

## WP5 campaign: the full statistical evidence

The campaign is the safety/statistics story: 500 dispersed missions per
catalog at fixed master seed `0x5AD5C0DECAFE2026`, 6 targets/mission, 4 kits,
140 m/s Δv budget. Outputs are `generated/wp5_campaign_*.csv`, **schema
1.1** (`generated/wp5_campaign_schema.md`) — the schema bumped from 1.0 to
1.1 when WP11 added the `dispersion_set_id`/`worst_abort_clearance_m`
columns; this file states the current version rather than carrying the
stale "1.0" reference forward (a drift the WP15 content-mapping pass found
and fixed, per R16's whole point). Unit-tested
(`tests/test_campaign.cpp`: seed determinism, Wilson edge cases,
percentiles, termination, schema stability, the guardrail that no artifact
emits legal-approval language). Regenerate with `./build/adsc_campaign`.

<!-- DOCS-WP5-FULL-START (source: generated/wp5_campaign_summary.csv, schema 1.1; verbatim from tools/docs/fill_docs_numbers.py build_wp5_full(), CI regenerates this from adsc_campaign, seed 0x5AD5C0DECAFE2026) -->
The full N = 500 campaign (both catalog presets) runs in a few tens of seconds
on a CI runner (see the Actions log of the current run for the actual figure).
`success` here means a **productive end** — the mission installed its
full 4-kit complement (`kit_exhausted`) or cleared all 6 targets (`completed`) —
not one cut short by Δv exhaustion or a keep-out violation.

| metric | SL-16 / Zenit-2 class | SL-8 / Kosmos-3M class |
|---|---|---|
| success rate | **0.556** [0.512, 0.599] | **0.542** [0.498, 0.585] |
| nonproductive-termination rate (= 1 − success) | **0.444** [0.401, 0.488] | **0.458** [0.415, 0.502] |
| gate-abort-run rate (abort-path exposure) | **0.444** [0.401, 0.488] | **0.458** [0.415, 0.502] |
| keep-out-violation rate | **0.000** [0.000, 0.008] [L0, ds-v1] | **0.000** [0.000, 0.008] [L0, ds-v1] |
| Δv used p05/p50/p95 [m/s] | 124 / 124 / 136 | 124 / 124 / 136 |
| kits used p05/p50/p95 | 3 / 4 / 4 | 3 / 4 / 4 |
| removals/mission p05/p50/p95 | 3 / 4 / 4 | 3 / 4 / 4 |
| sync arrival p05/p50/p95 [s] | 14.47 / 17.68 / 20.09 | 14.13 / 17.72 / 20.16 |
| failure counts (runs) | dv_exhausted 222, kit_exhausted 278, keep_out 0, completed 0 | dv_exhausted 229, kit_exhausted 271, keep_out 0, completed 0 |
| per-target events | gate_abort 292, sync_timeout 0 | gate_abort 291, sync_timeout 0 |

Two abort-related rates are reported and are deliberately distinct:
**`gate-abort-run rate`** is the abort-path exposure (fraction of runs with ≥ 1
closing-speed gate abort — what the spec calls the "abort rate"), while
**`nonproductive-termination rate`** is 1 − success. Under the current *flat
PLACEHOLDER* Δv cost the two coincide numerically — every aborting mission needs
an extra target-slot to still install its kits and so exhausts the 140 m/s
budget — but they are separate concepts and will diverge once the cost model
gains structure. The honest campaign finding: the servicer is **Δv-limited about
44% of the time** (`dv_exhausted` = 222/500) and installs its full kit
complement the rest; the attitude sync **never** times out across the sampled
tumble/attitude/actuator dispersions, and keep-out violations are **0 of 500**
per catalog [L0: linear CW, dispersion set ds-v1, Wilson 95% upper bound
0.0076] — the WP11 clearing-abort law accepts an abort only when the analytic
post-burn ellipse clears keep-out plus a design margin (mechanism forensics:
`generated/wp10_violation_forensics.md`; audit:
`generated/wp11_abort_audit.md`). `completed` (all 6 targets) is 0 by
construction — 4 kits cannot service 6 targets. The `Δv used` / `kits used`
percentiles matching across the two presets is expected (a flat PLACEHOLDER cost
takes quantized, catalog-independent values — not a copy-paste bug). Full per-run
records and the column schema are in [generated/](../generated/).
<!-- DOCS-WP5-FULL-END -->

WP5 produces **no** charts and performs **no** legal/regulatory
determination; visualization is [docs/technical_architecture.md](technical_architecture.md)'s
WP7a pack, and the regulatory precheck is
[docs/legal_regulatory.md](legal_regulatory.md).

## Contact-energy budget (D4/WP3, open trade T8)

Clamping at the gated closing speed (**0.15 m/s**, `max_v_rel`) with the
**29.6 kg** dry+kit servicer mass carries **0.333 J** of kinetic energy
(`reference_metrics.csv: wp3_contact_energy_j, wp3_contact_speed_m_s,
wp3_contact_mass_kg`). The budget exists to show the geometry-keyed clamp
(D4) does not depend on aggressive grappling of a decades-degraded surface —
shedding MLI or paint would manufacture the very debris the mission removes.
**Whether this energy is below the actual MLI/paint damage threshold is not
claimed here** — open trade **T8**: the 0.333 J figure is reported without
claiming it is below any validated threshold until a citable source exists
(`[CITATION NEEDED — PLACEHOLDER: low-speed-contact damage-threshold source
for aged MLI/paint]`, evidence pack section 3).

A worst-case figure exists for the residual-propellant condition (heavier
servicer at the same 0.15 m/s gate speed); it is **not** backed by a
committed CSV or generator today, so it is documented as an honest
scope-gap rather than restated here — see
[docs/limitations.md](limitations.md) for the number and its arithmetic
(R16: one home per number).

## What this safety case does not claim

- No claim beyond **L0/L1/L2** model scope for passive-safety numbers, and no
  claim beyond **L4/L5** scope for guidance/actuation honesty numbers — see
  the L-tags above and [docs/limitations.md](limitations.md) for the full
  model-scoping discussion.
- No system-level or flight-readiness claim; TRL stays 4, element-scoped
  (spec section 9; [docs/limitations.md](limitations.md)).
- No claim that the contact-energy budget is below any validated
  material-damage threshold (T8, above).
- No legal or regulatory determination — see
  [docs/legal_regulatory.md](legal_regulatory.md).
