# Roadmap

Not a mission proposal; TRL 4, GNC software element only — this page is the
full work-package history and the full "why this does not raise TRL"
rationale that the README only summarizes. Binding spec:
`adsc-specification-v5.md` (work packages, hard rules R1-R16, locked
decisions D1-D13). WP9 is a fixed point throughout this history: **the
processor-in-the-loop (PIL) / flight-software track remains RESERVED and not
started, and is the only path above element TRL 4.** Nothing in WP10-WP15
changes that.

## History: what changed v1.21 → v2.0

The v1.21 README named an SR-UKF and a sliding-mode DACS, but the source only
declared unused state and printed status lines — no estimator, no control
law, no dynamics loop. v2.0 replaced the prints with an actual closed-loop
simulation and stated plainly what is and isn't implemented; every release
since has kept that discipline (D8: implemented/not-implemented lists and
the TRL statement updated every WP; negative results are first-class
deliverables).

## Work-package history

- **WP1 — Relative orbital motion + passive safety** — done. Clohessy-Wiltshire
  relative motion with an analytic STM + RK4 cross-check, drift-free safety
  ellipse, passively-safe approach corridor.
- **F1/F2 — Capped-abort honesty + model-scope note** — done. Capped-impulse
  aborts report their actual verified coast instead of implying safety when
  the cap binds; passive-safety guarantees stated as model-scoped (linear CW:
  circular orbit, no J2, no differential drag, small separations).
- **WP2 — Tumble synchronization** — done. Tracking sliding-mode control with
  torque-free feedforward against a precessing target, plus a
  feedforward-honesty test.
- **WP3 — Attach event + kit decay trades** — done. Contact-energy budget at
  the gated closing speed; sail vs. EDT decay trade with an honest negative
  result for the heavy class; mission reflowed into
  approach→sync→attach→depart phases.
- **WP4 — Estimator + sensor models** — done. Translation EKF + multiplicative
  attitude EKF; the control loop runs on estimates, not truth; NEES/NIS
  consistency watchdog.
- **WP5 — Campaign Monte-Carlo** — done. Dispersions under a fixed master
  seed; success/abort/keep-out rates with Wilson CIs; Δv/kit/removal
  percentiles; stable machine-readable CSV (schema **1.1** — corrected here;
  the CSV was schema 1.0 at WP5's original landing and was bumped to 1.1 by
  WP11's additive columns, per `generated/wp5_campaign_schema.md`).
- **WP6 — Parametric cost model** — done. Relative-unit `C_campaign`;
  amortization curve; FoM under ≥2 congestion weightings (open trade T5);
  tornado sensitivity (schema **1.1** — corrected here for the same reason as
  WP5, per `generated/wp6_cost_schema.md`).
- **WP7a — Visualization Pack** — done. Python 3 stdlib-only static SVG
  figures + static report page from the committed CSVs; no external
  dependencies.
- **WP8 — Compliance Matrix Generator / Regulatory Precheck** — done.
  Versioned rulepacks; precheck + evidence matrix; explicitly not legal
  advice, no legal-conformity determination.
- **WP7 — Evidence Pack** — done. `evidence/adsc_evidence_pack.md`, generated,
  claim-audited, zero hand-written numbers — the project's actual product.
- **WP9 — Processor-in-the-loop / flight-software track** — **reserved, not
  started.** The only path above element TRL 4; see spec §9 and the
  discussion below.
- **WP10 — Phase 0: adoption, citations, forensics** — done. Spec v5 adopted;
  citation-fill; keep-out-violation forensics
  (`generated/wp10_violation_forensics.md`, the legacy-law archive superseded
  by WP11). No behavior change.
- **WP11 — Safety hardening + closed-loop guidance** — done. Clearing-abort
  law, reachability screen, guided approach; closed the pre-WP11 nonzero
  keep-out-violation baseline to **0 of 500 per catalog** [L0, ds-v1, Wilson
  95% upper bound 0.0076] — the legacy pre-WP11 rate (0.014 [0.007, 0.029])
  is archived at the `v0.10-phase0-baseline` release per R15, not deleted.
- **WP12 — Fidelity ladder** — done. L0/L1/L2 (+drag, +J2) and L4/L5
  (estimate-driven guidance, MIB actuator) elements; zero keep-out violations
  re-verified at L1/L2 [ds-v1/ds-v2] — see `generated/wp12_ladder.md` and
  [safety.md](safety.md) for the numbers.
- **WP13 — Kit-class trade + EDT physics** — done. Aligned-dipole EDT-v1 band
  model with mandatory η(i) = |cos i|..cos²i (derivation:
  `wp13-edt-derivation.md`, citations: `wp13-literature.md`); per-class
  recommended-kit table; class-C controlled-reentry comparison; libration T7
  explicitly **unresolved** — an EDT candidate with open risks, not a closed
  recommendation.
- **WP14 — Cost ranges + FoM upgrade** — **done** (corrected here: the
  pre-WP15 README stated "WP14–WP15 — not started", which was already stale
  by the time of this rewrite). Absolute cost as low/mid/high ranges only,
  itemized and every row sourced-or-PLACEHOLDER; tug-architecture comparison
  restricted to the same target set; target-prioritization table generated
  (`generated/wp14_prioritization.md`); decision-critical PLACEHOLDER count
  tracked in the evidence-pack changelog. Relative CU results remain primary.
- **WP15 — Proposal package** — this release. README restructured to the
  What/Why/Concept/Implemented/Not-implemented/Key results/Safety
  status/Reproduce/Structure/Roadmap/Limitations/License shape; detail moved
  to `docs/` (this tree); R16 enforced across `docs/` via an extended
  claims-audit; one-page overview, 5-page technical summary, pitch-deck
  source, target-prioritization pointer, and this limitations/roadmap pair;
  release engineering (version tag, DOI-carrying archived release or its
  documented deferral) tracked separately — see
  [pitch_agencies.md](pitch_agencies.md) and [limitations.md](limitations.md).

## Why none of this raises the TRL

**TRL 4 — for the GNC software element only.** With WP4 the attitude-sync
GNC element (tracking controller + estimator + sensor models) runs closed
loop **on estimated states** in a laboratory/simulation environment, with
filter consistency verified (NEES/NIS) rather than assumed — that is the
element-level TRL 4 definition; the earlier "TRL 5–6" framing (v1.21) was not
supportable, and WP1–WP3 stayed at TRL 3–4 because control still consumed
true states at that point. Scope caveat, stated plainly: **TRL 4 applies to
the GNC software element, not the system** — system-level TRL is undefined
and not claimed anywhere in this package.

Work package by work package, why the TRL does not move:

- **WP5** adds robustness-under-dispersions evidence — this strengthens the
  TRL-4 evidence base, it does not raise the TRL.
- **WP6** adds a relative-unit cost/FoM model on top of that evidence — an
  economic-viability argument, not a GNC maturity change.
- **WP10** (forensics), **WP11** (safety hardening + closed-loop guidance),
  and **WP12** (the fidelity ladder — higher-fidelity propagation is a wider
  validated envelope, not a maturity change) all widen the validated
  envelope **at** TRL 4; none of them raises it.
- **WP13** (kit-class trade + EDT physics) and **WP14** (cost ranges + FoM
  upgrade) are, respectively, a target/kit-selection argument and an
  economic argument — neither is a GNC maturity change.
- **WP15** (this package) is a packaging/communication exercise — it changes
  how the evidence is presented, not what TRL the evidence supports.

Reaching TRL 5 for the element requires real-time processor-in-the-loop
execution on representative hardware — the WP9 track, reserved, not started,
unchanged by v5 and by every WP since. This is stated once, here, as the
canonical version; the README's Safety status section references it rather
than restating the full rationale.
