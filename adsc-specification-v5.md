# ADSC v5 Specification — Safety Hardening, Fidelity Ladder, and Proposal Readiness

Status: ACTIVE (supersedes adsc-specification-v4.md, revision v4.2)
Owner: HeliCorgi
Audience of this file: coding agents (Claude Code main thread + project subagents)
working on the ADSC repository.

v5 changes vs v4.2:
- Adopts the owner's improvement plan (safety hardening, closed-loop rendezvous
  guidance, fidelity ladder, kit-class strategy, EDT physics, cost ranges,
  proposal packaging) as work packages WP10–WP15.
- New hard rules: R14 (model-scoped claims), R15 (pin re-baselining protocol),
  R16 (no duplicated numbers in docs/).
- Phase 0 (WP10) defined: spec adoption, citation-fill, keep-out-violation
  forensics — no behavior-changing code.
- WP9 (PIL / flight-representative software) remains RESERVED and is the only
  path to TRL 5. Nothing in WP10–WP15 changes the TRL.
- All v4.2 content (D1–D11, R1–R13, T1–T6, §4 FoM) carries over except where
  explicitly amended below.

---

## 1. Project identity (carried, with language discipline)

ADSC is an open-source, reproducible **evidence package** for installer-type
active debris removal, framed as **Kessler-precursor removal**: instead of
towing large debris or chasing fragments, a small servicer installs a passive
deorbit kit on the massive derelicts that would otherwise become the next
fragment clouds, and departs. The audience is mission designers at the agencies
and contractors of major launching states. The product is credibility; the
success criterion is that a skeptical engineer can clone the repository and
regenerate every claimed number (demonstrated: `tools/regenerate_all.sh`,
byte-identical across three independent platforms).

The owner's one-line goal for v5: **maximum orbital-environment cleanup value
per unit cost, with approach safety treated as a design requirement rather
than a statistic.**

Language discipline (binding on README, docs, evidence pack):
- The project anchor phrase "Kessler-precursor removal" appears in all summary
  texts.
- Until the WP11 completion criterion is met, approach safety is described as
  **"passively-safe approach design"** — never "strict approach safety" or
  "safe satellite", because the current campaign shows a nonzero keep-out
  violation rate. Upgrading this wording is itself an R15-documented change.
- "Minimum cost" as an absolute claim is banned. The defensible claim is
  **cost-effectiveness: cost per removal and cost per unit risk reduction.**

Approved summary blurb (EN), to be used verbatim or trimmed, never inflated:
> ADSC is not a debris collector. It is a low-cost concept for
> Kessler-precursor removal: a small servicer installs a passive deorbit kit
> on high-risk massive derelicts and departs, preventing future fragment
> clouds before they form — maximizing cleanup value per unit cost with a
> passively-safe approach design.

Japanese equivalent may accompany it in owner-facing docs; adoption-facing
deliverables remain English (R11).

---

## 2. Verified baseline (WP1–WP8 complete)

State at v5 adoption (master after PR #14):
- Modules: fuel_store (TMR/CRC32), dynamics (RK4), controller (regulation +
  tracking SMC with torque-free feedforward and transport term), relmotion
  (CW STM + RK4 cross-check, safety ellipse, corridor, capped-abort honesty),
  estimator (translation EKF + MEKF, NEES/NIS consistency), mission
  (approach→sync→attach→depart), decay (Vallado Table 8-4, closed-form-tested
  integrator), campaign MC (N=500×2, Wilson intervals, deterministic seeding),
  cost model (amortization curve, tornado, two-weighting FoM), T6 flux_sweep,
  visualization pack (stdlib SVG), compliance matrix generator (rule engine,
  BLOCK on unconsented ADR), evidence pack (417 lines, zero hand-written
  numbers, claims-audit tests).
- Reproducibility: `bash tools/regenerate_all.sh build` regenerates every
  artifact; CI gate `git diff --exit-code -- generated/ evidence/`;
  demonstrated byte-identical on Windows, CI Linux, and a third container.
- Pinned reference numbers (R15 protects all of these and every number quoted
  in README/evidence): 19.15 s detumble; 424.3 m WP1 coast; 16.87 s sync;
  17.07 s estimate-driven sync; NIS/NEES bands; 0.333 J contact; 0.285×
  amortization at N=4; decay trade rows (e.g., 135–2155 m² / 7–115 m²);
  campaign rates as committed. Machine-readable pin list:
  generated/reference_metrics.csv.
- Honest open items: 95 PLACEHOLDER constants (auto-inventoried);
  5 [CITATION NEEDED] items in the evidence pack; keep-out violation rate
  0.014 [0.007, 0.029] per catalog; campaign sensor dispersions drawn but
  not re-propagated; EDT is a parametric knob; TRL 4 (GNC software element,
  laboratory/simulation environment).

---

## 3. Locked decisions

D1–D11 carry over from v4.2 unchanged (installer-not-tug; class-level catalogs;
sail/EDT kits with T2 open; geometry-keyed clamp; passively-safe trajectories;
tumble sync before contact; COTS+TMR; honesty discipline; operator-legality
framing; evidence discipline; dual-use guardrail). New:

- **D12. Legal accessibility is a gate and metadata flags, never a multiplier
  in the FoM.** The compliance engine (WP8) already blocks unconsented ADR;
  target prioritization consumes its PASS/BLOCK output and flags. Rationale:
  T5 already shows metric-choice sensitivity; folding a subjective legal
  weight into a product would multiply arbitrariness and damage credibility.
- **D13. Keep-out violation is a design-unacceptable failure condition**, not
  merely a reported statistic. The known primary mechanism (from F1/WP5
  semantics: capped-abort residual drift re-entering the keep-out sphere under
  dispersed initial conditions) is treated as a hypothesis to be confirmed by
  WP10 forensics and eliminated by WP11 design, not assumed.

---

## 4. Figure of merit (amended)

The v4.2 two-weighting FoM stands (spatial-density and criticality-class
weightings, both reported, disagreement tracked as T5). Amendments:
- Legal accessibility enters as a **gate + flags** (D12), not a factor.
- "Expected fragments prevented" is realized by **strengthening the citation
  basis of the criticality weighting** (e.g., published most-concerning-object
  rankings), not by implementing a homebrew breakup model.
- Cost side comes from WP14 ranges; headline metrics: cost per removal and
  cost per risk-equivalent-mass unit, each as p05/p50/p95 and low/mid/high.

---

## 5. Hard rules

R1–R13 carry over verbatim from v4.2 (no silent behavior changes; README
parity; zero warnings; tests fail in every build type; TMR non-atomic;
reproducible numbers; one WP per PR; conventions; Eigen+stdlib for sim code
and Python-3-stdlib-only for tooling; PLACEHOLDER discipline; English
deliverables; generated-not-hand-edited with SHA gate; dual-use guardrail).
New:

- **R14 — Model-scoped claims.** Every safety or performance claim in README,
  docs/, or the evidence pack carries an explicit fidelity-level tag
  (L0…L6, defined in WP12). Example: "keep-out violations: 0 of 500
  [L0: linear CW, dispersion set ds-v1, Wilson 95% upper bound 0.0076]".
  Claims without a level tag fail the claims-audit test.
- **R15 — Pin re-baselining protocol.** Quoted numbers may change only when
  the PR body contains a BEFORE/AFTER table, the physical/algorithmic reason,
  and the evidence pack gains a changelog entry archiving the old value and
  its supersession. Silent updates to make tests pass are banned. Legacy GNC
  pins (19.15 / 424.3 / 16.87 / 17.07) remain regression-tested for as long
  as their code paths exist. Dispersion sets are version-tagged
  (`dispersion_set_id`, stamped into campaign CSVs; additive schema change →
  schema minor version bump with schema.md and consumer updates in the same
  PR so the reproducibility gate stays green).
- **R16 — No duplicated numbers in docs/.** Files under docs/ may state
  numbers only via generated includes or explicit references to evidence
  artifacts ("see evidence pack §5"). The claims-audit banned-word and
  numeric-consistency scans extend to docs/.

---

## 6. Roadmap and work packages

Numbering: WP9 = PIL/flight-representative track, **RESERVED, unchanged, the
only path to TRL 5**. The improvement plan maps to WP10–WP15. One WP per PR
(R7); the CI-hardening mini-WP may run in parallel any time after WP10.

### WP10 — Phase 0: adoption, citations, forensics (no behavior change)
a. **Spec adoption**: add this file at repo root; retire v4.2 with a pointer
   (spec-only PR, no code).
b. **Citation-fill**: resolve the 5 [CITATION NEEDED] items with web-verified
   primary sources. Candidate directions (verify before inserting; anything
   unverifiable stays PLACEHOLDER rather than being fabricated):
   catalog-class parameters → the published "50 statistically-most-concerning
   derelict objects in LEO" ranking; Iridium-33/Cosmos-2251 catalogued
   fragment statistics; the specific ESA Annual Space Environment Report
   edition for the ~800–900 km environmental-index peak; MASTER-8 release
   reference; aged-MLI low-velocity damage threshold (may legitimately remain
   open — keep the "budget does not claim to be below threshold" phrasing).
c. **Keep-out violation forensics**: from the committed
   wp5_campaign_runs.csv, re-run the violating seeds with a read-only
   analysis tool; classify each against the capped-abort-residual-drift
   hypothesis (initial relative position/velocity, capped vs clean abort,
   corridor deviation, abort timing); write
   generated/wp10_violation_forensics.md (+ CSV) through the normal
   regeneration pipeline. No control-law changes in this WP.
Completion: forensic classification of all violating runs; replay recipe
(seed + config) documented; citation-fill PR merged or items explicitly left
open with reasons.

### WP11 — Phase 1: safety hardening + closed-loop rendezvous guidance
Design goal: make D13 true at L0 before climbing the fidelity ladder.
- **Reachability-based screening / safe-set**: never enter states from which
  a capped abort cannot clear the keep-out sphere (this generalizes F1's
  honesty into a constraint). Barrier-function or explicit reachable-set
  check; boring and testable preferred over elegant.
- **Multi-stage abort** and `abort_dv` treated as a design variable (trade
  documented).
- **Closed-loop translation guidance**, extending (not replacing — R1) the
  existing approach→sync→attach→depart phase structure: far approach →
  hold point → sync → final approach → contact → retreat, with per-phase
  entry/exit/abort conditions, Δv/fuel in transition logic, and contact-speed
  limit satisfied by guidance rather than only gated. Recommended baseline:
  V-bar/glideslope CW-impulse guidance with a reachability check at every
  hold point. This closes the long-standing §1 known limit "v_rel is enforced
  as a gate, not produced by guidance".
- Optional: line-of-sight cone constraint on final approach.
Completion criteria:
- Every WP10-classified violation mechanism is addressed by a named design
  element; before/after campaign comparison committed.
- Keep-out violations: 0 of ≥500 per catalog at L0 under dispersion set
  ds-v1, quoted **with the Wilson 95% upper bound** (a zero still carries an
  interval) and the R14 level tag.
- Mode machine diagram + per-mode conditions documented; abort feasibility
  demonstrated from every mode; failure-case replay retained in the campaign
  report.

### WP12 — Phase 2: fidelity ladder
Levels (each a build-selectable propagation option, not a fork):
L0 CW (baseline, kept) → L1 +J2 numerical propagation → L2 +altitude/solar
drag with ballistic-coefficient uncertainty → L3 +SRP, scoped honestly (the
dominant effect is post-departure target+sail decay and attitude coupling —
state which is verified) → L4 navigation-error extensions (dropout, bias
walk) → L5 actuator model (thrust delay, **minimum impulse bit — this
finally retires the continuous-torque deadband caveat**, saturation,
misalignment, single-fault case) → L6 Monte Carlo + deterministic worst-case
search (fixed seeds, R6).
- **Cross-validation rule (mandatory, tested)**: each level with its new
  physics disabled reproduces the previous level's pinned numbers within a
  stated ε.
- Safety claims re-issued per level with R14 tags; a CW-safe-but-L2-unsafe
  case, if found, is a first-class deliverable (negative results welcome).
Completion: ladder implemented through at least L2 with L4/L5 elements;
per-level comparison table in the evidence pack; "which model guarantees
what" section written.

### WP13 — Phase 3: kit-class trade study + EDT physics
Target classes: A low-altitude/light (sail; required area, deployment
reliability, 25-yr and FCC-5-yr feasibility) · B high-altitude/medium (EDT,
sail-hybrid) · C massive/high-risk (controlled reentry as a separate mission
class; casualty risk, footprint, Δv, consent, cost — comparison against tug
architectures allowed here only, same-target-set rule per R-cost below).
EDT minimum physics model (replacing the parametric knob):
- tether length/mass parameterization; current model; simplified geomagnetic
  interaction with **inclination-dependent v×B efficiency (mandatory — at
  71° the efficiency loss is first-order; omitting it overstates EDT for
  catalog A)**; drag force; power budget; deployment-failure modes;
  tether-cut secondary-debris risk.
- **Libration/dynamic stability is explicitly unresolved**: model it as an
  open risk with literature pointers; never claim it solved.
- Plasma density as a cited parameter (no IRI implementation — out of scope).
Completion: per-class recommended kit table; sail-only-infeasible targets
honestly classified; sail/EDT/hybrid comparison (mass, time, risk) feeding
WP14; EDT deorbit-time calculable with stated uncertainty.

### WP14 — Phase 4: cost ranges + FoM upgrade
- Absolute cost as **low/mid/high ranges only**, itemized (development,
  manufacturing, launch/rideshare, ground segment, ops, SSA/tracking,
  insurance/licensing, kit unit, failure contingency, per-target,
  per-campaign). **Every row carries a source or stays PLACEHOLDER** — this
  is D10 applied, not relaxed. Relative CU results remain primary.
- Tug-architecture comparison only on the **same target set** (comparing
  different targets is meaningless and banned).
- FoM per §4 amendments; target-prioritization table generated; PLACEHOLDER
  burn-down: auto-inventory gains an importance column; decision-critical
  items (target masses/altitudes/inclinations, collision-risk weights, cost
  coefficients, phasing Δv, sensor/actuator uncertainties, contact threshold,
  density/solar model, EDT performance) prioritized; unresolved ones keep an
  explicit impact statement.
Completion: cost-range table with sources; prioritization defensible in one
paragraph per target class; decision-critical PLACEHOLDER count reduced and
tracked in the evidence pack changelog.

### WP15 — Phase 5: proposal package
- README restructured as a short strong entry (What/Why/Concept/Implemented/
  Not-implemented/Key results/Safety status/Reproduce/Structure/Roadmap/
  Limitations/License); details moved to docs/ (concept, technical
  architecture, gnc, safety, target_selection, cost_model, legal_regulatory,
  limitations, roadmap, **pitch_agencies.md** — agency-agnostic by design:
  the project seeds ideas for any launching state, per §1 identity).
- R16 enforced across docs/; claims-audit extended.
- One-page overview, 5-page technical summary, pitch deck source (Markdown;
  rendering may be manual), target-prioritization table, limitations doc.
- Release engineering: version tag; archived, DOI-carrying release (e.g.,
  Zenodo) considered and documented.
- Baseline releases may be tagged at phase boundaries; the DOI-carrying
  archived release remains a WP15 deliverable.
Completion: a reviewer can understand the value in the README's first screen;
strengths and limits equally visible; "what is done vs what is research" is
one glance away.

### CI-hardening mini-WP (parallel, any time after WP10)
GCC+Clang matrix; sanitizers on **test builds only** (artifact generation
stays plain Release — sanitizer-built binaries must never write generated/
or the SHA gate dies); static analysis; coverage **informational, never a
gate**; keep the reproducibility gate the single authority.

---

## 7. Non-goals (carried, updated)
Flight-qualified code (WP9 reserved); plane-change optimization; homebrew
breakup modeling; IRI plasma implementation; absolute point-value cost
claims; nation-specific proposal paperwork; live-TLE ingestion or
target-specific operational products (D11); any claim of system-level or
hardware feasibility; legal or regulatory approval (precheck only).

---

## 8. Open trades
T1–T6 carry over (sail/EDT crossover; controlled vs uncontrolled reentry;
kit mass vs count; solar-activity dispersion; FoM metric sensitivity;
small-debris exclusion, resolved-and-visible). New:
- **T7 — EDT libration stability**: unresolved dynamics risk; tracked with
  literature pointers, never silently closed (WP13).
- **T8 — Contact-damage threshold for aged surfaces**: the 0.333 J budget is
  reported without claiming it is below any validated threshold until a
  citable source exists (WP10b may close it).

---

## 9. TRL statement (binding)
TRL remains **4 (GNC software element, laboratory/simulation environment)**
throughout WP10–WP15. Fidelity upgrades widen the validated envelope at
TRL 4; they do not raise it. TRL 5 requires real-time processor-in-the-loop
work, which is WP9 — reserved, not started. README and evidence pack keep
saying exactly this.

---

## 10. Workflow notes
- One WP per PR; branch `wpN-<name>`; CI green; reproducibility gate green;
  R15 table whenever quoted numbers move.
- Subagent economy (non-normative): delegate inspection, builds,
  regeneration, and routine edits to project subagents; reserve the main
  thread for architecture, derivations (WP11 guidance/safe-set, WP13 EDT
  physics, WP14 FoM), claim adjudication, and final review. The
  adversarial-verifier agent re-derives statistics before any PR that
  touches quoted numbers.
- When this spec conflicts with repository state, say so in the PR instead
  of improvising. Negative results remain first-class deliverables.
