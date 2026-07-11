<!--
ADSC pitch deck — Markdown slide source.
Rendering stays manual (spec §6, WP15): pass this file through whatever slide
tool the presenter prefers (Marp, pandoc -t revealjs, or a plain read-through);
no rendering pipeline is committed here. Slides are separated by `---`.
Every safety/cost claim keeps its [L#] fidelity tag and its source pointer —
do not strip them when adapting slides for a live talk.
-->

# ADSC
### Kessler-precursor removal — an installer-architecture evidence package

Not a debris collector. Not flight software. Not a mission proposal.
TRL 4, GNC software element only.

*Reproduce every number: `bash tools/regenerate_all.sh build` — byte-identical
from a clean clone.*

---

## The problem

The collisional cascade's fuel is the population of **massive intact
derelicts** in congested orbital bands — fragments are the symptom, not the
cause. One intact-intact collision (Iridium 33 / Cosmos 2251, 2009) produced
2,201 cataloged fragments (Liou 2014, NASA ODPO). Removing, or equipping for
removal, the objects that would become the *next* fragment cloud attacks the
source term. That is what **Kessler-precursor removal** names — full
citations in [concept.md](concept.md).

---

## The installer concept

Not a tug. A small servicer:

1. Rendezvous with a massive derelict upper stage (class-level target, open
   literature, no live ephemerides — D11).
2. Attaches a **passive** deorbit kit (drag sail or electrodynamic tether)
   at a geometry-keyed capture feature present by construction (nozzle
   throat / adapter ring, D4) — no generic manipulator.
3. Departs. The kit, not the servicer, carries the deorbit propellant cost
   (drag / electrodynamic drag).
4. One mission services **several** targets in one plane.

Deorbit Δv scales with the mass decelerated — a tug must decelerate the full
multi-tonne stage; the installer decelerates nothing but its own small kit
transfer. Mechanism detail: [gnc.md](gnc.md),
[technical_architecture.md](technical_architecture.md).

---

## The argument that survives every placeholder

Batch amortization across N targets per mission drives cost-per-removal down
until the mission **Δv budget**, not the kit count, caps removals — a
mass-ratio scaling argument, not a point estimate. It holds regardless of
which cost coefficient or target mass is still an open PLACEHOLDER. Measured
figure: README **Key results** / [cost_model.md](cost_model.md).

This is the lead argument precisely *because* it is placeholder-independent
— everything downstream (absolute costs, FoM weightings, EDT performance) is
honestly still open, and is presented as such later in this deck.

---

## How the evidence is built

- C++17 GNC simulator: rigid-body dynamics, sliding-mode attitude control,
  translation EKF + attitude MEKF, one code path across fidelity levels
  (L0 CW → L1 +J2 → L2 +drag; no fork).
- Monte-Carlo campaign engine: N = 500 dispersed missions per catalog, fixed
  master seed, Wilson 95% CIs — never bare point estimates.
- Every number in this deck traces to a committed, regenerable artifact:
  `tools/regenerate_all.sh` reproduces `generated/` and `evidence/`
  byte-for-byte; CI enforces it on every push.
- Claim discipline: every safety/performance claim carries a fidelity tag
  (R14, L0…L6); the claims-audit test fails on an unaudited headline claim.

---

## GNC evidence snapshot

- Attitude sync demonstrated truth-driven **[L0]** and re-demonstrated
  estimate-driven under sensor noise **[L0, estimator-in-the-loop]**, with an
  NIS/NEES watchdog that rejects covariance-inflation fakes.
- Guided approach with reachability-screened aborts, retained as an
  independent gate (not guidance alone) **[L0]**.
- Estimation gap reported honestly, not hidden: under an unestimated
  range-bias walk, the filter is measurably *optimistic* about its own
  accuracy **[L4]** — the identified fix (bias states) is not yet
  implemented.
- Continuous-torque idealization caveat **retired by measurement**: sync
  still completes under a minimum-impulse-bit actuator with command delay
  and a single-axis fault **[L5]**.

Full numbers and per-channel breakdown: [gnc.md](gnc.md).

---

## The safety case

**Passively-safe approach design with clearance-verified aborts
(keep-out violations 0/500 per catalog [L0, ds-v1, Wilson ≤ 0.0076])** —
never "strict approach safety."

- History (R15): a pre-WP11 baseline showed a nonzero violation rate,
  archived at `v0.10-phase0-baseline`; WP11's clearing-abort law closed it.
- Re-verified at **L1** (+J2) and **L2** (+drag): zero violations over every
  re-verified abort event at both higher levels — a negative-negative
  result, reported with the same discipline as a positive one.
- All guarantees are model-scoped. A real mission re-verifies every coast
  against a higher-fidelity propagator before flight.

Full table: [safety.md](safety.md).

---

## Kit trade — the honest negative first

- **~9 t reference class:** sail-only does **not** close the IADC 25-year
  guideline (impractical sail area). An electrodynamic tether is carried as
  an honest **candidate with open risks**, not a closed recommendation
  (libration trade T7 open; plasma-density inputs PLACEHOLDER).
- **~1.4 t reference class:** sail-only **closes** at a practical area —
  no tether needed.
- **A third, separate class** (defunct massive sun-synchronous payload) is
  evaluated on controlled-reentry terms, not kit-ranked against the two
  above — kit-only deorbit is explicitly not the recommended path for it.
- WP14 prioritization ranks the heavy class first on figure-of-merit under
  both weightings evaluated, with a one-paragraph defense per class.

Full trade: [target_selection.md](target_selection.md),
`generated/wp14_prioritization.md`.

---

## Cost and figure of merit

- Primary metric: **relative cost units (CU)**. "Minimum cost" as an
  absolute claim is **banned** — the defensible claim is
  **cost-effectiveness: cost per removal, cost per unit risk reduction.**
- Amortization curve bottoms at a specific kit count (Δv-limited, not
  monotonic) — the honest capacity story.
- Figure of merit reported under **two independent congestion weightings**
  that can disagree on band priority (open trade T5) — kept visible, not
  resolved by fiat.
- WP14 absolute-cost ranges: cited **low/mid/high**, never a point value;
  every itemized row sourced or explicitly PLACEHOLDER (D10). The external
  currency anchor is explicitly framed as low-standing next to an agency's
  own parametric model.

Full model: [cost_model.md](cost_model.md).

---

## What is NOT claimed

- **Not** TRL 5+; system-level TRL is not claimed at all. WP9
  (processor-in-the-loop) is reserved and the only path above TRL 4.
- **Not** flight software, **not** a mission proposal, **not** legal advice.
- **Not** a solved EDT performance case (libration T7 open); **not** a
  solved absolute-cost case (ranges only, D10).
- **Not** validated against a real environment: linear-CW safety statements
  hold in the stated models (L0/L1/L2), not the real one.
- **140 PLACEHOLDER marks remain (56 decision-critical)** — inventoried
  automatically, none hidden. [limitations.md](limitations.md).

---

## Roadmap

WP1–WP8 baseline → WP10 forensics → WP11 safety hardening (0/500 keep-out) →
WP12 fidelity ladder (L0–L2, L4/L5 elements) → WP13 kit trade + EDT physics →
WP14 cost ranges + FoM → **WP15, this package**. WP9
(processor-in-the-loop, the only path to TRL 5) remains reserved, not
started. Detail: [roadmap.md](roadmap.md).

---

## The ask

This is offered as a **citable, reproducible reference** for the ADR
research and concept-study community first; adoption by an agency
mission-design process is the aspirational case, not a claimed outcome.

- **Reproduce it yourself:**
  `git clone https://github.com/HeliCorgi/ADSC.git && cd ADSC && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build && bash tools/regenerate_all.sh build`
- **Read the honest scope first:** [limitations.md](limitations.md), evidence
  pack §10 (PLACEHOLDER inventory).
- **Engage:** open an issue or discussion on
  <https://github.com/HeliCorgi/ADSC> — no other channel by design.
