# ADSC v4 Specification — Open Reference Evidence Package for Minimum-Cost Debris Remediation

Status: ACTIVE, revision v4.1 (supersedes adsc-specification-v3.md)
Date: 2026-07-02
v4.1 changes: Kessler-precursor framing (§1, §4); cascade source-term argument
added to WP7.2; contact-energy acceptance line added to WP3; new T6
(small-debris exclusion with committed flux calculation); §7 updated to match.
Owner: HeliCorgi
Audience of this file: coding agent (Claude Code) working on https://github.com/HeliCorgi/ADSC

---

## 1. Project identity — READ THIS FIRST, IT CHANGED

ADSC is an **open-source, reproducible evidence package** for installer-type
active debris remediation. The owner has **no intention of flying it**. The
product is **credibility**: a package convincing enough that mission designers
at the agencies and contractors of major launching states (US, Russia, China,
ESA, JAXA, ...) look at it and think *"low cost, safe — worth prototyping."*

Mission framing: **Kessler-precursor removal.** The collisional cascade's fuel
is the population of massive derelicts in congested bands; fragments are the
symptom. One intact-vs-intact collision produces thousands of trackable
fragments, so removing (or equipping for removal) the objects that would
become the next fragment clouds is the highest-leverage intervention per cost.
ADSC does not chase fragments — see T6 for the documented physics of why not.

Why that audience: under the Outer Space Treaty, jurisdiction and ownership of
an orbital object remain with its launching state indefinitely. The only actors
who can legally remediate the large legacy debris are those states and their
contractors. Conveniently, the states with the worst debris record also own the
most legally-touchable targets. The evidence package therefore speaks to them,
in their terms, with their debris classes as reference cases.

Success criterion for the whole project: **a skeptical engineer can clone the
repo on a clean machine, build it, and regenerate every claimed number in under
one hour.** Anything that does not survive that test does not ship.

What ADSC is NOT: flight software, a mission proposal for any specific nation,
or a claim of system-level feasibility. The claim is narrower and defensible:
the GNC core and the architecture trades are runnable, tested, and honest.

The single most important project value is unchanged: **claims must match
implementation.**

---

## 2. Verified baseline (v2.0 + WP1, commit `f6ca7f3`, master)

Layout:

```
include/adsc/   controller  dynamics  fuel_store  mission  relmotion  thermal
src/            matching .cpp + main.cpp (simulation driver)
tests/          test_fuel_store.cpp  test_relmotion.cpp
.github/workflows/ci.yml   CMakeLists.txt
```

Verifiably implemented (all regenerated from a clean clone, g++ 13, Eigen 3.4,
`-Wall -Wextra -Wpedantic`, zero warnings, tests pass in Release/`-DNDEBUG`):

- `fuel_store` — TMR scalar store: 3 copies, per-copy CRC32 (IEEE reflected,
  0xEDB88320, full 8-byte double + version via `memcpy`), median-3 vote,
  scrub-on-read, conservative min-of-survivors fallback, `Unrecoverable`
  status, `inject_bitflip` hook. Deliberately **not** `std::atomic` (no
  lock-free 128-bit CAS on RAD5545-class PPC; TMR defends SEUs, not races).
- `dynamics` — quaternion kinematics + Euler's equation, full inertia tensor,
  fixed-step RK4.
- `controller` — quaternion sliding-mode: `s = w + λ·sign(q_e0)·q_ev`,
  boundary layer, per-axis deadband + torque clamp. Currently **regulation
  only** (fixed target attitude, zero target rate).
- `relmotion` (WP1) — Clohessy–Wiltshire model: analytic STM (verified against
  the closed form term-by-term), RK4 propagation cross-validated against the
  STM, drift-free safety ellipse (`vy = −2n·x` by construction), approach
  corridor generator, safe abort re-expressed as a CW impulse onto a bounded
  relative orbit.
- `mission` — closing-speed gate, parallel-axis inertia update, closed-loop
  detumble at dt = 0.01 s.

Reference numbers (must remain regenerable, R6):

- Detumble: |w| 0.2121 → 0.000361 rad/s, settled 19.15 s.
- WP1 passive safety: 120 thrust-off coast samples over 2 orbital periods,
  worst closest approach 424.3 m > 200 m keep-out.
- README TRL statement: 3–4, explicitly unchanged by WP1. Keep it that way
  until an estimator exists.

Known WP1 limits → fixed first as F1/F2 (see §5).

---

## 3. Locked architecture decisions — do not relitigate

An agent may flag a concern in a PR note, but must not silently implement
something else.

- **D1. Installer, not tug.** The servicer attaches a deorbit kit and departs;
  it never tows. Deorbit Δv scales with combined mass, so a small servicer
  towing multi-tonne stages does not close physically. The evidence pack must
  make this argument quantitatively (WP7).
- **D2. Reference target catalogs per launching state (REVISED).** Targets are
  **class parameters**, presented so each potential adopter sees its own
  debris. Presets (all fields configurable):
  - `catalog_A` — SL-16 / Zenit-2 second-stage class: ≈9 t, ≈840 km, ≈71°.
  - `catalog_B` — SL-8 / Kosmos-3M second-stage class: ≈1.4 t, 750–1000 km,
    74–83°.
  - `catalog_C` — CZ upper-stage class: mass/altitude/inclination
    `PLACEHOLDER` — fill from public debris-ranking literature with citation.
  - `catalog_D` (optional) — US Delta-class stage: `PLACEHOLDER`, same rule.
  One mission = one orbital plane, multiple targets (batch amortization).
- **D3. Kits:** drag sail (lower LEO) / electrodynamic tether (higher LEO).
  Controlled-reentry variant is open trade T2 (demisability of steel/Ti
  components); surface it, never silently resolve it.
- **D4. Capture interface:** geometry-keyed clamp on nozzle throat / adapter
  ring — features guaranteed on upper stages — not a generic manipulator.
- **D5. Passively-safe relative trajectories.** From any nominal-approach
  point, a thrust-off coast must not enter the keep-out sphere. Autonomous
  abort available in every phase.
- **D6. Tumble synchronization before contact.** Contact only after sync
  criteria hold for a dwell time.
- **D7. Avionics philosophy: COTS + TMR/FDIR**, not full rad-hard. TMR stays
  non-atomic; threading assumptions documented at call sites.
- **D8. Honesty discipline.** README implemented / NOT-implemented lists and
  the TRL statement updated every WP. Negative results are first-class
  deliverables.
- **D9. Operator-legality framing (REVISED — was a constraint, now the
  targeting rationale).** The package assumes the operator is, or is
  contracted/consented by, the launching state of the target. State this in
  the README and the evidence pack. Nothing in the repo may assume or enable
  unconsented approach to another state's object.
- **D10. Evidence discipline (NEW).** All adoption-facing deliverables are in
  English. Every figure and table in the evidence pack is regenerated by a
  committed target. Reproduction time on a clean machine: under one hour.
- **D11. Dual-use guardrail (NEW).** Methods stay at open-literature level
  (CW, sliding mode, EKF are textbook material). Targets remain class-level
  parameters: **no mission-planning products against specific catalog IDs and
  no ingestion of live ephemerides/TLEs.** The peaceful-use disclaimer stays
  in the README.

---

## 4. Figure of merit (NEW)

The headline metric is **debris-risk reduction per cost**:

```
FoM = Σ_i ( m_i · w(h_i) ) / C_campaign
```

where `m_i` is removed mass and `w(h)` a normalized congestion weight for the
target band (from public spatial-density data; `PLACEHOLDER` table, cite the
source when filling). The FoM is deliberately a **cascade-fuel metric**:
removed mass in a congested band is a proxy for future fragments prevented
(§1, WP7.2). Honesty requirements:

- Metric choice affects rankings. Report FoM under **at least two weightings**
  (e.g., spatial density and a published criticality-style ranking) and track
  the disagreement as open trade T5.
- `C_campaign` comes from the WP6 parametric model in **relative units**;
  absolute currency appears only as cited ranges (see WP6).

---

## 5. Work packages

Order: **F1+F2 → WP2 → WP3 → WP4 → WP5 → WP6 → WP7.** WP6 may start any time
after WP3. WP7 is last. One WP per branch/PR (`wp2-sync`, ...), CI green
before merge. F1+F2 may share one small PR (`wp1-followups`).

### F1 — Capped-abort honesty (small, do first)
`compute_safe_abort` caps |Δv| at `Config::abort_dv`; when the cap binds, the
along-track drift is not fully nulled and the bounded-orbit guarantee is lost.
Return a status (`{clean, capped}`); when capped, propagate the residual coast
and report/verify the actual minimum range instead of implying safety. Extend
`test_relmotion` with a capped case.

### F2 — Model-scope note (same PR)
README known-limits addition: passive safety is exact only in the linear CW
model; J2 and differential drag erode safety ellipses over time; all
passive-safety claims are model-scoped. One honest paragraph, no hedging fog.

### WP2 — Tumble synchronization (tracking control)
- Target: torque-free tumble via existing `RigidBody` (default 2 deg/s,
  range 0.5–5 deg/s, configurable).
- Extend the SMC from regulation to **tracking**: `q_e = q_t* ⊗ q`,
  `w_e = w − C(q_e)·w_t`, surface `s = w_e + λ·sign(q_e0)·q_ev`, target-motion
  feedforward. Public API change → R1 note. Regulation must remain as the
  `w_t = 0` special case; the v2 detumble scenario stays as a regression.
  (Convention: `C(q_e)` is the textbook reference→body DCM, so `C(q_e)·w_t`
  rotates the target rate into the servicer body frame. In the codebase's Eigen
  Hamilton quaternions — mapping body→inertial, with `q_e = q_t* ⊗ q` — that
  rotation is `q_e.conjugate() * w_t`, so the WP2 implementation writes
  `w_e = w − q_e.conjugate()·w_t`: the same quantity.)
- **Acceptance:** |w_rel| < 0.1 deg/s and attitude error < 2° held 30 s
  (defaults, configurable) against a tumbling target; asserted in a test with
  fixed constants.

### WP3 — Attach event + kit decay trades
- Attach after sync dwell: servicer loses kit mass/inertia; target gains kit
  mass and area (A/m changes).
- Decay: quasi-circular `da/dt = −ρ(h)·C_d·(A/m)·√(μa)`, piecewise-exponential
  atmosphere (values cited in comments), **solar min/max as a dispersion**.
- EDT: first-order thrust/efficiency placeholder, marked (R10).
- **Acceptance:** committed target emits the trade table (sail area × altitude
  × solar activity → decay years) for each catalog preset. The expected honest
  outcome — **sail-only at 800–850 km may exceed the 25-year guideline** — is
  a deliverable that motivates the EDT branch (T1), not a bug.
- **Acceptance (contact honesty):** the attach event reports the contact
  kinetic energy at the gated closing speed (½·m·v² at `max_v_rel`; ≈0.33 J
  for the reference 29.6 kg at 0.15 m/s), so the evidence pack can cite a
  no-fragmentation-at-contact budget and the compliant-clamp rationale (D4).
  Grabbing a decades-degraded surface hard enough to shed MLI or paint flakes
  would manufacture new debris; the budget line exists to show we costed that.

### WP4 — Estimator + sensor models
- Range + bearing sensor (10 Hz, Gaussian noise + bias placeholders);
  gyro/star-tracker abstraction for own attitude.
- EKF on relative translation + relative attitude. Control consumes
  **estimates, not truth**; truth kept only for error reporting.
- **Acceptance:** WP2 sync criteria met closed-loop under defined noise at
  fixed seeds; estimation-error statistics reported.

### WP5 — Campaign Monte Carlo (extended)
- Dispersions: initial relative state, tumble rate/axis, sensor noise/bias,
  actuator errors, atmosphere (solar activity).
- **Campaign layer:** N targets in one plane per mission; inter-target
  phasing modeled as a parameterized Δv/time cost (`PLACEHOLDER`, documented
  as simplified — no plane-change optimization, §7).
- ≥ 500 runs, fixed seeds. Outputs per catalog preset: removals/mission,
  Δv + kit usage, success / abort / keep-out-violation rates (violation rate
  reported even if zero). Feeds WP6.

### WP6 — Parametric cost model (NEW)
```
C_campaign = C_dev + C_bus(m_dry) + N·C_kit + C_launch(m_wet, band) + C_ops(T)
```
- Primary output in **relative units**. Absolute currency only as ranges
  anchored to cited public figures; **no point-value dollar claims.**
- Required outputs, per catalog preset: the **amortization curve**
  (cost per removal vs N targets/mission — the core installer argument),
  tornado sensitivity of FoM/cost to each parameter, FoM under ≥2 weightings
  (§4).
- All parameters in a config struct, placeholders marked (R10).

### WP7 — Evidence pack (NEW, the actual product)
A generated English report under `evidence/`, produced by a committed target
(Markdown; every figure/table regenerated, never hand-edited). Contents:

1. Executive summary (one page: what, for whom, headline FoM/cost curves).
2. Architecture + the quantitative why-installer argument (Δv scaling, D1),
   plus the **cascade source-term argument**: removing massive derelicts from
   the peak-risk band attacks the fuel of the Kessler cascade, not the
   symptom. Cite at fill time: the classical few-removals-per-year
   stabilization result (Liou & Johnson class), ESA's environmental-index
   peak near ~850 km / 70–80° inclination (external validation of
   `catalog_A`), and MASTER-8 population figures.
3. Safety case: WP1 coast statistics, abort coverage **including capped
   cases (F1)**, WP5 keep-out-violation rates.
4. Decay trades incl. the honest negative results (WP3).
5. Campaign statistics + cost/FoM per catalog preset (WP5/WP6).
6. Limitations, stated plainly: linear CW scope (F2), no plane-change
   optimization, estimator fidelity, placeholder-marked parameters, and the
   small-debris exclusion (T6) with its committed flux calculation.
7. **Flight-software migration path — written annex, deliberately NOT
   implemented:** HAL layer, allocation-free control path, fixed-rate
   scheduler + WCET hooks, mode machine (SAFE/HOLD/APPROACH/SYNC/ABORT)
   FDIR, telemetry/command dictionary. Rationale: adopters rewrite flight
   code anyway; what they cannot cheaply redo is validated architecture
   trades. Spending effort there is the minimum-cost allocation.
8. Reproduction instructions (clean machine → all numbers, < 1 hour, D10).

**Acceptance:** a fresh clone regenerates the entire pack with one documented
command sequence; CI builds the pack or verifies its generator runs.

---

## 6. Hard rules

R1–R10 carry over from v3 verbatim in spirit:

- **R1** No silent behavioral changes (`BEHAVIOR CHANGE:` note in PR body).
- **R2** README parity every WP (implemented / NOT lists + TRL).
- **R3** Zero warnings (`-Wall -Wextra -Wpedantic`); no UB; `memcpy` for byte
  views; no type-punning casts.
- **R4** Tests fail in every build type (explicit `return 1`, no bare
  `assert`). Already migrated; keep it that way.
- **R5** TMR stays non-atomic; threading assumptions documented.
- **R6** Every quoted number regenerable by a committed target, fixed seeds
  where stochastic.
- **R7** One WP per PR, branch `wpN-<name>` / `wp1-followups`, CI green.
- **R8** SI units; frames documented (ECI, LVLH x radial / y along-track /
  z cross-track, body); Eigen quaternion convention stated once (Hamilton,
  storage x,y,z,w — use named accessors).
- **R9** Dependencies: Eigen + standard library only; anything else needs an
  approval note before use.
- **R10** Placeholders marked `PLACEHOLDER`, centralized in config structs,
  never bare literals in logic.

New:

- **R11** All adoption-facing deliverables (README, evidence pack) in
  English. Internal comments may stay terse but must be English too.
- **R12** The evidence pack is generated, never hand-edited; its generator is
  a committed, CI-verified target. (Extension of R6.)
- **R13** Dual-use guardrail (enforces D11): no specific-object operations
  products, no live-ephemeris ingestion, methods at open-literature level.
  If a WP seems to require crossing this line, stop and flag it instead.

---

## 7. Non-goals

Flight-qualified code (annex-only, WP7.7), RTOS/scheduling, thruster hardware
allocation beyond the deadband/clamp abstraction, multi-node thermal networks,
plane-change/phasing optimization (parameterized cost only), direct
small-fragment (1–10 cm) removal (see T6), absolute cost
prediction, nation-specific proposal paperwork, and any claim of system-level
or hardware feasibility. The README must keep saying so.

---

## 8. Open trades — keep visible, never silently resolve

- **T1** Sail vs EDT crossover altitude per catalog preset (WP3 output).
- **T2** Controlled vs uncontrolled reentry, driven by demisability — a
  safety/cost conflict; surfaced in the evidence pack.
- **T3** Kit mass/volume vs targets per mission (Δv + inventory).
- **T4** Solar-activity dispersion on decay times — always ranges, never
  point values.
- **T5** FoM metric sensitivity: rankings under different congestion/
  criticality weightings (§4).
- **T6 (resolved, kept visible so adopters see it was considered, not
  missed).** Direct removal of 1–10 cm fragments is **out of scope**, and the
  exclusion is evidence, not neglect. Physics: at ~10 km/s the specific
  kinetic energy is ~50 MJ/kg (~12× TNT); a 1 cm aluminum sphere (~1.4 g)
  carries ~71 kJ (~17 g TNT equivalent). No material "catches" a cm-class
  hypervelocity impactor intact — shields work by shattering and spreading,
  and shed secondary ejecta themselves. Flux: with the ≥1 cm population
  (~1.2 M objects, MASTER-8 class figures, cite at fill time) even a 100 m²
  collector in a peak band intercepts on the order of one cm-class object per
  ~3 years; removing 1 %/yr of the population needs km²-scale collection area
  absorbing grenade-class hits without shedding — while itself being the
  largest collision cross-section in the band. **Deliverable:** a small
  committed target (`flux_sweep`, C++ like everything else, R9) regenerates
  this table (specific energies, hit rates per collector area at
  average/peak densities, area required for 1 %/yr removal); the evidence
  pack's limitations section cites it. Laser photon-pressure/ablation nudging
  is noted as the research lane for that population — reference only, no
  implementation.

---

## 9. Workflow notes for the agent

- Read this file fully before touching code. If it conflicts with repository
  state, say so in the PR instead of improvising.
- F1/F2 come first; they are small and unblock honest safety claims that WP7
  will later cite.
- `Mission::post_capture_stabilization` carries v2 "capture" semantics; from
  WP2 onward the flow becomes approach → sync → attach → depart. Rename/
  refactor is expected and falls under R1. Keep the v2 detumble scenario and
  the WP1 coast statistics alive as regression outputs — they are quoted
  reference numbers (§2).
- Prefer boring, checkable code over cleverness. When a WP's honest result is
  negative, report it prominently — in this project, a well-documented dead
  end is a deliverable, and the evidence pack exists to show adopters we do
  not hide them.
