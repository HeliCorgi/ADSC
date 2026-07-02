# ADSC v4 — Active Debris Self-Cleanup

A C++17 GNC **numerical simulator** for an installer-type active-debris-remediation
(ADR) servicer, framed as **Kessler-precursor removal**: the collisional cascade's
fuel is the population of massive derelict upper stages in congested orbital bands —
fragments are the symptom — so the servicer targets the objects that would become
the next fragment clouds, attaches a passive deorbit kit, and departs. ADSC is an
**open, reproducible evidence package** for that architecture (spec:
`adsc-specification-v4.md`), not flight software and not a mission proposal. It
assumes the operator is, or is contracted/consented by, the launching state of the
target; nothing in this repo assumes or enables unconsented approach to another
state's object. Governing value: **claims must match implementation** — every
number in this README is regenerable by running committed code.

**v4 status (work-package based):** WP1 (relative orbital motion + passive
safety), the F1/F2 honesty follow-ups, WP2 (tumble synchronization), and WP3
(attach event + kit decay trades, with the mission reflowed into
approach→sync→attach→depart phases) are implemented on top of the v2.0 GNC
core. WP4–WP7 (estimator/sensors, campaign Monte-Carlo, cost model, evidence
pack) are **not yet** implemented — see the roadmap below.

**What changed from v1.21 → v2.0:** the v1.21 README named an SR-UKF and a
sliding-mode DACS, but the source only declared unused state and printed status
lines — no estimator, no control law, no dynamics loop. v2.0 replaced the prints
with an actual closed-loop simulation and stated plainly what is and isn't
implemented; v3 continues that discipline.

## What is actually implemented

- **TMR fuel store** (`fuel_store`): three redundant copies guarded by a real
  CRC32 (IEEE, over the full 8-byte value via `memcpy`), majority voting, and
  scrubbing. Recovers single- and double-copy SEU flips and flags an
  unrecoverable triple failure instead of returning a silent bad value. Unit
  tested (`tests/test_fuel_store.cpp`).
- **Rigid-body attitude dynamics** (`dynamics`): quaternion kinematics + Euler's
  equation with a full inertia tensor, integrated with fixed-step RK4.
- **Sliding-mode attitude controller, tracking form (WP2)** (`controller`): a
  quaternion sliding surface on the *relative* motion, `s = w_e +
  λ·sign(q_e0)·q_ev` with `w_e = w − C(q_e)ᵀ·w_t`, boundary layer (saturation
  instead of `sign` to suppress chatter), per-axis DACS deadband and torque
  clamp, and target-motion feedforward: the target's torque-free acceleration
  `ẇ_t = I_t⁻¹(−w_t × I_t w_t)` plus the transport term `−w_e × C(q_e)ᵀw_t`.
  Regulation (v2 detumble) is the `w_t = 0` special case and its code path is
  kept verbatim so the quoted regression numbers cannot drift; the equivalence
  is asserted in `tests/test_sync.cpp`.
- **Tumble synchronization (WP2)** (`run_tumble_sync`): closed-loop sync
  against a torque-free tumbling target with deliberately asymmetric inertia
  (so the rate vector precesses — an isotropic target would tumble at constant
  rate and make tracking trivially easy). Acceptance — |w_rel| < 0.1 deg/s and
  attitude error < 2° held 30 s — is asserted in `tests/test_sync.cpp`, plus a
  **feedforward-honesty test** that disables the reaching term (k = 0) so SMC
  robustness cannot mask a wrong feedforward: on a perfectly synchronized start
  the sliding variable must stay parked near zero with nothing to rescue it.
- **Point-mass inertia update** on capture (parallel-axis), with numerical
  regularization before inversion.
- **Relative orbital motion** (`relmotion`, WP1): Clohessy-Wiltshire (Hill)
  linear dynamics about a circular target orbit in the LVLH frame, with **both**
  an analytic state-transition matrix and a fixed-step RK4 integrator
  (cross-validated against each other). Includes drift-free "safety ellipse"
  construction and a passively-safe approach-corridor generator. Unit-tested,
  including a ≥100-point thrust-off coast that stays outside the keep-out sphere
  over ≥2 orbital periods (D5 passive safety).
- **CW safe-abort with capped-impulse honesty (F1)**: `compute_safe_abort`
  returns a Clohessy-Wiltshire impulse toward a drift-free relative orbit (a
  bounded safety ellipse) through the current position, capped at the thruster
  budget. The result carries a `Clean`/`Capped` status plus the **verified**
  minimum range of the propagated post-burn coast — when the cap binds, the
  bounded-orbit guarantee is lost and the code reports the actual coast range
  instead of implying safety. Unit-tested, including a capped case.
- **Installer mission flow (WP3)** (`run_installer_mission`): the mission runs
  as approach → sync → attach → depart. Approach verifies the passively-safe
  corridor; sync gates on WP2; **attach** clamps at the gated closing speed and
  hands the deorbit kit over (the servicer loses the kit mass, the target gains
  kit mass + sail area, changing its A/m); depart transfers to a bounded,
  keep-out-clearing relative orbit. The attach reports the **contact-energy
  budget** ½·m·v² at `max_v_rel` (≈ 0.33 J for the 29.6 kg servicer at
  0.15 m/s) — the no-fragmentation-at-contact argument for a compliant,
  geometry-keyed clamp. Unit-tested (`tests/test_mission.cpp`). The v2
  `post_capture_stabilization` is retained only as the detumble regression
  reference; the installer paradigm does not detumble the multi-tonne target.
- **Kit deorbit-decay trades (WP3)** (`decay`): a quasi-circular drag-decay
  integrator for `da/dt = −ρ(h)·C_d·(A/m)·√(μa)` over a Vallado exponential
  atmosphere (cross-checked against its constant-density closed form), plus a
  first-order EDT parametric knob. Produces the sail-area × altitude ×
  solar-activity → decay-years trade table for the catalog presets, always as a
  solar-min..max **range** (T4), with the sail area required for the 25-year
  guideline. The honest result — sail-only on a ~9 t stage at ~840 km needs an
  impractically large sail while the lighter/lower class can close — is the
  deliverable that motivates trade T1. Unit-tested (`tests/test_decay.cpp`).
- **First-order PCM thermal budget** integrated over the control loop.
- **Deorbit gating**: autonomous by default, human-in-the-loop only on the
  emergency path, blocked below the fuel reserve.

The simulation driver (`adsc_sim`) runs a real post-capture detumble: it seeds a
tumble, closes the control loop at dt = 0.01 s, and reports the settling time and
final body rate — so "stable at dt = 0.01 s" is now something you can run and
verify rather than a claim in prose.

## What is explicitly NOT implemented (honest scope)

- **No state estimator.** There is no UKF/SR-UKF, no MAGNAV, no sensor model.
  The controller runs on the true state from the simulator. Adding an estimator
  is the obvious next step.
- **Continuous-torque DACS approximation**, not a discrete-impulse thruster
  allocator with real minimum-impulse-bit quantization.
- **Sync-hold relies on the continuous-torque approximation** (WP2 observation):
  the fine firing deadband (3×10⁻⁴ on the sliding variable) that bounds the
  sync-hold error assumes continuous torque; a real minimum-impulse-bit DACS
  would either chatter around that deadband or need reaction wheels to hold it.
- **Thermal model is a single lumped PCM bucket** — no eclipse/sunlight
  radiative balance, no per-node conduction.
- **No closed-loop rendezvous guidance yet.** WP3 reflows the mission into
  approach→sync→attach→depart phases, but there is still no translation guidance
  law that actively flies the approach to contact — approach safety is a passive
  corridor check and attach happens at the gated closing speed. The
  v_rel ≤ 0.15 m/s limit is enforced as a gate, not produced by guidance, and
  all states (including the target attitude/rate the sync loop consumes) are
  truth from the simulator — no estimator/sensors until WP4.
- **Decay model is first-order and single-object** (WP3): quasi-circular drag
  decay over an exponential atmosphere with a single altitude-independent
  solar-cycle factor (a coarse proxy — the real swing is altitude-dependent);
  no J2, no attitude-dependent ballistic area, no re-entry demisability model
  (that safety/cost conflict is open trade T2). The EDT branch is a parametric
  placeholder, not a tether performance model.
- **Passive-safety claims are model-scoped (F2).** The keep-out and
  safety-ellipse guarantees are exact only in the linear Clohessy–Wiltshire
  model: circular target orbit, no J2, no differential drag, small separations.
  J2 and differential drag erode drift-free safety ellipses over time, and the
  CW linearization error grows with separation — so every passive-safety number
  in this README holds in the model, not in the real environment. A real
  mission would re-verify all coasts against a higher-fidelity propagator.
- Numbers for mass, inertia, power, PCM capacity and the target orbit/keep-out
  are plausible placeholders, not derived from a specific bus or target design.

## TRL assessment (honest)

This is a **software prototype at roughly TRL 3–4**: the algorithms run in a
self-contained numerical simulation. The earlier "TRL 5–6" framing was not
supportable from code that did no computation. WP1 adds runnable relative-motion
kinematics and a passive-safety check, WP2 a tracking controller demonstrated in
simulation, and WP3 an attach/decay trade study — but the TRL is **unchanged**:
the control loops still consume true states, the decay/attach models are
first-order, and reaching TRL 5–6 would need at minimum a state estimator, a
sensor/actuator error model, and hardware-in-the-loop testing.

## Build

Requires a C++17 compiler and Eigen 3.3+.

```
git clone https://github.com/HeliCorgi/ADSC.git
cd ADSC
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # add -DADSC_WERROR=ON for R3
cmake --build build
./build/adsc_sim
ctest --test-dir build      # fuel-store + relmotion + sync + decay + mission (fail in any build type)
```

## Layout

```
include/adsc/   public headers (fuel_store, dynamics, controller, thermal,
                relmotion, decay, mission)
src/            implementations + main.cpp simulation driver
tests/          fuel-store + relmotion + sync + decay + mission unit tests
.github/        CI (Ubuntu + Eigen: cmake build + ctest, warnings-as-errors)
adsc-specification-v4.md   active spec (work packages, hard rules, locked decisions)
```

## Roadmap (v4 work packages — see `adsc-specification-v4.md`)

- **WP1 — Relative orbital motion + passive safety** ✅ implemented.
- **F1/F2 — Capped-abort honesty + model-scope note** ✅ implemented.
- **WP2 — Tumble synchronization** ✅ implemented (tracking SMC with torque-free
  feedforward against a precessing target).
- **WP3 — Attach event + kit decay trades** ✅ implemented (contact-energy
  budget; sail vs EDT decay trade with honest negative results).
- **WP4 — Estimator + sensor models** (control on estimates, not truth).
- **WP5 — Campaign Monte-Carlo** (dispersions; success / abort / keep-out rates).
- **WP6 — Parametric cost model** (relative units; the amortization curve).
- **WP7 — Evidence pack** (generated English report; the actual product).

Reproducible WP1 numbers (regenerate with `./build/adsc_sim`): for the 825 km
reference orbit, mean motion n ≈ 1.03×10⁻³ rad/s and period ≈ 6084 s; the closest
thrust-off approach across the sampled corridor is ≈ 424 m — comfortably outside
the 200 m keep-out sphere.

Reproducible WP2 numbers (regenerate with `./build/adsc_sim`, scenario 5):
against a 2.0 deg/s precessing tumble (target inertia ratios 1.0 / 0.6 / 0.3)
from a 40° attitude offset at zero rate, sync is achieved at **16.87 s** and the
criteria then hold for the 30 s dwell; after the dwell the errors stay at
max |w_rel| = 0.0213 deg/s (tol 0.1) and max attitude error = 0.0985° (tol 2°),
ending the 120 s run at 0.00043 deg/s and 0.098°. The feedforward-honesty bound
(sliding variable with the reaching term disabled, limit 5×10⁻³) is asserted and
printed by `./build/test_sync`.

Reproducible WP3 numbers (regenerate with `./build/adsc_sim`, scenario 6): the
installer mission reaches all phases (approach corridor closest 424.3 m → sync
at 16.87 s → attach → depart, bounded coast 400 m) for a catalog-A-mass target,
with a **contact-energy budget of 0.333 J** (29.6 kg servicer at 0.15 m/s) — the
no-fragmentation-at-contact argument. The sail-only decay trade to the 180 km
handoff (range = solar max .. solar min) makes the honest structure concrete:

| target | 100 m² sail | 1000 m² sail | sail area for 25 yr |
|---|---|---|---|
| A: SL-16, ≈9 t, 840 km | 33.7 .. 538.8 yr | 3.4 .. 53.9 yr | **135 .. 2155 m²** |
| B: SL-8, ≈1.4 t, 750 km | 1.8 .. 28.8 yr | 0.2 .. 2.9 yr | **7 .. 115 m²** |

Sail-only closes for the lighter/lower class B (tens of m²) but **not** for the
heavy, high stage A (hundreds-to-thousands of m² — impractical): the honest
negative that brackets open trade T1 (drag sail vs electrodynamic tether). The
EDT parametric study spans, e.g., 36.1 yr at 50 m/day down to 1.2 yr at
1500 m/day of along-track decay.

## Disclaimer / 免責

Conceptual prototype for peaceful ADR research and education only. Real flight
hardware requires formal design review, qualification, and compliance with
international space law (IADC guidelines, UN COPUOS). No military or re-entry-body
application is intended.

本プロジェクトは平和的デブリ除去技術の教育・研究目的の概念プロトタイプです。実際の
宇宙機運用・打ち上げには専門機関による設計審査・認定・国際法遵守が必要です。兵器・
高速再突入体への使用は一切想定していません。
