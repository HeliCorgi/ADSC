# ADSC v3 — Active Debris Self-Cleanup

A C++17 GNC **numerical simulator** for an installer-type active-debris-removal
(ADR) servicer for large derelict upper stages. This is a **conceptual prototype
for education and research**, not flight software. Governing value: **claims must
match implementation** — every number in this README is regenerable by running
committed code.

**v3 status (work-package based):** WP1 (relative orbital motion + passive
safety) is implemented on top of the v2.0 GNC core. WP2–WP5 (tumble sync,
attach/kit-decay, estimator/sensors, campaign Monte-Carlo) are **not yet**
implemented — see the roadmap below.

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
- **Sliding-mode attitude controller** (`controller`): a genuine quaternion
  sliding surface `s = w + λ·sign(q₀)·q_v` with a boundary layer (saturation
  instead of `sign` to suppress chatter), plus a per-axis DACS deadband and
  torque clamp.
- **Point-mass inertia update** on capture (parallel-axis), with numerical
  regularization before inversion.
- **Relative orbital motion** (`relmotion`, WP1): Clohessy-Wiltshire (Hill)
  linear dynamics about a circular target orbit in the LVLH frame, with **both**
  an analytic state-transition matrix and a fixed-step RK4 integrator
  (cross-validated against each other). Includes drift-free "safety ellipse"
  construction and a passively-safe approach-corridor generator. Unit-tested,
  including a ≥100-point thrust-off coast that stays outside the keep-out sphere
  over ≥2 orbital periods (D5 passive safety).
- **CW safe-abort**: `compute_safe_abort` returns a Clohessy-Wiltshire impulse
  that places the servicer on a drift-free relative orbit (a bounded safety
  ellipse) through the current position, capped at the thruster budget, so a
  thrust-off coast stays clear of the target.
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
- **Thermal model is a single lumped PCM bucket** — no eclipse/sunlight
  radiative balance, no per-node conduction.
- **No closed-loop rendezvous guidance / tumble sync yet.** WP1 delivers CW/Hill
  relative dynamics plus *passively-safe* hold ellipses and an approach corridor,
  but no guidance law flies the approach to contact and no tumble
  synchronization exists (WP2). The v_rel ≤ 0.15 m/s capture limit is still
  enforced as a gate, not produced by guidance, and the relative state is treated
  as truth (no estimator/sensors until WP4).
- **No attach/kit-decay model** (WP3): no drag-sail/EDT deorbit modelling.
- Numbers for mass, inertia, power, PCM capacity and the target orbit/keep-out
  are plausible placeholders, not derived from a specific bus or target design.

## TRL assessment (honest)

This is a **software prototype at roughly TRL 3–4**: the algorithms run in a
self-contained numerical simulation. The earlier "TRL 5–6" framing was not
supportable from code that did no computation. WP1 adds runnable relative-motion
kinematics and a passive-safety check, but the TRL is **unchanged** — reaching
TRL 5–6 would still need at minimum a state estimator, a sensor/actuator error
model, and hardware-in-the-loop testing.

## Build

Requires a C++17 compiler and Eigen 3.3+.

```
git clone https://github.com/HeliCorgi/ADSC.git
cd ADSC
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # add -DADSC_WERROR=ON for R3
cmake --build build
./build/adsc_sim
ctest --test-dir build      # fuel-store + relmotion unit tests (fail in any build type)
```

## Layout

```
include/adsc/   public headers (fuel_store, dynamics, controller, thermal,
                relmotion, mission)
src/            implementations + main.cpp simulation driver
tests/          fuel-store + relmotion unit tests
.github/        CI (Ubuntu + Eigen: cmake build + ctest, warnings-as-errors)
```

## Roadmap (v3 work packages)

- **WP1 — Relative orbital motion + passive safety** ✅ implemented.
- **WP2 — Tumble synchronization** (tracking SMC against a tumbling target).
- **WP3 — Attach event + kit decay model** (drag-sail / EDT trade).
- **WP4 — Estimator + sensor models** (control on estimates, not truth).
- **WP5 — Campaign Monte-Carlo** (dispersions; success / abort / keep-out rates).

Reproducible WP1 numbers (regenerate with `./build/adsc_sim`): for the 825 km
reference orbit, mean motion n ≈ 1.03×10⁻³ rad/s and period ≈ 6084 s; the closest
thrust-off approach across the sampled corridor is ≈ 424 m — comfortably outside
the 200 m keep-out sphere.

## Disclaimer / 免責

Conceptual prototype for peaceful ADR research and education only. Real flight
hardware requires formal design review, qualification, and compliance with
international space law (IADC guidelines, UN COPUOS). No military or re-entry-body
application is intended.

本プロジェクトは平和的デブリ除去技術の教育・研究目的の概念プロトタイプです。実際の
宇宙機運用・打ち上げには専門機関による設計審査・認定・国際法遵守が必要です。兵器・
高速再突入体への使用は一切想定していません。
