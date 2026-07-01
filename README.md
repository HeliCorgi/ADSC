# ADSC v2.0 — Active Debris Self-Cleanup

A C++17 GNC prototype for a 20 kg-class active-debris-removal (ADR) chaser.
This is a **conceptual prototype for education and research** — a redesign of the
earlier "reality edition" that closes the gap between what the README claimed and
what the code actually did.

**What changed from v1.21:** the previous version named an SR-UKF and a
sliding-mode DACS in the README, but the source only declared unused state and
printed status lines — there was no estimator, no control law, and no dynamics
loop, so no claim about stability could be checked. v2.0 replaces the prints
with an actual closed-loop simulation and states plainly what is and isn't
implemented.

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
- **Guarded safe-abort**: repulsive (radial) + closing-velocity-cancellation
  impulse, both terms protected against near-zero geometry (no NaN escape).
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
- **No orbital/relative-motion dynamics** (no CW/Hill propagation, no guidance
  loop to *achieve* v_rel ≤ 0.15 m/s — that limit is enforced as a gate, not
  produced by guidance).
- Numbers for mass, inertia, power and PCM capacity are plausible placeholders,
  not derived from a specific bus design.

## TRL assessment (honest)

This is a **software prototype at roughly TRL 3–4**: the algorithms run in a
self-contained numerical simulation. The earlier "TRL 5–6" framing was not
supportable from code that did no computation. Reaching TRL 5–6 would need at
minimum a state estimator, a sensor/actuator error model, and hardware-in-the-loop
testing.

## Build

Requires a C++17 compiler and Eigen 3.3+.

```
git clone https://github.com/HeliCorgi/ADSC.git
cd ADSC
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/adsc_sim
ctest --test-dir build      # runs the fuel-store unit tests
```

## Layout

```
include/adsc/   public headers (fuel_store, dynamics, controller, thermal, mission)
src/            implementations + main.cpp simulation driver
tests/          fuel-store unit tests
```

## Disclaimer / 免責

Conceptual prototype for peaceful ADR research and education only. Real flight
hardware requires formal design review, qualification, and compliance with
international space law (IADC guidelines, UN COPUOS). No military or re-entry-body
application is intended.

本プロジェクトは平和的デブリ除去技術の教育・研究目的の概念プロトタイプです。実際の
宇宙機運用・打ち上げには専門機関による設計審査・認定・国際法遵守が必要です。兵器・
高速再突入体への使用は一切想定していません。
