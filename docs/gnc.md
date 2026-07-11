# GNC — dynamics, control, estimation, guidance

Not a mission proposal; TRL 4, GNC software element only
([roadmap.md](roadmap.md)). This page owns the **mechanism** of the core GNC
algorithms (WP1 relative motion, WP2 controller + tumble sync, WP4 estimator,
the WP11 guidance-law mechanics) and, per R16, states numbers only via
generated includes, explicit references to [safety.md](safety.md) /
`generated/reference_metrics.csv` / the evidence pack, or — where no CSV
backs a number yet — prose with the exact regeneration command cited. The
campaign-level safety claims these mechanisms produce (keep-out rates,
abort-event statistics, contact-energy budgets) are [safety.md](safety.md)'s
to own; this page never restates them.

## Relative orbital motion (WP1) — `relmotion`

Clohessy-Wiltshire (Hill) linear dynamics about a circular target orbit in
the LVLH frame, with **both** an analytic state-transition matrix and a
fixed-step RK4 integrator, cross-validated against each other. Includes a
drift-free "safety ellipse" construction and a passively-safe
approach-corridor generator; `compute_safe_abort` returns a
Clohessy-Wiltshire impulse toward a drift-free relative orbit through the
current position, capped at the thruster budget, with a `Clean`/`Capped`
status carrying the verified minimum range of the propagated post-burn coast
(when the cap binds, the code reports the actual coast range rather than
implying safety — the honesty argument and its numbers are
[safety.md](safety.md)'s). Unit tested (`tests/test_relmotion.cpp`),
including a ≥100-point thrust-off coast check.

**Reproducible WP1 numbers** — mean motion and orbital period for the 825 km
reference orbit are printed by `./build/adsc_sim` console output only; there
is no committed CSV row backing them yet (a documented gap, not a hidden
one). Regenerate and read them directly: `./build/adsc_sim` (n ≈ 1.03×10⁻³
rad/s, period ≈ 6084 s, at the time of writing — treat the console output as
authoritative, not this sentence). The keep-out-corridor claim itself (worst
coast range vs. the 200 m keep-out sphere) is a safety number owned by
[safety.md](safety.md), which cites `generated/reference_metrics.csv`
(`wp1_coast_samples`, `wp1_worst_coast_min_range_m`, `wp1_keep_out_radius_m`).

## Rigid-body dynamics + sliding-mode controller (WP2) — `dynamics`, `controller`

**Rigid-body attitude dynamics**: quaternion kinematics + Euler's equation
with a full inertia tensor, integrated with fixed-step RK4.

**Sliding-mode attitude controller, tracking form**: a quaternion sliding
surface on the *relative* motion, `s = w_e + λ·sign(q_e0)·q_ev` with
`w_e = w − C(q_e)ᵀ·w_t`, boundary layer (saturation instead of `sign` to
suppress chatter), per-axis DACS deadband and torque clamp, and
target-motion feedforward: the target's torque-free acceleration
`ẇ_t = I_t⁻¹(−w_t × I_t w_t)` plus the transport term
`−w_e × C(q_e)ᵀw_t`. Regulation (the v2 detumble case) is the `w_t = 0`
special case, and its code path is kept verbatim so the quoted regression
numbers cannot drift — the equivalence is asserted in `tests/test_sync.cpp`.

**Tumble synchronization** (`run_tumble_sync`): closed-loop sync against a
torque-free tumbling target with deliberately asymmetric inertia (so the
rate vector precesses — an isotropic target would tumble at constant rate
and make tracking trivially easy). Acceptance — |w_rel| < 0.1 deg/s and
attitude error < 2° held 30 s — is asserted in `tests/test_sync.cpp`, plus a
**feedforward-honesty test** that disables the reaching term (k = 0) so SMC
robustness cannot mask a wrong feedforward: on a perfectly synchronized start
the sliding variable must stay parked near zero with nothing to rescue it.

**Point-mass inertia update** on capture (parallel-axis), with numerical
regularization before inversion.

**Reproducible WP2 numbers** (regenerate: `./build/adsc_sim`, scenario 5;
against a 2.0 deg/s precessing tumble, target inertia ratios 1.0/0.6/0.3,
from a 40° attitude offset at zero rate):

<!-- DOCS-GNC-WP2-START (copied from generated/reference_metrics.csv: wp2_sync_time_s, wp2_max_rate_after_dwell_deg_s, wp2_max_att_after_dwell_deg) -->
| metric | value | tolerance |
|---|---:|---:|
| sync time | **16.87 s** | criteria first held, then dwelled 30 s |
| max \|w_rel\| after dwell | **0.021284 deg/s** | tol 0.1 deg/s |
| max attitude error after dwell | **0.098531°** | tol 2.0° |
<!-- DOCS-GNC-WP2-END -->

The v2 regulation-mode detumble regression reference (a different scenario —
the `w_t = 0` special case, not the tracking sync above) is
`v2_detumble_settle_time_s` — see `generated/reference_metrics.csv`, or the
evidence pack §3, rather than restated here.

Two figures from the same scenario have no committed CSV backing today (a
documented gap, prose-pinned only): the end-of-120s-run figures
(0.00043 deg/s max rate, 0.098° max attitude error — regenerate:
`./build/adsc_sim` scenario 5, read the console tail) and the
feedforward-honesty bound (sliding variable with the reaching term disabled,
limit 5×10⁻³ — this is a test threshold constant in `tests/test_sync.cpp`,
not a CSV row; run `./build/test_sync` to see it asserted and printed).

## Estimator + sensor models (WP4) — `estimator`

The sync control loop consumes **estimates, not truth** — enforced
structurally: the controller is handed an `EstimatedState` only, and truth is
read solely by the sensor models and the error-statistics recorder. Two
decoupled filters (assumptions documented in the header): a **translation
EKF** on the 6-state LVLH relative motion (prediction = the analytic WP1 CW
state-transition matrix; 10 Hz range + line-of-sight **unit-vector**
measurements — az/el is avoided because its Jacobian is singular at
boresight while `d(r/|r|)/dr = (I−r̂r̂ᵀ)/|r|` is regular everywhere), and a
**multiplicative attitude EKF** (3-component small-angle error states,
covariance 3×3/6×6; an additive quaternion EKF is deliberately not used)
with gyro-propagated own attitude + star-tracker updates and a 2 Hz vision
relative-pose channel estimating `q_rel` and the target rate `w_t`.
Covariance updates are Joseph-form + symmetrized; the tests check P symmetry
and positive definiteness over a full run, and a **NEES/NIS consistency
watchdog** (fixed seed, χ² bands) rejects the classic fake of inflating Q/R
until acceptance passes. All randomness is one fixed-seed `mt19937` +
explicit Box-Muller (bit-stable across platforms).

Estimator scope is deliberately narrow: target inertia is assumed **known**
for the torque-free feedforward and the MEKF model; sensor models are
Gaussian abstractions (no outliers, no dropouts in WP4 itself, no
star-tracker occlusion, no vision-pose ambiguity); gyro/rangefinder biases
exist as config knobs but are **not estimated** in the WP4 baseline (the
consistency tests assume the zero defaults). Attitude sync in the campaign
remains the truth-driven WP2 primitive (a documented WP5 simplification —
see [limitations.md](limitations.md)).

**Reproducible WP4 numbers** — sync time, RMS (relative attitude, relative
position), and NIS/NEES (translation channel) are committed in
`generated/reference_metrics.csv` (`wp4_sync_time_s`, `wp4_rms_att_rel_deg`,
`wp4_rms_pos_m`, `wp4_nis_trans`, `wp4_nees_trans`) — see
[safety.md](safety.md) / the evidence pack §3 for the stated values rather
than a second copy here. The **per-channel breakdown** — own-attitude RMS,
target-rate RMS, velocity RMS, star-tracker/vision NIS/NEES, covariance
min-eigenvalue/asymmetry — is the largest documented gap in this package:
none of it is in `reference_metrics.csv` today. Regenerate and read it
directly: `./build/adsc_sim` scenario 7 (fixed seed 20260703), console
output. Extending `reference_metrics.csv`/`sim_metrics` to carry these
figures, or dropping them from any future headline claim, is an open
decision flagged for the owner rather than silently resolved here.

## Guidance mechanics (WP11 law + WP12 L4)

**Clearing-abort law + reachability screen + closed-loop guidance**
(`compute_clearing_abort` / `clearing_abort_for`, `guidance`): a 3-stage
escalation ladder that accepts an abort only when the analytic post-burn
ellipse (`bounded_coast_min_range`, closed form) clears keep-out plus a
design margin — drift-null baseline, bounded radial reshape, two-impulse
retreat hop — with the legacy `Capped` honesty preserved past the Δv budget.
The campaign keep-out screen runs this law (D13); a truth-fed L0
guided-approach demo (far approach → hold → sync → final approach → contact
→ retreat) screens abort feasibility before every committed impulse and
produces the contact speed by a glideslope-with-floor profile rather than
merely gating it. Unit tested (`tests/test_guidance.cpp`) plus the pinned
forensic-14 regression (`tests/test_forensic14.cpp`): the legacy law
reproduces all 14 WP10c violations analytically, the clearing law clears
every one. The campaign-level abort/keep-out statistics this law produces
are [safety.md](safety.md)'s.

**L4 — estimate-driven guidance** (WP12 fidelity-ladder element, mechanism
only; the ladder's own architecture is [technical_architecture.md](technical_architecture.md)'s):
the same guidance mode machine, but with the WP4 translation EKF in the
loop — truth is error-recording only — under measurement dropout and an
unestimated range-bias random walk. This is the honest place the estimator
and the guidance law meet: an unestimated bias walk makes the filter
optimistic about its own error (elevated NEES), which is a first-class
reported finding, not a hidden one — the exact figures live in
[safety.md](safety.md) / the evidence pack §3 (fidelity ladder) rather than
here.

## Subsystem detail

- **First-order PCM thermal budget** integrated over the control loop — a
  single lumped bucket, no eclipse/sunlight radiative balance, no per-node
  conduction.
- **Deorbit gating**: autonomous by default, human-in-the-loop only on the
  emergency path, blocked below the fuel reserve.

Full honest-scope caveats for everything on this page (estimator narrowness,
the retired continuous-torque DACS approximation, model-scoped passive
safety, campaign dispersion scope) are [limitations.md](limitations.md)'s to
own in one place, not repeated here.
