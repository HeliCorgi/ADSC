# Digital Twin Phase 1 (WP16)

Not a mission proposal; TRL 4, GNC software element only ([roadmap.md](roadmap.md),
spec section 9). WP16 is an owner-directed extension, not a spec-mandated work
package -- see [roadmap.md](roadmap.md) for how it sits in the work-package
history. **NO real asset exists anywhere in this WP.** Everything below is a
twin-to-twin exercise: a purely simulated lumped-mass EDT tether model stands
in for "Real", and a second, reduced-order model assimilates its noisy
simulated sensor output as "Virtual". Nothing here is a hardware digital twin,
a flight-software element, or a resolution of open trade **T7** (EDT libration
dynamic-stability, `_tasks_local/t7-libration-study.md`) -- T7 stays **OPEN**.

Numbers are not restated in this document (R16): every figure quoted below
lives in `generated/wp16_twin.csv` (schema: `generated/wp16_twin_schema.md`)
and in the evidence pack's "Digital Twin Phase 1 (WP16)" section
(`evidence/adsc_evidence_pack.md`) -- regenerate with `adsc_twin`
(`src/main_twin.cpp`) and see those artifacts for exact values, Wilson 95% CIs
and percentile bands. This page is the qualitative model summary, the honesty
framing, and the cross-reference map; it is not a second home for the pinned
figures.

## Model summary (Deliverables 1-3)

The lumped-mass tether model (`include/adsc/tether.hpp` / `src/tether.cpp`)
is a **planar, Phase-1-only** reimplementation of the T7 dumbbell/multi-bead
geometry:

- **N point-mass beads** (default **8**; `n_beads=2` selects the rigid
  dumbbell-limit configuration used for the Deliverable-4 validation below)
  connected by tension-only spring-dashpot segments, in the linearized
  Hill/LVLH rotating frame (the same radial/along-track sign convention as
  `relmotion.hpp`'s `CwModel`).
- **Slack segments are physical, not an error state**: a segment in
  compression (`l_j < L0`) carries zero tension rather than pushing, tracked
  per-step (`n_slack`, `slack_fraction`) rather than being clamped away
  silently.
- **Aligned-dipole electrodynamic forcing**: the per-segment Lorentz force is
  driven only by the orbit-normal field component `B_n`, which is *constant*
  around an aligned-dipole orbit (`wp13-edt-derivation.md` Sec 2.3) -- the DC
  / in-plane channel. The cross-track components that drive the real Pelaez
  out-of-plane pumping instability (`_tasks_local/t7-libration-study.md`
  Sec 3.5) are **not** integrated in this Phase-1 model (see Phase-2 items
  below) -- a stated, pre-registered model boundary, not an oversight.
- **Fixed-step RK4 integration** with a per-run energy audit (the rotating-
  frame Jacobi integral `E_J = KE_rel + U_gg + U_spring`, tracked against the
  integrated Lorentz + damping power every step and reconciled once per
  orbit) plus three hard divergence guards (`DivergeStatus`): `DivergedAngle`
  (chord angle-from-vertical exceeds the T7 80-deg divergence threshold),
  `DivergedVelocity`, and `Overstrain` (segment strain ratio exceeds 1.5).
- **`EnergySpike` guard, recalibrated.** A fourth guard flags genuine
  integrator blow-ups via a per-step energy-residual check. Its first cut
  false-positived on ordinary slack-segment transients and normal RK4
  discretization noise; per the adversarial review in
  `_tasks_local/wp16_xcheck.py`, it was recalibrated to a warmed-up,
  floor-seeded exponential-moving-average residual gated by *both* a
  relative *and* an absolute threshold (see `include/adsc/tether.hpp`'s
  `DivergeStatus::EnergySpike` comment and `src/tether.cpp`'s guard
  constants) so it trips only on genuine integrator failure, not on the
  model's own ordinary slack/strain dynamics. This recalibration is why the
  Monte Carlo per-status table (evidence pack / `generated/wp16_twin.csv`)
  can attribute non-Ok outcomes to a specific cause instead of one opaque
  "diverged" bucket.

## Twin-to-twin sync frame (Deliverables 6, 7)

**This demonstrates assimilation machinery only.** `include/adsc/twin.hpp` /
`src/twin.cpp` pairs a perturbed-parameter instance of the same bead model
("truth twin", playing the role of "Real") with a 4-state EKF running a
reduced pitch-pendulum-equivalent model ("virtual twin"), estimating
`[theta, thetadot, I_eff, c_hat]` from noisy simulated (angle, tension)
measurements. The virtual twin never sees the truth twin's actual physical
parameters. The headline demo is **virtual-to-real pushback**: for the
controlled cases, the current schedule commanded to the truth twin is
computed from the *virtual* twin's own estimated state, not from the truth
twin's internal state -- "Real runs the schedule Virtual computed." There is
no real asset, no operational twin, and no telemetry pipeline anywhere in
this repository; this is an assimilation-loop feasibility exercise on
simulated data, evaluated for internal consistency (NEES/NIS watchdog,
per-parameter relative error) exactly as the rest of this repository's
estimators are (`docs/gnc.md`).

## Three findings

1. **C2 (fixed-duty) can excite libration; C1 (phase-gated) and the
   uncontrolled baseline do not, in this model.** The Monte Carlo controller
   comparison (Deliverables 5, 7; `generated/wp16_twin.csv`
   `controller_comparison` rows) runs N=200 dispersed trials per controller
   under the Deliverable-7 dispersion set. The fixed-duty (C2) schedule
   produces a materially nonzero combined non-converged rate (every non-Ok
   `DivergeStatus` counted, Wilson 95% CI -- see the CSV / evidence pack for
   the exact rate and interval), while the phase-gated (C1) controller and
   the uncontrolled constant-current baseline do not, at this seed and
   dispersion set. The mechanism read off the model: naive time-based duty
   cycling adds a parametric-forcing line to what would otherwise be (in the
   1-DOF reduced picture) a DC torque, and that forcing can pump libration
   energy depending on switch phase and dispersed initial conditions -- the
   same qualitative mechanism the T7 study's own Sec 5.2/5.4 duty-cycle
   sensitivity already flags. **This is an in-model result under this
   dispersion set only; both controllers remain PROPOSALS, and T7 is not
   resolved by it.**
2. **`c_hat` (the EKF's effective pitch-damping state) is weakly observable;
   `I_eff` is not.** The twin-sync Monte Carlo (`twin_sync` rows) shows
   `I_eff` converging with a small relative error (it enters the pitch
   dynamics directly as the Lorentz torque and moves the angle innovation),
   while `c_hat`'s relative error against the truth twin's axial dashpot
   `c_true` is reported for the record and is **not** expected to be small.
   This is a genuine physical fact, not a filter defect: an axial dashpot
   produces near-zero direct pitch damping in near-rigid rotation, and the
   tension channel that *does* respond to `c_true` is not connected to
   `c_hat` by the measurement model. See the `c_hat_rel_err` note in
   `generated/wp16_twin.csv`, the schema's `OBSERVABILITY NOTE`
   (`generated/wp16_twin_schema.md`), the `TwinSyncReport` finding comment in
   `include/adsc/twin.hpp`, and `_tasks_local/wp16_xcheck.py` for the full
   sensitivity cross-check.
3. **Dumbbell-limit validation agrees with the T7 study, with a stated
   model-family offset.** The N=2 rigid, massless-tether configuration
   (Deliverable 4, target **T4a only**) reproduces the T7 study's own
   bounded/tumble classification at all three cross-checked `eps` cases
   (`_tasks_local/t7-libration-study.md` Sec 5.1). At the bounded case, this
   lumped-mass fixed-step RK4 reimplementation and T7's own 1-DOF adaptive
   DOP853 integration agree on qualitative behavior (bounded) but not on the
   exact peak angle -- an explicit, expected **model-family / integrator
   offset**, not a disagreement about the underlying physics (see the CSV
   note on the `max_angle_deg` row for the exact figures). **T4b** (the 3D
   pitch+roll pumping-onset target) requires the Phase-2 out-of-plane model
   and is explicitly **not attempted** in this Phase-1 planar implementation
   -- stated up front, no overclaim; the Phase-1 planar deliverable is
   validated by T4a alone.

## Controllers are proposals; T7 stays OPEN

Both C1 (phase-gated, bang-bang on the sign of a unit-current Lorentz-power
probe) and C2 (fixed-duty square wave) are **in-model PROPOSALS**, evaluated
only against this simulated planar physics -- neither is a validated,
resolved, or flight-candidate stability mechanism. `eta_libration` /
`duty_on` remains what it has been since WP13: an average-thrust bookkeeping
factor, never a stability margin. Finding 1 above, if anything, sharpens the
T7 caution already on record (`wp13-edt-derivation.md`,
`_tasks_local/t7-libration-study.md`): naive duty-cycling is not a free
substitute for the still-open libration-control problem. **T7 (EDT libration
dynamic-stability trade) remains OPEN.** Nothing in WP16 Phase 1 closes it,
narrows it to a specific control law, or claims active libration control is
solved.

## Phase-2 items (not done here)

- **Out-of-plane (roll) dynamics.** The real Pelaez energy-pumping channel
  (cross-track field components `B_r`, `B_t` driving roll, which couples
  back into pitch) is the mechanism the T7 study identifies as the dominant
  instability risk and this Phase-1 planar model cannot see by construction.
  Adding it is the prerequisite for attempting **T4b**.
- **Flexible tether modes.** The current model is a small (default N=8)
  lumped-mass chain tuned for Monte Carlo tractability under a fixed-step
  RK4 integrator; higher-resolution or genuinely flexible (continuum /
  modal) tether dynamics are not modeled.
- **Real-time / estimate-driven closed-loop realism.** The twin-sync EKF
  runs on simulated, already-generated sensor streams offline; there is no
  real-time execution, no actuator realism (MIB/delay/fault, cf. the WP12
  fidelity ladder's L5 treatment for the rendezvous GNC element), and no
  hardware-in-the-loop path defined for WP16. Any such extension would need
  its own pre-registered plan, on the model of `docs/wp9_pil_plan.md`.

## Cross-references

- Closed-form EDT torque/efficiency derivation this model reuses constants
  and field conventions from: `wp13-edt-derivation.md` (repo root).
- The independent 1-DOF/2-DOF libration study this Phase-1 model is
  cross-checked against, including the ADVERSARIAL CORRECTION governing its
  own headline numbers: `_tasks_local/t7-libration-study.md`.
- Kit-class trade, EDT-v1 model scope, and the T7 open-risk framing at the
  mission level: [docs/target_selection.md](target_selection.md).
- Full pinned WP16 figures, Wilson CIs and percentile bands: the evidence
  pack's "Digital Twin Phase 1 (WP16)" section
  ([evidence/adsc_evidence_pack.md](../evidence/adsc_evidence_pack.md)) and
  `generated/wp16_twin.csv` / `generated/wp16_twin.md` /
  `generated/wp16_twin_schema.md`.
- Adversarial review notes and finding provenance for the EnergySpike
  recalibration and the observability cross-check:
  `_tasks_local/wp16_xcheck.py`.
