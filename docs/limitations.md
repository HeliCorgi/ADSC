# Limitations (honest scope)

This is the project's one-stop, honest "what is not real yet" list — the
full detail behind the README's **Not-implemented** and **Limitations**
lines. It carries the old README's NOT-implemented bullets forward in full,
the binding TRL statement, the model-scoping summary, a pointer to the
mechanically-collected PLACEHOLDER inventory, and the contact-energy
worst-case caveat with its arithmetic. Per R16, where a number below already
has a home in another docs/ file, this document cross-refers to it instead
of restating it — the qualitative limitation is repeated here (that is the
point of a single honest list), but the pinned figure is not duplicated.

## What is explicitly not implemented

- **Estimator scope is deliberately narrow (WP4).** The target inertia is
  assumed **known** for the torque-free feedforward and the MEKF model —
  exact in this simulation, but a real mission needs inertia identification.
  Sensor models are Gaussian abstractions (no outliers, no dropouts, no
  star-tracker occlusion, no vision pose ambiguity); gyro/rangefinder biases
  exist as config knobs but are **not estimated**. The translation state is
  estimated, consistency-tested, and — since WP12 L4 — **used for control**:
  the guided approach flies on the translation EKF's estimate under
  measurement dropout and an unestimated range-bias walk, including the
  honest NEES inconsistency that the unestimated bias produces (pinned
  figures and L-tags: [docs/safety.md](safety.md) WP12 section). Attitude
  sync in the campaign remains the truth-driven WP2 primitive (a documented
  WP5 simplification).
- **Continuous-torque DACS approximation — retired at L5 (WP12), with an
  honest remainder.** The v2–v4 controller assumed continuous torque; WP12
  L5 re-demonstrates sync under a delta-sigma minimum-impulse-bit actuator
  model instead (pinned sync-time comparison: [docs/safety.md](safety.md)).
  The MIB magnitude is PLACEHOLDER and ideal duty-cycling (no thruster
  hardware model, no minimum on-time) is still assumed; the faster-looking
  crossing under degradation is a metric artifact of the
  first-crossing-then-dwell criterion, not a physical speed-up — the claim
  is only that sync still completes under MIB + delay + fault.
- **Thermal model is a single lumped PCM bucket** — no eclipse/sunlight
  radiative balance, no per-node conduction.
- **Closed-loop rendezvous guidance is L0 truth-fed (WP11) plus L4
  estimate-driven (WP12), not flight guidance.** The L0 mode machine (far
  approach → hold → sync → final approach → contact → retreat) still flies
  the approach to contact truth-fed; WP12 L4 makes it estimate-driven under
  measurement dropout and an unestimated range-bias random walk, with the
  identified-but-not-implemented fix being explicit bias states in the
  filter (pinned contact-speed and NEES figures:
  [docs/safety.md](safety.md)). As of WP4 the attitude-sync loop consumes
  estimates from noisy sensors; attitude sync in the campaign remains the
  truth-driven WP2 primitive, and the truth-driven variants remain in the
  test suite as control-law regression references.
- **Decay model is first-order and single-object (WP3).** Quasi-circular
  drag decay over an exponential atmosphere with a single altitude-
  independent solar-cycle factor (a coarse proxy — the real swing is
  altitude-dependent); no J2, no attitude-dependent ballistic area, no
  re-entry demisability model — that safety/cost conflict is **open trade
  T2** (controlled vs uncontrolled reentry, driven by demisability; see
  [docs/target_selection.md](target_selection.md) for the Class-C
  discussion). The EDT branch is a physics-based band model (WP13), not a
  tether-hardware performance guarantee, and libration/dynamic stability
  remains **open trade T7** ([docs/target_selection.md](target_selection.md)).
- **Digital Twin Phase 1 (WP16) is twin-to-twin, not twin-to-real, and
  planar-only.** No real asset exists anywhere in WP16 (an owner-directed
  extension, not a spec-mandated work package): the "truth" twin the EKF
  assimilates is itself a perturbed-parameter instance of the same
  simulated lumped-mass bead-tether model, and the bead dynamics are planar
  (aligned-dipole in-plane forcing only) -- the out-of-plane (roll) channel
  that drives the real Pelaez libration-pumping instability is not
  integrated (a stated Phase-2 item). WP16 does not change the T7 open-
  trade status above; neither in-model controller is a resolved stability
  mechanism. Detail: [docs/digital_twin.md](digital_twin.md).
- **Passive-safety claims are model-scoped (F2).** The keep-out and
  safety-ellipse guarantees are exact only in the linear Clohessy-Wiltshire
  model: circular target orbit, no J2, no differential drag, small
  separations. J2 and differential drag erode drift-free safety ellipses
  over time, and the CW linearization error grows with separation — so
  every passive-safety number in this package holds in the model, not in
  the real environment. A real mission would re-verify all coasts against a
  higher-fidelity propagator; the WP12 fidelity ladder
  ([docs/safety.md](safety.md)) is a first concrete step toward that, not a
  substitute for it.
- **Campaign scope is deliberately narrow (WP5).** The campaign reuses the
  truth-driven `run_tumble_sync` primitive for tractable N≥500 Monte Carlo,
  so the dispersions that flow through it are the initial-condition and
  vehicle ones (tumble rate/axis, servicer initial attitude, actuator
  torque-scale and misalignment); **sensor noise/bias dispersions are drawn
  and their realized factors recorded but not re-propagated per run** — the
  closed-loop sensor effect is characterized by the WP4/WP12-L4
  estimate-driven acceptance instead. The Δv, time, and phasing cost
  coefficients and all dispersion magnitudes are PLACEHOLDER. WP5 produces
  **no** visualization/charts and performs **no** legal or regulatory
  determination: the future-facing compliance columns are passive research-
  profile metadata (`owner_consent_assumed` is a research scenario
  assumption per D9, never a legal fact — see
  [docs/legal_regulatory.md](legal_regulatory.md)).
- **Cost model is relative and parametric (WP6).** Every cost/FoM
  coefficient and both congestion-weight tables are PLACEHOLDER; outputs are
  in relative CU. WP6 predicts **no absolute program cost** on its own — a
  cited CU→currency range is the WP14 upgrade
  ([docs/cost_model.md](cost_model.md)), and no point-value dollar figure is
  ever emitted anywhere. Per-catalog FoM is dominated by removed mass; the
  metric-choice disagreement between the two weightings is **open trade
  T5** ([docs/cost_model.md](cost_model.md)). WP6 emits no charts.
- **The compliance precheck is not legal advice (WP8).** It checks a
  declared profile against research-grade rulepacks and reports evidence
  gaps — it does not and cannot determine legal conformity. Rulepacks are
  versioned snapshots that can go stale, and jurisdiction coverage is
  honestly partial (UN treaties + US + ESA reference only — Russian,
  Chinese, and Japanese national law, despite being adopter targets, are
  **not** yet covered). Full detail: [docs/legal_regulatory.md](legal_regulatory.md).
- **Small-debris (1–10 cm) removal is out of scope — open trade T6, resolved
  and visible, not neglect.** At ≈10 km/s the specific kinetic energy is
  ≈50 MJ/kg (≈12× the TNT specific-energy ratio); a 1 cm aluminum fragment
  carries ≈70.7 kJ (≈16.9 g TNT-equivalent); removing 1%/yr of the ≥1 cm
  population would need km²-scale collection area that would itself be the
  largest collision cross-section in the band. Full table:
  `generated/t6_flux_sweep.md`.
- Numbers for mass, inertia, power, PCM capacity, and the target
  orbit/keep-out radius are plausible placeholders, not derived from a
  specific bus or target design.

## TRL statement (binding, spec section 9)

**TRL 4 — for the GNC software element only.** With WP4 the attitude-sync
GNC element (tracking controller + estimator + sensor models) runs closed
loop **on estimated states** in a laboratory/simulation environment, with
filter consistency verified (NEES/NIS) rather than assumed — this is the
element-level TRL 4 definition. Scope caveat, stated plainly: **TRL 4
applies to the GNC software element, not the system** — system-level TRL is
undefined and not claimed here. Reaching TRL 5 for the element requires
real-time processor-in-the-loop execution on representative hardware (WP9,
**reserved, not started** — the flight-software migration path is documented
in the evidence pack's annex, not implemented). WP5 (campaign robustness),
WP6 (cost/FoM), WP10 (forensics), WP11 (safety hardening + guidance), WP12
(fidelity ladder), and WP13 (kit-class trade) all strengthen the TRL-4
evidence base or widen the validated envelope at TRL 4 — none of them raises
the TRL. Fidelity upgrades widen the validated envelope; they do not raise
maturity. This statement holds throughout WP10–WP15 and is repeated verbatim
across README, this file, and the evidence pack so it cannot silently drift.

## Model-scoping summary

Every safety or performance claim in this package carries an explicit
fidelity-level tag (L0…L6, R14). The short version: **L0** = linear
Clohessy-Wiltshire, the baseline every committed number traces back to;
**L1** = +J2 numerical propagation; **L2** = +free-molecular drag under an
independent dispersion stream; **L4** = navigation-error extensions (dropout,
bias walk) in the guidance loop; **L5** = actuator realism (minimum impulse
bit, command delay, single-axis fault). No claim in this package extends
past L2 for passive-safety geometry, or past L5 for actuator/guidance
realism — see [docs/safety.md](safety.md) for exactly which numbers carry
which tag, and [docs/gnc.md](gnc.md) for the underlying math each level
touches.

## Contact-energy worst case: the 0.41 J caveat

The committed contact-energy budget is **0.333 J** at the 0.15 m/s gate
speed with the 29.6 kg **dry+kit** servicer mass — that number is
machine-read from `generated/reference_metrics.csv` and owned by
[docs/safety.md](safety.md) (R16: one home per number). A second, worse-case
figure is quoted for the condition of **maximum residual propellant**
still aboard at contact, and it is documented here, in full, as an honest
gap rather than a mechanically-generated one:

```
dry_mass_kg (27.2) + kit_mass_kg (2.4) + initial_fuel_kg (7.2) = 36.8 kg
0.5 * 36.8 kg * (0.15 m/s)^2 = 0.414 J  (quoted as "~0.41 J")
```

The three mass terms are committed `Config` constants
(`include/adsc/mission.hpp: dry_mass_kg, kit_mass_kg, initial_fuel_kg`); the
speed term is the same `max_v_rel` gate used for the 0.333 J figure. **This
arithmetic is not currently emitted by any generated CSV or by any binary's
console output** — it is a hand-computed illustrative combination of already
-committed constants, which is exactly the category R16 is designed to
close over time. It is kept as prose, marked as a gap, rather than silently
promoted to a machine-checked number it is not yet: **regenerate the
underlying masses** with `./build/adsc_sim` (scenario 6 prints the dry+kit
contact mass and gate speed) and cross-check `initial_fuel_kg` directly in
`include/adsc/mission.hpp`; turning this specific combination into a
committed row of `generated/reference_metrics.csv` (via `sim_metrics`) is
open follow-up work, not done in this PR. The gate speed is the conservative
term in this comparison (fixed by design, not by mass); the mass term is
not — a heavier residual-propellant case is a real, if less likely,
operating point. **Open trade T8** tracks whether either energy figure is
below any validated material-damage threshold for aged MLI/paint — neither
is claimed to be (see [docs/safety.md](safety.md)).

## PLACEHOLDER inventory

Every unvalidated parameter in the repository is marked with the uppercase
`PLACEHOLDER` tag (R10) and mechanically collected into one appendix — never
hand-maintained, never silently dropped. Current total: **140** marks
(**56 decision-critical**, **43 moderate**, **41 cosmetic**) — see
[evidence/adsc_evidence_pack.md](../evidence/adsc_evidence_pack.md) appendix
("Appendix — PLACEHOLDER inventory") for the full, per-line, per-importance
table, regenerated by `tools/evidence/make_evidence.py`. If a value is
listed there, treat it as unvalidated until a cited source replaces it.
Decision-critical items (target masses/altitudes/inclinations,
collision-risk weights, cost coefficients, phasing Δv, sensor/actuator
uncertainties, contact threshold, density/solar model, EDT performance) are
the priority burn-down list; unresolved ones keep an explicit impact
statement rather than a silent PASS.

## Cross-references

- Pinned safety/campaign figures with their L-tags: [docs/safety.md](safety.md)
- GNC/estimator math and per-level model detail: [docs/gnc.md](gnc.md)
- Kit-class trade, EDT model scope, T1/T7 open items: [docs/target_selection.md](target_selection.md)
- Cost/FoM model, T5 open item: [docs/cost_model.md](cost_model.md)
- Regulatory precheck scope and jurisdiction gaps: [docs/legal_regulatory.md](legal_regulatory.md)
- One-code-path fidelity-ladder architecture: [docs/technical_architecture.md](technical_architecture.md)
