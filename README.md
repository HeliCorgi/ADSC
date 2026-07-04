# ADSC v4 — Active Debris Self-Cleanup

> **The product:** [**evidence/adsc_evidence_pack.md**](evidence/adsc_evidence_pack.md)
> — the generated English evidence pack (executive summary, installer argument,
> safety case, honest negatives, campaign/cost statistics, limitations,
> regulatory precheck, reproduction instructions). Every number in it is
> machine-read from committed artifacts; `bash tools/regenerate_all.sh build`
> regenerates all of it byte-for-byte.

A C++17 GNC **numerical simulator** for an installer-type active-debris-remediation
(ADR) servicer, framed as **Kessler-precursor removal**: the collisional cascade's
fuel is the population of massive derelict upper stages in congested orbital bands —
fragments are the symptom — so the servicer targets the objects that would become
the next fragment clouds, attaches a passive deorbit kit, and departs, using a
**passively-safe approach design** with clearing-verified aborts (WP11:
keep-out violations 0 of 500 per catalog at L0, ds-v1, Wilson 95% upper bound
0.0076, re-verified at L1 (+J2) and L2 (+drag) with zero violations
[ds-v1/ds-v2]). ADSC is an **open, reproducible evidence package** for that architecture (spec:
`adsc-specification-v5.md`), not flight software and not a mission proposal. It
assumes the operator is, or is contracted/consented by, the launching state of the
target; nothing in this repo assumes or enables unconsented approach to another
state's object. Governing value: **claims must match implementation** — every
number in this README is regenerable by running committed code.

**v4 status (work-package based):** WP1 (relative orbital motion + passive
safety), the F1/F2 honesty follow-ups, WP2 (tumble synchronization), WP3
(attach event + kit decay trades, mission reflowed into
approach→sync→attach→depart phases), WP4 (estimator + sensor models — the
control loop now runs on estimates, not truth), WP5 (campaign Monte-Carlo
under dispersions, with stable machine-readable CSV outputs), WP6 (parametric
cost model + figure of merit, consuming the WP5 CSV), and WP7a (a Python 3
stdlib-only Visualization Pack of static SVGs over the committed CSVs) are
implemented on top of the v2.0 GNC core, together with WP8 (Compliance Matrix
Generator — a research-grade **regulatory precheck**, explicitly **not legal
advice**) and WP7 (the **Evidence Pack** — the project's actual product,
generated at `evidence/adsc_evidence_pack.md`). **All work packages WP1–WP8 are
complete; WP9 (processor-in-the-loop / flight-software track) is reserved and
not started.** WP7a draws **static** figures only (no interactive dashboard);
WP8 produces a precheck/evidence matrix, **never a legal conformity
determination**; WP6 predicts **no absolute cost** (relative units only). None
of this changes the TRL (still 4, element-scoped).

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
- **Clearing-abort law + reachability screen + closed-loop guidance (WP11)**
  (`compute_clearing_abort` / `clearing_abort_for`, `guidance`): a 3-stage
  escalation ladder that accepts an abort only when the analytic post-burn
  ellipse (`bounded_coast_min_range`, closed form) clears keep-out plus a
  design margin — drift-null baseline, bounded radial reshape, two-impulse
  retreat hop — with the legacy `Capped` honesty preserved past the Δv budget.
  The campaign keep-out screen runs this law (D13); a truth-fed L0
  guided-approach demo (far approach → hold → sync → final approach → contact
  → retreat) screens abort feasibility before every committed impulse and
  produces the contact speed by a glideslope-with-floor profile
  (`generated/wp11_guidance_modes.md`). Unit-tested (`tests/test_guidance.cpp`)
  plus the pinned forensic-14 regression (`tests/test_forensic14.cpp`): the
  legacy law reproduces all 14 WP10c violations analytically, the clearing law
  clears every one.
- **Fidelity ladder (WP12)** (`propagation`, `adsc_ladder`): runtime-selectable
  **L0/L1/L2** propagation on **one code path** (no fork) — L0 is the original
  WP1 Clohessy-Wiltshire linearization (every committed WP5 number stays
  byte-identical), L1 differences a full inertial two-body+J2 RK4 propagation
  of both craft into the target's instantaneous LVLH frame, L2 adds a
  per-craft free-molecular drag term under an **independent**
  ballistic-coefficient dispersion stream (`ds-v2`; the committed campaign's
  own `ds-v1` stream is untouched). The mandatory cross-validation is tested
  (`tests/test_ladder.cpp`): with J2 forced off, L1 reproduces the CW closed
  form within a stated **2.0 m** linearization bound (the CW model's own
  error, measured at worst 0.52 m across the forensic-14 states); with drag
  forced off, L2 reproduces L1 **bit-for-bit**. Re-verifies every committed
  campaign abort event and the pinned forensic-14 states at L1/L2 with
  **zero** keep-out violations, and measures safety-ellipse margin decay over
  repeated orbits (`generated/wp12_ladder.{csv,md}`). Adds **L4**
  estimate-driven guidance (translation EKF in the loop, under measurement
  dropout and an unestimated range-bias walk) and **L5** a delta-sigma
  minimum-impulse-bit actuator model (command delay + single-axis fault),
  re-demonstrating tumble sync under real actuator quantization. Unit-tested
  (`tests/test_ladder.cpp`). Regenerate with `./build/adsc_ladder` (ladder) /
  `./build/sim_metrics` (L4/L5 rows of `generated/reference_metrics.csv`).
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
  Note: 25 years is the **IADC guideline** figure; under a US FCC license the
  current rule (FCC 22-74, adopted 2022) requires **5-year** post-mission
  disposal for LEO space stations — the sail-only negative for the heavy class
  only gets harder under the 5-year standard. Both rules are carried separately
  in the WP8 compliance rulepacks; the trade tables keep the IADC 25-year
  reference line and no quoted number changes.
- **Estimator + sensor models (WP4)** (`estimator`): the sync control loop now
  consumes **estimates, not truth** — enforced structurally: the controller is
  handed an `EstimatedState` only, and truth is read solely by the sensor
  models and the error-statistics recorder. Two decoupled filters (assumptions
  documented in the header): a **translation EKF** on the 6-state LVLH relative
  motion (prediction = the analytic WP1 CW state-transition matrix; 10 Hz range
  + line-of-sight **unit-vector** measurements — az/el is avoided because its
  Jacobian is singular at boresight while `d(r/|r|)/dr = (I−r̂r̂ᵀ)/|r|` is
  regular everywhere), and a **multiplicative attitude EKF** (3-component
  small-angle error states, covariance 3×3/6×6; an additive quaternion EKF is
  deliberately not used) with gyro-propagated own attitude + star-tracker
  updates and a 2 Hz vision relative-pose channel estimating `q_rel` and the
  target rate `w_t`. Covariance updates are Joseph-form + symmetrized; the
  tests check P symmetry and positive definiteness over a full run, and a
  **NEES/NIS consistency watchdog** (fixed seed, χ² bands) rejects the
  classic fake of inflating Q/R until acceptance passes. All randomness is one
  fixed-seed `mt19937` + explicit Box-Muller (bit-stable across platforms).
- **Campaign Monte-Carlo (WP5)** (`campaign`): one mission processes N
  class-level catalog targets sequentially in one plane (batch amortization),
  tracking Δv budget and kit inventory and ending on Δv exhaustion, kit
  exhaustion, all-targets-processed, or a terminal keep-out violation.
  Inter-target phasing is a parameterized Δv/time cost (PLACEHOLDER; no
  plane-change optimization). Dispersions (all PLACEHOLDER, centralized in
  `CampaignConfig`) cover initial relative state, tumble rate/axis, servicer
  initial attitude, sensor noise/bias, actuator torque-scale (~±10%) and
  misalignment, and solar activity; every per-run seed is a deterministic
  `SplitMix64(master_seed, run_index)`. Outputs per catalog preset — success /
  abort / keep-out-violation **rates with Wilson 95% CIs** (all three reported
  even at zero), and Δv-used / kits-used / removals / sync-time **percentiles
  (p05/p50/p95)**, plus the failure classification counts — are written as
  stable machine-readable CSV (`generated/wp5_campaign_*.csv`, schema
  `1.0`, documented in `generated/wp5_campaign_schema.md`) intended for future
  WP6/WP7/WP8 consumption. Unit-tested (`tests/test_campaign.cpp`: seed
  determinism, Wilson edge cases, percentiles, termination, schema stability,
  the guardrail that no artifact emits legal-approval language). Regenerate with
  `./build/adsc_campaign`.
- **Parametric cost model + figure of merit (WP6)** (`cost`): the spec's
  `C_campaign = C_dev + C_bus(m_dry) + N·C_kit + C_launch(m_wet, band) +
  C_ops(T)` in **relative cost units (CU)** — **no point-value dollar figure is
  ever emitted** (R6/D10); the CU→currency anchor is a PLACEHOLDER cited range
  left for WP7. Consumes the WP5 campaign (schema 1.0): the **amortization
  curve** (cost/removal vs kits carried N) re-runs the WP5 engine across a kit
  sweep at the fixed master seed — the baseline reproduces
  `wp5_campaign_runs.csv` exactly — and falls until the Δv budget (not the kit
  count) bounds removals, which is the quantitative installer/batch argument.
  Cost, cost/removal and **FoM = Σ mᵢ·w(hᵢ)/C_campaign** are propagated **per
  run** (p05/p50/p95, never mean-only). FoM is reported under **two** normalized
  congestion weightings (spatial-density + criticality-style, both PLACEHOLDER);
  they disagree on band priority — open trade **T5**. A machine-readable
  **tornado** ranks each cost parameter's ± impact. Outputs
  `generated/wp6_cost_summary.{md,csv}` + `wp6_cost_schema.md` (schema `1.0`).
  Unit-tested (`tests/test_cost.cpp`). Regenerate with `./build/adsc_cost`.
- **Visualization Pack (WP7a)** (`tools/viz/make_viz.py`, `decay_trade`): a
  **Python 3 standard-library-only** generator (spec v4.2 R9 tooling exception —
  no plotting library, no JS/CDN/external fonts; SVG is written directly) that
  turns the committed WP5/WP6/WP3 CSVs into nine static **SVG** figures plus a
  static report page. It draws: WP5 mission-outcome proportions (zeros shown),
  Δv / removals / sync-time **p05/p50/p95** ranges (never mean-only), the
  keep-out-violation **rate + Wilson 95% CI + N** (a rate panel, *not* a fake
  trajectory), the WP6 **amortization curve** with the Δv-limited N=4 turn, the
  **tornado**, the **FoM two-weighting T5 band-flip**, and the WP3 sail-decay
  band (solar min..max, 25-yr line, catalog A vs B). It reads **every number
  from a committed CSV** (nothing hand-written); each caption carries the source
  CSV, master seed and schema_version; **no timestamps/run-times are embedded**
  (byte-reproducible — a CI gate asserts regeneration matches the committed
  bytes). It does **not** draw charts from any live/streaming source, embed real
  imagery, or make interactive dashboards, and the keep-out figure states it is
  a *simplified research visualization, not a flight safety certificate*. The
  `decay_trade` target re-emits the existing WP3 decay numbers as CSV (no
  physics change). Outputs `generated/viz/*.svg` + `wp5_dashboard.html` (a static
  report page); tested via `tools/viz/test_viz.py` (ctest `viz`). Regenerate
  with `python3 tools/viz/make_viz.py . generated/viz`.
- **Compliance Matrix Generator / Regulatory Precheck (WP8)**
  (`tools/compliance/`): a Python 3 **standard-library-only** checker (spec v4.2
  R9; no `jsonschema` — validation is hand-written and dual-maintained with the
  documented `mission_profile.schema.json`) that evaluates a declared mission
  profile against **versioned rulepacks** (UN treaties, US FCC/FAA/NOAA, ITU,
  ESA reference, ADSC internal policy; primary sources only) and emits
  `generated/compliance_findings.json` + `evidence/compliance_matrix.{md,csv}`
  (schema `1.0`, deterministic, no timestamps). Every finding separates
  **binding law / non-binding guideline / agency guidance / ADSC policy /
  placeholder research assumption**; *unknown is safer than a false PASS*
  (missing inputs are UNKNOWN or fail, never pass); missing mandatory evidence
  → WARN/BLOCK; unconsented active interference with a registered object →
  **BLOCK** (ADSC policy). It also cross-checks the WP5 campaign compliance
  metadata columns against the profile. **This is a precheck, not legal
  advice**, and the rulepacks are versioned research artifacts that **can go
  stale** as regulations change. Coverage today: UN treaties + US + ESA
  reference only — Russian/Chinese/Japanese national law is **not** yet
  covered (future work). Tested via `tools/compliance/test_compliance.py`
  (ctest `compliance`).
- **Evidence Pack (WP7 — the actual product)** (`tools/evidence/`): a Python 3
  stdlib-only generator writes `evidence/adsc_evidence_pack.md` — executive
  summary, the quantitative installer argument + cascade source-term framing
  (full primary citations where confident, `[CITATION NEEDED — PLACEHOLDER]`
  where not, never fabricated), safety case (WP1 coast statistics, F1
  clean+capped abort coverage, WP5 keep-out rate with Wilson CI, contact-energy
  budget), the honest decay negatives, campaign/cost/FoM statistics with the T5
  disagreement kept open, a plain-language limitations section, the regulatory-
  precheck summary ("This is not legal advice." in the body), the flight-
  software migration annex (deliberately NOT implemented; WP9 reserved),
  one-command reproduction instructions, and an **auto-collected PLACEHOLDER
  inventory** of every unvalidated parameter in the tree. **Zero hand-written
  numbers**: everything is machine-read from the committed CSVs (a new
  `sim_metrics` target re-emits the pinned WP1/F1/WP2/WP3/WP4 demo numbers as
  `generated/reference_metrics.csv`; no physics change). A **claim-audit test**
  (ctest `evidence`) rejects forbidden overclaims (flight-ready/-proven,
  TRL 5/6 claims, "guaranteed", legal-conclusion words), verifies required
  honesty sections, cross-checks every quoted headline number against its
  source CSV, and enforces determinism + committed==regenerated.
  `tools/regenerate_all.sh` is the single regeneration entry point (CI runs
  exactly it).
- **First-order PCM thermal budget** integrated over the control loop.
- **Deorbit gating**: autonomous by default, human-in-the-loop only on the
  emergency path, blocked below the fuel reserve.

The simulation driver (`adsc_sim`) runs a real post-capture detumble: it seeds a
tumble, closes the control loop at dt = 0.01 s, and reports the settling time and
final body rate — so "stable at dt = 0.01 s" is now something you can run and
verify rather than a claim in prose.

## What is explicitly NOT implemented (honest scope)

- **Estimator scope is deliberately narrow (WP4).** The target inertia is
  assumed **known** for the torque-free feedforward and the MEKF model — exact
  in this simulation, but a real mission needs inertia identification. Sensor
  models are Gaussian abstractions (no outliers, no dropouts, no star-tracker
  occlusion, no vision pose ambiguity); gyro/rangefinder biases exist as
  Config knobs but are **not estimated** (the consistency tests assume the
  zero defaults). The translation state is estimated, consistency-tested and —
  since WP12 L4 — **used for control**: the guided approach flies on the
  translation EKF's estimate under measurement dropout and an unestimated
  range-bias walk (see the closed-loop guidance bullet below for the measured
  numbers, including the honest NEES inconsistency that unestimated bias
  produces). Attitude sync in the campaign remains the truth-driven WP2
  primitive (documented WP5 simplification).
- **Continuous-torque DACS approximation — retired at L5 (WP12).** The v2–v4
  controller assumed continuous torque, with sync-hold relying on that
  approximation (the fine firing deadband, 3×10⁻⁴ on the sliding variable,
  bounded the sync-hold error only under continuous torque). WP12 L5
  re-demonstrates sync under a delta-sigma minimum-impulse-bit actuator model
  (MIB 2×10⁻⁴ N·m·s PLACEHOLDER, 1-step command delay, single-axis 50% fault)
  [L5: MIB delta-sigma, 1-step delay, single-axis fault]: sync at **16.28 s**
  vs **16.87 s** continuous (`wp12_l5_sync_time_s` = 16.280000,
  `wp2_sync_time_s` = 16.870000, `generated/reference_metrics.csv`). Honest
  remainder: the MIB magnitude is PLACEHOLDER and ideal duty-cycling (no
  thruster hardware model, no minimum on-time) is still assumed.
- **Thermal model is a single lumped PCM bucket** — no eclipse/sunlight
  radiative balance, no per-node conduction.
- **Closed-loop rendezvous guidance: L0 truth-fed (WP11) plus L4
  estimate-driven (WP12).** A deterministic, truth-fed L0 guided-approach mode
  machine (far_approach → hold → sync_hold → final_approach → contact →
  retreat, with a reachability screen evaluated before every committed
  impulse — screen held at every step in the demo: yes) still flies the
  approach to contact truth-fed (`generated/wp11_guidance_modes.md`). The
  contact speed is PRODUCED by a glideslope-with-floor profile (0.10 m/s
  design) rather than merely gated, closing the long-standing "gate, not
  guidance" limit at L0; the v_rel ≤ 0.15 m/s gate is retained as an
  independent check, not removed. **WP12 L4 makes this guidance
  ESTIMATE-DRIVEN**: the translation EKF is in the loop (truth is
  error-recording only) under measurement dropout and an unestimated
  range-bias random walk; the truth-evaluated contact speed is **0.0998 m/s**
  (`wp12_est_guided_contact_speed_m_s` = 0.099759, gate 0.15 m/s,
  `generated/reference_metrics.csv`) [L4: L0 dynamics + dropout + bias walk,
  deterministic seed]. Honest addition: under the unestimated bias walk the
  translation-EKF NEES runs to **~310** (`wp12_est_nees_trans` = 310.437825)
  against an ideal **~6** — the filter is optimistic about (i.e.
  underestimates) its own error; adding bias states is the identified fix,
  not implemented here (evidence pack §3 fidelity ladder, §6 limitations).
  As of WP4 the attitude-sync loop consumes estimates from noisy sensors;
  attitude sync in the campaign remains the truth-driven WP2 primitive
  (documented WP5 simplification), and the truth-driven variants remain in
  the test suite as the control-law regression references.
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
- **Campaign scope is deliberately narrow (WP5).** The campaign reuses the
  truth-driven `run_tumble_sync` primitive for tractable N≥500 Monte-Carlo, so
  the dispersions that flow through it are the initial-condition and vehicle
  ones (tumble rate/axis, servicer initial attitude, actuator torque-scale and
  misalignment); **sensor noise/bias dispersions are drawn and their realized
  factors recorded but not re-propagated per run** — the closed-loop sensor
  effect is characterized by the WP4 estimate-driven acceptance instead. The Δv,
  time and phasing cost coefficients and all dispersion magnitudes are
  PLACEHOLDER. WP5 produces **no visualization/charts** and performs **no legal
  or regulatory determination**: the future-facing compliance columns are
  passive research-profile metadata (`owner_consent_assumed` is a research
  scenario assumption per D9, never a legal fact), reserved for WP7/WP8.
- **Cost model is relative and parametric (WP6).** Every cost/FoM coefficient
  (`CostConfig`) and both congestion-weight tables are **PLACEHOLDER**; outputs
  are in relative CU. **WP6 predicts no absolute program cost** — a cited
  CU→currency range is deliberately left unfilled (a WP7 deliverable), and no
  point-value dollar figure is emitted anywhere. Per-catalog FoM is dominated by
  removed mass (the ~9 t class outranks the ~1.4 t class under both weightings);
  the metric-choice disagreement that WP6 surfaces is the **band-priority flip**
  between the two weightings (open trade T5). WP6 emits no charts.
- **The compliance precheck is not legal advice (WP8).** It checks a declared
  profile against research-grade rulepacks and reports evidence gaps — it does
  not and cannot determine legal conformity, and its outputs never say
  otherwise. Rulepacks are **versioned snapshots that can go stale**; every
  rule carries `source_date_or_version` and a `limitations` field, low-confidence
  rules are downgraded to `placeholder`, and jurisdiction coverage is honestly
  partial (UN treaties + US + ESA reference; **no** Russian/Chinese/Japanese
  national law yet, despite those being adopter targets — future work).
- **Small-debris (1–10 cm) removal is out of scope (T6)** — a documented
  exclusion, not neglect: the `flux_sweep` target regenerates the physics (≈50
  MJ/kg / ≈12× TNT at 10 km/s; a 1 cm Al fragment ≈ 71 kJ ≈ 17 g TNT; km²-scale
  collector area — ≈32 km² average, ≈3.8 km² peak — for 1 %/yr removal) into
  `generated/t6_flux_sweep.md`.
- Numbers for mass, inertia, power, PCM capacity and the target orbit/keep-out
  are plausible placeholders, not derived from a specific bus or target design.

## TRL assessment (honest)

**TRL 4 — for the GNC software element only**: with WP4 the attitude-sync GNC
element (tracking controller + estimator + sensor models) runs closed loop **on
estimated states** in a laboratory/simulation environment, with filter
consistency verified (NEES/NIS) rather than assumed. That is the element-level
TRL 4 definition; the earlier "TRL 5–6" framing (v1.21) was not supportable and
WP1–WP3 stayed at 3–4 because control consumed true states. Scope caveats,
stated plainly: **TRL 4 applies to the GNC software element, not the system** —
system-level TRL is undefined and not claimed here. Reaching TRL 5 for the
element requires real-time processor-in-the-loop execution on representative
hardware (see the WP7.7 flight-software migration annex direction in the spec),
which this repo deliberately does not attempt. WP5 adds
robustness-under-dispersions evidence, but this strengthens the TRL-4 evidence
base rather than raising the TRL. WP6 adds a relative-unit cost/FoM model on top
of that evidence — an economic-viability argument, not a GNC maturity change —
so the TRL stays 4. The same holds for Phase 0 and beyond (spec v5 §9, binding):
WP10 forensics, WP11 safety hardening + closed-loop guidance, and the WP12
fidelity ladder (higher-fidelity propagation is a wider validated envelope,
not a maturity change) widen the validated envelope at TRL 4 — they do not
raise it; raising it requires the real-time processor-in-the-loop track
(WP9, reserved, not started).

## Build

Requires a C++17 compiler and Eigen 3.3+.

```
git clone https://github.com/HeliCorgi/ADSC.git
cd ADSC
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # add -DADSC_WERROR=ON for R3
cmake --build build
./build/adsc_sim
./build/adsc_campaign          # WP5: regenerates generated/wp5_campaign_* (full N=500)
./build/adsc_cost              # WP6: regenerates generated/wp6_cost_* (consumes the WP5 summary)
./build/decay_trade            # WP3: re-emits the sail-decay trade as generated/wp3_decay_trade.csv
python3 tools/viz/make_viz.py . generated/viz   # WP7a: regenerates generated/viz/*.svg + dashboard
python3 tools/compliance/check_compliance.py    # WP8: regenerates generated/compliance_findings.json
python3 tools/compliance/generate_matrix.py     # WP8: regenerates evidence/compliance_matrix.{md,csv}
python3 tools/evidence/make_evidence.py         # WP7: regenerates evidence/adsc_evidence_pack.md
bash tools/regenerate_all.sh build              # ...or regenerate EVERYTHING in order (CI runs this)
ctest --test-dir build      # fuel_store/relmotion/sync/decay/mission/estimator/campaign/cost/flux/viz/compliance/evidence
```

When invoking bare g++ directly, prefer passing Eigen via `-isystem` to avoid
GCC `-Wmaybe-uninitialized` false positives; CMake handles this automatically.

## Layout

```
include/adsc/   public headers (fuel_store, dynamics, controller, thermal,
                relmotion, decay, estimator, mission, campaign, cost, flux)
src/            implementations + main.cpp sim + WP5/WP6/T6/WP3 emitters (main_*.cpp)
tests/          C++ unit tests (+ Python tests under tools/, ctest `viz`/`compliance`)
tools/viz/      WP7a Python 3 stdlib-only visualization generator (make_viz.py)
tools/compliance/  WP8 regulatory precheck: rulepacks, schema, checker, matrix generator
tools/evidence/    WP7 evidence-pack generator + claim-audit test
tools/regenerate_all.sh  single regeneration entry point (CI runs exactly this)
generated/      committed artifacts: WP5 campaign, WP6 cost/FoM, T6 flux, WP3 decay,
                reference_metrics CSVs, viz/ SVGs, compliance_findings.json,
                WP10/WP11 forensics (wp10_violation_forensics.{md,csv} — legacy-law
                archive since WP11; wp11_abort_audit.{md,csv}; wp11_guidance_modes.md),
                WP12 fidelity-ladder re-verification (wp12_ladder.{csv,md})
evidence/       committed generated evidence artifacts (adsc_evidence_pack.md,
                compliance matrix; same reproducibility gate as generated/)
.github/        CI (Ubuntu + Eigen: cmake build + ctest, warnings-as-errors, reproducibility gate)
adsc-specification-v5.md   active spec (work packages, hard rules, locked decisions)
adsc-specification-v4.md   retired v4.2 spec (superseded by v5; kept because v5
                           references its D/R/T items)
```

## Roadmap (work packages — see `adsc-specification-v5.md`)

- **WP1 — Relative orbital motion + passive safety** ✅ implemented.
- **F1/F2 — Capped-abort honesty + model-scope note** ✅ implemented.
- **WP2 — Tumble synchronization** ✅ implemented (tracking SMC with torque-free
  feedforward against a precessing target).
- **WP3 — Attach event + kit decay trades** ✅ implemented (contact-energy
  budget; sail vs EDT decay trade with honest negative results).
- **WP4 — Estimator + sensor models** ✅ implemented (translation EKF +
  multiplicative attitude EKF; control on estimates; NEES/NIS consistency
  watchdog).
- **WP5 — Campaign Monte-Carlo** ✅ implemented (dispersions; success / abort /
  keep-out rates with Wilson CIs; Δv / kit / removal percentiles; stable
  machine-readable CSV for WP6/WP7/WP8).
- **WP6 — Parametric cost model** ✅ implemented (relative-unit C_campaign;
  amortization curve; FoM under ≥2 weightings with T5; tornado sensitivity).
- **WP7a — Visualization Pack** ✅ implemented (Python 3 stdlib-only static SVG
  figures + static report page from the committed CSVs; no external deps).
- **WP8 — Compliance Matrix Generator / Regulatory Precheck** ✅ implemented
  (versioned rulepacks; precheck + evidence matrix; **not legal advice**, no
  legal conformity determination).
- **WP7 — Evidence Pack** ✅ implemented (`evidence/adsc_evidence_pack.md`,
  generated, claim-audited, zero hand-written numbers — the actual product).
- **WP9 — Processor-in-the-loop / flight-software track** — **reserved, not
  started** (the only path above element TRL 4; see the spec).
- **WP10 — Phase 0: adoption, citations, forensics** ✅ implemented (spec v5
  adopted; citation-fill; keep-out-violation forensics — see
  `generated/wp10_violation_forensics.md`, now the legacy-law archive
  superseded by WP11).
- **WP11 — Safety hardening + closed-loop guidance** ✅ implemented
  (clearing-abort law, reachability screen, guided approach); keep-out
  violations 0 of 500 per catalog at L0/ds-v1, Wilson 95% upper bound 0.0076.
- **WP12 — Fidelity ladder** ✅ implemented (L0/L1/L2 + L4/L5 elements; zero
  keep-out violations re-verified at L1/L2 [ds-v1/ds-v2]; the
  CW-safe-but-higher-level-unsafe case does NOT materialize for this
  dispersion set — measured, see `generated/wp12_ladder.md`).
- **WP13–WP15 (v5)** — not started; `adsc-specification-v5.md` defines the
  kit-class trade + EDT physics, cost ranges + FoM, and the proposal package.
  WP9 remains the only path to TRL 5, unchanged by v5.

**Roadmap end state: WP1–WP8 complete; Phase 0 (WP10), WP11 safety
hardening, and WP12 fidelity ladder complete.** The package is regenerable
end-to-end with one command
(`bash tools/regenerate_all.sh build`) and CI enforces byte-identity of every
committed artifact on every push.

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
1500 m/day of along-track decay. (The 25-year column is the IADC guideline;
under a US FCC license the current rule is **5 years** — FCC 22-74, 2022 — which
makes the class-A sail-only negative strictly harder. See the WP8 rulepacks.)

Reproducible WP4 numbers (regenerate with `./build/adsc_sim`, scenario 7; fixed
seed 20260703): driven purely by estimates under sensor noise, sync is achieved
on the **truth** state at **17.07 s** with post-dwell max |w_rel| = 0.0130 deg/s
(tol 0.1) and max attitude error = 0.0848° (tol 2°). Estimation RMS over the
stats window: own attitude 0.0020°, relative attitude 0.0341°, target rate
0.00106 deg/s, relative position 0.038 m, velocity 0.0011 m/s. Filter
consistency: NIS 3.944/4 (translation, N=800), 3.063/3 (star tracker, N=400),
3.190/3 (vision, N=160); NEES 7.796/6, 2.660/3, 4.066/6 — all inside their
documented χ² bands; covariance health min eigenvalue 1.3×10⁻¹², max asymmetry
exactly 0.

Reproducible WP5 numbers (regenerate with `./build/adsc_campaign`; fixed master
seed `0x5AD5C0DECAFE2026`, full N = 500 runs per catalog preset, 6 targets /
mission, 4 kits, 140 m/s Δv budget). CI runs the same full N = 500 and uploads
the four `generated/` artifacts (the committed copies), so routine CI and the
full campaign are the same run — no reduced CI subset was needed (runtime is
well under the 5-minute guideline; `adsc_campaign 100` produces a stratified
subset if a faster smoke run is ever wanted). Rates carry a Wilson 95% CI
(z = 1.959963984540054); distributions are p05/p50/p95.

<!-- WP5-NUMBERS-START (filled from CI adsc_campaign, seed 0x5AD5C0DECAFE2026) -->
The full N = 500 campaign (both catalog presets) runs in a few tens of seconds
on a CI runner (see the Actions log of the current run for the actual figure).
`success` here means a **productive end** — the mission installed its
full 4-kit complement (`kit_exhausted`) or cleared all 6 targets (`completed`) —
not one cut short by Δv exhaustion or a keep-out violation.

| metric | SL-16 / Zenit-2 class | SL-8 / Kosmos-3M class |
|---|---|---|
| success rate | **0.556** [0.512, 0.599] | **0.542** [0.498, 0.585] |
| nonproductive-termination rate (= 1 − success) | **0.444** [0.401, 0.488] | **0.458** [0.415, 0.502] |
| gate-abort-run rate (abort-path exposure) | **0.444** [0.401, 0.488] | **0.458** [0.415, 0.502] |
| keep-out-violation rate | **0.000** [0.000, 0.008] [L0, ds-v1] | **0.000** [0.000, 0.008] [L0, ds-v1] |
| Δv used p05/p50/p95 [m/s] | 124 / 124 / 136 | 124 / 124 / 136 |
| kits used p05/p50/p95 | 3 / 4 / 4 | 3 / 4 / 4 |
| removals/mission p05/p50/p95 | 3 / 4 / 4 | 3 / 4 / 4 |
| sync arrival p05/p50/p95 [s] | 14.47 / 17.68 / 20.09 | 14.13 / 17.72 / 20.16 |
| failure counts (runs) | dv_exhausted 222, kit_exhausted 278, keep_out 0, completed 0 | dv_exhausted 229, kit_exhausted 271, keep_out 0, completed 0 |
| per-target events | gate_abort 292, sync_timeout 0 | gate_abort 291, sync_timeout 0 |

Two abort-related rates are reported and are deliberately distinct:
**`gate-abort-run rate`** is the abort-path exposure (fraction of runs with ≥ 1
closing-speed gate abort — what the spec calls the "abort rate"), while
**`nonproductive-termination rate`** is 1 − success. Under the current *flat
PLACEHOLDER* Δv cost the two coincide numerically — every aborting mission needs
an extra target-slot to still install its kits and so exhausts the 140 m/s
budget — but they are separate concepts and will diverge once the cost model
gains structure. The honest campaign finding: the servicer is **Δv-limited about
44% of the time** (`dv_exhausted` = 222/500) and installs its full kit
complement the rest; the attitude sync **never** times out across the sampled
tumble/attitude/actuator dispersions, and keep-out violations are **0 of 500**
per catalog [L0: linear CW, dispersion set ds-v1, Wilson 95% upper bound
0.0076] — the WP11 clearing-abort law accepts an abort only when the analytic
post-burn ellipse clears keep-out plus a design margin (mechanism forensics:
`generated/wp10_violation_forensics.md`; audit:
`generated/wp11_abort_audit.md`). `completed` (all 6 targets) is 0 by
construction — 4 kits cannot service 6 targets. The `Δv used` / `kits used`
percentiles matching across the two presets is expected (a flat PLACEHOLDER cost
takes quantized, catalog-independent values — not a copy-paste bug). Full per-run
records and the column schema are in [generated/](generated/).
<!-- WP5-NUMBERS-END -->

WP5 produces **no** charts and performs **no** legal/regulatory determination;
the CSV schema (`generated/wp5_campaign_schema.md`, version `1.0`) is stabilized
for future WP6 cost/FoM, WP7 visualization, and WP8 compliance tooling.

Reproducible WP6 numbers (regenerate with `./build/adsc_cost`; relative cost
units **CU**, **no absolute-dollar figure claimed**). WP6 consumes the WP5
campaign (schema 1.0), re-running the engine across a kits-carried sweep at the
fixed master seed; the baseline (N = 4) reproduces `wp5_campaign_runs.csv`.

<!-- WP6-NUMBERS-START (filled from CI adsc_cost) -->
The full sweep (500 runs × 2 catalogs × N = 1..8 kits) runs in a few tens of
seconds on a CI runner (see the Actions log of the current run for the actual
figure), and the baseline (N = 4) removals reproduce the committed WP5 CSV
exactly (`MATCH (schema 1.0 consumed)`).

**Amortization curve — cost/removal vs kits carried N (SL-16 class):**

| N | cost/removal p50 [CU] | ratio to N=1 | removals p50 |
|---:|---:|---:|---:|
| 1 | 157.07 | 1.000 | 1 |
| 2 | 82.22 | 0.523 | 2 |
| 3 | 57.27 | 0.365 | 3 |
| **4** | **44.80** | **0.285** | 4 |
| 5 | 46.14 | 0.294 | 4 |
| 8 | 50.15 | 0.319 | 4 |

Batch amortization drives cost/removal down to **28.5 % of the single-target
(N = 1) baseline** — a **3.5× per-removal saving** — bottoming at N = 4 where the
**Δv budget (not the kit count)** caps removals at 4; carrying more kits then
adds cost without removals and the curve turns back up. That shape *is* the
quantitative installer/batch argument (SL-8 class is within ~0.3 %).

**FoM = Σ mᵢ·w(hᵢ)/C_campaign at N = 4 (p50, kg/CU):**

| catalog | spatial-density | criticality |
|---|---:|---:|
| SL-16 (~9 t, 840 km) | 156.70 (w = 0.78) | 200.90 (w = 1.00) |
| SL-8 (~1.4 t, 750 km) | 31.35 (w = 1.00) | 21.32 (w = 0.68) |

The ~9 t class outranks the ~1.4 t class under **both** weightings (FoM is
mass-dominated), but the **band weight flips** — spatial density values the
750 km band more, the criticality index values the 840 km band more — a genuine
metric-choice disagreement tracked as **open trade T5**. **Tornado** (SL-16,
±30 %, ranked by cost/removal swing): `c_dev_cu` (15.0 CU) ≫ `c_bus_cu` (4.5) >
`c_launch_cu_per_kg` (3.7) > `c_kit_cu` (2.4) > `c_ops_cu_per_day` (1.2). All
values are relative CU; **no absolute-dollar figure is claimed**.
<!-- WP6-NUMBERS-END -->

## Disclaimer / 免責

Conceptual prototype for peaceful ADR research and education only. Real flight
hardware requires formal design review, qualification, and compliance with
international space law (IADC guidelines, UN COPUOS). No military or re-entry-body
application is intended.

本プロジェクトは平和的デブリ除去技術の教育・研究目的の概念プロトタイプです。実際の
宇宙機運用・打ち上げには専門機関による設計審査・認定・国際法遵守が必要です。兵器・
高速再突入体への使用は一切想定していません。
