# ADSC — Technical Summary

*A 5-page-equivalent technical summary for reviewers who want the mechanism,
not just the headline. Every safety/performance claim below carries its
fidelity tag (L0…L6, WP12); numbers are stated by reference to a committed
artifact, never re-derived. Scope: TRL 4, GNC software element only; not
flight software; not a mission proposal; not legal advice — see
[roadmap.md](roadmap.md) and [limitations.md](limitations.md).*

## 1. Architecture

ADSC simulates an **installer**, not a tug (D1): a small servicer attaches a
passive deorbit kit (drag sail or electrodynamic tether) to a massive
derelict upper stage and departs, rather than decelerating the full stage
mass itself. The C++17 simulator is one code path across fidelity levels
(no fork): a TMR/CRC32-protected fuel store models actuator-command
integrity; RK4 rigid-body dynamics drive a quaternion sliding-mode attitude
controller (regulation + tracking, torque-free feedforward, transport term);
relative motion runs on Clohessy-Wiltshire at L0 and on full inertial
two-body+J2 (L1) or +drag (L2) propagation differenced back into the target
LVLH frame at higher levels — every committed L0 campaign number is
byte-identical regardless of which level is compiled in. A phase-machine
mission driver (SAFE/APPROACH/SYNC/attach/depart) wraps the GNC core; a
Monte-Carlo campaign engine runs N=500 dispersed missions per catalog at a
fixed master seed; a relative-cost-unit (CU) model with WP14 absolute-range
extensions computes cost/FoM; a regulatory precheck evaluates a declared
mission profile against versioned rulepacks. Full module map:
[technical_architecture.md](technical_architecture.md);
GNC internals: [gnc.md](gnc.md).

## 2. GNC evidence

Attitude sync (tumble arrest to pointing lock) is demonstrated
truth-driven at **L0** and re-demonstrated estimate-driven under sensor
noise: the translation EKF + attitude MEKF run on estimates only (truth is
structurally isolated to sensor models and error recording), and the
NIS/NEES consistency watchdog rejects the classic covariance-inflation
fake. The guided approach flies closed-loop with reachability-screened
aborts, retained as an independent gate check rather than mere guidance
(closing the long-standing "gate, not guidance" limit). Under **L4**
(navigation-error extensions: measurement dropout + an unestimated
range-bias random walk), the estimator is honestly reported as *optimistic*
about its own accuracy — a documented estimation gap, not a hidden one;
adding bias states is the identified fix, not yet implemented. Under **L5**
(minimum-impulse-bit actuator realization with command delay and a
single-axis fault), the long-standing continuous-torque idealization
caveat is retired **by measurement**: sync still completes, and guidance
contact velocity quantized at the actuator's translation MIB still meets
its gate. L3 (SRP) and L6 (deterministic worst-case Monte Carlo) are not
yet implemented — stated here rather than implied. Pinned regression
values and full per-channel breakdown: [gnc.md](gnc.md).

## 3. Safety case

Approach safety is a **passively-safe approach design with
clearance-verified aborts (keep-out violations 0/500 per catalog [L0,
ds-v1, Wilson ≤ 0.0076])** — never "strict approach safety" or "safe
satellite" (spec §1 binding wording). History (R15): a pre-WP11 baseline
showed a nonzero violation rate, archived at `v0.10-phase0-baseline`; WP11's
clearing-abort law closed it to zero, and the **L1** (+J2) / **L2** (+drag)
fidelity ladder re-verifies zero violations over every committed abort
event at both higher levels — a negative-negative result (the anticipated
CW-safe-but-higher-level-unsafe failure mode did not materialize for this
dispersion set), reported with the same discipline as a positive one.
Safety-ellipse margin decay under J2 is measured, not merely flagged
qualitatively as in earlier revisions. A contact-energy budget bounds the
kinetic energy transferred at the gated closing speed, framed explicitly as
a geometry-keyed-clamp argument, not a claim that the resulting energy is
below any specific material damage threshold (that threshold remains
PLACEHOLDER-cited). All guarantees are model-scoped: a real mission
re-verifies every coast against a higher-fidelity propagator before flight.
Full re-verification table, forensic-regression detail, and campaign
statistics: [safety.md](safety.md).

## 4. Kit trade

The kit-decay trade reports the honest negative first (WP13): meeting the
IADC 25-year post-mission-disposal guideline needs an impractically large
sail for the ~9 t reference class, while the same guideline closes at a
practical sail area for the ~1.4 t reference class. For the heavy class, an
electrodynamic tether is carried as an honest **candidate with open
risks** — not a closed recommendation — because the along-track libration
dynamics of a rigid current-carrying tether remain an open trade (T7,
cited literature pointer, not resolved by this package) and plasma-density
inputs remain PLACEHOLDER at both solar-min and solar-max. A third,
separate mission class (a defunct massive sun-synchronous payload) is
evaluated on controlled-reentry terms, not kit-ranked against the two
classes above — a kit-only deorbit is explicitly not the recommended path
for that class. The WP14 target-prioritization table joins this trade with
the cost/FoM model (no new simulation, a pure derived view over already-
committed CSVs) and ranks the heavy class first under both figure-of-merit
weightings evaluated, with a one-paragraph defense per class. Full trade,
per-class kit recommendation, and prioritization defense:
[target_selection.md](target_selection.md),
`generated/wp14_prioritization.md`.

## 5. Cost and figure of merit

Cost is reported in **relative cost units (CU)** as the primary metric —
"minimum cost" as an absolute claim is banned throughout (spec §1); the
defensible claim is **cost-effectiveness: cost per removal and cost per
unit risk reduction**. The amortization curve bottoms at a specific kit
count because the mission Δv budget, not the kit count, caps removals per
mission — the honest capacity story a mission designer needs, not a
monotonically-improving curve. Parameter sensitivity is ranked by a
one-at-a-time tornado (development cost dominates). The figure of merit
(debris-risk reduction per cost) is reported under two independent
congestion weightings that can disagree about band priority (open trade
T5, kept visible rather than resolved by fiat); absolute per-catalog FoM is
mass-dominated under both. WP14 additionally derives a cited absolute-cost
**range** (never a point value) from an itemized cost table in which every
row carries a source or stays explicitly PLACEHOLDER (D10); the external
currency anchor is explicitly framed as low-standing relative to the
primary CU results, because an unaffiliated author's absolute-cost
estimate is inherently uncertain next to an agency's own parametric model.
Full model, itemized sources, and the honesty framing quoted verbatim from
the schema doc: [cost_model.md](cost_model.md).

## 6. Limitations

Stated plainly, none hidden: safety statements are exact only in the
linearized model families exercised (L0 Clohessy-Wiltshire; L1/L2 widen the
validated envelope but do not certify the real environment); inter-target
phasing Δv/time is a flat parameterized cost, not an optimized trajectory;
campaign-level sensor dispersions are drawn but not re-propagated through
the full closed loop at N=500 scale (the closed-loop sensor argument is
carried separately at smaller N); target inertia is assumed known (a real
mission needs inertia identification); estimator sensor biases are not yet
estimated, and this is measured, not merely disclosed, at L4; small-debris
(1–10 cm) removal is out of scope by physics, not neglect (collection-area
argument, full table cited); the regulatory precheck's jurisdiction
coverage is partial (UN/US/ITU/ESA references only; several adopter-state
national-law bodies are not yet covered); and 140 PLACEHOLDER marks remain
in the codebase (56 decision-critical), mechanically inventoried, not
hand-curated. None of this is claimed solved anywhere in this package.
Full list: [limitations.md](limitations.md), evidence pack §10.

## Key numbers (pinned, current committed values)

<!-- DOCS-5P-KEY-START -->
| metric | value | fidelity / notes | source |
|---|---|---|---|
| Campaign success (productive end), SL-16 / SL-8 | 0.556 [0.512, 0.599] / 0.542 [0.498, 0.585] (Wilson 95%) | L0, ds-v1 | `generated/wp5_campaign_summary.csv` |
| Keep-out violations, both classes | 0 / 500, Wilson UB ≤ 0.0076 | L0, ds-v1; zero re-verified over every re-verified abort event at L1/L2 | `generated/wp5_campaign_summary.csv`, `generated/wp12_ladder.csv` |
| Amortization minimum (N = 4) | 44.80 CU/removal = 0.285× the N = 1 baseline | Δv-limited, measured | `generated/wp6_cost_summary.csv` |
| FoM p50, SL-16 class (spatial / criticality) | 156.70 / 200.90 kg/CU | two independent weightings, disagreement tracked (T5) | `generated/wp6_cost_summary.csv` |
| Cost/removal p50, SL-16 class (low / mid / high) | 2.81 / 6.92 / 18.87 MUSD | external anchor, cited range only (D10) | `generated/wp6_cost_summary.csv` (WP14 rows) |
| Attitude sync (truth-driven pin) | 16.87 s | L0 | `reference_metrics.csv` |
| Attitude sync, estimate-driven under sensor noise | 17.07 s | L0, estimator-in-the-loop | `reference_metrics.csv` |
| Guided-approach demo: contact speed / total Δv | 0.10 m/s (gate 0.15 m/s) / 2.53 m/s | L0, truth-fed guidance | evidence pack §3, `reference_metrics.csv` |

*This block is generated by `tools/docs/fill_docs_numbers.py` from the same
committed CSVs as README's WP5-NUMBERS/WP6-NUMBERS blocks and
`reference_metrics.csv`; run with `--check` to verify, in-place to refresh.*
<!-- DOCS-5P-KEY-END -->
