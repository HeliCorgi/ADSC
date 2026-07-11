# ADSC — Adoption Brief for Mission-Design Teams

> This is an **adoption brief / concept dossier for mission-design teams —
> not a mission proposal** and not a funding request. It is agency-agnostic
> by design: ADSC seeds ideas for any launching state's space agency or its
> contractors (spec §1 identity), not a pitch addressed to one in particular.
> Nothing here is legal advice, a flight-readiness claim, or a commitment on
> behalf of any agency. TRL 4, GNC software element only — see
> [roadmap.md](roadmap.md).

ADSC is a low-cost concept for **Kessler-precursor removal**: a small
servicer installs a passive deorbit kit on high-risk massive derelicts and
departs, preventing future fragment clouds before they form — maximizing
cleanup value per unit cost with a passively-safe approach design (spec §1
blurb, verbatim). Full framing: [concept.md](concept.md).

## Who this is realistically for

Read plainly, not oversold: the nearer-term, higher-confidence value of this
package today is as a citable, reproducible reference for the active-debris-
removal research and concept-study community. Adoption by an agency
mission-design process is the aspirational case this brief is written for,
not a claimed outcome — an internal review of the honest odds put
P(this artifact alone moves an agency design decision) low, and
P(it earns citation/reference value in the ADR research community) high.
That asymmetry is why the brief leads with the argument and the evidence,
not with a request.

## The core argument — survives every open parameter

An installer never carries per-target deorbit Δv and never tows or
detumbles a multi-tonne stage; it transfers a small passive kit (drag sail
or electrodynamic tether) and departs, so the deorbit propellant cost is
carried by drag, not by the servicer. Batch amortization across several
targets in one mission drives cost-per-removal down sharply as kit count
grows, until the Δv budget — not the kit count — caps removals (measured,
not asserted; see README **Key results** / [cost_model.md](cost_model.md)
for the amortization-minimum figure and the curve that produces it). This
is a mass-ratio scaling argument, not a point estimate: it holds regardless
of which cost coefficient or target mass is still PLACEHOLDER. Lead with
this when the placeholder-dependent numbers invite scrutiny — the argument
does not depend on them.

## What is done vs. what is research — one glance

| Done (simulated, reproducible, TRL 4 GNC element) | Research / explicitly open |
|---|---|
| Closed-loop attitude (SMC) + translation (EKF) GNC, campaign Monte-Carlo robustness, WP12 fidelity ladder (L0 CW → L1 +J2 → L2 +drag, one code path), clearing-abort guidance | Flight-qualified code (WP9, reserved — the only path to TRL 5) |
| Passive-kit decay trade: drag sail + WP13 electrodynamic-tether (EDT) physics band | EDT libration dynamics (open trade T7), plasma-density citation gaps |
| Relative-cost-unit (CU) model, primary metric, plus WP14 cited absolute-cost ranges | Point-value absolute costs (excluded by design — see below) |
| Regulatory precheck against versioned rulepacks (UN/FCC/ITU/ESA references) | Legal determinations — the precheck is explicitly **not legal advice** |
| Reproducible, byte-identical evidence pack regenerated from a clean clone | Inertia identification, plane-change optimization, homebrew breakup/plasma models |

Full honest-scope list, one click away: [limitations.md](limitations.md) and
the evidence pack's PLACEHOLDER inventory (140 marks, 56 decision-critical —
evidence pack §10).

## Own the counterfactual: what an agency already has

None of the following is being reproduced or claimed here; they are cited
for context so the niche this package fills is stated plainly rather than
implied. Full citations and dates: [concept.md](concept.md).

- **A funded capture/tug-class demonstration.** ESA's ClearSpace-1 —
  contracted at ~€86M, now targeting the defunct PROBA-1 satellite,
  presently expected to launch ~2028 — is a single-target capture-and-
  deorbit tug demonstration. It does not answer the installer-vs-tug
  mass-ratio question this package argues quantitatively.
- **A funded national rocket-body-removal demonstration.** JAXA's CRD2 —
  Astroscale's ADRAS-J2, contracted at ~¥13.2B — targets a single
  unprepared upper stage, presently planned toward FY2027–28. It validates
  that agencies will pay for large-object removal, not that an installer
  architecture is the cost-effective way to do it at scale.
- **Published passive-kit physics, without an installer-architecture
  trade.** The bare-electrodynamic-tether literature (Sanmartín et al.,
  the EU FP7 BETs project) and the drag-sail deorbit literature are mature
  and citable. Neither supplies an open, honesty-audited trade of the
  installer architecture itself.

(These external figures are publicly reported program numbers, not
ADSC-computed results — no committed CSV backs them, and they are outside
this package's own numeric-consistency scan by construction; they are cited
for context, not audited as evidence-pack claims.)

**The niche**: an open, reproducible, honesty-audited trade for the
*installer* architecture specifically — attach-a-kit-and-depart, batch
amortization, geometry-keyed capture, a campaign-level Monte-Carlo safety
case — sitting between the funded single-target programs above and the
published component-level kit physics. That combination does not otherwise
exist in the open literature.

## Safety case, stated the required way

Approach safety is a **passively-safe approach design with
clearance-verified aborts (keep-out violations 0/500 per catalog [L0,
ds-v1, Wilson ≤ 0.0076])** — never "strict approach safety" or "safe
satellite." Re-verified at L1 (+J2) and L2 (+drag) with zero violations
over the re-verified abort events, per catalog (README **Safety status** /
[safety.md](safety.md)). A pre-WP11 legacy baseline showed a nonzero
violation rate, archived under R15 at `v0.10-phase0-baseline`; WP11 closed
it, and the guarantees remain model-scoped (R14 tags throughout) — a real
mission re-verifies every coast against a higher-fidelity propagator before
flight. Full re-verification table and campaign statistics:
[safety.md](safety.md).

## Kit trade and target prioritization

The kit trade is an honest negative-first result: a drag sail closes the
IADC 25-year guideline for the ~1.4 t class but does **not** close it for
the ~9 t class (impractical sail area at that mass); an electrodynamic
tether is carried as an honest **candidate with open risks** for the heavy
class, not a closed recommendation (libration trade T7 unresolved). The
WP14 target-prioritization table ranks the ~9 t class first on
figure-of-merit under both weightings evaluated, with the ~1.4 t class
second (closes today with a practical sail kit) and a separate
controlled-reentry mission class evaluated on its own terms, not
kit-ranked against the two above. Full trade, per-class recommended kit,
and the prioritization defense paragraphs:
[target_selection.md](target_selection.md) and
`generated/wp14_prioritization.md`.

## Cost: cost-effectiveness, never "minimum cost"

"Minimum cost" as an absolute claim is banned throughout this package
(spec §1). The defensible, and only claimed, framing is
**cost-effectiveness — cost per removal and cost per unit risk
reduction** — reported as relative cost units (CU, primary) plus cited
low/mid/high absolute-cost ranges (never a point-value currency figure,
D10). Headline numbers, all sourced from committed artifacts, not
re-derived here: README **Key results** and [cost_model.md](cost_model.md).

## Reproduce in one command

```
git clone https://github.com/HeliCorgi/ADSC.git && cd ADSC && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build && bash tools/regenerate_all.sh build
```

A clean clone regenerates every committed number byte-for-byte
(`git diff --exit-code -- generated/ evidence/ README.md docs/` stays
clean). Requirements: C++17 + Eigen 3.3+, CMake, Python 3 stdlib only.

## Contact

This project has no dedicated inbox or gatekeeper by design (agency-agnostic,
per spec §1) — open an issue or discussion on the repository:
<https://github.com/HeliCorgi/ADSC>.
