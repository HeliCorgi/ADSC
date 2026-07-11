# Concept

> This is a concept dossier for a research/concept-study audience — **not a
> mission proposal**. Nothing here is legal advice, a flight-readiness claim,
> or a commitment on behalf of any agency. TRL 4, GNC software element only
> (see [roadmap.md](roadmap.md)).

## What ADSC is

ADSC is an open-source, reproducible **evidence package** for installer-type
active debris removal, framed as **Kessler-precursor removal**: instead of
towing large debris or chasing fragments, a small servicer installs a passive
deorbit kit on the massive derelicts that would otherwise become the next
fragment clouds, and departs. The audience is mission designers at the
agencies and contractors of major launching states. The product is
credibility; the success criterion is that a skeptical engineer can clone the
repository and regenerate every claimed number
(`bash tools/regenerate_all.sh build`, byte-identical across three independent
platforms — see [technical_architecture.md](technical_architecture.md)).

Canonical summary blurb, used verbatim (spec §1):

> ADSC is not a debris collector. It is a low-cost concept for
> Kessler-precursor removal: a small servicer installs a passive deorbit kit
> on high-risk massive derelicts and departs, preventing future fragment
> clouds before they form — maximizing cleanup value per unit cost with a
> passively-safe approach design.

The owner's one-line governing value for v5: maximum orbital-environment
cleanup value per unit cost, with approach safety treated as a design
requirement rather than a statistic. That approach safety is a
**passively-safe approach design with clearance-verified aborts (keep-out
violations 0/500 per catalog [L0, ds-v1, Wilson ≤ 0.0076])**, re-verified at L1
(+J2) and L2 (+drag) — never "strict approach safety" or "safe satellite";
the full campaign statistics and re-verification tables are
[safety.md](safety.md)'s to own, not repeated here.

"Minimum cost" as an absolute claim is banned throughout this package. The
defensible claim is **cost-effectiveness — cost per removal and cost per unit
risk reduction** (see [cost_model.md](cost_model.md)).

## Kessler-precursor removal — the cascade framing

The collisional cascade's fuel is the population of massive intact derelicts
in congested orbital bands; fragments are the symptom. One intact-intact
collision produces thousands of trackable fragments — the 2009 Iridium 33 /
Cosmos 2251 collision alone produced 598 + 1603 = 2201 fragments cataloged by
the U.S. Space Surveillance Network as of January 2013 (Liou, J.-C., "An
Analysis of the FY-1C, Iridium 33, and Cosmos 2251 Fragments", NASA Orbital
Debris Program Office, NTRS 20150003820, 2014). Removing, or equipping for
removal, the objects that would become the next fragment clouds attacks the
source term rather than the symptom — that is what **Kessler-precursor
removal** names.

Anchors in the open literature:

- Kessler, D. J., Cour-Palais, B. G., "Collision Frequency of Artificial
  Satellites: The Creation of a Debris Belt", *Journal of Geophysical
  Research* 83(A6), 1978 — the cascade mechanism itself.
- Liou, J.-C., Johnson, N. L., "Risks in Space from Orbiting Debris",
  *Science* 311(5759), 2006 — LEO population growth even without new launches
  (the instability argument).
- Liou, J.-C., Johnson, N. L., Hill, N. M., "Controlling the growth of future
  LEO debris populations with active debris removal", *Acta Astronautica*
  66(5-6), 2010 — the classical few-removals-per-year
  environment-stabilization-class result (~5 objects/yr, jointly with ~90%
  post-mission-disposal compliance — removal alone does not carry the result)
  that sizes the campaign cadence ADSC targets.
- McKnight, D., et al., "Identifying the 50 statistically-most-concerning
  derelict objects in LEO", *Acta Astronautica* 181, 282-291, 2021,
  doi:10.1016/j.actaastro.2021.01.021 — a consensus ranking (11 expert teams)
  dominated at the top by SL-16 / Zenit-2 second stages; average mass of the
  top-50 ≈ 5,295 kg; removing the top-50 would roughly halve LEO
  collision-risk (top-10 ≈ 30% reduction). This is the citation basis for
  ADSC's reference target classes (D2).
- ESA Space Debris Office, "ESA's Annual Space Environment Report",
  GEN-DB-LOG-00288-OPS-SD, Issue 10.0, 1 May 2026, §7.1 — the
  environmental-index peak near ~800-900 km / 70-80° inclination matches
  ADSC's catalog_A reference class (~840 km / ~71°), under a 90%
  post-mission-disposal-success assumption.
- Population figures (≥1 cm ~1.2 M objects) are MASTER-8-class model values
  (MASTER-8, released March 2019, reference epoch November 2016; Horstmann,
  A., Hesselbach, S., Wiedemann, C., "Enhancement of S/C Fragmentation and
  Environment Evolution Models", Final Report, ESA contract
  4000115973/15/D/SR, 2020), carried as PLACEHOLDER inputs in the T6 flux
  table (`generated/t6_flux_sweep.md`).

These are bibliographic/regulatory citations (years, DOIs, report editions),
not ADSC simulation results, and are cited here rather than machine-included
from a committed CSV.

## Installer, not tug — the quantitative argument (D1)

Deorbit Δv scales with the mass it must decelerate. A tug must decelerate the
full ~9,000 kg SL-16-class stage — roughly 304× the entire ADSC servicer's
own contact mass (29.6 kg dry+kit; see evidence pack §2) — and a realistic tug
bus would be heavier still. The installer instead transfers a 2.4 kg passive
kit and departs, so the propellant cost of the deorbit itself is carried by
drag (sail) or electrodynamic drag (tether), not by the servicer. One mission
services several targets in one plane (batch amortization —
[cost_model.md](cost_model.md) owns the amortization-curve numbers); the
capture interface is a geometry-keyed clamp on features present on every
upper stage by construction (nozzle throat / adapter ring, D4), not a generic
manipulator. This mass-ratio argument is the placeholder-independent core of
the package: it survives every open parameter, because it is a scaling
argument, not a point estimate.

The GNC element itself is textbook-level by design (D11): Clohessy-Wiltshire
relative motion, quaternion sliding-mode tracking, multiplicative EKF — open
literature methods applied to class-level targets, with no live ephemerides
and no target-specific operational products
([gnc.md](gnc.md) owns the mechanism detail).

## What an agency already has — the counterfactual

An agency evaluating this package already has, or can buy, several adjacent
things. None of them is the installer trade this package makes:

- **A funded capture/tug-class demonstration.** ESA's ClearSpace-1 mission —
  contracted at **~€86M** (November 2020; ~€100M total project cost), now
  targeting the defunct PROBA-1 satellite (retargeted from the Vespa adapter
  in April 2024) — is presently expected to launch **~2028**. This is a
  single-target capture-and-deorbit tug demonstration, not a multi-target
  batch-amortized installer, and it does not by itself answer the
  installer-vs-tug mass-ratio question this package argues quantitatively.
- **A funded national rocket-body-removal demonstration.** JAXA's Commercial
  Removal of Debris Demonstration Phase II (CRD2) — Astroscale Japan's
  ADRAS-J2, contracted at **~¥13.2B** (2024) — targets an unprepared Japanese
  upper stage, with the demonstration presently planned toward **FY2027–28**.
  Like ClearSpace-1, this is a single-target removal demonstration by a funded
  agency program; it validates that agencies will pay for large-object
  removal, not that an installer architecture is the cost-effective way to do
  it at scale.
- **Published passive-kit physics, without an installer-architecture trade.**
  The bare electrodynamic tether (EDT) literature is mature and citable:
  Sanmartín, Martínez-Sánchez & Ahedo's 1993 OML bare-tether foundation paper
  (*J. Propulsion and Power* 9(3), 353-360, doi:10.2514/3.23629), and the EU
  FP7 BETs project (GA 262972, coordinated by Prof. Sanmartín, UPM,
  2010-2014), which demonstrates order-of-magnitude tether-mass-vs-deorbit-time
  results (e.g. a 10 kg tether deorbiting a 1000 kg spacecraft from 1000 km in
  about a month). Drag-sail deorbit is similarly well published. What this
  literature does not provide is an open, honesty-audited trade of the
  *installer architecture itself* — batch amortization across several
  targets per mission, the geometry-keyed capture interface, and a
  campaign-level Monte-Carlo safety case with clearance-verified aborts.

**The niche this package fills**: an open, reproducible, honesty-audited
*installer-architecture trade* — not a new capture demonstration and not a
new tether-physics result — sitting between the funded single-target
tug/removal programs above and the published component-level kit physics,
making the batch-amortization and installer-vs-tug argument in a form a
skeptical engineer can regenerate byte-for-byte.

## Scope guardrails

- **Operator-legality framing (D9).** The package assumes the operator is, or
  is contracted/consented by, the launching state of the target. Nothing in
  this repository assumes or enables unconsented approach to another state's
  object.
- **Dual-use guardrail (D11).** Methods stay at open-literature level (CW,
  sliding-mode control, EKF are textbook material). Targets remain
  class-level parameters: no mission-planning products against specific
  catalog IDs, and no ingestion of live ephemerides/TLEs.
- **Language discipline (spec §1, binding across README/docs/evidence pack).**
  The anchor phrase **"Kessler-precursor removal"** appears in every summary
  text, including this one. Adoption-facing deliverables — README, docs/,
  pitch docs — stay in English (R11); a Japanese equivalent of the summary
  blurb may accompany it only in owner-facing material, not here.
