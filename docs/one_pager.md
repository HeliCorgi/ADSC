# ADSC — One-Page Overview

> ADSC is not a debris collector. It is a low-cost concept for
> Kessler-precursor removal: a small servicer installs a passive deorbit kit
> on high-risk massive derelicts and departs, preventing future fragment
> clouds before they form — maximizing cleanup value per unit cost with a
> passively-safe approach design.

**What this is.** An open, reproducible **evidence package** for
installer-type active debris removal (a C++17 GNC simulator plus generated,
claim-audited artifacts) — not flight software and not a mission proposal
(TRL 4, GNC software element only). The product is the evidence pack itself:
every number regenerates byte-for-byte from a clean clone.

**The placeholder-independent core.** An installer never carries per-target
deorbit Δv and never tows or detumbles a multi-tonne stage; batch
amortization across several targets per mission is a mass-ratio scaling
argument that survives every open parameter — see README **Key results** /
[concept.md](concept.md).

## Five headline numbers (each sourced, none re-derived here)

1. **Batch amortization** drops cost-per-removal to a fraction of the
   single-target baseline at the Δv-limited optimum kit count — README
   **Key results** / [cost_model.md](cost_model.md).
2. **Approach safety**: zero keep-out violations across the full campaign
   catalog, re-verified at two higher fidelity levels (+J2, +drag) with zero
   violations — README **Safety status** / [safety.md](safety.md).
3. **Campaign robustness**: a majority-productive mission-success rate under
   full Monte-Carlo dispersion, reported with Wilson 95% confidence
   intervals, never as a bare point estimate — README **Key results** /
   [safety.md](safety.md).
4. **Kit trade**: a drag sail closes the 25-year deorbit guideline for the
   lighter reference class but not the heavier one; an electrodynamic
   tether is carried as an honest open candidate for the heavier class —
   [target_selection.md](target_selection.md).
5. **Figure of merit** (debris-risk reduction per cost) ranks the heavier
   reference class first under both weightings evaluated — cited, not
   asserted — [cost_model.md](cost_model.md) /
   `generated/wp14_prioritization.md`.

## The honest negatives

- **TRL 4, GNC software element only** — system-level TRL is not claimed;
  WP9 (processor-in-the-loop) is reserved and the only path to TRL 5.
- **Not flight software, not a mission proposal, not legal advice.**
- **Linear-CW scope**: exact passive-safety statements hold in the stated
  models (L0/L1/L2), not the real environment; a real mission re-verifies
  against a higher-fidelity propagator.
- **No inertia identification, no plane-change optimization, no homebrew
  breakup/plasma models.**
- **140 PLACEHOLDER marks remain (56 decision-critical)** — every one is
  inventoried, none is hidden. Full list: [limitations.md](limitations.md),
  evidence pack §10.
- **Absolute costs are cited low/mid/high ranges only** — never a
  point-value figure, and "minimum cost" is a banned absolute claim
  throughout (the defensible claim is cost-effectiveness).

## Reproduce

```
git clone https://github.com/HeliCorgi/ADSC.git && cd ADSC && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build && bash tools/regenerate_all.sh build
```

Full package: [README](../README.md) ·
[evidence pack](../evidence/adsc_evidence_pack.md).
