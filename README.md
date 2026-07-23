# ADSC — Active Debris Self-Cleanup

> ADSC is not a debris collector. It is a low-cost concept for
> Kessler-precursor removal: a small servicer installs a passive deorbit kit
> on high-risk massive derelicts and departs, preventing future fragment
> clouds before they form — maximizing cleanup value per unit cost with a
> passively-safe approach design.

**What this is:** an open, reproducible **evidence package** for
installer-type active debris removal — a C++17 GNC simulator plus generated,
claim-audited artifacts. **The product is
[evidence/adsc_evidence_pack.md](evidence/adsc_evidence_pack.md)**: every
number in it (and in this README) is machine-read from committed artifacts and
regenerates byte-for-byte from a clean clone. This is **not flight software and
not a mission proposal** (TRL 4, GNC software element, simulation environment).

**Why:** the collisional cascade's fuel is the population of massive derelict
upper stages in congested bands — fragments are the symptom. The servicer
targets the objects that would become the next fragment clouds, attaches a
passive deorbit kit (drag sail or electrodynamic tether), and departs. What an
agency already has — funded capture/tug programs (ClearSpace-1, ADRAS-J2) and
published kit physics — does not include an open, honesty-audited **trade for
the installer architecture**; that is the niche this package fills
([docs/concept.md](docs/concept.md)).

**The placeholder-independent core:** an installer never carries per-target
deorbit Δv and never tows or detumbles a multi-tonne stage; batch amortization
drives cost-per-removal to **28.5 % of the single-target baseline** at N = 4
kits (Δv-limited, measured — see Key results). This is a mass-ratio scaling
argument that survives every open parameter.

## What is implemented / not implemented

Closed-loop GNC (sliding-mode attitude control, translation EKF + attitude
MEKF, guided approach with reachability-screened aborts), campaign
Monte-Carlo (N = 500/catalog, Wilson CIs), a WP12 fidelity ladder (CW → +J2 →
+drag; estimate-driven guidance; minimum-impulse-bit actuator), kit decay
trades (sail + WP13 EDT physics band), a relative-unit cost model with WP14
absolute-cost ranges, a regulatory precheck (not legal advice), and the
generated evidence pack. Module-by-module detail:
[docs/technical_architecture.md](docs/technical_architecture.md) ·
[docs/gnc.md](docs/gnc.md).

Deliberately **not** implemented: flight-qualified code (WP9 reserved — the
only path to TRL 5), inertia identification, plane-change optimization,
homebrew breakup models, IRI plasma, absolute point-value costs, legal
determinations. Full honest scope: [docs/limitations.md](docs/limitations.md)
and the evidence pack's PLACEHOLDER inventory (140 marks, 56 decision-critical
— one glance away, by design).

## Key results

<!-- WP5-NUMBERS-START (filled from CI adsc_campaign, seed 0x5AD5C0DECAFE2026) -->
| campaign (N = 500/catalog) | SL-16 class | SL-8 class |
|---|---|---|
| productive-end rate | **0.556** [0.512, 0.599] | **0.542** [0.498, 0.585] |
| keep-out violations | **0.000** [0.000, 0.008] [L0, ds-v1] | **0.000** [0.000, 0.008] [L0, ds-v1] |
| Δv used p50 [m/s] | 124 | 124 |
| removals/mission p50 | 4 | 4 |

Full tables + failure taxonomy: [docs/safety.md](docs/safety.md).
<!-- WP5-NUMBERS-END -->

<!-- WP6-NUMBERS-START (filled from CI adsc_cost) -->
| cost/FoM (relative CU; WP14 MUSD ranges) | value |
|---|---|
| amortization minimum (N = 4) | **44.80 CU/removal = 0.285× the N = 1 baseline** |
| FoM p50, SL-16 (spatial / criticality) | 156.70 / 200.90 kg/CU |
| derived anchor (low / mid / high) | 0.0628 / 0.1546 / 0.4213 MUSD/CU |
| cost/removal p50 at the anchor | 2.81 / 6.92 / 18.87 MUSD |

Full model + sources-or-PLACEHOLDER itemization:
[docs/cost_model.md](docs/cost_model.md).
<!-- WP6-NUMBERS-END -->

Kit-class trade (WP13): sail closes for the ~1.4 t class (7..115 m²) but
**not** for the ~9 t class (135..2155 m²); EDT is an honest **candidate with
open risks** for the heavy class (2.98..9.17 yr band; libration T7 unresolved)
— [docs/target_selection.md](docs/target_selection.md).

## Safety status

Approach safety is a **passively-safe approach design with clearance-verified
aborts (keep-out violations 0/500 per catalog [L0, ds-v1, Wilson ≤ 0.0076])**,
re-verified at L1 (+J2) and L2 (+drag) with zero violations over the 292/291
re-verified abort events [ds-v1/ds-v2] — never "strict approach safety": the
guarantees are model-scoped (R14 tags everywhere), and a real mission would
re-verify against a higher-fidelity propagator.
[docs/safety.md](docs/safety.md).

## Reproduce

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
bash tools/regenerate_all.sh build     # regenerates EVERY committed artifact
```

CI enforces byte-identity of `generated/`, `evidence/`, this README's number
blocks, and `docs/` on every push. Requires C++17 + Eigen 3.3; Python 3
stdlib only for tooling. Details: [docs/technical_architecture.md](docs/technical_architecture.md).

## Structure

```
evidence/            the product: generated, claim-audited evidence pack
docs/                concept, architecture, gnc, safety, targets, cost,
                     legal, limitations, roadmap, adoption brief, one-pager,
                     5-page summary, deck source, release engineering
generated/           committed machine-readable artifacts (CSV/md/SVG)
include/ src/ tests/ C++17 simulator + unit tests
tools/               stdlib-only generators, audits, compliance precheck
adsc-specification-v5.md   the binding spec (rules R1-R16, decisions D1-D13)
```

## Roadmap

WP1–WP8 ✅ · WP10 forensics ✅ · WP11 safety hardening ✅ · WP12 fidelity
ladder ✅ · WP13 kit trade + EDT ✅ · WP14 cost ranges + FoM ✅ · **WP15
proposal package ✅ (this release)** · WP9a flight-software groundwork
(software-only, TRL unchanged) ✅ · WP16 digital twin Phase 1 (owner-directed
extension, not spec-mandated; T7 libration trade still OPEN — see
[docs/digital_twin.md](docs/digital_twin.md)) · WP9 processor-in-the-loop —
**reserved, not started, the only path above TRL 4** (WP9's pre-registered
plan: [docs/wp9_pil_plan.md](docs/wp9_pil_plan.md)). Detail:
[docs/roadmap.md](docs/roadmap.md).

## Limitations

TRL 4 applies to the GNC software element only; system-level TRL is not
claimed. Passive-safety numbers hold in the stated models, not the real
environment. Decision-critical inputs remain PLACEHOLDER until cited
(inventoried automatically in the evidence pack §10).
[docs/limitations.md](docs/limitations.md) is the honest one-stop list.

## License / Disclaimer

MIT ([LICENSE](LICENSE)). Conceptual prototype for peaceful ADR research and
education only; no military or re-entry-weapon application is intended.
Regulatory outputs are research prechecks, **not legal advice**.

本プロジェクトは平和的デブリ除去技術の教育・研究目的の概念プロトタイプです。
実際の宇宙機運用には専門機関による設計審査・認定・国際法遵守が必要です。
