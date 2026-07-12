# Technical architecture

Not a mission proposal; TRL 4, GNC software element only
([roadmap.md](roadmap.md)). This page owns: the module map (which module
lives where and who else documents its numbers), the build/reproduction
instructions, the repository layout, and the CI reproducibility gate. GNC
algorithm detail (dynamics, controller, estimator, guidance mechanics) is
[gnc.md](gnc.md)'s; safety numbers are [safety.md](safety.md)'s; cost numbers
are [cost_model.md](cost_model.md)'s; kit/target trade numbers are
[target_selection.md](target_selection.md)'s; regulatory-precheck detail is
[legal_regulatory.md](legal_regulatory.md)'s. R16: this file states numbers
only via explicit references to committed artifacts, never a restated,
un-audited literal.

## Module map

Modules this file owns architecturally (mechanism, not GNC math or safety
numbers):

- **TMR fuel store** (`include/adsc/fuel_store.hpp`, `fuel_store.cpp`): three
  redundant copies guarded by a real CRC32 (IEEE, over the full 8-byte value
  via `memcpy`), majority voting, and scrubbing. Recovers single- and
  double-copy SEU flips and flags an unrecoverable triple failure instead of
  returning a silent bad value. Unit tested (`tests/test_fuel_store.cpp`).
  This is the avionics-philosophy component (D7: COTS + TMR/FDIR, not
  full rad-hard); TMR state is non-atomic, and threading assumptions are
  documented at call sites.
- **Fidelity ladder (WP12), one-code-path architecture**
  (`include/adsc/propagation.hpp`, `src/main_ladder.cpp`, target
  `adsc_ladder`): runtime-selectable **L0/L1/L2** propagation on **one code
  path** — no fork. L0 is the original WP1 Clohessy-Wiltshire linearization
  (every committed WP5 campaign number stays byte-identical under it); L1
  differences a full inertial two-body+J2 RK4 propagation of both craft into
  the target's instantaneous LVLH frame; L2 adds a per-craft free-molecular
  drag term under an *independent* ballistic-coefficient dispersion stream
  (`ds-v2`; the committed campaign's own `ds-v1` stream is untouched). L4
  (estimate-driven guidance) and L5 (delta-sigma minimum-impulse-bit actuator)
  extend the same ladder — see [gnc.md](gnc.md) for what each level changes
  about the control/estimation loop, and [safety.md](safety.md) for the
  re-verification numbers this architecture exists to produce.
- **Installer mission flow (WP3), phase-machine structure**
  (`include/adsc/mission.hpp`, `run_installer_mission`): the mission runs as
  a fixed phase sequence — approach → sync → attach → depart. Approach
  verifies the passively-safe corridor; sync gates on the WP2 tumble-sync
  criteria; attach clamps at the gated closing speed and hands the deorbit
  kit over (the servicer loses the kit mass, the target gains kit mass + sail
  area, changing its A/m); depart transfers to a bounded, keep-out-clearing
  relative orbit. The contact-energy budget this phase machine produces is a
  safety number ([safety.md](safety.md) owns it); the kit decay-trade
  numbers this phase feeds are [target_selection.md](target_selection.md)'s.
  Unit tested (`tests/test_mission.cpp`).
- **Visualization Pack (WP7a)** (`tools/viz/make_viz.py`, `decay_trade`
  target): a **Python 3 standard-library-only** generator (spec R9 tooling
  exception — no plotting library, no JS/CDN/external fonts; SVG is written
  directly) that turns the committed WP5/WP6/WP3 CSVs into nine static SVG
  figures plus a static report page. It reads every number from a committed
  CSV (nothing hand-written); each caption carries the source CSV, master
  seed and schema_version; no timestamps/run-times are embedded — a CI gate
  asserts regeneration matches the committed bytes. It does not draw from any
  live/streaming source, embed real imagery, or make interactive dashboards.
  Tested via `tools/viz/test_viz.py` (ctest `viz`).
- **Evidence Pack generator (WP7 — the actual product)**
  (`tools/evidence/make_evidence.py`): writes `evidence/adsc_evidence_pack.md`
  — executive summary, the installer argument, safety case, honest
  negatives, campaign/cost/FoM statistics, limitations, regulatory-precheck
  summary, reproduction instructions, and an auto-collected PLACEHOLDER
  inventory. Zero hand-written numbers: everything is machine-read from
  committed CSVs. A claim-audit test (ctest `evidence`) rejects forbidden
  overclaims (flight-ready/-proven, TRL 5/6 claims, "guaranteed",
  legal-conclusion words), verifies required honesty sections, cross-checks
  every quoted headline number against its source CSV, and enforces
  determinism + committed==regenerated. `tools/regenerate_all.sh` is the
  single regeneration entry point (CI runs exactly it — see below).

Modules whose mechanism lives here only in one line, with detail owned
elsewhere:

| Module | One-line role | Detail owned by |
|---|---|---|
| Rigid-body dynamics, SMC controller, tumble sync, point-mass inertia, relative motion (CW) | core GNC math | [gnc.md](gnc.md) |
| F1 capped-abort honesty, WP11 clearing-abort law + reachability screen | safety argument + campaign numbers | [safety.md](safety.md); guidance-law mechanics in [gnc.md](gnc.md) |
| Estimator + sensor models (WP4) | translation EKF + attitude MEKF | [gnc.md](gnc.md) |
| Campaign Monte-Carlo (WP5) | N=500/catalog dispersion campaign | [safety.md](safety.md) |
| Cost model + FoM (WP6) | relative-CU cost, WP14 absolute ranges | [cost_model.md](cost_model.md) |
| Kit decay trades, WP13 EDT physics (WP3/WP13) | sail/EDT trade, per-class kit table | [target_selection.md](target_selection.md) |
| Compliance Matrix Generator (WP8) | regulatory precheck, not legal advice | [legal_regulatory.md](legal_regulatory.md) |
| First-order PCM thermal budget, deorbit gating | minor subsystem detail | [gnc.md](gnc.md) |

## Build

Requires a C++17 compiler and Eigen 3.3+; Python 3 stdlib only for tooling
(spec R9 — no third-party Python packages).

```
git clone https://github.com/HeliCorgi/ADSC.git
cd ADSC
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # add -DADSC_WERROR=ON for R3
cmake --build build
```

## Reproduce every committed artifact

`bash tools/regenerate_all.sh build` runs the following fixed order (its own
header comment is the single source of truth; CI runs exactly this script):

| Step | Regenerates | Command |
|---:|---|---|
| 1/12 | WP5 campaign Monte-Carlo (N=500 × 2 catalogs) | `./build/adsc_campaign 500 generated` |
| 2/12 | WP12 fidelity-ladder re-verification (L0/L1/L2) | `./build/adsc_ladder 500 generated` |
| 3/12 | WP6 cost model + FoM (kit sweep, consumes WP5) | `./build/adsc_cost 500 generated` |
| 4/12 | WP3 decay-trade CSV | `./build/decay_trade generated` |
| 5/12 | WP13 kit-class trade + class-C comparison | `./build/kit_trade generated` |
| 6/12 | WP14 prioritization table (joins WP6 + WP13) | `python3 tools/prioritization/make_prioritization.py . generated` |
| 7/12 | T6 small-debris flux sweep | `./build/flux_sweep generated` |
| 8/12 | Reference metrics (pinned WP1/F1/WP2/WP3/WP4 numbers) | `./build/sim_metrics generated` |
| 9/12 | WP10c keep-out-violation forensics (read-only replay) | `python3 tools/forensics/make_forensics.py` |
| 10/12 | WP7a visualization pack | `python3 tools/viz/make_viz.py . generated/viz` |
| 11/12 | WP8 compliance precheck + matrix | `python3 tools/compliance/check_compliance.py`; `python3 tools/compliance/generate_matrix.py` |
| 12/12 | WP7 evidence pack | `python3 tools/evidence/make_evidence.py` |

```
./build/adsc_sim                 # closed-loop detumble driver (also scenarios 5/6/7, see gnc.md)
bash tools/regenerate_all.sh build   # ...or regenerate EVERYTHING above in order (CI runs this)
ctest --test-dir build           # fuel_store/relmotion/sync/decay/mission/estimator/campaign/
                                  # cost/flux/ladder/guidance/forensic14/viz/compliance/evidence/
                                  # forensics/prioritization/readme_numbers
```

When invoking bare `g++` directly, prefer passing Eigen via `-isystem` to
avoid GCC `-Wmaybe-uninitialized` false positives; CMake handles this
automatically.

## Layout

```
include/adsc/   public headers (fuel_store, dynamics, controller, thermal,
                relmotion, decay, estimator, mission, campaign, cost, flux,
                propagation)
src/            implementations + main.cpp sim + WP3/WP5/WP6/WP13/T6 emitters (main_*.cpp)
tests/          C++ unit tests (+ Python tests under tools/, ctest viz/compliance/
                forensics/prioritization/readme_numbers)
tools/viz/      WP7a Python 3 stdlib-only visualization generator (make_viz.py)
tools/compliance/  WP8 regulatory precheck: rulepacks, schema, checker, matrix generator
tools/evidence/    WP7 evidence-pack generator + claim-audit test
tools/prioritization/  WP14 prioritization-table generator (joins WP6 + WP13)
tools/forensics/   WP10c keep-out-violation forensics (read-only replay of WP5)
tools/readme/      README.md marker-block filler + test (WP14 review C4 fix)
tools/regenerate_all.sh  single regeneration entry point (CI runs exactly this)
generated/      committed artifacts: WP5 campaign, WP6 cost/FoM, T6 flux, WP3 decay,
                WP13 kit trade + class-C, WP14 prioritization, reference_metrics CSVs,
                viz/ SVGs, compliance_findings.json, WP10/WP11 forensics,
                WP12 fidelity-ladder re-verification
evidence/       committed generated evidence artifacts (adsc_evidence_pack.md,
                compliance matrix; same reproducibility gate as generated/)
docs/           concept, technical architecture (this file), gnc, safety,
                target_selection, cost_model, legal_regulatory, limitations,
                roadmap, pitch_agencies (spec:274), plus the one-pager,
                5-page summary, deck source and release-engineering notes
                (WP15 tasks 4-5)
.github/        CI: 4 jobs -- (1) Ubuntu/GCC build+ctest+regenerate+
                reproducibility gate (single authority), (2) Ubuntu GCC+Clang
                build+ctest matrix, (3) Debug ASan/UBSan ctest-only, (4)
                Windows+macOS Python-suite cross-platform check (see "CI
                gates" below)
adsc-specification-v5.md   active spec (work packages, hard rules, locked decisions)
adsc-specification-v4.md   retired v4.2 spec (superseded by v5; kept because v5
                           references its D/R/T items)
```

## CI gates

`.github/workflows/ci.yml` runs four jobs on every push and pull request.
Only the first is authoritative for reproducibility; the other three are
non-gating hardening layers added by the CI-hardening mini-WP
(adsc-specification-v5.md:296-300) and upload no artifacts.

1. **`build-test` (Ubuntu / GCC) -- the single authority.** Installs Eigen3
   (apt), configures with `-DADSC_WERROR=ON` (R3, warnings-as-errors), builds,
   runs `ctest`, runs `./build/adsc_sim`, then runs
   `bash tools/regenerate_all.sh build` (the exact script above — "single
   source of truth for reproduction" per its own header),
   `python3 tools/readme/fill_readme_numbers.py .` (fills README's two number
   marker-blocks from the committed WP5/WP6 CSVs) and
   `python3 tools/docs/fill_docs_numbers.py .` (fills every `docs/*.md`
   DOCS-*-START/END marker block from the CSVs it cites — the docs/ sibling
   of the README step, same "separate step before the gate" convention,
   since neither README.md nor docs/ lives under `generated/` or
   `evidence/`). It then runs the **reproducibility gate**:

   ```
   git diff --exit-code -- generated/ evidence/ README.md docs/
   ```

   — any drift between committed bytes and freshly regenerated bytes fails
   the build (R6). This is the one job that gates reproducibility; it is
   the only job that uploads artifacts (`adsc-generated`, even on gate
   failure, so an R15 re-baselining PR can take CI's regenerated bytes as
   its new committed reference instead of requiring a local toolchain).

2. **`build-matrix` (Ubuntu / GCC + Clang).** Configures Release with
   `-DADSC_WERROR=ON` under both compilers and runs `ctest` — catches
   compiler-specific warnings/UB a single-compiler job would miss. Stops at
   `ctest`: no regeneration, no fillers, no gate, no artifact upload.

3. **`sanitizers` (Debug, ASan+UBSan, ctest only).** Configures a Debug build
   with `-fsanitize=address,undefined` injected via `-DCMAKE_CXX_FLAGS`/
   `-DCMAKE_EXE_LINKER_FLAGS` at the CI-job level (no `ADSC_SANITIZE` option
   was added to `CMakeLists.txt` — this is a zero-CMakeLists-change route),
   then runs `ctest` restricted to the C++ unit tests. Per
   adsc-specification-v5.md:287-291/297-300 ("sanitizers on TEST builds
   only... sanitizer-built binaries must never write `generated/` or the SHA
   gate dies"), this job never runs `adsc_sim`, `tools/regenerate_all.sh`,
   the fillers, or the gate, and excludes the Python-driven ctest tests
   (`viz`, `compliance`, `evidence`, `forensics`, `prioritization`,
   `readme_numbers`, `docs_numbers`) since those regenerate-in-place against
   `generated/`/`evidence/`/`README.md`/`docs/` by design and exercise no
   sanitizer-instrumented binary.

4. **`python-cross-platform` (Windows + macOS).** Runs the Python 3
   stdlib-only `evidence`/`compliance`/`viz`/`forensics`/`prioritization`/
   `readme_numbers`/`docs_numbers` suites plus both fillers in `--check` mode
   directly against the artifacts already committed under `generated/` and
   `evidence/` — no C++ build, no Eigen setup. This turns the
   "byte-identical across platforms" claim from author assertion into
   CI-enforced evidence. C++ regeneration itself stays Ubuntu-only (job 1):
   standing up Eigen3 + a C++ toolchain on Windows/macOS runners is
   disproportionate cost that this job's Python-only check already makes
   unnecessary for the reproducibility claim.

**docs/ and the reproducibility gate.** R16 requires every number under
`docs/` to come from a generated include or an explicit reference, with the
same drift protection `generated/`/`evidence/`/`README.md` already have. As
of WP15 task 3, the gate's diff path above includes `docs/` and the
`docs/`-scoped claims-audit (banned-word, numeric-consistency, L-tag
patterns, run as part of the `evidence` ctest target/`test_evidence.py`) is
wired in — this is no longer an open item.
