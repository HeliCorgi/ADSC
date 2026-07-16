# WP9 PIL Pre-Registration Plan (WP9a groundwork + frozen WP9 criteria)

Not a mission proposal; TRL 4, GNC software element only
([roadmap.md](roadmap.md), spec section 9). This page is a **pre-registration
plan**, not a maturity change. It freezes, before any hardware exists, the
criteria a future WP9 processor-in-the-loop (PIL) campaign would have to meet,
and scopes the software-only groundwork (WP9a) that can be done now on a
workstation. Binding spec: `adsc-specification-v5.md` section 9 (TRL statement)
and section 6 (WP9 RESERVED, the only path above element TRL 4).

Two hard facts govern every line below:

1. **TRL stays 4.** Nothing in this plan, and nothing WP9a can produce without
   hardware, raises the TRL. WP9 PIL itself remains **RESERVED, not started**.
   This document is a plan for reserved work; writing the plan is not doing the
   work.
2. **No committed generated artifacts for WP9.** PIL and soak measurements are
   timing-dependent and non-deterministic; committing them would break the
   repository's byte-identity reproducibility gate
   (`git diff --exit-code -- generated/ evidence/`). Every WP9/WP9a measurement
   therefore prints to stdout / the CI log and is referenced GAP-style
   (e.g. `regenerate: ctest -R wp9_flightlike` for the WP9a probe committed
   here; a future WP9 PIL campaign would register its own ctest name under
   the same discipline), the same honesty pattern the package already uses
   for prose-pinned, non-committed numbers (see [gnc.md](gnc.md) WP1/WP2
   console-only figures).

Number discipline (R16): this document is the **home** for the WP9 completion
thresholds it freezes; it does not restate any pinned result number owned by
another file. Threshold values here are pre-registered design targets, not
measured results. External hardware/RTOS prices are rough, sourced-or-
PLACEHOLDER (D10), never folded into the FoM (D12).

---

## 1. Purpose and freeze status

WP9 is expensive and hard to reverse: it means buying representative avionics,
bringing up an RTOS, and porting the GNC control path off the workstation. The
purpose of pre-registration is to decide the pass/fail criteria **before** the
hardware is in hand, so a later "it basically works" cannot be talked into a
TRL claim it does not support. This mirrors the project's falsification
discipline: fixed criteria and fixed kill-conditions, set in advance.

Freeze rule: the criteria in sections 3 and 6 are frozen. Changing any of them
follows R15 (a BEFORE/AFTER entry with the reason, archived in the evidence
pack changelog). The exact procured part number and its clock are pre-registered
by an R15 addendum to this file at procurement time, not chosen after seeing
results.

WP9a (software-only groundwork, section 5) can start now and needs no hardware.
WP9 proper (PIL on the board, sections 3-4) stays reserved until, at minimum,
WP9a has run and none of the section 6 kill-criteria fired.

---

## 2. What TRL 5 for a GNC software element requires

### 2.1 The TRL 4 -> TRL 5 gap, honestly cited (quoted, not claimed)

NASA defines the two levels this project sits between as (short definitions):

- **TRL 4 - "Component/subsystem validation in a laboratory environment."**
  Standalone prototype implementation and test; integration of technology
  elements; experiments with full-scale problems or data sets.
- **TRL 5 - "System/subsystem/component validation in a relevant environment."**
  Thorough testing of the prototype in a representative environment; basic
  technology elements integrated with reasonably realistic supporting elements.

For software specifically, NASA's TRL definitions describe TRL 5 as:
*end-to-end software elements implemented and interfaced with existing
systems/simulations conforming to the target environment; the end-to-end
software system tested in a relevant environment, meeting predicted
performance.* TRL 4's software wording stops at the **laboratory** (development)
environment. These are NASA's published levels, quoted for reference only --
this document does not claim TRL 5 anywhere; see section 7 for the one
binding TRL statement this package makes.

Sources (verify against the primary document before quoting in any adoption
deliverable):
- NASA, "Technology Readiness Level Definitions"
  (`458490main_trl_definitions.pdf`), Hardware / Software / Exit-Criteria table.
- NASA Systems Engineering Handbook, NASA/SP-2016-6105 Rev 2, Appendix G
  (Technology Assessment / TRL definitions and the "relevant environment"
  definition).

The single load-bearing word is **environment**. TRL 4 is laboratory; TRL 5 is
relevant. For a GNC flight-software element the relevant environment is not a
faster desktop simulation - it is the **real-time flight-representative
processor**, executing the control path under its actual timing, memory, and
fault constraints, with the plant supplied by a hardware/processor-in-the-loop
rig. That is exactly the WP9 track and exactly why fidelity upgrades
(WP12 L0-L5) do not cross the gap: a higher-fidelity plant on a workstation is
a richer laboratory, not a relevant environment.

### 2.2 Where ADSC actually is

The GNC element (tracking controller + translation EKF / MEKF + sensor models
+ WP11 clearing-abort guidance) runs closed loop on **estimated** states in a
laboratory/simulation environment, with filter consistency verified
(NEES/NIS) rather than assumed. That is the element-level TRL 4 definition
(see [limitations.md](limitations.md) "TRL statement"). The control loop
already runs at a nominal **100 Hz** step (`control_dt`, a 0.01 s constant
in `include/adsc/mission.hpp`; `./build/adsc_sim` prints
"Control step dt" - console-only, not a committed number). Running at 100 Hz in
a desktop sim is a design choice; **meeting** a 100 Hz deadline on a
representative processor is the thing that has never been demonstrated.

### 2.3 Scope caveat (carried, binding)

TRL 4 applies to the **GNC software element, not the system**. System-level TRL
is undefined and not claimed anywhere in this package. Consequently, even a
fully successful future WP9 would support at most a TRL 5 assessment **of the
element**, by an independent assessor - never a system-level maturity claim and
never "flight-ready." This plan asserts none of those.

---

## 3. Pre-registered WP9 completion criteria (frozen)

These are the criteria a future WP9 PIL campaign must meet for its evidence to
support a TRL 5 assessment **of the GNC element** by an assessor. Stating them
here is not meeting them. Each is frozen per section 1.

### 3.1 Representative processor class

The target must be a member of the flight-representative real-time processor
class, meaning all of:

- **Core**: ARM Cortex-M7 or Cortex-R5(F), or a rad-hard-lineage equivalent
  (e.g. SPARC LEON3/4, or a rad-hard Cortex-M) - "or similar representative
  flight-processor class."
- **Hardware FPU**: present. If single-precision only, the double-to-single
  impact on the EKF/guidance math is measured and reported, not waved away.
- **Deterministic memory**: control path fits in tightly-coupled / on-chip SRAM,
  or caches are locked or disabled for the control path, so cache misses do not
  inject timing jitter.
- **Real-time execution**: an RTOS or a bare-metal fixed-rate scheduler capable
  of a hard 100 Hz tick.

The exact part and its clock frequency are pre-registered by R15 addendum at
procurement. Dev-board silicon is acceptable for a first campaign provided its
class membership and its representativeness limits (section 4) are stated;
flight-grade parts are a later, more expensive campaign, not a WP9 entry gate.

### 3.2 Control deadline, margin, and soak (frozen thresholds)

- **Period**: 10 ms (the committed 100 Hz `control_dt`; sourced in 2.2).
- **Deadline**: every control cycle completes within its period. The measured
  worst-case control-path execution time (WCET-hwm, section 3.3) must satisfy
  `WCET-hwm <= 0.75 x 10 ms = 7.5 ms`, i.e. a pre-registered timing margin of
  **>= 25%** of the period held at the worst case.
- **Soak**: zero missed deadlines over a continuous run of **>= 1 hour**
  (>= 1 h x 3600 s/h x 100 cycles/s = >= 360,000 control cycles), with a
  **>= 24 h** endurance run as a pre-registered stretch goal. A single missed
  deadline in the soak is a fail, not a rounding note.

Explicit non-claim: "deadline met with >= 25% margin over the soak" is a
**measured** statement about a specific board, workload, and duration. It is not
a hard real-time guarantee and not a formal WCET bound (see 3.3). No text in
this package will upgrade it to one.

### 3.3 WCET measurement method (pre-registered before hardware)

The method is fixed now so the number cannot be reverse-engineered from a
desired result:

1. **Instrumented high-water-mark**: a free-running cycle counter
   (DWT CYCCNT on Cortex-M, the PMU cycle counter on Cortex-R) latches the
   maximum cycles consumed by the control path per cycle across the whole soak.
   Report as cycles and as time at the pre-registered clock.
2. **Worst-case-path forcing**: the soak deliberately drives the deepest
   control path, not the nominal one, so the high-water-mark reflects the true
   worst branch. Concretely: force the 3-stage clearing-abort escalation to its
   deepest branch (two-impulse retreat hop; see [gnc.md](gnc.md) guidance
   mechanics), coincide all measurement updates (range+LOS, star-tracker,
   vision) in the same cycle, and drive the actuator into saturation. A
   worst-case-search harness (fixed seeds, R6-style) selects the inputs.
3. **Structural cross-check**: a static argument that the measured maximum is
   the structural maximum - no recursion, no unbounded or data-dependent loop
   counts on the control path, no dynamic allocation (WP9a establishes this,
   section 5). The measured high-water-mark and the structural argument must
   agree in shape.

Honesty ceiling: this yields a **measured** WCET high-water-mark with a
structural sanity check, labeled WCET-hwm throughout. It is deliberately **not**
a formal / static-analysis WCET bound (no aiT-class tool, no exhaustive path
proof is in WP9 scope). The distinction is stated wherever the number appears.

### 3.4 Fault-injection cases (frozen set)

Each case has a pre-defined pass condition; "the board did not obviously crash"
is not a pass. The GNC element must, in every case, either continue meeting the
deadline or transition the mode machine (SAFE / HOLD / APPROACH / SYNC / ABORT)
correctly, within a stated bound, with no unbounded execution time:

- **F-DROP**: sensor measurement dropout mid-approach -> guidance holds or
  aborts per its L4 dropout logic; deadline still met.
- **F-TMR**: single bit-flip injected into the TMR fuel store -> the non-atomic
  TMR voter (D-rule TMR) recovers the value; no deadline impact.
- **F-NAN**: NaN/Inf injected into an estimator input -> the control path
  detects/saturates and does not spend unbounded time or fault the processor.
- **F-SAT**: actuator command saturation / stuck-at command -> guidance and the
  contact-speed gate behave per model; abort feasibility preserved.
- **F-OVR**: a deliberately induced single-cycle scheduler overrun -> FDIR
  detects the overrun and commands SAFE; the event is logged, not silently
  absorbed.
- **F-WDT**: an induced control-path hang -> the hardware watchdog resets the
  processor to SAFE within a pre-registered bound.

### 3.5 HIL / PIL data-path (pre-registered topology)

- The **GNC element** (controller + estimator + guidance + mode machine) runs
  on the target board as the flight-representative code.
- The **plant** (Clohessy-Wiltshire / rigid-body dynamics + sensor models - the
  same L0/L2 simulation code, reused not reimplemented) runs on a host acting as
  truth.
- They exchange sensor measurements (host -> board) and actuator commands
  (board -> host) once per control cycle over a defined link
  (UART / SPI / Ethernet), with a pre-registered link-latency budget that fits
  inside the 10 ms period alongside WCET-hwm.
- Loop mode is pre-registered as PIL (processor executes flight code;
  deterministic message-passing to a host plant) with a documented statement of
  whether real I/O timing (true HIL) is or is not in scope for the first
  campaign.

Representativeness limits, stated up front: no real sensors, no flight harness,
no radiation/thermal-vacuum/EMI environment, host-supplied plant. This is
PIL/HIL-lite. It is the relevant *processing* environment, not the relevant
*mission* environment - which is why the ceiling is a TRL 5 element assessment,
not TRL 6+.

### 3.6 Reporting discipline

All of the above print to stdout / the CI log and are referenced GAP-style
(the same discipline as `tests/test_wp9_flightlike.cpp` today, section 5.1 --
a future WP9 PIL campaign registers its own ctest name once hardware exists).
None is committed to `generated/`: soak timing,
WCET-hwm, and fault-recovery latencies are non-deterministic across boards,
clocks, and even runs, and committing them would break the byte-identity gate
(the very gate that makes the rest of the package reproducible). The WP9 result
lives in a human-written, R15-tracked summary that quotes the console figures as
prose-pinned gaps, exactly as WP1/WP2 console-only numbers are handled today.

---

## 4. Candidate hardware / RTOS options (rough cost)

Rough external order-of-magnitude estimates (USD, mid-2020s list prices),
sourced-or-PLACEHOLDER (D10), for scoping only. Not project results, not folded
into the FoM (D12), verify with vendor quotes before any procurement. A "tier"
is a representativeness/cost trade, not a recommendation.

| Tier | Example board (class) | RTOS options | Rough board cost | Representativeness |
|---|---|---|---:|---|
| Bring-up | STM32H7 Nucleo / Discovery (Cortex-M7, ~480 MHz, FPU) | bare-metal fixed-rate; FreeRTOS (free); Zephyr (free) | ~USD 25-100 | Low: real M7 timing/FPU/TCM, but COTS silicon, no lockstep |
| Safety-oriented | TI Hercules TMS570 LaunchPad (Cortex-R5F lockstep); NXP i.MX RT (M7) | FreeRTOS; Zephyr; SAFERTOS (paid) | ~USD 100-500 | Mid: lockstep / safety features, still COTS |
| Flight-lineage | Cobham Gaisler LEON GR712RC / GR740 eval; rad-hard Cortex-M eval | RTEMS (free, space heritage); VxWorks (paid); PikeOS/DEOS (certifiable, paid) | ~USD 5k-50k+ boards | High: flight processor lineage; boards and certifiable RTOS licenses dominate cost |

RTOS notes: bare-metal fixed-rate is the most deterministic and the least
featureful; FreeRTOS (MIT) and Zephyr (Apache-2) are free and adequate for a
first deadline/soak campaign; RTEMS carries real space heritage; VxWorks and the
certifiable kernels add large (order USD 10k-100k+) license and support costs and
belong to a later, flight-track campaign, not to WP9 entry.

Order-of-magnitude first-campaign envelope: a credible bring-up-tier PIL rig is
~USD 1k-5k of hardware plus a few engineer-weeks. That small, knowable spend is
precisely what WP9a (section 5) de-risks before it is committed.

---

## 5. WP9a - software-only groundwork (does / does not)

WP9a is the part of the flight-software migration path (evidence pack section 8)
that needs **no hardware** and can run in CI on the existing workstation
toolchains. It extends the migration annex from a bullet list into checkable
probes whose output prints to stdout
(`regenerate: ctest -R wp9_flightlike`).

Concrete status: `tests/test_wp9_flightlike.cpp` is the first such probe. It
drives the WP4 combined control+estimation step for a fixed step count, twice
from an identical fixed-seed scenario, and checks allocation-count
determinism against a generous ceiling, bit-identical final state across the
two runs, and prints per-step wall time tagged "CI hardware, NOT
flight-representative - baseline only" against a loose sanity bound. This
covers the allocation-behavior and same-toolchain-determinism bullets below
on this machine's own compiler; it does NOT yet cover the cross-compiled/
emulated-target determinism check or the portability-blocker inventory also
described below -- those remain open WP9a increments, not yet done.

### 5.1 What WP9a DOES establish

- **Allocation behavior.** The control path is - or is made - free of dynamic
  heap allocation and of exceptions on the hot path: fixed-size Eigen types,
  bounded stack, `new`/`delete` counted via an overridden allocator in a probe
  build, an `-fno-exceptions` control-path build that still links, and
  compiler stack-usage output (`-fstack-usage`) bounding the worst frame. This
  is the precondition for the section 3.3 structural WCET argument.
- **Determinism.** The fixed-seed `mt19937` + explicit Box-Muller path is
  already bit-stable across three platforms; WP9a extends that to a
  cross-compiled build of the control path (host-hosted or emulator, e.g. QEMU)
  and checks bit-identical control outputs, flagging any floating-point
  divergence introduced by the cross-toolchain or optimization level.
- **Portability blockers.** A portability report enumerating exactly what
  prevents building the control path for a freestanding / cross target: any
  host-only headers, `iostream`/filesystem use on the control path,
  exceptions/RTTI dependence, Eigen features that require dynamic allocation,
  double-vs-single FP assumptions, endianness, and libc dependencies. Blockers
  are listed as concrete file/line items, PLACEHOLDER where a fix is not yet
  designed.
- **HAL boundary.** A defined interface between the GNC core and device drivers
  (the annex's hardware abstraction layer), so the same control path that runs
  in the desktop sim is the one that would later cross to the board unchanged.

### 5.2 What WP9a DOES NOT establish (explicit)

- **No TRL change.** WP9a does not raise the TRL. Like WP10-WP15, it prepares
  and widens; it does not mature. TRL stays 4, element-scoped.
- **No real-time behavior.** With no processor in the loop there is no timing,
  no WCET number, no deadline result, and no margin claim. WP9a says nothing
  about whether 100 Hz is met on any hardware.
- **No PIL.** WP9a is not processor-in-the-loop. PIL is WP9 proper - reserved.
- **No flight readiness / environmental qualification.** No radiation, thermal,
  vacuum, EMI, or vibration claim; none is in scope at any point in WP9.
- **No real-time guarantee, ever.** Even a completed WP9 yields a measured
  high-water-mark with margin (3.3), not a formal guarantee.

---

## 6. Kill-criteria (element NOT ready to attempt PIL)

The value of WP9a is cheap falsification: results, obtainable on the workstation
with no hardware spend, that show the GNC element is not ready to even attempt
PIL - so the money in section 4 should be withheld until the software is fixed.
Any one firing halts WP9 procurement and reopens design. Frozen per section 1.

- **K1 - allocation cannot be bounded.** The control path cannot be made free of
  dynamic allocation without algorithmic redesign (e.g. an Eigen dynamic-size
  allocation in the estimator/guidance hot loop that cannot be bounded to a
  fixed size). No allocation-free path -> no credible WCET -> do not attempt PIL.
- **K2 - un-removable host dependency.** Cross-compilation to a representative
  freestanding target is blocked by a dependency the control path structurally
  requires - exceptions/RTTI in a core algorithm, or host libc / filesystem /
  iostream on the control path - that cannot be removed without a redesign.
- **K3 - determinism fails on desktop.** Control outputs diverge across
  toolchains or optimization levels beyond a pre-registered epsilon. If the math
  is that numerically fragile before any radiation or hardware FP quirk, it is a
  redesign flag - and it would also mean the current cross-platform byte-identity
  is toolchain luck rather than a property of the algorithm.
- **K4 - unbounded worst-case path.** The structural analysis finds an unbounded
  or data-dependent loop count on the control path (e.g. an iterative solver
  with no iteration cap), so no WCET-hwm can be argued even before hardware.
- **K5 - budget blown on paper.** A generous static instruction-/cycle-count
  budget of the worst path, evaluated at the pre-registered core's clock, already
  exceeds the 7.5 ms margin target (3.2) by a factor no reasonable optimization
  closes. Attempting PIL would only confirm a known failure - fix the algorithm
  or lower the rate first.

A kill is a first-class deliverable, not a project failure: it converts an
expensive hardware disappointment into a cheap, documented software finding, and
it is reported with the same weight as a pass (negative results are first-class,
spec section 10).

---

## 7. TRL statement (binding, spec section 9) and cross-references

**TRL 4 - for the GNC software element only**, laboratory/simulation
environment; system-level TRL undefined and not claimed. Reaching TRL 5 for the
element requires real-time processor-in-the-loop execution on representative
hardware - the WP9 track, **RESERVED, not started**. This plan pre-registers
WP9's criteria and scopes WP9a's software-only groundwork; it performs neither
the PIL work nor any maturity change. Meeting the section 3 criteria is a future,
hardware-gated event whose ceiling is a TRL 5 assessment of the element by an
independent assessor. No real-time guarantee, no flight-readiness claim, and no
TRL 5 claim is made anywhere in this package.

Cross-references:
- Migration path this plan operationalizes: evidence pack section 8 (annex).
- Binding TRL rationale and "why nothing raises it": [roadmap.md](roadmap.md),
  [limitations.md](limitations.md), spec section 9.
- Worst-case control path (WCET forcing target) and guidance mechanics:
  [gnc.md](gnc.md).
- One-code-path fidelity ladder (why higher fidelity is a wider laboratory, not
  a relevant environment): [technical_architecture.md](technical_architecture.md).
