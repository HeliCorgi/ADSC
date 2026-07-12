cmake_minimum_required(VERSION 3.16)
# VERSION tracks the git-tag scheme in docs/release_engineering.md
# (v0.<phase-number>-<short-descriptor>, e.g. v0.15-proposal-package): the
# CMake-internal version now bumps in lockstep with the phase tags instead of
# carrying its own separate number. Reconciled 2026-07-12 (was a stale 4.0
# flagged as a known inconsistency in release_engineering.md).
project(ADSC VERSION 0.15.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# R3: zero-warning policy. OFF by default so a future compiler's new diagnostics
# do not break local/downstream builds; CI turns it ON (-DADSC_WERROR=ON).
option(ADSC_WERROR "Treat compiler warnings as errors (R3 zero-warning policy)" OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Shared warning / optimization flags applied to every first-party target so the
# R3 zero-warning policy is enforced uniformly. Eigen headers arrive via the
# imported Eigen3::Eigen target and are treated as SYSTEM includes, so they do
# not trip -Werror.
add_library(adsc_flags INTERFACE)
target_compile_options(adsc_flags INTERFACE
    -Wall -Wextra -Wpedantic
    $<$<CONFIG:Debug>:-g>
    # -O3 only in Release. -march=native is intentionally NOT set: the flight
    # target is a RAD5545 (PowerPC), so tuning for the x86 build host is wrong.
    # Set an explicit target march in the cross-compile toolchain file instead.
    $<$<CONFIG:Release>:-O3>
    $<$<BOOL:${ADSC_WERROR}>:-Werror>
)

# Core GNC library (dynamics, control, TMR store, thermal, relative motion,
# mission logic).
add_library(adsc_core
    src/fuel_store.cpp
    src/dynamics.cpp
    src/controller.cpp
    src/thermal.cpp
    src/relmotion.cpp
    src/decay.cpp
    src/estimator.cpp
    src/mission.cpp
    src/campaign.cpp
    src/cost.cpp
    src/flux.cpp
    src/guidance.cpp
    src/propagation.cpp
)
target_include_directories(adsc_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(adsc_core PUBLIC Eigen3::Eigen)
target_link_libraries(adsc_core PRIVATE adsc_flags)

# Simulation driver.
add_executable(adsc_sim src/main.cpp)
target_link_libraries(adsc_sim PRIVATE adsc_core adsc_flags)

# WP5 campaign Monte Carlo driver: regenerates the committed generated/ CSV+MD
# artifacts and prints the campaign summary.
add_executable(adsc_campaign src/main_campaign.cpp)
target_link_libraries(adsc_campaign PRIVATE adsc_core adsc_flags)

# WP6 parametric cost/FoM driver: regenerates the committed generated/wp6_cost_*
# artifacts (amortization curve, FoM under >=2 weightings, tornado sensitivity).
add_executable(adsc_cost src/main_cost.cpp)
target_link_libraries(adsc_cost PRIVATE adsc_core adsc_flags)

# T6 deliverable: small-debris flux / hypervelocity-impact sweep. Regenerates
# generated/t6_flux_sweep.md (and the same table to stdout).
add_executable(flux_sweep src/main_flux.cpp)
target_link_libraries(flux_sweep PRIVATE adsc_core adsc_flags)

# WP3 decay-trade CSV emitter (feeds the WP7a visualization): re-emits the
# existing sail-decay trade as machine-readable generated/wp3_decay_trade.csv.
# No physics/number change -- reuses src/decay.cpp.
add_executable(decay_trade src/main_decay_trade.cpp)
target_link_libraries(decay_trade PRIVATE adsc_core adsc_flags)

# WP13 kit-class trade CSV emitter: regenerates generated/wp13_kit_trade.{csv,md}
# (sail/EDT/hybrid per-class A/B/C trade) and generated/wp13_classC.{csv,md}
# (controlled-reentry mission-class comparison). Reuses src/decay.cpp's
# existing sail model plus the new EDT-v1 physics core; no other physics
# change.
add_executable(kit_trade src/main_kit_trade.cpp)
target_link_libraries(kit_trade PRIVATE adsc_core adsc_flags)

# WP7 reference-metrics emitter: re-computes the pinned WP1/F1/WP2/WP3/WP4
# demo numbers as generated/reference_metrics.csv so the evidence pack quotes
# committed data only (R6). Reuses existing code paths; no physics change.
add_executable(sim_metrics src/main_metrics.cpp)
target_link_libraries(sim_metrics PRIVATE adsc_core adsc_flags)

# WP12 fidelity-ladder driver: regenerates generated/wp12_ladder.{csv,md} --
# per-level (L0/L1/L2) re-verification of the committed WP5 campaign's abort
# events, the forensic-14 per-level clearance table, and the margin-decay
# measurement (F2). Reuses run_one_mission's WP12 audit hook + propagation.cpp;
# does not touch a single wp5_campaign_* byte.
add_executable(adsc_ladder src/main_ladder.cpp)
target_link_libraries(adsc_ladder PRIVATE adsc_core adsc_flags)

# Tests.
enable_testing()

add_executable(test_fuel_store tests/test_fuel_store.cpp)
target_link_libraries(test_fuel_store PRIVATE adsc_core adsc_flags)
add_test(NAME fuel_store COMMAND test_fuel_store)

add_executable(test_relmotion tests/test_relmotion.cpp)
target_link_libraries(test_relmotion PRIVATE adsc_core adsc_flags)
add_test(NAME relmotion COMMAND test_relmotion)

add_executable(test_sync tests/test_sync.cpp)
target_link_libraries(test_sync PRIVATE adsc_core adsc_flags)
add_test(NAME sync COMMAND test_sync)

add_executable(test_decay tests/test_decay.cpp)
target_link_libraries(test_decay PRIVATE adsc_core adsc_flags)
add_test(NAME decay COMMAND test_decay)

add_executable(test_mission tests/test_mission.cpp)
target_link_libraries(test_mission PRIVATE adsc_core adsc_flags)
add_test(NAME mission COMMAND test_mission)

add_executable(test_estimator tests/test_estimator.cpp)
target_link_libraries(test_estimator PRIVATE adsc_core adsc_flags)
add_test(NAME estimator COMMAND test_estimator)

add_executable(test_campaign tests/test_campaign.cpp)
target_link_libraries(test_campaign PRIVATE adsc_core adsc_flags)
add_test(NAME campaign COMMAND test_campaign)

# WP11 forensic-14 regression: pins the 14 keep-out-violating runs classified
# in generated/wp10_violation_forensics.csv (WP10c) as a permanent regression
# that the WP11 clearing-abort law clears (D13, R15).
add_executable(test_forensic14 tests/test_forensic14.cpp)
target_link_libraries(test_forensic14 PRIVATE adsc_core adsc_flags)
add_test(NAME forensic14 COMMAND test_forensic14)

# WP11 closed-loop translation guidance: bounded_coast_min_range
# cross-validation, the clearing-abort escalation ladder, and the
# GuidedApproach demo (D13/D5).
add_executable(test_guidance tests/test_guidance.cpp)
target_link_libraries(test_guidance PRIVATE adsc_core adsc_flags)
add_test(NAME guidance COMMAND test_guidance)

# WP12 fidelity ladder: the MANDATORY cross-validation (L1/J2-off vs the CW
# closed form, L2/drag-off vs L1, a bounded J2-on sanity check) plus L4/L5
# demo assertions (D13/R14 extension of the WP1/WP11 regression discipline).
add_executable(test_ladder tests/test_ladder.cpp)
target_link_libraries(test_ladder PRIVATE adsc_core adsc_flags)
add_test(NAME ladder COMMAND test_ladder)

add_executable(test_cost tests/test_cost.cpp)
target_link_libraries(test_cost PRIVATE adsc_core adsc_flags)
add_test(NAME cost COMMAND test_cost)

add_executable(test_flux tests/test_flux.cpp)
target_link_libraries(test_flux PRIVATE adsc_core adsc_flags)
add_test(NAME flux COMMAND test_flux)

# WP7a Visualization Pack test: Python 3 stdlib-only generator (spec v4.2 R9
# tooling exception). Regenerates the SVGs from the committed CSVs and checks
# well-formedness, caption provenance, no timestamps/stub markers, and byte
# reproducibility against the committed generated/viz.
find_program(PYTHON3_EXECUTABLE NAMES python3 python)
if(PYTHON3_EXECUTABLE)
    add_test(NAME viz COMMAND ${PYTHON3_EXECUTABLE}
             ${CMAKE_CURRENT_SOURCE_DIR}/tools/viz/test_viz.py
             ${CMAKE_CURRENT_SOURCE_DIR})
    # WP8 Compliance Matrix Generator / Regulatory Precheck test (Python 3
    # stdlib only, spec v4.2 R9). Not legal advice; research-grade precheck.
    add_test(NAME compliance COMMAND ${PYTHON3_EXECUTABLE}
             ${CMAKE_CURRENT_SOURCE_DIR}/tools/compliance/test_compliance.py
             ${CMAKE_CURRENT_SOURCE_DIR})
    # WP7 evidence-pack claim audit: forbidden overclaims, required honesty
    # sections, number-transcription cross-check vs the committed CSVs,
    # determinism, committed == regenerated.
    add_test(NAME evidence COMMAND ${PYTHON3_EXECUTABLE}
             ${CMAKE_CURRENT_SOURCE_DIR}/tools/evidence/test_evidence.py
             ${CMAKE_CURRENT_SOURCE_DIR})
    # WP10c keep-out violation forensics audit (Python 3 stdlib only, R9):
    # the generator's internal 1000-run replay cross-validation gate, plus
    # determinism, committed == regenerated, taxonomy, and claim discipline.
    add_test(NAME forensics COMMAND ${PYTHON3_EXECUTABLE}
             ${CMAKE_CURRENT_SOURCE_DIR}/tools/forensics/test_forensics.py
             ${CMAKE_CURRENT_SOURCE_DIR})
    # WP14 prioritization table: a pure derived/joined view over
    # wp6_cost_summary.csv + wp13_kit_trade.csv + wp5_campaign_summary.csv
    # (Python 3 stdlib only, R9). Checks regeneration, hygiene/claims
    # discipline, D10/R6 currency-range discipline, content, and
    # determinism/committed-bytes reproducibility.
    add_test(NAME prioritization COMMAND ${PYTHON3_EXECUTABLE}
             ${CMAKE_CURRENT_SOURCE_DIR}/tools/prioritization/test_prioritization.py
             ${CMAKE_CURRENT_SOURCE_DIR})
    # WP14 README marker-block filler (review C4): regenerates the
    # WP5-NUMBERS/WP6-NUMBERS marker blocks in README.md from the committed
    # wp5_campaign_summary.csv/wp6_cost_summary.csv and checks --check mode
    # passes against the committed file, catches a corrupted number, and is
    # deterministic/idempotent.
    add_test(NAME readme_numbers COMMAND ${PYTHON3_EXECUTABLE}
             ${CMAKE_CURRENT_SOURCE_DIR}/tools/readme/test_readme_numbers.py
             ${CMAKE_CURRENT_SOURCE_DIR})
    # WP15 docs/ marker-block filler (R16 mechanism): regenerates every
    # DOCS-*-START/END marker block across docs/*.md from the committed CSVs
    # they cite (the docs/ sibling of readme_numbers above) and checks
    # --check mode passes against the committed files, catches a corrupted
    # number, and is deterministic/idempotent.
    add_test(NAME docs_numbers COMMAND ${PYTHON3_EXECUTABLE}
             ${CMAKE_CURRENT_SOURCE_DIR}/tools/docs/test_docs_numbers.py
             ${CMAKE_CURRENT_SOURCE_DIR})
else()
    message(WARNING "python3 not found; WP7a viz, WP8 compliance, WP7 evidence, WP10c forensics, WP14 prioritization and WP15 docs_numbers tests not registered")
endif()
