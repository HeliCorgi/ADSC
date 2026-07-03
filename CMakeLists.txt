cmake_minimum_required(VERSION 3.16)
project(ADSC VERSION 4.0 LANGUAGES CXX)

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
else()
    message(WARNING "python3 not found; WP7a viz and WP8 compliance tests not registered")
endif()
