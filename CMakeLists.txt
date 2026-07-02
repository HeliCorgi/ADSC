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
)
target_include_directories(adsc_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(adsc_core PUBLIC Eigen3::Eigen)
target_link_libraries(adsc_core PRIVATE adsc_flags)

# Simulation driver.
add_executable(adsc_sim src/main.cpp)
target_link_libraries(adsc_sim PRIVATE adsc_core adsc_flags)

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
