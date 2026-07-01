cmake_minimum_required(VERSION 3.16)
project(ADSC VERSION 2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Core GNC library (dynamics, control, TMR store, thermal, mission logic).
add_library(adsc_core
    src/fuel_store.cpp
    src/dynamics.cpp
    src/controller.cpp
    src/thermal.cpp
    src/mission.cpp
)
target_include_directories(adsc_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(adsc_core PUBLIC Eigen3::Eigen)

target_compile_options(adsc_core PRIVATE
    -Wall -Wextra -Wpedantic
    $<$<CONFIG:Debug>:-g>
    # NOTE: -O3 only. -march=native is intentionally NOT set: the flight target
    # is a RAD5545 (PowerPC), so tuning for the x86 build host is wrong. Set an
    # explicit target march in the cross-compile toolchain file instead.
    $<$<CONFIG:Release>:-O3>
)

# Simulation driver.
add_executable(adsc_sim src/main.cpp)
target_link_libraries(adsc_sim PRIVATE adsc_core)

# Tests.
enable_testing()
add_executable(test_fuel_store tests/test_fuel_store.cpp)
target_link_libraries(test_fuel_store PRIVATE adsc_core)
add_test(NAME fuel_store COMMAND test_fuel_store)
