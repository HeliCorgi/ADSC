cmake_minimum_required(VERSION 3.16)
project(ADSC_v1.21 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(adsc_v121
    src/ADSC_Core.cpp
    src/main.cpp
)

target_link_libraries(adsc_v121 PRIVATE Eigen3::Eigen)
target_include_directories(adsc_v121 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_compile_options(adsc_v121 PRIVATE
    -Wall -Wextra -Wpedantic
    $<$<CONFIG:Release>:-O3 -march=native>
    $<$<CONFIG:Debug>:-g>
)
