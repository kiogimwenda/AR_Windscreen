# Host unit tests — see docs/BUILD_GUIDE.md Part 13.1.
#
# This is a STANDALONE CMake project, deliberately independent of host/CMakeLists.txt. None of the
# tests here touch TensorRT, CUDA, SDL2, or a live camera/LiDAR/serial device — they exercise pure
# logic against synthetic data — so this file links only GTest and, as later phases land, Eigen3,
# PCL and OSRM. That is the only reason CI (Part 13.5) can run this suite on an ordinary
# GitHub-hosted runner, which has no GPU.
#
# If a unit test ever needs to #include something from the TensorRT/SDL2 side of the tree, that is a
# sign the class under test needs splitting — not a reason to widen this file's dependencies.
cmake_minimum_required(VERSION 3.25)
project(ar_windscreen_unit_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

# Every test target gets the host include tree and GTest's own main().
# add_unit_test(<name> <sources...>) keeps the per-phase additions below to one line each.
function(add_unit_test name)
    add_executable(${name} ${ARGN})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
    target_link_libraries(${name} PRIVATE GTest::gtest GTest::gtest_main)
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    gtest_discover_tests(${name})
endfunction()

# --- Phase 1 ----------------------------------------------------------------------------------
add_unit_test(test_crc16 test_crc16.cpp)

# --- Later phases (targets added as each phase implements its subsystem) ------------------------
# Phase 7:  test_ekf.cpp, test_multi_object_tracker.cpp          (adds Eigen3)
# Phase 8:  test_map_matcher.cpp                                  (adds OSRM)
# Phase 9:  test_road_surface_projector.cpp                       (adds Eigen3)
# Phase 10: test_decision_arbiter.cpp
