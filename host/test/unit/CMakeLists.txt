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

# --- Phase 3 ----------------------------------------------------------------------------------
# Threads::Threads adds -pthread portably; RingBuffer, EventLog and SystemManager tests all run
# several threads.
find_package(Threads REQUIRED)

# The bus message types are generated from the .fbs schemas by flatc — the same module the main
# host build uses, so both builds compile against identical generated code. flatc and the
# FlatBuffers headers are CPU-only, so this keeps the suite GPU-free (Part 13.1).
include(${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/FlatBufferSchemas.cmake)
ar_add_schema_library(ar_schemas ${CMAKE_CURRENT_SOURCE_DIR}/../../include/ar_drive_assist/common/schemas)

set(HOST_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

add_unit_test(test_ring_buffer test_ring_buffer.cpp)
target_link_libraries(test_ring_buffer PRIVATE Threads::Threads)

add_unit_test(test_event_log test_event_log.cpp ${HOST_SRC}/system/EventLog.cpp)
target_link_libraries(test_event_log PRIVATE ar_schemas Threads::Threads)

# SystemManager reads the YAML config, so this is the one Phase 3 test that needs yaml-cpp.
# HOST_SOURCE_DIR lets it load the real host/config/*.yaml files, so a config file edited into an
# invalid state fails CI rather than failing at startup in the car.
find_package(yaml-cpp REQUIRED)
add_unit_test(test_system_manager test_system_manager.cpp
              ${HOST_SRC}/system/SystemManager.cpp ${HOST_SRC}/system/EventLog.cpp)
target_link_libraries(test_system_manager PRIVATE ar_schemas yaml-cpp::yaml-cpp Threads::Threads)
target_compile_definitions(test_system_manager PRIVATE
                           HOST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../..")

# --- Phase 5 ----------------------------------------------------------------------------------
# Model-output decoding only. The GPU side (TrtEngine, pre-processing) is tested in
# host/test/integration/ under the `gpu` label, since CI has no GPU.
add_unit_test(test_postprocess test_postprocess.cpp ${HOST_SRC}/inference/Postprocess.cpp)

# --- Later phases (targets added as each phase implements its subsystem) ------------------------
# Phase 7:  test_ekf.cpp, test_multi_object_tracker.cpp          (adds Eigen3)
# Phase 8:  test_map_matcher.cpp                                  (adds OSRM)
# Phase 9:  test_road_surface_projector.cpp                       (adds Eigen3)
# Phase 10: test_decision_arbiter.cpp
