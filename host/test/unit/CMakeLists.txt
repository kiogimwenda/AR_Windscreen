# Host unit tests — see docs/BUILD_GUIDE.md Part 13.1.
#
# This is a STANDALONE CMake project, deliberately independent of host/CMakeLists.txt. None of the
# tests here touch TensorRT, CUDA, SDL2, or a live camera/LiDAR/serial device — they exercise pure
# logic against synthetic data — so this file links only Eigen3, PCL, GTest and (for the nav tests)
# OSRM. That is the only reason CI (Part 13.5) can run this suite on an ordinary GitHub-hosted
# runner, which has no GPU.
#
# If a unit test ever needs to #include something from the TensorRT/SDL2 side of the tree, that is a
# sign the class under test needs splitting — not a reason to widen this file's dependencies.
#
# TODO(Phase 1): add the first target (test_crc16) and enable_testing()/gtest_discover_tests.
cmake_minimum_required(VERSION 3.25)
project(ar_windscreen_unit_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()
