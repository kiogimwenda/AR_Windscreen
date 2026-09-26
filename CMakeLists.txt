# Top-level CMake entry point — see docs/BUILD_GUIDE.md Part 5.2.
#
# This file deliberately does almost nothing itself: it fixes the minimum CMake version and the
# project identity, then hands off to host/. The firmware under firmware/sensor_actuator_hub/ is
# NOT built from here — it is a PlatformIO project with its own toolchain (see Part 4.7), and
# keeping the two build systems separate mirrors the hard host/hub boundary from Part 0.
cmake_minimum_required(VERSION 3.25)

project(ar_windscreen
        VERSION 0.1.0
        DESCRIPTION "AR-Assisted Semi-Autonomous Driving System"
        LANGUAGES CXX CUDA)

# Export compile_commands.json so clangd / IDEs resolve includes without a separate config.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Needed at the top level too: ctest only finds tests registered below a directory that enables them.
enable_testing()
add_subdirectory(host)
