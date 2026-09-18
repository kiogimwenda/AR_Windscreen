# Host application build — see docs/BUILD_GUIDE.md Part 5.2.
#
# ---------------------------------------------------------------------------------------------
# How to read this file, if CMake's model is unfamiliar:
#
# CMake is a build-system *generator*. Nothing here compiles anything. Running `cmake -S . -B build`
# executes this script once (the CONFIGURE step), during which it locates dependencies and records
# what needs building, then writes a Ninja/Make build graph into build/. Running `cmake --build
# build` is a separate step that executes that graph. This two-step split is why "cmake configures
# without error" is a meaningful Phase 0 exit criterion even though not one line of this project's
# logic exists yet: configuring proves every dependency was FOUND, independently of whether the
# code that uses them compiles.
#
# find_package(X REQUIRED) searches for a package and hard-fails configure if it is missing. On
# success it defines IMPORTED TARGETS (e.g. Eigen3::Eigen) — objects that carry their own include
# directories, compile flags and link libraries. Linking such a target with target_link_libraries
# pulls all of that in transitively, which is why this file never manually adds -I paths.
# ---------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.25)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies (Part 2.3-2.8 installs all of these) ----------------------------------------
find_package(OpenCV 4.10 REQUIRED)
# PCL is requested BY COMPONENT, not as a whole — a deviation from the guide's bare
# `find_package(PCL 1.13 REQUIRED)`, forced by Debian 13 packaging and logged in docs/decisions.md.
#
# A whole-package PCL request (and, specifically, its `io` component) pulls in VTK, and Debian's
# VTK 9.3 CMake package is broken: it declares link dependencies on targets — JsonCpp::JsonCpp,
# MPI::MPI_C, and others behind them — without ever calling find_dependency() to define them, so
# configuring fails outright. Chasing each missing target would mean linking OpenMPI and friends
# into a real-time driving system purely to satisfy a packaging bug.
#
# Naming components avoids VTK entirely and is the right thing independently. Part 8.1 needs
# VoxelGrid (filters), SACSegmentation (segmentation + sample_consensus) and
# EuclideanClusterExtraction (kdtree + search); this project renders through OpenCV and SDL2 and
# never uses pcl::visualization. `io` is deliberately excluded — if Phase 6 wants debug point-cloud
# dumps, write the PCD directly rather than re-admitting VTK through the back door.
find_package(PCL 1.13 REQUIRED COMPONENTS common filters segmentation kdtree search sample_consensus)
find_package(Eigen3 REQUIRED)
find_package(glm REQUIRED)
find_package(spdlog REQUIRED)
find_package(SDL2 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Flatbuffers REQUIRED)
find_package(GTest REQUIRED)

# TensorRT ships no CMake config package, so this project supplies its own find-module.
# NOTE: the guide's Part 5.2 snippet writes ${CMAKE_SOURCE_DIR}/../cmake/FindTensorRT.cmake, which
# assumed host/ was the top-level source directory. It is not — the repo root is (see the top-level
# CMakeLists.txt) — so ${CMAKE_SOURCE_DIR}/.. would point outside the repository entirely. Using
# CMAKE_CURRENT_SOURCE_DIR keeps the path correct and correct-if-relocated. Logged in
# docs/decisions.md.
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/FindTensorRT.cmake)

# --- Target ------------------------------------------------------------------------------------
# GLOB_RECURSE is what the guide specifies. It has a known downside: CMake only re-globs when it
# reconfigures, so a newly added .cpp is not picked up until something triggers a reconfigure.
# CONFIGURE_DEPENDS makes the build system re-check the glob on every build, which costs a stat
# sweep and removes the surprise.
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)

add_executable(ar_drive_assist ${SOURCES})
target_include_directories(ar_drive_assist PRIVATE include)
target_link_libraries(ar_drive_assist PRIVATE
    ${OpenCV_LIBS} ${PCL_LIBRARIES} Eigen3::Eigen glm::glm
    spdlog::spdlog SDL2::SDL2 CUDA::cudart TensorRT::TensorRT
    flatbuffers osrm
)

# The unit suite under test/unit/ is intentionally NOT built from here — see test/CMakeLists.txt.
add_subdirectory(test)
