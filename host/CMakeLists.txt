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
# Phase 11: WindowedSink's OpenGL (GLEW loads the entry points).
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Flatbuffers REQUIRED)
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)   # SystemManager::loadConfig (Phase 3); see docs/decisions.md

# TensorRT ships no CMake config package, so this project supplies its own find-module.
# NOTE: the guide's Part 5.2 snippet writes ${CMAKE_SOURCE_DIR}/../cmake/FindTensorRT.cmake, which
# assumed host/ was the top-level source directory. It is not — the repo root is (see the top-level
# CMakeLists.txt) — so ${CMAKE_SOURCE_DIR}/.. would point outside the repository entirely. Using
# CMAKE_CURRENT_SOURCE_DIR keeps the path correct and correct-if-relocated. Logged in
# docs/decisions.md.
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/FindTensorRT.cmake)

# OSRM is a static library whose dependencies must be named at the final link. See FindOSRM.cmake
# for why the guide's bare `osrm` stops linking as soon as real code exists. Logged in
# docs/decisions.md.
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/FindOSRM.cmake)

# Bus message types generated from include/ar_drive_assist/common/schemas/*.fbs (Part 3.5). The
# same module generates them for the unit-test build, so both compile identical code.
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/FlatBufferSchemas.cmake)
ar_add_schema_library(ar_schemas ${CMAKE_CURRENT_SOURCE_DIR}/include/ar_drive_assist/common/schemas)

# --- Target ------------------------------------------------------------------------------------
# GLOB_RECURSE is what the guide specifies. It has a known downside: CMake only re-globs when it
# reconfigures, so a newly added .cpp is not picked up until something triggers a reconfigure.
# CONFIGURE_DEPENDS makes the build system re-check the glob on every build, which costs a stat
# sweep and removes the surprise.
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)

# Livox SDK2 (Part 2.8, the live Mid-360): optional. Without it the host still builds and replays
# recordings; live LiDAR capture (src/lidar/LivoxCapture.cpp) is left out and main says so.
# Installed system-wide (Part 2.8) or into ~/.local/livox (cmake -DCMAKE_INSTALL_PREFIX=...).
find_path(LIVOX_INCLUDE_DIR livox_lidar_api.h HINTS $ENV{HOME}/.local/livox/include /usr/local/include)
find_library(LIVOX_LIBRARY livox_lidar_sdk_static HINTS $ENV{HOME}/.local/livox/lib /usr/local/lib)
if(LIVOX_INCLUDE_DIR AND LIVOX_LIBRARY)
    message(STATUS "Livox SDK2: ${LIVOX_LIBRARY} (live Mid-360 capture enabled)")
    set(AR_HAVE_LIVOX ON)
else()
    message(STATUS "Livox SDK2 not found: live LiDAR capture left out (replays still work)")
    list(FILTER SOURCES EXCLUDE REGEX ".*/lidar/LivoxCapture\\.cpp$")
endif()

add_executable(ar_drive_assist ${SOURCES})
target_include_directories(ar_drive_assist PRIVATE include)
if(AR_HAVE_LIVOX)
    target_compile_definitions(ar_drive_assist PRIVATE AR_HAVE_LIVOX=1)
    target_include_directories(ar_drive_assist SYSTEM PRIVATE ${LIVOX_INCLUDE_DIR})
    target_link_libraries(ar_drive_assist PRIVATE ${LIVOX_LIBRARY} pthread)
endif()
target_link_libraries(ar_drive_assist PRIVATE
    ${OpenCV_LIBS} ${PCL_LIBRARIES} Eigen3::Eigen glm::glm
    spdlog::spdlog SDL2::SDL2 OpenGL::GL GLEW::GLEW CUDA::cudart TensorRT::TensorRT
    flatbuffers OSRM::osrm
    ar_schemas yaml-cpp::yaml-cpp
)

# --- Tools ------------------------------------------------------------------------------------
# Developer tools live in tools/, not src/, so the GLOB above never picks up their main().
# Phase 5: plays recorded video through the inference engine and draws the results.
add_executable(inference_viewer tools/inference_viewer.cpp src/inference/MlInferenceEngine.cpp
               src/inference/TrtEngine.cpp src/inference/Preprocess.cpp
               src/inference/Postprocess.cpp src/system/EventLog.cpp)
target_include_directories(inference_viewer PRIVATE include)
target_link_libraries(inference_viewer PRIVATE ${OpenCV_LIBS} CUDA::cudart TensorRT::TensorRT
                      ar_schemas)

# Phase 8: replays a recorded GPS track (GPX) through NavigationEngine + MapMatcher and draws the
# result on a map: the "visibly snaps onto the correct road" exit check.
add_executable(nav_replay tools/nav_replay.cpp src/nav/NavigationEngine.cpp src/nav/MapMatcher.cpp
               src/nav/MapMatcherOsrm.cpp src/nav/Route.cpp)
target_include_directories(nav_replay PRIVATE include src/nav)
target_link_libraries(nav_replay PRIVATE OSRM::osrm ar_schemas)

# Phase 11: Part 10.4's benchmark checkpoint, and a visual check of WindowedSink.
add_executable(render_bench tools/render_bench.cpp src/render/WindowedSink.cpp
                            src/render/OverlayScene.cpp)
target_include_directories(render_bench PRIVATE include)
target_link_libraries(render_bench PRIVATE ${OpenCV_LIBS} SDL2::SDL2 OpenGL::GL GLEW::GLEW Eigen3::Eigen)

# Phase 3 (with no hub yet): a software hub on a pseudo-terminal, running the FIRMWARE's own
# SafetyCore and FrameCodec. HubSim.cpp alone sees the firmware's include directory.
# Phase 4's exit check: the sustained frame rate the camera delivers at each candidate size.
add_executable(camera_check tools/camera_check.cpp src/camera/CameraPipeline.cpp
               src/camera/CameraConfig.cpp src/system/EventLog.cpp)
target_include_directories(camera_check PRIVATE include)
target_link_libraries(camera_check PRIVATE ${OpenCV_LIBS} Eigen3::Eigen yaml-cpp::yaml-cpp ar_schemas)

find_package(Threads REQUIRED)
# The simulated hub: a library (it compiles the FIRMWARE's SafetyCore; only HubSim.cpp sees the
# firmware headers) used by the hub_sim tool and replay_inspect.
add_library(hub_sim_lib STATIC tools/hub_sim/HubSim.cpp)
target_include_directories(hub_sim_lib PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/sensor_actuator_hub/include
                           PUBLIC tools/hub_sim)
target_link_libraries(hub_sim_lib PUBLIC Threads::Threads)
add_executable(hub_sim tools/hub_sim/main.cpp)
target_link_libraries(hub_sim PRIVATE hub_sim_lib)

# Item 1 of the pre-hardware list: a recording through the whole chain, frame by frame (Part 13.2).
add_executable(replay_inspect tools/replay_inspect.cpp
               src/camera/CameraPipeline.cpp src/camera/CameraConfig.cpp
               src/inference/MlInferenceEngine.cpp src/inference/TrtEngine.cpp
               src/inference/Preprocess.cpp src/inference/Postprocess.cpp
               src/lidar/LidarReplay.cpp src/lidar/InjectedObstacle.cpp src/lidar/LidarProcessor.cpp
               src/system/Recording.cpp
               src/system/EventLog.cpp
               src/system/SystemManager.cpp src/scene/FusionThread.cpp src/scene/ExtrinsicMonitor.cpp
               src/scene/SceneReconstruction.cpp src/decision/DecisionThread.cpp
               src/decision/DecisionArbiter.cpp src/safety/MultiObjectTracker.cpp
               src/safety/ImmFilter.cpp src/safety/Hungarian.cpp src/safety/MotionPredictor.cpp
               src/safety/RecklessDrivingDetector.cpp src/fusion/HubReportFeeder.cpp
               src/fusion/ImuMount.cpp src/fusion/SensorFusion.cpp src/vehicle/VehicleInterface.cpp)
target_include_directories(replay_inspect PRIVATE include)
target_link_libraries(replay_inspect PRIVATE hub_sim_lib ${OpenCV_LIBS} CUDA::cudart
                      TensorRT::TensorRT Eigen3::Eigen yaml-cpp::yaml-cpp ar_schemas)

# The unit suite under test/unit/ is intentionally NOT built from here — see test/CMakeLists.txt.
# enable_testing() must come before the tests are declared, so ctest can find the integration tests.
enable_testing()
add_subdirectory(test)
