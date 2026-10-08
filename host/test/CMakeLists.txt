# Test aggregation for the main host build — see docs/BUILD_GUIDE.md Part 13.
#
# The unit suite under unit/ is a standalone project (see unit/CMakeLists.txt for why) and is NOT
# add_subdirectory()'d from here — pulling it in would silently give it this build's CUDA/TensorRT/
# SDL2 dependencies and break the CI runner. It is configured and built separately:
#     cmake -S host/test/unit -B host/test/unit/build && ctest --test-dir host/test/unit/build
#
# Integration tests (Part 13.2). These need real hardware, a GPU at minimum, and never run in CI.
# Run them from the build directory with:   ctest --test-dir build -L gpu
include(GoogleTest)

# Phase 5: TrtEngine against a tiny engine the test builds itself, so no downloaded model is needed.
add_executable(test_trt_engine integration/test_trt_engine.cpp ../src/inference/TrtEngine.cpp)
target_include_directories(test_trt_engine PRIVATE ../include)
target_link_libraries(test_trt_engine PRIVATE TensorRT::TensorRT CUDA::cudart GTest::gtest
                      GTest::gtest_main)
target_compile_definitions(test_trt_engine PRIVATE HOST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
gtest_discover_tests(test_trt_engine PROPERTIES LABELS gpu)

# Phase 5: GPU pre-processing against hand-computable frames.
add_executable(test_preprocess integration/test_preprocess.cpp ../src/inference/Preprocess.cpp
                               ../src/inference/Postprocess.cpp)
target_include_directories(test_preprocess PRIVATE ../include)
target_link_libraries(test_preprocess PRIVATE ${OpenCV_LIBS} CUDA::cudart GTest::gtest
                      GTest::gtest_main)
gtest_discover_tests(test_preprocess PROPERTIES LABELS gpu)

# Phase 8: OSRM routing and map-matching, against a road network the test writes and builds with
# the real OSRM tools, plus checks on the real Nairobi map when it has been built
# (host/scripts/refresh_osm.sh). Needs the OSRM tools, not a GPU:
#     ctest --test-dir build -L osrm
add_executable(test_osrm_navigation integration/test_osrm_navigation.cpp
               ../src/nav/NavigationEngine.cpp ../src/nav/MapMatcher.cpp
               ../src/nav/MapMatcherOsrm.cpp ../src/nav/Route.cpp ../src/nav/RoadStatus.cpp
               ../src/nav/RoadNetworkUpdater.cpp)
target_include_directories(test_osrm_navigation PRIVATE ../include ../src/nav)
target_link_libraries(test_osrm_navigation PRIVATE OSRM::osrm ar_schemas GTest::gtest
                      GTest::gtest_main)
target_compile_definitions(test_osrm_navigation PRIVATE
                           AR_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../data/maps")
gtest_discover_tests(test_osrm_navigation PROPERTIES LABELS osrm DISCOVERY_TIMEOUT 60)

# Phase 11: WindowedSink on the real GPU: shader projection vs the camera model, rule 4's
# depth-aware occlusion, video pass-through. Hidden window; needs a display (WSLg) and the GPU.
add_executable(test_windowed_sink integration/test_windowed_sink.cpp ../src/render/WindowedSink.cpp
               ../src/render/OverlayScene.cpp)
target_include_directories(test_windowed_sink PRIVATE ../include)
target_link_libraries(test_windowed_sink PRIVATE ${OpenCV_LIBS} SDL2::SDL2 OpenGL::GL GLEW::GLEW
                      Eigen3::Eigen GTest::gtest GTest::gtest_main)
gtest_discover_tests(test_windowed_sink PROPERTIES LABELS gpu DISCOVERY_TIMEOUT 60)

# Phase 4: CameraPipeline through real OpenCV + GStreamer, with GStreamer's test source standing in
# for the camera (no device, no GPU):   ctest --test-dir build -L camera
add_executable(test_camera_pipeline integration/test_camera_pipeline.cpp
               ../src/camera/CameraPipeline.cpp ../src/camera/CameraConfig.cpp
               ../src/system/EventLog.cpp)
target_include_directories(test_camera_pipeline PRIVATE ../include)
target_link_libraries(test_camera_pipeline PRIVATE ${OpenCV_LIBS} Eigen3::Eigen yaml-cpp::yaml-cpp
                      ar_schemas GTest::gtest GTest::gtest_main)
target_compile_definitions(test_camera_pipeline PRIVATE HOST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
gtest_discover_tests(test_camera_pipeline PROPERTIES LABELS camera DISCOVERY_TIMEOUT 60)

# The glue between the threads (Part 5.1): FusionThread's measurements, DecisionThread's cycle,
# RenderThread's inputs, the replay's injected obstacle, recordings. CPU only; here because the
# message headers include CUDA types.   ctest --test-dir build -L pipeline
add_executable(test_pipeline integration/test_pipeline.cpp
               ../src/scene/FusionThread.cpp ../src/scene/SceneReconstruction.cpp
               ../src/scene/ExtrinsicMonitor.cpp
               ../src/decision/DecisionThread.cpp ../src/decision/DecisionArbiter.cpp
               ../src/render/RenderThread.cpp ../src/render/ArRenderer.cpp
               ../src/render/OverlayScene.cpp ../src/render/SignTracker.cpp
               ../src/render/WindowedSink.cpp
               ../src/safety/MultiObjectTracker.cpp ../src/safety/ImmFilter.cpp
               ../src/safety/Hungarian.cpp ../src/safety/MotionPredictor.cpp
               ../src/safety/RecklessDrivingDetector.cpp ../src/fusion/HubReportFeeder.cpp
               ../src/fusion/ImuMount.cpp ../src/fusion/SensorFusion.cpp
               ../src/lidar/InjectedObstacle.cpp ../src/lidar/LidarProcessor.cpp
               ../src/system/Recording.cpp ../src/system/EventLog.cpp
               ../src/system/Recorder.cpp ../src/camera/CameraConfig.cpp
               ../src/inference/Postprocess.cpp ../src/vehicle/VehicleInterface.cpp
               ../src/system/SystemManager.cpp ../src/nav/Route.cpp
               ../src/nav/RoadSurfaceProjector.cpp)
target_include_directories(test_pipeline PRIVATE ../include)
target_link_libraries(test_pipeline PRIVATE ${OpenCV_LIBS} Eigen3::Eigen yaml-cpp::yaml-cpp
                      ar_schemas CUDA::cudart TensorRT::TensorRT SDL2::SDL2 OpenGL::GL GLEW::GLEW
                      GTest::gtest GTest::gtest_main)
gtest_discover_tests(test_pipeline PROPERTIES LABELS pipeline DISCOVERY_TIMEOUT 60)

# ExtrinsicMonitor (Part 12.2.1): needs OpenCV's image processing, so here rather than in the
# CPU-only unit suite. Synthetic scenes always; the KITTI drive when data/recordings/kitti_0005
# exists (skipped otherwise).
add_executable(test_extrinsic_monitor integration/test_extrinsic_monitor.cpp
               ../src/scene/ExtrinsicMonitor.cpp ../src/scene/SceneReconstruction.cpp
               ../src/inference/Postprocess.cpp
               ../src/system/Recording.cpp ../src/system/EventLog.cpp)
target_include_directories(test_extrinsic_monitor PRIVATE ../include)
target_compile_definitions(test_extrinsic_monitor PRIVATE HOST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(test_extrinsic_monitor PRIVATE ${OpenCV_LIBS} Eigen3::Eigen yaml-cpp::yaml-cpp ar_schemas
                      GTest::gtest GTest::gtest_main)
gtest_discover_tests(test_extrinsic_monitor PROPERTIES LABELS pipeline DISCOVERY_TIMEOUT 60)
