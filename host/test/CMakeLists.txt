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
