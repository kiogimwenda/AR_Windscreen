# Test aggregation for the main host build — see docs/BUILD_GUIDE.md Part 13.
#
# The unit suite under unit/ is a standalone project (see unit/CMakeLists.txt for why) and is NOT
# add_subdirectory()'d from here — pulling it in would silently give it this build's CUDA/TensorRT/
# SDL2 dependencies and break the CI runner. It is configured and built separately:
#     cmake -S host/test/unit -B host/test/unit/build && ctest --test-dir host/test/unit/build
#
# TODO(Part 13.2 / Phase 3): integration test targets, which DO link the full host build.
