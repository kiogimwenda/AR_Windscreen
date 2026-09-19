# Implementation Decisions

A short index of implementation choices made where `BUILD_GUIDE.md` was ambiguous, under-specified,
or internally inconsistent. One line of rationale each — the detailed narrative lives in
`progress-log.md`, and the two files are never merged.

---

## Phase 0 — Scaffold

**Repository root name.** The guide's Part 1 tree is rooted at `ar-drive-assist/`; this repository
is `AR_Windscreen`. The tree's *contents* were created at the repository root rather than nested
inside a second `ar-drive-assist/` directory, since the guide's tree describes repository layout,
not a subdirectory.

**`cmake/` directory.** Not listed in Part 1's tree, but Part 5.2 requires
`cmake/FindTensorRT.cmake`. Created at the repository root, which is where Part 5.2's relative path
points.

**`FindTensorRT.cmake` include path.** Part 5.2 writes
`include(${CMAKE_SOURCE_DIR}/../cmake/FindTensorRT.cmake)`. That snippet assumed `host/` was the
top-level CMake source directory; here the repository root is, so `${CMAKE_SOURCE_DIR}/..` would
resolve outside the repository. Changed to `${CMAKE_CURRENT_SOURCE_DIR}/../cmake/...`, which is
correct both now and if the tree is relocated.

**`host/src/` mirrors `host/include/` with header-only exceptions.** Part 1 says "one .cpp per .h".
Six headers are header-only by nature and were given no `.cpp`: `common/Types.h`,
`common/RingBuffer.h` (a template), `vehicle/HubProtocol.h` (packed structs),
`vehicle/Crc16.h` (a single `inline` function), `lidar/GroundPlaneModel.h` (a struct), and
`render/DisplaySink.h` (a pure-virtual interface). An empty translation unit for each would add
build-graph noise with no benefit. `vehicle/Crc16.h` and `vehicle/HubProtocol.h` additionally must
stay byte-for-byte identical to their firmware counterparts, so nothing may be moved out of them.

**`common/schemas/` and `common/RingBuffer.h`.** Not in Part 1's tree, but required by Part 3.5 and
Part 5.3 respectively. Created as stubs in Phase 0 alongside everything else so the structure is
stable; contents land in Phase 3.

**`.clang-format` added.** CI's `format-check` job (Part 13.5) runs `clang-format --dry-run
--Werror` but the guide specifies no style file, which would leave CI enforcing LLVM defaults by
accident. Pinned to Google style with a 100-column limit and 4-space indents, matching the code
samples throughout the guide itself.

**`host/test/CMakeLists.txt` does not add `test/unit/`.** Part 5.2's `add_subdirectory(test)` and
Part 13.1's "unit tests build as their own lightweight CMake project" pull in opposite directions.
Resolved in favour of 13.1, which states the reason explicitly: pulling `unit/` into the main build
would silently give it CUDA/TensorRT/SDL2 dependencies and break the GPU-less CI runner. The unit
suite is configured separately via `cmake -S host/test/unit -B host/test/unit/build`.

**`file(GLOB_RECURSE ...)` given `CONFIGURE_DEPENDS`.** The guide specifies a bare glob, which only
re-evaluates on reconfigure — meaning a newly added `.cpp` silently fails to build until something
else triggers a reconfigure. `CONFIGURE_DEPENDS` costs a stat sweep per build and removes that
failure mode.

**LICENSE: MIT.** The guide lists `LICENSE` in the tree without specifying a license. MIT chosen as
a permissive default; change it if the institution's FYP submission rules require otherwise.

**Firmware build is not driven from the top-level `CMakeLists.txt`.** The firmware is a PlatformIO
project with its own toolchain (Part 4.7). Keeping the two build systems separate mirrors the hard
host/hub boundary from Part 0.

## Phase 0 — Environment

**OpenCV 4.14.0-pre (source build at `/usr/local`) accepted in place of the guide's 4.10.0.** Part
2.5 pins 4.10.0, but the OpenCV already installed on this machine is a 4.14.0 source build with
CUDA 13.1 + cuDNN 9.19.1 and working `cv::cuda` on `sm_120`. Part 2.4's stronger rule — use the
CUDA/TensorRT versions already validated here, do not mix major versions — points the same way:
4.10.0 predates CUDA 13 and would not build against the installed toolkit. `find_package(OpenCV
4.10 REQUIRED)` is a minimum-version check, so the guide's CMake is unchanged. The Python `cv2`
4.10.0 in `~/ml-env` is an unrelated pip wheel with no CUDA and is not what the C++ build links.

**PCL requested by component, not as a whole.** Debian 13's VTK 9.3 CMake package declares link
dependencies on targets it never calls `find_dependency()` for (`JsonCpp::JsonCpp`, then
`MPI::MPI_C`, and more behind those), so any PCL lookup that reaches VTK fails at configure time.
Requesting `common filters segmentation kdtree search sample_consensus` avoids VTK entirely and is
correct on its own terms — Part 8.1 needs exactly those, and this project renders through OpenCV
and SDL2, never `pcl::visualization`. `io` is excluded specifically because it re-admits VTK; if
Phase 6 wants debug point-cloud dumps, write the PCD directly.

**`lib_deps = FreeRTOS` replaced with `stm32duino/STM32duino FreeRTOS@^10.3.3`.** Part 4.1's bare
name resolves to feilipu's AVR-only `Arduino_FreeRTOS_Library` and fails on `<avr/io.h>` when
cross-compiling for ARM. The STM32 port exposes the native FreeRTOS API (`xTaskCreate`,
`vTaskStartScheduler`) that Part 4.3 uses directly, so no Part 4.3 code needs rewriting — a
CMSIS-RTOS wrapper would have forced that.

**OSRM built from source (v6.0.0) rather than `apt install osrm-tools`.** Part 2.7's package does
not exist in Debian 13 under any name. Built against system Boost 1.83 and oneTBB 2022.1 into
`~/src/osrm-backend`. The same gap affects Part 13.5's CI job, which installs `libosrm-dev` on
`ubuntu-latest` — that will need addressing before Phase 8 puts anything in `test_map_matcher.cpp`.

**STM32 board left as the guide's `blackpill_f411ce` placeholder.** Part 4.1 says to substitute the
exact board on hand; that board has not been identified yet. To be revisited at the start of
Phase 2.

**OSRM built as a pinned v6.0.0 against system Boost/oneTBB, installed to `/usr/local`.** Source
kept in `~/src/osrm-backend`, outside this repository — it is a system dependency, not project
source. Using the distribution's Boost 1.83 and oneTBB rather than bundled copies keeps a single
copy of each in the host process, which matters because the host binary links OSRM alongside PCL
and PCL also uses Boost. Its build needs `-Wno-error=array-bounds` and friends passed via
`CMAKE_CXX_FLAGS_RELEASE` (not `CMAKE_CXX_FLAGS`, where OSRM's own appended `-Werror` would defeat
them) to get past GCC 14 false positives in OSRM's vendored fmt/sol2 headers.

## Phase ordering under a hardware-free start

**Phase 0 treated as complete-except-check-4, rather than blocking all work.** Part 2.10's camera
check cannot pass until hardware is purchased. The guide's rule against starting a phase before the
previous one's exit criteria are met exists so that later work is not built on an unverified
foundation — and nothing in Phase 1 (the shared protocol and its CRC tests) depends on a camera
existing. Blocking Phase 1 on check 4 would serve the letter of the rule against its purpose. The
check stays explicitly open in `progress-log.md` rather than being waived.
