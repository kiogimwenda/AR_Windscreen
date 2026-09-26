# Progress Log

A dated, per-task record of work on this project — one entry per discrete task, written as the task
finishes. Separate from `decisions.md`, which is the short index of choices made under ambiguity.

Format is fixed by `BUILD_GUIDE.md` Part 1.1.

---

## 2026-09-18 — Phase 0: repository scaffold (Part 1 tree)

**Attempted:** Create the complete Part 1 directory tree with stub files in a single pass, before
any real logic exists, so the structure stays stable while individual files are filled in phase by
phase. This is Part 1's explicit instruction for Phase 0 and the first half of that phase's
deliverables.

**Built/changed:** Repository root now holds `README.md` (rewritten to describe the system and lead
with the safety rule), `LICENSE` (MIT), `.gitignore` (with `.claude/` as its literal first line per
Part 1.2), `.clang-format`, and a top-level `CMakeLists.txt` that adds `host/` and nothing else.
Created `docs/` (this file, `decisions.md`, `BUILD_GUIDE.md` itself, `architecture/`, `report/`,
`presentation/`), `cmake/FindTensorRT.cmake` (written in full — it is a build dependency, not
project logic), `firmware/sensor_actuator_hub/` (`platformio.ini` per Part 4.1, three `include/hub/`
headers, `main.cpp`, four task stub pairs, six driver stub pairs, one test stub),
`host/include/ar_drive_assist/` (23 header stubs across 12 subsystem directories, plus four `.fbs`
schema stubs), `host/src/` (17 `.cpp` stubs mirroring the headers, plus `main.cpp`),
`host/config/` (six YAML files carrying the templates the guide specifies literally),
`host/scripts/` (five stubs), `host/test/` (six unit-test stubs, a standalone `unit/CMakeLists.txt`,
an aggregating `test/CMakeLists.txt`, an `integration/README.md`), `tools/bench_rig/` (two stubs),
and `.github/workflows/build.yml` (Part 13.5, verbatim).

**Reasoning:** Stubs carry a `#pragma once` and a `TODO(Part N)` comment naming both the guide Part
that specifies the file and the phase that fills it, so the tree itself records the execution plan
and a stale stub is obvious at a glance. Safety-critical stubs carry more than a pointer:
`BrakeActuatorDriver.h` states up front that it is the second, independent force ceiling and that
`kMaxSafeIntensity` is never raised without a bench re-test, so that constraint is present in the
file from before the file does anything. `host/include/.../vehicle/HubProtocol.h` and `Crc16.h` were
created as literal copies of their firmware counterparts rather than as separately-worded stubs, so
CI's `protocol-sync-check` diff is green from the first commit and any later drift is a real signal
rather than pre-existing noise. Config YAML files were given the guide's literal template values
rather than left empty, since those values are specification, not logic — with measured quantities
(camera matrix, extrinsics) left as empty lists so a placeholder can never be mistaken for a real
calibration.

**Problems hit:** Part 5.2's `include(${CMAKE_SOURCE_DIR}/../cmake/FindTensorRT.cmake)` resolves
outside the repository once `host/` is a subdirectory of a top-level project rather than the
top-level project itself; changed to `CMAKE_CURRENT_SOURCE_DIR`. Part 5.2's `add_subdirectory(test)`
also conflicts with Part 13.1's requirement that the unit suite build standalone without CUDA/SDL2;
resolved in favour of 13.1, whose rationale (GPU-less CI) is stated explicitly. Both logged in
`decisions.md`, along with four smaller under-specifications (missing `cmake/` directory, no license
named, no clang-format style, "one .cpp per .h" against six header-only files).

**Still open:** Every stub is still a stub — no logic anywhere. The environment half of Phase 0
(Part 2.10's five checks) has not been run yet, and `cmake` configuring `host/` will fail until the
missing dependencies are installed. `platformio.ini`'s board is a placeholder until the exact F4
board on hand is confirmed. `attach_usb_devices.ps1` cannot be completed until `usbipd list` on the
Windows side gives the three bus IDs.

## 2026-09-18 — Phase 0: development environment (Part 2)

**Attempted:** Bring the WSL2 Debian instance up to Part 2's dependency list and run Part 2.10's
five verification checks, which are the environment half of Phase 0's exit criteria.

**Built/changed:** Surveyed what was already present (CUDA 13.1, TensorRT, PlatformIO 6.1.19,
Eigen3, spdlog, GTest, GStreamer) and installed the rest of Part 2.3/2.6 via apt: `libpcl-dev`
(1.15.0), `libsdl2-dev` (2.32.4), `libglm-dev`, `libflatbuffers-dev` + `flatbuffers-compiler`
(23.5.26), `v4l-utils`, plus `libjsoncpp-dev`. Fixed `firmware/sensor_actuator_hub/platformio.ini`
and filled in `src/main.cpp` enough to link. Amended `host/CMakeLists.txt`'s PCL lookup. Cloned and
configured OSRM v6.0.0 from source into `~/src/osrm-backend` (build running).

**Reasoning:** Part 2.4 says to use the CUDA/TensorRT versions already validated on this machine
rather than mixing major versions, so nothing GPU-side was reinstalled — only verified. The
existing OpenCV turned out to be a source build at `/usr/local` (4.14.0-pre, CUDA 13.1 + cuDNN
9.19.1, CUDA modules present), which is what the C++ build links against; the 4.10.0 reported by
`import cv2` is an unrelated pip wheel inside `~/ml-env` with no CUDA, which is why the earlier
survey looked contradictory. The installed build satisfies Part 2.10's intent (OpenCV with working
CUDA) at a newer version than the guide names, and is arguably the version that *should* be there:
the guide's 4.10.0 predates CUDA 13 and would not build against the toolkit actually installed.

**Verification (Part 2.10, five checks):**
1. `nvcc --version` — PASS, CUDA 13.1.115.
2. Trivial CUDA kernel compiles and runs — PASS. A `saxpy` kernel built with `-arch=sm_120` runs on
   the "NVIDIA GeForce RTX 5060 Laptop GPU", reported as `sm_120`, max error 0.0. This confirms
   Blackwell compute capability 12.0 directly, which is the number Part 2.5 warns is the single
   most common copy-paste mistake when reusing old build scripts (the Orin Nano's 8.7).
3. OpenCV with CUDA — PASS with a documented version deviation. A C++ program linking the
   `/usr/local` OpenCV reports 1 CUDA-enabled device, `sm_120`, and a GPU `cv::cuda::multiply`
   returning correct results.
4. USB camera at `/dev/video0` — **NOT YET RUN.** No `/dev/video*` node exists because nothing has
   been attached through `usbipd-win` on the Windows side. Requires physical action.
5. `pio run` on the firmware project — PASS, after two fixes (below). Firmware links at 0.8% RAM /
   2.1% flash on the F411CE.

Additionally, `cmake -S . -B build` configures the whole project without error — the other half of
Phase 0's exit criteria — locating OpenCV 4.14.0, PCL 1.15.0, Eigen3, glm, spdlog, SDL2, CUDA 13.1,
Flatbuffers, GTest and TensorRT (via the project's own `cmake/FindTensorRT.cmake`).

**Problems hit:** Three real defects, none of them cosmetic.

*Part 4.1's `lib_deps = FreeRTOS` is wrong for this target.* That bare name resolves in the
PlatformIO registry to feilipu's `Arduino_FreeRTOS_Library`, which is an AVR port and fails
immediately on `#include <avr/io.h>` when cross-compiling for ARM. Replaced with
`stm32duino/STM32duino FreeRTOS@^10.3.3`, which is the STM32 port and — importantly — exposes the
native FreeRTOS API (`xTaskCreate`, `vTaskStartScheduler`) that Part 4.3 uses directly, rather than
a CMSIS-RTOS wrapper that would have required rewriting Part 4.3's task setup.

*Firmware would not link with a comment-only `main.cpp`.* PlatformIO's Arduino framework supplies
its own `main()` which calls `setup()` and `loop()`, so both must exist. Written as a documented
stub explaining why `loop()` stays permanently empty under FreeRTOS.

*Debian 13's VTK 9.3 CMake package is broken, and PCL drags it in.* `find_package(PCL)` fails at
configure time because VTK's config declares link dependencies on targets (`JsonCpp::JsonCpp`, then
`MPI::MPI_C`, and more behind them) without ever calling `find_dependency()` to define them.
Installing `libjsoncpp-dev` and finding it first cleared the first error and revealed the next, so
chasing the chain would have meant linking OpenMPI into a real-time driving system to satisfy a
packaging bug. Instead PCL is now requested by component — `common filters segmentation kdtree
search sample_consensus`, exactly what Part 8.1 needs — which avoids VTK entirely. `io` is
deliberately excluded because it is what pulls VTK back in.

*Part 2.7's `apt install osrm-tools` does not work on Debian 13* — OSRM is not packaged in trixie at
all, under any name. Building v6.0.0 from source against system Boost 1.83 and oneTBB 2022.1
instead; it configures cleanly and the compile is running.

**Still open:** Part 2.10 check 4 (camera at `/dev/video0`) needs `usbipd-win` attachment from the
Windows side — a physical/host-side action. The OSRM source build is still running; until it
installs, `host/CMakeLists.txt`'s `osrm` link entry will fail at *link* time (it configures fine,
since a bare library name is passed straight to the linker without a configure-time check). Part
13.5's CI `host-unit-tests` job installs `libosrm-dev`, which does not exist on `ubuntu-latest`
either — that job will need the same treatment before Phase 8, and is noted but not yet fixed.
`platformio.ini`'s board is still the guide's placeholder `blackpill_f411ce`.

## 2026-09-18 — Phase 0: OSRM built from source (Part 2.7)

**Attempted:** Provide the OSRM routing/map-matching dependency that Part 11.2–11.4 are built on,
since Part 2.7's `apt install osrm-tools` has no package on Debian 13.

**Built/changed:** Cloned OSRM v6.0.0 into `~/src/osrm-backend` (outside this repository — it is a
system dependency, not project source), built it against system Boost 1.83 and oneTBB 2022.1, and
installed to `/usr/local`. Provides `osrm-extract`, `osrm-partition`, `osrm-customize`,
`osrm-contract`, `osrm-datastore`, `osrm-routed`, plus `libosrm.a` and headers under
`/usr/local/include/osrm`.

**Reasoning:** v6.0.0 rather than master, so the version is pinned and reproducible for the report.
Built against the distribution's Boost and oneTBB rather than OSRM's bundled alternatives, to keep
one copy of each library in the process — this matters because the host binary will link OSRM
alongside PCL, which also uses Boost.

**Problems hit:** The first build failed at 87% with seven `-Werror=array-bounds` errors, all of
them GCC 14 false positives inside OSRM's *own* vendored third-party headers (fmt 10 and sol2), not
in OSRM's code or ours. OSRM adds `-Werror` to `CMAKE_CXX_FLAGS` via `cmake/warnings.cmake`.
Suppressing it needed care: passing `-Wno-error=array-bounds` in `CMAKE_CXX_FLAGS` would have been
silently defeated, because OSRM *appends* its own flags to that variable and later flags win — so
`-Werror` would land after the exemption and re-enable it. Putting the exemptions in
`CMAKE_CXX_FLAGS_RELEASE` instead works, because CMake emits `${CMAKE_CXX_FLAGS}` followed by
`${CMAKE_CXX_FLAGS_<CONFIG>}`, placing them after `-Werror`. Rebuilt clean.

**Verification:** A C++ program including `<osrm/osrm.hpp>`, `<osrm/route_parameters.hpp>` and
`<osrm/match_parameters.hpp>` compiles, links against `libosrm`, and constructs an `EngineConfig`
with the MLD algorithm — the exact API surface Part 11.2 and 11.3 specify. This is the real proof
the dependency is usable, rather than just that files landed on disk.

**Still open:** `pkg-config --modversion libosrm` reports the literal string `packagejson.version`
— an unexpanded variable in OSRM's installed `.pc` file. Harmless here since `host/CMakeLists.txt`
links OSRM by plain library name, but worth knowing before anyone tries to drive the link line from
pkg-config. Part 13.5's CI job still installs the nonexistent `libosrm-dev`; unchanged, and still
due before Phase 8.

## 2026-09-19 — Phase 0: check 4 reclassified as blocked on procurement, not on setup

**Attempted:** Close Part 2.10's fifth check (a USB camera appearing as `/dev/video0` after a
`usbipd-win` attach), the last outstanding item in Phase 0.

**Built/changed:** Nothing built. The check is reclassified from "pending a Windows-side attach" to
"blocked until hardware is purchased" and the earlier entry's framing is corrected here rather than
edited away.

**Reasoning:** The 2026-09-18 entry recorded check 4 as not yet run "because nothing has been
attached through usbipd-win", which implied a camera existed and was merely unattached. No project
hardware has been bought at all. That matters well beyond this one check: it means the guide's Part
14 roadmap cannot be walked straight through in order, because Phases 2, 3, 4, 6, 11, 12, 13, 14
and 15 all have exit criteria that require physical hardware in hand, and Phases 7 and 9 can be
written but not fully verified without it.

**Problems hit:** The error was mine in the earlier entry — I inferred from a missing `/dev/video*`
node that a camera was present but unattached, rather than asking. Recorded here rather than
silently rewritten, since the point of this log is what actually happened.

**Still open:** Part 2.10 check 4 stays open until a camera exists. Phase 0's other six criteria
(the four other Part 2.10 checks, `cmake` configuring `host/`, and `pio run` building the firmware)
all pass, so Phase 0 is complete except for that one hardware-gated item. A hardware bill of
materials and a decision on which non-hardware phases to bring forward are both now on the critical
path, and neither exists yet.

## 2026-09-19 — Phase 1: shared protocol headers (Part 3.1, 3.2)

**Attempted:** Implement `Protocol.h` and `Crc16.h` — the host<->hub wire format and its checksum —
as two byte-for-byte identical copies, one under `firmware/`, one under `host/`.

**Built/changed:** `firmware/sensor_actuator_hub/include/hub/Crc16.h` and `.../hub/Protocol.h`,
copied verbatim to `host/include/ar_drive_assist/vehicle/Crc16.h` and `.../vehicle/HubProtocol.h`.
`Crc16.h` is Part 3.2's bit-serial CRC-16/CCITT-FALSE. `Protocol.h` carries Part 3.1's
`START_BYTE`, `MAX_PAYLOAD`, `MessageType` and the three packed payload structs, plus four
additions described below.

**Reasoning:** Four things were added beyond Part 3.1's literal text, all in service of the
"identical on both ends" requirement rather than in spite of it.

*Frame geometry constants* (`HEADER_SIZE`, `CRC_SIZE`, `CRC_COVERED_OFFSET`, `MAX_FRAME_SIZE`) are
derived from Part 3.1's ASCII frame diagram. Without them, `VehicleInterface` (Part 11.1) and
`CommsTask` (Part 4.3) would each re-derive the same offsets by reading that diagram — and the CRC
covered range in particular is the single most likely framing bug, because getting it wrong leaves
both ends self-consistent and mutually incompatible, which only shows up on a bench.

*`static_assert` on every struct size.* This is the important one. The two ends are different
architectures — Cortex-M4 at 32 bits, x86-64 — with different compilers. `#pragma pack(push, 1)`
is what removes padding, but nothing checks that it worked. These asserts turn a silent layout
disagreement (the hub reading a brake intensity out of what the host thought was a timestamp) into
a build failure. They cost nothing at runtime and are checked on both sides because both sides
compile this same file.

*`#include <cstddef>`.* Part 3.1's snippet uses `size_t` but includes only `<cstdint>`. It happens
to compile where `<cstdint>` transitively drags in `<cstddef>`, which is not guaranteed.

*A note that HEARTBEAT carries no payload.* Part 3.3 requires the message but defines no struct for
it. Recorded as length 0 explicitly, so the next person does not have to infer it. Staleness of a
real request is already covered by `ActuationCommand::hostTimestampMs`, so the heartbeat needs to
carry nothing.

**Problems hit:** Nothing went wrong in writing the headers. The one thing worth recording is what
the struct sizes turned out to be: `SensorReport` is 63 bytes against a `MAX_PAYLOAD` of 64. One
byte of headroom. A `static_assert` now guards that, because adding a single `float` to
`SensorReport` would push it over the cap and frames would start being rejected as oversized with
no obvious connection to the change that caused it.

**Still open:** Nothing from this task. Framing encode/decode belongs to `VehicleInterface` and
`CommsTask` in Phases 3 and 2 respectively, per Part 11.1 — this file deliberately holds only the
shared format, not the shared machinery.

## 2026-09-19 — Phase 1: CRC and layout tests, both sides of the link

**Attempted:** Satisfy Phase 1's exit criteria — a passing host-side CRC test, a matching
firmware-side test confirming identical behaviour, and a green `protocol-sync-check`.

**Built/changed:** `host/test/unit/test_crc16.cpp` (12 GoogleTest cases) and
`firmware/sensor_actuator_hub/test/test_protocol_crc.cpp` (7 Unity cases). Wired up
`host/test/unit/CMakeLists.txt` with an `add_unit_test()` helper so later phases add one line each,
and added a `[env:native]` PlatformIO environment plus `default_envs = stm32f4_hub` so `pio test -e
native` runs the firmware's tests on this machine while a bare `pio run` still builds only the
STM32 target (which is what CI's `firmware-build` job invokes). Added `#include "hub/Protocol.h"`
and `"hub/Crc16.h"` to the otherwise-still-stubbed `CommsTask.cpp`.

**Reasoning:** The CRC vectors are deliberately duplicated between the two test files rather than
factored into a shared header. CI's `protocol-sync-check` already proves the two headers are
byte-identical; what the tests add is that each copy independently produces the *standard* results.
A shared vector header would collapse that into one point of failure, which is the opposite of what
Part 3's "implemented twice, kept identical" rule is for. The canonical check value
(`crc16_ccitt_false("123456789") == 0x29B1`) is asserted on both sides specifically because it
proves the implementation is the standard CRC-16/CCITT-FALSE rather than merely self-consistent —
a self-consistent but non-standard CRC would pass every other test in both files.

The `CommsTask.cpp` include is not premature scaffolding. `Protocol.h`'s `static_assert`s only
execute in a translation unit that compiles the header, and the layout question they answer is
specifically about the Cortex-M4 — but both test suites run on x86-64. Without something in the
firmware including the header, Phase 1's "identical on both ends" claim would have rested entirely
on two tests that ran on the same architecture. `CommsTask` is the task that genuinely owns framing
(Part 4.3), so this is where the include belongs anyway.

**Verification:** Host suite 12/12 via `ctest`. Firmware suite 7/7 via `pio test -e native`. `pio
run` cross-compiles the firmware for ARM successfully, which is what actually exercises the layout
asserts on the real target. Both `diff`s in CI's `protocol-sync-check` are clean, and the
`format-check` command passes.

The layout assert was then negative-tested rather than assumed: changing the expected
`SensorReport` size to 64 and running `pio run` fails the ARM build with `static assertion failed:
SensorReport layout changed - update BOTH copies`, and restoring it builds clean again. A safety
net that has never been observed to catch anything is not yet known to be a safety net.

**Still open:** Nothing from this task. The five later test stubs remain stubs; their targets are
listed as comments in `host/test/unit/CMakeLists.txt` against the phases that will add them.

## 2026-09-19 — Fix: `.clang-format-ignore` was silently matching nothing

**Attempted:** Confirm CI's `format-check` command passes when run locally.

**Built/changed:** Corrected `.clang-format-ignore` from `firmware/sensor_actuator_hub/.pio/*` to
`**/.pio/**` and `**/build/**`.

**Reasoning:** In a clang-format ignore file, patterns are globs relative to the file's directory
and `*` does not cross a `/`. `.pio/*` therefore matched only the first level under `.pio` and
nothing beneath it, so the vendored Arduino/FreeRTOS/Adafruit sources were still being checked.

**Problems hit:** This was a false pass on 2026-09-18, not a new break. That day's check reported
clean only because `.pio/` had just been wiped, so there was nothing for the broken pattern to fail
to match. The ignore file was written and never actually exercised. Fixed and then verified in both
directions: the full command now reports zero violations, and dropping a deliberately misformatted
file into `host/src/` still produces violations, confirming the ignore is not simply swallowing
everything.

**Still open:** Nothing.

## 2026-09-24 — Hardware bill of materials with Kenyan sourcing

**Attempted:** Produce the hardware bill of materials that the 2026-09-19 entry identified as
being on the critical path, sourcing from local (Kenyan) suppliers first and naming an import route
only where nothing suitable is sold locally.

**Built/changed:** `docs/bill-of-materials.md` — every hardware item referenced by Parts 4, 6, 8,
13.3 and 15, grouped as hub, sensors, actuation/safety, power, bench rig and installation, each with
the spec that matters, a supplier, a price and stock status as seen on 2026-09-24. Includes an
import-bundle cost estimate and a budget summary. Four entries added to `decisions.md`.

**Reasoning:** Suppliers were checked online (K-Technics, Pixel Electric, Nerokas, Jumia, Kilimall).
Four items had no local listing: the Livox Mid-360, its aviation cable (not included with the
sensor), the BNO085, and a suitable brake actuator. These are grouped into a single import so duty
and clearing are paid once. The camera line requires manual focus control because autofocus moving
the lens would silently invalidate the Part 12.1 intrinsics.

**Problems hit:** Three issues that the guide does not surface, found while matching parts to specs.
First, every locally sold linear actuator is a lead-screw type, which holds position when unpowered.
Coupled rigidly to the pedal, it would leave the brake applied after a kill-switch cut, directly
contradicting the hard safety rule. The actuator is therefore marked "do not buy" pending a design
decision, not substituted. Second, the STM32's 3.3 V outputs sit below the input threshold of the
common BTS7960 and relay modules. Third, a 100–140 W USB-C car charger cannot power an RTX 5060
laptop under load. Many shop listings showed "sold out" (GPS, APDS-9960, F411 board, Pixel's
BTS7960), so all prices are marked for phone confirmation before purchase.

**Still open:** Brake actuator mechanical design (BOM note B), to be agreed with the supervisor.
Laptop in-car power approach (note D). BNO085 import vs local BNO055 plus a driver change. A clearing
agent's quote for the LiDAR, and whether departmental education duty relief applies. Part 2.10
check 4 remains open until the camera is bought.

## 2026-09-24 — Phase 3 (software part): `RingBuffer` message bus (Part 5.3)

**Attempted:** Implement the single-producer/single-consumer ring buffer that every host
subsystem communicates through. Phase 3 was started with its hardware-free parts only
(`RingBuffer`, `EventLog`, `SystemManager`); `VehicleInterface` and the Phase 3 exit test wait for
the Phase 2 hub.

**Built/changed:** `host/include/ar_drive_assist/common/RingBuffer.h`: `push` (copy and move),
`pop`, `popLatest`, `sizeApprox`. `host/test/unit/test_ring_buffer.cpp` has 9 tests, 2 of them
two-threaded. It is registered in `host/test/unit/CMakeLists.txt` with `Threads::Threads`.
`.gitignore` now also covers `host/test/unit/build-*/` (sanitizer build directories).

**Reasoning:** Both indices are monotonically increasing counters, each written by one thread
only, so no compare-and-swap is needed. Using counters rather than wrapped indices gives a
capacity of exactly N instead of N−1; N must be a power of two so the modulo stays correct across
counter wrap. The producer publishes a slot with a `release` store of `head_`, and the consumer's
`acquire` load of `head_` guarantees it sees the fully written item. The mirror-image pair on
`tail_` stops the producer overwriting a slot the consumer is still moving out of. `head_` and
`tail_` sit on separate cache lines to avoid false sharing.

Part 5.3's "drop oldest" policy is realised by the consumer (`popLatest`), not the producer. In an
SPSC queue only the consumer owns `tail_`, so a producer discarding the oldest item would race the
consumer reading it. `push` therefore never overwrites; it returns false and leaves the policy to
the caller, as Part 5.3's own signature comment says. `popLatest` resets discarded slots to `T{}`
so a frame bus doesn't pin up to N stale `cv::Mat` buffers.

**Problems hit:** The first ThreadSanitizer run aborted at startup ("unexpected memory mapping")
because this kernel's address-space randomisation is incompatible with TSan's shadow memory. An
earlier run had appeared to pass only because the random layout happened to work that time. It
now runs under `setarch -R` (randomisation disabled for that process only):
`cmake -S host/test/unit -B host/test/unit/build-tsan -DCMAKE_CXX_FLAGS="-fsanitize=thread -O1 -g"`,
then `setarch -R host/test/unit/build-tsan/test_ring_buffer`.

**Verification:** 9/9 pass in the normal build (21/21 across the suite including CRC). Under TSan,
three consecutive runs are 9/9 with zero race reports. Negative test: weakening the producer's
`head_.store` from `release` to `relaxed` makes TSan report a data race on the consumer's slot
read (`RingBuffer.h:92`), which is exactly the failure the ordering exists to prevent. Restored
and re-verified clean.

**Still open:** Nothing from this task.

## 2026-09-24 — Phase 3 (software part): FlatBuffers message schemas and generation (Part 3.5)

**Attempted:** Fill in the four bus message schemas deferred to Phase 3 by Phase 0, and generate
their C++ headers at build time as Part 3.5 requires. `EventLog` needs the generated
`ActuationRequest` type.

**Built/changed:** `detections.fbs`, `vehicle_pose.fbs`, `actuation_request.fbs` and
`road_projected_route.fbs`, with Part 3.5's definitions verbatim. New
`cmake/FlatBufferSchemas.cmake` provides `ar_add_schema_library()`, which runs
`flatc --cpp --gen-object-api` per schema into the build tree and exposes the output as an
INTERFACE library. `common/Types.h` aliases the generated object-API structs under the guide's
names (`ActuationRequest` = `schema::ActuationRequestT`, and so on) and defines the `Config`
struct. Built `flatc` 24.3.25 from OSRM's vendored FlatBuffers source (`~/src/flatc-24.3.25-build`)
and installed only that binary to `/usr/local/bin/flatc`.

**Reasoning:** The bus carries the object-API structs (`...T`), not serialized buffers. Serialising
and parsing on every hop between two threads of one process buys nothing, and the byte form stays
available for recording and replay (Part 13.2) with no second definition of any type.
`Config` only holds the sections a built subsystem reads (vehicle params, decision thresholds).
Later phases add theirs with the code that consumes them.

**Problems hit:** The first build failed on every generated header with FlatBuffers' own
"Non-compatible flatbuffers version included". Two versions are installed. Debian's 23.5.26 is in
`/usr/include` and matches `/usr/bin/flatc`. OSRM's 24.3.25, installed on 2026-09-18, is in
`/usr/local/include`, which the compiler searches first. Removing OSRM's copy was not an option:
OSRM's public headers (`osrm/engine/api/base_result.hpp`) include it, and the host links OSRM. So
the whole host standardises on 24.3.25, and a matching `flatc` was built from the exact source OSRM
vendors.

To stop this recurring as an unexplained compile error, `FlatBufferSchemas.cmake` now compares
`flatc --version` with the version in the `flatbuffers/base.h` the compiler will see, and fails
configure with both versions and paths named. Verified against the real case: the stale cached
`/usr/bin/flatc` made configure fail with that message, and clearing the cache fixed it. CI is
unaffected: Ubuntu's `flatc` and headers come from the same apt package.

**Still open:** The main host build still has to link `ar_schemas` (done with `SystemManager`,
below).

## 2026-09-24 — Phase 3 (software part): `EventLog` (Part 11.6)

**Attempted:** Implement the host's actuation evidence log. Part 11.6 requires a monotonic
timestamp on every line and flushing that survives a crash.

**Built/changed:** `system/EventLog.h` and `src/system/EventLog.cpp`: `logActuationRequest`,
`logAckStatus`, `logFault`, `logGeneral`, plus `healthy()`. `test_event_log.cpp` has 11 tests.

**Reasoning:** Each line is one POSIX `write()` on an `O_APPEND` descriptor, with no userspace
buffer, so it is in the kernel the moment `write()` returns and survives any crash of the
process. Actuation, ack and fault lines are also `fsync()`'d, so they survive a power cut, which
matters in a car where ignition-off can take the laptop down seconds after an actuation.
`logGeneral` skips `fsync` because it is too slow for routine lines.

Timestamps are `steady_clock` milliseconds since the log opened. Wall-clock time can jump under
NTP or GPS correction and would distort request-to-ack gaps. The `SESSION_START` line records the
UTC wall time for mapping. A per-line sequence number makes a missing line provable. The timestamp
is taken inside the mutex, so line order, sequence order and time order always agree. Values are
quoted and escaped, so one event is always exactly one line.

A write or `fsync` failure latches `healthy()` false and reports to stderr. The log cannot record
its own failure. Phase 10's arbiter is meant to read this flag: a system that can no longer record
actuation evidence should not request actuation. The constructor throws if the file can't be
opened, so the system can't start without its evidence trail.

**Verification:** 11/11 pass, and 32/32 across the unit suite. The key test forks a child that
logs a fault and then `abort()`s (no destructors, no flushing). The parent finds the line on disk
and confirms `SESSION_END` is absent, proving the destructor really never ran. A concurrency test
with 8 threads × 500 lines checks that every line is well-formed, the sequence has no gaps and
matches line order, timestamps never go backwards, and no message is lost or doubled. It is clean
under ThreadSanitizer. The disk-full path is exercised by logging to `/dev/full`.

**Still open:** Wiring `healthy()` into the arbiter's arming check is Phase 10's.

## 2026-09-24 — Phase 3 (software part): `SystemManager` and `main.cpp` (Part 5.4, Part 11.5)

**Attempted:** Implement subsystem lifecycle, config loading, signal handling and the ordered
shutdown with its final zeroed `ActuationCommand`, then wire the real host entry point.

**Built/changed:** `system/SystemManager.h` and `src/system/SystemManager.cpp`: `loadConfig`,
`start<T>(args...)`, `runUntilShutdown`, `requestShutdown`, `setFinalCommandHook`,
`installSignalHandlers`, `registerCrashFrame`. `src/main.cpp` loads config, opens the log, installs
handlers and runs; each later phase adds its own `start<>` line. `test_system_manager.cpp` has 19
tests. yaml-cpp was added to both builds.

**Reasoning:** Shutdown sets `stop`, joins every thread in start order, and only then calls the
final-command hook. Once all threads have returned, nothing can produce a new actuation request,
so the zeroed command is provably the last thing the hub hears. Called earlier, a still-running
arbiter could slip a brake request in behind it.

A subsystem whose `run()` throws, or returns before being asked to, is logged as a FAULT and takes
the whole system down. A pipeline missing a stage is broken, not degraded, and stopping hands
control to the hub watchdog.

Signal handlers only do async-signal-safe things:
- SIGINT/SIGTERM set a lock-free flag, which `runUntilShutdown` polls every 50 ms.
- A second SIGINT/SIGTERM means shutdown is stuck. It writes the crash frame and calls `_exit`.
- Crash signals (SEGV/BUS/FPE/ILL/ABRT) make a single `write()` of a pre-encoded zeroed frame to a
  registered fd, then re-raise, so the process still dies with its real cause.

VehicleInterface will register that frame and fd when it opens the port. This is Part 5.4's
belt-and-braces measure; the hub watchdog does not depend on it.

`loadConfig` fails with the file and key named for anything missing, mistyped or out of range.
The brake ceiling is read as a wide integer and range-checked (0–255) rather than converted
straight to `uint8_t`.

**Problems hit:**
- *Host binary failed to link inside `libosrm.a`* (undefined Boost.Thread symbols). A static
  archive records none of its own dependencies, and `libosrm.a`'s `osrm.cpp.o` contains weak copies
  of `std::string` member functions. Once `main.cpp` used `std::string`, the linker pulled that
  object in, and its Boost references with it. This was confirmed by intersecting our objects'
  undefined symbols with the archive's definitions, not guessed. OSRM's own `libosrm.pc` can't
  supply the missing list: it contains CMake target names instead of linker flags. The new
  `cmake/FindOSRM.cmake` defines `OSRM::osrm` with the dependencies (Boost date_time, iostreams and
  thread, TBB, zlib, rt) and OSRM's compile definitions. This is the link-time failure Phase 0's
  entry predicted.
- *Test expectation wrong about the component name:* demangled type names are fully qualified.
  The test was fixed; the code was right.
- *Format check:* clang-format wrapped an example log line in `EventLog.h`'s comment, making it
  read as two lines. The example was shortened. `.clang-format-ignore` gained `**/build-*/**`.

**Verification:** 51/51 unit tests. SystemManager tests were clean under ThreadSanitizer in 3 runs.
- *Negative test:* with the final-command hook moved before the joins, the ordering test fails (0
  of 3 subsystems finished when the hook ran). Restored.
- *Signal paths, tested in forked children:*
  - SIGINT gives an ordered shutdown, exit 0, and the hook runs.
  - SIGSEGV delivers the registered frame byte-exact through a pipe and still kills the child with
    SIGSEGV.
  - An unregistered frame writes nothing on SIGABRT.
  - With a subsystem that ignores `stop`, a second SIGINT exits 130 and delivers the frame.
- *Live binary:* `build/host/ar_drive_assist` runs, and Ctrl+C gives exit 0 with the log trail
  `shutdown requested by signal → stopping 0 subsystem(s) → shutdown complete → SESSION_END`.
  A missing config dir refuses to start (exit 1, file named).
- A manual "two SIGTERMs back to back" check proved nothing: standard signals sent together
  coalesce into one delivery. That path is covered by the stuck-subsystem unit test instead.

**Still open:** VehicleInterface (Phase 3's hardware half) must call `setFinalCommandHook` and
`registerCrashFrame`, and unregister the frame before closing the port.

## 2026-09-24 — Phase 3 (software part): CI unit job repaired; architecture note on fan-out buses

**Attempted:** Get CI's `host-unit-tests` job green. It has been red since Phase 0, failing at
`apt-get install libosrm-dev`.

**Built/changed:** `.github/workflows/build.yml` drops `libosrm-dev`, which does not exist on
Ubuntu, and adds `flatbuffers-compiler`, `libflatbuffers-dev` and `libyaml-cpp-dev`.
`docs/architecture/system-architecture.md` now states that a bus with several consumers is one
`RingBuffer` per consumer.

**Reasoning:** Nothing in the unit suite needs OSRM until Phase 8's `test_map_matcher.cpp`, which
will have to choose how CI obtains OSRM. On Ubuntu, `flatc` and the FlatBuffers headers come from
one package set, so the version guard passes there. The fan-out note matters because
`frameBus`/`detectionBus` each feed several threads in Part 5.1's table, and sharing one SPSC ring
between two readers would be a data race.

**Still open:** Confirm the job actually goes green after the push.

## 2026-09-24 — Fix: clang-format disagreement between local (22) and CI (18)

**Attempted:** Confirm CI after pushing the Phase 3 software work.

**Built/changed:** `struct sigaction sa{}` became `struct sigaction sa = {}` in `SystemManager.cpp`,
and a one-statement scope block in `test_event_log.cpp` became a temporary `EventLog{path};`.

**Reasoning:** `host-unit-tests` went green, its first green run since Phase 0, which confirms the
CI dependency fix. `format-check` failed on three lines that pass locally: this machine has
clang-format 22.1.0, and CI's `ubuntu-latest` installs 18. The two versions disagree about the
spacing in `sa{}` and about collapsing a one-statement block onto a single line. The CI version
(18.1.8) was reproduced exactly in a scratch venv (`pip install "clang-format>=18,<19"`), and the
three CI errors matched. Both constructs were rewritten into forms the two versions format
identically, rather than reformatting with one version and breaking the other.

**Verification:** Both clang-format 22 and 18 report 0 violations across all tracked
firmware/host sources. 51/51 unit tests; the host binary builds.

**Still open:** The two versions can diverge again on any new code. Checking with the scratch
clang-format 18 before pushing avoids surprises until CI's formatter is pinned. `ubuntu-latest`
moves to Ubuntu 26 on 2026-10-19 per GitHub's notice, which will change CI's version again.

## 2026-09-24 — Phase 5: model selection, weights, and the ONNX/engine scripts (Part 7.1–7.3)

**Attempted:** Obtain pretrained weights for the three Part 7.1 models and write the export and
engine-build scripts.

**Built/changed:** `host/scripts/export_onnx.py` (`--model yolov8m|ufld|midas`). Each export is
verified against PyTorch through ONNX Runtime before it counts as a success.
`host/scripts/build_tensorrt_engines.sh` runs `trtexec --fp16` per model and keeps logs and
benchmark numbers in `models/engines/*.log`. Weights were downloaded to the newly gitignored
`host/models/weights/`: `yolov8m.pt` (ultralytics release) and `culane_res18.pth` (UFLDv2 release,
Google Drive). The UFLDv2 source was cloned to `~/src/Ultra-Fast-Lane-Detection-v2`. The export
venv `~/src/ar-export-env` layers over `~/ml-env` via a `.pth` path file and adds only timm 0.9.16,
gdown and onnxslim, leaving `~/ml-env` unchanged.

**Reasoning:** Part 7.1's input sizes do not match the models it names, which was checked against
the upstream sources rather than assumed:
- UFLDv2 ships only 320×800 (TuSimple) and 320×1600 (CULane/CurveLanes). 800×288 is UFLD **v1**.
  I chose CULane ResNet-18 at 320×1600, because CULane (urban, night, crowded, faded markings) is
  nearer to Nairobi roads than TuSimple's clear US highways.
- No "MiDaS v3.1 Small at 384×384" exists. v3.1's small SwinV2-T model fails to load under
  timm ≥ 0.7 (layer restructuring: size mismatches in every downsample layer). timm 0.6.x, the
  version MiDaS pins, does not import on Python 3.13. MiDaS **v2.1** Small (256×256) is used. Depth
  is relative, and metric near-field depth comes from the LiDAR.
- YOLOv8m is COCO-pretrained, so Part 7.1's five classes will be a mapping from COCO classes in
  post-processing. COCO has no generic "obstacle" class.

**Problems hit:**
- timm 0.6.12 raises a dataclass error on import under Python 3.13, and MiDaS's hub file imports
  timm even for models that don't use it.
- MiDaS small pulls a second torch-hub repository for its backbone, which prompted for trust
  interactively. It was added to `~/.cache/torch/hub/trusted_list`.
- **Running the exports was stopped by the permission classifier.** Loading the downloaded UFLD
  checkpoint with `torch.load(weights_only=False)` executes pickled code from an external file.
  The script now uses `weights_only=True` for UFLD. YOLOv8m's load inside ultralytics still
  unpickles, and MiDaS runs torch-hub repository code. No export has been run, pending Ian's
  decision.

**Still open:** Run the three exports and the engine build once permitted, then check the
`trtexec` latencies against Part 7.1's FPS targets.

## 2026-09-24 — Phase 5: `TrtEngine` (Part 7.4)

**Attempted:** Implement the per-model TensorRT wrapper against the installed TensorRT 10.16.

**Built/changed:** `inference/TrtEngine.h` and `src/inference/TrtEngine.cpp`: `load`, `enqueue`,
`sync`, `inferBlocking`, `deviceInput`, `hostOutput`, `input` and `outputs`. The new
`host/test/integration/test_trt_engine.cpp` has 7 GPU tests, ctest label `gpu`, and is not in CI.
`enable_testing()` was added to the top-level and host CMake files so ctest discovers the
integration tests.

**Reasoning:** TensorRT 10 has no binding indices or `enqueueV2`, the API Part 7.4's snippet
assumes. Tensors are found by name and given device addresses once at load with
`setTensorAddress`, and inference is `enqueueV3`.

Inference is split into `enqueue()` and `sync()`, so that MlInferenceEngine can queue all three
models before waiting on any. That is what lets the Part 7.4 streams overlap. Each engine gets a
*non-blocking* stream, so that any stray default-stream call cannot re-serialise them.

Outputs are copied into pinned host memory, so the copy is truly asynchronous. The input is a
device pointer that pre-processing writes into on the same stream, so frames never round-trip
through host memory.

The engine is rejected at load if it has a dynamic shape, non-FP32 I/O, or more than one input.
All three are signs that the wrong engine was loaded, and each would otherwise corrupt a buffer
size silently.

**Problems hit:** The `cudaMallocHost(float**)` overload lives in `cuda_runtime.h`, not the C
API header that was included. Fixed by allocating into `void*`.

**Verification:** The test needs no downloaded model. It builds its own two-output network
(`y = 2x+1`, `r = relu(x)`) with TensorRT's builder API, serialises it, and loads it through
`TrtEngine`. 7/7 pass on the RTX 5060:
- every output element is exact;
- repeated runs see their own input;
- two engines on separate streams, both enqueued before either is synced, are both correct;
- wrong-sized input throws;
- missing and garbage engine files throw.
Formatting was checked clean with clang-format 22 and 18.

**Still open:** Pre-processing into `deviceInput()` and the per-model post-processing come next.

## 2026-09-24 — Phase 5: model resolution chosen, exports run, TensorRT engines built (Part 7.3)

**Attempted:** Ian permitted running the exports and asked for the highest useful resolution per
model, one that works for both distant and very close objects.

**Built/changed:**
- `export_onnx.py` gained `--width/--height` for yolov8m and midas, and refuses them for ufld.
- Exports at the chosen sizes:
  - `yolov8m.onnx` at 1280×736;
  - `ufld.onnx` at 1600×320 (fixed by the model);
  - `midas.onnx` at 448×256.
- `build_tensorrt_engines.sh` builds all three FP16 engines, and now checks each engine's input
  shape against the expected one.
- `test_trt_engine.cpp` gained a parameterised test over the real engines. It skips when they are
  not built on the machine.

**Reasoning (resolution):**
- **YOLO only:** it is the only model where resolution is a free choice.
- **Far objects:** with an assumed ~70° horizontal FOV (to be replaced by Phase 4's measured
  value), a 1.7 m pedestrian is ≈ 1.39·W/d pixels tall at distance d. At the ~12 px YOLO needs,
  W = 1280 detects out to ~148 m. That covers ~100 km/h stopping distance (~98 m) plus ~1 s of
  tracking history.
- **Close objects:** YOLOv8 trained at 640 with up to 1.5× scale augmentation, so it has seen
  objects up to ~960 px. At 1280 a car 3 m away is ~600 px, inside that range. At 1920 and above,
  close objects exceed it, and close range is what braking depends on.
- **Shape:** 16:9 at 1280×736 (720 rounded up to a multiple of 32) spends no compute on letterbox
  padding.
- **UFLDv2:** cannot change size. Its fully-connected head's input is H/32 × W/32 × 8.
- **MiDaS:** 448×256 at the frame's 16:9 shape. *Correction, same day:* this was first
  described as MiDaS's "native 256 short side". Reading MiDaS's own small-model transform
  (`resize_method="upper_bound"`, 256) showed it caps the *long* side at 256, i.e. 256×128 for
  16:9. 448×256 is therefore ~1.75× its standard scale, a moderate increase for sharper depth
  edges. It is kept moderate because MiDaS far past its training scale loses global consistency.

**Problems hit:**
- *UFLD export failed:* its model file imports `utils.common`, which imports NVIDIA DALI and other
  training-only packages. A stand-in `utils.common` with a no-op `initialize_weights` is
  registered instead. That is safe because every weight is overwritten by the checkpoint, and
  `strict=True` proves nothing is left at its initial value. The config file is read with `runpy`,
  avoiding the `addict` dependency.
- *First engine build failed:* TensorRT rejects Part 7.3's `--shapes` flag for a fully static
  model. The flag was dropped and replaced by a post-build shape check.
- *That check falsely failed at first:* trtexec capitalises "Input binding". The match is now
  case-insensitive.

**Verification:**
- *Exports:* each matches PyTorch through ONNX Runtime. Max |Δ|: YOLO 1.1e-3 on scores, UFLD
  ≤ 8.2e-6 on all four outputs, MiDaS 8.5e-4. The UFLD checkpoint loaded with `strict=True` and
  `weights_only=True`.
- *Engines, median GPU compute each in isolation (RTX 5060 laptop):*

  | Model | Input | Median | Throughput |
  |---|---|---|---|
  | YOLOv8m | 1280×736 | 6.58 ms | 133/s |
  | UFLDv2 | 1600×320 | 2.51 ms | 344/s |
  | MiDaS | 448×256 | 1.57 ms | 556/s |

  All three are well above Part 7.1's targets (30+/30+/15–20), and even run in sequence they total
  ~10.7 ms of a 33 ms frame.
- *YOLO resolution sweep (benchmark-only builds, not kept), median:* 640×384 2.77 ms, 640×640
  3.07, 960×544 3.76, **1280×736 6.58**, 1920×1088 13.97, 2560×1440 26.45 ms. Camera-native would
  use ~80% of the frame budget on detection alone.
- *Engines in C++:* all three load in `TrtEngine` with exactly the expected tensor names and
  shapes, and produce finite outputs on a grey frame. GPU tests 10/10. Both clang-format versions
  are clean.

**Still open:**
- Isolated latencies are not the exit criterion. Part 7.1's FPS must be met on a live feed with
  pre-processing, three concurrent models and display, on a laptop GPU that throttles when hot.
- The 70° FOV assumption must be replaced by the real camera's value once Phase 4 measures it. If
  it is much wider, re-run the range arithmetic.

## 2026-09-24 — Phase 5: output decoding (YOLO boxes, UFLD lanes)

**Attempted:** Turn raw engine outputs into boxes and lane points in source-frame pixels.

**Built/changed:** `inference/Postprocess.h` and `src/inference/Postprocess.cpp` contain
`mapCocoClass`, `InputMapping` (with `letterbox`), `decodeYolo` (with class-aware NMS), `iou` and
`decodeUfld`. `test_postprocess.cpp` has 18 tests and sits in the GPU-free unit suite, so CI runs
it.

**Reasoning:**
- **No OpenCV/CUDA dependency,** so the logic every later decision depends on is checked in CI.
- **COCO → Part 7.1 class mapping:**
  - person → Pedestrian;
  - bicycle and motorcycle → Cyclist (a two-wheeler with an exposed rider, i.e. boda-boda);
  - car, bus, truck → Vehicle;
  - traffic light, stop sign → Sign;
  - large animals (dog, horse, sheep, cow, elephant, bear, zebra, giraffe) → Obstacle. COCO has
    no generic obstacle class, and livestock and wildlife are real hazards on Kenyan roads.
- **Classes are mapped *before* NMS,** so a vehicle scored as both car and truck becomes one box.
- **Safety rule: post-processing never removes a pedestrian box because it overlaps another
  class.** The common "rider suppression" trick (drop a person box overlapping a two-wheeler)
  would also drop a real person standing beside one, and a missed pedestrian is the unsafe
  direction.
- **`decodeUfld` ports UFLDv2's `pred2coords`:** row anchors for the ego-lane boundaries, column
  anchors for the outer lanes, the > ½ and > ¼ presence rules, and a soft-argmax over ±1 cell.
  Its softmax subtracts the max logit first (overflow-safe), and it keeps float coordinates where
  upstream truncates to int.

**Verification:** 18/18, and 69/69 across the unit suite. The expected values are hand-computed:
- the production letterbox (2560×1440 → 1280×720 + 8 px bands) and the 1080p fallback;
- decoding into source pixels, thresholding, unmapped-class handling, NMS, car+truck merge,
  clipping into padding, and `maxDetections`;
- the pedestrian-never-suppressed rule;
- lane anchor rows, both presence thresholds at their exact boundaries, soft-argmax refinement
  towards a strong neighbour, and finiteness with 500-valued logits.

**Still open:** Cross-check both decoders against the upstream Python on *real* model output for
a real road frame (needs footage).

## 2026-09-24 — Phase 5: GPU pre-processing

**Attempted:** Convert a GPU camera frame into each engine's input tensor, on that engine's
stream.

**Built/changed:** `inference/Preprocess.h` and `src/inference/Preprocess.cpp`: `PreprocessSpec`
(`yolo`, `imagenet`) and `Preprocessor::run`. `host/test/integration/test_preprocess.cpp` has 5
GPU tests.

**Reasoning:** The pipeline is region → resize (letterbox for YOLO, pad 114 as in its training)
→ BGR→RGB → /255 → per-channel normalise (ImageNet stats for UFLD/MiDaS, none for YOLO) → HWC to
CHW. The last step is `cuda::split` into three GpuMat headers laid over the engine's input
buffer, so there is no extra copy. Every call is on the caller's stream, so the three models'
pre-processing overlaps like their inference. Scratch buffers are allocated once per model.

The lane model will receive a CULane-shaped band (2.78:1) around the horizon rather than the
whole frame squashed. It was trained on that shape, and squashing 16:9 into it distorts road
geometry by ~1.7× vertically. The band's position will be configuration, because it depends on
the final camera mount.

**Verification:** 5/5 GPU tests with hand-computable expectations:
- a pure-red frame gives exactly `(1−.485)/.229`, `−.456/.224`, `−.406/.225` in the R, G, B
  planes;
- YOLO's 8 padding rows each side are exactly 114/255, with the content exactly 1.0;
- a white square lands where the returned mapping predicts;
- an ROI excludes the rest of the frame and offsets the mapping;
- a bad ROI and a wrong pixel type throw.

Negative test: with BGR→RGB removed, the channel test fails. Restored and re-verified. (The first
two attempts at this negative test printed nothing: my grep pattern had one space where
GoogleTest prints two, in `FAILED  ]`. The failure was confirmed from unfiltered output.)

**Still open:** The Phase 4 camera is not bought yet, so real frames for the band placement and
the live check must come from recorded footage.

## 2026-09-24 — Phase 5: `MlInferenceEngine`, debug viewer, and a first run on real footage

**Attempted:** Assemble the inference thread and check its output on real road video, since the
Phase 4 camera does not exist yet.

**Built/changed:**
- `camera/CameraFrame.h`.
- `inference/MlInferenceEngine.h` and `src/inference/MlInferenceEngine.cpp`: `process()`, `run()`
  (SystemManager contract) and `laneBand()`. It publishes a `DetectionFrame` on the detection bus
  and a `DepthFrame` on its own bus, paired through `depth_map_ref` = frame sequence number.
- `host/tools/inference_viewer.cpp`, a new CMake target outside `src/` so its `main` never enters
  the app. It draws boxes, lanes, the lane band and a depth inset, reports latency percentiles, and
  takes `--band-top`, `--snapshots`, `--out` and `--dump`.
- `Preprocessor::setRoi`.
- Test footage in `data/footage/` (gitignored), with source and CC BY 3.0 attribution in
  `data/README.md`.

**Reasoning:**
- **Upload ordering:** the frame is uploaded once. Each model stream waits on a CUDA event recorded
  after the upload (`cudaStreamWaitEvent`), so the ordering is enforced on the GPU without
  blocking the CPU.
- **Sizes from the engines:** pre-processing sizes come from each engine's input shape, so a
  rebuilt engine at another resolution needs no code change.
- **UFLD outputs by name,** because TensorRT does not guarantee declaration order.
- **Lane band:** UFLD sees the bottom 60% (upstream `crop_ratio`) of a CULane-shaped band. The
  decoder maps against the full band, as upstream does.
- **Footage:** "City Driving 4K – Kraków Poland 2024" (Relaxing Roads 4K, CC BY 3.0). It is
  2560×1440, exactly the planned camera resolution. Only the first 120 s was fetched, in one
  contiguous range request.

**Problems hit:**
- *Clip download:* seeking inside the remote WebM made many range requests and Wikimedia returned
  HTTP 429. It was replaced by a single range request for the first 200 MB, cut locally with
  `ffmpeg -c copy`.
- *Privacy slip, mine:* the earlier `ffprobe` of that URL sent Ian's email address in its
  User-Agent. Wikimedia's policy asks for contact details, but Ian had not agreed to share his
  email with any service. All later requests used a User-Agent without it.
- *Lane band placement:* the default band (top at 0.25) sat too high for this footage (horizon
  ~58% down) and cut off the nearest ~100 px of road. The band top was changed to 0.32, and the
  ego-left lane then followed the double white line down to the bonnet. The 0.32 default fits
  this footage's camera mount, not the project's; Part 12 must re-tune it.
- *Viewer crash:* the viewer aborted with an uncaught exception when run outside the repository
  root, because engine paths are relative. It now reports the error and exits 1, verified.

**Verification:**
- *Timing, 600 frames, 2560×1440, first 10 excluded as warm-up:*

  | Measure | Median | p95 | Max |
  |---|---|---|---|
  | Upload → all three models synced | 10.5 ms | 14.1 ms | 18.4 ms |
  | Including decode | 12.8 ms | 16.6 ms | 20.9 ms |

  That is ~78 FPS at the median. The worst frame is still inside 33 ms, so Part 7.1's 30+/30+/15–20
  targets are met with all three models running concurrently.
- *Visual, frames 0/120/240/360/480:*
  - boxes on parked and moving cars, a van, and a pedestrian crossing ahead (0.87);
  - a distant pedestrian ~48 px tall detected at 0.71;
  - ego-left lane on the double white line;
  - no ego lanes at an unmarked junction, which is correct;
  - depth brightest on the road nearest the car.

  The ego-right lane followed a faint worn marking (plausible, unconfirmed), and the outer-right
  lane was noisy.
- *Independent cross-check:* ultralytics' reference pipeline (its own letterbox + PyTorch FP32 +
  its NMS) on the identical frames, with the same class map:

  | Frame | Ours | Reference | IoU | Confidence difference |
  |---|---|---|---|---|
  | 120 | 5 | 5, all matched | 0.976–0.997 | ≤ 0.03 |
  | 360 | 3 | 3, all matched | 0.964–0.989 | ≤ 0.05 |

  No missing or extra boxes. This validates GPU pre-processing, the TensorRT FP16 engine, the
  decoder and NMS together.
- Unit 69/69, GPU 15/15, and both clang-format versions clean.

**Still open (Phase 5 is NOT complete):**
- Part 14's exit criterion requires a **live camera feed** at target FPS. Recorded footage is not
  a substitute; this needs the Phase 4 camera.
- The UFLD decoder is validated by unit tests and visually (points lie on real markings), but not
  numerically against upstream `pred2coords` on real output.
- The footage is European. Accuracy on Kenyan roads (boda-bodas, matatus, unmarked roads,
  livestock) is untested until the project's own recordings exist.
- Frame upload uses pageable memory. Pinned host memory (`cv::cuda::HostMem`) would make it
  asynchronous if profiling shows the upload matters.

## 2026-09-24 — Phase 5: full annotated run of the test footage, for review

**Attempted:** Ian asked to see the models running on real road footage.

**Built/changed:** `inference_viewer` gained dark backing panels behind its text (it was
unreadable over bright sky), a class/lane legend, and the CC BY 3.0 attribution line the footage
licence requires. It now writes video at the source frame rate, so playback is real-time. The
output is `data/footage/annotated/krakow_annotated.mp4` (120 s, 1280×720, 60 fps, H.264, 133 MB,
gitignored).

**Verification:**
- *Timing, all 7,200 frames at 2560×1440:* median 13.05 ms, p95 16.6 ms, max 25.1 ms. The worst
  frame in two minutes is inside the 33 ms budget. Mean 9.7 boxes per frame. Ego lanes present in
  ~78–79% of frames.
- *Spot-checked frame at 38 s:* stop sign 0.90 and traffic light 0.30. Three pedestrians on the
  pavement, including a child at 0.29, a low-confidence detection of exactly the kind the
  never-suppress rule protects. Ego lane on the kerb and road edge.
- *One clear false positive:* a large "vehicle" box on a building wall at the left frame edge.
  Single-frame false positives like this are what Phase 7's tracker (a detection must persist
  across frames to become a track) is meant to filter.

**Still open:** Ian's review of the full video.

## 2026-09-24 — Road-sign detection extension: scope, overnight training launched, AR behaviours

**Attempted:** Ian found the detector weak on road signs, and chose a dedicated fourth model
trained on the Mapillary Traffic Sign Dataset (MTSD). He also defined the purpose of signs:
understand each sign and show the driver spatially, racing-game style (a holographic stop
barrier, barriers across forbidden turns, the speed limit against actual speed with an F1-style
colour-graded route line), for navigation and safety.

**Built/changed:** `docs/architecture/sign-behaviours.md` gives per-class AR behaviours for all 29
detector classes, 7 cross-cutting rules, and the phase each capability lands in. An overnight
background agent was launched to obtain MTSD, train the model, export and benchmark it, and
evaluate it on openly licensed Kenyan imagery. Its own progress-log entries and report
(`docs/experiments/2026-09-25-sign-detector-overnight.md`) follow.

**Reasoning:**
- **Root cause is vocabulary, not resolution:** COCO has only "stop sign" and "traffic light".
  That rules out threshold or resolution fixes.
- **A separate small model (YOLOv8s)** rather than retraining YOLOv8m keeps the safety-critical
  detector untouched.
- **Speed limits are one class per value,** so the detector reads the number without a second
  classifier.
- **No horizontal flips** in training (they swap left/right meanings).
- **Kenya is left-hand traffic,** which decides which lane boundary "no overtaking" marks and which
  side signs appear on.
- **The overriding rule, kept from Part 9:** signs never actuate. A wrong sign can at worst draw a
  wrong graphic.

**Problems hit:** MTSD needs Ian's own Mapillary login. The download links are emailed and valid
for 5 days, so no unattended process could fetch it without him. Ian chose to let the agent read
**only** the `from:mapillary` email in his Gmail to take the links. At launch the email had not
arrived yet; Ian was asked to request the dataset. The one public mirror checked (Hugging Face) was
a 2,000-image shape-only subset with no licence, so it is unusable. The agent was told not to use
YouTube for Kenyan footage (licence and ToS) and to use KartaView (CC BY-SA 4.0) and Wikimedia
Commons instead.

**Still open:**
- The overnight results.
- Whether ~10 h of wall time allows both the ~42 GB download and meaningful training depends on
  bandwidth.
- The behaviours are design only; implementation lands in Phases 7–11.

## 2026-09-24 — Sign detector: MTSD label mapping and YOLO conversion

**Attempted:** Turn the Mapillary Traffic Sign Dataset (fully annotated part) into a YOLO dataset
with the 29 target classes (stop ... other_regulatory), as the first step of a fourth,
sign-specific detector.

**Built/changed:**
- `host/scripts/prepare_mtsd.py`: reads the real label list (401 labels, 206,386 boxes over
  41,909 annotation files), maps every label, writes `host/scripts/mtsd_class_map.json`, converts
  train/val to `data/datasets/mtsd_yolo/` (long side capped at 2048 px, boxes rescaled through
  normalised coordinates) and writes `mtsd.yaml` and `instance_counts.json`. It is re-runnable:
  images already re-saved are not re-encoded, but labels are always rewritten.
- `host/scripts/mtsd_uncertain_unit_speed_variants.json`: speed-limit designs dropped as mph or
  unclear.
- `host/scripts/extract_growing_zip.py`: extracts complete entries of a zip that is still
  downloading (MTSD stores its JPEGs uncompressed, sizes in each local header, CRC checked), so
  conversion can overlap the slow download.
- `.gitignore`: `host/models/training/` and `data/datasets/`.

**Reasoning:** The mapping was written only after reading the real labels and looking at crops of
every mapped design variant (e.g. the MTSD names are `regulatory--yield`, `warning--road-bump`,
not "give-way"/"speed-bump"). The conservative choices are in docs/decisions.md.

**Verification:** 10 random converted train images drawn with their boxes and looked at (visual
check, not measured): every box sits on a sign and the class names match (stop, give_way,
no_entry, speed_limit_50/60, pedestrian_crossing on a yellow school-crossing diamond,
keep_left_or_right on blue keep-right discs, no_parking on the P-with-slash disc). No conversion
bug found. Val zip MD5 matches Mapillary's list.

**Problems hit:** On the partial data (6,314 train + 5,320 val images) many classes are small:
speed_limit_10 (13 train boxes), 110 (15), 120 (21), end_of_restriction (36), 90 (38) and 20 (39).
The full train set is ~5.8x larger.

**Still open:** train.0/1/2 still downloading (~2.2 MB/s aggregate). Final per-class counts come
from the final conversion run.

## 2026-09-25 — Sign detector: training data ready, training started (train.0 + val)

**Attempted:** Download the fully annotated MTSD parts and start training on what the time budget
allows.

**Built/changed:**
- Downloaded and MD5-verified against Mapillary's own list: annotations (121 MB), val (4.57 GB,
  5,320 images) and train.0 (10.38 GB, 12,197 images; the zip's entry count matches the files
  extracted). train.1 and train.2 (10.4 GB each) are still downloading in the background for a
  later run. Test (no public labels) and the partially annotated set (not fully labelled) were not
  downloaded.
- Final conversion: 12,692 train images (train.0 plus ~500 that train.1/2 had delivered while
  paused) and 5,320 val images; 391 panoramas skipped. Per-class train/val box counts are in
  `data/datasets/mtsd_yolo/instance_counts.json`. Smallest: speed_limit_10 23/12, 110 29/14,
  120 38/14, end_of_restriction 68/37, speed_limit_20 72/28, 90 77/27, 100 89/44.
- `host/scripts/train_sign_detector.py` started at 00:35 EAT: YOLOv8s from COCO `yolov8s.pt`,
  imgsz 1280, batch 8, fliplr 0, flipud 0, amp, 8 workers, `time` = 4.94 h (ends by 05:30).

**Reasoning:** At the measured ~2.2 MB/s the full train set would have arrived around 03:10,
leaving about 2 h of training. That is below the brief's 4 h threshold, so its fallback was used
(docs/decisions.md). A later request to extend training to 08:00 could not be applied in this
run, so the original deadline stands.

**Problems hit:** A first run on partial data (23:53) was stopped before its first epoch ended. It
left no weights (`host/models/training/signs_v1_partial_superseded/`). The overnight session was
also paused by a usage limit around 23:00–23:50; the downloads kept running.

**Still open:** Training results; train.1/2 are for a retrain on the full ~36.6k training images.

## 2026-09-25 — Sign detector: Kenyan evaluation imagery collected

**Attempted:** Collect openly licensed real Kenyan road imagery with signs, to evaluate the sign
detector on the target domain.

**Built/changed:** `host/scripts/fetch_kenya_imagery.py` (KartaView, CC BY-SA 4.0) and
`host/scripts/fetch_commons_kenya.py` (Wikimedia Commons, licence read from Commons metadata,
only CC0/CC BY/CC BY-SA kept). 300 images in `data/footage/kenya/` (199 KartaView from Nairobi and
the A104, 101 Commons), with per-file author/licence manifests. Sources and attribution recorded
in `data/README.md`.

**Reasoning:** Both sources have explicit open licences and public APIs. Requests were serial,
with pauses, back-off on 429/5xx and a User-Agent without personal data.

**Problems hit:** KartaView's v2 bounding-box sequence search answers "Restricted access"; the v1
list endpoint works but returns only ~15 sequences for Nairobi and none for Mombasa, Eldoret,
Kisumu or Thika town. One KartaView storage host returned HTTP 500 for over an hour. Commons began
answering HTTP 429 after ~100 files (the thumbnail width was not a standard size; now fixed to
1280). No Thika Road / Mombasa Road / Waiyaki Way sequences specifically were found.

**Still open:** The set is dominated by one contributor and many frames have no sign. Ian's own
dashcam recordings will be the real test set.

## 2026-09-25 — Sign detector: overnight run reviewed; full-data retrain until no improvement

**Attempted:** Review what the overnight agent completed, then carry out Ian's instruction to keep
training "until its confidence cannot be improved anymore".

**Built/changed:**
- `host/scripts/train_sign_detector.py` gained `--name`, `--epochs`, `--patience` and
  `--end none`. The defaults keep the old behaviour.
- A detached pipeline, `host/models/training/run_signs_v2.sh` (gitignored, log beside it in
  `signs_v2_pipeline.log`):
  - re-extracts train.1 and train.2;
  - verifies every image in all four zips is on disk at full size;
  - converts, skipping already-converted images;
  - trains `signs_v2`, warm-started from `signs_v1/weights/best.pt`, on all ~36.6k training
    images, with `patience=15` and `epochs=60`.

**Reasoning:**
- *What happened overnight:* the agent was stopped twice by usage limits (~23:00 and ~04:50). It
  never received the 08:00 extension. `signs_v1` trained on train.0 only (12,411 images) for
  22 epochs within its 4.94 h budget.
- *Its curve shows it was still improving:* mAP50 0.16 → 0.39 (ep 10) → 0.533 (ep 22); mAP50-95
  0.114 → 0.418. More data and more epochs should help.
- *"Cannot be improved anymore"* is implemented as standard early stopping. Training ends once
  validation fitness (0.1·mAP50 + 0.9·mAP50-95) has not improved for 15 consecutive epochs.
  `epochs=60` is a safety cap, and it also sets the learning-rate decay length. At ~31 min per
  epoch on 36.6k images (extrapolated from v1's ~10.7 min on 12.4k), the cap is ~31 h.
- *Warm start from v1:* it already knows the sign classes, so it converges faster than starting
  from COCO.
- *Detached (`setsid nohup`):* a session or usage-limit cutoff can no longer stop the run. This is
  what cost the night twice.

**Problems hit:** The first extraction of train.1/2 was cut off with the session. The images on
disk (36,314) were short of the ~41.9k expected, and a half-written JPEG would have been skipped
by unzip's no-overwrite mode. The pipeline re-extracts with overwrite and then verifies sizes
against the zip listings before converting.

**Still open:**
- The v2 results.
- v1 was never exported or evaluated on the Kenyan imagery (the agent stopped first). Evaluation
  will be done once, on the final model.
- The overnight report `docs/experiments/2026-09-25-sign-detector-overnight.md` was never
  written.

## 2026-09-25 — Guide amendment: AR overlays rendered on the GPU (Part 10)

**Attempted:** At Ian's instruction, switch the guide's overlay rendering from OpenCV CPU drawing
to GPU work, so the holographic sign behaviours can be drawn at the full 2K resolution. He also
asked what the box overlays are for.

**Built/changed:** `docs/BUILD_GUIDE.md` Part 10.1–10.4 rewritten, and the Phase 11 deliverables
in Part 14 updated:
- `CompositedFrame` carries video + an `OverlayScene` draw list;
- `OverlayItem` geometry is either image-space or road-space (vehicle-frame metres, projected in
  the vertex shader);
- `WindowedSink` draws the video texture (PBO upload) and then an overlay shader pass, at native
  display resolution, with no OS scaling;
- `ArRenderer` builds the scene (`addHazardHighlights`, `addLaneOverlay`, `addNavigationOverlay`,
  `addSignBehaviours`, `addSpeedAndWarnings`) and draws nothing itself;
- the 10.4 benchmark now times the upload and the overlay pass separately, and its native-Windows
  fallback sends video + `OverlayScene` over TCP.

**Reasoning:**
- **Why the GPU:** the translucency, glow, gradients and pulsing are per-pixel blending of
  several 2K layers per frame. That is trivial on the GPU and a real cost on the CPU.
- **Why a draw list instead of composited pixels:** a future optical `ProjectorSink` must draw
  overlays *without* the video, and pre-composited pixels could never support that.
- **Road-locked items:** projecting them with the same calibration as perception means they land
  where the detections are.
- **Checked display facts:** the development laptop's panel is 2560×1600 @ 240 Hz at 150% Windows
  scaling, so a 2560×1440 frame maps 1:1 if OS scaling is avoided. Otherwise it is resampled from
  1707 px and blurred.
- **What the boxes are for (answer to Ian):** they show the driver what the system has noticed
  and judged a hazard (Part 9.2: tailgating, erratic speed, swerving, forward-collision risk), so
  a warning or brake never arrives unexplained. They are also report evidence (Part 16). In the
  driver view they become threat-styled hazard highlights on relevant objects only; the
  every-detection boxes stay in the debug viewer.

**Still open:**
- The report's description of the `DisplaySink` interface must be updated to match.
- Whether CUDA/GL interop is available under WSLg (not assumed) will be settled by the 10.4
  benchmark.

## 2026-09-25 — AR overlay design: hazards join the sign language; guide amended

**Attempted:** Ian's design for hazards. Instead of boxes, a dangerous car, pedestrian, animal or
obstacle is shown with the same road-placed elements as signs, plus a subtle shimmering glow on
the object whose colour follows its risk. Tailgating is shown as a following-distance zone (my
proposal, agreed). He asked for the documentation and guide edits now, and the model work after
the sign training.

**Built/changed:**
- `docs/architecture/ar-overlay-design.md` replaces `sign-behaviours.md`:
  - §0: a shared visual-language table;
  - §1: 8 rules for all overlays. Rule 4 is now "never hide a hazard"; rule 8, "ordinary traffic
    gets nothing", is new;
  - §2: the sign behaviours (unchanged, except that the crossing escalation now uses hazards);
  - §3: hazards, with risk level `r`, the object glow, and forward-collision, pedestrian/animal,
    tailgating, swerving-neighbour and obstacle behaviours;
  - §4: a capability/phase table, adding YOLOv8m-seg masks and `r`.
- `docs/BUILD_GUIDE.md`:
  - Part 3.5: `mask_ref` in `DetectionFrame`;
  - Part 7.1: table amended (YOLOv8m-seg; the sign-detector row; the real lane/depth models and
    sizes), with 7.3 superseded by the scripts;
  - Part 10.1: object-mask geometry, and rule-4 enforcement (rim glow; road items masked by
    silhouettes);
  - Part 10.3: `addHazards`, and the hazards-not-boxes specification;
  - Part 10.4: the benchmark scene now includes shimmering glows;
  - Part 14: Phase 11 deliverables.

**Reasoning:**
- *One visual language:* every element means the same thing, whether it comes from a sign or a
  hazard.
- *The glow follows the object's real outline,* which needs per-object masks. A box-shaped glow
  would reintroduce the rectangle Ian wants gone, so the detector moves to YOLOv8m-seg (same COCO
  classes and export path).
- *`r` comes from the same TTC and Part 9.2 measures the arbiter uses,* so the display and the
  brake decision agree. It never feeds actuation.
- *The shimmer is capped at ~3 Hz* to avoid strobing.
- *Road graphics are masked by silhouettes,* so they sit behind objects, as rule 4 requires.

**Still open:** the YOLOv8m-seg work, deferred until the sign training ends:
- export and engine;
- a mask decoder with unit tests;
- `detections.fbs` `mask_ref` and the mask bus;
- timing against the current 6.6 ms;
- confirming the box outputs match the current detector's behaviour.

The report's descriptions of the display interface and the model table must follow the guide.

## 2026-09-25 — Guide amendment: mask-based LiDAR fusion and startup extrinsic verification

**Attempted:** Two proposals from Ian, written into the guide:
- (1) match each object's YOLOv8m-seg mask with its LiDAR points;
- (2) have the system calibrate the LiDAR–camera mapping when it starts, for the most accurate
  mask/3D matching.

**Built/changed:** `docs/BUILD_GUIDE.md`:
- Part 8.3 rewritten: mask-based fusion, with 7 rules;
- new Part 12.2.1: `ExtrinsicMonitor`, with its states `UNVERIFIED`/`VERIFIED`/`REFINED`/
  `DEGRADED`;
- Part 13.1 gains `test_mask_lidar_fusion.cpp` and `test_extrinsic_monitor.cpp`;
- Part 14 Phase 6/7 deliverables and exit criteria include them.

Decisions logged.

**Reasoning:**
- *Masks over boxes:* masks give each object its own points, so range, ground-contact point,
  velocity and TTC are the object's own. This matters most for the thin or partly hidden
  objects that are hazards.
- *Each fusion rule answers a known failure mode:*
  - motion compensation, because the Livox pattern accumulates over ~100 ms;
  - mask erosion, because ¼-resolution masks are soft and extrinsic error shifts projections;
  - nearest depth mode, because a mean is dragged by stray points;
  - the parallax depth test, because the two sensors sit in different places.
- *The LiDAR-veto rule:* an unexplained cluster in the path is still an obstacle, so a camera
  miss can never hide a real object.
- *Startup calibration is verification + bounded refinement, not from-scratch:*
  - at startup the car is usually stationary in a poor scene, where a free solve can converge
    wrongly with confidence;
  - bounds, held-out checking, and rejection of solutions at the bound make a bad refinement
    detectable rather than silent;
  - in `DEGRADED`, braking relies on LiDAR-only ranges, which need no camera alignment. A wrong
    self-calibration can therefore never reach the brake, in keeping with the hard safety rule.

**Still open:** Implementation in Phases 6–7. The maths (projection, erosion, depth modes,
scoring, bounded search) can be built and unit-tested on synthetic data before the LiDAR exists.
The arbiter's LiDAR-only corridor mode belongs in Phase 10.

## 2026-09-25 — Guide amendment: motion prediction (Part 9.1)

**Attempted:** Ian proposed a model that collects points, derives their velocity vectors and
predicts their motion, and asked for the full mathematical and engineering treatment, then for it
to be written into the guide.

**Built/changed:**
- `docs/BUILD_GUIDE.md`:
  - Part 9.1 rewritten: `MultiObjectTracker` (IMM: CV / CTRV-UKF / stopping) and
    `MotionPredictor`;
  - new 9.1.1 (the limits of prediction), 9.1.2 (closest point of approach and collision
    probability), 9.1.3 (NIS, ADE/FDE) and 9.1.4 (scope);
  - Part 9.2 gains a tracker-based swerving definition plus sudden-braking and crossing/cut-in
    warnings;
  - Part 9.3 gains an explicit statement of what prediction may not do;
  - Part 10.3 gains predicted-path ribbons and latency compensation;
  - Part 13.1 gains `test_motion_predictor.cpp`;
  - Part 14 Phase 7 deliverables and exit criteria updated.
- `docs/architecture/ar-overlay-design.md`: §3.1 `r` includes collision probability; §3.4
  crossing barriers go at the predicted conflict point; new §3.8 (predicted paths, ghosts, latency
  compensation); §4 table.

**Reasoning:**
- *Object-level, not point-level:* the Livox scan is non-repetitive, so individual points have no
  successor, and per-point velocity is noise. Objects measured through their points (L-shape fits,
  ICP registration) give real motion. Centroids are rejected because visibility changes fake
  sideways velocity.
- *IMM:* road users switch behaviour. The stopping model's rising probability gives early warning
  of a lead vehicle braking, and matatus stopping without warning are a real case.
- *The limits are shown with numbers:* ~1.4 m/s raw velocity noise per frame at 10 Hz, and
  pedestrian position uncertainty ~0.35 m at 0.5 s, ~1.1 m at 1 s, ~4.1 m at 2 s. So predictions
  are displayed as regions, and ~1.5 s is the honest physics horizon.
- *CPA:* it covers crossing and cut-in cases, which TTC misses.
- *Braking stays on current measured range and closing speed:* forecast-based braking risks
  phantom stops and would make the brake rule unauditable. The IMM still changes the closing-speed
  estimate, so its error and lag are gated by a test before Phase 10.
- *Latency compensation:* ~40–60 ms is ~0.75 m for a car crossing at 15 m/s. Drawing at the
  predicted display-time position keeps graphics attached to moving objects.

**Still open:** Implementation in Phase 7. All the maths (UKF, IMM mixing, CPA, NIS) can be built
and unit-tested on synthetic trajectories before the LiDAR exists. The per-class noise must be
tuned on real recordings.

## 2026-09-25 — Sign detector: GPU fault at epoch 21 (power-source change); resumed with auto-retry

**Attempted:** A progress check found training stopped since 19:49.

**Built/changed:** `host/models/training/resume_signs_v2.sh` (gitignored) resumes `signs_v2` from
`last.pt` and retries up to 5 times after a crash, 60 s apart. It writes its PID to
`resume_signs_v2.pid`. Training resumed at 19:58 from epoch 21.

**Reasoning:**
- *Evidence:* validation after epoch 20 died with `torch.AcceleratorError: CUDA error: unknown
  error` at 19:49. Windows' System log shows two Kernel-Power "Power source change" events at
  19:48:37–38, one minute earlier. The likely cause is the laptop GPU's power-state transition when
  moving between mains and battery, under full load. It was not a training bug.
- *Nothing lost:* epoch 20 had completed and been saved; it is also the best so far (mAP50 0.684,
  mAP50-95 0.547, from 0.533 / 0.418 for v1).
- *Auto-retry:* a transient GPU fault costs at most the epoch in progress.

**Problems hit:** My liveness check `pgrep -f run_signs_v2.sh` matched the checking shell's own
command line, which contains that string. It kept reporting RUNNING after the pipeline had died.
The background watcher meant to trigger the YOLOv8m-seg work used the same check, so it would
never have fired. It was stopped and replaced by a watcher on the resume job's PID file.

**Still open:** Ultralytics' resume restores the epoch and optimiser state, but the early-stopping
window counts again from the resume point, which is more lenient by at most the epochs already
elapsed since the best. It is noted here in case the stop epoch looks late.

## 2026-09-26 — Sign detector: retry limit raised from 5 to 7

**Attempted:** Ian asked to raise the training job's crash-retry limit from 5 to 7. By then it
had used 3 attempts: crashes at 21:05 and 00:47, the same `CUDA error: unknown error`.

**Built/changed:**
- `host/models/training/resume_signs_v2.sh`: the attempt range is now configurable
  (`START_ATTEMPT`, `MAX_ATTEMPTS`, default 1..7). The new version was written to a temporary file
  and renamed over the old one.
- New `host/models/training/extend_retries.sh`: waits for the running job's PID to exit. If the log
  says it gave up, it continues with attempts 6–7. Both files are gitignored.

**Reasoning:** The live job's `for` loop was parsed when it started, so editing its script cannot
change its limit. Bash also reads scripts incrementally, so editing the file in place under a
running process can corrupt what it executes next. The atomic rename leaves the running process on
its already-open copy, and the extender adds the two extra attempts from outside it. The effective
limit is 7, and the live training was never touched.

**Still open:** None. Training was at epoch 45/60, best mAP50 0.7125 / mAP50-95 0.5696 (epoch 44).

## 2026-09-26 — Sign detector: signs_v2 training complete (60 epochs, full MTSD)

**Attempted:** Finish the full-data retrain until validation stopped improving (Ian's
instruction).

**Built/changed:** `host/models/training/signs_v2/weights/best.pt` (epoch 53) and `last.pt`
(epoch 60). The run finished normally at 16:05:54 on 2026-09-26, after 5 GPU-fault resumes
(epochs 21, 23, 30, 50, 51), all recovered automatically.
`host/scripts/show_results.py` prints `results.csv` as a table, marking the best epoch,
resumes and the mosaic-off epoch. `host/scripts/align_csv.py` was written and tested, then
unused, because Ian cancelled the in-place alignment of `results.csv`.

**Reasoning / results:**
- **Final validation of `best.pt`** (5,210 images, 6,330 signs): P 0.816, R 0.613,
  **mAP50 0.716, mAP50-95 0.572**, against v1's 0.533 / 0.418 (+34% / +37%).
- **The run ended at the 60-epoch cap, not by early stopping.** mAP50-95 had been flat within
  0.5713–0.5727 since ~epoch 47. Mosaic-off (epochs 51–60) gave no final gain. Training losses
  kept falling while validation losses stayed flat: converged, with the onset of overfitting.
- **Per class, strongest (mAP50):** stop 0.869, no_overtaking 0.866, road_hump 0.827,
  no_parking_or_stopping 0.827, pedestrian_crossing 0.818, give_way 0.811.
- **Weakest:** speed limits 10 (0.431), 20 (0.448), 120 (0.530), 60 (0.547), 80 (0.605), with
  low recall (speed_limit_60 R 0.361 despite 88 instances), plus no_left_turn 0.633.
- **The model misses more than it invents.** Precision is ≥ 0.75 for most classes, while
  recall is the limit (0.613 overall, flat since ~epoch 22). That points to data (rare classes;
  confusion between similar speed values), not training length.
- **Recall is per frame, not per sign.** A sign stays in view for many frames on approach, so
  the tracker's multi-frame confirmation sees it several times. Per-sign detection will be
  higher than 0.61, but that needs measuring on video, not assuming.
- **Speed:** ultralytics reports 8.0 ms inference per image at batch 1 in PyTorch. The
  TensorRT FP16 engine is measured next.

**Still open:** Export + engine + benchmark, the Kenyan evaluation, and the overnight report;
then the YOLOv8m-seg work.

## 2026-09-26 — Sign detector: known gaps and the improvement plan

**Attempted:** At Ian's request, record the sign model's gaps and how to close them, with evidence
from the final validation (per-class metrics, `signs_v2/confusion_matrix_normalized.png`,
`data/datasets/mtsd_yolo/instance_counts.json`).

**Gaps found:**

1. **Missed speed-limit signs (the largest gap).** In the confusion matrix, 40–55% of true speed
   limit 10/20/60/80/110/120 signs are predicted as *background*: not detected at all. Other
   speed limits lose ~25–35%. Confusion between values is small (~5–15%: 120→110, 10→70/30).
   - *Correction to my own earlier statement:* limit 60 (R 0.361) was attributed to confusing
     60/80/50. The matrix shows it is mostly missed, not misread.
   - *Also corrected:* training instances on the full data are 79 (10), 195 (20), 95 (110) and
     94 (120), not the 23–38 quoted from the train.0-only run. Limit 60 has 607, so scarcity
     alone cannot explain its recall.
2. **A cause we introduced ourselves: dropped boxes become negatives.**
   - `prepare_mtsd.py` removes boxes flagged `ambiguous` (68,560 across MTSD) and the
     uncertain-unit speed variants. Removing a label does not remove the sign from the image; it
     becomes *background*, so training teaches the model that small, distant or hard-to-read
     signs are "not a sign".
   - That fits the pattern: the most-missed classes are the ones most often small and far away.
   - The same applies to information/complementary signs dropped as background.
3. **False positives concentrate in the catch-all classes.** In the matrix's background column,
   other_warning (~0.4) and other_regulatory (~0.25) fire on non-signs. These are likely the
   dropped information/complementary signs, which look like signs.
4. **Recall is the limit overall.** Recall 0.613 with precision 0.816; recall flat since
   ~epoch 22, while more epochs only lowered training loss. It is a data problem, not a training
   length problem.
5. **Left/right asymmetry.** no_left_turn mAP50 0.633 vs no_right_turn 0.778. Horizontal flip is
   disabled (correctly: it swaps meanings), so each direction learns only from its own examples.
6. **Domain gap: no Kenyan training data.** MTSD is mostly Europe/Americas/Asia. Kenyan signs are
   UK-style, but faded, rusted, hand-painted, vegetation-covered and text-board signs
   ("BUMPS AHEAD", matatu stages) are unrepresented. Kenyan performance is unmeasured until the
   evaluation (next task).
7. **Evaluation gaps.** Recall is per frame. Per-sign recall over a video approach (with the
   tracker's multi-frame confirmation) is unmeasured, as is performance at night and in rain,
   and by sign pixel size.
8. **Resolution.** Images were capped at 2,048 px on the long side and trained at 1,280. Distant
   signs are a few pixels wide at that scale.

**Improvement plan, highest expected payoff first:**

1. **Stop teaching misses. Mask, don't drop.** In `prepare_mtsd.py`, paint ambiguous and excluded
   boxes out of the image (fill with the dataset mean grey), so they count as neither sign nor
   background, instead of deleting the label. Cheap: one retrain. It targets gap 1 at its
   likely cause.
2. **Kenyan data with active learning.** Record Kenyan drives. Run the model; queue for labelling
   the frames with low-confidence or flickering detections (where it is least sure). Label a few
   hundred in CVAT or Label Studio, and fine-tune. This targets gap 6 and gives a real Kenyan test
   set.
3. **Two-stage speed limits.**
   - The detector finds "a speed-limit sign" (one merged class, far more examples per class).
   - A small classifier reads the value from a crop taken from the full-resolution 2K frame.
   - Digits are much easier to read at native resolution than inside a 1280-wide detector input.
   - Targets gaps 1 and 8.
4. **Label-aware flip augmentation.**
   - Flip an image only when every sign in it is flip-safe, swapping labels as it flips:
     no_left↔no_right, keep-left↔keep-right. Symmetric signs are unchanged.
   - Images containing text or digits (speed limits, text boards) are never flipped.
   - This recovers the doubled data that plain flipping would give, without teaching wrong
     meanings. Targets gap 5.
5. **Class-balanced sampling.** Oversample images containing rare speed limits (10, 20, 110,
   120), or use loss weighting, so the two catch-all classes (≈ 21k of ~38k train instances)
   don't dominate.
6. **Train on information signs as an explicit class** (or mask them, as in 1), so the model
   stops calling them other_warning/other_regulatory. Targets gap 3.
7. **Higher effective resolution for small signs.** Train on sign-centred crops at native
   resolution, and/or run a second inference pass on a high-resolution crop of the sign-bearing
   region (upper half, verges) at run time. Targets gap 8.
8. **A larger model** (YOLOv8m, or a newer family) for signs, if items 1–7 plateau. The 33 ms
   budget has room; latency must be measured with all models running.
9. **Measure what matters.** Per-sign recall on video (did the system catch this sign at least
   once, in time?), recall by pixel size, day/night breakdown, and a Kenyan labelled test set.
   Per-frame mAP alone understates a tracked system and hides failure conditions.

**Mitigations already in the design** (so the gaps are not safety-critical):
- Signs never actuate (`docs/architecture/ar-overlay-design.md` rule 1).
- OSM `maxspeed`/stop/turn-restriction priors fill in missed signs (rule 7).
- Multi-frame confirmation suppresses one-frame false positives (rule 2).

## 2026-09-26 — Sign detector: export, TensorRT engine, Kenyan evaluation, report

**Attempted:** Finish the sign detector: export and benchmark it, evaluate it on real Kenyan
imagery, and write the report the overnight agent never wrote.

**Built/changed:**
- `host/models/onnx/signs.onnx`, verified against PyTorch (max |Δ| 9.8e-4, output 1×33×19320).
- `host/models/engines/signs.engine` (FP16), with all four engines rebuilt.
- `data/footage/kenya_annotated/` and `data/footage/krakow_signs/` (annotated images + CSVs).
- The report `docs/experiments/2026-09-26-sign-detector.md`.
- `docs/architecture/ar-overlay-design.md` gains a speed-value agreement rule.
- Small fixes to the overnight agent's scripts: the example weights paths in `export_onnx.py` and
  `eval_sign_detector.py` pointed at v1, and two docstring lines were over 100 columns.

**Reasoning / results:**
- **Speed:** the sign engine runs at 2.48 ms median. The four models total ≈ 11.3 ms, measured
  alone.
- **Kenyan audit, all 57 detections judged from crop contact sheets:**
  - at ≥ 0.5 confidence, 24 of 25 verifiable detections were correct;
  - the one error was a 30 sign read as 50 (0.57), the most harmful kind for the display, hence
    the new value-agreement rule;
  - below 0.25 it is mostly noise (logos, a face, sign backs);
  - a Kenyan text "NO ENTRY" sign was found only at 0.18.
- **Kenyan recall is unmeasured:** 11 of 12 randomly sampled no-detection frames contain no
  traffic sign, so the collected set cannot measure misses. This is stated plainly in the report
  rather than read as good (or bad) recall.

**Problems hit:** None new.

**Still open:**
- The Kenyan labelled test set (own drives).
- Improvement item 1 (mask ambiguous boxes, retrain).
- Concurrent four-model timing in `MlInferenceEngine`.
- Next: the YOLOv8m-seg work.

## 2026-09-26 — YOLOv8m-seg: export, engine, mask decoding, pipeline integration, cross-check

**Attempted:** The deferred work for the hazard glow (`ar-overlay-design.md` §3.2) and mask-based
LiDAR fusion (Part 8.3): per-object masks from YOLOv8m-seg.

**Built/changed:**
- **Export and engine:**
  - `host/models/weights/yolov8m-seg.pt` (official ultralytics release);
  - `export_onnx.py --model yolov8m-seg`, which verifies both outputs against PyTorch;
  - `models/onnx/yolov8m_seg.onnx` and `models/engines/yolov8m_seg.engine`;
  - an optional seg build in `build_tensorrt_engines.sh`.
- **Decoding** (`Postprocess.h/.cpp`): `Box::candidate`, `ObjectMask`, `maskCoefficients`,
  `decodeMask`, `maskContains`. 7 new unit tests; the unit suite is 76/76.
- **Pipeline:**
  - `MlInferenceEngine` recognises a seg engine by its two outputs, decodes one mask per kept box,
    and publishes a `MaskFrame` on a new `MaskBus`;
  - `DetectionFrame.mask_ref` = frame seq (appended last in `detections.fbs`);
  - the default detector engine is now `yolov8m_seg.engine`, and a box-only engine still works.
- **Tools and tests:** `inference_viewer` tints masks by class and `--dump` writes a mask label
  image. `test_trt_engine.cpp`'s real-engine list gains the seg and sign engines; GPU tests 17/17.

**Reasoning:**
- *Prototype masks:* YOLOv8-seg outputs 32 image-wide prototypes at ¼ resolution plus 32
  coefficients per candidate. A mask is the sign of the coefficient-weighted sum, cropped to the
  box. sigmoid(v) > 0.5 ⇔ v > 0, so no exponentials are needed.
- *Linking boxes to masks:* each kept `Box` records its candidate index, so masks cannot be
  attached to the wrong object after NMS reorders boxes by confidence.
- *Masks stay on the ¼-res grid, cropped to the box:* memory is small, and `maskContains`
  answers source-pixel queries, which is exactly what LiDAR fusion needs.

**Verification:**
- **Engine** (trtexec, alone): 6.75 ms median, vs 5.38 ms for box-only YOLOv8m (+1.37 ms,
  +25%). That is more than my estimate of 10–20%; the 4-model total is ≈ 12.7 ms.
- **Whole pipeline on Kraków 2560×1440** (600 frames, including CPU mask decoding): median
  14.6 ms, p95 18.3, max 22.3 (was 12.8 / 16.6 / 20.9 without masks). Detections per frame are
  unchanged at 7.0.
- **Negative test:** a threshold of `v >= 0` instead of `v > 0` fails 2 of the mask tests.
  Restored.
- **Cross-check against ultralytics' seg pipeline** (PyTorch FP32) on Kraków frames 120 and 360:
  - every road-relevant object matched, box IoU 0.950–0.995;
  - mask IoU on the ¼ grid: 0.97–0.99 for large objects, 0.81–0.92 for medium, 0.57 for a tiny
    distant pedestrian (14 vs 8 grid px);
  - our masks run consistently a few cells larger. Ultralytics upsamples before thresholding and
    cropping, while we crop whole partial cells at the box edge, which matters mostly for tiny
    objects. That is fine for the glow; Part 8.3's erosion rule covers fusion;
  - the one unmatched reference box is a truck 0.73 + car 0.35 duplicate on the same vehicle,
    merged by our class-mapped NMS by design.
- **Visual:** masks follow real outlines (a car body and wheels, a pedestrian's legs and bag).
- Both clang-format versions (22 local, 18.1.8 = CI) report 0 violations.

**Still open:**
- The sign detector is not yet a fourth engine inside `MlInferenceEngine`. It needs a class
  mapping-free decode, and then concurrent four-model timing.
- The hazard glow itself is Phase 11.
- Mask erosion and LiDAR association are Phase 7.
