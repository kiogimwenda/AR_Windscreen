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
