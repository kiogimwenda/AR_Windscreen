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

## Phase 1 — Shared protocol

**`#include <cstddef>` added to `Protocol.h`.** Part 3.1's snippet uses `size_t` while including
only `<cstdint>`, which compiles only where `<cstdint>` transitively provides it.

**Frame geometry constants added** (`HEADER_SIZE`, `CRC_SIZE`, `CRC_COVERED_OFFSET`,
`MAX_FRAME_SIZE`). Derived from Part 3.1's frame diagram, which otherwise leaves `VehicleInterface`
and `CommsTask` to re-derive the same offsets independently. The CRC covered range especially: a
mismatch there leaves both ends self-consistent and mutually incompatible, which surfaces only on
a bench.

**`static_assert` on every struct size and on `sizeof(float)`/`sizeof(double)`.** The two ends are
different architectures and compilers; `#pragma pack(push, 1)` removes padding but nothing
otherwise verifies that it did. These turn a silent layout disagreement into a build failure on
whichever side drifts. Negative-tested on the ARM cross-compile, not assumed.

**HEARTBEAT defined as a zero-payload message.** Part 3.3 requires it but specifies no struct.
Nothing needs to travel in it: the hub only needs evidence the host is alive, and staleness of an
actual request is already covered by `ActuationCommand::hostTimestampMs`.

**CRC test vectors duplicated in the host and firmware suites rather than shared.** The point of
Part 3's "implemented twice, kept identical" rule is independent confirmation. `protocol-sync-check`
proves the files match; the two suites prove each copy independently yields the standard results. A
shared vector header would reduce that to a single point of failure.

**`CommsTask.cpp` includes `Protocol.h` while still otherwise a stub.** `static_assert`s only fire
in a translation unit that compiles the header, and both test suites run on x86-64 — so without
this, the cross-architecture layout claim would be untested on the architecture it is about. A bare
`pio run` now checks it on ARM. `CommsTask` owns framing per Part 4.3, so the include belongs here.

**`[env:native]` added to `platformio.ini`, with `default_envs = stm32f4_hub`.** Gives Phase 1 an
automatable firmware-side test with no board attached, while keeping a bare `pio run` — which is
what CI's `firmware-build` job runs — building only the STM32 target.

**`.clang-format-ignore` patterns need `**`, not `*`.** Patterns are globs relative to the ignore
file and `*` does not cross `/`, so `.pio/*` matches nothing useful.

## Hardware procurement

**Bill of materials kept at `docs/bill-of-materials.md`.** Part 1's tree has no BOM file; it lives
beside `decisions.md` because it is a planning record, not code. Sourcing assumes Kenya (the dev
machine's timezone is Africa/Nairobi), local shops first, imports only where nothing local exists.

**STM32F401CC Black Pill proposed over the guide's F411.** The F411 is out of stock locally; the
F401 (K-Technics, KES 900) still meets Part 4.1's minimum. If bought, `platformio.ini`'s board
becomes `blackpill_f401cc` at the start of Phase 2.

**Level converters on every 3.3 V → 5 V module input.** BTS7960 boards' 74HC input buffers and
opto relay boards do not switch reliably from 3.3 V; marginal logic on a brake PWM line is not
acceptable under the Part 0 safety rule.

**Brake actuator purchase deferred pending a mechanical design decision.** Locally sold actuators
are non-back-drivable lead screws, which would hold the brake applied after a power cut if rigidly
coupled, contradicting the hard rule. Candidate direction (pull-only cable, fail-safe electromagnet
release, fast low-force actuator) recorded in the BOM's note B for supervisor review, not adopted.

**Two guide gaps flagged, not yet changed:** brake-light-switch sensing via optocoupler as a
fallback for `obdBrakePedalActive` (BOM note C), and Part 15.2's USB-C PD laptop charger being
undersized for an RTX 5060 laptop (BOM note D).

## Phase 3 — Host skeleton (software part)

**Phase 3 split: software parts built before Phase 2.** `RingBuffer`, `EventLog` and
`SystemManager` need no hardware. `VehicleInterface` and the Phase 3 exit test (loopback against the
real hub) wait for Phase 2. Phase 3 is not marked complete until then.

**`RingBuffer` drop-oldest is done by the consumer (`popLatest`), not the producer.** In an SPSC
queue only the consumer owns `tail_`; a producer that discarded the oldest item would race the
consumer reading that slot. `push` returns false when full and the caller applies the policy, as
Part 5.3's own comment on `push` says.

**`RingBuffer` capacity must be a power of two, and is exactly N.** Monotonic counters with
`% N` are only correct across counter wrap when N divides 2^64.

**Bus messages are FlatBuffers object-API structs (`--gen-object-api`), not serialized buffers.**
In-process hops gain nothing from serialisation; the byte form stays available for replay.

**`flatc` 24.3.25 built from OSRM's vendored source and installed to `/usr/local/bin`.** OSRM
installs FlatBuffers 24.3.25 headers into `/usr/local/include`, shadowing Debian's 23.5.26, and its
own public headers need them, so the host standardises on 24.3.25. `FlatBufferSchemas.cmake` fails
configure if `flatc` and the headers disagree.

**`EventLog`: raw `write()` per line; `fsync()` only for actuation, ack and fault lines.** `write()`
alone survives a process crash (Part 11.6's requirement). `fsync` adds power-loss durability where
it matters, without its cost on routine lines.

**`EventLog` failure latches `healthy()` false rather than throwing mid-run.** A throw from a logging
call inside a subsystem thread would take the thread down. The flag lets Phase 10's arbiter refuse
to actuate without an evidence trail.

**`OSRM::osrm` defined in `cmake/FindOSRM.cmake` instead of the guide's bare `osrm`.**
`libosrm.a` is static and records none of its dependencies, and OSRM's `libosrm.pc` is broken
(CMake target names in `Libs.private`). The target names Boost date_time/iostreams/thread, TBB,
zlib and rt explicitly.

**yaml-cpp added as a dependency.** Part 2.3 lists no YAML library, but the config files are
YAML. yaml-cpp is packaged on both Debian and Ubuntu and is CPU-only, so the unit suite stays
GPU-free.

**`start<T>(args...)` forwards constructor arguments** instead of Part 11.5's
`start<T, Sinks...>(Sinks&...)`. Phase 11 will construct the `WindowedSink` and pass it, e.g.
`start<ArRenderer>(sink)`.

**Subsystem contract: `void run(const std::atomic<bool>& stop)`.** Part 11.5 leaves it
unspecified. A throw or early return from `run()` shuts the whole system down.

**Shutdown joins every subsystem before the final-command hook runs.** That guarantees the zeroed
command is the last thing sent. The heartbeat gap during joins may trip the hub watchdog, which
fails safe.

**Crash signals write a pre-registered zeroed frame with one `write()`, then re-raise.** It is the
only async-signal-safe way to meet Part 5.4's "crash-triggered" shutdown requirement.

**`main` defaults: `host/config` and `logs/session.log`, run from the repo root.** Part 5.4's
`"config/"` assumed a working directory of `host/`, while `logs/` is at the repository root.
Both are overridable by argv.

**CI installs no OSRM until Phase 8.** `libosrm-dev` does not exist on Ubuntu.

## Phase 5 — ML inference

**Lane model: UFLDv2 CULane ResNet-18 at 320×1600.** Part 7.1's 800×288 is UFLD v1's size; v2
ships 320×800 (TuSimple) or 320×1600 (CULane). CULane's conditions are nearer Nairobi roads.

**Depth model: MiDaS v2.1 Small at 256×256.** "v3.1 Small at 384×384" matches no checkpoint.
v3.1's SwinV2-T weights do not load under timm ≥ 0.7, and timm 0.6 does not run on Python 3.13.
Depth is relative and secondary to LiDAR range.

**Detector: COCO-pretrained YOLOv8m. Part 7.1's classes are a mapping from COCO classes.** COCO
has no generic "obstacle" class.

**Exports verified against PyTorch through ONNX Runtime; UFLD loads with `strict=True` and
`weights_only=True`.** UFLD's own demo uses `strict=False`, which silently leaves mismatched
layers random.

**Export environment layered over `~/ml-env` rather than installing into it.** `~/ml-env` is
shared with unrelated CUDA work.

**`TrtEngine` targets TensorRT 10's name-based API; inference is split into `enqueue()` and
`sync()`.** Part 7.4's single `infer()` would serialise the three models. The engine rejects
dynamic shapes, non-FP32 I/O and multiple inputs at load.

**GPU tests live in `host/test/integration/` under ctest label `gpu`, outside CI.** Consistent
with Part 13.1 and Part 13.5.

**YOLOv8m at 1280×736 instead of 640×640.** At ~70° HFOV, a pedestrian stays ≥12 px out to
~148 m (enough for ~100 km/h plus tracking), and close objects stay within YOLOv8's trained scale
range (≤ ~960 px). Measured 6.6 ms median; camera-native 2560 measured 26.5 ms and would push close
objects outside that range. Revisit once Phase 4 measures the real FOV.

**MiDaS at 448×256 (16:9), ~1.75× its standard scale; UFLD stays 1600×320.** MiDaS's own small
transform caps the long side at 256 (256×128 for 16:9). 448×256 is a moderate increase for sharper
depth edges. The same entry previously said "native 256 short side", which was wrong (corrected
2026-09-24, after reading MiDaS's transform). UFLD's fully-connected head fixes its input size.

**`build_tensorrt_engines.sh` omits Part 7.3's `--shapes`.** TensorRT rejects it for fully static
models. The script verifies the built engine's input shape instead.

**UFLD export substitutes a no-op `utils.common` and reads its config with `runpy`.** Its real
`utils.common` pulls in DALI, tensorboard and other training-only packages. `strict=True` loading
proves every weight is replaced.

**COCO → Part 7.1 class map.** Motorcycles are Cyclists (two-wheelers with exposed riders). Large
animals are Obstacles. Birds and non-road classes are dropped. Classes are mapped before NMS.

**Post-processing never suppresses a pedestrian box because of overlap with another class.** No
"rider suppression": a missed pedestrian is the unsafe error.

**The lane model sees a CULane-shaped (2.78:1) band of the frame, positioned by config.** It is not
fed the whole 16:9 frame squashed.

**`MlInferenceEngine` loads its engines in the constructor, not a separate `init()`.** A bad model
set then fails at `SystemManager::start`, before any thread runs.

**Depth maps travel on their own bus; `DetectionFrame::depth_map_ref` is the frame sequence
number.** This follows Part 3.5's "handle, not inlined" without shared mutable storage.

**Lane band top defaults to 0.32 of frame height.** It was tuned on the Kraków footage and must be
re-tuned in Part 12 for the real camera mount.

**Offline footage: Kraków city drive (CC BY 3.0), 2560×1440.** It is used for pipeline and timing
validation only, not as an accuracy measure for Kenyan roads.

## Phase 5 extension — road-sign behaviours (design)

**A fourth model, a dedicated sign detector trained on MTSD, is added beyond the guide.** Ian
observed that COCO-trained YOLOv8m misses most signs. COCO knows only stop signs and traffic
lights. Part 7.2 allows fine-tuning for a failure mode found in testing.

**Sign behaviours are specified in `docs/architecture/sign-behaviours.md`.** Signs inform and never
actuate. Signs are shown only after multi-frame confirmation, in priority order, drawn under hazard
boxes, and degrade to screen-fixed banners when road geometry is missing. A detected sign confirms
or overrides OSM priors.

**Horizontal-flip augmentation is disabled for sign training.** Flipping swaps the meanings of
left/right-specific signs (no-left-turn and no-right-turn, keep-left and keep-right).

## Phase 5 extension — traffic-sign detector (overnight run)

**Only the fully annotated MTSD parts are downloaded.** Ian's signed MTSD links cover 11 zips. They
were identified by the MD5 lists shipped beside them and by reading each zip's central directory
from a ranged request: annotations (0.12 GB), val (4.57 GB, 5,320 images), train.0/1/2 (10.4 GB
each, ~11.8k images each), test (8.96 GB, 10,544 images; labels not public, skipped) and the
partially annotated set (annotations + 4 zips of ~11.9 GB, skipped). The three train zips are
assigned .0/.1/.2 by the alphabetical key range of their contents; the MD5 list confirms this
after download.

**Bandwidth forces training on train.0 only.** The measured aggregate download rate was ~2 MB/s
over five parallel streams, so the 35.8 GB needed would finish at ~03:30, leaving 2 h of training.
Following the plan's fallback, train.1 and train.2 were paused so that annotations + val + train.0
(15 GB) get all the bandwidth, and training starts as soon as those are unpacked. train.1/2 then
continue downloading in the background for later runs.

**Kenyan evaluation imagery comes from KartaView only (plus any openly licensed Commons photos).**
KartaView's v2 API rejects bounding-box sequence queries ("Restricted access"), so sequences are
listed through the v1 `list` endpoint over 0.05° tiles and each sequence's photos are fetched
through the v2 photo endpoint. Every 12th photo is kept, at most 10 per sequence, to spread the
set over many roads. The already-blurred ("proc") images are used.

**Plan change at ~23:55 (Ian): wait for ALL fully annotated training data, train until 08:00.**
This supersedes the "train on train.0 only" choice above. A run had been started at 23:53 on the
6,314 train images extracted by then; it was stopped within two minutes, before its first epoch
ended, so it produced no weights. Its directory is kept as
`host/models/training/signs_v1_partial_superseded/` (only `args.yaml`). train.1 and train.2 were
resumed. If the downloads are not complete by 03:00 EAT, training starts on everything complete
by then.

**Speed-limit variants whose unit is mph or unclear are dropped, not merged.** Crops of every
`maximum-speed-limit-*` variant were inspected. The round red-ringed designs (g1, and the LED
variants) are km/h and are merged into the speed classes. The rectangular white "SPEED LIMIT NN"
designs are US/Canadian and so usually mph: `30--g3`, `40--g3`, `40--g6` (a "41"-like sign) and
`50--g6` (plain white rectangle, unit unclear) have their boxes dropped (listed in
`host/scripts/mtsd_uncertain_unit_speed_variants.json`). Speed values with no target class (5, 15,
25, 35, 45, 55, 65 and the truck limit) go to other_regulatory. `100--g3` ("MAXIMUM 100 km/h")
is km/h and is kept.

**Narrow merges.** keep_left_or_right is keep-left, keep-right, pass-on-either-side and the
US-style warning "pass left or right". Mandatory turn-only / straight-only signs are NOT merged
into it (different meaning) and stay in other_regulatory. no_overtaking excludes the
heavy-goods-vehicle variant. end_of_restriction is end-of-maximum-speed-limit-*,
end-of-prohibition and end-of-speed-limit-zone; end-of-no-parking, end-of-priority-road and the
end-of-lane-type signs stay in other_regulatory. road_hump is warning--road-bump only (uneven-road
stays in other_warning).

**`information--pedestrians-crossing` (2,272 boxes, the blue square European design) is dropped**
as the brief requires for all information--* labels, so only the warning-triangle designs train
pedestrian_crossing. This is a real loss of a common sign: a blue pedestrian-crossing square will
be background to this model. Flagged for Ian.

**Panoramas are skipped** (260 of the images converted so far): equirectangular images with boxes
that wrap the edge, unlike the forward-facing production camera.

**Back to the original timeline (00:00).** The requested extension of training to 08:00 could not
be applied in this unattended run (the change was blocked by the session's permission checks, as
it moved the hard 07:00 deadline). The original brief therefore stands: training ends by 05:30
and the report is due by 06:45. Under that brief, a full download (~03:10 at the measured
2.2 MB/s) would leave well under 4 h of training, so its fallback applies: train.1/train.2 are
paused again, train.0 gets all the bandwidth, and training starts as soon as annotations + val +
train.0 are complete and converted. train.1/2 resume downloading after that, for later runs.
Ian should decide whether to retrain on the full set.

## Guide amendment: GPU overlay rendering (2026-09-25, at Ian's instruction)

**Part 10 now renders AR overlays on the GPU (OpenGL shaders in `WindowedSink`) instead of OpenCV
CPU drawing.** The holographic style (translucency, glow, gradients, pulsing) is per-pixel blending
over several full-screen 2K layers per frame, which would compete with the pipeline for the frame
budget on the CPU.

**The Part 10.1 `DisplaySink` interface changed:** `CompositedFrame` now carries the video frame
plus an `OverlayScene` (a draw list) instead of pre-composited pixels. It is the cleaner seam for
the optical phase: a `ProjectorSink` can draw only the overlays. **The FYP report's description
of this interface must be updated to match.**

**Road-locked overlay items are given in vehicle-frame metres and projected in the vertex shader
with the Part 12 calibration.** Barriers and the route band therefore land where perception places
them.

**Driver view shows hazard highlights styled by threat level, not a box around every detection.**
Per-class boxes with confidences remain in the debug viewer.

**Video upload via PBO; CUDA/GL interop not assumed under WSLg.** WSLg's OpenGL runs on Mesa's
D3D12 layer, not NVIDIA's GL driver. To be confirmed in the 10.4 benchmark.

## Hazard overlays replace boxes (2026-09-25, Ian's design)

**No rectangles on the driver display.** Hazards use the same road-placed language as signs:
- barriers at collision-risk objects;
- an amber → red route-line grade;
- a following-distance zone for tailgating;
- a red lane-line glow towards a swerving neighbour;
- ground rings under pedestrians and animals.

In addition, the object itself gets a subtle shimmering glow along its outline, coloured by a
risk level `r` (TTC + Part 9.2 flags, thresholds in config). Ordinary traffic gets nothing.

**Rule 4 changed from "draw nothing over a hazard" to "never hide a hazard".** The glow is a rim
plus ≤ ~25% tint. Road graphics are masked by object silhouettes so they appear behind objects.

**Object detection moves to YOLOv8m-seg (COCO).** The glow needs per-object masks, and boxes alone
would make it rectangular. It is the same class set and export path, plus a mask output. The work
is deferred until the sign-detector training finishes, so the GPU isn't shared with training.
A missing mask falls back to a soft ellipse.

**`sign-behaviours.md` became `docs/architecture/ar-overlay-design.md`** (signs + hazards + a
shared visual-language table). Earlier log entries naming the old file are left as written.

**`BUILD_GUIDE.md` Part 7.1 brought up to date:** YOLOv8m-seg, the sign detector row, and the
actual lane/depth models and sizes. The Part 7.3 commands are superseded by the scripts.
`detections.fbs` gains `mask_ref` in the guide. The schema file itself changes with the seg
implementation.

## Mask-based LiDAR fusion and startup extrinsic check (2026-09-25, Ian's proposal)

**Part 8.3 associates LiDAR points with objects through their YOLOv8m-seg masks,** not boxes or
cluster centroids. A box is mostly background, and box-selected points mix a pedestrian's range
with the wall behind. The rules are: motion-compensate, erode masks, take the nearest dominant
depth cluster, depth-test for parallax, report "no range" honestly, locate signs via
high-intensity returns inside their box, and treat unexplained LiDAR clusters in the corridor as
`UNKNOWN` obstacles. The camera may never veto the LiDAR.

**Part 12.2.1 adds `ExtrinsicMonitor`: startup and continuous verification with *bounded*
refinement** (~1°, ~3 cm) around the Part 12.2 checkerboard baseline. It is not a from-scratch
self-calibration, which can converge confidently to a wrong answer on a poor scene.
- It scores edge alignment plus mask agreement.
- It accepts a refinement only with enough varied evidence, held-out improvement, and a solution
  off the bounds.
- `DEGRADED` keeps the baseline, logs a fault, falls back to screen-fixed display, and makes the
  arbiter use LiDAR-only corridor ranges.
- Every start begins from the baseline, so errors don't accumulate.
- Intrinsics and camera-to-vehicle are not auto-adjusted.
- Braking never depends on an unverified refinement.

## Motion prediction (2026-09-25, Ian's proposal)

**Part 9.1's constant-velocity tracker is replaced by an IMM tracker with a `MotionPredictor`.**
- Models: CV for pedestrians, animals and unknown obstacles; CTRV via a UKF, with an explicit ω→0
  branch, for vehicles; a stopping model.
- Per-class process noise in `config/motion_prediction.yaml`.
- Tracking in a world-fixed frame via the EKF pose.
- The Appendix Q.7 loop (predict → Mahalanobis → Hungarian → gate → update → lifecycle) is kept.

**Prediction is object-level, measured from mask-fused LiDAR points**: L-shape fits and cluster
registration, never centroids. The non-repetitive Livox scan has no point-to-point
correspondence. Centroid shifts from changing visibility fake lateral velocity, which would
falsely trip the swerving rule.

**Risk for warnings and display uses closest point of approach and collision probability**
against a predicted ego path. TTC alone is blind to crossing traffic and cut-ins.

**Predictions are distributions, not lines.** Horizons above ~1.5 s are treated as intent, not
physics (uncertainty ≈ √(σv²t² + σa²t⁴/4)). Honesty is checked by NIS (chi-square) and by
ADE/FDE on recordings.

**Braking (Part 9.3 rule 1) uses only the current tracked range and closing speed,** never
forecasts, CPA, collision probability or IMM model probabilities. This avoids phantom braking and
keeps the brake rule auditable. Because the IMM changes the closing-speed estimate,
`test_motion_predictor.cpp` must bound its error and lag before Phase 10.

**Physics-based (verifiable) first.** Map-aware prediction next; a learned predictor only as a
display-only stretch goal.

**The renderer draws object-anchored overlays at their positions predicted for display time**
(latency compensation), plus predicted-path ribbons and `PREDICTED_ONLY` ghosts.

**YOLOv8m-seg masks are decoded on the CPU at prototype-grid resolution (¼), cropped to the box.**
Cost is small (≈ +1.8 ms whole-pipeline, measured), and it is exact to ultralytics for large
objects and slightly generous at box edges for tiny ones. Upsampling to full resolution is left to
the renderer.

**The engine type is recognised by its outputs** (two = segmentation). `mask_ref` was appended
last in `detections.fbs`, as FlatBuffers schema evolution requires.

## Phase 7 — Fusion and tracking

**EKF (ego): CTRV with IMU yaw rate as a measurement, not a control input.** IMU longitudinal
acceleration is unused in v1 (OBD observes speed). Joseph-form updates. `VehiclePose.heading_deg`
is the maths convention (CCW from east); compass conversion happens only at display.

**Tracker IMM: shared [px, py, ψ, v, ω] state, UKF prediction, asymmetric switching (rare entry
into STOP).** Symmetric rates biased cruise speed ~9% low (measured).

**Two-point velocity initialisation on a track's second sighting.** The speed+heading state
cannot learn direction from rest. A Cartesian CV model for pedestrians is the recorded
improvement for the remaining post-birth speed overshoot.

**Late measurements are replayed from snapshots within 500 ms.** Anything older than every
snapshot is dropped and counted, never applied out of order.

**Per-class motion noise lives in `config/motion_prediction.yaml`, as starting values.** They are
tuned against recordings via the NIS check (Part 9.1.3), never by eye.

**Collision probability is a Monte-Carlo estimate over sampled futures, drawn with exactly the
filter's own process noise, seeded by track id.** CPA (t*, d*) covers crossing traffic that
range/closing-speed TTC misses. Neither ever reaches the brake (Part 9.3).

**Prediction still uses the tracking noise, and that overstates lateral spread for
lane-following vehicles (measured: head-on 0.75, oncoming-in-other-lane 0.25).** Warning
thresholds are set on these measured values. The fixes are recorded, not applied by tuning:
prediction-specific noise calibrated on recorded drives (ADE/FDE, Part 9.1.4), then lane-aware
prediction once lanes are tracked.

**The filter reports speed as non-negative.** The models may internally hold (−v, ψ). The
reported state is flipped to (v, ψ+π), so the models are never flipped one by one and mixing
stays coherent.

**Mask-based fusion is pure geometry on plain values; bus adaptation waits for LidarProcessor
(Phase 6).** Every Part 8.3 rule is then testable on synthetic scenes.

**Fusion tests use ray-cast scenes, not hand-placed points.** The LiDAR is on the roof, offset
from the camera, and its angular scan stops at the first surface. Occlusion, parallax and angular
point density are therefore physically right, and camera-view masks are ray-cast too.

**Rules 2 (erosion) and 4 (depth test) are defence in depth behind rule 3.** In the test scenes,
rule 3 alone still recovered the range. What the two rules measurably do is raise the share of
mask-selected points that belong to the object:
- erosion: 49% → 89%;
- depth test: 35% → 85%.

That share is published as `points / maskPoints` for ExtrinsicMonitor (Part 12.2.1), since a
falling average is a symptom of extrinsic drift.

**Unexplained LiDAR obstacles ignore points more than 2.5 m above the road.** Gantries and
branches cannot hit the car. An object's points hidden by the depth test are still claimed by it
(inside its mask, within its depth band), so it never reappears as a duplicate UNKNOWN obstacle.

**MiDaS estimates fit rel = a/z + b** (MiDaS is affine-invariant in inverse depth, not only
scale-invariant) to at least two LiDAR-ranged objects in the frame. They are flagged ESTIMATED
and are never usable for braking.
