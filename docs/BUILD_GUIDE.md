# AR-Assisted Semi-Autonomous Driving System — Complete Build Guide

**A ground-up implementation specification, written to be executed by an AI coding agent (Claude Code) from an empty repository through to a demo-ready, presentation-ready final year project.**

Target platform: laptop with NVIDIA RTX 5060 (Blackwell, `sm_120`) + Intel Core Ultra 7 275HX, developed inside a Debian WSL2 instance. Sensor/actuator hub: STM32 microcontroller running FreeRTOS. Display: video-see-through AR rendered on the laptop's own screen, built behind a swappable `DisplaySink` interface so the project can later return to an optical-combiner HUD without a rewrite.

---

## How to use this document

This guide is written as an **execution plan**, not a reference manual. It is ordered so that each part can be built and verified before the next depends on it — do not skip ahead to a later part before the exit criteria of the current one are met. Every part that produces code specifies: the exact files to create, the interface each file must expose, and a concrete way to verify it works before moving on. Where a step is genuinely optional or "future work," it is labeled as such — everything else is in scope for this build.

If you are an AI coding agent executing this guide: work through the **Phased Roadmap** (Part 14) top to bottom. Each phase lists its deliverables and its exit criteria. Do not mark a phase complete until its exit criteria are demonstrably true (a build that compiles, a test that passes, a log line that appears) — not merely "code has been written for it." When a part of the spec is ambiguous or you must make an implementation choice not specified here, make the most conservative choice with respect to the safety rules in Part 9, and note the choice in `docs/decisions.md`.

This project has one hard rule that overrides every other instruction in this document: **the brake actuator must never be able to exert more force than a driver's foot can override, must never disconnect or block the driver's own pedal linkage, and must default to fully released/inert on any fault, timeout, link loss, or ambiguous state.** Every piece of firmware, protocol, and control logic touching actuation is written to that rule first and to functionality second.

**Progress logging is mandatory, per task, not per phase.** Alongside `docs/decisions.md` (the short index of choices made under ambiguity), this build keeps a second file, `docs/progress-log.md` — a detailed, dated record of every task completed. Append to it as each discrete piece of work finishes, not only at phase-exit checkpoints, and not reconstructed retroactively from memory at the end of a session. Part 1.1 gives its exact format. This file is what the person building this project pulls from to write supervisor progress reports without having to reconstruct what happened from diffs.

**This project has a second standing rule, alongside the actuation safety rule above: build this so the person following the guide ends up able to explain every part of it, not merely possessing a working system.** They bring strong C/C++ fundamentals but little to no prior exposure to several tools this build leans on — TensorRT, CUDA stream programming, PCL, FreeRTOS task design, EKF math, CMake's build model, OSRM. Do not write code first and explain later, and do not bury the reasoning in a wall of code with a comment on top: introduce the concept a piece of code depends on before or alongside writing it, calibrated to what's genuinely new to them (skip re-explaining C++ syntax they already know; go deep on the unfamiliar tools and math). Treat "can they explain what this phase built and why it works this way" as an unstated but mandatory addition to every phase's exit criteria in Part 14 — a phase whose code works but whose builder can't explain it is not actually done. This matters because they will be directly questioned on any part of this codebase by supervisors and examiners, and "an AI wrote it" is not an acceptable answer in a viva.

**Version control practice:** when committing and pushing to GitHub, commits are authored as the person alone. Never add a `Co-authored-by` trailer, a "Generated with Claude Code" footer, or any other mention of AI assistance in a commit message or its metadata.

---

## Table of Contents

**Part 0 — Project Recap and Scope**
**Part 1 — Repository Structure**
**Part 2 — Development Environment Setup**
**Part 3 — Shared Protocol and Message Schemas**
**Part 4 — Firmware: Sensor & Actuator Hub (STM32 / FreeRTOS)**
**Part 5 — Host Application Architecture**
**Part 6 — Camera Pipeline**
**Part 7 — ML Inference Engine and Model Pipeline**
**Part 8 — LiDAR Processing and Sensor Fusion**
**Part 9 — Decision / Arbiter and Reckless-Driving Detection**
**Part 10 — AR Renderer and Display Sink**
**Part 11 — Vehicle Interface, Navigation, and System Manager**
**Part 12 — Calibration Procedures**
**Part 13 — Testing: Unit, Integration, and Staged Real-World Validation**
**Part 14 — Phased Implementation Roadmap (Execution Order)**
**Part 15 — Physical Build and Vehicle Installation**
**Part 16 — Report and Presentation Guide**
**Appendices**

---

## Part 0 — Project Recap and Scope

**What this system does:** fuses camera, LiDAR, GPS, IMU, and OBD-II data into a real-time model of the road ahead; classifies hazards and reckless surrounding-vehicle behaviour; renders that understanding as an augmented-reality overlay on the laptop's screen — including a precision, road-locked turn-by-turn navigation overlay that tracks the actual road surface rather than floating over it — and, through a dedicated microcontroller hub, triggers indicators/lights/horn automatically and can apply a force-limited brake-pedal actuation when a collision risk crosses a defined threshold and the driver has not already braked.

**Navigation precision is a primary objective, not a stretch feature.** The route overlay must appear locked to the road surface and to the correct lane at close range, not as a generic arrow floating over the video. This is achieved by combining three things that already exist elsewhere in this system rather than by adding a new sensing modality: OSRM routing and map-matching (which road, how far along it), the LiDAR's ground-plane model from Part 8.1 (the real measured surface close to the vehicle, so the line doesn't float or sink through slopes and dips), and the lane-detection model's output (a lateral correction so the line sits in the correct lane rather than trusting GPS's few-metre error). Part 11.2–11.4 specify this pipeline in full. Precision degrades gracefully with distance — near-field (within LiDAR range) is measured-surface-accurate, far-field falls back to a flat-ground assumption — and the report should describe this honestly rather than imply uniform accuracy end-to-end.

**What is explicitly out of scope for this build** (do not implement these; they are future work — see Part 16's future-work section):
- The optical combiner / DLP projector display path (`ProjectorSink`) — the `DisplaySink` interface must exist and be clean to extend, but only `WindowedSink` is implemented now.
- Steering intervention of any kind.
- Full autonomous driving of any kind — this system assists and can brake; it never drives.
- A thermal/IR camera sensor — noted as future work only.
- Cloud connectivity, OTA updates, companion mobile app, fleet features.

**Non-negotiable safety properties the finished system must have** (verified explicitly in Part 13):
1. The driver's foot always overrides the actuator.
2. A physical kill switch removes actuator power independent of software.
3. The hub's watchdog releases the actuator within 200 ms of host link loss, with no dependency on host software correctness.
4. The host (laptop) never drives actuation hardware directly — it only ever sends a bounded *request* to the hub, and the hub enforces the real ceiling.
5. Every actuation event (request, hub decision, applied force/duration, release reason) is logged with a timestamp, on both host and hub, for later report/viva evidence.

---

## Part 1 — Repository Structure

Create this exact structure at the repository root before writing any code. Nothing here is arbitrary — the split between `firmware/` and `host/` mirrors the hard boundary from Part 0 (host never touches vehicle hardware directly), and `config/` is separated from `src/` so calibration data can change without a rebuild.

```
ar-drive-assist/
├── README.md
├── LICENSE
├── .gitignore                            # see Part 1.2 — .claude/ is the first line, non-negotiably
├── CMakeLists.txt                        # top-level: adds host/ as subdirectory
├── docs/
│   ├── BUILD_GUIDE.md                    # this document
│   ├── decisions.md                      # running log of implementation choices made under ambiguity
│   ├── progress-log.md                   # detailed, dated record of every task — see Part 1.1
│   ├── architecture/
│   │   └── system-architecture.md        # kept in sync with Part 5 diagrams
│   ├── report/                           # FYP report source (Part 16)
│   └── presentation/                     # slide deck source + demo script (Part 16)
│
├── firmware/
│   └── sensor_actuator_hub/
│       ├── platformio.ini
│       ├── include/
│       │   └── hub/
│       │       ├── Config.h
│       │       ├── Protocol.h            # shared wire format, see Part 3
│       │       └── Crc16.h
│       ├── src/
│       │   ├── main.cpp
│       │   ├── tasks/
│       │   │   ├── SensorTask.cpp / .h
│       │   │   ├── ActuationTask.cpp / .h
│       │   │   ├── WatchdogTask.cpp / .h
│       │   │   └── CommsTask.cpp / .h
│       │   └── drivers/
│       │       ├── ImuDriver.cpp / .h
│       │       ├── GpsDriver.cpp / .h
│       │       ├── ObdDriver.cpp / .h
│       │       ├── GestureDriver.cpp / .h
│       │       ├── RelayDriver.cpp / .h
│       │       └── BrakeActuatorDriver.cpp / .h
│       └── test/
│           └── test_protocol_crc.cpp
│
├── host/
│   ├── CMakeLists.txt
│   ├── include/ar_drive_assist/
│   │   ├── camera/CameraPipeline.h
│   │   ├── inference/MlInferenceEngine.h
│   │   ├── inference/TrtEngine.h
│   │   ├── fusion/SensorFusion.h
│   │   ├── lidar/LidarProcessor.h
│   │   ├── lidar/GroundPlaneModel.h      # published ground-surface output, see Part 8.1 — no longer a throwaway
│   │   ├── scene/SceneReconstruction.h
│   │   ├── safety/RecklessDrivingDetector.h
│   │   ├── safety/MultiObjectTracker.h
│   │   ├── decision/DecisionArbiter.h
│   │   ├── render/ArRenderer.h
│   │   ├── render/DisplaySink.h
│   │   ├── render/WindowedSink.h
│   │   ├── vehicle/VehicleInterface.h
│   │   ├── vehicle/HubProtocol.h         # copy of firmware/.../Protocol.h, kept identical — see Part 3
│   │   ├── vehicle/Crc16.h               # copy of firmware/.../Crc16.h, kept identical — see Part 3
│   │   ├── nav/NavigationEngine.h        # OSRM routing, see Part 11.2
│   │   ├── nav/MapMatcher.h              # snaps fused pose onto the route graph, see Part 11.3
│   │   ├── nav/RoadSurfaceProjector.h    # precision road-locked nav overlay, see Part 11.4
│   │   ├── system/SystemManager.h
│   │   ├── system/EventLog.h
│   │   └── common/Types.h
│   ├── src/                              # mirrors include/ tree, one .cpp per .h, plus:
│   │   └── main.cpp
│   ├── models/
│   │   ├── training/                     # Python: dataset prep, fine-tuning scripts
│   │   ├── onnx/                         # exported .onnx files (gitignored, regenerable)
│   │   └── engines/                      # built .engine files (gitignored, hardware-specific)
│   ├── config/
│   │   ├── camera_intrinsics.yaml
│   │   ├── camera_extrinsics.yaml
│   │   ├── lidar_camera_extrinsics.yaml
│   │   ├── vehicle_params.yaml
│   │   ├── decision_thresholds.yaml
│   │   └── road_projection.yaml          # RoadSurfaceProjector tuning, see Part 11.4
│   ├── scripts/
│   │   ├── setup_env.sh
│   │   ├── export_onnx.py
│   │   ├── build_tensorrt_engines.sh
│   │   ├── run_camera_calibration.py
│   │   └── attach_usb_devices.ps1        # Windows-side usbipd helper, see Part 2
│   └── test/
│       ├── unit/                         # GoogleTest, one file per subsystem
│       └── integration/                  # scripted end-to-end checks, see Part 13
│
├── tools/
│   └── bench_rig/
│       ├── log_actuator_bench.py         # captures force/current/response-time during Part 13 bench tests
│       └── replay_recorded_frames.py     # feeds recorded camera+LiDAR data through the pipeline offline
│
└── .github/                              # CI — see Part 13.5, not optional
    └── workflows/build.yml
```

**Rule for the agent building this:** create every directory and an empty placeholder file (or a stub with just a header guard and a `// TODO` comment matching the relevant Part number) in one pass at the start of Part 14, Phase 0, before writing any real logic. This keeps the structure stable while individual files are filled in phase by phase.

### 1.1 Progress log format

`docs/progress-log.md` is separate from `docs/decisions.md` — decisions.md stays a short index of "here's the one-line rationale for X," while progress-log.md is the detailed narrative, and the two are never merged into one file. Append one entry per discrete task, not per phase — a single phase in Part 14 will usually produce several entries. Shape:

```markdown
## 2026-09-18 — <short task title>
**Attempted:** <what was being worked on and why, in plain language>
**Built/changed:** <what was actually created or modified — files, behaviour, nothing left implicit>
**Reasoning:** <why this approach, especially anywhere a real choice was made>
**Problems hit:** <what went wrong and how it was resolved — omit the line if genuinely nothing did>
**Still open:** <anything left unfinished or deferred out of this task>
```

Write the entry as the task finishes, not retroactively in a batch at the end of a session — a log reconstructed from memory afterward loses exactly the detail this file exists to keep.

### 1.2 `.gitignore` contents

The first line of `.gitignore`, before anything else is added to the file, is:

```gitignore
# Claude Code's own local state — never committed, no exceptions.
.claude/
```

This is not ordering for its own sake: this repository is examined as the student's own work, and nothing that fingerprints it as AI-assisted belongs in version control — this rule is the same intent as the "Version control practice" rule in "How to use this document," applied to the filesystem instead of to commit metadata. Everything after that first line is ordinary project hygiene, and can be added in any order as the corresponding part of the build produces it:

```gitignore
# Build outputs
build/
host/build/
firmware/sensor_actuator_hub/.pio/

# Regenerable / hardware-specific ML artifacts (Part 7.3) — never portable across machines
host/models/onnx/
host/models/engines/

# Runtime logs (Part 5.4, Part 11.6)
logs/

# IDE
.idea/
.vscode/

# OS cruft
.DS_Store
Thumbs.db
```

If the agent building this ever finds itself about to `git add` something under `.claude/`, that is itself a signal to stop and re-check that `.gitignore` was actually applied — not a reason to add an exception for that one file.

---

## Part 2 — Development Environment Setup

Do this once, before Part 14 Phase 0. Every command below assumes Windows 11 host + WSL2 Debian, matching the environment already used for other CUDA/TensorRT work on this laptop.

### 2.1 WSL2 and networking mode

On Windows (PowerShell, admin), enable mirrored networking so the LiDAR's Ethernet interface is reachable from inside WSL2 later (Part 8):

```powershell
wsl --install -d Debian
wsl --set-default-version 2
notepad "$env:UserProfile\.wslconfig"
```
Contents of `.wslconfig`:
```ini
[wsl2]
networkingMode=mirrored
```
```powershell
wsl --shutdown
wsl -d Debian
```

### 2.2 usbipd-win (camera, LiDAR-Ethernet adapter, STM32 hub)

On Windows:
```powershell
winget install usbipd
usbipd list
usbipd bind --busid <busid-of-device>
usbipd attach --wsl --busid <busid-of-device>
```
Do this for the USB camera, the USB-to-Ethernet adapter, and the STM32 hub's USB-serial port. Devices must be re-attached after every WSL2 restart — write `attach_usb_devices.ps1` (see repo structure) to bind/attach all three in one command, and run it at the start of every dev session.

### 2.3 Base Debian packages

Inside WSL2 Debian:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git pkg-config curl wget unzip \
    libeigen3-dev libglm-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libspdlog-dev flatbuffers-compiler libflatbuffers-dev libsdl2-dev \
    libgoogle-glog-dev libgtest-dev python3-pip python3-venv v4l-utils
```

### 2.4 CUDA / cuDNN / TensorRT for Blackwell (sm_120)

Use the CUDA/TensorRT versions already validated on this laptop's WSL2 Debian instance for other CUDA work — do not mix major versions. Confirm after install:
```bash
nvcc --version
dpkg -l | grep tensorrt
python3 -c "import torch; print(torch.cuda.get_device_capability())"   # expect (12, 0) for Blackwell
```
If a fresh install is genuinely needed, follow NVIDIA's WSL2 CUDA installation guide for the Debian package path (not the `.run` installer) and install the TensorRT `.deb` release matching that CUDA version. Do not use `apt install nvidia-cuda-toolkit` — it installs an outdated version.

### 2.5 OpenCV built with CUDA support

```bash
git clone --branch 4.10.0 https://github.com/opencv/opencv.git
git clone --branch 4.10.0 https://github.com/opencv/opencv_contrib.git
mkdir opencv/build && cd opencv/build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D WITH_CUDA=ON -D WITH_CUDNN=ON -D OPENCV_DNN_CUDA=ON \
      -D CUDA_ARCH_BIN=12.0 \
      -D WITH_GSTREAMER=ON -D BUILD_EXAMPLES=OFF ..
make -j$(nproc)
sudo make install
sudo ldconfig
```
Note `CUDA_ARCH_BIN=12.0` — this is Blackwell's compute capability, not the Jetson Orin Nano's `8.7` from the original design. This is the single most common copy-paste mistake if reusing any old build scripts.

### 2.6 PCL (Point Cloud Library)

```bash
sudo apt install -y libpcl-dev
```
The Debian package is sufficient here (unlike OpenCV, PCL doesn't need a CUDA-enabled rebuild for the voxel/segmentation/clustering operations this project uses).

### 2.7 OSRM (offline navigation)

```bash
sudo apt install -y osrm-tools
```

### 2.8 Livox SDK2 (LiDAR)

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

### 2.9 STM32 firmware toolchain (PlatformIO)

```bash
python3 -m pip install --user platformio
pio pkg install --global --platform ststm32
```
`firmware/sensor_actuator_hub/platformio.ini` targets an STM32F4-series board with the FreeRTOS framework — see Part 4 for the exact `platformio.ini` contents.

### 2.10 Verify the environment before moving on

Exit criteria for this part: `nvcc --version` succeeds, a trivial CUDA kernel compiles and runs, `pkg-config --modversion opencv4` reports 4.10.0 with CUDA support (`cv2.getBuildInformation()` shows `CUDA: YES`), a USB camera bound via `usbipd` appears as `/dev/video0`, and `pio run` succeeds on an empty PlatformIO STM32 project. Do not proceed to Part 14 Phase 0 until all five are true.

---

## Part 3 — Shared Protocol and Message Schemas

This protocol is implemented **twice** — once in `firmware/sensor_actuator_hub/include/hub/Protocol.h` and once in `host/include/ar_drive_assist/vehicle/HubProtocol.h` — and the two implementations must be byte-for-byte identical. Do not let them drift; when one changes, change the other in the same commit. Consider generating both from a single source file in `tools/` if the agent building this finds it easier to keep in sync that way, but the guide below is the canonical spec either way.

### 3.1 Framing

Every message, in both directions, uses this frame:

```
+-----------+----------+-------------+---------------------+-----------+
| startByte | type     | length      | payload (length B)  | crc16     |
| 1 byte    | 1 byte   | 2 bytes LE  | variable, max 64 B  | 2 bytes LE|
| 0xAA      |          |             |                     |           |
+-----------+----------+-------------+---------------------+-----------+
```

CRC16 is CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) computed over `type + length + payload`. A receiver that gets a bad CRC discards the frame and does not act on it — this applies on both ends, but matters most on the hub, which must never act on a corrupted actuation command.

```cpp
// Protocol.h — identical in firmware and host
#pragma once
#include <cstdint>

namespace hub_protocol {

constexpr uint8_t START_BYTE = 0xAA;
constexpr size_t MAX_PAYLOAD = 64;

enum class MessageType : uint8_t {
    SENSOR_REPORT      = 0x01,  // hub -> host
    ACTUATION_COMMAND  = 0x02,  // host -> hub
    ACK_STATUS         = 0x03,  // hub -> host
    HEARTBEAT          = 0x04,  // host -> hub, required at >=10 Hz (see 3.4)
};

#pragma pack(push, 1)
struct SensorReport {
    uint32_t timestampMs;
    // GPS
    double   latitude;
    double   longitude;
    float    speedKph;
    uint8_t  gpsFixValid;       // 0/1
    // IMU
    float    accelX, accelY, accelZ;   // g
    float    gyroX, gyroY, gyroZ;      // deg/s
    float    headingDeg;
    // OBD-II
    float    obdSpeedKph;
    uint16_t obdRpm;
    uint8_t  obdBrakePedalActive;      // 0/1, from OBD PID if available, else 0xFF = unknown
    // Gesture
    uint8_t  gestureEvent;             // 0=none,1=left,2=right,3=up,4=down,5=hold
    // Hub state
    uint8_t  ignitionOn;               // 0/1
    uint8_t  killSwitchEngaged;        // 1 = actuator power physically cut
};

struct ActuationCommand {
    uint8_t  indicators;       // bit0 left, bit1 right, bit2 hazards
    uint8_t  lights;           // bit0 high beam, bit1 horn
    uint8_t  brakeRequest;     // 0-255 requested intensity; HUB clamps to configured ceiling
    uint16_t brakeDurationMs;  // hub still enforces its own max regardless of this value
    uint32_t hostTimestampMs;  // used by hub to detect stale/replayed commands
};

struct AckStatus {
    uint32_t timestampMs;
    uint8_t  lastCommandAccepted;   // 0/1
    uint8_t  actuatorFaultCode;     // 0 = none; see Appendix D for codes
    uint16_t appliedBrakeIntensity; // what the hub actually applied, post-ceiling
};
#pragma pack(pop)

} // namespace hub_protocol
```

### 3.2 CRC16 reference implementation

```cpp
// Crc16.h — identical in firmware and host
#pragma once
#include <cstdint>
#include <cstddef>

inline uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
```

### 3.3 Direction and rates

- Hub → host: one `SENSOR_REPORT` at 50 Hz (fixed rate, do not vary it — the host's EKF assumes a known dt).
- Host → hub: one `HEARTBEAT` at 10 Hz minimum, always, whether or not there's an actuation request pending. `ACTUATION_COMMAND` is sent only when the Decision/Arbiter has something to request; when it has nothing to request, it must explicitly send a zeroed `ActuationCommand` (all fields 0) rather than simply not sending anything — this makes "no request" and "link dead" distinguishable on the hub side.
- Hub → host: `ACK_STATUS` after every accepted or rejected `ACTUATION_COMMAND`.

### 3.4 The watchdog contract (read this before implementing either side)

The hub's watchdog task treats **either** of the following as link loss and immediately releases all actuation, resets indicators/lights off, and refuses to re-arm until a fresh valid `HEARTBEAT` is received:
1. No valid frame (any type) received from the host for more than 200 ms.
2. A `SensorReport`'s own `timestampMs` implies the hub's send loop itself has stalled for more than 200 ms (a self-check, catches a hung sensor task even if comms are fine).

This is implemented entirely in firmware (Part 4) and must not depend on any host-side code path. Test it by physically unplugging the USB cable mid-session (Part 13) — the correct outcome is the actuator releasing within 200 ms with no host code running at all.

### 3.5 Internal host message types (FlatBuffers)

Between host subsystems (camera → inference → fusion → decision → renderer), use FlatBuffers schemas rather than raw structs, since several of these cross thread boundaries via the message bus (Part 5.3). Define one schema file per message type in `host/include/ar_drive_assist/common/schemas/`:

```fbs
// detections.fbs
namespace ar_drive_assist.schema;

table BoundingBox {
  x: float; y: float; w: float; h: float;
  class_id: int; confidence: float; track_id: int;
}

table DetectionFrame {
  timestamp_ms: ulong;
  boxes: [BoundingBox];
  lane_points_left: [float];
  lane_points_right: [float];
  depth_map_ref: ulong;   // handle into the depth-map ring buffer, not inlined
  mask_ref: ulong;        // handle into the object-mask buffer (YOLOv8m-seg, Part 7.1), not inlined;
                          // box i's mask is instance i of that frame (amended 2026-09-25)
  signs: [BoundingBox];   // road-sign detector boxes; class_id indexes its 29 sign classes, not the
                          // object classes of `boxes` (amended 2026-09-26)
}
root_type DetectionFrame;
```

```fbs
// vehicle_pose.fbs
namespace ar_drive_assist.schema;

table VehiclePose {
  timestamp_ms: ulong;
  x: double; y: double; heading_deg: float;
  speed_kph: float; yaw_rate: float;
}
root_type VehiclePose;
```

```fbs
// actuation_request.fbs
namespace ar_drive_assist.schema;

enum RequestType : byte { NONE = 0, INDICATE_LEFT, INDICATE_RIGHT, HAZARDS, HORN, BRAKE }

table ActuationRequest {
  timestamp_ms: ulong;
  type: RequestType;
  intensity: ubyte;      // meaningful only for BRAKE
  reason_code: ubyte;    // see Appendix D — for logging/report evidence
}
root_type ActuationRequest;
```

```fbs
// road_projected_route.fbs — RoadSurfaceProjector's output, consumed by ArRenderer (Part 10.3)
namespace ar_drive_assist.schema;

table RoadOverlayPoint {
  image_x: float; image_y: float;
  world_distance_m: float;
  on_measured_surface: bool;   // true = locked to real LiDAR ground geometry, false = flat-ground fallback
}

table RoadProjectedRoute {
  timestamp_ms: ulong;
  polyline: [RoadOverlayPoint];
  next_turn_instruction: string;
  next_turn_distance_m: float;
}
root_type RoadProjectedRoute;
```

Generate C++ headers with `flatc --cpp` as part of the CMake build (add a custom command in `host/CMakeLists.txt`, do not commit generated headers).

---

## Part 4 — Firmware: Sensor & Actuator Hub (STM32 / FreeRTOS)

### 4.1 Board and toolchain

Target: an STM32F4-series board (e.g. STM32F407VG "Black Pill"/discovery-class board — any F4 with at least 2 I2C, 2 UART, 1 CAN or a spare UART for OBD-II, and enough timer channels for PWM actuator control is sufficient). `firmware/sensor_actuator_hub/platformio.ini`:

```ini
[env:stm32f4_hub]
platform = ststm32
board = blackpill_f411ce      ; substitute for the exact board on hand
framework = arduino            ; or stm32cube, if the agent prefers HAL directly
lib_deps =
    FreeRTOS
    adafruit/Adafruit BNO08x
    mikalhart/TinyGPSPlus
monitor_speed = 115200
build_flags = -DUSB_CDC_ON_BOOT=1
```

### 4.2 Pin/peripheral allocation (record actual pins used in `Config.h`, this is a template)

| Peripheral | Bus | Notes |
|---|---|---|
| BNO085 IMU | I2C1 | |
| u-blox GPS | UART2 | |
| ELM327 OBD-II | UART3 | or Bluetooth SPP bridged to a UART, agent's choice, document in `docs/decisions.md` |
| APDS-9960 gesture | I2C1 (shared) | |
| Indicator/hazard/horn/beam relays | 4–6 GPIO, opto-isolated relay board | active-low typical, confirm against the board used |
| Brake actuator H-bridge (PWM + direction + current-sense) | 1 timer channel PWM, 2 GPIO direction, 1 ADC channel current sense | |
| Kill-switch sense line | 1 GPIO input | **read-only on the hub — the switch itself cuts actuator power in hardware, this line only tells firmware it happened, it is not what enforces the cut** |
| USB-CDC to host | USB | carries the Part 3 protocol |

### 4.3 FreeRTOS task layout

```cpp
// main.cpp
void setup() {
    initDrivers();               // I2C/UART/PWM/ADC init, see 4.4
    xTaskCreate(SensorTask,    "sensor",   4096, nullptr, 3, nullptr);
    xTaskCreate(CommsTask,     "comms",    4096, nullptr, 3, nullptr);
    xTaskCreate(ActuationTask, "actuate",  2048, nullptr, 4, nullptr);  // higher priority than sensor/comms
    xTaskCreate(WatchdogTask,  "watchdog", 1024, nullptr, 5, nullptr);  // highest priority
    IWDG_Init(/* timeout */ 500 /* ms, hardware backstop above the 200ms software one */);
    vTaskStartScheduler();
}
```

Priority ordering is deliberate: `WatchdogTask` > `ActuationTask` > `{SensorTask, CommsTask}`. The watchdog must always be able to preempt and act, even if the comms task is stuck in a blocking read.

### 4.4 Driver interfaces

Each driver is a small class with `init()`, a read/poll method, and nothing else — no driver reaches directly into another driver's state.

```cpp
// drivers/ImuDriver.h
class ImuDriver {
public:
    bool init();
    struct Sample { float ax, ay, az, gx, gy, gz, headingDeg; uint32_t timestampMs; };
    bool read(Sample& out);   // non-blocking, returns false if no new sample
};

// drivers/GpsDriver.h
class GpsDriver {
public:
    bool init();
    struct Fix { double lat, lon; float speedKph; bool valid; uint32_t timestampMs; };
    bool read(Fix& out);
};

// drivers/ObdDriver.h
class ObdDriver {
public:
    bool init();
    struct Reading { float speedKph; uint16_t rpm; uint8_t brakePedalActive; bool valid; };
    bool read(Reading& out);   // polls PID 0x0D (speed), 0x0C (RPM); brakePedalActive via
                               // mode-1 PID if the vehicle's OBD-II exposes it, else always 0xFF
};

// drivers/GestureDriver.h
class GestureDriver {
public:
    bool init();
    enum class Event : uint8_t { NONE, LEFT, RIGHT, UP, DOWN, HOLD };
    Event poll();
};

// drivers/RelayDriver.h
class RelayDriver {
public:
    void init();
    void setIndicators(bool left, bool right, bool hazards);
    void setLights(bool highBeam, bool horn);
    void allOff();   // called by WatchdogTask and by ActuationTask on any fault
};

// drivers/BrakeActuatorDriver.h
class BrakeActuatorDriver {
public:
    void init();
    // request: 0-255. Driver itself clamps to kMaxSafeIntensity regardless of input —
    // this is the SECOND independent ceiling (the arbiter/host is the first, advisory only).
    void apply(uint8_t request, uint16_t durationMs);
    void release();               // immediately zero PWM, set direction to neutral
    float readCurrentAmps();      // for stall/overcurrent detection
    static constexpr uint8_t kMaxSafeIntensity = 90;   // tune during Part 13 bench testing; NEVER raise without a bench re-test
};
```

### 4.5 ActuationTask logic

```cpp
void ActuationTask(void*) {
    for (;;) {
        ActuationCommand cmd;
        if (popLatestCommand(cmd) && systemArmed()) {
            if (brakeActuator.readCurrentAmps() > kOvercurrentThreshold) {
                brakeActuator.release();
                relay.allOff();
                setFaultCode(FAULT_OVERCURRENT);
            } else {
                relay.setIndicators(cmd.indicators & 1, cmd.indicators & 2, cmd.indicators & 4);
                relay.setLights(cmd.lights & 1, cmd.lights & 2);
                brakeActuator.apply(cmd.brakeRequest, cmd.brakeDurationMs);  // clamped internally
            }
        } else {
            brakeActuator.release();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

`systemArmed()` is `false` whenever: the kill switch line reads engaged, the watchdog has flagged link loss, or an overcurrent fault is latched (requires a physical power cycle to clear — do not auto-clear an overcurrent fault in software).

### 4.6 WatchdogTask logic

```cpp
void WatchdogTask(void*) {
    for (;;) {
        if (millisSinceLastHostFrame() > 200 || killSwitchLine() == ENGAGED) {
            brakeActuator.release();
            relay.allOff();
            disarmSystem();
        }
        IWDG_Refresh();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```

### 4.7 Build, flash, and bring-up verification

```bash
cd firmware/sensor_actuator_hub
pio run                       # build
pio run -t upload             # flash over USB/ST-Link
pio device monitor            # serial console for debug prints
```

Exit criteria for this part: with the hub powered and connected to a PC (any PC, host app not required yet), `SensorReport` frames arrive at 50 Hz and decode correctly in a throwaway Python script using `pyserial` + the same framing as `Protocol.h`; sending a hand-crafted `ActuationCommand` from that script toggles a bench-wired relay; physically unplugging the USB cable causes the bench-wired relay to turn off within 200 ms without any command being sent to do so. Do this on the bench, with no vehicle and no brake actuator connected, before touching Part 15.

---

## Part 5 — Host Application Architecture

### 5.1 Process and threading model

One process, one thread per subsystem, connected by a lock-free-ish message bus (a set of single-producer/single-consumer ring buffers is sufficient — do not reach for a general pub/sub framework, it's unnecessary complexity for a fixed, known set of subsystems). Threads:

1. `CameraThread` — captures frames, pushes to `frameBus`.
2. `InferenceThread` — consumes `frameBus`, runs TensorRT models, pushes `DetectionFrame` to `detectionBus`.
3. `LidarThread` — consumes raw LiDAR packets (its own capture loop, independent rate from camera), pushes processed point cloud to `lidarBus`.
4. `FusionThread` — consumes IMU/GPS/OBD (from `VehicleInterface`, which itself reads the hub over serial), `lidarBus`, and `detectionBus`; produces `VehiclePose` and a fused scene model.
5. `DecisionThread` — consumes the fused scene model and pose, runs the reckless-driving/hazard classifiers, produces `ActuationRequest` messages, sent to `VehicleInterface` for transmission to the hub.
6. `RenderThread` — consumes `frameBus` + `detectionBus` + fused scene model + `navOverlayBus`, composites, presents via `DisplaySink`.
7. `VehicleInterfaceThread` — owns the USB-serial connection to the hub; the only thread that reads/writes `HubProtocol` frames directly (see Part 11).
8. `NavigationThread` — owns `NavigationEngine`, `MapMatcher`, and `RoadSurfaceProjector` (Part 11.2–11.4) as one sequential pipeline; consumes `VehiclePose` (from `FusionThread`), the ground-plane model (from `LidarThread`, Part 8.1), and lane points (from `detectionBus`); produces `RoadProjectedRoute` onto a new `navOverlayBus`. Runs at the LiDAR's ground-plane refresh rate — no benefit projecting the overlay faster than its geometry input updates.
9. Main thread — `SystemManager`: starts/stops the above, owns the config load, owns the `EventLog`.

### 5.2 Top-level CMake

```cmake
# host/CMakeLists.txt
cmake_minimum_required(VERSION 3.25)
project(ar_drive_assist LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4.10 REQUIRED)
find_package(PCL 1.13 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glm REQUIRED)
find_package(spdlog REQUIRED)
find_package(SDL2 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Flatbuffers REQUIRED)
find_package(GTest REQUIRED)
include(${CMAKE_SOURCE_DIR}/../cmake/FindTensorRT.cmake)

file(GLOB_RECURSE SOURCES src/*.cpp)
add_executable(ar_drive_assist ${SOURCES})
target_include_directories(ar_drive_assist PRIVATE include)
target_link_libraries(ar_drive_assist PRIVATE
    ${OpenCV_LIBS} ${PCL_LIBRARIES} Eigen3::Eigen glm::glm
    spdlog::spdlog SDL2::SDL2 CUDA::cudart TensorRT::TensorRT
    flatbuffers osrm
)

enable_testing()
add_subdirectory(test)
```

`cmake/FindTensorRT.cmake` should locate `nvinfer`, `nvinfer_plugin`, and `nvonnxparser` by searching the standard `/usr/lib/x86_64-linux-gnu` and `/usr/include/x86_64-linux-gnu` paths that the `.deb` TensorRT install uses on Debian — write it as a straightforward `find_library`/`find_path` pair, no need for anything fancier.

### 5.3 Message bus

```cpp
// common/RingBuffer.h
template <typename T, size_t N>
class RingBuffer {
public:
    bool push(const T& item);   // false if full — caller decides drop-oldest vs drop-newest policy
    bool pop(T& out);           // false if empty
private:
    std::array<T, N> buf_;
    std::atomic<size_t> head_{0}, tail_{0};
};
```
Policy: `frameBus` and `lidarBus` drop the oldest frame on overflow (real-time data, staleness is worse than a gap). `ActuationRequest`s are never dropped silently — if the bus to `VehicleInterface` is full, that is itself logged as an `EventLog` fault, because a dropped brake request is a safety-relevant event, not just a performance hiccup.

### 5.4 `main.cpp` skeleton

```cpp
int main(int argc, char** argv) {
    auto config = SystemManager::loadConfig("config/");
    EventLog log("logs/session.log");
    SystemManager mgr(config, log);

    mgr.start<CameraPipeline>();
    mgr.start<MlInferenceEngine>();
    mgr.start<LidarProcessor>();
    mgr.start<SensorFusion>();
    mgr.start<DecisionArbiter>();
    mgr.start<NavigationEngine>();   // bundles MapMatcher + RoadSurfaceProjector, Part 11.2–11.4
    mgr.start<ArRenderer, WindowedSink>();
    mgr.start<VehicleInterface>();

    mgr.runUntilShutdown();   // blocks; Ctrl+C or window close triggers ordered shutdown
    return 0;
}
```

`SystemManager::runUntilShutdown` must, on any shutdown path (normal or crash-triggered via a signal handler), request `VehicleInterface` to send one final zeroed `ActuationCommand` before closing the serial port — this is a deliberate belt-and-braces measure on top of the hub's own watchdog, not a substitute for it.

---

## Part 6 — Camera Pipeline

### 6.1 Interface

```cpp
// camera/CameraPipeline.h
class CameraPipeline {
public:
    bool init(const CameraConfig& cfg);   // device path, resolution, target fps
    void run();                           // loop: capture -> undistort -> push to frameBus
    void stop();
private:
    cv::VideoCapture cap_;
    cv::Mat cameraMatrix_, distCoeffs_;    // loaded from config/camera_intrinsics.yaml
};
```

### 6.2 Implementation notes

- Open via GStreamer pipeline string through OpenCV (`cv::VideoCapture(gstPipeline, cv::CAP_GSTREAMER)`) rather than the plain V4L2 backend — this gives more reliable control over resolution/format negotiation for a UVC device passed through `usbipd-win`.
- Target capture resolution: 2560×1440 if the camera and USB/IP link sustain it; **the pipeline must measure and log actual sustained fps during Part 14 Phase 1's exit check**, and fall back to 1920×1080 if 2K drops frames — do not hardcode 2K without verifying throughput first (this was flagged as the primary WSL2 risk in Part 2).
- Undistortion uses `cv::undistort()` with the intrinsics loaded from `config/camera_intrinsics.yaml` (produced in Part 12.1) — done once per frame, on CPU is fine at this stage; move to a CUDA `cv::cuda::undistort` only if profiling in Part 14 shows it's the bottleneck.
- Push undistorted frames (plus a monotonic timestamp) into `frameBus`; do not do any ML or rendering work in this thread.

---

## Part 7 — ML Inference Engine and Model Pipeline

### 7.1 Models

*Amended 2026-09-25.* The original table's lane and depth input sizes matched no released model.
Object detection became the segmentation variant, so that hazard overlays can follow object
outlines (Part 10.3). A fourth model detects road signs. The reasons are in `docs/decisions.md`
(Phase 5, Phase 5 extension, and "Guide amendment: GPU overlay rendering").

| Model | Architecture | Input | Output | Target FPS |
|---|---|---|---|---|
| Object Detection | YOLOv8m-seg (COCO) | 1280×736 RGB | boxes + class (vehicle, pedestrian, cyclist, sign, obstacle) + a per-object mask | 30+ |
| Road-Sign Detection | YOLOv8s, fine-tuned on MTSD | 1280×736 RGB | boxes + 29 sign classes (incl. speed-limit values) | 30+ |
| Lane Detection | Ultra-Fast-Lane-Detection-v2 (CULane, ResNet-18) | 1600×320 RGB (a CULane-shaped band) | up to 4 lane lines | 30+ |
| Depth Estimation | MiDaS v2.1 Small | 448×256 RGB | dense relative depth | 15–20 |
| Reckless-Driving Classifier | Temporal CNN + LSTM | 30-frame track history | threat level | 10+ per tracked vehicle |

The export commands and shapes in 7.3 are superseded by `host/scripts/export_onnx.py` and
`host/scripts/build_tensorrt_engines.sh`, which are the source of truth for names and shapes.

### 7.2 Getting to a working model without training from scratch

Given the timeline, **do not train YOLOv8m, the lane detector, or MiDaS from scratch** — start from each model's publicly released pretrained weights and fine-tune only if a specific failure mode shows up in Part 13 testing (e.g., poor pedestrian detection at dusk). The one model that does need building from the ground up is the reckless-driving classifier, since it's project-specific; even there, start with a small labeled dataset from your own recorded test drives (Part 13's field-test recordings) rather than trying to source a large external dataset first — a simple rule-based threat classifier (hard thresholds on lateral acceleration variance, following distance, and closing speed) is an acceptable placeholder for the first working end-to-end demo, with the learned classifier as a stretch goal if time remains.

### 7.3 Export and engine build pipeline

```bash
# scripts/export_onnx.py — run once per model, on the host (same machine as inference now)
python3 export_onnx.py --model yolov8m --weights yolov8m.pt --out models/onnx/yolov8m.onnx
python3 export_onnx.py --model lanenet  --weights ufld_v2.pth --out models/onnx/lanenet.onnx
python3 export_onnx.py --model midas    --weights midas_small.pt --out models/onnx/midas.onnx
```
```bash
# scripts/build_tensorrt_engines.sh
trtexec --onnx=models/onnx/yolov8m.onnx  --saveEngine=models/engines/yolov8m.engine  --fp16 --shapes=images:1x3x640x640
trtexec --onnx=models/onnx/lanenet.onnx  --saveEngine=models/engines/lanenet.engine  --fp16 --shapes=input:1x3x288x800
trtexec --onnx=models/onnx/midas.onnx    --saveEngine=models/engines/midas.engine    --fp16 --shapes=input:1x3x384x384
```
No `CUDA_ARCH_BIN`-style flag is needed here — `trtexec` builds for whatever GPU it's run on, which is now the same RTX 5060 the app runs on, so **engines built on this machine are only valid on this machine** — do not commit `.engine` files to git (they're already gitignored per Part 1's structure); regenerate them on any new machine.

### 7.4 Interface

```cpp
// inference/TrtEngine.h — thin wrapper, one instance per model
class TrtEngine {
public:
    bool load(const std::string& enginePath);
    void infer(const cv::cuda::GpuMat& input, std::vector<float>& output);  // async via CUDA stream
private:
    nvinfer1::IRuntime* runtime_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;
    cudaStream_t stream_;
};

// inference/MlInferenceEngine.h
class MlInferenceEngine {
public:
    bool init(const InferenceConfig& cfg);   // loads all three TrtEngines
    void run();   // loop: pop frameBus -> run 3 models -> push DetectionFrame to detectionBus
};
```

Run the three models on independent CUDA streams so they overlap rather than serialize — this matters more on a discrete desktop GPU with real concurrent-kernel support than it did on the Orin Nano.

---

## Part 8 — LiDAR Processing and Sensor Fusion

### 8.1 LiDAR capture and processing

```cpp
// lidar/GroundPlaneModel.h — published output, NOT an internal throwaway (see note below)
struct GroundPlaneModel {
    Eigen::Vector4f planeCoefficients;              // ax+by+cz+d=0, in vehicle frame
    std::vector<Eigen::Vector3f> nearFieldPatch;    // actual measured ground points within
                                                     // the drivable corridor, out to LiDAR range
    float maxValidRangeM;                           // beyond this, no measured surface exists
    uint64_t timestampMs;
};

// lidar/LidarProcessor.h
class LidarProcessor {
public:
    bool init(const LidarConfig& cfg);   // Livox SDK2 handle, static IP per Appendix C
    void run();   // loop: pull point cloud -> voxel downsample -> ground segmentation
                  // -> DBSCAN clustering -> road-anomaly pass -> push SceneCloud to lidarBus
                  //                                            -> push GroundPlaneModel to groundPlaneBus
};
```
Use `pcl::VoxelGrid` for downsampling, `pcl::SACSegmentation` (plane model) for ground removal, and `pcl::EuclideanClusterExtraction` for obstacle clustering — this is standard PCL usage, no custom kernels needed for the first working version. Road-surface anomaly detection (potholes/bumps) can be a simple "points below the fitted ground plane by more than X cm, within the drivable corridor" check to start; a PointPillars-style learned detector is future work, not required for the FYP demo.

**Important change from the original design:** the fitted ground plane used to be an internal step, discarded once obstacle clustering was done with it. It is no longer discarded — publish it as `GroundPlaneModel` on a new `groundPlaneBus`. `RoadSurfaceProjector` (Part 11.4) depends directly on this being real, current, measured geometry — it is what lets the navigation overlay stay locked to the actual road surface through slopes and dips at close range, instead of assuming a flat road. Do not skip this change; a `LidarProcessor` that still throws the plane away will silently force `RoadSurfaceProjector` onto the flat-ground fallback for every point, defeating the precision requirement without any obvious error.

### 8.2 Sensor fusion (EKF)

```cpp
// fusion/SensorFusion.h
class SensorFusion {
public:
    void predict(float dt);                          // IMU-driven prediction step, 100 Hz
    void updateGps(const GpsFix& fix);
    void updateObd(const ObdReading& reading);
    void updateLidarOdometry(const Eigen::Isometry3d& delta);  // optional refinement, not required for v1
    VehiclePose currentPose() const;
private:
    Eigen::VectorXd state_;      // [x, y, heading, speed, yawRate]
    Eigen::MatrixXd covariance_;
};
```
A standard 5-state EKF (position x/y, heading, speed, yaw rate) fed by IMU prediction and GPS/OBD-speed correction is sufficient — do not add LiDAR-odometry-based correction unless GPS accuracy proves inadequate in Part 13 testing; it's a real accuracy improvement but a nontrivial addition, and the project's actuation logic (Part 9) mainly needs relative distance/closing-speed to objects in the camera/LiDAR frame, not centimeter-accurate global position.

### 8.3 3D scene reconstruction: mask-based camera–LiDAR fusion

*Amended 2026-09-25: association uses each object's segmentation mask, not a box or a cluster
centroid. See `docs/decisions.md`, "Mask-based LiDAR fusion and startup extrinsic check".*

```cpp
// scene/SceneReconstruction.h
class SceneReconstruction {
public:
    SceneModel merge(const DetectionFrame& detections, const ObjectMasks& masks,
                     const SceneCloud& lidarCloud, const VehiclePose& pose,
                     const ExtrinsicState& extrinsics);   // from ExtrinsicMonitor, Part 12.2.1
};
```

**Idea.** Project every LiDAR point into the camera image, using the LiDAR→camera extrinsics
(Part 12.2 / 12.2.1) and the camera intrinsics (Part 12.1). The points that land inside an
object's YOLOv8m-seg mask belong to that object. Each object then has its class and outline from
the camera, and real 3D points from the LiDAR: distance, 3D position, the point where it meets the
road (where Part 10.3's barriers and ground rings are placed), and, tracked frame to frame, its
velocity for TTC. This is the "frustum"/"point-painting" family of fusion methods.

**Why masks, not boxes:** a box around a pedestrian is mostly road and background. Picking points
by box mixes the pedestrian's distance with the wall's behind them. A mask selects the object's own
points, which matters most for exactly the thin or partly hidden objects that are hazards
(pedestrians, cyclists, animals, a car behind a car).

**Rules (each exists because of a specific failure mode):**
1. **Motion-compensate before projecting.** The Livox pattern fills in over ~100 ms, so points are
   accumulated over a short window. Every point is corrected by the ego-motion between its own
   timestamp and the camera frame's (from `SensorFusion`). Otherwise points smear sideways off the
   object.
2. **Erode the mask a few pixels before selecting points.** YOLOv8-seg masks are computed at ¼
   resolution and upsampled, so edges are soft, and a small extrinsic error shifts projections
   outward. Erosion stops background points leaking in along the silhouette.
3. **Use the nearest dominant depth cluster, never the mean.** Among an object's selected points,
   take the nearest well-populated depth mode. One stray background point must not move a
   pedestrian 10 m away.
4. **Depth-test for parallax.** The LiDAR and camera sit in different places, so some points the
   LiDAR sees are hidden from the camera. Keep only the nearest point per small image cell before
   assigning points to objects.
5. **Report "no range" honestly.** If too few points survive (distant or small objects), the
   object has no LiDAR range this frame, and that is said, not guessed. MiDaS relative depth,
   scaled to metres using the frame's other LiDAR-matched objects, can give an *estimate*. It is
   flagged as such and is never used for Part 9.3 braking.
6. **Unexplained LiDAR obstacles are still obstacles.** A LiDAR cluster in the ego corridor that
   no camera mask accounts for becomes an `UNKNOWN` obstacle in the `SceneModel`. It is shown as a
   hazard and is eligible for Part 9.3 collision logic. The camera can add information, but it can
   never veto something the LiDAR physically measures.
7. **Signs (box-only detector):** points inside a sign's box that return high LiDAR intensity
   (signs are retroreflective) at a single depth give the sign's position. This places the stop
   barrier and the other sign behaviours of `docs/architecture/ar-overlay-design.md`.

**Tests (Part 13.1):** `test_mask_lidar_fusion.cpp` checks each rule with a synthetic scene: a
known camera, known extrinsics, a synthetic mask and a point cloud with a background wall, a stray
point and occluded points. Expected ranges and assignments are computed by hand, as for the
decoders.

---

## Part 9 — Decision / Arbiter and Reckless-Driving Detection

This is the subsystem the whole safety design in Part 0 exists to constrain. Read Part 0's non-negotiables again before writing any code here.

### 9.1 Multi-object tracker and motion prediction

*Amended 2026-09-25 (see `docs/decisions.md`, "Motion prediction").* The original Appendix Q.7
tracker propagated each track with `position += velocity * dt` and no uncertainty. It is replaced
by an interacting-multiple-model (IMM) tracker that estimates each object's motion **with its
uncertainty** and predicts it forward. The loop structure of the original is kept, because it is
sound:

> predict every track → Mahalanobis cost matrix → Hungarian assignment → gate → update → create
> unassigned → drop stale tracks → history.

What changes is *how* tracks are predicted and updated. The Mahalanobis distance now uses each
track's real innovation covariance $S$, which the original's `mahalanobis()` had no way to compute.

```cpp
// safety/MultiObjectTracker.h
class MultiObjectTracker {
public:
    // Measurements arrive asynchronously (camera 30 Hz, LiDAR 10 Hz), each at its own timestamp.
    void update(const std::vector<ObjectMeasurement>& measurements, const VehiclePose& egoPose);
    std::vector<Track> tracks() const;   // state, covariance, IMM model probabilities, class, age
};

// safety/MotionPredictor.h
class MotionPredictor {
public:
    // Distribution of a track's future position at each horizon (default 0.5/1.0/1.5/2.0 s):
    // mean + covariance, and the UKF sigma points for sampling.
    PredictedMotion predict(const Track& t, std::span<const float> horizonsS) const;
    EgoPath predictEgo(const VehiclePose& pose, const RoadProjectedRoute* route) const;
    CollisionRisk risk(const Track& t, const EgoPath& ego) const;   // CPA + probability, 9.1.2
};
```

**Frames.** Tracking happens in a local **world-fixed** frame: each measurement is transformed with
the EKF pose (Part 8.2). There a parked car has zero velocity, and a pedestrian's velocity is their
real walking speed. Results are transformed back into the vehicle frame for risk and display.
Tracking in the vehicle frame would make every parked car "approach" at the ego speed.

**What is measured.** Measurements come from Part 8.3's mask-based fusion, never from raw box
centres:
- **Vehicles:** an L-shape (rectangle) fit to the object's LiDAR points gives a reference point
  (the rear-axle / near corner) *and* a heading.
- **Displacement between scans:** registration (ICP) of the object's point cluster against its
  previous cluster. It matches the shape, so it measures true motion.
- **Why not the centroid:** the centroid of the visible points moves whenever the *visible part*
  changes (a car turning, emerging from behind another). That produces phantom lateral velocity,
  which would falsely trip 9.2's swerving rule.
- **Camera-only objects** (no LiDAR range this frame; Part 8.3 rule 5) contribute a bearing-only
  measurement. It narrows direction, not range.
- **De-skewing:** Livox points carry per-point timestamps. Points are de-skewed for ego-motion
  (8.3 rule 1) and, once a track exists, for the object's own estimated motion. That second
  correction is **damped** (a fraction of the estimated velocity, and only for well-converged
  tracks), so a wrong velocity cannot reinforce itself.

**Motion models, combined by an IMM:**

| Model | State | For | Notes |
|---|---|---|---|
| Constant velocity (CV) | $[p_x, p_y, v_x, v_y]$ | pedestrians, animals, unknown obstacles | linear Kalman filter; white-noise acceleration $Q=\sigma_a^2 GG^\top$, $G=[\tfrac{\Delta t^2}{2},\Delta t]^\top$ per axis |
| Constant turn rate & velocity (CTRV) | $[p_x, p_y, v, \psi, \omega]$ | cars, matatus, trucks, boda-bodas | unscented Kalman filter (the trigonometric model linearises badly in sharp turns); **explicit $\omega\to 0$ branch** falling back to straight-line motion, to avoid dividing by zero |
| Stopping | CV + strong deceleration prior | vehicles | captures sudden stops (e.g. matatus at unofficial stages) |

CTRV prediction:
$p_x' = p_x + \tfrac{v}{\omega}\big(\sin(\psi+\omega\Delta t)-\sin\psi\big)$,
$p_y' = p_y + \tfrac{v}{\omega}\big(\cos\psi-\cos(\psi+\omega\Delta t)\big)$,
$\psi'=\psi+\omega\Delta t$.

The IMM runs the models in parallel, mixes them each step through a Markov switching matrix, and
keeps a probability per model. **A rising stopping-model probability is itself an early warning**
that the vehicle ahead is braking, before its speed has visibly dropped.

**Per-class process noise** (`config/motion_prediction.yaml`, never hard-coded) encodes each road
user's unpredictability: high lateral noise for boda-bodas, high acceleration noise for
pedestrians, very high for livestock. It is tuned from data (9.1.3), not guessed.

**Track lifecycle:**
- **Birth:** a track exists only after several consecutive associated hits (configurable).
- **Coasting:** a lost track may run on prediction for a bounded time, and is marked
  `PREDICTED_ONLY` while it does. An occluded pedestrian behind a parked matatu is exactly when
  prediction matters most, and it is displayed as predicted, not as seen.
- **Death:** the track is dropped after that bound.

**Timing.** Measurements are applied at their own timestamps. A measurement that arrives after a
later one has been applied (an *out-of-sequence measurement*; the LiDAR path is slower than the
camera path) is handled by rewinding to a short buffer of past states and re-applying, never by
dropping or misordering it.

#### 9.1.1 What can honestly be predicted

Two limits shape every use of this subsystem:
- **Velocity is noisy.** Differencing positions with noise $\sigma_p$ over $\Delta t$ gives
  $\sigma_v\approx\sqrt2\,\sigma_p/\Delta t$. With $\sigma_p\approx0.1$ m at 10 Hz, that is
  ~1.4 m/s per frame. The filter averages it down, trading noise for lag. That is why better
  measurements (registration, L-shape fits) matter more than filter tuning.
- **Uncertainty grows roughly with the square of the horizon:**
  $\sigma_{pos}(t)\approx\sqrt{\sigma_v^2t^2+\tfrac14\sigma_a^2t^4}$. For a pedestrian
  ($\sigma_v=0.5$ m/s, $\sigma_a=2$ m/s²): ~0.35 m at 0.5 s, ~1.1 m at 1 s, ~4.1 m at 2 s.

Physics predicts usefully to ~1–1.5 s. Beyond that it is *intent*, which motion cannot know.
Predictions are therefore always kept and shown as **distributions (regions), never single
lines**.

#### 9.1.2 Collision risk from prediction: closest point of approach

Time to collision (range ÷ closing speed) only sees objects coming straight at the car. It is
blind to **crossing** traffic and **cut-ins**. For warnings and the display, risk uses the
**closest point of approach** (as in marine collision-avoidance radar). With relative position
$\mathbf r$ and relative velocity $\mathbf v$ (object minus ego, both from the predicted paths):

$$t^*=-\frac{\mathbf r\cdot\mathbf v}{\lVert\mathbf v\rVert^2},\qquad d^*=\lVert\mathbf r+\mathbf v\,t^*\rVert$$

The risk is high when $d^*$ is below a safety radius (ego half-width + object half-width + margin)
and $0<t^*<$ horizon. With uncertainty, the object's predicted distribution (its sigma points, or a
few hundred samples) is propagated against the **ego path**. The fraction that comes within the
safety radius inside the horizon is the **collision probability**.

The ego path is predicted from speed and yaw rate (Part 8.2), and from the route when navigating.
Otherwise "in the path" would mean "straight ahead", which is wrong on every curve.

The risk level $r$ that drives the hazard display (`docs/architecture/ar-overlay-design.md` §3.1)
becomes the maximum of the TTC-based value, the collision probability, and the 9.2 flag floors.

#### 9.1.3 Proving the uncertainty is honest

A predicted region is only useful if it contains the truth as often as it claims. Two checks:
- **NIS** (normalised innovation squared, $\nu^\top S^{-1}\nu$) must follow a chi-square
  distribution. If it is too large on average, the process noise is too small (overconfident). If
  it is too small, the filter is over-cautious. The per-class noise in
  `config/motion_prediction.yaml` is tuned until NIS passes, on synthetic data and then on
  recorded drives.
- **ADE / FDE** (average and final displacement error) at 1, 2 and 3 s on recorded drives,
  against where objects actually went. These are report numbers, alongside the NIS results.

#### 9.1.4 Scope

This is physics-based prediction (IMM), because it is verifiable. A learned trajectory predictor
(e.g. a transformer trained on nuScenes/Argoverse) is a **display-only stretch goal**. It would be
benchmarked against this baseline on the project's own recordings, and it never becomes an input to
actuation, for the same auditability reason as 9.3's rule on raw ML classifier output. Map-aware
prediction (vehicles following their lane, using lane detections and OSM) is the next step if time
allows. The IMM model probabilities and turn-rate statistics are also the natural input features
for the Part 7.1 reckless-driving classifier.

### 9.2 Reckless-driving / hazard classification

Start with the rule-based version described in Part 7.2:
- **Tailgating**: following distance (from the tracked lead vehicle's LiDAR range) below a speed-dependent threshold sustained for >1s.
- **Erratic speed**: variance of a tracked vehicle's speed over its 30-frame history above a threshold.
- **Lane swerving**: lateral position variance above a threshold, computed from the 9.1 track state (L-shape reference point, CTRV heading and turn rate), never from a centroid, whose visible-part shifts fake lateral motion.
- **Sudden braking ahead** (warning only): the 9.1 IMM stopping-model probability of the lead vehicle above a threshold.
- **Crossing / cut-in risk** (warning only): 9.1.2 closest point of approach and collision probability.
- **Forward collision risk** (this is the one that can trigger braking, see 9.3): time-to-collision (closing speed / range) below a threshold to any tracked object directly ahead in the ego lane.

### 9.3 Decision / Arbiter — the only subsystem allowed to produce an `ActuationRequest`

```cpp
// decision/DecisionArbiter.h
class DecisionArbiter {
public:
    void run();   // loop: consume SceneModel + VehiclePose + tracker output -> produce ActuationRequest
private:
    ActuationRequest evaluate(const SceneModel& scene, const VehiclePose& pose);
};
```

**Rules, in priority order (first match wins, only one request per cycle):**

1. If time-to-collision to the nearest in-lane obstacle < `config/decision_thresholds.yaml: ttc_brake_threshold_s` **and** the hub's last `SensorReport.obdBrakePedalActive` is false/unknown **and** the system is armed (kill switch not engaged, watchdog not tripped, per the hub's own `AckStatus`) → emit `ActuationRequest{type=BRAKE, intensity=<tuned value>, reason_code=FCW_TTC}`.
2. Else if hard-braking detected from the IMU (deceleration above a threshold) → emit `ActuationRequest{type=HAZARDS, reason_code=HARD_BRAKE_DETECTED}`.
3. Else if a reckless-driving track is classified HIGH/CRITICAL and is directly ahead → emit a visual/audio warning through the renderer (**not** an actuation request — reckless-driving detection informs the AR overlay and a warning tone, it does not brake or signal on its own; braking is reserved for the arbiter's own direct forward-collision check in rule 1, deliberately kept as the single simplest, most explainable trigger for a graded, safety-reviewed FYP).
4. Else → emit a zeroed `ActuationRequest{type=NONE}`.

**Never** emit a `BRAKE` request as a direct pass-through of a raw ML classifier's output — rule 1's trigger condition (time-to-collision, a simple, explainable geometric quantity derived from tracked range and closing speed) is deliberately the only thing allowed to request braking, precisely so the braking behaviour of the finished system can be fully explained and defended in the viva from a single, auditable rule rather than "the neural network decided to."

**What motion prediction may and may not do here (amended 2026-09-25).** Rule 1 is unchanged.
Its time-to-collision uses the in-lane obstacle's **current** tracked range and closing speed,
i.e. the 9.1 IMM state estimate *now*. It **never** uses predicted future positions, closest point
of approach, collision probabilities or IMM model probabilities. Those drive warnings (rule 3) and
the display only. Braking on a forecast risks *phantom braking* (stopping for something that was
never going to hit), and that is itself a hazard: a following car can hit you. Because the IMM
changes how closing speed is estimated, `test_motion_predictor.cpp` must bound its closing-speed
error and lag on synthetic approaches (constant closing, and the lead vehicle braking) before
Phase 10 relies on it. Any future use of prediction in rule 1 is a separate decision, needing its
own tests and bench evidence.

Every `ActuationRequest` this class produces is written to the `EventLog` with the full evaluation context (TTC value, range, closing speed, which rule fired) *before* being sent to `VehicleInterface` — this is the data that becomes your report's and viva's actuation-evidence trail (Part 16).

---

## Part 10 — AR Renderer and Display Sink

### 10.1 `DisplaySink` interface

*Amended 2026-09-25: overlays are rendered on the GPU. See `docs/decisions.md`, "Guide amendment:
GPU overlay rendering".* The renderer no longer hands the sink a finished picture. It hands over the
video frame plus an `OverlayScene`: a list of what to draw and where. The sink draws it. This is
also the cleaner seam for the future optical phase: a `ProjectorSink` draws only the overlay scene
through its combiner optics and ignores the video, which a pre-composited picture could never
support.

```cpp
// render/DisplaySink.h
struct OverlayItem;   // render/OverlayScene.h: one box, band, barrier, badge or icon, with its style
struct OverlayScene { std::vector<OverlayItem> items; };            // back-to-front draw order
struct CompositedFrame { cv::Mat video; OverlayScene overlays; uint64_t timestampMs; };
struct FrameGeometry { int width; int height; float aspectRatio; };

class DisplaySink {
public:
    virtual ~DisplaySink() = default;
    virtual bool init(const FrameGeometry& geometry) = 0;
    virtual void present(const CompositedFrame& frame) = 0;   // draws video + overlays
    virtual FrameGeometry outputGeometry() const = 0;
};
```

Each `OverlayItem` carries its geometry in ONE of three forms:
- **object mask** (camera pixels): the segmentation mask of one tracked object (Part 7.1,
  YOLOv8m-seg). This is used for the hazard glow, which follows the object's real outline.
- **image space** (camera pixels): HUD badges, icons, text, screen-edge fallback indicators;
- **road space** (metres, vehicle frame): anything that must sit ON the road, i.e. the route
  band, the stop barrier, no-turn barriers, hump markers. The GPU projects these into the image in
  the vertex shader, using the same intrinsics/extrinsics the perception side uses (Part 12), so
  a road-locked item lands exactly where the detections say it is.

It also carries a style: colour, opacity, glow, a shimmer/pulse rate (animated through a time
uniform in the shader), and a layer. The sink enforces `docs/architecture/ar-overlay-design.md`
rule 4, *never hide a hazard*, in two ways:
- a hazard glow is a rim of light plus at most a ~25% tint, so the object stays fully visible;
- road-space items are masked by hazard objects' silhouettes, so a barrier appears *behind* a
  pedestrian rather than painted across them.

### 10.2 `WindowedSink` (the only sink implemented this phase)

```cpp
// render/WindowedSink.h
class WindowedSink : public DisplaySink {
public:
    bool init(const FrameGeometry& geometry) override;   // SDL2 window + OpenGL context, shaders
    void present(const CompositedFrame& frame) override; // video texture, then overlay pass, swap
    FrameGeometry outputGeometry() const override;
private:
    SDL_Window* window_;
    SDL_GLContext glContext_;
    GLuint videoTexture_;
    GLuint overlayProgram_;   // one shader program, with branches per item kind
};
```

`present()` does two things:
1. **Video.** Upload the frame to `videoTexture_` and draw it full-screen. The upload uses a pixel
   buffer object (PBO), so the ~11 MB 2K frame transfers asynchronously. Under WSLg, OpenGL runs
   on Mesa's Direct3D-12 layer rather than NVIDIA's own GL driver. CUDA/OpenGL interop (sharing the
   GPU-resident frame directly) is therefore not expected to work there, and the host-to-GPU
   upload is the portable path; confirm this in 10.4.
2. **Overlays.** One pass over the `OverlayScene`, with alpha blending:
   - lines and bands as triangle strips, so edges are anti-aliased and widths are exact at any
     resolution;
   - glow as a soft falloff computed in the fragment shader;
   - pulsing from the time uniform;
   - road-space items projected in the vertex shader.

The window renders at the display's **native resolution**. On the development laptop's
2560×1600 panel, a 2560×1440 frame maps 1:1 with 80 px bars. OS scaling must not resample it:
Windows runs at 150%, so the process must be DPI-aware, or the WSLg scale factor must be 1.
Otherwise everything is drawn at 1707 px and stretched, which blurs it.

### 10.3 `ArRenderer`

`ArRenderer` decides WHAT to show. It builds the `OverlayScene` each frame and draws nothing
itself.

```cpp
// render/ArRenderer.h
class ArRenderer {
public:
    bool init(DisplaySink& sink);
    void run();   // loop: consume frameBus + detectionBus + SceneModel + navOverlayBus
                  //       -> build OverlayScene -> sink.present({frame, scene, t})
private:
    void addHazards(OverlayScene& s, const SceneModel& scene, const std::vector<Warning>& w);
    void addLaneOverlay(OverlayScene& s, const DetectionFrame& det);
    void addNavigationOverlay(OverlayScene& s, const RoadProjectedRoute& route, const VehiclePose& pose);
    void addSignBehaviours(OverlayScene& s, const std::vector<TrackedSign>& signs, const VehiclePose& pose);
    void addSpeedAndWarnings(OverlayScene& s, const VehiclePose& pose, const std::vector<Warning>& w);
};
```

- **Hazards, not boxes** (`docs/architecture/ar-overlay-design.md` §3). No rectangles are drawn
  on the driver display. A hazard is a tracked object flagged by the risk logic: forward-collision
  risk (Part 9.2/9.3), a Part 9.2 reckless-driving flag (tailgating, erratic speed, swerving), or a
  pedestrian or animal in or entering the ego path. Ordinary traffic gets nothing. A hazard is shown
  in the same road-placed language as the signs:
  - a barrier on the road at a collision-risk object;
  - the route line grading amber → red as the stopping distance shrinks;
  - a following-distance zone painted on the road for tailgating;
  - a red lane-line glow towards a swerving neighbour;
  - a ground ring under pedestrians and animals.
  
  The object itself gets a subtle **shimmering glow along its outline** (from its segmentation
  mask), with colour and shimmer pace set by a risk level `r` in [0, 1] derived from TTC and the
  9.2 flags: amber at caution, red at high risk. This is what makes a warning or a brake
  intervention never arrive unexplained. It never feeds actuation, which stays solely with the
  Part 9.3 arbiter. Drawing every detection with its class and confidence belongs to the debug
  viewer (`host/tools/inference_viewer`), not the driver display.
- **Navigation.** `addNavigationOverlay` emits `route.polyline` as a road-space band. Points with
  `on_measured_surface = false` (Part 11.4's far-field flat-ground fallback) are drawn more
  transparent or in a different shade than the near-field measured-surface points. That gives the
  driver an honest visual cue that precision degrades with distance, rather than presenting the
  whole line as equally certain. The same band carries the speed colour-grading of
  `docs/architecture/ar-overlay-design.md`.
- **Signs.** `addSignBehaviours` implements `docs/architecture/ar-overlay-design.md`: barriers, the
  speed badge, hump markers, and the priority/declutter rules.
- **Predicted motion** (Part 9.1). A hazard's predicted path is drawn as a fading ribbon on the
  road, whose width is its predicted uncertainty. Crossing barriers are placed where the predicted
  path meets the ego path. `PREDICTED_ONLY` (coasting, occluded) tracks are drawn visibly as
  predictions, not as seen objects.
- **Latency compensation.** Camera-to-screen latency is roughly 40–60 ms, and a crossing car
  moves ~0.75 m in that time at 15 m/s. Every object-anchored item (glow, ring, barrier) is drawn at
  the object's position **predicted for the display time**, from the 9.1 state, so overlays stick
  to moving objects instead of trailing them. The latency itself is measured in 10.4.

This is still video-see-through (Part 0). Overlays live in the camera's own image space (or are
projected into it with the camera's own calibration), so there is no separate projector/combiner
optical model to compute here.

**Why the GPU:** the holographic style (translucency, glow, gradients, pulsing) is per-pixel
blending over several full-screen layers per frame at 2K. The GPU does that in well under a
millisecond; on the CPU it would compete with the rest of the pipeline for the frame budget.

### 10.4 Benchmark checkpoint (do this before Part 14 marks this part complete)

Measure the actual end-to-end presented frame rate through `WindowedSink` running under WSLg, at
the resolution Part 6 settled on, with a realistic overlay scene (at least two shimmering hazard
glows, a route band, and one barrier). Record the video-upload time and the overlay-pass time separately. If the
rate doesn't reach the target (aim for 24+ fps), fall back to the plan noted in the original report:
keep CUDA/TensorRT inference in WSL2, and run a thin native Windows process that displays with
native OpenGL/DirectX on NVIDIA's own driver, instead of routing through WSLg's translation layer.

With GPU overlays, what crosses the local TCP socket is the video frame plus the small
`OverlayScene`, serialised with FlatBuffers like every other message. The Windows process runs
the same shaders. Document whichever path is used in `docs/decisions.md`.

---

## Part 11 — Vehicle Interface, Navigation, and System Manager

### 11.1 `VehicleInterface` — the only class that opens the serial port to the hub

```cpp
// vehicle/VehicleInterface.h
class VehicleInterface {
public:
    bool init(const std::string& serialDevice, int baud);   // e.g. /dev/ttyACM0, 115200
    void run();  // loop: read+decode frames -> publish SensorReport internally,
                 // AND send HEARTBEAT at 10Hz+ AND forward pending ActuationRequest as ActuationCommand
    void sendActuationRequest(const ActuationRequest& req);
    SensorReportSnapshot latestSensorReport() const;
private:
    int fd_;   // POSIX serial fd, opened via termios — matches the /dev/ttyACM0 path from Part 2.2's usbipd attach
};
```
This class implements the exact framing/CRC from Part 3 and nothing else — it does not interpret `ActuationRequest.reason_code` or apply any thresholds; all decision logic lives in `DecisionArbiter` (Part 9). Keep this boundary strict: if a future change means the arbiter's logic needs to see hub state to decide something, it reads `latestSensorReport()`, it does not reach into `VehicleInterface`'s serial handling.

### 11.2 `NavigationEngine` — routing

```cpp
// nav/NavigationEngine.h
class NavigationEngine {
public:
    bool init(const std::string& osrmDataPath);
    Route routeTo(double destLat, double destLon, const VehiclePose& currentPose);
};
```
Use the OSRM C++ API (`osrm::OSRM`, `RouteParameters`, MLD algorithm) against an offline-processed OpenStreetMap extract for the test area. **This subsystem is a primary project objective, not a stretch feature** — the precision, road-locked navigation overlay (11.4) is one of the two things this system does with what it senses (the other being hazard detection/actuation), so it does not get stubbed or deprioritized against the actuation subsystem's testing. `routeTo` returns a `Route` — the full geometry (an ordered list of world-coordinate points along the road network, not just turn text), because 11.3 and 11.4 need that geometry, not only the instructions a plain turn-by-turn UI would need.

### 11.3 `MapMatcher` — snapping the fused pose onto the route, not trusting raw GPS

Raw GPS/EKF position (from `SensorFusion`, Part 8.2) is only accurate to a few metres — nowhere near tight enough to know which lane, or sometimes even which road, the vehicle is actually on. OSRM's map-matching does for position what routing does for destinations: it snaps a noisy fix onto the correct edge of the road graph.

```cpp
// nav/MapMatcher.h
class MapMatcher {
public:
    bool init(const std::string& osrmDataPath);   // same offline OSM extract as NavigationEngine
    struct MatchedPosition {
        double latitude, longitude;    // snapped onto the road graph, not the raw fix
        double distanceAlongRouteM;    // arc-length progress along the current Route
        int laneCountHint;             // from OSM way tags where present; default 1 if untagged — a hint, not ground truth
        float roadHeadingDeg;
        bool valid;
    };
    MatchedPosition match(const VehiclePose& fusedPose, const Route& currentRoute);
};
```
Use OSRM's `Match` service/API rather than re-deriving this yourself. Run at 5–10 Hz — road identity and progress-along-route change far slower than the vehicle's raw dynamics, so there is no need to match on every fused-pose update. `laneCountHint` is exactly that, a hint: OSM tagging for lane count is inconsistent and must never be treated as reliable on its own — this is precisely why 11.4's lateral correction step exists and leans on the lane-detection model instead.

### 11.4 `RoadSurfaceProjector` — the precision, road-locked navigation overlay

This is the subsystem behind the actual "line drawn on the road" requirement. It does not sense anything new — it combines geometry that three other subsystems already produce: the route (11.2), the matched position (11.3), the LiDAR's measured ground surface (`GroundPlaneModel`, Part 8.1 — no longer a throwaway), and the lane-detection model's output (`DetectionFrame.lane_points_left/right`, Part 7).

```cpp
// nav/RoadSurfaceProjector.h
struct RoadOverlayPoint { float imageX, imageY; float worldDistanceM; bool onMeasuredSurface; };
struct RoadProjectedRoute { std::vector<RoadOverlayPoint> polyline; std::string nextTurnInstruction; float nextTurnDistanceM; };

class RoadSurfaceProjector {
public:
    bool init(const RoadProjectionConfig& cfg);   // config/road_projection.yaml
    RoadProjectedRoute project(const Route& route,
                                const MapMatcher::MatchedPosition& matchedPosition,
                                const GroundPlaneModel& groundPlane,
                                const DetectionFrame& laneDetections,
                                const VehiclePose& fusedPose,
                                const CameraIntrinsics& cameraCalib);   // Part 12.1
};
```

Algorithm, run once per `GroundPlaneModel` update (Part 8.1's refresh rate — no benefit projecting faster than its own geometry input changes):

1. From `matchedPosition.distanceAlongRouteM` onward, sample the route's world-space geometry into waypoints spaced roughly 1–2 m apart, out to a configured horizon (`config/road_projection.yaml: horizon_m`, start at 60).
2. **Near field** (within `groundPlane.maxValidRangeM`): project each waypoint's height onto `groundPlane.nearFieldPatch` — the actual measured surface — rather than assuming the road is flat. Mark these points `onMeasuredSurface = true`. This is the step that keeps the line glued to the road through slopes, crests, and dips instead of floating or sinking through the video image.
3. **Far field** (beyond LiDAR range): fall back to a flat-ground assumption relative to the vehicle's current ground height. Mark these `onMeasuredSurface = false`. This is an accepted, documented precision drop-off, not a bug — the report should state it plainly (Part 16).
4. Project every waypoint from vehicle-frame 3D coordinates into image space using `cameraCalib` — the same intrinsic/extrinsic calibration `SceneReconstruction` (Part 8.3) already uses, so this step introduces no new calibration dependency.
5. **Lateral correction**: where `laneDetections` has a confident lane boundary near a projected point's image x-position, nudge that point sideways (capped by `lateral_correction_max_m`) to sit centered between the detected lane lines, rather than trusting `matchedPosition.laneCountHint` or raw GPS for lane selection. This is what actually delivers lane-level precision — neither GPS nor OSM tagging is tight enough alone.
6. Emit the resulting `RoadProjectedRoute` on `navOverlayBus` for `ArRenderer` (Part 10.3) to draw.

`config/road_projection.yaml` starts with:
```yaml
horizon_m: 60                    # how far ahead to project the route line
waypoint_spacing_m: 1.5
lateral_correction_max_m: 1.0    # caps how far a single lane-detection frame can nudge the line sideways,
                                  # so one bad detection can't fling the overlay across lanes
```

### 11.5 `SystemManager`

```cpp
// system/SystemManager.h
class SystemManager {
public:
    static Config loadConfig(const std::string& configDir);
    template <typename T, typename... Sinks> void start(Sinks&... sinks);
    void runUntilShutdown();
    void requestShutdown();   // installed as SIGINT/SIGTERM handler
private:
    std::vector<std::thread> threads_;
    EventLog& log_;
};
```

### 11.6 `EventLog`

```cpp
// system/EventLog.h
class EventLog {
public:
    explicit EventLog(const std::string& path);
    void logActuationRequest(const ActuationRequest& req, const std::string& evaluationContext);
    void logAckStatus(const AckStatus& ack);
    void logFault(const std::string& component, const std::string& description);
    void logGeneral(const std::string& message);
};
```
Every log line includes a monotonic timestamp and is flushed immediately (not buffered) — this file is what you replay in the report and viva to show the actuation chain behaved correctly (Part 16), so it needs to survive a crash, not just a clean shutdown.

---

## Part 12 — Calibration Procedures

### 12.1 Camera intrinsic calibration

```bash
python3 scripts/run_camera_calibration.py --checkerboard 9x6 --square-size-mm 25 \
    --output config/camera_intrinsics.yaml
```
Print a 9×6 checkerboard, capture 20–30 images of it at varied angles/distances filling the frame, using the same camera/resolution/lens setting the final system will run at (recalibrate if you change resolution). The script wraps `cv::calibrateCamera()` and writes focal length, principal point, and distortion coefficients to the YAML consumed by `CameraPipeline` (Part 6) and `SceneReconstruction` (Part 8.3).

### 12.2 Camera-to-vehicle and LiDAR-to-camera extrinsic calibration

- **Camera-to-vehicle**: measure the camera's mounting position and orientation relative to the vehicle's reference frame (typically the rear axle midpoint, ground-projected) by hand — tape measure and a level are sufficient for the accuracy this system needs; write the result to `config/camera_extrinsics.yaml`.
- **LiDAR-to-camera**: place a checkerboard visible to both sensors simultaneously, capture a synchronized camera image and LiDAR scan, and compute the rotation/translation between them (a straightforward point-correspondence solve — OpenCV's `solvePnP` against the checkerboard corners found in both the image and, after manually picking the same corners in the point cloud, works for a one-off calibration; a dedicated LiDAR-camera calibration toolbox is a nice-to-have, not required). Write to `config/lidar_camera_extrinsics.yaml`.

### 12.2.1 Startup verification and bounded refinement of the LiDAR–camera extrinsics

*Added 2026-09-25.* Mounts shift slightly with vibration, temperature, or a knock during
installation, and mask-based fusion (Part 8.3) is only as good as the extrinsics. So every time the
system starts, and continuously while it runs, an `ExtrinsicMonitor` checks the alignment and may
fine-tune it, within strict limits.

```cpp
// scene/ExtrinsicMonitor.h
enum class ExtrinsicStatus { UNVERIFIED, VERIFIED, REFINED, DEGRADED };
struct ExtrinsicState { Eigen::Isometry3d lidarToCamera; ExtrinsicStatus status; float score; };

class ExtrinsicMonitor {
public:
    explicit ExtrinsicMonitor(const Eigen::Isometry3d& baseline);   // from Part 12.2's yaml
    void addFrame(const CameraFrame& frame, const ObjectMasks& masks, const SceneCloud& cloud);
    ExtrinsicState current() const;
};
```

**How it works:**
- **Baseline.** Part 12.2's checkerboard calibration stays mandatory, done once at installation.
  It is the reference. Each start begins from the baseline again, never from the last refinement,
  so small errors cannot accumulate across sessions.
- **Score.** Alignment is scored two ways:
  - *edge alignment:* LiDAR depth discontinuities projected onto image edges, via a distance
    transform of the edge map. This needs no objects in view;
  - *mask agreement:* the fraction of each object's LiDAR cluster that falls inside its eroded
    mask.
- **Verify.** Over the first frames, score the baseline. If it is good enough: `VERIFIED`.
- **Refine, bounded.** Search a small correction around the baseline: at most ~1° rotation and
  ~3 cm translation, as configuration. The correction is accepted (`REFINED`) only if **all** of
  these hold:
  - there is enough evidence: enough frames, with points spread over depths and image regions;
  - the improvement is clear *and* also holds on frames not used to fit it;
  - the solution is not pressed against the bounds. A correction at the bound means the real
    error is larger than a refinement may fix.
- **Degrade honestly.** Misalignment beyond the bounds, or no sufficient evidence within a
  configured time of driving, gives `DEGRADED`:
  - the baseline is kept;
  - an `EventLog` fault is logged, and the driver is told to recalibrate;
  - fused per-object ranges from the camera masks are not used for road-placed display graphics,
    which fall back to screen-fixed (`ar-overlay-design.md` rule 6);
  - the arbiter uses **LiDAR-only** ranges in a fixed vehicle-frame corridor, which depend on no
    camera alignment at all.
- **Never actuation-critical.** Braking never depends on an unverified refinement. The Part 9.3
  collision path must work in `DEGRADED` exactly as specified.
- **Continuous monitoring.** A sudden drop in score while driving (a knocked mount) moves the
  state to `DEGRADED` immediately.
- **Out of scope:** intrinsics are *not* auto-adjusted, because the camera is fixed-focus
  (Part 12.1), and neither is the hand-measured camera-to-vehicle transform.
- **Evidence.** Every state change and accepted refinement goes to the `EventLog`, with the before
  and after scores and the correction, for the report.
- **At startup** the car is usually stationary, often in a poor scene. Verification can then
  complete, but a refinement may only become possible once driving. That is expected, not a fault.

**Tests (Part 13.1):** `test_extrinsic_monitor.cpp` uses synthetic scenes with a known
perturbation of the extrinsics. It checks that:
- a small perturbation is recovered within tolerance;
- a large one ends `DEGRADED` rather than "fixed" at the bound;
- a featureless scene never produces `REFINED`.

### 12.3 What is explicitly *not* calibrated this phase

The projector-combiner angular mapping from the original optical-HUD design is not performed — there is no projector or combiner in this build (Part 0). This step returns in the future optical phase (see the original report's Section 8) and depends on nothing calibrated here changing.

### 12.4 Road-projection precision check (do this after Phase 11's exit criteria, before Part 13.4's staged tests)

This is not a one-time calibration like 12.1/12.2 — it's a repeatable sanity check, because `RoadSurfaceProjector`'s accuracy depends on calibration you've already done (camera intrinsics/extrinsics, LiDAR-camera extrinsics) staying correct, not on any new parameter of its own. Procedure:
1. Drive or walk the test vehicle slowly along a short, known stretch of road with visible lane markings.
2. Record the AR overlay output alongside the actual road.
3. Confirm visually: the projected route line sits inside the correct lane at close range (within `groundPlane.maxValidRangeM`), and does not visibly float above or sink below the road surface on any slope or dip present in the test stretch.
4. Repeat after any recalibration of camera intrinsics/extrinsics or LiDAR-camera extrinsics — a road-projection drift is often the most visible symptom of one of those calibrations having gone stale, and is a faster thing to notice than re-running the calibration scripts themselves.

### 12.5 Decision thresholds

`config/decision_thresholds.yaml` starts with conservative placeholder values and is tuned during Part 13's field testing, never guessed once and left alone:
```yaml
ttc_brake_threshold_s: 1.8      # time-to-collision below which a brake request is emitted
hard_brake_decel_g: 0.4         # IMU deceleration threshold for hazard-lights-on
tailgating_min_gap_s: 1.0       # following-distance-in-seconds threshold
brake_actuator_max_intensity: 90  # must match BrakeActuatorDriver::kMaxSafeIntensity on the hub — see Part 4.4
```

---

## Part 13 — Testing: Unit, Integration, and Staged Real-World Validation

### 13.1 Unit tests (GoogleTest, `host/test/unit/`)

At minimum, one test file per subsystem, covering the logic that doesn't need real hardware:
- `test_crc16.cpp` — known input/output vectors for `crc16_ccitt_false`, run identically against both the firmware and host copies of `Crc16.h`.
- `test_multi_object_tracker.cpp` — synthetic detection sequences, assert correct track creation/association/deletion.
- `test_decision_arbiter.cpp` — synthetic `SceneModel`/`VehiclePose` inputs covering every rule in Part 9.3, asserting the correct `ActuationRequest` (including the "no request" default case) — **this file should specifically include a test asserting that no combination of inputs produces a `BRAKE` request above `brake_actuator_max_intensity`**, since that's a config file, not a compiled constant, and a bad YAML value should be caught here, not on a bench.
- `test_ekf.cpp` — a known synthetic trajectory through `SensorFusion`, checking the estimate converges.
- `test_map_matcher.cpp` — a synthetic noisy GPS track against a small known test-area OSM extract, asserting the matched position snaps onto the correct road edge and that `distanceAlongRouteM` progresses monotonically.
- `test_motion_predictor.cpp` — Part 9.1 on synthetic trajectories with known noise:
  - CV, CTRV and stop-and-go motion; the $\omega\to0$ branch;
  - a pedestrian crossing; occlusion coasting to `PREDICTED_ONLY` and then death;
  - out-of-sequence measurements;
  - the centroid-bias case (a turning car's visible part shifting must not produce lateral
    velocity);
  - hand-computed CPA/TCPA cases;
  - NIS within its chi-square bounds;
  - the **closing-speed error and lag bound** that 9.3 rule 1 relies on.
- `test_mask_lidar_fusion.cpp` — synthetic camera, extrinsics, masks and point clouds covering every Part 8.3 rule (eroded-mask selection, nearest-depth cluster vs stray/background points, parallax depth test, "no range" when too few points, unexplained LiDAR cluster in the corridor becoming an `UNKNOWN` obstacle).
- `test_extrinsic_monitor.cpp` — Part 12.2.1: a small known perturbation is recovered, a large one ends `DEGRADED` rather than clamped at the bound, a featureless scene never yields `REFINED`.
- `test_road_surface_projector.cpp` — synthetic route waypoints, a synthetic `GroundPlaneModel` (including a non-flat patch, e.g. a sloped segment), and synthetic lane detections; assert that near-field points land on the supplied ground geometry (not a flat-ground assumption), far-field points fall back correctly, and the lateral-correction step never moves a point by more than `lateral_correction_max_m`.

Run with `ctest` from the `host/build` directory; wire this into Part 14's phase exit criteria — a phase touching a subsystem with a unit test file is not complete until that test passes.

**`host/test/unit/` builds as its own lightweight CMake project, deliberately independent of the top-level `host/CMakeLists.txt`.** None of the six files above touch TensorRT, SDL2, or a live camera/LiDAR/serial device — they exercise pure logic against synthetic data — so `host/test/unit/CMakeLists.txt` links only Eigen3, PCL, GTest, and (for `test_map_matcher.cpp`/`test_road_surface_projector.cpp`) OSRM, none of which need a GPU to build. This isn't just tidiness: it's the only reason Part 13.5's CI can run this suite on an ordinary GitHub-hosted runner, which has no NVIDIA GPU and nowhere to install a CUDA-built OpenCV or a live display for `WindowedSink`. Keep it this way — if a unit test ever needs to `#include` something from the TensorRT/SDL2 side of the tree, that's a sign the class under test needs splitting, not a reason to widen this CMakeLists.txt's dependencies.

### 13.2 Integration tests (`host/test/integration/`, scripted, run without a vehicle)

- `replay_recorded_frames.py` (in `tools/bench_rig/`) feeds a pre-recorded camera+LiDAR dataset through the full pipeline offline and checks the pipeline runs end-to-end without crashing and produces a plausible `DetectionFrame`/`ActuationRequest` stream — this is how you validate the whole software stack **before** any hub or vehicle hardware is involved.
- A loopback test against the firmware: run the firmware on the bench (Part 4.7's setup) and run the real `VehicleInterface` against it, asserting `SensorReport`s arrive at 50 Hz and a synthetic `ActuationRequest` results in the correct bench-wired relay/actuator behaviour.
- Part 12.4's road-projection precision check, repeated here as a gating integration check rather than a one-off — do not consider Phase 11 (Part 14) complete until it passes, since a visibly drifting nav overlay is as much a "this subsystem doesn't work yet" signal as a failing unit test.

### 13.3 Actuator bench test (before any vehicle involvement — mandatory gate)

Using `tools/bench_rig/log_actuator_bench.py` against the hub's `AckStatus.appliedBrakeIntensity` and the `BrakeActuatorDriver::readCurrentAmps()` telemetry:
1. Mount the actuator and a pedal fixture on a bench rig, not in the vehicle.
2. Command a range of `brakeRequest` intensities and log applied force/current/response time for each.
3. With the actuator engaged at its maximum configured intensity, have a person push back on the pedal by foot and confirm they can move it — this is the single most important pass/fail check in the entire project; do not proceed past it on a "should be fine" basis.
4. Confirm the kill switch cuts power with the actuator mid-actuation.
5. Confirm the watchdog releases the actuator within 200 ms of the USB cable being physically unplugged, with no software command telling it to release.
6. Only once all five pass repeatably does `brake_actuator_max_intensity` get treated as final and Part 15 begins.

### 13.4 Staged real-world validation (matches the report's testing plan, expanded to concrete checklists)

**Stage A — Bench rig.** Covered by 13.3. Exit criteria: all five checks pass on three consecutive attempts.

**Stage B — Stationary in-vehicle.** Full system installed (Part 15), vehicle stationary (handbrake engaged, ideally wheels chocked or on a jack stand). Checklist:
- [ ] `SensorReport`s reflect real vehicle OBD-II/GPS/IMU data.
- [ ] A manually triggered detection event (walk a person/object into frame at close range) produces the correct `ActuationRequest` in the `EventLog`.
- [ ] The brake actuator visibly engages and the driver can push through it, exactly as on the bench.
- [ ] Kill switch and watchdog checks repeated in-vehicle, not just assumed from Stage A.

**Stage C — Field test.** Empty field, no other people present, spotter present. Checklist:
- [ ] Actuation mechanism and override/watchdog behaviour hold at real (if low) vehicle speed.
- [ ] Explicitly log that stopping-distance/TTC calibration is **not** being validated here (loose/uneven ground doesn't represent road surface) — this note goes in the report, not just this checklist.

**Stage D — Campus road section.** The highest-stakes stage. Checklist:
- [ ] Formal closure arranged with campus security/facilities for the test window.
- [ ] Timing chosen for near-zero foot traffic.
- [ ] Spotters posted at both ends plus one walking alongside; agreed abort signal beyond the kill switch.
- [ ] Speed capped to 5–10 km/h.
- [ ] First pass(es) run with a manual "confirm-to-brake" step before allowing fully automatic triggering.
- [ ] Department/campus safety notification filed ahead of time.
- [ ] Full `EventLog` retained from every run for the report.

### 13.5 Continuous Integration

CI here has a deliberately honest scope: a GitHub-hosted runner has no NVIDIA GPU, nowhere to install a CUDA-built OpenCV at reasonable speed, and no STM32 board, camera, LiDAR, or actuator plugged into it. So CI is not a substitute for anything in 13.1–13.4 — it's a fast, free, runs-on-every-push layer *underneath* them, catching the mistakes that don't need hardware to catch. `.github/workflows/build.yml`:

```yaml
name: CI

on:
  push:
  pull_request:

jobs:
  protocol-sync-check:
    name: Protocol / CRC16 byte-for-byte check (firmware vs host)
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Protocol.h vs HubProtocol.h
        run: diff firmware/sensor_actuator_hub/include/hub/Protocol.h host/include/ar_drive_assist/vehicle/HubProtocol.h
      - name: Crc16.h vs Crc16.h
        run: diff firmware/sensor_actuator_hub/include/hub/Crc16.h host/include/ar_drive_assist/vehicle/Crc16.h

  firmware-build:
    name: Firmware build (PlatformIO)
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with: { python-version: "3.x" }
      - run: pip install platformio
      - run: cd firmware/sensor_actuator_hub && pio run

  host-unit-tests:
    name: Host unit tests (logic only — see Part 13.1's note on why this needs no GPU)
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Install CPU-only dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y libeigen3-dev libpcl-dev libgtest-dev libosrm-dev cmake build-essential
      - name: Configure and build host/test/unit only
        run: |
          cmake -S host/test/unit -B host/test/unit/build
          cmake --build host/test/unit/build
      - name: Run unit tests
        run: ctest --test-dir host/test/unit/build --output-on-failure

  format-check:
    name: clang-format check
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: sudo apt-get install -y clang-format
      - run: find firmware host \( -name "*.cpp" -o -name "*.h" \) -print0 | xargs -0 clang-format --dry-run --Werror
```

What each job actually catches, and why it's cheap enough to run on every push:
- **`protocol-sync-check`** turns Part 3's manual rule — "when one changes, change the other in the same commit" — into something enforced automatically instead of trusted to memory. No dependencies, runs in seconds, and it's precisely the kind of drift that's invisible in a code review but fatal on the bench.
- **`firmware-build`** only compiles the firmware; it never flashes or runs it. That still catches most syntax/link errors the moment they're introduced, well before the next bench session.
- **`host-unit-tests`** runs exactly the suite from 13.1, no more — this is why that suite was kept dependency-light in the first place.
- **`format-check`** is the cheapest possible check and the only one that needs nothing built at all.

What CI deliberately does **not** attempt, so nothing downstream assumes it did: it never builds `host/`'s full CUDA/TensorRT/SDL2-linked executable, never runs `test/integration/`, never touches the actuator, and proves nothing about the camera, LiDAR, or a real STM32 board. Those stay exactly where 13.2–13.4 already put them — on the actual machine, on the bench, and eventually on the vehicle.

---

## Part 14 — Phased Implementation Roadmap (Execution Order)

Work through these phases in order. Each phase names its deliverables, the guide Parts it draws from, and the exit criteria that must be true before moving to the next phase. Do not start a phase's exit-criteria check until every deliverable in that phase is actually implemented — "partially works" is not a passed exit criterion anywhere in this roadmap, least of all in the phases touching actuation.

Two things apply to every phase below even though neither is repeated in each phase's own entry: the comprehension checkpoint from "How to use this document" (the person must be able to explain what got built and why, not just have it working), and a `docs/progress-log.md` entry per task completed within the phase — not one entry for the whole phase.

**Phase 0 — Scaffold and environment**
Deliverables: full directory tree (Part 1) with stub files, including `.gitignore` (Part 1.2, `.claude/` as its first line) and `.github/workflows/build.yml` (Part 13.5); environment fully set up and verified (Part 2.10).
Exit criteria: Part 2.10's five checks all pass; `cmake` configures the (currently empty) `host/` project without error; `pio run` builds the (currently empty) firmware project without error. The CI workflow existing and running (even with jobs failing, since there's nothing to check yet) counts for this phase — `protocol-sync-check` and `firmware-build` going green is Phase 1 and Phase 2's exit criteria respectively, not this one's.

**Phase 1 — Shared protocol**
Deliverables: `Protocol.h` and `Crc16.h`, identical in `firmware/` and `host/` (Part 3).
Exit criteria: `test_crc16.cpp` passes on the host side; a matching firmware-side test (or a manual serial round-trip against known byte vectors) confirms identical CRC behaviour; CI's `protocol-sync-check` job is green.

**Phase 2 — Firmware hub, bench bring-up (no actuator, no vehicle)**
Deliverables: all drivers (Part 4.4) except `BrakeActuatorDriver`'s real hardware (stub it returning zero current, no motor connected yet), `SensorTask`, `CommsTask`, `WatchdogTask` fully implemented, `ActuationTask` implemented against relays only.
Exit criteria: Part 4.7's bench verification checklist passes in full (50 Hz sensor reports, relay toggles on command, watchdog releases relays within 200 ms of USB unplug); CI's `firmware-build` job is green.

**Phase 3 — Host skeleton and hub link**
Deliverables: `SystemManager`, `EventLog`, message bus (`RingBuffer`), `VehicleInterface` fully implemented against the real firmware from Phase 2.
Exit criteria: 13.2's loopback integration test passes — host receives 50 Hz `SensorReport`s from the real hub, and a synthetic `ActuationRequest` sent through `VehicleInterface` correctly toggles the bench-wired relay.

**Phase 4 — Camera pipeline**
Deliverables: `CameraPipeline` (Part 6).
Exit criteria: sustained fps at target resolution measured and logged; resolution finalized (2K or the 1080p fallback) and written to `docs/decisions.md`.

**Phase 5 — ML inference**
Deliverables: exported ONNX models, built TensorRT engines (Part 7.3), `TrtEngine` + `MlInferenceEngine` (Part 7.4).
Exit criteria: live camera feed produces detections/lane lines/depth at the target FPS from Part 7's table; a debug visualization (even a throwaway `cv::imshow` window) shows plausible boxes on real footage.

**Phase 6 — LiDAR and calibration**
Deliverables: `LidarProcessor` (Part 8.1), **including publishing `GroundPlaneModel` on `groundPlaneBus` rather than discarding the fitted plane**; camera intrinsic calibration (Part 12.1) and LiDAR-to-camera extrinsic calibration (Part 12.2) completed and saved to `config/`; `ExtrinsicMonitor` (Part 12.2.1) running at startup and continuously, with `test_extrinsic_monitor.cpp` passing.
Exit criteria: LiDAR point cloud visibly aligns with camera detections of the same real object in a debug overlay; `GroundPlaneModel` is visibly logged/published on every LiDAR frame, not silently dropped after clustering — check this explicitly, since Phase 9 depends on it and a missing publish step won't produce an obvious error, only a degraded overlay much later.

**Phase 7 — Fusion, scene reconstruction, tracking**
Deliverables: `SensorFusion` (Part 8.2), `SceneReconstruction` with mask-based camera–LiDAR fusion (Part 8.3), `MultiObjectTracker` + `MotionPredictor` (IMM, CPA risk; Part 9.1), `config/motion_prediction.yaml`.
Exit criteria: `test_ekf.cpp`, `test_multi_object_tracker.cpp`, `test_motion_predictor.cpp` (including NIS consistency and the closing-speed bound) and `test_mask_lidar_fusion.cpp` pass; a tracked object's range/velocity estimate visibly matches reality in a manual sanity check (e.g., walking toward the camera at a known pace).

**Phase 8 — Navigation: routing and map-matching**
Deliverables: `NavigationEngine` (Part 11.2), `MapMatcher` (Part 11.3), against an offline OSM extract for the test area.
Exit criteria: `test_map_matcher.cpp` passes; a route requested between two known points on the test-area extract returns sensible turn-by-turn geometry; feeding a recorded or live GPS/EKF track through `MapMatcher` produces a matched position that visibly snaps onto the correct road edge, with `distanceAlongRouteM` increasing monotonically as the track progresses.

**Phase 9 — Road Surface Projector: precision navigation overlay**
Deliverables: `RoadSurfaceProjector` (Part 11.4), consuming Phase 6's `GroundPlaneModel`, Phase 8's matched position and route, Phase 5's lane detections, and Phase 7's fused pose.
Exit criteria: `test_road_surface_projector.cpp` passes; a throwaway debug visualization (a plain `cv::imshow` overlay is enough, the same pattern as Phase 5/6 — the polished `ArRenderer` integration is Phase 11, not this one) shows the projected route line visibly tracking the correct lane at close range and following real ground geometry rather than floating or sinking, against a short real test stretch. Part 12.4's full precision check is the more rigorous re-run of this same idea once `ArRenderer` exists (Phase 11) — this phase's exit criteria is the earlier, rougher version of the same check, not a substitute for skipping it later.

**Phase 10 — Decision / Arbiter**
Deliverables: rule-based reckless-driving classifier (Part 9.2), `DecisionArbiter` (Part 9.3).
Exit criteria: `test_decision_arbiter.cpp` passes in full, **including the ceiling test** — this is a hard gate, do not proceed to Phase 12 without it green.

**Phase 11 — AR Renderer and display**
Deliverables: `DisplaySink`/`WindowedSink` with GPU overlay rendering (Part 10.1–10.2), `ArRenderer` (Part 10.3) building the `OverlayScene`, including `addNavigationOverlay` against Phase 9's `RoadProjectedRoute`, and the hazard and sign overlays of `docs/architecture/ar-overlay-design.md` (including the mask-based hazard glow).
Exit criteria: Part 10.4's benchmark checkpoint completed and the resulting display path (WSLg vs. native-Windows fallback) chosen and documented; the navigation overlay renders visibly distinguishing near-field (measured-surface) from far-field (flat-ground fallback) segments, per Part 10.3's note.

**Phase 12 — Brake actuator hardware and mandatory bench gate**
Deliverables: real `BrakeActuatorDriver` against actual actuator hardware (Part 4.4–4.5, completing what Phase 2 stubbed).
Exit criteria: **Part 13.3's full bench checklist, all five items, on three consecutive attempts.** This phase cannot be marked complete on partial results. If the mechanical coupling fails the "driver can push through it" check, redesign the coupling before touching software again — do not compensate for a mechanically unsafe coupling by lowering `kMaxSafeIntensity` and calling it done; the coupling itself must be safe independent of the software ceiling.

**Phase 13 — Full bench-top end-to-end integration (no vehicle yet)**
Deliverables: everything from Phases 1–12 running together against the bench-mounted actuator and a live camera/LiDAR feed on a desk/bench rig, including the navigation overlay rendering live alongside hazard detection.
Exit criteria: a real detection event (person walking toward the bench camera at close range) produces a logged `ActuationRequest`, a hub `AckStatus`, and visible actuator engagement, end to end, with no manual test-script intervention; the navigation overlay renders correctly at the same time, against a bench-scale or recorded route.

**Phase 14 — Vehicle installation**
Deliverables: physical installation per Part 15.
Exit criteria: Part 15's installation checklist complete; system powers on with ignition and all sensor reports/actuation channels test correctly with the vehicle stationary, before Phase 15 begins.

**Phase 15 — Staged real-world validation**
Deliverables: Stages A–D from Part 13.4, in order, no stage skipped.
Exit criteria: each stage's checklist fully checked off, with `EventLog` files retained from every run, before moving to the next stage. Stage D's checklist item about department/campus notification is itself a gate — do not run Stage D without it. Re-run Part 12.4's road-projection precision check during Stage B or C (whichever comes first with the vehicle actually moving) — a bench/desk-rig pass in Phase 9 does not guarantee the overlay still tracks correctly once real vehicle vibration and outdoor lighting are in play.

**Phase 16 — Report and presentation**
Deliverables: per Part 16.
Exit criteria: report structure complete with actual test evidence (not placeholders) from Phase 15; demo rehearsed at least once end-to-end in the actual presentation venue if possible.

---

## Part 15 — Physical Build and Vehicle Installation

Perform only after Phase 13 (Part 14) is complete — nothing here should be the first time any subsystem has run.

### 15.1 Pre-installation checklist

- [ ] Actuator bench-tested per 13.3, `kMaxSafeIntensity` finalized and not touched again without a re-test.
- [ ] Kill switch wired and tested on the bench.
- [ ] All cable lengths measured against the actual vehicle before cutting/terminating anything.

### 15.2 Mounting order

1. **GPS antenna** — dashboard near the base of the windshield, or roof-mounted with cable routed through a door seal; clear sky view is the only real requirement.
2. **Camera** — behind the rear-view mirror, forward-facing, secured so it can't vibrate loose; route the USB cable along the headliner and down the A-pillar trim, clear of airbag deployment zones.
3. **LiDAR** — roof, grille, or dashboard-behind-windshield per the trade-offs in the original design; connect via its Ethernet cable to the USB-to-Ethernet adapter, which plugs into the laptop.
4. **Sensor & Actuator Hub (STM32)** — mounted somewhere accessible for debugging (under-dash is typical); wire GPS/IMU/OBD-II/gesture sensor to it per Part 4.2's pin table.
5. **Relays** (indicators/hazards/horn/beam) — tapped onto the existing stalk/switch wiring per the manufacturer's wiring diagram for the specific vehicle; test each relay individually with the vehicle off before wiring to the hub.
6. **Brake actuator and kill switch** — installed per the bench-validated mechanical design (Part 13.3); kill switch mounted within easy driver reach; **the driver's own pedal linkage is left completely intact and undisturbed** — confirm by pressing the pedal normally with the actuator powered off before doing anything else.
7. **Laptop** — adjustable cabin mount positioned in the driver's forward-ish field of view; this is the actual AR display now, so mounting position and screen angle/glare matter more than they would for a low-dash-mounted compute unit.
8. **Power** — switched 12V accessory circuit (active only with ignition ON/ACC) feeds: the peripheral 12V→5V converter (hub, sensors, relay coils) through an inline fuse, and the laptop's 12V→USB-C PD car charger, separately.

### 15.3 Post-installation, pre-drive checklist (repeat Stage B of Part 13.4 here explicitly)

- [ ] System powers on with ignition ON, powers down cleanly with ignition OFF.
- [ ] All `SensorReport` fields show real, sane values (compare GPS to a phone, OBD speed to the dash, IMU heading to actual vehicle heading).
- [ ] Kill switch and watchdog re-tested in the installed configuration, not assumed from the bench.
- [ ] Driver confirms normal pedal feel and full manual override with the actuator powered.

---

## Part 16 — Report and Presentation Guide

### 16.1 Report structure

Map the final written report to the sections already established for this project (do not reinvent a structure — reuse the one already reviewed):

1. Executive Summary — the paragraph-length description already drafted for this project is a good starting point for this section, expanded to a page.
2. Motivation and Scope — including the explicit note that video-see-through-on-a-laptop is an acknowledged simplification, with the optical see-through concept preserved as documented future work.
3. System Architecture — the five-module breakdown, with the actual as-built architecture diagram (redraw the Part 5/ASCII diagram properly for the report, don't submit ASCII art).
4. Software Architecture — the subsystem table, updated with any real deviations noted in `docs/decisions.md`.
5. Actuation Safety Design — risk tiering, the driver-override/kill-switch/watchdog fail-safe rules, and this is the section that should read almost like a small safety case, since it's what a supervisor or examiner will scrutinize hardest.
6. Precision Navigation Overlay Design — the OSRM routing → map-matching → LiDAR ground-plane projection → lane-detection correction pipeline (Part 11.2–11.4), stated honestly as near-field measured-surface precision degrading to a far-field flat-ground fallback, with real lateral/vertical error figures from Part 12.4's checks, not just a description of the intended design.
7. Hardware Bill of Materials — actual costs incurred, not just estimates.
8. Calibration — methodology and results (reprojection error from Part 12.1, etc., plus the road-projection precision check's results from Part 12.4).
9. Testing and Validation — this is where Part 13's `EventLog` evidence goes: real timestamps, real TTC values, real applied-intensity numbers from actual test runs, not simulated numbers.
10. Results and Discussion — what worked, what didn't, what was cut for time (be honest about anything moved to future work under time pressure — examiners respond well to an accurate account of trade-offs made under a real deadline, not a report that pretends everything went exactly to plan).
11. Future Work — optical see-through return path (already documented), thermal/IR sensing (discussed and deliberately deferred), any stretch goals not reached.
12. Conclusion.

### 16.2 Evidence to capture during Part 14/15 specifically because the report will need it

- Screenshots or screen recordings of the AR overlay running live, ideally with a real detection/warning visible.
- Footage of the navigation overlay tracking a real road stretch, ideally including one section with a slope or dip that visibly demonstrates the line following measured ground geometry rather than floating — this is the clearest way to show the LiDAR-fusion approach actually did something a flat-ground assumption couldn't.
- The `EventLog` output from at least one full run of each testing stage (A–D).
- Photos of the physical installation (camera mount, LiDAR mount, hub wiring, actuator coupling, kill switch location) — these substitute well for the original design's CAD-style installation figures.
- A short video of the bench actuator test showing the driver-override check (13.3, item 3) — this single clip is likely to be the most persuasive piece of evidence in the whole presentation, since it's the one thing that directly demonstrates the safety design works, not just that it was designed.

### 16.3 Demo script for the live presentation

Structure the live demo around the same staged-risk idea as the testing plan, since it reads well to an audience and mirrors how the system was actually validated:
1. Open with the bench rig clip (16.2) — establishes the safety story before anything else, which pre-empts the "isn't this dangerous" question before it's asked.
2. Live AR overlay demo, stationary vehicle (Stage B conditions) — walk a person into frame, show the detection, overlay, and (if within venue safety constraints) the actuator visibly engaging with a driver's foot overriding it.
3. If a moving demo (Stage C/D footage) is available and appropriate for the venue, show it as recorded video rather than attempting a live moving demo in the presentation room — a live moving vehicle demo in front of an audience is its own safety consideration a room usually can't accommodate.
4. Close on the future-work slide (optical see-through, thermal sensing) — this is a good place to reiterate that the `DisplaySink` abstraction makes the optical phase a real, planned continuation rather than a hand-wave.

### 16.4 Slide outline

Title → Motivation (1 slide) → Architecture diagram (1 slide) → Display simplification & why (1 slide) → Actuation & safety design (2 slides — this is worth more slide time than any other single topic, given it's the highest-scrutiny part) → Precision navigation overlay (1 slide — the near-field/far-field precision split is worth a visual, e.g. a before/after showing flat-ground vs. ground-plane-locked projection) → Live/recorded demo → Results (metrics: detection FPS, TTC accuracy, actuator response time, navigation lateral/vertical error) → Future work → Questions.

### 16.5 Anticipated viva questions worth preparing for explicitly

- "What happens if the laptop crashes while the actuator is engaged?" — answer from the hub's watchdog design (Part 3.4, Part 4.6), not from the host software.
- "How do you know the driver can override the actuator?" — answer from the bench test evidence (13.3 item 3), with the video.
- "Why not use the Jetson/optical HUD as originally planned?" — answer from Part 0's scope note, framed as a deliberate, supervisor-directed timeline trade-off, not an admission of failure.
- "How is a false positive on braking prevented?" — answer from Part 9.3's single, auditable TTC-based trigger rule, deliberately kept simple and explainable rather than a black-box ML decision.
- "Why not just use GPS for navigation like a normal sat-nav?" — answer from Part 0's navigation-precision note: GPS alone is only accurate to a few metres, nowhere near tight enough to lock a line to a specific lane; the LiDAR ground-plane and lane-detection steps are what actually deliver the precision, GPS/OSRM only establish which road and how far along it.

---

## Appendices

### Appendix A — Config File Templates

```yaml
# config/vehicle_params.yaml
wheelbase_m: 2.6
camera_height_m: 1.3
obd_pid_brake_active: "0x?? "   # fill in if the target vehicle exposes this PID; else leave blank
serial_device: "/dev/ttyACM0"
serial_baud: 115200
```

```yaml
# config/camera_intrinsics.yaml   (produced by Part 12.1, shown here as a template)
image_width: 2560
image_height: 1440
camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_coefficients: [k1, k2, p1, p2, k3]
reprojection_error_px: 0.0   # fill in from the calibration script's output; keep below ~0.5px
```

### Appendix B — Fault Codes (`AckStatus.actuatorFaultCode`, referenced in Part 3.1 and Part 4.5)

| Code | Meaning | Firmware behaviour |
|---|---|---|
| 0 | No fault | Normal operation |
| 1 | Overcurrent detected | Actuator released, relays off, latched until power cycle |
| 2 | Kill switch engaged | All actuation disarmed, host informed via `AckStatus` |
| 3 | Host link timeout (>200ms) | Watchdog release triggered |
| 4 | CRC/frame error rate exceeded threshold | Actuation disarmed pending clean frames |

### Appendix C — WSL2 LiDAR Networking Reference (expanded from earlier discussion)

```bash
# Inside WSL2 Debian, after enabling mirrored networking (Part 2.1):
ip addr show                      # confirm the host's Ethernet interface is visible
sudo ip addr add 192.168.1.50/24 dev eth1   # match the LiDAR's subnet
ping 192.168.1.100                # LiDAR default IP — confirm reachability before starting LidarProcessor
```
If mirrored mode isn't available (pre-22H2 Windows), fall back to binding the USB-to-Ethernet adapter itself via `usbipd-win` so it enumerates as its own interface directly inside WSL2, then apply the same static IP configuration to that interface.

### Appendix D — Troubleshooting Quick Reference

| Symptom | Likely cause | Where to look |
|---|---|---|
| Camera drops frames at 2K | USB/IP bandwidth limit under WSL2 | Part 6.2 — fall back to 1080p or Windows-side capture |
| LiDAR unreachable from WSL2 | Networking mode not mirrored, or adapter not bound | Appendix C |
| `SensorReport` stops arriving | USB-CDC re-enumerated after WSL2 restart | Re-run `attach_usb_devices.ps1` (Part 2.2) |
| Actuator doesn't release on unplug test | Watchdog timing or IWDG not actually enabled | Re-check Part 4.6/4.3, this is a hard blocker, do not proceed until fixed |
| AR overlay stutters badly | WSLg OpenGL translation layer bottleneck | Part 10.4's native-Windows-display fallback |
| Navigation line floats above or sinks through the road | `LidarProcessor` still discarding the ground plane instead of publishing `GroundPlaneModel` | Part 8.1's "important change" note — re-check the plane is actually reaching `groundPlaneBus` |
| Navigation line sits in the wrong lane | Lateral-correction step in `RoadSurfaceProjector` not applying, or `lateral_correction_max_m` set too low to matter | Part 11.4 step 5; check lane-detection confidence isn't being silently filtered out upstream |
| Navigation line jumps or jitters between frames | Map-matching running faster than needed and reacting to per-fix GPS noise | Part 11.3 — confirm `MapMatcher` is throttled to 5–10 Hz, not run on every raw fix |
| TensorRT engine fails to load | Engine built on a different GPU/driver/TensorRT version than the one running it | Rebuild locally per Part 7.3 — engines are never portable across machines |

### Appendix E — Glossary

- **EKF** — Extended Kalman Filter, used for sensor fusion (Part 8.2).
- **TTC** — Time-to-collision, the single geometric quantity allowed to trigger a brake request (Part 9.3).
- **`DisplaySink`** — the abstraction separating rendering logic from the physical output device, so the project can move between `WindowedSink` (this phase) and a future `ProjectorSink` without touching perception/decision code.
- **Hub** — the STM32/FreeRTOS sensor-and-actuator microcontroller; the only component with direct electrical access to vehicle sensors and actuation wiring.
- **Arbiter** — `DecisionArbiter`, the single subsystem permitted to produce an `ActuationRequest`.
- **Map-matching** — snapping a noisy GPS/EKF fix onto the correct edge of the road network graph, rather than trusting the raw coordinate (Part 11.3).
- **`GroundPlaneModel`** — the LiDAR-fitted road surface, published rather than discarded, that lets the navigation overlay track real ground geometry at close range (Part 8.1).

### References

Carry forward from the original project report: WayRay technical background and automotive HUD market context (now background/motivation only, not the display architecture actually built), YOLOv8/MiDaS/Ultra-Fast-Lane-Detection-v2 papers, Livox SDK2 and OSRM documentation, TensorRT documentation. Add for this build: STM32 FreeRTOS documentation, PlatformIO documentation, `usbipd-win` project documentation and README, WSL2 networking-mode documentation (Microsoft Learn), NVIDIA CUDA-on-WSL documentation.
