# Evidence pack for the final report (as of 2026-10-08)

**What this is:** the measured evidence, figures and sources for each part of the final report,
mapped to the proposal's three specific objectives (proposal 1.3.2) and its Table 3. It is
**evidence, not prose**: numbers with their source, date and the command that reproduces them;
pointers to the decision log and progress log for the reasoning; what changed since the proposal;
what is still pending hardware. The writing is yours.

**Rules used here:**
- Every number was re-measured on 2026-10-08 unless marked with another date, in which case it
  is carried from the progress log entry of that date (the code it measures has not changed since).
- "Pending hardware" means exactly that: not claimed.
- Figures are in `figures/`, made by `tools/report/make_evidence_figures.py` from the system's own
  output. Images from the KITTI drive are **CC BY-NC-SA 3.0**: credit Geiger et al. (IJRR 2013) in
  the caption.

---

## 1. Table 3, updated (measurable targets per objective)

| Obj. | Metric | Target | Proposal (Sept) | Now (2026-10-08) | Source / reproduce | Status |
|---|---|---|---|---|---|---|
| 1 | Four networks together, per 2K frame | ≤ 33 ms | median 17.8 ms; p95 19.9; max 26.2 | **median 16.3 ms; p95 20.5; max 28.3** (7,200 frames, 61 fps at the median) | `build/host/inference_viewer data/footage/krakow_0-120s.webm --frames 7200` | Met (re-measured) |
| 1 | Road-sign detection | mAP50 ≥ 0.70 | 0.716 (mAP50-95 0.581); Kenyan audit 24/25 | unchanged (2026-09-26) | `docs/experiments/2026-09-26-sign-detector.md` | Met |
| 1 | LiDAR range per detected object | range for every object, ≥ 3 points | verified on synthetic scenes | **on real LiDAR data (KITTI):** measured gap within 0.3 m of the truth (mean +0.01 m; worst +0.24 m) | replay harness, §3.1 | Met on recorded data; Mid-360 pending |
| 1 | Live LiDAR capture (Mid-360) | — (new) | — | packet decoding and clock mapping unit-tested without the SDK (5 tests) | `test_livox_packets` | Built; live unit pending |
| 2 | AR render rate at 2560×1440 | ≥ 24 fps | 54–82 fps; overlay ≈ 2 ms | **55.3 fps overall; overlay pass 2.16 ms GPU** (RTX 5060, D3D12) | `build/host/render_bench --video data/footage/krakow_0-120s.webm` | Met (re-measured) |
| 2 | Overlay projection vs camera model | ≤ 2 px | within 2 px | unchanged; the test still passes | `test_windowed_sink` (integration, label `gpu`) | Met |
| 2 | Map matching | correct road; median < 5 m | median 3.1 m; 98.7% < 20 m (2,773 fixes) | summary reproduced: 2,773/2,773 matched, progress never decreased (snap statistics 2026-09-27) | `build/host/nav_replay --gpx "data/drives/GPX Test Drive.gpx"` (private file) | Met |
| 2 | Reroute after a closure | < 2 s | 1.2 s | unchanged (2026-09-26) | progress log, Phase 8 | Met |
| 2 | Route line in the correct lane | inside the lane within LiDAR range | pending | pending | Part 12.4 check | **Pending hardware** |
| 3 | Brake trigger time (simulated lead car braking at 6 m/s²) | TTC 1.2–1.9 s | 1.70 s | **1.50 s** (rule 1 now has an onset confirmation, §4) | `test_decision_arbiter --gtest_filter=*LeadCar*` | Met |
| 3 | Brake on a stopped obstacle, real recorded data | — (new) | — | **onset at true TTC 1.58 s and 1.05 s** (two placements); braking holds; 0 false brakes | replay harness, §3.1 | Met on recorded data |
| 3 | Brake never above the ceiling | 0 violations | 0 in 200,000 (host); 0 in 600,000 (hub) | unchanged; tests pass | `test_decision_arbiter` (ArbiterCeiling); firmware `pio test -e native` | Met |
| 3 | Watchdog release after link loss | ≤ 200 ms | logic verified in unit tests | unchanged | firmware tests 38/38; `test_vehicle_interface` | Bench pending |
| 3 | Driver override at maximum intensity | pedal moves, 3/3 | pending | pending | `tools/bench_rig/log_actuator_bench.py checklist` | **Pending hardware** |
| 3 | Power box disconnected mid-brake | released; fault 5; no false latch | logic verified | unchanged | firmware tests | Bench pending |
| 3 | OBD-II over CAN | never transmits at a wrong bit rate | verified against a simulated car | unchanged | firmware `test_obd` | Car pending |
| All | Automated tests | 100% | host 232/232; GPU+routing 29/29; firmware 38/38 | **host unit 289/289; host integration 52/52; Python 8/8; firmware 38/38; harness 22/22** | §6 | Met |

---

## 2. Objective 1: perception and sensor fusion

**Evidence:**
- Inference timing above (Table 3).
- **The whole host running together on a real recorded drive** (camera + LiDAR + GNSS/IMU; KITTI
  raw 2011_09_26 drive 0005): every thread of Part 5.1 from `main`, against the simulated hub
  running the firmware's own SafetyCore. Decisions: "The whole system on a recorded drive, before any
  hardware (2026-10-07)".
- **Five perception defects found on real data, each fixed and tested** (progress log 2026-10-07,
  items 1-5): a ghost track braked on; a corner-exit false brake; a flat-road assumption making a
  rising road an "obstacle"; median-based gap bias (now the object's near face); duplicate
  measurements of one surface. Each is a "problem found, cause, fix, verification" story.
- **Ground plane fitted per LiDAR scan** (seeded RANSAC + least squares): `LidarProcessor`, 6 unit
  tests. Fixed the flat-road false brake.
- **Live Mid-360 capture** built ahead of the hardware: byte-level packet decoding held to the SDK's
  structs at compile time; LiDAR-to-host clock mapping by the minimum offset. Decisions: "The live
  Livox Mid-360 capture... (2026-10-07)".

**Figures:** `fig_demo_frame.png` (the sensors' view beside the driver's view and the decision).

---

## 3. Objective 3: hazard assessment and fail-safe actuation

(Before Objective 2 here because most of the new evidence is about it.)

### 3.1 The replay harness (`python3 tools/bench_rig/replay_recorded_frames.py`): 22/22 PASS

| Check | Result |
|---|---|
| Drive as recorded: no brake decision | 0 brakes in 153 frames |
| Obstacle at 2.4 s (in an S-bend, beside a cyclist): braking starts at true TTC ≥ 1.0 s | 1.05 s |
| Obstacle at 11.0 s: braking starts at true TTC ≥ 1.0 s | 1.58 s |
| Braking holds (no lapse longer than the 300 ms hold) while true TTC > 0.5 s | no lapses, both |
| Measured gap vs truth while braking | mean +0.01 m (worst +0.24 / +0.17 m) |
| No brake while no obstacle is present | 0, both |
| Timing jitter, seeds 1-10, drive as recorded: no brake | 0 |
| Real time (threaded host + hub_sim + the firmware's SafetyCore), with obstacle | BRAKE requests logged; hub applied the brake (peak intensity 60); no FAULT; ordered shutdown with the final zeroed command; exit 0 |
| Real time, drive as recorded | 0 requested, 0 applied (and 6 further runs: 0) |

**Figure:** `fig_brake_timeline.png` (true gap and TTC; threshold; every brake request).

### 3.2 Robustness to real-time timing (`replay_inspect --jitter SEED`)

| | Before the fixes (2026-10-07) | After (2026-10-08) |
|---|---|---|
| Drive as recorded, 40 seeds: runs with a false brake | 6 of 40 | **0 of 40** |
| Obstacle at 11 s, 16 seeds: true TTC at onset | 0.0-1.58 s; 10 below 1.0 s; one never braked | **1.16-1.58 s in 15 of 16**; one at 0.85 s |
| Obstacle at 2.4 s (S-bend), 16 seeds | (not separated) | 0.95-1.05 s in 13 of 16; 0.46-0.76 s in 3 |

**Figure:** `fig_jitter.png`. **Causes and fixes:** progress log 2026-10-07, "Item 1 re-verified under
real-time timing" (one-cycle false brakes; a frame without LiDAR counted as a miss; confirmation
completed by a prediction).

### 3.3 Unit and integration evidence
- Brake ceiling fuzz (200,000 cycles, host; 600,000 events, hub): 0 violations.
- Onset confirmation, measured-run rule, unobserved-cycle rule: dedicated tests in
  `test_decision_arbiter` and `test_multi_object_tracker`.
- Firmware SafetyCore, protocol, OBD: 38/38 (`pio test -e native`).

### 3.4 Pending hardware (state it as such)
The six bench checks (Part 13.3) three times; Stage B-D (Part 13.4). The bench procedure and tool
exist: `docs/assembly/bench_rig.md`, `tools/bench_rig/log_actuator_bench.py`.

---

## 4. Changes since the proposal the report must declare

| Change | Why (evidence) | Where documented |
|---|---|---|
| Rule 1 also requires the **straight** path | corner-exit false brake on KITTI | BUILD_GUIDE 9.3 amendment; progress log 2026-10-07 item 2 |
| Rule 1 requires a **200 ms unbroken LiDAR-measured run** | ghost-track false brake | same, item 1 |
| Rule 1 **onset confirmation**: same target, measured again ≥ 100 ms later (`brake_confirm_ms`) | 6/40 one-cycle false brakes under jitter | progress log 2026-10-07 (re-verification); decisions 2026-10-07 |
| Proposal's simulated brake time 1.70 s is now **1.50 s** | cost of the confirmation (100 ms); still inside 1.2-1.9 s | `test_decision_arbiter` LeadCar |
| A frame without a usable LiDAR scan is **not** a tracker miss | obstacle never braked for under jitter | progress log, same entry |
| LiDAR-camera calibration from **board planes**, not hand-picked corners | Mid-360's non-repeating scan | decisions 2026-10-08; BUILD_GUIDE 12.2 amendment |
| ExtrinsicMonitor: **edge alignment only** (mask agreement dropped), **rotation only**, decisions on held-out frames, no evidence while turning | each from a failure on KITTI's real frames | BUILD_GUIDE 12.2.1 amendment; progress log 2026-10-08 item 3 |
| AR barrier only where rule 1 can brake | barrier drawn at a parked car never braked for | progress log 2026-10-08 (demonstration) |
| OpenCV 4.14 CUDA build accepted for 4.10 | 4.10 predates CUDA 13 | decisions, Phase 0 |
| KITTI used as a stand-in for the project's own recorded drives | no hardware yet; real synchronised camera + LiDAR | decisions 2026-10-07 |

---

## 5. Objective 2, calibration and the rest

- **Calibration tools** (methodology 3.10), tested on synthetic scenes with known answers (CI):
  - camera intrinsics: the recovered lens projects within **0.34 px** of the truth over the region
    the views covered (up to a 0.09° rotation the extrinsics absorb); 2.1 px in the extreme corners
    no view reached; refuses frontal-only sets. `host/test/scripts/test_camera_calibration.py`.
  - LiDAR-camera (board planes): from a start 3° and 10 cm off, **0.39°**, LiDAR points within
    **6.8 px** of the truth at 5-40 m (f = 1000 px). `test_lidar_camera_calibration.py`.
  - **Figure:** `fig_calibration.png` (synthetic inputs through a known lens).
- **ExtrinsicMonitor on KITTI's real frames:** from KITTI's own calibration and from a mount knocked
  0.72° off it, both end within 0.22° and 0.34° of KITTI's; a 2.5° knock is DEGRADED; the 2.5° error
  is never "verified". `test_extrinsic_monitor` (integration).
- **The system records itself** (`--record`): KITTI recorded through the live host (153 frames, 154
  scans, 840 hub reports, nothing dropped) and replayed: no brake as recorded; braking from TTC 1.37 s
  with the obstacle.
- **Hardware documentation:** schematics pass ERC and the netlist cross-check against the firmware's
  pin map (`hardware/gen/check.sh`); assembly procedures generated from the board descriptions
  (`docs/assembly/`), e.g. +5 V rail computed at 5.00 V.
- **Navigation** (route line): the overlay design and projection are measured above; the route line
  in a real lane is **pending** (Part 12.4) and absent from the KITTI demonstration (the map is
  Nairobi's).

---

## 6. Test counts (2026-10-08)

| Suite | Count | Command |
|---|---|---|
| Host unit (CPU, CI) | 289/289 | `ctest --test-dir host/test/unit/build` |
| Host integration (pipeline, camera, gpu, osrm) | 52/52 | `ctest --test-dir build` |
| Calibration tools (Python, CI) | 8/8 | `python3 -m pytest host/test/scripts` |
| Firmware (native) | 38/38 | `cd firmware/sensor_actuator_hub && pio test -e native` |
| Whole-system replay harness | 22/22 | `python3 tools/bench_rig/replay_recorded_frames.py` |
| Hardware design checks | ALL PASS | `hardware/gen/check.sh` |

---

## 7. Limitations to report as they are

- **S-bends:** braking for a stopped obstacle starts late (true TTC 0.46-1.05 s under jitter):
  rule 1's constant-curvature path cannot follow a reversing bend, and it also requires the
  straight path. Road geometry from the map is the candidate remedy (untested).
- **One jitter outlier at 0.85 s:** the injected obstacle stands beside a real cyclist whose track
  takes its measurements on stale-scan frames: partly an artefact of the test scene.
- **KITTI is not the project's sensors:** a Velodyne HDL-64E and a German street under Kenyan rules
  and a Kenyan sign detector. It proves the pipeline, not the Mid-360's coverage.
- **Calibration accuracy** is limited by the camera's estimate of each board plane; the monitor
  refines rotation only; translation is weakly observable from edges.
- **Everything physical is pending:** bench gate, installation, Stage B-D, the route line in a lane,
  the overlay tuning on the real display.

---

## 8. Methods that need a citation (verify each on Google Scholar, IEEE format)

| Method used | Candidate reference |
|---|---|
| Camera calibration (OpenCV's calibrateCamera) | Z. Zhang, "A flexible new technique for camera calibration," *IEEE TPAMI*, vol. 22, no. 11, 2000 |
| Camera-LiDAR calibration from board planes | Q. Zhang and R. Pless, "Extrinsic calibration of a camera and laser range finder," *IROS*, 2004 |
| Edge-alignment camera-LiDAR monitoring | J. Levinson and S. Thrun, "Automatic online calibration of cameras and lasers," *RSS*, 2013 |
| The recorded drive | A. Geiger, P. Lenz, C. Stiller, R. Urtasun, "Vision meets robotics: The KITTI dataset," *IJRR*, vol. 32, no. 11, 2013 |
| Ground-plane fitting | M. Fischler and R. Bolles, "Random sample consensus," *Comm. ACM*, vol. 24, no. 6, 1981 |
| Tracker motion models | H. Blom and Y. Bar-Shalom, "The interacting multiple model algorithm for systems with Markovian switching coefficients," *IEEE TAC*, vol. 33, no. 8, 1988 |

Check them against the proposal's reference list first; several of its entries may already cover
these.

---

## 9. Demonstration (oral defence)

- `data/demo/demo_obstacle.mp4` and `demo_clean.mp4` (not in git; regenerate with
  `python3 tools/bench_rig/make_demo_video.py --out data/demo/obstacle` and `--obstacle "" --out
  data/demo/clean`).
- Say on screen what it is: a recorded drive replayed through the whole system, a simulated
  obstacle, no hardware; braking is a request.

## 10. Regenerating this pack's numbers

`tools/bench_rig/make_demo_video.py` (both runs), then `tools/report/make_evidence_figures.py
--jitter-seeds 16`, then the commands in Tables 1 and 6.
