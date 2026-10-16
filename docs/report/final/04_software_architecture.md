# 4. Software Architecture

## 4.1 Process and threads

The host is one process with one thread per subsystem, joined by single-producer,
single-consumer ring buffers. A general publish–subscribe framework was rejected: the set of
subsystems is fixed, and a ring with exactly one reader and one writer is simple to reason about.
Where two threads need the same messages, the producer writes one ring for each. Real-time
consumers read only the newest item, which is where stale camera frames and LiDAR scans are
dropped.

| Thread | Consumes | Produces |
|---|---|---|
| Camera | the camera | undistorted frames, each carrying its own camera model |
| Inference | frames | detections, masks, signs, lanes, depth |
| LiDAR | LiDAR packets (live or replayed) | scans with their fitted ground plane |
| Fusion | hub reports, scans, detections | the ego pose; the fused, tracked scene |
| Decision | the scene and pose | one actuation request per cycle, to the vehicle interface |
| Navigation | pose, ground plane, lanes | the road-projected route |
| Render | frames, detections, scene, route | the overlay scene, to the display sink |
| Vehicle interface | the hub's serial link | hub reports in; heartbeats and commands out |
| Main | — | the system manager: configuration, start and stop order, the event log |

Start order matters: inference first (its engines take seconds to load), then fusion, render,
navigation, the hub link and decision, and the sensors last, so a replayed drive's clocks start
together.

## 4.2 Subsystems as built

| Subsystem | As built | Change from the proposal (decision log) |
|---|---|---|
| Camera pipeline | OpenCV capture (the camera, or a recorded file for replay), undistortion by a precomputed remap | frames carry their own camera model (2026-10-07) |
| Inference | TensorRT engines from ONNX: YOLOv8m-seg {{cite:terven2023}}, YOLOv8s fine-tuned on MTSD {{cite:ertler2020}}, UFLD v2 {{cite:qin2024}}, MiDaS {{cite:ranftl2022}} | segmentation instead of boxes, so overlays follow outlines; a fourth network for signs (Phase 5) |
| LiDAR | ground plane per scan (seeded RANSAC and least squares); live Mid-360 capture through Livox SDK2 | none |
| Scene reconstruction | LiDAR points inside each object's camera mask give its range; MiDaS only where no LiDAR point lands, and never for braking | mask-based fusion (2026-09-25) |
| Ego-motion | an extended Kalman filter {{cite:kalman1960}} on GNSS, IMU (rotated into the vehicle frame) and OBD speed | hub reports feed the filter directly (2026-10-06) |
| Tracking and prediction | an interacting-multiple-model tracker {{cite:blom1988}}: constant velocity for people, CTRV by an unscented filter {{cite:julier2004}} for vehicles, a stopping model; Hungarian assignment {{cite:kuhn1955}} | replaces the constant-velocity tracker (2026-09-25) |
| Reckless driving | rules on the tracked state: tailgating (time gap), erratic speed (detrended spread), swerving (lateral spread after a fitted path) | the learned temporal classifier was not built; the rules are explainable and were tuned on a discrimination table |
| Decision arbiter | the single time-to-collision brake rule, hazard lights on hard braking, warnings | conditions made explicit and each tested (Phase 10, 2026-10-07) |
| Navigation | OSRM on a Nairobi-region extract {{cite:luxen2011,delling2011}}; trace map-matching {{cite:newson2009}}; road status in three layers | live road status layer (2026-09-26) |
| Road-surface projector | route on the measured ground, lane-corrected | heading and lateral corrections as persistent states (Phase 9) |
| Renderer | the GPU draws an overlay scene: object glows, road-space bands and barriers, image badges | GPU rendering; hazards as glows, not boxes (2026-09-25) |
| Vehicle interface | the only owner of the hub link; tested against a simulated hub | built against a simulated hub (2026-10-06) |
| Event log and recorder | every actuation event with its context; the system records itself in the replay format | the recorder (2026-10-08) |

## 4.3 The display seam

The renderer does not hand the display a finished picture. It hands over the video frame and an
**overlay scene**: a list of items, each with its geometry in one of three spaces and a style.
- *Object-mask space* (camera pixels): a tracked object's segmentation mask, for the hazard glow
  that follows its outline.
- *Image space* (camera pixels): badges, icons, text.
- *Road space* (metres in the vehicle frame): the route band, stop barriers, hump markers, which
  the GPU projects in its vertex shader with the same camera model perception uses.

The display enforces the overlay design's rule "never hide a hazard": a glow is a rim of light and
at most a light tint, and road-space items are masked by hazard silhouettes, so a barrier appears
behind a pedestrian rather than across them. The same seam lets an optical display draw only the
overlays through a combiner and ignore the video (Section 11).

## 4.4 Verification

[RESULT: the final test counts; as of 2026-10-08: host unit 289/289, host integration 52/52,
Python 8/8, firmware 38/38, the replay harness 22/22 (`docs/report/evidence/README.md`)]

- Unit tests (GoogleTest) for every subsystem; integration tests run the GPU and routing paths;
  the firmware's safety logic is tested on the laptop (`pio test -e native`).
- **The whole system on a recorded drive** (KITTI): every thread from `main`, against a simulated
  hub running the firmware's own safety core, with an obstacle injected so both the camera and the
  LiDAR see it. Eleven defects were found and fixed this way before any hardware existed.
- Continuous integration builds the host and firmware, runs the tests, and checks that the two
  copies of the protocol are identical.
