# Integration tests — see docs/BUILD_GUIDE.md Part 13.2

Unlike `host/test/unit/`, these exercise the real stack and need real dependencies or hardware:

- **Offline replay** — `tools/bench_rig/replay_recorded_frames.py` feeds a pre-recorded camera+LiDAR
  dataset through the full pipeline and checks it runs end to end and produces a plausible
  `DetectionFrame`/`ActuationRequest` stream. This validates the whole software stack *before* any
  hub or vehicle hardware is involved.
- **Firmware loopback** — the real `VehicleInterface` against the real firmware on the bench,
  asserting 50 Hz `SensorReport`s and correct bench-wired relay behaviour from a synthetic
  `ActuationRequest`. This is Phase 3's exit criterion.
- **Road-projection precision check** — Part 12.4's procedure, run here as a gating check rather
  than a one-off. Phase 11 is not complete until it passes; a visibly drifting nav overlay is as
  much a "this subsystem doesn't work yet" signal as a failing unit test.

Built with the main host build and run with `ctest --test-dir build -L <label>`:

| Label | Tests | Needs |
|---|---|---|
| `gpu` | TensorRT engine, GPU pre-processing, the windowed renderer | the GPU (and WSLg for the window) |
| `osrm` | routing and map-matching | the OSRM tools |
| `camera` | `CameraPipeline` through real OpenCV and GStreamer: order, timestamps, rate, buffer ownership, both camera formats' decode pipelines, undistortion against the published model, calibration mismatch, stall detection, prompt stop, bus overflow, file replay | GStreamer only: `videotestsrc` stands in for the camera |

None of these run in CI (Part 13.5 is explicit about that) — they run on the actual machine, on the
bench, and eventually on the vehicle.
