# System Architecture

Kept in sync with `BUILD_GUIDE.md` Part 5. This is the **as-built** record — when the implementation
deviates from Part 5, the deviation is described here and indexed in `decisions.md`, rather than
this file being quietly rewritten to match whatever the code happens to do.

> Status: Phase 0 scaffold. Nothing below is implemented yet; this file records the intended
> structure so the report's architecture chapter (Part 16.1 §3) has a source that grew with the
> build rather than being reconstructed at the end.

## The hard boundary

The single most important structural fact about this system is the split between `host/` and
`firmware/`:

- The **host** (laptop) does all perception, fusion, decision and rendering. It never drives
  actuation hardware. It can only ever send a bounded *request* to the hub.
- The **hub** (STM32/FreeRTOS) is the only component with direct electrical access to vehicle
  sensors and actuation wiring. It enforces the real force ceiling, in firmware, independent of
  whether the host is correct or even running.

Everything else in the architecture follows from keeping that boundary intact.

## Threads (Part 5.1)

One process, one thread per subsystem, connected by single-producer/single-consumer ring buffers
(Part 5.3) — deliberately not a general pub/sub framework, since the set of subsystems is fixed.

A bus named below with more than one consumer (`frameBus` feeds inference and render;
`detectionBus` feeds fusion, navigation and render) is **one `RingBuffer` per consumer**, and the
producer pushes to each one. An SPSC ring is only race-free with exactly one reader, so sharing one
between two consumers would be a data race. Real-time consumers read with `popLatest()`, which is
where frame/LiDAR "drop oldest" happens (see `RingBuffer.h`).

| Thread | Consumes | Produces |
|---|---|---|
| `CameraThread` | camera device | `frameBus` |
| `InferenceThread` | `frameBus` | `detectionBus` |
| `LidarThread` | LiDAR packets | `lidarBus`, `groundPlaneBus` |
| `FusionThread` | `VehicleInterface`, `lidarBus`, `detectionBus` | `VehiclePose`, fused scene model |
| `DecisionThread` | fused scene model, pose | `ActuationRequest` → `VehicleInterface` |
| `NavigationThread` | `VehiclePose`, `groundPlaneBus`, `detectionBus` | `navOverlayBus` |
| `RenderThread` | `frameBus`, `detectionBus`, scene model, `navOverlayBus` | `DisplaySink` |
| `VehicleInterfaceThread` | hub serial link | `SensorReport`, hub frames out |
| main | — | owns `SystemManager`, config, `EventLog` |

`NavigationThread` runs at the LiDAR ground-plane refresh rate: there is no benefit to projecting
the route overlay faster than the geometry it is projected onto updates.

## Data flow

```
camera ──► CameraPipeline ──► frameBus ──┬──► MlInferenceEngine ──► detectionBus ──┐
                                         │                                         │
                                         └──────────────────────────────► ArRenderer ──► DisplaySink
LiDAR ──► LidarProcessor ──┬──► lidarBus ────► SceneReconstruction ──┐              ▲
                           │                                          │              │
                           └──► groundPlaneBus ──────────────┐        │              │
                                                             ▼        ▼              │
hub ──► VehicleInterface ──► SensorReport ──► SensorFusion ──► SceneModel ──► DecisionArbiter
                    ▲                              │                                │
                    │                              └──► NavigationEngine ──► MapMatcher
                    │                                        └──► RoadSurfaceProjector ──► navOverlayBus
                    └──────────────── ActuationCommand ◄───────────────────────────────┘
```

## Notes to carry into the report

- `DecisionArbiter` is the only subsystem permitted to emit an `ActuationRequest`, and only its
  time-to-collision rule may request braking — never a raw ML classifier output (Part 9.3).
- `GroundPlaneModel` is *published*, not discarded after clustering. Dropping that publish produces
  no error, only a nav overlay that silently falls back to a flat-ground assumption everywhere
  (Part 8.1).
- `DisplaySink` exists so the future optical-combiner path is a clean swap; only `WindowedSink` is
  implemented in this build (Part 0).
