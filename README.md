# AR_Windscreen

**AR-Assisted Semi-Autonomous Driving System** — a retrofit driver-support platform centered on
precision, sensor-fused AR navigation and force-limited collision-avoidance braking.

The system fuses camera, LiDAR, GPS, IMU and OBD-II data into a real-time model of the road ahead,
renders that understanding as a road-locked augmented-reality overlay on the vehicle's forward view,
and — through a dedicated STM32 sensor/actuator hub — can trigger indicators, lights and a
force-limited brake-pedal actuation when a collision risk crosses a defined threshold.

## Safety

This project has one hard rule that overrides every other design consideration:

> The brake actuator must never be able to exert more force than a driver's foot can override, must
> never disconnect or block the driver's own pedal linkage, and must default to fully released/inert
> on any fault, timeout, link loss, or ambiguous state.

The host laptop never drives actuation hardware directly — it sends a bounded *request* to the hub,
and the hub's firmware enforces the real ceiling, independent of host software correctness. A
physical kill switch removes actuator power independent of software, and the hub's watchdog releases
the actuator within 200 ms of host link loss.

See `docs/BUILD_GUIDE.md` Part 0 and Part 9 for the full safety design.

## Layout

| Path | Contents |
|---|---|
| `docs/` | Build guide, design decisions, progress log, architecture, report and presentation sources |
| `firmware/sensor_actuator_hub/` | STM32 / FreeRTOS firmware — the only component with direct electrical access to the vehicle |
| `host/` | Laptop-side C++ application: perception, fusion, decision, navigation, rendering |
| `tools/bench_rig/` | Bench-test instrumentation and offline replay harness |
| `cmake/` | Custom CMake find-modules (TensorRT) |

## Documentation

- `docs/BUILD_GUIDE.md` — the full implementation specification this project is built from
- `docs/decisions.md` — running log of implementation choices made under ambiguity
- `docs/progress-log.md` — dated, per-task record of work completed
- `docs/architecture/system-architecture.md` — as-built architecture
