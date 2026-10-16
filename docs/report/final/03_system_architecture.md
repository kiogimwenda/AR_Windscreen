# 3. System Architecture

[FIGURE: the as-built architecture, redrawn from `docs/architecture/system-architecture.md`;
the proposal's `fig_architecture.svg` and `fig_installation.svg` are the starting point]

## 3.1 Four layers, one principle

The system has four layers. **Sensors** observe the road and the car: a camera, a Livox Mid-360
LiDAR, a GNSS receiver, an IMU, the car's own OBD-II data, the brake-light switch and a gesture
sensor. A **hub** (an STM32F405 microcontroller running FreeRTOS) reads the low-rate sensors, owns
every electrical connection to the car's actuators, and enforces the safety rules. The **host** (a
laptop with an NVIDIA RTX 5060) reads the high-rate sensors (the camera over USB, the LiDAR over
Ethernet) and runs perception, fusion, tracking, navigation, decision and rendering. The
**outputs** are the AR view, the signal relays (indicators, hazard lights, horn, high beam), the
brake actuator, and an evidence log.

The split follows one principle: **the laptop decides, the hub disposes.** The laptop has the
computing power for vision and mapping but runs a general-purpose operating system that can crash
or stall. The hub is small, deterministic and always running. The laptop may only send a bounded
*request*; the hub decides whether to act on it, clamps it to its own limits, and releases
everything by itself if the laptop falls silent (Section 5).

## 3.2 The five modules

| Module | Runs on | Does |
|---|---|---|
| Perception | host GPU | object detection with masks (YOLOv8m-seg), road signs (YOLOv8s on MTSD), lanes (UFLD v2), relative depth (MiDaS); LiDAR ground plane and object ranges |
| Fusion and tracking | host CPU | the ego-motion filter (GNSS, IMU, OBD speed); camera-mask LiDAR fusion; an IMM tracker with motion prediction |
| Decision | host CPU | reckless-driving classification (warnings); the arbiter, the only producer of actuation requests |
| AR display and navigation | host GPU | OSRM routing, map-matching, the road-surface projector; the overlay renderer and its display sink |
| Sensor and actuator hub | STM32F405 | 50 Hz sensor reports; the watchdog; the brake, magnet and relay drivers; OBD-II over CAN |

## 3.3 The link between host and hub

One USB serial link carries a small framed protocol (start byte, type, length, payload, CRC-16),
defined once and copied to both sides; continuous integration fails if the two copies differ.
- The hub sends a `SensorReport` at a fixed 50 Hz (GNSS, IMU, OBD speed and rpm, brake pedal,
  ignition, kill-switch state), which the host's filter relies on for a known time step.
- The host sends a `HEARTBEAT` at 10 Hz always, and an `ActuationCommand` every decision cycle,
  zeroed when it has nothing to request. "No request" and "link dead" are therefore
  distinguishable on the hub.
- The hub answers every command with an `AckStatus` (accepted or rejected, the applied values,
  any fault code).

## 3.4 Physical layout: the two-box hub

The hub is built as two units on two custom printed circuit boards, joined by a 25-way cable.

- **The windscreen pod**, behind the rear-view mirror, holds the camera and the pod board
  (70 × 53 mm, 4 layers): the microcontroller, the BNO085 IMU, the NEO-M8N GNSS, the gesture-sensor
  connector, the USB link and status LEDs. The camera and the board are bolted to the two faces of
  one rigid carrier wall, so the IMU measures the camera's own motion and the camera–IMU geometry
  cannot drift.
- **The power box**, under the dash on the driver's side, holds everything that switches current,
  on the power board (95 × 66 mm, 2 layers, 2 oz copper): 12 V input protection, the 5 V supply,
  the brake actuator's motor driver and current sensor, the cable magnet, five relays, the 40 A
  kill relay, the CAN transceiver for the OBD-II port, and the brake-light optocoupler. High current
  stays short and far from the IMU and GNSS.
- **The LiDAR** is on the roof's front edge on a plate tilted 15° forward, so its lower beams reach
  the road about 4 m ahead instead of 12.6 m. It cannot go behind the windscreen: glass at a
  shallow angle reflects and bends its 905 nm beams, the cabin hides most of its 360° view, and it
  is rated only to 55 °C {{cite:livox2024}}.
- **Within the driver's reach:** a mushroom-head emergency stop and the gesture sensor.

Every line from the pod enters the power board through a pull-down resistor, so with the pod
unpowered, resetting or unplugged, every actuator line reads "off" in hardware. Section 5 shows
that each safety property survives the split.

## 3.5 Design method

The system was built in seventeen phases, each with written exit criteria. Everything that does
not need hardware was built and verified first against recorded road video, a recorded GNSS drive,
a public recorded drive with LiDAR (KITTI) and synthetic scenes, so that the hardware's arrival
leaves only integration. The two PCBs were described in Python, generated as KiCad schematics,
checked against the firmware's pin assignments and the cable table, placed and routed by script,
and their key circuits simulated before ordering (Section 9.1, Figures 9.1–9.5). Every task is recorded in a progress log and every
choice made under ambiguity in a decision log.
