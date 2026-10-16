# 5. Actuation Safety Design

This section is written as a small safety case: what can go wrong, what the design does about it,
and what evidence shows that it works. It follows the spirit of ISO 26262's hazard-driven
approach {{cite:iso26262}} without claiming compliance with it.

## 5.1 Risk tiers

The system's outputs are tiered by the harm a wrong output could do, and each tier gets
proportionate protection.

| Tier | Outputs | Worst wrong output | Protection |
|---|---|---|---|
| Information | the AR view, warnings, tones | a missed or false warning | the driver remains responsible; "never hide a hazard" in the renderer |
| Signalling | indicators, hazard lights, horn, high beam | a needless signal | relays off by default in hardware; the hub drops them on link loss |
| Braking | the brake actuator | an unwanted or unreleased brake | every measure below |

## 5.2 The braking rule

Only one rule may request braking, and it is a geometric quantity that can be explained in a
viva: **time to collision** {{cite:lee1976}}, the gap to the nearest object in the car's path
divided by the closing speed, below 1.8 s, while the driver is not already braking. Machine-learned
outputs never request braking directly: the networks find objects, but the LiDAR measures their
range, and the rule decides.

A target may trigger the rule only if **all** of these hold (each tested):
- it is a confirmed track, not tentative and not coasting unseen;
- its range was **measured by the LiDAR** within 200 ms, never estimated from monocular depth;
- the LiDAR has measured it **without a break for 200 ms** (three scans);
- it lies in **both** the car's turning path and its straight path, within 1.2 m either side;
- it is closing at more than 0.5 m/s, and every number involved is finite;
- the gap is taken from the front bumper, not the tracking reference at the rear axle.

Braking **starts** only after the condition has held on the same target on every cycle and the
LiDAR has measured it again at least 100 ms later. A measurement confirms; a prediction never
does. The last three conditions were added after replaying a real recorded drive (KITTI) under
realistic timing variations: each removed a class of one-cycle false brake (a track re-associated
to an unrelated fragment, a corner exit sweeping across a parked car, a one-scan velocity spike).
Their cost is bounded and stated: up to 200 ms after a missed association, and 100 ms (0.8 m at
30 km/h) at onset.

Motion prediction (the IMM tracker's forecasts, closest point of approach, collision
probability) drives warnings and the display only. Braking on a forecast risks *phantom braking*,
itself a hazard to the car behind.

## 5.3 Armed, and the triple ceiling

The host considers the system **armed** only with a hub report no older than 100 ms, the kill
switch not engaged, an acknowledgement with no fault, and a healthy event log: a system that
cannot record actuation evidence must not actuate. Unknown always counts as not armed.

The brake intensity is limited three times: the arbiter requests 60 (of 255), clamped to the
configured maximum of 90; the hub's safety core clamps to its compiled 90; and the brake driver
clamps again at the last line before the PWM output (a compile-time check keeps the two literals
equal). The system manager refuses to start with a configured ceiling above the hub's. The hub
also limits any one brake application to 1.5 s, whatever the host asks.

## 5.4 The hub's rules, independent of the laptop

All of the hub's safety rules live in one hardware-free class, `SafetyCore`, unit-tested and
fuzzed on the laptop; the FreeRTOS tasks only feed it and apply what it returns.
- **Watchdog (property 3):** no valid frame from the host for 200 ms, or the hub's own report loop
  stalled for 200 ms, releases the brake and the relays and disarms. Re-arming needs a fresh
  heartbeat while every other condition holds, and every disarm forgets the last command, so a
  re-arm never resumes an old brake. The watchdog task reads the release flag without taking the
  core's lock, so a task hung while holding it cannot keep the brake on.
- **Independent hardware watchdog:** if the firmware itself hangs, the microcontroller's
  independent watchdog resets it within 500 ms, and during reset every line to the power box falls
  to "off" through its pull-down.
- **Overcurrent:** above 8 A for three consecutive 10 ms ticks (so motor inrush does not trip it),
  everything is released and a fault latches until a power cycle; software cannot clear it.
- **The power box's presence:** if it is absent (cable out, or no ignition) the hub disarms and
  reports fault 5, and ignores the current-sense reading, which on an open wire would read as a
  false 25 A overcurrent.
- **The car's CAN bus:** the hub reads OBD-II over CAN {{cite:iso15765}} but must never disturb a
  bus that carries engine, ABS and airbag traffic. It listens in silent mode to find the bit rate,
  sends read-only requests once each at the load of a handheld scan tool, and its transceiver is
  held in standby in hardware whenever the pod is silent.

## 5.5 The hardware: the driver and the kill switch win

**Property 1, the driver's foot overrides.** [CHECK: the actuator design as approved by the supervisor; this is the design put to him (BOM note B).] The actuator pulls the pedal through a cable that
can only pull: if the driver presses further, the cable goes slack, so the pedal linkage can never
be blocked. The cable is held by a 12 V electromagnet powered through the kill relay: any loss of
power drops it free, and its rated holding force is a mechanical force ceiling no software can
exceed. A lead-screw actuator was rejected: it is not back-drivable, so a power cut would leave the
brake applied. [RESULT: the bench test, Part 13.3 item 3: the pedal moves under the driver's foot
at maximum intensity, 3/3, with the video]

**Property 2, the kill switch.** A mushroom-head emergency stop with a normally-closed contact
carries the 40 A kill relay's coil current. Pressing it, a broken wire or no ignition all open the
relay and remove power from the actuator and the magnet, with no software in the path. Its second
contact tells the hub, wired normally-closed to ground so that a broken sense wire also reads
"engaged". [RESULT: kill switch tested on the bench and installed]

**Property 4, the laptop only requests.** The laptop has no electrical path to any actuator: it
reaches the car only through the hub's protocol. On the power board, every line from the pod
passes through a 10 kΩ pull-down, so an unpowered, resetting or unplugged pod leaves the brake
drive, the magnet and every relay off. Simulated with the real cable and pull-down values, an
unplugged pod leaves the driver's gate at 0 V and the coil off; with the pod driving, the gate
reaches 2.99 V and the magnet lets go 6.6 ms after release (Section 9.1.3, Figure 9.3).

## 5.6 Failure analysis

| Failure | What happens | Why it is safe | Evidence |
|---|---|---|---|
| Laptop crashes or USB pulled | the pod stays up on cable power; its watchdog releases within 200 ms | firmware, no host code involved | [RESULT: hub-measured release time, Part 13.3 item 5] |
| Inter-box cable unplugged or cut | pull-downs release everything; the pod reports fault 5 | hardware first; firmware agrees within 10 ms; no false overcurrent latch | firmware tests; [RESULT: bench, item 6] |
| The pod loses power | lines float to their pull-downs | hardware | relay-driver simulation, pod unplugged: gate 0 V, coil off (Figure 9.3) |
| The firmware hangs | the independent watchdog resets within 500 ms | hardware backstop | — |
| The power box loses 12 V | the kill relay opens; fault 5 | hardware | — |
| Current-sense wire breaks | reads high: latched release | fails towards release | firmware tests |
| CAN wires cut, pod silent | transceiver in standby | cannot disturb the car's bus | firmware tests (simulated car) |
| A false target ahead | the rule's conditions and onset confirmation | measured, confirmed range only | replay harness: 0 false brakes on the recorded drive |
| A real target, driver not braking | brake at TTC < 1.8 s, intensity ≤ 90, ≤ 1.5 s | bounded and released on any fault | [RESULT: staged tests, Part 13.4] |

## 5.7 Evidence trail (property 5)

Every actuation request the arbiter produces is written to the event log with its full context
(the rule that fired, the range, closing speed and time to collision, the requested and clamped
intensity) before it is sent, and the first idle cycle after a request is logged as its release.
The hub reports what it applied and why it released in every acknowledgement.
[RESULT: event-log extracts from the staged tests]
