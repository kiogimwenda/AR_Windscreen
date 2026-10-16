# 2. Motivation and Scope

## 2.1 Motivation

Driving is mostly a visual task, and every glance away from the road leaves the driver briefly
blind to what is happening ahead. Naturalistic driving studies have found such glances among the
strongest predictors of a crash {{cite:klauer2006,dingus2016}}. Kenya's National Transport and
Safety Authority recorded 4,690 road deaths in 2022, 1,682 of them pedestrians and 1,254
motorcyclists {{cite:ntsa2023}}; worldwide, more than nine in ten road deaths happen in low- and
middle-income countries {{cite:who2023}}.

Two families of technology address the driver's side of this problem. Augmented-reality head-up
displays put guidance and hazard cues where the driver is already looking, attached to the real
scene {{cite:gabbard2014,bark2014}}; driver-assistance systems such as forward collision warning
and automatic emergency braking react when the driver does not, and have been associated with
large reductions in rear-end crashes {{cite:cicchino2017}}. Both are factory-fitted to new,
mid-range and premium vehicles. The older imports that make up most of Kenya's fleet will never
receive them.

This project builds a retrofit: one sensor-fusion model of the road ahead that drives both an AR
view (hazards and a road-locked navigation line) and a bounded automatic response (indicators,
hazard lights, horn and, as a last resort, a force-limited brake the driver can always overpower).
Its components are now affordable: a consumer laptop GPU runs four neural networks on 2K video in
real time, a compact LiDAR measures the road surface and the range to every object, and a
microcontroller enforces the safety rules in hardware.

## 2.2 Scope

**In scope (built):**
- Perception: object detection with per-object masks, road-sign detection, lane detection and
  relative depth on the GPU; LiDAR processing (ground plane per scan, object ranges from
  camera-masked LiDAR points); an ego-motion filter fusing GNSS, IMU and OBD-II speed.
- Tracking and motion prediction of every road user (an interacting-multiple-model tracker), and
  rule-based classification of reckless driving (tailgating, erratic speed, swerving).
- A single, auditable braking rule based on time-to-collision, and a decision arbiter that is the
  only part of the software allowed to request actuation.
- An AR renderer that draws hazards, signs and a road-locked route line over the live video.
- Offline navigation on OpenStreetMap (OSRM), map-matched to the road, with live road status.
- A two-box hub (a windscreen pod and an under-dash power box, on two custom PCBs) that owns every
  electrical connection to the car and enforces the safety rules independently of the laptop.

**Video see-through, not an optical windscreen display.** The AR view is shown on the laptop's
screen as the live camera image with the graphics composited into it. This is an acknowledged
simplification of the proposal's "windscreen display": it removes the optical combiner, the
projector and the eye-position calibration, so the project's time goes to perception, safety and
navigation precision. The software keeps the optical path open: the renderer hands every display
an overlay scene (what to draw, and where in the road or the image), not a finished picture, so an
optical `ProjectorSink` can later draw only the overlays through a combiner without changing
anything upstream (Section 11).

**Out of scope, deliberately:** steering of any kind; autonomous driving of any kind (the system
informs, signals and can brake; it never drives); a thermal or infrared camera; cloud services,
over-the-air updates and companion apps.

## 2.3 Safety properties the system must have

Five properties were fixed before any code was written and are verified on the bench and in the
car (Sections 5 and 9):
1. The driver's foot always overrides the actuator.
2. A physical kill switch removes actuator power independently of any software.
3. The hub releases the actuator within 200 ms of losing the laptop, with no dependency on the
   laptop's software.
4. The laptop never drives actuation hardware: it only sends a bounded request, and the hub
   enforces the real ceiling.
5. Every actuation event is logged with a timestamp, on both the laptop and the hub.

## 2.4 Navigation precision as a primary objective

A navigation arrow floating over a video is easy; a route line that lies on the road surface, in
the correct lane, through slopes and dips, is not. GNSS alone is good to a few metres, which is
not enough to choose a lane. The system therefore combines three things it already measures:
map-matching to know which road and how far along it, the LiDAR's measured ground surface to know
the road's height, and the lane-detection network to place the line in the lane. The precision is
honest about distance: near the car, within the LiDAR's range, the line sits on the measured
surface; farther out it falls back to a flat-ground assumption (Section 6).
