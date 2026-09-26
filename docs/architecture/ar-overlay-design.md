# AR overlay design: road signs and hazards

**Status:** design, agreed direction. Road signs 2026-09-24; hazards added 2026-09-25, when this
file replaced `sign-behaviours.md`. This is an extension beyond `BUILD_GUIDE.md`, which
Part 10.3 now points to. Rendering is GPU work (Part 10, amended 2026-09-25). Everything here is
implemented in the phases named in the last section.

**Goal (from Ian):** the system shows the driver what it understands, immediately and spatially,
the way racing games do. The two objectives are **navigation** and **road safety**.
- **Road signs:** a stop sign raises a holographic barrier where the car must stop. A no-turn sign
  closes off the forbidden road with barriers. A speed limit is shown against the car's actual
  speed, and the route line is colour-graded, F1-game style, to say "slow down".
- **Hazards** (a dangerous car, pedestrian, animal or other obstacle): no rectangles. They use the
  same road-placed language as signs. In addition, the object itself gets a subtle, shimmering
  glow whose colour follows its risk.

---

## 0. One visual language

The driver learns one set of meanings, and signs and hazards both use it:

| Element | Always means | Used by |
|---|---|---|
| Barrier standing on the road | do not go past here / stop before this | stop, no entry, no-turn, collision risk, obstacle |
| Route-line colour, green → amber → red | adjust speed: fine / ease off / brake now | speed limits, humps, crossings, collision risk |
| Zone painted on the road | keep this space clear | tailgating gap, crossing area |
| Glowing lane line (+ hatching beyond it) | don't cross / don't drift this way | no overtaking, swerving neighbour |
| Shimmering glow on an object | this object is a hazard; colour = how dangerous | any hazard object |
| Ring on the ground at an object's base | the warning refers to this object | pedestrians, animals, obstacles |

Colour means the same thing everywhere: **green** is fine, **amber** is caution or ease off, and
**red** is danger or stop. White is used only for neutral information.

---

## 1. Rules that apply to every overlay

These come first, because an AR overlay that is wrong, or in the way, is itself a hazard.

1. **Overlays inform. They never actuate.** No overlay, sign detection or rendering state ever
   produces an `ActuationRequest`. Braking stays exactly as Part 9.3 specifies (collision risk,
   decided by the arbiter). A hazard overlay *explains* a brake decision; it is never an input to
   it. A wrong detection can at worst draw a wrong graphic. It can never move the brake.
2. **Confirm before showing.** A sign or hazard is displayed only once it has been tracked across
   several consecutive frames above a confidence floor (starting at ≥ 3 frames and ≥ 0.5, tuned
   from real footage). One-frame flickers, like the wall "vehicle" false positive seen in the
   Kraków video, never reach the screen.
3. **Only what applies to us.** Kenya drives on the **left**. Signs for our carriageway sit mainly
   on the left verge or overhead. A sign or object well away from the ego path is shown with lower
   prominence or not at all. The tracker's position (and the LiDAR range) decides this, not the
   class alone.
4. **Never hide a hazard.** The object glow is a rim of light along the object's outline, with at
   most a faint (~25%) tint inside, so the object itself stays fully visible. Road-placed graphics
   (barriers, zones, the route band) are masked by the object's silhouette, so they appear *behind*
   a pedestrian or car, never painted across them.
5. **Priority and declutter.** Hazards are **never** removed for declutter. Sign elements are
   capped at three visible at once. Priority: collision risk > other hazards > stop / no-entry >
   speed > turn restrictions > advisories > information. Lower-priority sign items fade out when
   something higher appears.
6. **Degrade honestly.** Road-placed graphics need the object's distance and the road surface
   (Parts 8, 11.4). Without them, the same information is shown screen-fixed: a banner for signs, a
   red chevron at the screen edge pointing towards the hazard. Never a 3D element that might float
   in the wrong place. If an object has no segmentation mask, its glow falls back to a soft ellipse
   inside its box. This is the Part 0 honesty rule, applied to the display.
7. **Map and sign together.** OpenStreetMap (already used through OSRM) carries speed limits
   (`maxspeed`), stop and give-way positions, and turn restrictions. A detected sign **confirms or
   overrides** the map. The map **fills in** when a sign is missed, occluded or absent, which on
   many Kenyan roads is often.
8. **Ordinary traffic gets nothing.** Cars, people and animals that are not a hazard get no glow,
   no ring and no marker. The screen stays clean, and a glow always means something.

---

## 2. Road signs

Class ids are those of the sign detector (`host/scripts/mtsd_class_map.json`, 29 classes).

### Stop (0): the holographic barrier
- **Meaning:** stop completely at the stop line, then proceed only when clear.
- **AR:** a translucent red barrier stands across the ego lane on the road surface, at the stop line
  (or at the sign's position if no stop line is found). It has a distance countdown ("STOP 42 m").
- **Escalation:** from speed and distance, the deceleration needed to stop at the barrier is
  `a = v² / 2d`. The barrier pulses if `a` exceeds a comfortable ~0.25 g, and turns solid bright red
  above ~0.4 g ("you will not stop in time comfortably").
- **Clears:** once the car has been stationary (speed ≈ 0 for ≥ 1 s) within a few metres of it,
  the barrier dissolves.

### Give way (1)
- **Meaning:** yield to traffic on the main road. Stop only if necessary.
- **AR:** a dashed amber "give-way" line of chevrons across the lane at the junction mouth. It is
  softer than the stop barrier, because stopping is not mandatory. It turns red only if a detected
  vehicle is approaching on the crossing road.

### No entry (2)
- **Meaning:** this road must not be entered from here.
- **AR:** a solid red barrier wall across the entrance of the forbidden road, racing-game style.
- **Navigation:** if the active route goes through it, the route line breaks, turns red at the
  barrier, and a re-route is requested from `NavigationEngine`.

### No left turn (3), no right turn (4), no U-turn (5)
- **Meaning:** that manoeuvre is forbidden at the next junction.
- **AR:** barrier walls close off the mouth of the forbidden exit, like the barriers across closed
  side roads in racing games. The permitted exits stay open. Junction geometry comes from the map
  plus the road-surface model. Without it, a large crossed-arrow banner is shown instead (rule 6).
- **Navigation:** any route through the forbidden manoeuvre is re-planned. This is a direct
  navigation benefit, since Kenyan OSM turn restrictions are incomplete.

### No overtaking (6), and end of restriction (19)
- **Meaning:** do not cross into the opposing lane to overtake, until the restriction ends.
- **AR:** the lane boundary on the overtaking side (the **right** boundary, since Kenya is
  left-hand traffic) glows as a solid red line, and the opposing lane gets faint red hatching.
- **Persistence:** stays until an end-of-restriction sign or the map says it ends. It is not tied to
  the sign staying in view.

### Speed limits (7–18) and end of restriction (19): the F1-style speed grading
- **Meaning:** maximum speed from this point on.
- **HUD:** a sign-shaped badge ("50") next to the car's actual speed. The speed comes from OBD, with
  GPS speed as fallback.
- **Speed colour:**

  | Actual speed | Colour |
  |---|---|
  | ≤ limit | green |
  | ≤ limit + 10 % | amber |
  | beyond that | red |

  If no speed source is available, only the limit is shown and there is no grading. The system
  never guesses the speed.
- **Upcoming lower limit:** the route line is graded like a racing game's braking line. Ahead of
  the sign, each segment is coloured by the deceleration needed to be at the limit by the time the
  car reaches the sign: green (fine), amber (ease off now), red (brake now).
- **Value agreement (added 2026-09-26):** a new limit is shown only once several confirming
  frames agree on the **value**, not merely that "a speed sign is here". If the frames disagree
  (e.g. 30 vs 50), the badge keeps the previous limit, or the OSM `maxspeed` when there is none,
  and never shows a guess. Evidence: in the Kenyan audit the detector read a 30 sign as 50 at
  confidence 0.57, and a wrong limit is worse than a missing one
  (`docs/experiments/2026-09-26-sign-detector.md` §6).
- **Persistence:** the active limit persists after the sign leaves view. It is replaced by the next
  speed sign, cleared by end-of-restriction (19) back to the map's `maxspeed`, or reset when map
  matching says the car has turned onto another road.

### Pedestrian crossing (20)
- **Meaning:** pedestrians have priority at the crossing ahead.
- **AR:** the zebra area on the road is highlighted with a soft white glow.
- **Escalation:** if pedestrians are at or near it, the crossing glow turns amber, and those
  pedestrians become hazards (§3): glow, ground ring, and a barrier if they are crossing the path.
- **Speed:** an advisory "slow" grade on the route line approaching it. It is advisory, not a legal
  limit.

### Children / school (21)
- **Meaning:** a school zone. Children may cross unpredictably.
- **AR:** a yellow school-zone tint on the route line for the zone's length, and an advisory speed.
  It is labelled "advice", because Kenyan school-zone limits are not uniform.
- **Proposal, not decided:** raise pedestrian-hazard sensitivity inside the zone. That changes
  Part 9 thresholds, so it is a Phase 10 decision that needs a bench test, not a display choice.

### Road hump / speed bump (22)
- **Meaning:** a hump ahead. It is very common in Kenya, and often unmarked or badly marked.
- **AR:** an amber hump marker "painted" on the road at its position, with an advisory grade on the
  route line approaching it (as for speed limits, with a low advisory speed).
- **Tie-in:** Part 8.1's LiDAR road-anomaly check also finds humps. A hump seen by the LiDAR with
  no sign is shown too. That is the case where this system helps most, because the sign is
  missing.

### Roundabout (23)
- **Meaning:** a roundabout ahead. Give way to traffic already on it (from the right, in Kenya).
- **AR:** the route line curves clockwise round the roundabout, and a give-way chevron line appears
  at the entry. The exit to take is highlighted, which is a navigation aid.

### Traffic signals ahead (24)
- **AR:** an advisory traffic-light icon with its distance.
- **Future:** reading the light's state (red/amber/green) is future work. COCO's "traffic light"
  class finds the light, but not its colour.

### Keep left / keep right (25)
- **Meaning:** pass the obstruction (island, works) on the indicated side.
- **AR:** chevrons on the permitted side, and a low barrier on the forbidden side of the island.

### No parking / no stopping (26)
- **AR:** a small icon, shown only at low speed (< 20 km/h), when parking or stopping is actually
  likely. At speed it is clutter.

### Other warning (27), other regulatory (28)
- **AR:** a generic warning (triangle) or regulatory (circle) icon with its distance, and no
  road-locked graphic. The class does not say what the sign means precisely enough to draw anything
  more specific. Showing more would pretend to an understanding the model doesn't have.

---

## 3. Hazards

A **hazard** is a tracked object (vehicle, pedestrian, cyclist, animal or other obstacle) that the
risk logic flags. That covers forward-collision risk (Part 9.2/9.3), a Part 9.2 reckless-driving
flag (tailgating, erratic speed, swerving), or being in or entering the ego path. Everything else
is ordinary traffic and gets nothing (rule 8).

### 3.1 Risk level: one number that drives colour and motion
Each hazard carries a risk level `r` from 0 to 1, from the same measures the arbiter uses, plus
the motion prediction of Part 9.1:
- **Time to collision:** `r = 1` at or below `ttc_brake_threshold_s` (Part 12.5), falling linearly
  to 0 at twice that threshold.
- **Collision probability** (Part 9.1.2): the chance, from the predicted motion of object and ego,
  that they come within the safety radius inside the prediction horizon. Unlike TTC, this also
  catches crossing traffic and cut-ins.
- **Reckless-driving flag, or a pedestrian/animal in or entering the ego path:** at least
  `r = 0.4`, whatever the TTC says.
- The larger value wins. Thresholds live in configuration and are tuned in Part 12.5, never
  hard-coded.

`r` sets the colour, continuously from **amber (r ≈ 0.4)** to **red (r = 1)**, and the speed and
brightness of the shimmer.

### 3.2 The object glow (all hazards)
- **What:** a subtle, shimmering glow over the object itself, following its true outline. A soft
  band of light moves slowly across it. Its colour and pace follow `r`: a slow amber shimmer at
  caution, becoming a faster, brighter red at high risk.
- **How it stays subtle:** a rim of light along the silhouette edge, and at most a ~25% tint
  inside. The object stays fully readable (rule 4).
- **Outline:** from the detector's per-object segmentation mask (YOLOv8m-seg, see §4). Without a
  mask, the fallback is a soft ellipse inside the box (rule 6). Never a rectangle.
- **Motion:** the shimmer phase advances with a shader time uniform. It never flashes faster than
  ~3 Hz, to avoid a strobing effect.

### 3.3 Forward collision risk (something ahead in the ego path)
Something ahead is closing: a car braking hard, a stopped vehicle, a pedestrian, an obstacle.
- **Barrier:** a translucent barrier on the road at the object's ground position, like the stop
  barrier, with a distance countdown.
- **Route line:** grades amber → red as the stopping distance shrinks, using the stop sign's
  `a = v²/2d` rule. It pulses once a comfortable stop is no longer possible.
- **Object:** glow (§3.2).
- **With braking:** when the arbiter does brake (Part 9.3), this is what the driver sees, so the
  intervention is always explained.

### 3.4 Pedestrians and animals near or entering the path
- **Glow** (§3.2).
- **Ground ring:** a ring on the road at their feet, so it is clear who the warning is about.
- **Crossing barrier:** if their *predicted* path (Part 9.1) crosses the ego path, a barrier
  segment appears at the predicted conflict point, not where they stand now.

### 3.5 Tailgating (agreed 2026-09-25)
- **Zone:** the gap between the car and the vehicle ahead is painted on the road as a
  following-distance zone. It turns red and visibly shrinks when the time gap drops below
  `tailgating_min_gap_s`.
- **Object:** the lead vehicle gets the glow (§3.2).

### 3.6 Swerving or erratic neighbour
- **Lane line:** the boundary on that side glows red, and the neighbour's lane gets faint red
  hatching ("don't drift this way"), matching the no-overtaking line.
- **Object:** the vehicle gets the glow.

### 3.7 Obstacle or animal on the road (not moving across)
- **Barrier:** a barrier on the road at its position.
- **Marker:** a red ground marker, like the hump marker.
- **Object:** glow.

### 3.8 Predicted motion and latency compensation (Part 9.1)
- **Predicted path:** a hazard's predicted path is a fading ribbon on the road, as wide as its
  predicted uncertainty. It is a region, never a single confident line. Beyond ~1.5 s, prediction
  is intent rather than physics, and the ribbon widens and fades accordingly.
- **Occluded but predicted:** a coasting track (`PREDICTED_ONLY`, e.g. a pedestrian hidden behind a
  parked matatu) is drawn as a dashed "ghost" at its predicted position. It is visibly a
  prediction, not a sighting.
- **Sticking to moving objects:** every object-anchored element (glow, ring, barrier) is drawn at
  the object's position predicted for the moment the frame reaches the screen. The ~40–60 ms
  pipeline latency would otherwise make graphics trail a crossing car by most of a metre.
- **Warnings, not brakes:** predictions drive warnings and display only. Braking stays on measured,
  current range and closing speed (Part 9.3 rule 1).

### 3.9 What the driver never sees
Per-class boxes, class names and confidence scores. They belong to the development viewer
(`host/tools/inference_viewer`), not the driver display.

---

## 4. What each behaviour needs, and when it lands

| Capability | Needed for | Phase |
|---|---|---|
| Sign detection, 29 classes | all sign behaviours | 5 (sign detector, training) |
| **Object masks (YOLOv8m-seg)** | hazard glow outline; masking road graphics behind objects | 5 follow-up, after sign training |
| Tracking across frames (rule 2), persistence | signs and hazards | 7 (`MultiObjectTracker`, extended to signs) |
| Object and sign range, 3D position (LiDAR + camera) | barriers, zones, rings, countdowns | 6–7 |
| Risk level `r`: TTC, collision probability, reckless-driving flags | hazard colour, shimmer, collision barrier | 7 (tracker, `MotionPredictor`), 9.2 / 10 (arbiter) |
| Motion prediction (IMM, CPA, predicted distributions) | predicted-path ribbons, crossing barriers, ghosts, latency compensation | 7 (Part 9.1) |
| Map priors: `maxspeed`, turn restrictions, junction geometry | speed persistence, no-turn barriers, rerouting | 8 (OSRM / map matching) |
| Road-surface geometry | every road-placed element | 9 (`RoadSurfaceProjector`) |
| Vehicle speed (OBD, GPS fallback) | speed grading, stop and collision escalation | 2–3 (hub), fusion 7 |
| GPU rendering: glow, barriers, zones, graded line, priorities, fallbacks | display | 11 (`ArRenderer` + `WindowedSink`, Part 10) |

The Phase 12 bench gate and the Part 13 staged tests are unaffected, because no overlay touches
actuation (rule 1).
