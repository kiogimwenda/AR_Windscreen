# 6. Precision Navigation Overlay Design

## 6.1 The requirement

The route must appear **on the road**: on its surface through slopes, crests and dips, and in the
correct lane, not as an arrow floating over the video. GNSS alone is good to a few metres and a
compass to about 3°, which is 3 m sideways at 60 m ahead: neither can choose a lane. The design
adds no new sensor. It combines four things the system already produces, in one sequential
pipeline on the navigation thread, run at the LiDAR's ground-plane rate (there is no benefit in
projecting faster than the geometry underneath changes).

[FIGURE: the pipeline, routing → map-matching → ground-plane projection → lane correction; the
proposal's `fig_navigation.svg` is the starting point]

## 6.2 Routing: which road

OSRM {{cite:luxen2011}} with the multi-level Dijkstra algorithm {{cite:delling2011}} routes on an
offline OpenStreetMap extract covering 100 km around Nairobi (Thika, Machakos, Kajiado, Murang'a,
Naivasha). A route is the full geometry along the road network, not only turn instructions. A
moving car's route must leave within ±45° of its heading, so it never begins with a U-turn. Road
status comes in three layers, and the map never overrules the sensors: the base map, rebuilt
nightly from OpenStreetMap and switched in only if a test route passes; a live layer of traffic
and incidents that re-weights the road graph (designed for TomTom's feed, deferred under its terms
of use [CHECK: state as decided]); and the car's own sensors, which see what is physically ahead.

## 6.3 Map-matching: how far along it

The fused position is snapped onto the road graph by OSRM's hidden-Markov map-matching
{{cite:newson2009}}, given a short trace (up to eight points at least 1 s apart, the newest the
current pose) rather than a single fix. On a road 15 m from a parallel one with 5 m of position
noise, a single fix was nearer the wrong road 7.2 % of the time; the trace kept the car on the right
road 99.6 % of the time. Progress along the route never decreases; leaving the route or driving
the wrong way needs three consecutive matches before a reroute, so one glitch never reroutes;
between matches, progress is dead-reckoned from the fused speed.

## 6.4 Projection onto the measured road

From the matched position, the route ahead is sampled every 1.5 m out to 60 m, and each point's
height is taken in one of three ways:
1. **Measured:** within the LiDAR's range, the mean height of the ground points within 1 m of it,
   from the ground surface the LiDAR fits on every scan. This is what keeps the line on the road
   through slopes and dips.
2. **The fitted plane:** where the measured surface has a gap (a car ahead hides the road), the
   plane fitted to the scan. Marked as not measured.
3. **Flat ground:** beyond the LiDAR's range, a flat road at the car's own height. Marked as not
   measured.

Each point is then projected into the image with the same camera model the perception side uses,
so the route and the detections cannot disagree about where the road is.

## 6.5 Lane correction

The lane-detection network {{cite:qin2024}} gives the lane's boundaries. Their centre line, fitted
5–30 m ahead, corrects two things:
- **heading:** a pose error, so it rotates the whole line;
- **lateral offset:** it moves the line from OpenStreetMap's road centre into the car's lane. It
  applies along the current road and tapers out over 10 m after the next turn, since the lane on
  the new road is not yet known.

Both are persistent states, and each frame may change them only a little (at most 1.0 m and a
capped angle per frame), so one bad detection cannot fling the line across lanes, yet a normal
1.75 m lane offset is still reached. A frame is used only if both boundaries are found and the lane
is 2.3–5 m wide. Without lanes, the corrections decay over about 10 s, and they reset on the next
road.

## 6.6 Drawing it

The route reaches the renderer as 3D points in the vehicle frame with the pose time they belong
to. The renderer moves them by the car's own motion between that pose and the displayed frame
(latency compensation) and draws a band in road space on the GPU; hazards mask it, so it passes
behind a pedestrian rather than across them.

## 6.7 Precision, stated honestly

Precision is not uniform with distance, and the overlay says so: near the car, within the LiDAR's
range, the line lies on the measured surface; beyond it, the line rests on a flat-ground
assumption and may float above a falling road or sink into a rising one. Lateral precision is
the lane detector's where lanes are visible, and the map-matcher's (metres) where they are not.

[RESULT: Part 12.4's road-projection check: lateral and vertical error of the line against the
surveyed lane centre at 10, 20, 40 and 60 m, near field and far field, with a frame showing the
line following a slope or dip that a flat-ground projection would miss]
[RESULT: map-matching on the recorded Nairobi drive: as of 2026-09-27, 2,773 of 2,773 fixes
matched, median 3.1 m, progress never decreasing (`docs/report/evidence/README.md`)]
