#pragma once
// TODO(Part 11.4): RoadSurfaceProjector — the precision road-locked nav overlay. Senses nothing
// new: combines the route (11.2), the matched position (11.3), the LiDAR ground surface (8.1) and
// lane detections (Part 7). Near field projects onto measured ground; far field falls back to flat
// ground and is marked as such. A lane-detection lateral correction, capped by
// lateral_correction_max_m, is what actually delivers lane-level precision.
//
// Filled in during Phase 9.
