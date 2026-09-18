// TODO(Part 13.1): test_road_surface_projector
//
// Synthetic route waypoints, a synthetic GroundPlaneModel including a non-flat (sloped) patch, and
// synthetic lane detections. Assert that:
//   - near-field points land on the supplied ground geometry, not a flat-ground assumption;
//   - far-field points fall back correctly and are marked onMeasuredSurface = false;
//   - the lateral-correction step never moves a point by more than lateral_correction_max_m.
//
// Filled in during Phase 9.
