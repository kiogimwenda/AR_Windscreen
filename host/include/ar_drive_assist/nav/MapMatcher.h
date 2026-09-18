#pragma once
// TODO(Part 11.3): MapMatcher — snaps the fused pose onto the correct edge of the road graph via
// OSRM's Match service, because raw GPS/EKF position is only good to a few metres. Throttled to
// 5-10 Hz: road identity changes far slower than vehicle dynamics, and matching on every raw fix
// makes the overlay jitter.
//
// Filled in during Phase 8.
