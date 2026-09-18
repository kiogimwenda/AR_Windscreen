#pragma once
// TODO(Part 8.1): LidarProcessor — Livox SDK2 capture loop: voxel downsample -> ground segmentation
// -> DBSCAN clustering -> road-anomaly pass. Pushes SceneCloud to lidarBus AND GroundPlaneModel to
// groundPlaneBus. Dropping that second publish silently degrades the nav overlay with no obvious
// error — see Part 8.1's change note.
//
// Filled in during Phase 6.
