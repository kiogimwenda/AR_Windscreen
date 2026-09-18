#pragma once
// TODO(Part 7.4): MlInferenceEngine — owns all three TrtEngines (YOLOv8m, UFLDv2, MiDaS). Loop: pop
// frameBus -> run 3 models on independent CUDA streams -> push DetectionFrame to detectionBus.
//
// Filled in during Phase 5.
