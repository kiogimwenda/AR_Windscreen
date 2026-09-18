// TODO(Part 13.1): test_decision_arbiter
//
// Synthetic SceneModel/VehiclePose inputs covering every rule in Part 9.3, including the "no
// request" default case.
//
// MUST include the ceiling test: no combination of inputs produces a BRAKE request above
// brake_actuator_max_intensity. That value lives in config/decision_thresholds.yaml, not in a
// compiled constant, so a bad YAML value has to be caught here rather than on a bench.
//
// Filled in during Phase 10.
