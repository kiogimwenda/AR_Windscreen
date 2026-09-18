#pragma once
// TODO(Part 9.3): DecisionArbiter — the ONLY subsystem permitted to produce an ActuationRequest.
// Four rules, first match wins, one request per cycle. A BRAKE request is never a pass-through of a
// raw ML classifier output: only rule 1's time-to-collision check may request braking, so the
// behaviour is defensible from a single auditable rule.
//
// Filled in during Phase 10.
