#include "tasks/ActuationTask.h"

// TODO(Part 4.5): Applies the latest ActuationCommand ONLY while systemArmed(); releases the
// actuator on every other path. Checks overcurrent before applying. See Part 4.5 — systemArmed() is
// false whenever the kill switch reads engaged, the watchdog has flagged link loss, or an
// overcurrent fault is latched (latched faults require a physical power cycle; never auto-clear one
// in software).
//
// Filled in during Phase 2.
