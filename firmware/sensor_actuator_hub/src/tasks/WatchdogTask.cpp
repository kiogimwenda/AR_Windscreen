#include "tasks/WatchdogTask.h"

// TODO(Part 4.6): Highest-priority task. Releases the actuator and turns all relays off if no valid
// host frame has arrived for >200 ms or the kill-switch line reads engaged. Refreshes the IWDG.
// This logic must not depend on any host-side code path (Part 3.4).
//
// Filled in during Phase 2.
