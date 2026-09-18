#include "tasks/CommsTask.h"

// TODO(Part 4.3): Owns the USB-CDC link to the host: decodes incoming frames (discarding any with a
// bad CRC without acting on them), tracks time-since-last-valid-host-frame for the watchdog, and
// transmits SensorReport / AckStatus frames.
//
// Filled in during Phase 2.
