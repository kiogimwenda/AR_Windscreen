#pragma once
// TODO(Part 11.5): SystemManager — owns config load, the EventLog, and the lifecycle of every
// subsystem thread. On ANY shutdown path, normal or signal-triggered, it must request one final
// zeroed ActuationCommand before the serial port closes — belt-and-braces on top of the hub
// watchdog, never a substitute for it.
//
// Filled in during Phase 3.
