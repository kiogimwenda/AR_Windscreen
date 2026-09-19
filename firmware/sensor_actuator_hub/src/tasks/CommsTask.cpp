#include "tasks/CommsTask.h"

#include "hub/Crc16.h"
#include "hub/Protocol.h"

// TODO(Part 4.3): Owns the USB-CDC link to the host: decodes incoming frames (discarding any with
// a bad CRC without acting on them), tracks time-since-last-valid-host-frame for the watchdog, and
// transmits SensorReport / AckStatus frames.
//
// Filled in during Phase 2.
//
// The two includes above are NOT premature. Protocol.h's static_asserts on struct layout only
// execute in a translation unit that actually compiles the header — and the layout question they
// exist to answer is specifically about THIS target, the Cortex-M4, not the x86-64 host where the
// unit tests run. Including them here means a plain `pio run` cross-compiles those asserts for ARM,
// so a host/hub layout disagreement becomes a firmware build failure. Without this, Phase 1's
// "identical on both ends" claim would rest on two tests that both ran on x86-64.
