#pragma once
// TODO(Part 11.1): VehicleInterface — the only class that opens the serial port to the hub.
// Implements Part 3's framing/CRC and nothing else: it applies no thresholds and does not interpret
// reason codes. Sends HEARTBEAT at >=10 Hz unconditionally, and a zeroed ActuationCommand when the
// arbiter has nothing to request, so 'no request' and 'link dead' stay distinguishable on the hub.
//
// Filled in during Phase 3.
