#pragma once
// TODO(Part 4.4): BrakeActuatorDriver — H-bridge PWM + direction GPIO + ADC current sense.
//
// ---------------------------------------------------------------------------------------------
// SAFETY: this file implements the SECOND, INDEPENDENT force ceiling.
//
// The host's DecisionArbiter (Part 9.3) produces only an advisory *request*. This driver clamps
// that request to kMaxSafeIntensity regardless of what arrived over the wire — a corrupted,
// replayed, or simply wrong host command can never exceed the hub's own limit.
//
// kMaxSafeIntensity starts at 90 and is finalized by the Part 13.3 bench test. It is NEVER raised
// without a full bench re-test. If the "driver can push through it" check (13.3 item 3) fails, the
// mechanical coupling gets redesigned — lowering this constant is not an acceptable substitute for
// a coupling that is unsafe on its own.
//
// release() must be safe to call from any state and must leave the actuator fully inert.
// ---------------------------------------------------------------------------------------------
//
// Real hardware is deliberately NOT wired in Phase 2 — this is stubbed to report zero current with
// no motor connected. The real implementation lands in Phase 12, behind that phase's mandatory
// bench gate.
