#pragma once
// BrakeActuatorDriver — BTS7960 H-bridge (PWM + enable) and ACS712 current sense, both on the
// power board in the under-dash power box, reached through the inter-box cable (Part 4.8). See
// docs/BUILD_GUIDE.md Part 4.4, and docs/bill-of-materials.md note B for the actuator design this
// assumes (pull-only cable, holding electromagnet, fast low-force actuator; PENDING approval).
//
// ---------------------------------------------------------------------------------------------
// SAFETY: this driver is the THIRD force ceiling. The host arbiter clamps (advisory), SafetyCore
// clamps, and apply() clamps AGAIN to kMaxSafeIntensity here, at the last line of code before the
// PWM register, whatever arrived over the wire or from any caller.
//
// kMaxSafeIntensity starts at 90 and is finalised by the Part 13.3 bench test. It is NEVER raised
// without a full bench re-test. If the "driver can push through it" check (13.3 item 3) fails, the
// mechanical coupling gets redesigned: lowering this constant is not an acceptable substitute for
// a coupling that is unsafe on its own. The host's test_decision_arbiter.cpp reads THIS literal
// and requires decision_thresholds.yaml to match it.
//
// release() must be safe to call from any state, repeatedly, from any task, and leaves the
// actuator fully inert: both PWM lines low, both half-bridges disabled, electromagnet off (the
// cable drops free). The power board reaches the same state by itself (pull-downs) whenever this
// firmware is not driving the lines: in reset, unpowered, or with the cable unplugged.
//
// Not yet run on hardware (no board or actuator yet, 2026-09-27). Phase 12's bench gate is where
// it is proven.
// ---------------------------------------------------------------------------------------------

#include <cstdint>

#include "hub/Config.h"

class BrakeActuatorDriver {
public:
    static constexpr uint8_t kMaxSafeIntensity =
        90;  // tune on the bench (Part 13.3); NEVER raise without a re-test
    static_assert(kMaxSafeIntensity == hub_config::kMaxSafeIntensity,
                  "the two ceilings must agree");

    void init();
    // request: 0-255, clamped to kMaxSafeIntensity. Duration is enforced by SafetyCore, which
    // stops requesting when it ends; the driver has no clock of its own to trust.
    void apply(uint8_t request);
    void release();
    void setCableMagnet(bool on);
    float readCurrentAmps();
    uint8_t appliedDuty() const { return duty_; }
    bool cableMagnetOn() const { return magnet_; }

private:
    volatile uint8_t duty_ = 0;
    volatile bool magnet_ = false;
};
