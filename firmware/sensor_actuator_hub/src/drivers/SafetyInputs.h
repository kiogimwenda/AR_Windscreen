#pragma once
// SafetyInputs — the hub's safety sense lines. See docs/BUILD_GUIDE.md Part 4.2 and
// docs/bill-of-materials.md notes 3.4 and C.
//
// Kill switch. The E-stop cuts actuator power in HARDWARE (through the relay, BOM 3.5); this line
// only tells the firmware. Wire the E-stop's second contact block NORMALLY CLOSED to ground, read
// with the pull-up: released = LOW; pressed = HIGH; and a broken or unplugged wire also reads HIGH
// = engaged. Fail-safe. (Part 4.2's table does not say which way round; the other way, a broken
// wire would read "not engaged". Recorded in docs/decisions.md.)
//
// Brake-light switch (BOM note C, suggested addition) via a PC817 optocoupler: LOW = pedal pressed.
//
// Power-box presence (Part 4.8): pulled low inside the power box only while the box is powered.
// LOW = present; unplugged, broken or unpowered reads HIGH = absent (fail-safe). All three lines
// come up the cable, and all three read their safe value when it is out.

#include <Arduino.h>

#include "hub/Config.h"

namespace safety_inputs {

inline void init() {
    pinMode(HUB_PIN_KILL_SENSE, INPUT_PULLUP);
    pinMode(HUB_PIN_BRAKE_LIGHT_SENSE, INPUT_PULLUP);
    pinMode(HUB_PIN_BOX_PRESENT, INPUT_PULLUP);
}
inline bool killSwitchEngaged() {
    return digitalRead(HUB_PIN_KILL_SENSE) == HIGH;
}
inline bool powerBoxPresent() {
    return digitalRead(HUB_PIN_BOX_PRESENT) == LOW;
}
inline bool brakePedalPressed() {
    return digitalRead(HUB_PIN_BRAKE_LIGHT_SENSE) == LOW;
}

}  // namespace safety_inputs
