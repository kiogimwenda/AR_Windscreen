#pragma once
// StatusLeds — three LEDs on the pod's driver-facing edge: host link, armed, fault. See
// docs/BUILD_GUIDE.md Part 4.8.2. Driven by ActuationTask every 10 ms from SafetyCore's state.
// Informational only: nothing reads them back, and no safety decision depends on them.
// (PC13, the link LED, can sink only ~3 mA: 1 k series resistor.)

#include <Arduino.h>

#include "hub/Config.h"

namespace status_leds {

inline void init() {
    for (uint32_t p : {HUB_PIN_LED_LINK, HUB_PIN_LED_ARMED, HUB_PIN_LED_FAULT}) {
        digitalWrite(p, LOW);
        pinMode(p, OUTPUT);
    }
}
inline void show(bool linkUp, bool armed, bool fault) {
    digitalWrite(HUB_PIN_LED_LINK, linkUp ? HIGH : LOW);
    digitalWrite(HUB_PIN_LED_ARMED, armed ? HIGH : LOW);
    digitalWrite(HUB_PIN_LED_FAULT, fault ? HIGH : LOW);
}

}  // namespace status_leds
