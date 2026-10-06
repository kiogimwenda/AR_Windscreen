#include "drivers/RelayDriver.h"

#include <Arduino.h>

#include "hub/Config.h"

namespace {
constexpr uint32_t kPins[] = {HUB_PIN_RELAY_LEFT, HUB_PIN_RELAY_RIGHT, HUB_PIN_RELAY_HAZARD,
                              HUB_PIN_RELAY_HORN, HUB_PIN_RELAY_BEAM};
// ACTIVE-HIGH (Part 4.8): each line drives a low-side MOSFET on the power board, whose gate is
// pulled down, so a relay is off unless this firmware actively holds its line high.
void set(uint32_t pin, bool on) {
    digitalWrite(pin, on ? HIGH : LOW);
}
}  // namespace

void RelayDriver::init() {
    for (uint32_t p : kPins) {
        digitalWrite(p, LOW);  // latch "off" before the pin becomes an output: no glitch at boot
        pinMode(p, OUTPUT);
    }
}

void RelayDriver::setIndicators(bool left, bool right, bool hazards) {
    set(HUB_PIN_RELAY_LEFT, left);
    set(HUB_PIN_RELAY_RIGHT, right);
    set(HUB_PIN_RELAY_HAZARD, hazards);
}

void RelayDriver::setLights(bool highBeam, bool horn) {
    set(HUB_PIN_RELAY_BEAM, highBeam);
    set(HUB_PIN_RELAY_HORN, horn);
}

void RelayDriver::allOff() {
    for (uint32_t p : kPins) set(p, false);
}
