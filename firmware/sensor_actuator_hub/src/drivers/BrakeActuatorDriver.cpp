#include "drivers/BrakeActuatorDriver.h"

#include <Arduino.h>

#include <cmath>

void BrakeActuatorDriver::init() {
    // Latch LOW before each pin becomes an output. The power board's pull-downs already hold these
    // lines low while the MCU is in reset; this keeps them there through the switch to output.
    for (uint32_t p : {HUB_PIN_BRAKE_LPWM, HUB_PIN_BRAKE_EN, HUB_PIN_CABLE_MAGNET}) {
        digitalWrite(p, LOW);
        pinMode(p, OUTPUT);
    }
    analogWriteResolution(8);
    analogWriteFrequency(hub_config::kBrakePwmHz);
    analogReadResolution(12);
    release();
}

void BrakeActuatorDriver::apply(uint8_t request) {
    const uint8_t d = request < kMaxSafeIntensity ? request : kMaxSafeIntensity;  // third ceiling
    if (d == 0) {
        release();
        return;
    }
    digitalWrite(HUB_PIN_BRAKE_LPWM, LOW);  // never drive both halves
    digitalWrite(HUB_PIN_BRAKE_EN, HIGH);   // R_EN and L_EN, tied on the power board
    analogWrite(HUB_PIN_BRAKE_RPWM, d);
    duty_ = d;
}

void BrakeActuatorDriver::release() {
    analogWrite(HUB_PIN_BRAKE_RPWM, 0);
    digitalWrite(HUB_PIN_BRAKE_LPWM, LOW);
    digitalWrite(HUB_PIN_BRAKE_EN, LOW);
    setCableMagnet(false);
    duty_ = 0;
}

void BrakeActuatorDriver::setCableMagnet(bool on) {
    digitalWrite(HUB_PIN_CABLE_MAGNET, on ? HIGH : LOW);
    magnet_ = on;
}

float BrakeActuatorDriver::readCurrentAmps() {
    // ACS712: 2.5 V at 0 A, 100 mV/A, divided x2/3 on the power board. Magnitude only: the pull
    // direction is the only one ever driven. Meaningless while the power box is unplugged (the
    // line floats low and reads ~25 A): SafetyCore ignores it then.
    const float adcV =
        analogRead(HUB_PIN_BRAKE_CURRENT) * hub_config::kAdcVref / hub_config::kAdcMax;
    const float sensorV = adcV / hub_config::kCurrentDivider;
    return std::fabs(sensorV - hub_config::kAcs712ZeroV) / hub_config::kAcs712VoltsPerAmp;
}
