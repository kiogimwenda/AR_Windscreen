#include "drivers/LoadCell.h"

#include <Arduino.h>

#include "hub/Config.h"

void LoadCell::init() {
    pinMode(HUB_PIN_HX711_SCK, OUTPUT);
    pinMode(HUB_PIN_HX711_DOUT, INPUT);
    digitalWrite(HUB_PIN_HX711_SCK, LOW);
}

long LoadCell::raw() {
    long v = 0;
    for (int i = 0; i < 24; ++i) {
        digitalWrite(HUB_PIN_HX711_SCK, HIGH);
        delayMicroseconds(1);
        v = (v << 1) | digitalRead(HUB_PIN_HX711_DOUT);
        digitalWrite(HUB_PIN_HX711_SCK, LOW);
        delayMicroseconds(1);
    }
    digitalWrite(HUB_PIN_HX711_SCK, HIGH);  // 25th pulse: channel A, gain 128 next time
    delayMicroseconds(1);
    digitalWrite(HUB_PIN_HX711_SCK, LOW);
    if (v & 0x800000) v |= ~0xFFFFFFL;  // sign-extend 24 bits
    return v;
}

bool LoadCell::read(float& newtons) {
    if (digitalRead(HUB_PIN_HX711_DOUT) == HIGH) return false;  // conversion not ready
    newtons = (raw() - offset_) * kNewtonsPerCount;
    return true;
}

void LoadCell::tare(int samples) {
    long sum = 0;
    int n = 0;
    const uint32_t start = millis();
    while (n < samples && millis() - start < 2000) {
        if (digitalRead(HUB_PIN_HX711_DOUT) == LOW) {
            sum += raw();
            ++n;
        }
    }
    if (n) offset_ = sum / n;
}
