#pragma once
// LoadCell — HX711 amplifier and a 50 kg load cell on the pedal fixture (BOM 5.2). BENCH BUILD
// ONLY: the applied force Part 13.3 step 2 logs. Bit-banged, 24-bit two's complement, channel A
// gain 128. Calibrate kNewtonsPerCount on the bench with a known weight. Not yet run on hardware.

#include <cstdint>

class LoadCell {
public:
    void init();
    bool read(float& newtons);  // false if no conversion is ready (non-blocking)
    void tare(int samples = 10);

private:
    long raw();
    long offset_ = 0;
    static constexpr float kNewtonsPerCount = 0.0005f;  // PLACEHOLDER until calibrated
};
