#pragma once
// ImuDriver — BNO085 over I2C (Adafruit_BNO08x). See docs/BUILD_GUIDE.md Part 4.4.
// Accelerometer (g), calibrated gyro (deg/s) and a heading (degrees clockwise from north, the
// compass convention of SensorReport; SensorFusion converts, Part 8.2) from the rotation vector.
// read() is non-blocking. Not yet run on hardware.

#include <cstdint>

class ImuDriver {
public:
    struct Sample {
        float ax = 0, ay = 0, az = 0;  // g
        float gx = 0, gy = 0, gz = 0;  // deg/s
        float headingDeg = 0;
        uint32_t timestampMs = 0;
    };
    bool init();
    bool read(Sample& out);  // false if nothing new since the last call
    bool ok() const { return ok_; }

private:
    bool ok_ = false;
    Sample latest_;
    bool fresh_ = false;
};
