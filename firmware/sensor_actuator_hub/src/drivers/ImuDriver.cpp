#include "drivers/ImuDriver.h"

#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <Wire.h>

#include <cmath>

#include "hub/Config.h"

namespace {
Adafruit_BNO08x bno;
sh2_SensorValue_t value;
constexpr float kG = 9.80665f;
constexpr float kRadToDeg = 57.29578f;
}  // namespace

bool ImuDriver::init() {
    Wire.setSDA(HUB_IMU_SDA);  // I2C1 on the pod board (Config.h)
    Wire.setSCL(HUB_IMU_SCL);
    Wire.begin();
    ok_ = bno.begin_I2C();
    if (!ok_) return false;
    const uint32_t us = 10000;  // 100 Hz reports; SensorTask samples at 50 Hz
    ok_ = bno.enableReport(SH2_ACCELEROMETER, us) &&
          bno.enableReport(SH2_GYROSCOPE_CALIBRATED, us) &&
          bno.enableReport(SH2_ROTATION_VECTOR, us);
    return ok_;
}

bool ImuDriver::read(Sample& out) {
    if (!ok_) return false;
    if (bno.wasReset()) init();
    while (bno.getSensorEvent(&value)) {
        switch (value.sensorId) {
            case SH2_ACCELEROMETER:
                latest_.ax = value.un.accelerometer.x / kG;
                latest_.ay = value.un.accelerometer.y / kG;
                latest_.az = value.un.accelerometer.z / kG;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                latest_.gx = value.un.gyroscope.x * kRadToDeg;
                latest_.gy = value.un.gyroscope.y * kRadToDeg;
                latest_.gz = value.un.gyroscope.z * kRadToDeg;
                break;
            case SH2_ROTATION_VECTOR: {
                const float qr = value.un.rotationVector.real, qi = value.un.rotationVector.i,
                            qj = value.un.rotationVector.j, qk = value.un.rotationVector.k;
                // Yaw from the quaternion (counter-clockwise from the sensor's x axis), turned into
                // a compass heading. The mounting yaw offset is calibrated in Part 12.
                const float yaw =
                    std::atan2(2.0f * (qr * qk + qi * qj), 1.0f - 2.0f * (qj * qj + qk * qk));
                float compass = 90.0f - yaw * kRadToDeg;
                while (compass < 0) compass += 360.0f;
                while (compass >= 360.0f) compass -= 360.0f;
                latest_.headingDeg = compass;
                break;
            }
            default:
                break;
        }
        latest_.timestampMs = millis();
        fresh_ = true;
    }
    if (!fresh_) return false;
    out = latest_;
    fresh_ = false;
    return true;
}
