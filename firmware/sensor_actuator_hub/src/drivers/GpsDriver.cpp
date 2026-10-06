#include "drivers/GpsDriver.h"

#include <Arduino.h>
#include <TinyGPSPlus.h>

#include "hub/Config.h"

namespace {
TinyGPSPlus gps;
Uart gpsSerial(HUB_GPS_RX,
               HUB_GPS_TX);  // USART2; declared here: this board's core does not create Serial2
constexpr uint32_t kGpsBaud = 9600;  // u-blox NEO-M8N default
}  // namespace

bool GpsDriver::init() {
    gpsSerial.begin(kGpsBaud);
    return true;
}

bool GpsDriver::read(Fix& out) {
    while (gpsSerial.available() > 0) gps.encode(static_cast<char>(gpsSerial.read()));
    if (!gps.location.isUpdated()) return false;
    out.lat = gps.location.lat();
    out.lon = gps.location.lng();
    out.speedKph = gps.speed.isValid() ? static_cast<float>(gps.speed.kmph()) : 0.0f;
    out.valid = gps.location.isValid() && gps.location.age() < 2000;
    out.timestampMs = millis();
    return true;
}
