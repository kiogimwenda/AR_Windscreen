#pragma once
// GpsDriver — u-blox receiver on USART2, NMEA parsed by TinyGPS++. See docs/BUILD_GUIDE.md
// Part 4.4. read() drains the UART without blocking. Not yet run on hardware.

#include <cstdint>

class GpsDriver {
public:
    struct Fix {
        double lat = 0, lon = 0;
        float speedKph = 0;
        bool valid = false;
        uint32_t timestampMs = 0;
    };
    bool init();
    bool read(Fix& out);  // false if no new fix since the last call
};
