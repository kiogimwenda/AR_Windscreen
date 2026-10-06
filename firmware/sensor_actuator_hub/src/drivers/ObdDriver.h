#pragma once
// ObdDriver — OBD-II vehicle speed and engine rpm. See docs/BUILD_GUIDE.md Part 4.4 and 4.8.5.
//
// Two implementations, exactly one built:
//   ObdDriverCan.cpp     (default) the vehicle's own CAN bus through the pod board's TJA1051T/3,
//                        ISO 15765-4. Every decision is in include/hub/ObdCan.h, tested natively.
//   ObdDriverElm327.cpp  (-DHUB_OBD_ELM327, env:hub_elm327) an ELM327 adapter on USART3, the
//                        fallback for a pre-CAN (K-line) vehicle.
// Both are non-blocking: read() does one small step per call and never waits for the car, because
// blocking would stall the 50 Hz SensorReport and Part 3.4's self-check would disarm the hub.
// brakePedalActive: no standard mode-01 PID carries it; it is 0xFF (unknown) here, and SensorTask
// fills it from the brake-light switch where fitted (BOM note C). Not yet run on hardware.

#include <cstdint>

class ObdDriver {
public:
    struct Reading {
        float speedKph = 0;
        uint16_t rpm = 0;
        uint8_t brakePedalActive = 0xFF;
        bool valid = false;
    };
    bool init();
    bool read(Reading& out);  // true when the reading is valid (fresh)
};
