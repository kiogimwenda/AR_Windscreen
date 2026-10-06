#pragma once
// GestureDriver — APDS-9960 on I2C (shared bus). See docs/BUILD_GUIDE.md Part 4.4. Not yet run on
// hardware.

#include <cstdint>

class GestureDriver {
public:
    enum class Event : uint8_t { NONE = 0, LEFT, RIGHT, UP, DOWN, HOLD };
    bool init();
    Event poll();

private:
    bool ok_ = false;
};
