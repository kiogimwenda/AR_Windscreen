#include "drivers/GestureDriver.h"

#include <Adafruit_APDS9960.h>
#include <Wire.h>

#include "hub/Config.h"

// The gesture puck hangs on its own ~1.5 m cable near the steering column (Part 4.8), on I2C2, so
// a fault on that cable cannot hang the IMU's bus. 100 kHz: the cable's capacitance (~100 pF/m)
// stays well inside I2C's 400 pF budget at this speed.
namespace {
Adafruit_APDS9960 apds;
TwoWire gestureWire(HUB_GESTURE_SDA, HUB_GESTURE_SCL);
}  // namespace

bool GestureDriver::init() {
    gestureWire.begin();
    gestureWire.setClock(100000);
    ok_ = apds.begin(10, APDS9960_AGAIN_4X, APDS9960_ADDRESS, &gestureWire);
    if (ok_) {
        apds.enableProximity(true);
        apds.enableGesture(true);
    }
    return ok_;
}

GestureDriver::Event GestureDriver::poll() {
    if (!ok_) return Event::NONE;
    switch (apds.readGesture()) {
        case APDS9960_LEFT:
            return Event::LEFT;
        case APDS9960_RIGHT:
            return Event::RIGHT;
        case APDS9960_UP:
            return Event::UP;
        case APDS9960_DOWN:
            return Event::DOWN;
        default:
            return Event::NONE;
    }
}
