#pragma once
// RelayDriver — indicators, hazards, horn, high beam through an opto-isolated relay board. See
// docs/BUILD_GUIDE.md Part 4.4. The board is ACTIVE-LOW (typical; confirm at bring-up, Part 4.2):
// a relay closes when its input is pulled LOW, so every output is driven HIGH at init() before it
// is switched to an output, and allOff() drives them all HIGH.

#include <cstdint>

class RelayDriver {
public:
    void init();
    void setIndicators(bool left, bool right, bool hazards);
    void setLights(bool highBeam, bool horn);
    void allOff();  // called by WatchdogTask and ActuationTask whenever not armed
};
