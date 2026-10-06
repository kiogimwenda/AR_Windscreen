#include "tasks/ActuationTask.h"

#include <Arduino.h>

#include "drivers/SafetyInputs.h"
#include "drivers/StatusLeds.h"
#include "tasks/HubContext.h"

// Part 4.5, with every decision in SafetyCore::tick(): read the kill-switch line and the actuator
// current, ask the core what the hardware must do, and do exactly that. When nothing is to be
// applied (not armed, command ended, fault), the brake is RELEASED and the relays switched off on
// this very tick.
void ActuationTask(void*) {
    HubContext& h = hubContext();
    TickType_t wake = xTaskGetTickCount();
#ifdef HUB_BENCH_TELEMETRY
    bool wasLost = true, releaseSeen = true, zeroSeen = true;
    uint32_t lossFrameMs = 0;
#endif
    for (;;) {
        hub::HardwareInputs in;
        in.nowMs = millis();
        in.killSwitchEngaged = safety_inputs::killSwitchEngaged();
        in.powerBoxPresent = safety_inputs::powerBoxPresent();
        in.currentAmps = h.brake.readCurrentAmps();
        xSemaphoreTake(h.coreMutex, portMAX_DELAY);
        const hub::HardwareOutputs out = h.core.tick(in);
        const bool linkUp = !h.core.linkLost(in.nowMs);
        const bool armed = h.core.armed(in.nowMs);
        const bool fault = h.core.faultCode(in.nowMs) != hub::FAULT_NONE;
        xSemaphoreGive(h.coreMutex);
        status_leds::show(linkUp, armed, fault);
        if (out.brakeIntensity > 0) {
            h.brake.setCableMagnet(true);  // hold the cable before pulling on it
            h.brake.apply(out.brakeIntensity);
        } else {
            h.brake.release();
        }
        h.relays.setIndicators(out.indicators & 1, out.indicators & 2, out.indicators & 4);
        h.relays.setLights(out.lights & 1, out.lights & 2);
#ifdef HUB_BENCH_TELEMETRY
        // Part 13.3 item 5: time from the last host frame to release, and to zero current.
        xSemaphoreTake(h.coreMutex, portMAX_DELAY);
        const bool lost = h.core.linkLost(in.nowMs);
        const uint32_t lastFrame = h.core.lastFrameMs();
        xSemaphoreGive(h.coreMutex);
        if (lost && !wasLost) {
            lossFrameMs = lastFrame;
            releaseSeen = zeroSeen = false;
        }
        if (lost && !releaseSeen && h.brake.appliedDuty() == 0) {
            h.lastLossReleaseMs = static_cast<uint16_t>(in.nowMs - lossFrameMs);
            releaseSeen = true;
        }
        if (lost && !zeroSeen && in.currentAmps < 0.2f) {
            h.lastLossCurrentZeroMs = static_cast<uint16_t>(in.nowMs - lossFrameMs);
            zeroSeen = true;
        }
        wasLost = lost;
#endif
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(hub_config::kActuationPeriodMs));
    }
}
