#include "tasks/WatchdogTask.h"

#include <Arduino.h>
#include <IWatchdog.h>

#include "drivers/SafetyInputs.h"
#include "tasks/HubContext.h"

// Part 4.6. Every 20 ms: if the host link is lost (>200 ms), the report loop has stalled, the kill
// switch is engaged, the power box is unplugged or an overcurrent is latched, release the brake and
// switch every relay off, DIRECTLY, without the core's mutex and without waiting for ActuationTask.
// Then refresh the hardware watchdog (IWDG, 500 ms): if this task itself stops running, the MCU
// resets, and a reset leaves the H-bridge disabled and the cable magnet off.
void WatchdogTask(void*) {
    HubContext& h = hubContext();
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        if (h.core.mustRelease(millis(), safety_inputs::killSwitchEngaged(),
                               safety_inputs::powerBoxPresent())) {
            h.brake.release();
            h.relays.allOff();
        }
        IWatchdog.reload();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(hub_config::kWatchdogPeriodMs));
    }
}
