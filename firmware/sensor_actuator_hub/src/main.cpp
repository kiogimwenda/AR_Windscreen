// Sensor & Actuator Hub — firmware entry point. See docs/BUILD_GUIDE.md Part 4.3.
//
// ---------------------------------------------------------------------------------------------
// Why setup()/loop() and not main(): PlatformIO's Arduino framework supplies its own main(), which
// initialises the core and calls setup() once. The last thing setup() does is start the FreeRTOS
// scheduler, which never returns; from then on every line of work runs inside a task, and loop()
// stays empty by design.
//
// Task priorities (Part 4.3) are a safety requirement, not a tuning knob:
//     WatchdogTask (5) > ActuationTask (4) > { SensorTask (3), CommsTask (3) }
// The scheduler always runs the highest-priority READY task and preempts lower ones the instant a
// higher one becomes ready: the watchdog can always take the CPU from a comms task stuck in a read
// and release the actuator.
//
// Order of start-up matters: the actuator and relays are put in their SAFE state first (released,
// off), before any task can run and before the host has said anything. (On the two-box hub the
// power board's pull-downs have held them there since power-on; see Part 4.8.) SafetyCore starts
// disarmed: nothing actuates until the report loop is alive AND the host has sent a heartbeat.
//
// The hardware watchdog (IWDG, 500 ms) is the backstop above the 200 ms software one: if the
// scheduler itself dies, only a reset helps, and a reset leaves the H-bridge disabled.
// ---------------------------------------------------------------------------------------------

#include <Arduino.h>
#include <IWatchdog.h>
#include <STM32FreeRTOS.h>

#include "drivers/SafetyInputs.h"
#include "drivers/StatusLeds.h"
#include "tasks/ActuationTask.h"
#include "tasks/CommsTask.h"
#include "tasks/HubContext.h"
#include "tasks/SensorTask.h"
#include "tasks/WatchdogTask.h"

namespace {
HubContext gHub;
}

HubContext& hubContext() {
    return gHub;
}

void sendFrame(hub_protocol::MessageType type, const void* payload, size_t len) {
    uint8_t buf[hub_protocol::MAX_FRAME_SIZE];
    const size_t n = hub::encodeFrame(type, payload, len, buf);
    if (!n) return;
    xSemaphoreTake(gHub.serialMutex, portMAX_DELAY);
    Serial.write(buf, n);
    xSemaphoreGive(gHub.serialMutex);
}

void setup() {
    // Safe state before anything else.
    gHub.brake.init();   // released, both half-bridges disabled, magnet off
    gHub.relays.init();  // all off
    safety_inputs::init();
    status_leds::init();

    Serial.begin(115200);  // USB-CDC: the baud rate is nominal
    gHub.coreMutex = xSemaphoreCreateMutex();
    gHub.serialMutex = xSemaphoreCreateMutex();

    xTaskCreate(SensorTask, "sensor", 1024, nullptr, 3, nullptr);  // stack in words (4 KB)
    xTaskCreate(CommsTask, "comms", 1024, nullptr, 3, nullptr);
    xTaskCreate(ActuationTask, "actuate", 512, nullptr, 4, nullptr);
    xTaskCreate(WatchdogTask, "watchdog", 256, nullptr, 5, nullptr);

    IWatchdog.begin(hub_config::kIwdgTimeoutMs * 1000);  // microseconds
    vTaskStartScheduler();
}

void loop() {
    // Never reached: the scheduler owns the CPU once started (see above).
}
