// Sensor & Actuator Hub — firmware entry point. See docs/BUILD_GUIDE.md Part 4.3.
//
// PHASE 0 STUB. setup() and loop() exist only so the project links; the real task creation lands
// in Phase 2.
//
// ---------------------------------------------------------------------------------------------
// Why setup()/loop() and not main():
//
// PlatformIO's Arduino framework supplies its own main(), which initialises the core and then
// calls setup() once followed by loop() forever. Under FreeRTOS that outer loop is vestigial: the
// last thing setup() will do is call vTaskStartScheduler(), which never returns. From that point
// the scheduler owns the CPU and every line of real work runs inside a task, so loop() stays empty
// by design rather than by omission.
//
// Task priorities (Part 4.3) are a safety requirement, not a tuning knob:
//     WatchdogTask (5) > ActuationTask (4) > { SensorTask (3), CommsTask (3) }
// FreeRTOS here is preemptive and priority-based: the scheduler always runs the highest-priority
// READY task, and preempts a lower-priority one the instant a higher-priority task becomes ready.
// That is exactly the property the safety design leans on — the watchdog can always take the CPU
// away from a comms task stuck in a blocking read and release the actuator. Equal-priority tasks
// (SensorTask and CommsTask) time-slice against each other; neither can starve the two above them.
//
// The independent hardware watchdog (IWDG) is initialised here too, at a 500 ms timeout — a
// backstop ABOVE the 200 ms software watchdog in WatchdogTask. If the scheduler itself dies, no
// software watchdog can help, and only the IWDG resets the MCU (which drops the actuator, since
// reset leaves the H-bridge inert).
// ---------------------------------------------------------------------------------------------

#include <Arduino.h>

void setup() {
    // TODO(Part 4.3, Phase 2): initDrivers(), then xTaskCreate() for sensor/comms/actuate/watchdog,
    // then IWDG_Init(500) and vTaskStartScheduler().
}

void loop() {
    // Intentionally empty — see the note above. Once vTaskStartScheduler() is called in setup(),
    // control never reaches here.
}
