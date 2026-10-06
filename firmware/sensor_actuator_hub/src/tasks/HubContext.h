#pragma once
// HubContext — what the four FreeRTOS tasks share. See docs/BUILD_GUIDE.md Part 4.3.
//
//   coreMutex    guards SafetyCore: CommsTask reports frames and commands, ActuationTask ticks
//                it, SensorTask reports each SensorReport sent.
//   serialMutex  guards writes to the USB serial port (CommsTask's acks, SensorTask's reports).
// WatchdogTask takes NEITHER: it reads SafetyCore::mustRelease() and the kill-switch line directly
// and releases the hardware itself. A task stuck while holding coreMutex therefore cannot keep the
// brake on. The read is racy but benign on the single-core Cortex-M4: it reads a few aligned
// 32-bit and byte fields, and a stale answer is at most one 20 ms period old.

#include <STM32FreeRTOS.h>

#include "drivers/BrakeActuatorDriver.h"
#include "drivers/RelayDriver.h"
#include "hub/FrameCodec.h"
#include "hub/SafetyCore.h"

struct HubContext {
    hub::SafetyCore core;
    BrakeActuatorDriver brake;
    RelayDriver relays;
    SemaphoreHandle_t coreMutex = nullptr;
    SemaphoreHandle_t serialMutex = nullptr;
    // Bench build: Part 13.3 item 5, measured on the hub (see BenchTelemetry).
    volatile uint16_t lastLossReleaseMs = 0xFFFF;
    volatile uint16_t lastLossCurrentZeroMs = 0xFFFF;
};

HubContext& hubContext();

// Encodes and writes one frame to the host (thread-safe).
void sendFrame(hub_protocol::MessageType type, const void* payload, size_t len);
