#pragma once
// SafetyCore — every actuation safety rule of the hub, in one class with no hardware in it. See
// docs/BUILD_GUIDE.md Part 3.4 (the watchdog contract), 4.5 (ActuationTask), 4.6 (WatchdogTask)
// and Appendix B (fault codes).
//
// ---------------------------------------------------------------------------------------------
// Why a separate core: the rules are what must be right, and they can be tested exhaustively on
// the laptop (`pio test -e native`) with a fake clock, long before there is a board. The FreeRTOS
// tasks are thin wrappers: CommsTask reports frames to the core, ActuationTask asks it what the
// hardware must do and does exactly that, WatchdogTask checks it independently.
//
// ARMED (actuation allowed) requires ALL of:
//   - a valid host frame within the last kLinkTimeoutMs (200 ms)                        [3.4 (1)]
//   - the hub's own SensorReport loop alive: a report sent within kReportStallMs        [3.4 (2)]
//   - a fresh HEARTBEAT received since the last time the system was disarmed            [3.4]
//   - the kill switch not engaged                                                       [4.5]
//   - the power box connected: the inter-box cable's presence loop reads present        [4.8]
//   - no latched overcurrent (latched until POWER CYCLE: never cleared in software)     [4.5]
//   - the frame error rate within limits (fault 4, recovering after clean frames)       [App. B]
// Whenever it is not armed: the brake is released, indicators and lights go off, and the last
// command is forgotten, so re-arming never resumes an old request.
//
// Commands (Part 3.3):
//   - A command that requests nothing (the arbiter's zeroed "no request") is always accepted: it
//     only ever releases.
//   - A command that requests something is accepted only while armed, and only if its
//     hostTimestampMs is newer than the last accepted one: a stale or replayed frame never acts.
//   - Brake intensity is clamped to kMaxSafeIntensity (the SECOND ceiling; the driver clamps a
//     THIRD time), duration to kMaxBrakeDurationMs. The brake releases when the duration ends.
// Fault code reported (one number, most serious first): 1 overcurrent, 5 power box disconnected,
// 2 kill switch, 4 frame errors, 3 link timeout, 0 none. (5 outranks 2 because an unplugged
// inter-box cable also opens the kill-sense line: 5 is the cause, 2 its symptom.)
//
// Two-box hub (Part 4.8, amended 2026-09-30). The MCU sits in the windscreen pod; the H-bridge,
// current sensor, magnet and relays sit in the under-dash power box, at the far end of a 2 m
// cable. Whenever the power box is not present:
//   - nothing is armed and everything is released (the power board's pull-downs also force its
//     outputs off in hardware, whatever the MCU does);
//   - the actuator current reading is NOT trusted, and cannot count toward the overcurrent latch.
//     An unplugged sense line reads about 0 V, which the ACS712 conversion turns into ~25 A; a
//     trusted reading would latch a false overcurrent that only a power cycle clears.
//
// Not thread-safe by itself: the firmware calls it under one FreeRTOS mutex. WatchdogTask also
// checks mustRelease() and releases the hardware DIRECTLY, so a hung ActuationTask cannot hold
// the brake on.
// ---------------------------------------------------------------------------------------------

#include <cstdint>

#include "hub/Config.h"
#include "hub/Protocol.h"

namespace hub {

enum Fault : uint8_t {
    FAULT_NONE = 0,
    FAULT_OVERCURRENT = 1,
    FAULT_KILL_SWITCH = 2,
    FAULT_LINK_TIMEOUT = 3,
    FAULT_FRAME_ERRORS = 4,
    FAULT_POWER_BOX = 5,
};

struct HardwareInputs {
    uint32_t nowMs = 0;
    bool killSwitchEngaged = true;  // unknown is treated as engaged
    bool powerBoxPresent = false;   // unknown is treated as absent
    float currentAmps = 0;          // meaningful only while the power box is present
};

struct HardwareOutputs {
    uint8_t brakeIntensity = 0;  // 0-255 scale, already clamped
    bool cableMagnet = false;    // holding electromagnet: on only while braking
    uint8_t indicators = 0;      // bit0 left, bit1 right, bit2 hazards
    uint8_t lights = 0;          // bit0 high beam, bit1 horn
};

class SafetyCore {
public:
    // --- Comms side ---
    void onValidFrame(uint32_t nowMs, hub_protocol::MessageType type) {
        if (!everHadFrame_ || linkLost(nowMs)) needHeartbeat_ = true;  // coming back from silence
        everHadFrame_ = true;
        lastFrameMs_ = nowMs;
        recordFrame(true);
        if (type == hub_protocol::MessageType::HEARTBEAT) needHeartbeat_ = false;
    }
    void onFrameError() { recordFrame(false); }
    void onReportSent(uint32_t nowMs) {
        reportStarted_ = true;
        lastReportMs_ = nowMs;
    }

    hub_protocol::AckStatus onCommand(uint32_t nowMs, const hub_protocol::ActuationCommand& c) {
        bool accepted = false;
        const bool requestsNothing = c.brakeRequest == 0 && c.indicators == 0 && c.lights == 0;
        if (requestsNothing) {
            clearCommand();
            accepted = true;
            if (!haveHostTs_ || c.hostTimestampMs > lastHostTs_) {
                lastHostTs_ = c.hostTimestampMs;
                haveHostTs_ = true;
            }
        } else if (armed(nowMs) && (!haveHostTs_ || c.hostTimestampMs > lastHostTs_)) {
            lastHostTs_ = c.hostTimestampMs;
            haveHostTs_ = true;
            indicators_ = c.indicators & 0x07;
            lights_ = c.lights & 0x03;
            const uint8_t intensity = c.brakeRequest < hub_config::kMaxSafeIntensity
                                          ? c.brakeRequest
                                          : hub_config::kMaxSafeIntensity;
            const uint16_t dur = c.brakeDurationMs < hub_config::kMaxBrakeDurationMs
                                     ? c.brakeDurationMs
                                     : hub_config::kMaxBrakeDurationMs;
            if (intensity > 0 && dur > 0) {
                brakeIntensity_ = intensity;
                brakeUntilMs_ = nowMs + dur;
            } else {
                brakeIntensity_ = 0;
            }
            accepted = true;
        }
        hub_protocol::AckStatus ack{};
        ack.timestampMs = nowMs;
        ack.lastCommandAccepted = accepted ? 1 : 0;
        ack.actuatorFaultCode = faultCode(nowMs);
        ack.appliedBrakeIntensity = applied_;
        return ack;
    }

    // --- Actuation side: once per ActuationTask tick ---
    HardwareOutputs tick(const HardwareInputs& in) {
        kill_ = in.killSwitchEngaged;
        boxPresent_ = in.powerBoxPresent;
        if (boxPresent_ && in.currentAmps > hub_config::kOvercurrentAmps) {
            if (++overTicks_ >= hub_config::kOvercurrentTicks) overcurrentLatched_ = true;
        } else {
            overTicks_ = 0;
        }
        const bool isArmed = armed(in.nowMs);
        if (!isArmed) {
            // Any disarm needs a fresh heartbeat before re-arming, and forgets the last command.
            needHeartbeat_ = needHeartbeat_ || kill_ || !boxPresent_ || frameFault_ ||
                             linkLost(in.nowMs) || reportStalled(in.nowMs);
            clearCommand();
        }
        if (brakeIntensity_ > 0 && static_cast<int32_t>(in.nowMs - brakeUntilMs_) >= 0)
            brakeIntensity_ = 0;
        HardwareOutputs out;
        out.brakeIntensity = brakeIntensity_;
        out.cableMagnet = brakeIntensity_ > 0;
        out.indicators = indicators_;
        out.lights = lights_;
        applied_ = brakeIntensity_;
        return out;
    }

    // --- Independent checks (WatchdogTask) ---
    bool linkLost(uint32_t nowMs) const {
        return !everHadFrame_ || nowMs - lastFrameMs_ > hub_config::kLinkTimeoutMs;
    }
    bool reportStalled(uint32_t nowMs) const {
        return !reportStarted_ || nowMs - lastReportMs_ > hub_config::kReportStallMs;
    }
    bool mustRelease(uint32_t nowMs, bool killSwitchEngaged, bool powerBoxPresent) const {
        return linkLost(nowMs) || reportStalled(nowMs) || killSwitchEngaged || !powerBoxPresent ||
               overcurrentLatched_;
    }

    bool armed(uint32_t nowMs) const {
        return !linkLost(nowMs) && !reportStalled(nowMs) && !needHeartbeat_ && !kill_ &&
               boxPresent_ && !overcurrentLatched_ && !frameFault_;
    }
    uint8_t faultCode(uint32_t nowMs) const {
        if (overcurrentLatched_) return FAULT_OVERCURRENT;
        if (!boxPresent_) return FAULT_POWER_BOX;
        if (kill_) return FAULT_KILL_SWITCH;
        if (frameFault_) return FAULT_FRAME_ERRORS;
        if (everHadFrame_ && linkLost(nowMs)) return FAULT_LINK_TIMEOUT;
        return FAULT_NONE;
    }
    bool overcurrentLatched() const { return overcurrentLatched_; }
    uint32_t lastFrameMs() const { return lastFrameMs_; }
    uint8_t appliedIntensity() const { return applied_; }

private:
    void clearCommand() {
        brakeIntensity_ = 0;
        indicators_ = 0;
        lights_ = 0;
    }
    // Frame error rate over a sliding window (Appendix B fault 4).
    void recordFrame(bool ok) {
        if (windowFilled_ == hub_config::kFrameWindow && !window_[windowPos_])
            --errors_;  // slides out
        window_[windowPos_] = ok;
        if (!ok) ++errors_;
        windowPos_ = (windowPos_ + 1) % hub_config::kFrameWindow;
        if (windowFilled_ < hub_config::kFrameWindow) ++windowFilled_;
        cleanStreak_ = ok ? cleanStreak_ + 1 : 0;
        if (errors_ > hub_config::kFrameErrorsMax) frameFault_ = true;
        if (frameFault_ && cleanStreak_ >= hub_config::kCleanFramesToRecover &&
            errors_ <= hub_config::kFrameErrorsMax)
            frameFault_ = false;
    }

    bool everHadFrame_ = false, needHeartbeat_ = true, reportStarted_ = false;
    uint32_t lastFrameMs_ = 0, lastReportMs_ = 0;
    bool kill_ = true;
    bool boxPresent_ = false;
    bool overcurrentLatched_ = false;
    int overTicks_ = 0;
    bool window_[hub_config::kFrameWindow] = {};
    int windowPos_ = 0, windowFilled_ = 0, errors_ = 0, cleanStreak_ = 0;
    bool frameFault_ = false;
    bool haveHostTs_ = false;
    uint32_t lastHostTs_ = 0;
    uint8_t brakeIntensity_ = 0, indicators_ = 0, lights_ = 0, applied_ = 0;
    uint32_t brakeUntilMs_ = 0;
};

}  // namespace hub
