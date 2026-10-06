#pragma once
// ObdCan — OBD-II over the vehicle's CAN bus (ISO 15765-4 / SAE J1979 mode 01), with no hardware
// in it. See docs/BUILD_GUIDE.md Part 4.8.5. The HAL wrapper (src/drivers/ObdDriver.cpp) does
// only what `ObdPoller::step()` tells it; every decision is here and tested natively
// (`pio test -e native`).
//
// ---------------------------------------------------------------------------------------------
// Why this replaces the ELM327 + HC-05: the ELM327 is itself a small microcontroller translating
// AT commands into these same CAN frames, over a Bluetooth link that has to pair, drops out, and
// adds tens of milliseconds. Cheap clones mishandle commands. Talking CAN directly removes two
// devices and a radio link. Vehicles now entering Kenya are covered: used imports must have been
// first registered in 2019 or later (KEBS rule from January 2026), and CAN became the OBD-II norm
// around 2008 (mandatory in the US from model year 2008; other markets followed). An older test
// car may still use K-line: that falls back to an ELM327 (-DHUB_OBD_ELM327, Part 4.8.5).
//
// The request (functional, to all emissions ECUs):
//     11-bit: ID 0x7DF      29-bit: ID 0x18DB33F1
//     data:   02 01 <PID> 00 00 00 00 00      (single frame, 2 bytes: service 01, the PID; DLC 8)
// A response (physical, from one ECU):
//     11-bit: ID 0x7E8..0x7EF      29-bit: ID 0x18DAF1xx
//     data:   <len> 41 <PID> A [B ...]        (single frame; len = bytes after the first)
//     speed PID 0x0D: A km/h;  rpm PID 0x0C: (256 A + B) / 4
//
// Rule one: THE HUB MUST NEVER DISTURB THE CAR'S OWN BUS. The car's CAN carries its engine,
// brakes and airbags. So:
//   1. At each candidate bit rate the controller first LISTENS in silent mode (it cannot transmit
//      or even acknowledge) for `kListenMs`. Bus errors while listening mean the wrong bit rate:
//      move on without ever sending. Only a clean listen (or a quiet bus, common behind a gateway)
//      allows a request.
//   2. A request is sent ONCE (automatic retransmission off). If the rate is wrong after all, that
//      one frame fails; it is not repeated against the car's bus.
//   3. Two PIDs, one request at a time, each only after the previous answer or its timeout: at
//      most ~20 requests a second, the load of any handheld scan tool.
//   4. Only service 01 (read current data). Nothing that writes, clears codes or resets an ECU is
//      even representable here.
// The transceiver itself is also held in standby in hardware until the firmware enables it, and
// sits beside the OBD port so the hub adds only a short stub to the car's bus (Config.h,
// HUB_PIN_CAN_SILENT).
//
// Order of candidates, per ISO 15765-4's initialisation sequence: 500 kbit/s 11-bit, 500 29-bit,
// 250 11-bit, 250 29-bit. After every candidate fails, the poller waits `kRetryMs` and starts
// over (the ignition may simply be off). Once locked, `kLostAfterTimeouts` unanswered requests
// in a row drop the lock and restart the search.
// ---------------------------------------------------------------------------------------------

#include <cstdint>

namespace hub::obd {

struct CanFrame {
    uint32_t id = 0;
    bool extended = false;  // 29-bit identifier
    uint8_t len = 0;
    uint8_t data[8] = {};
};

struct BusConfig {
    uint32_t bitrate = 500000;
    bool extended = false;
};

constexpr BusConfig kCandidates[] = {
    {500000, false}, {500000, true}, {250000, false}, {250000, true}};
constexpr int kCandidateCount = sizeof(kCandidates) / sizeof(kCandidates[0]);

constexpr uint8_t kPidSpeed = 0x0D;
constexpr uint8_t kPidRpm = 0x0C;
constexpr uint32_t kRequest11 = 0x7DF;
constexpr uint32_t kRequest29 = 0x18DB33F1;
constexpr uint32_t kResponse11First = 0x7E8, kResponse11Last = 0x7EF;
constexpr uint32_t kResponse29Base = 0x18DAF100;  // + ECU address in the low byte

constexpr uint32_t kListenMs = 250;
constexpr uint32_t kReplyTimeoutMs = 100;  // J1979: ECUs answer within 50 ms
constexpr int kProbeTries = 2;
constexpr int kLostAfterTimeouts = 10;
constexpr uint32_t kRetryMs = 2000;
constexpr uint32_t kStaleMs = 1000;  // a reading older than this is not valid

inline CanFrame makeRequest(uint8_t pid, bool extended) {
    CanFrame f;
    f.id = extended ? kRequest29 : kRequest11;
    f.extended = extended;
    f.len = 8;
    f.data[0] = 0x02;
    f.data[1] = 0x01;
    f.data[2] = pid;
    return f;
}

inline bool isResponseId(const CanFrame& f, bool extended) {
    if (f.extended != extended) return false;
    if (!extended) return f.id >= kResponse11First && f.id <= kResponse11Last;
    return (f.id & 0x1FFFFF00u) == kResponse29Base;
}

// A positive single-frame service-01 answer for `pid`: its data bytes into a[], count into n.
inline bool parseResponse(const CanFrame& f, bool extended, uint8_t pid, uint8_t a[4], uint8_t& n) {
    if (!isResponseId(f, extended) || f.len < 4) return false;
    const uint8_t pci = f.data[0];
    if ((pci & 0xF0) != 0x00) return false;  // not a single frame (ISO 15765-2)
    const uint8_t len = pci & 0x0F;
    if (len < 3 || len > 7 || len + 1 > f.len) return false;
    if (f.data[1] != 0x41 || f.data[2] != pid) return false;  // 0x7F = negative response
    n = static_cast<uint8_t>(len - 2);
    if (n > 4) n = 4;
    for (uint8_t i = 0; i < n; ++i) a[i] = f.data[3 + i];
    return true;
}

struct Reading {
    float speedKph = 0;
    uint16_t rpm = 0;
    bool valid = false;  // a speed answered within kStaleMs
};

// What the hardware must do after a step.
struct Action {
    bool configure = false;  // (re)initialise the controller with `config`, silent or not
    BusConfig config;
    bool silent = true;
    bool send = false;  // transmit `frame` once
    CanFrame frame;
};

class ObdPoller {
public:
    enum class State { LISTEN, PROBE, LOCKED, BACKOFF };

    // Call regularly (every 20 ms from SensorTask). Feed every received frame with onFrame() and
    // any bus error seen since the last step with onBusError() BEFORE calling step().
    Action step(uint32_t nowMs) {
        Action act;
        if (!started_) {
            started_ = true;
            enterListen(0, nowMs, act);
            return act;
        }
        switch (state_) {
            case State::LISTEN:
                if (busError_) {
                    nextCandidate(nowMs, act);
                } else if (nowMs - since_ >= kListenMs) {
                    state_ = State::PROBE;
                    tries_ = 0;
                    act.configure = true;  // leave silent mode
                    act.config = kCandidates[candidate_];
                    act.silent = false;
                    request(kPidSpeed, nowMs, act);
                }
                break;
            case State::PROBE:
                if (answered_) {
                    state_ = State::LOCKED;
                    timeouts_ = 0;
                    request(kPidRpm, nowMs, act);
                } else if (busError_) {
                    nextCandidate(nowMs, act);
                } else if (nowMs - sentMs_ >= kReplyTimeoutMs) {
                    if (++tries_ >= kProbeTries)
                        nextCandidate(nowMs, act);
                    else
                        request(kPidSpeed, nowMs, act);
                }
                break;
            case State::LOCKED:
                if (answered_) {
                    timeouts_ = 0;
                    request(pending_ == kPidSpeed ? kPidRpm : kPidSpeed, nowMs, act);
                } else if (nowMs - sentMs_ >= kReplyTimeoutMs) {
                    if (++timeouts_ >= kLostAfterTimeouts)
                        enterListen(0, nowMs, act);
                    else
                        request(pending_ == kPidSpeed ? kPidRpm : kPidSpeed, nowMs, act);
                }
                break;
            case State::BACKOFF:
                if (nowMs - since_ >= kRetryMs) enterListen(0, nowMs, act);
                break;
        }
        busError_ = false;
        answered_ = false;
        return act;
    }

    void onFrame(const CanFrame& f, uint32_t nowMs) {
        if (state_ != State::PROBE && state_ != State::LOCKED) return;
        const bool ext = kCandidates[candidate_].extended;
        uint8_t a[4] = {}, n = 0;
        if (!parseResponse(f, ext, pending_, a, n)) return;
        if (pending_ == kPidSpeed && n >= 1) {
            reading_.speedKph = a[0];
            lastSpeedMs_ = nowMs;
            haveSpeed_ = true;
            answered_ = true;
        } else if (pending_ == kPidRpm && n >= 2) {
            reading_.rpm = static_cast<uint16_t>((a[0] * 256u + a[1]) / 4u);
            answered_ = true;
        }
    }
    void onBusError() { busError_ = true; }

    Reading reading(uint32_t nowMs) const {
        Reading r = reading_;
        r.valid = haveSpeed_ && state_ == State::LOCKED && nowMs - lastSpeedMs_ <= kStaleMs;
        return r;
    }
    State state() const { return state_; }
    BusConfig config() const { return kCandidates[candidate_]; }

private:
    void enterListen(int candidate, uint32_t nowMs, Action& act) {
        state_ = State::LISTEN;
        candidate_ = candidate;
        since_ = nowMs;
        act.configure = true;
        act.config = kCandidates[candidate_];
        act.silent = true;
    }
    void nextCandidate(uint32_t nowMs, Action& act) {
        if (candidate_ + 1 < kCandidateCount) {
            enterListen(candidate_ + 1, nowMs, act);
        } else {
            state_ = State::BACKOFF;
            since_ = nowMs;
            act.configure = true;  // park the controller silent while waiting
            act.config = kCandidates[0];
            act.silent = true;
        }
    }
    void request(uint8_t pid, uint32_t nowMs, Action& act) {
        pending_ = pid;
        sentMs_ = nowMs;
        act.send = true;
        act.frame = makeRequest(pid, kCandidates[candidate_].extended);
    }

    bool started_ = false;
    State state_ = State::LISTEN;
    int candidate_ = 0, tries_ = 0, timeouts_ = 0;
    uint32_t since_ = 0, sentMs_ = 0, lastSpeedMs_ = 0;
    uint8_t pending_ = kPidSpeed;
    bool busError_ = false, answered_ = false, haveSpeed_ = false;
    Reading reading_;
};

}  // namespace hub::obd
