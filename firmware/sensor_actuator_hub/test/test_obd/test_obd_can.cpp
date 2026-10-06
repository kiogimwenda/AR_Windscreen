// ObdCan tests — see docs/BUILD_GUIDE.md Part 4.8.5. `pio test -e native`; no board or car needed.
//
// A simulated car answers (or does not) at one bit rate and addressing mode; the poller must find
// it, read speed and rpm, never transmit while its listen at a candidate saw bus errors, never
// retry a failed frame, and recover when the car goes quiet.

#include <unity.h>

#include <cstdint>

#include "hub/ObdCan.h"

using namespace hub::obd;

void setUp(void) {}
void tearDown(void) {}

namespace {

// A car on the bus: `rate` and `ext` are what its ECU speaks. It answers a request only if the
// controller is configured the same way; at a wrong rate any transmission is a bus error.
struct SimCar {
    uint32_t rate = 500000;
    bool ext = false;
    bool answers = true;
    bool chattyBus = false;  // other traffic on the bus: a wrong-rate listen sees errors
    uint8_t speed = 57;
    uint16_t rpm = 1726;
    uint8_t ecu = 0;  // 0x7E8 + ecu
};

struct Harness {
    ObdPoller p;
    SimCar car;
    BusConfig cfg;
    bool silent = true;
    int sends = 0, sendsWhileSilent = 0, sendsAtWrongRate = 0, sendsAfterListenErrors = 0;
    bool listenSawErrors = false;
    uint32_t t = 0;

    bool matches() const { return cfg.bitrate == car.rate && cfg.extended == car.ext; }

    void run(uint32_t untilMs) {
        for (; t <= untilMs; t += 20) {
            const Action a = p.step(t);
            if (a.configure) {
                cfg = a.config;
                silent = a.silent;
                listenSawErrors = false;
            }
            // Listening at the wrong rate on a busy bus sees errors (before the next step).
            if (silent && car.chattyBus && cfg.bitrate != car.rate) {
                listenSawErrors = true;
                p.onBusError();
            }
            if (a.send) {
                ++sends;
                if (silent) ++sendsWhileSilent;
                if (listenSawErrors) ++sendsAfterListenErrors;
                TEST_ASSERT_EQUAL_UINT8(0x01, a.frame.data[1]);  // only ever service 01
                if (cfg.bitrate != car.rate) {
                    ++sendsAtWrongRate;
                    p.onBusError();  // no ACK at the wrong rate: one failed frame
                } else if (car.answers && matches()) {
                    CanFrame r;
                    r.extended = car.ext;
                    r.id = car.ext ? (kResponse29Base | 0x10u) : kResponse11First + car.ecu;
                    r.len = 8;
                    const uint8_t pid = a.frame.data[2];
                    r.data[1] = 0x41;
                    r.data[2] = pid;
                    if (pid == kPidSpeed) {
                        r.data[0] = 3;
                        r.data[3] = car.speed;
                    } else {
                        r.data[0] = 4;
                        r.data[3] = static_cast<uint8_t>((car.rpm * 4) >> 8);
                        r.data[4] = static_cast<uint8_t>((car.rpm * 4) & 0xFF);
                    }
                    p.onFrame(r, t + 5);
                }
            }
        }
    }
};

}  // namespace

static void test_request_frame_layout(void) {
    const CanFrame f = makeRequest(kPidSpeed, false);
    TEST_ASSERT_EQUAL_HEX32(0x7DF, f.id);
    TEST_ASSERT_FALSE(f.extended);
    TEST_ASSERT_EQUAL_UINT8(8, f.len);
    const uint8_t expect[8] = {0x02, 0x01, 0x0D, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, f.data, 8);
    TEST_ASSERT_EQUAL_HEX32(0x18DB33F1, makeRequest(kPidRpm, true).id);
}

static void test_response_parsing(void) {
    CanFrame r;
    r.id = 0x7E8;
    r.len = 8;
    const uint8_t rpm[8] = {0x04, 0x41, 0x0C, 0x1A, 0xF8, 0, 0, 0};  // (0x1A*256 + 0xF8)/4 = 1726
    for (int i = 0; i < 8; ++i) r.data[i] = rpm[i];
    uint8_t a[4], n = 0;
    TEST_ASSERT_TRUE(parseResponse(r, false, kPidRpm, a, n));
    TEST_ASSERT_EQUAL_UINT8(2, n);
    TEST_ASSERT_EQUAL_UINT8(0x1A, a[0]);
    TEST_ASSERT_EQUAL_UINT8(0xF8, a[1]);
    TEST_ASSERT_FALSE_MESSAGE(parseResponse(r, false, kPidSpeed, a, n), "wrong PID");
    TEST_ASSERT_FALSE_MESSAGE(parseResponse(r, true, kPidRpm, a, n), "wrong addressing");
    r.id = 0x7F0;
    TEST_ASSERT_FALSE_MESSAGE(parseResponse(r, false, kPidRpm, a, n), "not an ECU response ID");
    r.id = 0x7E8;
    r.data[1] = 0x7F;  // negative response
    TEST_ASSERT_FALSE(parseResponse(r, false, kPidRpm, a, n));
    r.data[1] = 0x41;
    r.data[0] = 0x10;  // first frame of a multi-frame message
    TEST_ASSERT_FALSE(parseResponse(r, false, kPidRpm, a, n));
    CanFrame e;
    e.extended = true;
    e.id = 0x18DAF110;
    e.len = 8;
    const uint8_t sp[8] = {0x03, 0x41, 0x0D, 57, 0, 0, 0, 0};
    for (int i = 0; i < 8; ++i) e.data[i] = sp[i];
    TEST_ASSERT_TRUE(parseResponse(e, true, kPidSpeed, a, n));
    TEST_ASSERT_EQUAL_UINT8(57, a[0]);
    e.id = 0x18DAF210;  // addressed to another tester
    TEST_ASSERT_FALSE(parseResponse(e, true, kPidSpeed, a, n));
}

// The common case: 500 kbit/s, 11-bit. Locks after one clean listen and reads both values.
static void test_locks_on_500k_11bit_and_reads_speed_and_rpm(void) {
    Harness h;
    h.run(1000);
    TEST_ASSERT_EQUAL(static_cast<int>(ObdPoller::State::LOCKED), static_cast<int>(h.p.state()));
    const Reading r = h.p.reading(h.t);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_FLOAT(57.0f, r.speedKph);
    TEST_ASSERT_EQUAL_UINT16(1726, r.rpm);
    TEST_ASSERT_EQUAL(0, h.sendsWhileSilent);
    TEST_ASSERT_EQUAL(0, h.sendsAtWrongRate);
}

// A 250 kbit/s 29-bit car on a busy bus: every wrong-rate listen sees errors and moves on WITHOUT
// transmitting; the only wrong-format sends are at the right rate (11-bit vs 29-bit), which are
// harmless valid frames nobody answers.
static void test_busy_bus_wrong_rates_never_transmitted_on(void) {
    Harness h;
    h.car.rate = 250000;
    h.car.ext = true;
    h.car.chattyBus = true;
    h.run(3000);
    TEST_ASSERT_EQUAL(static_cast<int>(ObdPoller::State::LOCKED), static_cast<int>(h.p.state()));
    TEST_ASSERT_TRUE(h.p.config().extended);
    TEST_ASSERT_EQUAL_UINT32(250000, h.p.config().bitrate);
    TEST_ASSERT_EQUAL_MESSAGE(0, h.sendsAtWrongRate, "transmitted at a rate the bus rejected");
    TEST_ASSERT_EQUAL(0, h.sendsAfterListenErrors);
    TEST_ASSERT_EQUAL(0, h.sendsWhileSilent);
    TEST_ASSERT_TRUE(h.p.reading(h.t).valid);
}

// A quiet bus behind a gateway (no traffic to listen to): the probe at a wrong rate fails ONCE,
// is not retried, and the poller moves to the next candidate.
static void test_quiet_bus_wrong_rate_fails_once_then_moves_on(void) {
    Harness h;
    h.car.rate = 250000;
    h.run(3000);
    TEST_ASSERT_EQUAL(static_cast<int>(ObdPoller::State::LOCKED), static_cast<int>(h.p.state()));
    TEST_ASSERT_EQUAL_UINT32(250000, h.p.config().bitrate);
    TEST_ASSERT_EQUAL_MESSAGE(2, h.sendsAtWrongRate,
                              "one failed frame per wrong-rate candidate (500/11 and 500/29)");
}

// Ignition off: no answers anywhere. The poller backs off, reports invalid, never floods.
static void test_no_answer_backs_off_and_stays_invalid(void) {
    Harness h;
    h.car.answers = false;
    h.run(10000);
    TEST_ASSERT_FALSE(h.p.reading(h.t).valid);
    // Per cycle: 4 candidates x 2 probe tries; one cycle per ~(4 x 250 + 8 x 100 + 2000) ms.
    TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(40, h.sends, "flooding a silent bus");
}

// The car goes quiet after locking (engine switched off): the reading goes stale within
// kStaleMs, the lock drops, and the poller finds the car again when it comes back.
static void test_loss_goes_invalid_then_recovers(void) {
    Harness h;
    h.run(1000);
    TEST_ASSERT_TRUE(h.p.reading(h.t).valid);
    h.car.answers = false;
    h.run(1000 + kStaleMs + 100);
    TEST_ASSERT_FALSE(h.p.reading(h.t).valid);
    h.run(5000);
    TEST_ASSERT_NOT_EQUAL(static_cast<int>(ObdPoller::State::LOCKED),
                          static_cast<int>(h.p.state()));
    h.car.answers = true;
    h.run(12000);
    TEST_ASSERT_TRUE(h.p.reading(h.t).valid);
}

// Request rate once locked: one request per answer, so at most ~2 per 20 ms step here, and never
// more than one outstanding.
static void test_request_rate_is_bounded(void) {
    Harness h;
    h.run(1000);
    const int before = h.sends;
    h.run(2000);
    TEST_ASSERT_LESS_OR_EQUAL(51, h.sends - before);  // one per 20 ms step at most
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_request_frame_layout);
    RUN_TEST(test_response_parsing);
    RUN_TEST(test_locks_on_500k_11bit_and_reads_speed_and_rpm);
    RUN_TEST(test_busy_bus_wrong_rates_never_transmitted_on);
    RUN_TEST(test_quiet_bus_wrong_rate_fails_once_then_moves_on);
    RUN_TEST(test_no_answer_backs_off_and_stays_invalid);
    RUN_TEST(test_loss_goes_invalid_then_recovers);
    RUN_TEST(test_request_rate_is_bounded);
    return UNITY_END();
}
