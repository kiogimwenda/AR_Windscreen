// SafetyCore and FrameCodec tests — see docs/BUILD_GUIDE.md Part 3.4, 4.5, 4.6, Appendix B.
// Run on the laptop with a fake clock: `pio test -e native`. No board needed.
//
// Each rule of the hub's actuation safety is tested where it must act and where it must not, and
// a randomised fuzz checks the invariants that must hold after EVERY tick, whatever happens.

#include <unity.h>

#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include "hub/FrameCodec.h"
#include "hub/SafetyCore.h"

using namespace hub;
using hub_protocol::ActuationCommand;
using hub_protocol::MessageType;

void setUp(void) {}
void tearDown(void) {}

namespace {

HardwareInputs ok(uint32_t t, float amps = 0) {
    HardwareInputs in;
    in.nowMs = t;
    in.killSwitchEngaged = false;
    in.powerBoxPresent = true;
    in.currentAmps = amps;
    return in;
}

ActuationCommand brake(uint8_t intensity, uint16_t durMs, uint32_t hostTs) {
    ActuationCommand c{};
    c.brakeRequest = intensity;
    c.brakeDurationMs = durMs;
    c.hostTimestampMs = hostTs;
    return c;
}

// A hub that has booted, sent a report and heard a heartbeat at time t: armed.
SafetyCore armedAt(uint32_t t) {
    SafetyCore s;
    s.onReportSent(t);
    s.onValidFrame(t, MessageType::HEARTBEAT);
    s.tick(ok(t));
    return s;
}

// Keeps the link and report loop alive every 20 ms from `from` to `to`.
void keepAlive(SafetyCore& s, uint32_t from, uint32_t to,
               MessageType type = MessageType::HEARTBEAT) {
    for (uint32_t t = from; t <= to; t += 20) {
        s.onReportSent(t);
        s.onValidFrame(t, type);
        s.tick(ok(t));
    }
}

}  // namespace

// --- Arming ------------------------------------------------------------------------------------

static void test_boots_disarmed_and_inert(void) {
    SafetyCore s;
    const HardwareOutputs o = s.tick(ok(0));
    TEST_ASSERT_FALSE(s.armed(0));
    TEST_ASSERT_EQUAL_UINT8(0, o.brakeIntensity);
    TEST_ASSERT_FALSE(o.cableMagnet);
    TEST_ASSERT_FALSE(s.onCommand(0, brake(50, 500, 1)).lastCommandAccepted);
}

// Arming needs a HEARTBEAT that arrives while every other condition already holds. A heartbeat
// before the report loop has started does not count; the next one (within 100 ms at the host's
// 10 Hz) arms. Conservative by design: re-arming is always an explicit, current signal.
static void test_needs_report_and_heartbeat_to_arm(void) {
    SafetyCore s;
    s.onValidFrame(0, MessageType::HEARTBEAT);
    s.tick(ok(0));
    TEST_ASSERT_FALSE_MESSAGE(s.armed(0), "no SensorReport sent yet: report loop not proven alive");
    s.onReportSent(10);
    s.tick(ok(10));
    TEST_ASSERT_FALSE_MESSAGE(s.armed(10), "that heartbeat came before the report loop was alive");
    s.onValidFrame(100, MessageType::HEARTBEAT);
    s.onReportSent(100);
    s.tick(ok(100));
    TEST_ASSERT_TRUE(s.armed(100));
}

// --- Brake command: the second ceiling and the duration limit ----------------------------------

static void test_brake_is_clamped_to_the_hub_ceiling(void) {
    SafetyCore s = armedAt(0);
    const auto ack = s.onCommand(5, brake(255, 800, 1));
    TEST_ASSERT_TRUE(ack.lastCommandAccepted);
    const HardwareOutputs o = s.tick(ok(10));
    TEST_ASSERT_EQUAL_UINT8(hub_config::kMaxSafeIntensity, o.brakeIntensity);
    TEST_ASSERT_TRUE(o.cableMagnet);
}

static void test_brake_releases_when_its_duration_ends(void) {
    SafetyCore s = armedAt(0);
    s.onCommand(0, brake(60, 300, 1));
    keepAlive(s, 20, 280);
    TEST_ASSERT_EQUAL_UINT8(60, s.tick(ok(290)).brakeIntensity);
    s.onReportSent(300);
    s.onValidFrame(300, MessageType::HEARTBEAT);
    TEST_ASSERT_EQUAL_UINT8(0, s.tick(ok(300)).brakeIntensity);
}

static void test_duration_is_capped_by_the_hub(void) {
    SafetyCore s = armedAt(0);
    s.onCommand(0, brake(60, 60000, 1));  // the host asks for a minute
    keepAlive(s, 20, hub_config::kMaxBrakeDurationMs - 20);
    s.onReportSent(hub_config::kMaxBrakeDurationMs);
    s.onValidFrame(hub_config::kMaxBrakeDurationMs, MessageType::HEARTBEAT);
    TEST_ASSERT_EQUAL_UINT8(0, s.tick(ok(hub_config::kMaxBrakeDurationMs)).brakeIntensity);
}

// --- Link loss (Part 3.4) ----------------------------------------------------------------------

// No valid frame for MORE than 200 ms: released on that tick. Exactly 200 ms is still alive.
static void test_link_loss_releases_after_200_ms(void) {
    SafetyCore s = armedAt(0);
    s.onCommand(0, brake(60, 1500, 1));
    for (uint32_t t = 10; t <= 200; t += 10) {
        s.onReportSent(t);
        TEST_ASSERT_EQUAL_UINT8(60, s.tick(ok(t)).brakeIntensity);
    }
    s.onReportSent(201);
    const HardwareOutputs o = s.tick(ok(201));
    TEST_ASSERT_EQUAL_UINT8(0, o.brakeIntensity);
    TEST_ASSERT_FALSE(o.cableMagnet);
    TEST_ASSERT_EQUAL_UINT8(FAULT_LINK_TIMEOUT, s.faultCode(201));
    TEST_ASSERT_TRUE(s.mustRelease(201, false, true));
}

// After a link loss, a non-heartbeat frame does not re-arm; a heartbeat does. The old brake
// request is NOT resumed.
static void test_rearm_needs_a_fresh_heartbeat_and_forgets_the_old_command(void) {
    SafetyCore s = armedAt(0);
    s.onCommand(0, brake(60, 1500, 1));
    s.onReportSent(300);
    s.tick(ok(300));  // link lost
    keepAlive(s, 320, 400, MessageType::ACTUATION_COMMAND);
    TEST_ASSERT_FALSE_MESSAGE(s.armed(400), "only a HEARTBEAT re-arms");
    s.onValidFrame(410, MessageType::HEARTBEAT);
    s.onReportSent(410);
    const HardwareOutputs o = s.tick(ok(410));
    TEST_ASSERT_TRUE(s.armed(410));
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, o.brakeIntensity, "the old request must not resume");
}

// The hub's own report loop stalling (Part 3.4 (2)) disarms, even with the link fine.
static void test_report_loop_stall_disarms(void) {
    SafetyCore s = armedAt(0);
    s.onCommand(0, brake(60, 1500, 1));
    for (uint32_t t = 20; t <= 300; t += 20) s.onValidFrame(t, MessageType::HEARTBEAT);
    const HardwareOutputs o = s.tick(ok(300));
    TEST_ASSERT_EQUAL_UINT8(0, o.brakeIntensity);
    TEST_ASSERT_TRUE(s.mustRelease(300, false, true));
}

// --- Kill switch ---------------------------------------------------------------------------------

static void test_kill_switch_releases_and_rejects(void) {
    SafetyCore s = armedAt(0);
    s.onCommand(0, brake(60, 1500, 1));
    HardwareInputs in = ok(10);
    in.killSwitchEngaged = true;
    const HardwareOutputs o = s.tick(in);
    TEST_ASSERT_EQUAL_UINT8(0, o.brakeIntensity);
    TEST_ASSERT_EQUAL_UINT8(FAULT_KILL_SWITCH, s.faultCode(10));
    TEST_ASSERT_FALSE(s.onCommand(15, brake(60, 500, 2)).lastCommandAccepted);
    // Released again: needs a fresh heartbeat.
    s.onReportSent(40);
    s.tick(ok(40));
    TEST_ASSERT_FALSE(s.armed(40));
    s.onValidFrame(50, MessageType::HEARTBEAT);
    s.onReportSent(50);
    s.tick(ok(50));
    TEST_ASSERT_TRUE(s.armed(50));
}

// --- Overcurrent (latched until power cycle)
// ------------------------------------------------------

static void test_overcurrent_spike_does_not_latch_but_sustained_does(void) {
    SafetyCore s = armedAt(0);
    const float over = hub_config::kOvercurrentAmps + 1;
    s.onCommand(0, brake(60, 1500, 1));
    keepAlive(s, 10, 10);
    s.tick(ok(20, over));
    s.tick(ok(30, over));  // two ticks: an inrush spike
    s.tick(ok(40, 1.0f));
    TEST_ASSERT_FALSE(s.overcurrentLatched());
    for (uint32_t t = 50; t <= 70; t += 10) s.tick(ok(t, over));  // three ticks
    TEST_ASSERT_TRUE(s.overcurrentLatched());
    TEST_ASSERT_EQUAL_UINT8(FAULT_OVERCURRENT, s.faultCode(70));
    // Nothing in software clears it: normal current, heartbeats, new commands.
    keepAlive(s, 80, 2000);
    TEST_ASSERT_TRUE(s.overcurrentLatched());
    TEST_ASSERT_FALSE(s.onCommand(2000, brake(60, 500, 99)).lastCommandAccepted);
    TEST_ASSERT_EQUAL_UINT8(0, s.tick(ok(2010)).brakeIntensity);
}

// --- Commands ------------------------------------------------------------------------------------

static void test_stale_or_replayed_commands_never_act(void) {
    SafetyCore s = armedAt(0);
    TEST_ASSERT_TRUE(s.onCommand(0, brake(40, 500, 100)).lastCommandAccepted);
    TEST_ASSERT_FALSE_MESSAGE(s.onCommand(5, brake(80, 500, 100)).lastCommandAccepted, "replay");
    TEST_ASSERT_FALSE_MESSAGE(s.onCommand(6, brake(80, 500, 99)).lastCommandAccepted, "older");
    TEST_ASSERT_EQUAL_UINT8(40, s.tick(ok(10)).brakeIntensity);
    TEST_ASSERT_TRUE(s.onCommand(12, brake(80, 500, 101)).lastCommandAccepted);
    TEST_ASSERT_EQUAL_UINT8(80, s.tick(ok(20)).brakeIntensity);
}

// The arbiter's zeroed "no request" (Part 3.3) always releases, armed or not, whatever its
// timestamp.
static void test_zeroed_command_always_releases(void) {
    SafetyCore s = armedAt(0);
    s.onCommand(0, brake(60, 1500, 500));
    ActuationCommand zero{};
    TEST_ASSERT_TRUE(s.onCommand(5, zero).lastCommandAccepted);
    TEST_ASSERT_EQUAL_UINT8(0, s.tick(ok(10)).brakeIntensity);
    SafetyCore d;  // disarmed
    TEST_ASSERT_TRUE(d.onCommand(0, zero).lastCommandAccepted);
}

static void test_relays_follow_the_command_and_drop_when_disarmed(void) {
    SafetyCore s = armedAt(0);
    ActuationCommand c{};
    c.indicators = 0x04;  // hazards
    c.lights = 0x02;      // horn
    c.hostTimestampMs = 1;
    s.onCommand(0, c);
    HardwareOutputs o = s.tick(ok(10));
    TEST_ASSERT_EQUAL_UINT8(0x04, o.indicators);
    TEST_ASSERT_EQUAL_UINT8(0x02, o.lights);
    s.onReportSent(300);
    o = s.tick(ok(300));  // link lost
    TEST_ASSERT_EQUAL_UINT8(0, o.indicators);
    TEST_ASSERT_EQUAL_UINT8(0, o.lights);
}

// --- Frame errors (Appendix B fault 4) -----------------------------------------------------------

static void test_frame_error_rate_disarms_and_recovers(void) {
    SafetyCore s = armedAt(0);
    uint32_t t = 0;
    for (int i = 0; i < hub_config::kFrameErrorsMax + 1; ++i) {
        s.onFrameError();
        s.onValidFrame(t += 5, MessageType::HEARTBEAT);
        s.onReportSent(t);
    }
    s.tick(ok(t));
    TEST_ASSERT_EQUAL_UINT8(FAULT_FRAME_ERRORS, s.faultCode(t));
    TEST_ASSERT_FALSE(s.armed(t));
    for (int i = 0; i < hub_config::kFrameWindow; ++i) {  // a clean window of heartbeats
        s.onValidFrame(t += 5, MessageType::HEARTBEAT);
        s.onReportSent(t);
        s.tick(ok(t));
    }
    TEST_ASSERT_TRUE(s.armed(t));
}

static void test_fault_priority(void) {
    SafetyCore s = armedAt(0);
    for (uint32_t t = 10; t <= 40; t += 10) s.tick(ok(t, 50.0f));  // overcurrent
    HardwareInputs in = ok(50);
    in.killSwitchEngaged = true;
    in.powerBoxPresent = false;
    s.tick(in);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(FAULT_OVERCURRENT, s.faultCode(50),
                                    "overcurrent outranks everything");
    // An unplugged inter-box cable also opens the kill-sense line: report the cause (5), not the
    // symptom (2).
    SafetyCore u = armedAt(0);
    u.tick(in);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(FAULT_POWER_BOX, u.faultCode(50),
                                    "power box outranks the kill switch it disconnects");
}

// --- Two-box hub: the power box (Part 4.8)
// --------------------------------------------------------

// Unplugging the inter-box cable mid-brake releases on that tick, rejects new requests, and
// reports fault 5. The power board's pull-downs release in hardware too; this is the firmware half.
static void test_power_box_disconnect_releases_and_rejects(void) {
    SafetyCore s = armedAt(0);
    s.onCommand(0, brake(60, 1500, 1));
    TEST_ASSERT_EQUAL_UINT8(60, s.tick(ok(10)).brakeIntensity);
    HardwareInputs in = ok(20);
    in.powerBoxPresent = false;
    const HardwareOutputs o = s.tick(in);
    TEST_ASSERT_EQUAL_UINT8(0, o.brakeIntensity);
    TEST_ASSERT_FALSE(o.cableMagnet);
    TEST_ASSERT_EQUAL_UINT8(0, o.indicators);
    TEST_ASSERT_FALSE(s.armed(20));
    TEST_ASSERT_TRUE(s.mustRelease(20, false, false));
    TEST_ASSERT_EQUAL_UINT8(FAULT_POWER_BOX, s.faultCode(20));
    TEST_ASSERT_FALSE(s.onCommand(25, brake(60, 500, 2)).lastCommandAccepted);
}

// With the cable out, the current-sense line reads ~0 V, which converts to ~25 A. That reading
// must never count toward the overcurrent latch, or a routine unplug would need a power cycle.
static void test_floating_current_sense_never_latches_while_box_absent(void) {
    SafetyCore s = armedAt(0);
    for (uint32_t t = 10; t <= 500; t += 10) {
        HardwareInputs in = ok(t, 25.0f);
        in.powerBoxPresent = false;
        s.tick(in);
    }
    TEST_ASSERT_FALSE(s.overcurrentLatched());
    TEST_ASSERT_EQUAL_UINT8(FAULT_POWER_BOX, s.faultCode(500));
}

// Plugging the cable back in does not re-arm by itself: a fresh heartbeat must arrive while the
// box is present, and the brake request from before the unplug is gone.
static void test_power_box_reconnect_needs_a_heartbeat_and_forgets_the_command(void) {
    SafetyCore s = armedAt(0);
    s.onCommand(0, brake(60, 1500, 1));
    HardwareInputs in = ok(10);
    in.powerBoxPresent = false;
    s.tick(in);
    s.onReportSent(20);
    s.onValidFrame(20, MessageType::ACTUATION_COMMAND);
    s.tick(ok(20));  // box back, but no heartbeat since
    TEST_ASSERT_FALSE_MESSAGE(s.armed(20), "reconnecting alone must not re-arm");
    TEST_ASSERT_EQUAL_UINT8(FAULT_NONE, s.faultCode(20));
    s.onReportSent(40);
    s.onValidFrame(40, MessageType::HEARTBEAT);
    const HardwareOutputs o = s.tick(ok(40));
    TEST_ASSERT_TRUE(s.armed(40));
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, o.brakeIntensity, "the old request must not resume");
}

// --- Invariants under random event sequences -----------------------------------------------------

// 200 runs x 3,000 random events: frames of any type, frame errors, commands with any fields and
// timestamps, kill-switch changes, power-box unplugs, current spikes, gaps of up to 400 ms. After
// EVERY tick:
//   - brake intensity <= kMaxSafeIntensity, and nonzero only while armed;
//   - the cable magnet is on exactly when braking;
//   - if mustRelease() says release, nothing is applied;
//   - once latched, overcurrent never clears;
//   - with the power box absent: nothing applied, and a floating current reading never latches.
static void test_invariants_under_random_sequences(void) {
    std::mt19937 rng(20260927);
    int brakingTicks = 0;
    for (int run = 0; run < 200; ++run) {
        SafetyCore s;
        uint32_t t = 0, hostTs = 0;
        bool kill = false, box = true, latched = false;
        for (int k = 0; k < 3000; ++k) {
            t += (rng() % 100 < 97) ? rng() % 30 : 200 + rng() % 200;
            switch (rng() % 10) {
                case 0:
                case 1:
                case 2:
                    s.onValidFrame(t, MessageType::HEARTBEAT);
                    break;
                case 3:
                    s.onValidFrame(t, MessageType::ACTUATION_COMMAND);
                    {
                        ActuationCommand c{};
                        c.brakeRequest = static_cast<uint8_t>(rng());
                        c.brakeDurationMs = static_cast<uint16_t>(rng());
                        c.indicators = static_cast<uint8_t>(rng());
                        c.lights = static_cast<uint8_t>(rng());
                        hostTs += (rng() % 10 < 8) ? 1 + rng() % 50 : 0;  // some replays
                        c.hostTimestampMs = (rng() % 20 == 0) ? hostTs - rng() % 100 : hostTs;
                        s.onCommand(t, c);
                    }
                    break;
                case 4:
                    s.onFrameError();
                    break;
                case 5:
                case 6:
                    s.onReportSent(t);
                    break;
                case 7:
                    if (rng() % 20 == 0) kill = !kill;
                    if (rng() % 25 == 0) box = !box;
                    break;
                default:
                    break;
            }
            HardwareInputs in;
            in.nowMs = t;
            in.killSwitchEngaged = kill;
            in.powerBoxPresent = box;
            // With the box absent, the sense line floats: garbage, often far above the limit.
            in.currentAmps = !box                ? static_cast<float>(rng() % 30)
                             : (rng() % 50 == 0) ? hub_config::kOvercurrentAmps + 5
                                                 : 2.0f;
            const bool latchedBefore = s.overcurrentLatched();
            const bool release = s.mustRelease(t, kill, box);
            const HardwareOutputs o = s.tick(in);
            TEST_ASSERT_LESS_OR_EQUAL_UINT8(hub_config::kMaxSafeIntensity, o.brakeIntensity);
            TEST_ASSERT_EQUAL(o.brakeIntensity > 0, o.cableMagnet);
            if (o.brakeIntensity > 0) {
                ++brakingTicks;
                TEST_ASSERT_TRUE(s.armed(t));
                TEST_ASSERT_FALSE(release);
            }
            if (latched) TEST_ASSERT_TRUE(s.overcurrentLatched());
            if (!box && !latchedBefore)
                TEST_ASSERT_FALSE_MESSAGE(s.overcurrentLatched(), "latched on a floating line");
            if (!box) TEST_ASSERT_EQUAL_UINT8(0, o.brakeIntensity);
            latched = s.overcurrentLatched();
        }
    }
    TEST_ASSERT_GREATER_THAN_MESSAGE(1000, brakingTicks, "the fuzz must actually reach braking");
}

// --- FrameCodec ----------------------------------------------------------------------------------

// Known answer, from an independent Python CRC implementation: a HEARTBEAT frame (empty payload).
static void test_heartbeat_frame_known_bytes(void) {
    uint8_t out[hub_protocol::MAX_FRAME_SIZE];
    const std::size_t n = encodeFrame(MessageType::HEARTBEAT, nullptr, 0, out);
    const uint8_t expect[] = {0xAA, 0x04, 0x00, 0x00, 0x5C, 0x10};
    TEST_ASSERT_EQUAL(sizeof(expect), n);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expect, out, n);
}

static void test_round_trip_through_garbage(void) {
    ActuationCommand c = brake(77, 900, 12345);
    uint8_t frame[hub_protocol::MAX_FRAME_SIZE];
    const std::size_t n = encodeFrame(MessageType::ACTUATION_COMMAND, &c, sizeof c, frame);
    std::vector<uint8_t> stream = {0x13, 0x37, 0x00};  // noise before the frame
    stream.insert(stream.end(), frame, frame + n);
    FrameParser p;
    int frames = 0;
    for (uint8_t b : stream) {
        if (p.feed(b) == FrameParser::Result::FRAME) {
            ++frames;
            TEST_ASSERT_EQUAL(static_cast<int>(MessageType::ACTUATION_COMMAND),
                              static_cast<int>(p.type()));
            ActuationCommand got{};
            TEST_ASSERT_TRUE(p.as(got));
            TEST_ASSERT_EQUAL_UINT8(77, got.brakeRequest);
            TEST_ASSERT_EQUAL_UINT32(12345, got.hostTimestampMs);
        }
    }
    TEST_ASSERT_EQUAL(1, frames);
}

// A corrupted actuation command is NEVER delivered (Part 3.1). And the parser resynchronises
// IMMEDIATELY: the very next good frame is delivered, even when the corruption hit the length
// field (the bad frame's swallowed bytes are re-scanned).
static void test_corrupted_frame_is_rejected_and_parser_recovers(void) {
    ActuationCommand c = brake(90, 500, 7);
    uint8_t frame[hub_protocol::MAX_FRAME_SIZE];
    const std::size_t n = encodeFrame(MessageType::ACTUATION_COMMAND, &c, sizeof c, frame);
    uint8_t hb[hub_protocol::MAX_FRAME_SIZE];
    const std::size_t hn = encodeFrame(MessageType::HEARTBEAT, nullptr, 0, hb);
    for (std::size_t bit = 8; bit < n * 8; ++bit) {  // every single-bit flip after the start byte
        FrameParser p;
        uint8_t bad[hub_protocol::MAX_FRAME_SIZE];
        std::memcpy(bad, frame, n);
        bad[bit / 8] ^= static_cast<uint8_t>(1u << (bit % 8));
        int commands = 0, heartbeats = 0;
        auto drain = [&] {
            for (FrameParser::Result r; (r = p.next()) != FrameParser::Result::NONE;)
                if (r == FrameParser::Result::FRAME) {
                    if (p.type() == MessageType::ACTUATION_COMMAND) ++commands;
                    if (p.type() == MessageType::HEARTBEAT) ++heartbeats;
                }
        };
        for (std::size_t i = 0; i < n; ++i) p.push(bad[i]);
        drain();
        for (int rep = 0; rep < 5; ++rep) {  // then five good heartbeats, each due on its last byte
            const int before = heartbeats;
            for (std::size_t i = 0; i < hn; ++i) p.push(hb[i]);
            drain();
            TEST_ASSERT_EQUAL_MESSAGE(before + 1, heartbeats,
                                      "heartbeat delayed behind the corrupted frame");
        }
        // With a corrupted length, the parser is still waiting for the bogus payload while the
        // heartbeats arrive; the re-scan must still find every one once the bogus frame fails.
        // A frame whose flipped bit landed in `type` is still a valid frame of another type only
        // if its CRC matched, which it cannot: every flip must be rejected.
        TEST_ASSERT_EQUAL_MESSAGE(0, commands, "a corrupted command was delivered");
        TEST_ASSERT_EQUAL_MESSAGE(5, heartbeats,
                                  "every good frame after the corruption, delivered on arrival");
    }
}

static void test_oversized_length_is_an_error(void) {
    const uint8_t hdr[] = {0xAA, 0x02, 65, 0};  // length 65 > MAX_PAYLOAD
    FrameParser p;
    FrameParser::Result r = FrameParser::Result::NONE;
    for (uint8_t b : hdr) r = p.feed(b);
    TEST_ASSERT_EQUAL(static_cast<int>(FrameParser::Result::ERROR), static_cast<int>(r));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_boots_disarmed_and_inert);
    RUN_TEST(test_needs_report_and_heartbeat_to_arm);
    RUN_TEST(test_brake_is_clamped_to_the_hub_ceiling);
    RUN_TEST(test_brake_releases_when_its_duration_ends);
    RUN_TEST(test_duration_is_capped_by_the_hub);
    RUN_TEST(test_link_loss_releases_after_200_ms);
    RUN_TEST(test_rearm_needs_a_fresh_heartbeat_and_forgets_the_old_command);
    RUN_TEST(test_report_loop_stall_disarms);
    RUN_TEST(test_kill_switch_releases_and_rejects);
    RUN_TEST(test_overcurrent_spike_does_not_latch_but_sustained_does);
    RUN_TEST(test_stale_or_replayed_commands_never_act);
    RUN_TEST(test_zeroed_command_always_releases);
    RUN_TEST(test_relays_follow_the_command_and_drop_when_disarmed);
    RUN_TEST(test_frame_error_rate_disarms_and_recovers);
    RUN_TEST(test_fault_priority);
    RUN_TEST(test_power_box_disconnect_releases_and_rejects);
    RUN_TEST(test_floating_current_sense_never_latches_while_box_absent);
    RUN_TEST(test_power_box_reconnect_needs_a_heartbeat_and_forgets_the_command);
    RUN_TEST(test_invariants_under_random_sequences);
    RUN_TEST(test_heartbeat_frame_known_bytes);
    RUN_TEST(test_round_trip_through_garbage);
    RUN_TEST(test_corrupted_frame_is_rejected_and_parser_recovers);
    RUN_TEST(test_oversized_length_is_an_error);
    return UNITY_END();
}
