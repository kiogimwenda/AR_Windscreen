// Firmware-side CRC16 + protocol layout test — see docs/BUILD_GUIDE.md Part 13.1, Phase 1.
//
// Runs on the HOST machine via `pio test -e native`, with no STM32 board attached. It compiles the
// FIRMWARE's own copies of Crc16.h and Protocol.h and asserts exactly the values that
// host/test/unit/test_crc16.cpp asserts against the host's copies.
//
// That duplication is deliberate. CI's protocol-sync-check proves the two files are byte-identical;
// these two test suites prove each copy independently produces the standard results. Factoring the
// vectors into a shared header would collapse that into a single point of failure, which is the
// opposite of what Part 3's "implemented twice, kept identical" rule is trying to achieve.
//
// Note what this does NOT prove: it runs on x86-64, not on the Cortex-M4. The static_asserts in
// Protocol.h are what cover the actual cross-architecture layout question, and those are checked
// whenever the firmware is compiled for the real target by `pio run`.

#include <unity.h>

#include <cstdint>
#include <cstring>

#include "hub/Crc16.h"
#include "hub/Protocol.h"

void setUp(void) {}
void tearDown(void) {}

// The published check value for CRC-16/CCITT-FALSE. Same assertion as the host suite.
static void test_canonical_check_value(void) {
    const uint8_t input[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16_ccitt_false(input, sizeof(input)));
}

static void test_empty_input_returns_init_value(void) {
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc16_ccitt_false(NULL, 0));
}

static void test_known_vectors(void) {
    const uint8_t v00[] = {0x00};
    const uint8_t vff[] = {0xFF};
    const uint8_t va5[] = {0xA5};
    const uint8_t v0000[] = {0x00, 0x00};
    const uint8_t v1234[] = {0x01, 0x02, 0x03, 0x04};
    TEST_ASSERT_EQUAL_HEX16(0xE1F0, crc16_ccitt_false(v00, sizeof(v00)));
    TEST_ASSERT_EQUAL_HEX16(0xFF00, crc16_ccitt_false(vff, sizeof(vff)));
    TEST_ASSERT_EQUAL_HEX16(0x04BF, crc16_ccitt_false(va5, sizeof(va5)));
    TEST_ASSERT_EQUAL_HEX16(0x1D0F, crc16_ccitt_false(v0000, sizeof(v0000)));
    TEST_ASSERT_EQUAL_HEX16(0x89C3, crc16_ccitt_false(v1234, sizeof(v1234)));
}

static void test_single_bit_flip_changes_result(void) {
    const uint8_t base[] = {0x01, 0x02, 0x03, 0x04};
    const uint16_t expected = crc16_ccitt_false(base, sizeof(base));
    for (size_t byte = 0; byte < sizeof(base); ++byte) {
        for (int bit = 0; bit < 8; ++bit) {
            uint8_t flipped[sizeof(base)];
            memcpy(flipped, base, sizeof(base));
            flipped[byte] ^= (uint8_t)(1u << bit);
            TEST_ASSERT_NOT_EQUAL(expected, crc16_ccitt_false(flipped, sizeof(flipped)));
        }
    }
}

static void test_length_is_respected(void) {
    const uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0xFF};
    TEST_ASSERT_EQUAL_HEX16(0x89C3, crc16_ccitt_false(data, 4));
    TEST_ASSERT_NOT_EQUAL(0x89C3, crc16_ccitt_false(data, 5));
}

static void test_struct_sizes_match_wire_format(void) {
    TEST_ASSERT_EQUAL_UINT(63, sizeof(hub_protocol::SensorReport));
    TEST_ASSERT_EQUAL_UINT(9, sizeof(hub_protocol::ActuationCommand));
    TEST_ASSERT_EQUAL_UINT(8, sizeof(hub_protocol::AckStatus));
    TEST_ASSERT_TRUE(sizeof(hub_protocol::SensorReport) <= hub_protocol::MAX_PAYLOAD);
}

// The CRC covers type + length + payload — not the start byte, not itself. The hub must never act
// on a corrupted actuation command (Part 3.1), so getting this range right on the firmware side is
// the safety-relevant half of the check.
static void test_crc_covers_type_length_and_payload_only(void) {
    hub_protocol::ActuationCommand cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.brakeRequest = 42;
    cmd.brakeDurationMs = 250;
    cmd.hostTimestampMs = 0x01020304;

    uint8_t frame[hub_protocol::MAX_FRAME_SIZE];
    size_t n = 0;
    frame[n++] = hub_protocol::START_BYTE;
    frame[n++] = (uint8_t)hub_protocol::MessageType::ACTUATION_COMMAND;
    const uint16_t len = (uint16_t)sizeof(cmd);
    frame[n++] = (uint8_t)(len & 0xFF);
    frame[n++] = (uint8_t)(len >> 8);
    memcpy(frame + n, &cmd, sizeof(cmd));
    n += sizeof(cmd);

    const size_t off = hub_protocol::CRC_COVERED_OFFSET;
    const uint16_t expected = crc16_ccitt_false(frame + off, n - off);

    // Start byte is outside the covered range.
    uint8_t other[hub_protocol::MAX_FRAME_SIZE];
    memcpy(other, frame, n);
    other[0] = 0x55;
    TEST_ASSERT_EQUAL_HEX16(expected, crc16_ccitt_false(other + off, n - off));

    // Every covered byte is inside it.
    for (size_t i = off; i < n; ++i) {
        uint8_t corrupted[hub_protocol::MAX_FRAME_SIZE];
        memcpy(corrupted, frame, n);
        corrupted[i] ^= 0x01;
        TEST_ASSERT_NOT_EQUAL(expected, crc16_ccitt_false(corrupted + off, n - off));
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_canonical_check_value);
    RUN_TEST(test_empty_input_returns_init_value);
    RUN_TEST(test_known_vectors);
    RUN_TEST(test_single_bit_flip_changes_result);
    RUN_TEST(test_length_is_respected);
    RUN_TEST(test_struct_sizes_match_wire_format);
    RUN_TEST(test_crc_covers_type_length_and_payload_only);
    return UNITY_END();
}
