// CRC-16/CCITT-FALSE tests — see docs/BUILD_GUIDE.md Part 13.1, Phase 1.
//
// This file exercises the HOST copy of Crc16.h. The firmware copy is exercised by the matching
// Unity test at firmware/sensor_actuator_hub/test/test_protocol_crc.cpp, which asserts the SAME
// expected values against the same inputs. Between the two of them plus CI's protocol-sync-check
// diff, "identical CRC behaviour on both ends" is covered three ways: the files are byte-identical,
// and each copy is independently confirmed to produce the standard results.
//
// The expected values below are duplicated verbatim in the firmware test on purpose. They are not
// factored into a shared header, because a shared header is one more file that can drift, and the
// whole point here is independent confirmation rather than shared machinery.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "ar_drive_assist/vehicle/Crc16.h"
#include "ar_drive_assist/vehicle/HubProtocol.h"

namespace {

uint16_t crc(const std::vector<uint8_t>& v) {
    return crc16_ccitt_false(v.data(), v.size());
}

}  // namespace

// The published check value for this CRC variant. If this one assertion passes, the implementation
// is the standard CRC-16/CCITT-FALSE and not merely self-consistent — which is the entire reason
// for picking a named variant instead of inventing one.
TEST(Crc16, CanonicalCheckValue) {
    const std::vector<uint8_t> input = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(crc(input), 0x29B1);
}

// Empty input returns the init value untouched. Worth pinning: it is what distinguishes
// CCITT-FALSE (init 0xFFFF) from the other CCITT variant (init 0x0000).
TEST(Crc16, EmptyInputReturnsInitValue) {
    EXPECT_EQ(crc16_ccitt_false(nullptr, 0), 0xFFFF);
}

TEST(Crc16, KnownVectors) {
    EXPECT_EQ(crc({0x00}), 0xE1F0);
    EXPECT_EQ(crc({0xFF}), 0xFF00);
    EXPECT_EQ(crc({0xA5}), 0x04BF);
    EXPECT_EQ(crc({0x00, 0x00}), 0x1D0F);
    EXPECT_EQ(crc({0x01, 0x02, 0x03, 0x04}), 0x89C3);
}

// A CRC that returns a constant, or ignores its input, would pass the vectors above by luck but
// fail here. This is the property the link actually depends on.
TEST(Crc16, SingleBitFlipChangesResult) {
    const std::vector<uint8_t> base = {0x01, 0x02, 0x03, 0x04};
    for (size_t byte = 0; byte < base.size(); ++byte) {
        for (int bit = 0; bit < 8; ++bit) {
            std::vector<uint8_t> flipped = base;
            flipped[byte] ^= static_cast<uint8_t>(1u << bit);
            EXPECT_NE(crc(flipped), crc(base))
                << "flipping byte " << byte << " bit " << bit << " did not change the CRC";
        }
    }
}

// All-zero payloads are the realistic failure mode on a dead or floating line, and are also
// exactly what a zeroed ActuationCommand looks like (Part 3.3). Distinct lengths of zeros must not
// collide, or "link dead" and "no request" become indistinguishable on the hub.
TEST(Crc16, DistinctZeroLengthsDoNotCollide) {
    std::vector<uint16_t> seen;
    for (size_t n = 0; n <= 16; ++n) {
        const std::vector<uint8_t> zeros(n, 0x00);
        const uint16_t c = crc(zeros);
        for (uint16_t prev : seen) EXPECT_NE(c, prev) << "collision at length " << n;
        seen.push_back(c);
    }
}

TEST(Crc16, LengthIsRespected) {
    const std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0xFF};
    EXPECT_EQ(crc16_ccitt_false(data.data(), 4), 0x89C3);  // ignores the trailing 0xFF
    EXPECT_NE(crc16_ccitt_false(data.data(), 5), 0x89C3);
}

// --- Protocol layout --------------------------------------------------------------------------
// HubProtocol.h already static_asserts these, so a layout change is a build failure rather than a
// test failure. Restated here so the numbers appear in test output as documented, checked facts —
// the report's protocol section cites them, and a reader of the test suite should not have to open
// a header to learn what goes on the wire.

TEST(HubProtocol, StructSizesMatchWireFormat) {
    EXPECT_EQ(sizeof(hub_protocol::SensorReport), 63u);
    EXPECT_EQ(sizeof(hub_protocol::ActuationCommand), 9u);
    EXPECT_EQ(sizeof(hub_protocol::AckStatus), 8u);
}

TEST(HubProtocol, PayloadsFitTheFrame) {
    EXPECT_LE(sizeof(hub_protocol::SensorReport), hub_protocol::MAX_PAYLOAD);
    EXPECT_EQ(hub_protocol::MAX_FRAME_SIZE,
              hub_protocol::HEADER_SIZE + hub_protocol::MAX_PAYLOAD + hub_protocol::CRC_SIZE);
}

// Packing removed all padding: the struct is exactly the sum of its members, and the last member
// sits at sizeof - 1. If a compiler ever re-introduces padding, this catches it in a way that
// names the offending field.
TEST(HubProtocol, SensorReportIsFullyPacked) {
    EXPECT_EQ(offsetof(hub_protocol::SensorReport, timestampMs), 0u);
    EXPECT_EQ(offsetof(hub_protocol::SensorReport, latitude), 4u);
    EXPECT_EQ(offsetof(hub_protocol::SensorReport, longitude), 12u);
    EXPECT_EQ(offsetof(hub_protocol::SensorReport, speedKph), 20u);
    EXPECT_EQ(offsetof(hub_protocol::SensorReport, killSwitchEngaged),
              sizeof(hub_protocol::SensorReport) - 1);
}

TEST(HubProtocol, ActuationCommandIsFullyPacked) {
    EXPECT_EQ(offsetof(hub_protocol::ActuationCommand, indicators), 0u);
    EXPECT_EQ(offsetof(hub_protocol::ActuationCommand, brakeRequest), 2u);
    EXPECT_EQ(offsetof(hub_protocol::ActuationCommand, brakeDurationMs), 3u);
    EXPECT_EQ(offsetof(hub_protocol::ActuationCommand, hostTimestampMs), 5u);
}

// --- CRC over a real frame --------------------------------------------------------------------
// The CRC covers type + length + payload, skipping the start byte and itself. Getting that range
// wrong is the single most likely framing bug, and it fails in the worst possible way: both ends
// agree with themselves and disagree with each other only on the bench.

TEST(HubProtocol, CrcCoversTypeLengthAndPayloadOnly) {
    hub_protocol::ActuationCommand cmd{};
    cmd.brakeRequest = 42;
    cmd.brakeDurationMs = 250;
    cmd.hostTimestampMs = 0x01020304;

    std::vector<uint8_t> frame;
    frame.push_back(hub_protocol::START_BYTE);
    frame.push_back(static_cast<uint8_t>(hub_protocol::MessageType::ACTUATION_COMMAND));
    const uint16_t len = sizeof(cmd);
    frame.push_back(static_cast<uint8_t>(len & 0xFF));
    frame.push_back(static_cast<uint8_t>(len >> 8));
    const size_t payloadStart = frame.size();
    frame.resize(payloadStart + sizeof(cmd));
    std::memcpy(frame.data() + payloadStart, &cmd, sizeof(cmd));

    const uint16_t computed = crc16_ccitt_false(frame.data() + hub_protocol::CRC_COVERED_OFFSET,
                                                frame.size() - hub_protocol::CRC_COVERED_OFFSET);

    // Changing the start byte must NOT change the CRC — it is outside the covered range.
    std::vector<uint8_t> otherStart = frame;
    otherStart[0] = 0x55;
    EXPECT_EQ(crc16_ccitt_false(otherStart.data() + hub_protocol::CRC_COVERED_OFFSET,
                                otherStart.size() - hub_protocol::CRC_COVERED_OFFSET),
              computed);

    // Changing any covered byte MUST change it.
    for (size_t i = hub_protocol::CRC_COVERED_OFFSET; i < frame.size(); ++i) {
        std::vector<uint8_t> corrupted = frame;
        corrupted[i] ^= 0x01;
        EXPECT_NE(crc16_ccitt_false(corrupted.data() + hub_protocol::CRC_COVERED_OFFSET,
                                    corrupted.size() - hub_protocol::CRC_COVERED_OFFSET),
                  computed)
            << "corrupting covered byte " << i << " did not change the CRC";
    }
}

// A zeroed ActuationCommand is what the arbiter sends when it has nothing to request (Part 3.3).
// It must produce a valid, stable frame — "no request" is an explicit message, not an absence.
TEST(HubProtocol, ZeroedActuationCommandProducesStableCrc) {
    hub_protocol::ActuationCommand a{};
    hub_protocol::ActuationCommand b{};
    uint8_t bufA[sizeof(a)];
    uint8_t bufB[sizeof(b)];
    std::memcpy(bufA, &a, sizeof(a));
    std::memcpy(bufB, &b, sizeof(b));
    EXPECT_EQ(crc16_ccitt_false(bufA, sizeof(bufA)), crc16_ccitt_false(bufB, sizeof(bufB)));
}
