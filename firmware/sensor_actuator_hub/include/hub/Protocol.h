#pragma once
// Host <-> hub wire protocol — see docs/BUILD_GUIDE.md Part 3.
//
// This file exists TWICE and the two copies must be byte-for-byte identical:
//   firmware/sensor_actuator_hub/include/hub/Protocol.h
//   host/include/ar_drive_assist/vehicle/HubProtocol.h
// When one changes, change the other in the SAME commit. CI's `protocol-sync-check` job
// (Part 13.5) enforces this with a plain `diff`.
//
// ---------------------------------------------------------------------------------------------
// Why this file is shared source rather than two independent implementations
//
// The two ends of this link are not just different programs, they are different MACHINES: an ARM
// Cortex-M4 at 32 bits on one side, x86-64 on the other, built by two different compilers with two
// different standard libraries. Anything that depends on how a compiler lays out a struct is a
// place where those two can silently disagree — and a silent disagreement here means the hub reads
// a brake intensity out of what the host thought was a timestamp.
//
// Three properties keep that from happening, and all three are load-bearing:
//
//   1. #pragma pack(push, 1) removes padding. By default a compiler inserts padding bytes so that
//      each member sits on its natural alignment — an 8-byte double on an 8-byte boundary, and so
//      on. How much padding, and where, is an ABI decision, so the same struct can be a different
//      size on the two machines. Packing to 1 forces a layout that is purely the sum of the member
//      sizes, identical everywhere. The cost is real but irrelevant here: unaligned member access
//      is slower, and on some ARM configurations taking a POINTER to an unaligned member is
//      undefined. So read these structs by value, never by forming a pointer to a field.
//
//   2. Fixed-width types only. `int` is 32 bits on both of these targets, but nothing in the
//      language guarantees it, and `long` genuinely differs (32-bit on the MCU, 64-bit on Linux).
//      Every field below is uint8_t/uint16_t/uint32_t/float/double, which pin both size and
//      representation.
//
//   3. static_assert on every struct size, at the bottom of this file. This is the part that turns
//      the above from an intention into a guarantee: if either compiler ever lays one of these out
//      differently, the build fails at that assert rather than the link succeeding and the hub
//      misreading commands on a bench. It costs nothing at runtime and it is checked on BOTH
//      sides, because both sides compile this same file.
//
// Endianness is assumed little-endian on both ends. ARM Cortex-M4 and x86-64 are both
// little-endian, so the multi-byte fields below and the length/CRC fields in the frame header can
// be memcpy'd directly with no byte swapping. This IS an assumption, not a fact about the
// protocol, and it is the first thing to revisit if the hub is ever moved to a big-endian part.
// ---------------------------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>

namespace hub_protocol {

// --- Framing (Part 3.1) -----------------------------------------------------------------------
//
//   +-----------+----------+-------------+---------------------+-----------+
//   | startByte | type     | length      | payload (length B)  | crc16     |
//   | 1 byte    | 1 byte   | 2 bytes LE  | variable, max 64 B  | 2 bytes LE|
//   | 0xAA      |          |             |                     |           |
//   +-----------+----------+-------------+---------------------+-----------+
//
// CRC16 is computed over type + length + payload — NOT over the start byte, and not over itself.
// A receiver that gets a bad CRC discards the frame and does not act on it. That rule applies on
// both ends but matters most on the hub, which must never act on a corrupted actuation command.

constexpr uint8_t START_BYTE = 0xAA;
constexpr size_t MAX_PAYLOAD = 64;

// Frame geometry, derived from the diagram above. Both ends need these and they must agree, so
// they live here rather than being recomputed in VehicleInterface and CommsTask independently.
constexpr size_t HEADER_SIZE = 4;  // startByte + type + length(2)
constexpr size_t CRC_SIZE = 2;
constexpr size_t CRC_COVERED_OFFSET = 1;  // CRC starts at `type`, skipping startByte
constexpr size_t MAX_FRAME_SIZE = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE;

enum class MessageType : uint8_t {
    SENSOR_REPORT = 0x01,      // hub -> host
    ACTUATION_COMMAND = 0x02,  // host -> hub
    ACK_STATUS = 0x03,         // hub -> host
    HEARTBEAT = 0x04,          // host -> hub, required at >=10 Hz (see Part 3.4)
};

// HEARTBEAT carries no payload: it is sent with length 0 and exists purely so the hub's watchdog
// can distinguish "the host is alive and has nothing to ask for" from "the link is dead". Staleness
// of an actual request is detected separately, via ActuationCommand::hostTimestampMs.

#pragma pack(push, 1)
struct SensorReport {
    uint32_t timestampMs;
    // GPS
    double latitude;
    double longitude;
    float speedKph;
    uint8_t gpsFixValid;  // 0/1
    // IMU
    float accelX, accelY, accelZ;  // g
    float gyroX, gyroY, gyroZ;     // deg/s
    float headingDeg;
    // OBD-II
    float obdSpeedKph;
    uint16_t obdRpm;
    uint8_t obdBrakePedalActive;  // 0/1, from OBD PID if available, else 0xFF = unknown
    // Gesture
    uint8_t gestureEvent;  // 0=none,1=left,2=right,3=up,4=down,5=hold
    // Hub state
    uint8_t ignitionOn;         // 0/1
    uint8_t killSwitchEngaged;  // 1 = actuator power physically cut
};

struct ActuationCommand {
    uint8_t indicators;        // bit0 left, bit1 right, bit2 hazards
    uint8_t lights;            // bit0 high beam, bit1 horn
    uint8_t brakeRequest;      // 0-255 requested intensity; HUB clamps to configured ceiling
    uint16_t brakeDurationMs;  // hub still enforces its own max regardless of this value
    uint32_t hostTimestampMs;  // used by hub to detect stale/replayed commands
};

struct AckStatus {
    uint32_t timestampMs;
    uint8_t lastCommandAccepted;     // 0/1
    uint8_t actuatorFaultCode;       // 0 = none; see Appendix B for codes
    uint16_t appliedBrakeIntensity;  // what the hub actually applied, post-ceiling
};
#pragma pack(pop)

// --- Layout guarantees ------------------------------------------------------------------------
// See the note at the top of this file. These are compiled on BOTH the hub and the host, so a
// layout disagreement between the two toolchains becomes a build failure rather than a field that
// silently reads the wrong bytes.
static_assert(sizeof(SensorReport) == 63, "SensorReport layout changed - update BOTH copies");
static_assert(sizeof(ActuationCommand) == 9,
              "ActuationCommand layout changed - update BOTH copies");
static_assert(sizeof(AckStatus) == 8, "AckStatus layout changed - update BOTH copies");

// SensorReport is the largest payload and sits 1 byte under the cap. Anything added to it needs
// MAX_PAYLOAD raised on both ends first, or frames will be silently rejected as oversized.
static_assert(sizeof(SensorReport) <= MAX_PAYLOAD, "SensorReport exceeds MAX_PAYLOAD");
static_assert(sizeof(ActuationCommand) <= MAX_PAYLOAD, "ActuationCommand exceeds MAX_PAYLOAD");
static_assert(sizeof(AckStatus) <= MAX_PAYLOAD, "AckStatus exceeds MAX_PAYLOAD");

// Both ends memcpy multi-byte fields directly, which is only correct because both are
// little-endian and use IEEE-754 floats of these exact widths.
static_assert(sizeof(float) == 4, "float is not 4 bytes - wire format assumption broken");
static_assert(sizeof(double) == 8, "double is not 8 bytes - wire format assumption broken");

}  // namespace hub_protocol
