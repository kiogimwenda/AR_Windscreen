#pragma once
// CRC-16/CCITT-FALSE — see docs/BUILD_GUIDE.md Part 3.2.
//
// This file exists TWICE and the two copies must be byte-for-byte identical:
//   firmware/sensor_actuator_hub/include/hub/Crc16.h
//   host/include/ar_drive_assist/vehicle/Crc16.h
// When one changes, change the other in the SAME commit. CI's `protocol-sync-check` job
// (Part 13.5) enforces this with a plain `diff`.
//
// ---------------------------------------------------------------------------------------------
// What a CRC is doing here, and what it is not
//
// A CRC is not a hash and not a signature. It is a deliberately cheap check on a noisy channel:
// the USB-CDC link between the host and the hub. Its one job is to make an accidentally corrupted
// frame overwhelmingly unlikely to look valid. It does nothing at all against a deliberate
// attacker, who can simply recompute it — that is not the threat model on a wire inside one
// vehicle.
//
// The mechanism: treat the message as one very long binary number, divide it by a fixed constant
// (the "polynomial"), and send the remainder. The receiver divides the same way and compares. The
// arithmetic is carry-less — addition and subtraction are both XOR — which is why the loop below
// is shifts and XORs rather than an actual division.
//
// The loop, line by line:
//   crc ^= data[i] << 8      bring the next byte into the TOP half of the 16-bit register, because
//                            this variant processes the most-significant bit first
//   for 8 bits:
//     if the top bit is set  shift left, then XOR the polynomial — this is one step of long
//                            division: the divisor "fits", so subtract (XOR) it
//     else                   shift left only — the divisor does not fit, so the quotient bit is 0
//
// Why these specific constants matter, since they look arbitrary:
//   poly 0x1021  is x^16 + x^12 + x^5 + 1. It is chosen, not invented: this polynomial is known to
//                catch ALL single-bit errors, all double-bit errors, all odd numbers of bit
//                errors, and every burst of 16 or fewer consecutive corrupted bits — which is
//                exactly the error shape a serial link produces.
//   init 0xFFFF  rather than 0. Starting from zero would make a message of all-zero bytes produce
//                a CRC of zero, so a frame of leading zeros (or a dead line reading as zeros)
//                would checksum "correctly". Starting from all-ones removes that blind spot. The
//                "FALSE" in CCITT-FALSE refers to this init value, distinguishing it from the
//                other CCITT variant that inits to 0x0000.
//
// Sanity check for anyone reading this later: this variant's published check value is
// crc16_ccitt_false("123456789") == 0x29B1. test_crc16.cpp asserts exactly that, which is how we
// know this implementation is the standard one and not merely self-consistent.
//
// Not table-driven: 8 iterations per byte over a 63-byte payload at 50 Hz is a few thousand cycles
// a second on the hub. A 512-byte lookup table would trade that for flash on an MCU where the
// bit-serial version is already far below the noise floor. Revisit only if profiling in Part 4
// says otherwise, and change BOTH copies if so.
// ---------------------------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>

inline uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}
