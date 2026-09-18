#pragma once
// TODO(Part 3.2): CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type + length + payload.
//
// This file is implemented TWICE and the two copies must be byte-for-byte identical:
//   firmware/sensor_actuator_hub/include/hub/Crc16.h
//   host/include/ar_drive_assist/vehicle/Crc16.h
// When one changes, change the other in the same commit. CI's `protocol-sync-check` job
// (Part 13.5) enforces this with a plain `diff`.
//
// Filled in during Phase 1.
