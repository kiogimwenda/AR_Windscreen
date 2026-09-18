#pragma once
// TODO(Part 3.1): Shared host<->hub wire protocol — framing, MessageType, and the
// SensorReport / ActuationCommand / AckStatus payload structs.
//
// This file is implemented TWICE and the two copies must be byte-for-byte identical:
//   firmware/sensor_actuator_hub/include/hub/Protocol.h
//   host/include/ar_drive_assist/vehicle/HubProtocol.h
// When one changes, change the other in the same commit. CI's `protocol-sync-check` job
// (Part 13.5) enforces this with a plain `diff`.
//
// Filled in during Phase 1.
