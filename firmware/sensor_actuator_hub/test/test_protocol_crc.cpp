// TODO(Part 13.1 / Phase 1): Firmware-side CRC16 + framing test.
//
// Runs the same known input/output vectors as host/test/unit/test_crc16.cpp against the firmware's
// own copy of Crc16.h, confirming identical CRC behaviour on both sides of the link. Part 14
// Phase 1's exit criteria accept either this test or a manual serial round-trip against known byte
// vectors — this file is the automatable half of that.
//
// Filled in during Phase 1.
