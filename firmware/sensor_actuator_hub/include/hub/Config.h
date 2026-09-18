#pragma once
// TODO(Part 4.2): Pin and peripheral allocation for the STM32F4 hub.
//
// Part 4.2 gives a template pin table; the ACTUAL pins wired on the board go here, because the
// drivers in src/drivers/ must not hardcode pin numbers. Also holds the hub-side safety constants
// (overcurrent threshold, watchdog timeouts) referenced by Part 4.5 and 4.6.
//
// Filled in during Phase 2.
