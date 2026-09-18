#pragma once
// TODO(Part 4.4): RelayDriver — Opto-isolated relay board on 4-6 GPIO: indicators, hazards, horn,
// high beam. allOff() is called by WatchdogTask and by ActuationTask on any fault.
//
// Part 4.4's rule for every driver in this directory: a small class with init(), a read/poll
// method, and nothing else. No driver reaches into another driver's state.
//
// Filled in during Phase 2.
