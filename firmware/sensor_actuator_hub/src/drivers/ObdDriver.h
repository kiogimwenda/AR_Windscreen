#pragma once
// TODO(Part 4.4): ObdDriver — ELM327 over UART3. Polls PID 0x0D (speed) and 0x0C (RPM);
// brakePedalActive via a mode-1 PID if the target vehicle exposes one, else 0xFF = unknown.
//
// Part 4.4's rule for every driver in this directory: a small class with init(), a read/poll
// method, and nothing else. No driver reaches into another driver's state.
//
// Filled in during Phase 2.
