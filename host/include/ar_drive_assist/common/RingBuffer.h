#pragma once
// TODO(Part 5.3): RingBuffer<T, N> — single-producer/single-consumer lock-free-ish message bus
// primitive. Deliberately not a general pub/sub framework: the set of subsystems is fixed and
// known. Overflow policy differs per bus — frameBus and lidarBus drop oldest (staleness is worse
// than a gap), while a dropped ActuationRequest is logged as an EventLog fault because it is
// safety-relevant.
//
// Filled in during Phase 3.
