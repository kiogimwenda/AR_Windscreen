#!/usr/bin/env python3
"""TODO(Part 13.3): Actuator bench-test instrumentation.

Drives a range of brakeRequest intensities at the hub and logs, per step, the applied force, the
measured current from BrakeActuatorDriver::readCurrentAmps(), the response time, and the hub's own
AckStatus.appliedBrakeIntensity (i.e. what the hub applied AFTER its internal ceiling, not what was
asked for). That last field is the point of the exercise: it is the evidence that the hub's ceiling
is real and independent of the host.

This runs against a bench rig with the actuator and a pedal fixture mounted on a bench, NOT in a
vehicle. Its output is the raw data behind Part 13.3's checklist and the report's actuation
evidence (Part 16.2).

The single most important check in this project — a person pushing back on the pedal by foot while
the actuator is engaged at maximum configured intensity and being able to move it — is a PHYSICAL
check performed by a person. This script records the surrounding telemetry; it cannot pass that
check on its own.

Filled in during Phase 12.
"""
