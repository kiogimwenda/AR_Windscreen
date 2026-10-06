#pragma once
// Config — pin allocation and the hub's safety constants. See docs/BUILD_GUIDE.md Part 4.2, 4.5,
// 4.6, 4.8 and Appendix B.
//
// ---------------------------------------------------------------------------------------------
// Board (amended 2026-09-30, the two-box hub, Part 4.8): an STM32F405RGT6 (LQFP64, 168 MHz, two
// bxCAN controllers). It sits on the custom POD board in the windscreen pod, behind the mirror,
// with the camera, the BNO085 and the GNSS receiver. Bring-up before the PCB exists uses any
// F405RG development board with an 8 MHz crystal (BOM 1.1), wired to the same pins.
//
// The actuation hardware is NOT on this board. The BTS7960, the ACS712, the cable magnet's MOSFET,
// the signal relays, the kill relay and the brake-light optocoupler are on the POWER board in the
// under-dash power box, at the far end of the 2 m inter-box cable (Part 4.8.3, pin by pin).
// Consequences for every output and input below:
//   - Every output that reaches the power box is FAIL-SAFE BY HARDWARE: the power board buffers
//     it (74HCT244, whose TTL thresholds also read 3.3 V logic reliably, replacing the old level
//     shifters) with a 10 k pull-down on the buffer input. MCU in reset, MCU unpowered, or the
//     cable unplugged: every line reads LOW = brake released, magnet off, relays off. Relays are
//     therefore ACTIVE-HIGH now (the old relay module was active-low).
//   - Every input from the power box reads FAIL-SAFE through the pod's pull-ups: kill sense HIGH =
//     engaged, brake light HIGH = not pressed, box-present HIGH = absent.
//   - The current-sense line is analogue over 2 m. SafetyCore ignores it whenever the box is absent
//     (it would read ~0 V = ~25 A and latch a false overcurrent).
//
// Pins are chosen from the F405's peripheral map (STM32duino variant F405RGT_F415RGT) and are the
// PCB's netlist: change one here, change the schematic. Drivers take their pins from here and
// never hard-code them. Kept free on purpose: PA13/PA14 (SWD), PA11/PA12 (USB), PH0/PH1 (8 MHz
// crystal), PB2 (BOOT1), PC14/PC15 (no LSE fitted).
// ---------------------------------------------------------------------------------------------

#include <cstdint>

namespace hub_config {

// --- Pins (Arduino-STM32 names) -----------------------------------------------------------------
// Brake actuator (power box), BTS7960 half-bridges. RPWM drives the actuator in the PULL direction
// (applying the brake through the pull-only cable); LPWM would drive it back and is only ever held
// low. R_EN and L_EN are tied together on the power board: one enable line. PA8 = TIM1_CH1.
#define HUB_PIN_BRAKE_RPWM PA8
#define HUB_PIN_BRAKE_LPWM PC6
#define HUB_PIN_BRAKE_EN PB12
// Actuator current: the ACS712-20A on the power board, its output divided x2/3 THERE (so nothing
// above 3.3 V ever travels up the cable), into ADC1 channel 0.
#define HUB_PIN_BRAKE_CURRENT PA0
// Fail-safe cable release (BOM note B, subject to the actuator design being approved): the 12 V
// holding electromagnet's MOSFET on the power board, powered through the kill relay. LOW = the
// cable drops free.
#define HUB_PIN_CABLE_MAGNET PB14
// Kill-switch SENSE: the E-stop's second contact block, NC to ground at the power box
// (INPUT_PULLUP here). Read-only: the E-stop cuts actuator power in hardware.
#define HUB_PIN_KILL_SENSE PB15
// Power-box presence: pulled LOW inside the power box by a small MOSFET whose gate is fed from the
// box's own 5 V rail, pulled up here. LOW = the cable is in AND the box is powered (the ignition is
// on). Unplugged, broken, or box unpowered: HIGH = absent. An unplugged cable is also seen by the
// kill sense opening: two independent indications.
#define HUB_PIN_BOX_PRESENT PC0
// Signal relays on the power board (automotive relays, low-side MOSFETs, ACTIVE-HIGH): left, right
// indicators; hazards; horn; high beam.
#define HUB_PIN_RELAY_LEFT PB0
#define HUB_PIN_RELAY_RIGHT PB1
#define HUB_PIN_RELAY_HAZARD PC1
#define HUB_PIN_RELAY_HORN PC2
#define HUB_PIN_RELAY_BEAM PC3
// Brake-light switch via the power board's PC817 optocoupler (BOM note C): LOW = pedal pressed.
#define HUB_PIN_BRAKE_LIGHT_SENSE PB5
// Bench rig only: HX711 load-cell amplifier, reached through the cable's two bench conductors.
#define HUB_PIN_HX711_DOUT PA1
#define HUB_PIN_HX711_SCK PA4
// OBD-II over the vehicle's own CAN bus (ISO 15765-4), replacing the ELM327 + HC-05 pair:
//   CAN1 PB8 RX, PB9 TX, as 3.3 V logic down the inter-box cable to an SN65HVD230 transceiver on
//   the POWER board, beside the OBD-II cable (J1962 pins 6 CAN_H, 14 CAN_L). The transceiver sits
//   there, not in the pod, so the unterminated branch this hub adds to the car's bus is the short
//   OBD lead (~0.5 m), not 2.5 m of cable: a long stub reflects on a 500 kbit/s bus.
//   Its Rs pin is pulled HIGH on the power board (standby: driver off, receiver listening), and
//   its D input pulled HIGH (recessive). The firmware drives Rs LOW only while it may transmit.
//   Cable out, pod in reset or unpowered: the transceiver cannot drive the car's bus.
#define HUB_CAN1_RX PB8
#define HUB_CAN1_TX PB9
#define HUB_PIN_CAN_SILENT PC12
// Fallback for pre-CAN vehicles (Part 4.8.5): an ELM327 on USART3 at PC10 TX / PC11 RX, selected
// with -DHUB_OBD_ELM327. Only one OBD driver is ever built.
#define HUB_OBD_TX PC10
#define HUB_OBD_RX PC11
// Buses on the pod board:
//   I2C1   PB6 SCL, PB7 SDA      BNO085 IMU, on the pod board next to the camera
//   I2C2   PB10 SCL, PB11 SDA    APDS-9960 gesture puck, on its own cable (~1.5 m, 100 kHz): its
//                                own bus, so a cable fault cannot hang the IMU
//   USART2 PA2 TX, PA3 RX        u-blox NEO-M8N (antenna on the pod's top face, or a roof antenna)
//   USB    PA11, PA12            USB-CDC to the host (the Part 3 protocol)
#define HUB_IMU_SDA PB7
#define HUB_IMU_SCL PB6
#define HUB_GESTURE_SDA PB11
#define HUB_GESTURE_SCL PB10
#define HUB_GPS_TX PA2
#define HUB_GPS_RX PA3
// Status LEDs on the pod's driver-facing edge (active-high): host link, armed, fault.
#define HUB_PIN_LED_LINK PC13
#define HUB_PIN_LED_ARMED PC8
#define HUB_PIN_LED_FAULT PC9

// --- Clock
// ---------------------------------------------------------------------------------------- USB full
// speed needs 48 MHz within +-0.25 %; the F405's internal RC oscillator is only ~1 %, and it has no
// crystal-less USB clock recovery. The pod board carries an 8 MHz crystal (src/SystemClock.cpp: HSE
// 8 MHz -> 168 MHz core, 48 MHz USB, 42 MHz APB1 for bxCAN).
constexpr uint32_t kHseHz = 8000000;
constexpr uint32_t kApb1Hz = 42000000;

// --- OBD-II over CAN (ISO 15765-4)
// ---------------------------------------------------------------- Bit timing at APB1 = 42 MHz: 14
// time quanta per bit (1 sync + 11 + 2), sample point 85.7 %.
constexpr uint32_t kCanTqPerBit = 14;
constexpr uint32_t kCanBs1Tq = 11;
constexpr uint32_t kCanBs2Tq = 2;
static_assert(1 + kCanBs1Tq + kCanBs2Tq == kCanTqPerBit, "bit timing must add up");
static_assert(kApb1Hz % (500000 * kCanTqPerBit) == 0 && kApb1Hz % (250000 * kCanTqPerBit) == 0,
              "500 and 250 kbit/s must divide exactly");

// --- Timing (Part 3.3, 3.4, 4.3) ----------------------------------------------------------------
constexpr uint32_t kSensorReportPeriodMs = 20;  // 50 Hz, fixed (the host EKF assumes this dt)
constexpr uint32_t kLinkTimeoutMs = 200;        // no valid host frame for longer: link lost
constexpr uint32_t kReportStallMs = 200;        // report loop silent for longer: self-check fails
constexpr uint32_t kActuationPeriodMs = 10;     // ActuationTask
constexpr uint32_t kWatchdogPeriodMs = 20;      // WatchdogTask
constexpr uint32_t kIwdgTimeoutMs = 500;        // hardware backstop above the 200 ms software one

// --- Brake actuator ------------------------------------------------------------------------------
// The SECOND, independent force ceiling (the host arbiter's is the first). Starts at 90, finalised
// by the Part 13.3 bench test, NEVER raised without a full bench re-test. host/test/unit/
// test_decision_arbiter.cpp cross-checks decision_thresholds.yaml against the value in
// BrakeActuatorDriver.h, which takes it from here.
constexpr uint8_t kMaxSafeIntensity = 90;
// The hub's own limit on how long one brake request may hold, whatever the host asks.
constexpr uint16_t kMaxBrakeDurationMs = 1500;
// PWM: 20 kHz, above hearing, within the BTS7960's 25 kHz limit.
constexpr uint32_t kBrakePwmHz = 20000;
// Overcurrent: above this for `kOvercurrentTicks` consecutive ActuationTask ticks (so a motor's
// inrush spike does not trip it) latches fault 1 until a power cycle. PLACEHOLDER, set from the
// chosen actuator's stall current on the bench (Part 13.3).
constexpr float kOvercurrentAmps = 8.0f;
constexpr int kOvercurrentTicks = 3;  // 30 ms
// ACS712-20A: 2.5 V at 0 A, 100 mV/A, 5 V supply, read through a x2/3 divider into a 12-bit ADC
// at 3.3 V. Calibrate the zero offset at bring-up (it varies part to part).
constexpr float kAdcVref = 3.3f;
constexpr int kAdcMax = 4095;
constexpr float kCurrentDivider = 2.0f / 3.0f;
constexpr float kAcs712ZeroV = 2.5f;
constexpr float kAcs712VoltsPerAmp = 0.100f;

// --- Link integrity (Appendix B fault 4) ---------------------------------------------------------
// Over the last kFrameWindow frames, more than kFrameErrorsMax bad ones disarms actuation, which
// stays disarmed until kCleanFramesToRecover consecutive good frames arrive.
constexpr int kFrameWindow = 50;
constexpr int kFrameErrorsMax = 10;  // 20%
constexpr int kCleanFramesToRecover = 20;

}  // namespace hub_config
