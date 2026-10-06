#include "tasks/SensorTask.h"

#include <Arduino.h>

#include <cmath>

#include "drivers/GestureDriver.h"
#include "drivers/GpsDriver.h"
#include "drivers/ImuDriver.h"
#include "drivers/ObdDriver.h"
#include "drivers/SafetyInputs.h"
#include "tasks/HubContext.h"
#ifdef HUB_BENCH_TELEMETRY
#include "drivers/LoadCell.h"
#endif

// Part 3.3: exactly every 20 ms (vTaskDelayUntil, so the period does not drift with the work done),
// because the host's EKF assumes that dt. Each report sent is reported to SafetyCore: Part 3.4
// (2)'s self-check, which disarms the hub if this loop ever stalls for more than 200 ms. The bench
// build also sends BENCH_TELEMETRY (actuator current, applied duty, load cell) at the same rate,
// for tools/bench_rig/log_actuator_bench.py (Part 13.3).
namespace {
constexpr uint32_t kGpsFreshMs = 1500;  // a fix older than this is reported as not valid
}  // namespace

void SensorTask(void*) {
    HubContext& h = hubContext();
    ImuDriver imu;
    GpsDriver gps;
    ObdDriver obd;
    GestureDriver gesture;
    imu.init();
    gps.init();
    obd.init();
    gesture.init();
#ifdef HUB_BENCH_TELEMETRY
    LoadCell cell;
    cell.init();
    cell.tare();
    float newtons = 0;
    bool cellValid = false;
#endif
    ImuDriver::Sample s;
    GpsDriver::Fix fix;
    ObdDriver::Reading od;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        imu.read(s);
        gps.read(fix);
        obd.read(od);
        hub_protocol::SensorReport r{};
        r.timestampMs = millis();
        r.latitude = fix.lat;
        r.longitude = fix.lon;
        r.speedKph = fix.speedKph;
        // `fix` keeps the last fix between NMEA sentences (and forever if the GNSS goes quiet, in a
        // tunnel or with a broken wire). Valid only while fresh, so the host never fuses a stale
        // repeat as a new measurement.
        r.gpsFixValid = (fix.valid && millis() - fix.timestampMs < kGpsFreshMs) ? 1 : 0;
        r.accelX = s.ax;
        r.accelY = s.ay;
        r.accelZ = s.az;
        r.gyroX = s.gx;
        r.gyroY = s.gy;
        r.gyroZ = s.gz;
        r.headingDeg = s.headingDeg;
        r.obdSpeedKph = od.valid ? od.speedKph : NAN;  // 0 would read as "stopped" to the EKF
        r.obdRpm = od.rpm;
        // Brake pedal: the brake-light switch where fitted (BOM note C), else the OBD value
        // (usually 0xFF, unknown). The optocoupler line reads "released" when not fitted:
        // HUB_BRAKE_LIGHT_FITTED must be defined only on a hub that actually has it wired.
#ifdef HUB_BRAKE_LIGHT_FITTED
        r.obdBrakePedalActive = safety_inputs::brakePedalPressed() ? 1 : 0;
#else
        r.obdBrakePedalActive = od.brakePedalActive;
#endif
        r.gestureEvent = static_cast<uint8_t>(gesture.poll());
        // The power box runs from the switched ACC circuit; its presence line reads "present" only
        // while it is powered (Part 4.8), so it doubles as the ignition signal.
        r.ignitionOn = safety_inputs::powerBoxPresent() ? 1 : 0;
        r.killSwitchEngaged = safety_inputs::killSwitchEngaged() ? 1 : 0;
        sendFrame(hub_protocol::MessageType::SENSOR_REPORT, &r, sizeof r);
        xSemaphoreTake(h.coreMutex, portMAX_DELAY);
        h.core.onReportSent(r.timestampMs);
        const bool armed = h.core.armed(r.timestampMs);
        const uint8_t fault = h.core.faultCode(r.timestampMs);
        xSemaphoreGive(h.coreMutex);
#ifdef HUB_BENCH_TELEMETRY
        float n;
        if (cell.read(n)) {
            newtons = n;
            cellValid = true;
        }
        hub_protocol::BenchTelemetry b{};
        b.timestampMs = r.timestampMs;
        b.currentAmps = h.brake.readCurrentAmps();
        b.appliedIntensity = h.brake.appliedDuty();
        b.cableMagnetOn = h.brake.cableMagnetOn() ? 1 : 0;
        b.armed = armed ? 1 : 0;
        b.faultCode = fault;
        b.loadCellNewtons = newtons;
        b.loadCellValid = cellValid ? 1 : 0;
        b.killSwitchEngaged = r.killSwitchEngaged;
        b.lastLossReleaseMs = h.lastLossReleaseMs;
        b.lastLossCurrentZeroMs = h.lastLossCurrentZeroMs;
        sendFrame(hub_protocol::MessageType::BENCH_TELEMETRY, &b, sizeof b);
#else
        (void)armed;
        (void)fault;
#endif
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(hub_config::kSensorReportPeriodMs));
    }
}
