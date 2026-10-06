// ObdDriver over the vehicle's CAN bus (default build). See ObdDriver.h and include/hub/ObdCan.h,
// where every decision lives; this file only moves frames between bxCAN1 and ObdPoller.
//
// Hardware: CAN1 on PB8/PB9 (Config.h) -> 3.3 V logic down the inter-box cable -> SN65HVD230 on
// the power board -> OBD-II pins 6 (CAN_H) and 14 (CAN_L). The transceiver's Rs pin is held HIGH
// (standby: cannot drive the bus) except while the poller is allowed to transmit.
//
// Controller settings that follow from ObdCan.h's rule one (never disturb the car's bus):
//   - silent mode while listening: the controller neither transmits nor acknowledges;
//   - automatic retransmission OFF: a failed request is not repeated against the bus;
//   - automatic bus-off recovery OFF: if the controller ever went bus-off it stays off until
//     the poller reconfigures it.
// Errors are read from the error status register's last-error code (LEC), which the driver
// resets to 7 ("no change") after every look.
#ifndef HUB_OBD_ELM327
#include <Arduino.h>

#include <cstring>

#include "drivers/ObdDriver.h"
#include "hub/Config.h"
#include "hub/ObdCan.h"

namespace {
CAN_HandleTypeDef hcan;
hub::obd::ObdPoller poller;
bool running = false;

void armLec() {
    hcan.Instance->ESR |= CAN_ESR_LEC;
}  // LEC = 7: "no error since last look"

bool errorSinceLastLook() {
    const uint32_t lec = (hcan.Instance->ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos;
    armLec();
    return lec != 0 && lec != 7;
}

bool configure(const hub::obd::BusConfig& c, bool silent) {
    digitalWrite(HUB_PIN_CAN_SILENT, HIGH);  // transceiver silent while reconfiguring
    if (running) {
        HAL_CAN_Stop(&hcan);
        HAL_CAN_DeInit(&hcan);
        running = false;
    }
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = hub_config::kApb1Hz / (c.bitrate * hub_config::kCanTqPerBit);
    hcan.Init.Mode = silent ? CAN_MODE_SILENT : CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK) return false;

    // Listening: accept everything (any traffic proves the bit rate). Probing and locked: only ECU
    // responses, so the 3-deep FIFO is not flooded by the car's own traffic between our polls.
    CAN_FilterTypeDef f{};
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation = ENABLE;
    f.SlaveStartFilterBank = 14;
    if (!silent) {
        // 32-bit filter register layout: STID[31:21] or EXID[31:3], IDE bit 2, RTR bit 1.
        uint32_t id, mask;
        if (c.extended) {
            id = (hub::obd::kResponse29Base << 3) | CAN_ID_EXT;
            mask = (0x1FFFFF00u << 3) | CAN_ID_EXT;
        } else {
            id = hub::obd::kResponse11First << 21;
            mask = (0x7F8u << 21) | CAN_ID_EXT;  // 0x7E8..0x7EF, standard IDs only
        }
        f.FilterIdHigh = id >> 16;
        f.FilterIdLow = id & 0xFFFF;
        f.FilterMaskIdHigh = mask >> 16;
        f.FilterMaskIdLow = mask & 0xFFFF;
    }
    if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK) return false;
    if (HAL_CAN_Start(&hcan) != HAL_OK) return false;
    running = true;
    armLec();
    if (!silent) digitalWrite(HUB_PIN_CAN_SILENT, LOW);
    return true;
}

void transmit(const hub::obd::CanFrame& fr) {
    CAN_TxHeaderTypeDef h{};
    h.IDE = fr.extended ? CAN_ID_EXT : CAN_ID_STD;
    h.StdId = fr.extended ? 0 : fr.id;
    h.ExtId = fr.extended ? fr.id : 0;
    h.RTR = CAN_RTR_DATA;
    h.DLC = fr.len;
    h.TransmitGlobalTime = DISABLE;
    uint32_t mailbox = 0;
    uint8_t data[8];
    std::memcpy(data, fr.data, sizeof data);
    HAL_CAN_AddTxMessage(&hcan, &h, data, &mailbox);
}
}  // namespace

bool ObdDriver::init() {
    pinMode(HUB_PIN_CAN_SILENT, OUTPUT);
    digitalWrite(HUB_PIN_CAN_SILENT, HIGH);
    __HAL_RCC_CAN1_CLK_ENABLE();
    pinmap_pinout(digitalPinToPinName(HUB_CAN1_RX), PinMap_CAN_RD);
    pinmap_pinout(digitalPinToPinName(HUB_CAN1_TX), PinMap_CAN_TD);
    return true;
}

bool ObdDriver::read(Reading& out) {
    const uint32_t now = millis();
    if (running) {
        CAN_RxHeaderTypeDef h;
        uint8_t d[8];
        while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0 &&
               HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &h, d) == HAL_OK) {
            hub::obd::CanFrame f;
            f.extended = h.IDE == CAN_ID_EXT;
            f.id = f.extended ? h.ExtId : h.StdId;
            f.len = static_cast<uint8_t>(h.DLC > 8 ? 8 : h.DLC);
            std::memcpy(f.data, d, f.len);
            poller.onFrame(f, now);
        }
        if (errorSinceLastLook()) poller.onBusError();
    }
    const hub::obd::Action a = poller.step(now);
    if (a.configure && !configure(a.config, a.silent)) poller.onBusError();
    if (a.send && running) transmit(a.frame);
    const hub::obd::Reading r = poller.reading(now);
    out.speedKph = r.speedKph;
    out.rpm = r.rpm;
    out.brakePedalActive = 0xFF;
    out.valid = r.valid;
    return r.valid;
}
#endif  // !HUB_OBD_ELM327
