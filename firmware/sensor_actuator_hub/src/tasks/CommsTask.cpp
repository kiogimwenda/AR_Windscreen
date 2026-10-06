#include "tasks/CommsTask.h"

#include <Arduino.h>

#include "tasks/HubContext.h"

// Every byte from the host goes through the FrameParser. A CRC-valid frame refreshes the link in
// SafetyCore (any type counts, Part 3.4 (1)); a HEARTBEAT re-arms after a disarm; an
// ACTUATION_COMMAND is judged by SafetyCore and answered with an ACK_STATUS (Part 3.3). A bad frame
// counts towards Appendix B fault 4 and is never acted on.
void CommsTask(void*) {
    HubContext& h = hubContext();
    hub::FrameParser parser;
    for (;;) {
        while (Serial.available() > 0) parser.push(static_cast<uint8_t>(Serial.read()));
        for (hub::FrameParser::Result r; (r = parser.next()) != hub::FrameParser::Result::NONE;) {
            const uint32_t now = millis();
            if (r == hub::FrameParser::Result::ERROR) {
                xSemaphoreTake(h.coreMutex, portMAX_DELAY);
                h.core.onFrameError();
                xSemaphoreGive(h.coreMutex);
                continue;
            }
            hub_protocol::ActuationCommand cmd{};
            const bool isCommand =
                parser.type() == hub_protocol::MessageType::ACTUATION_COMMAND && parser.as(cmd);
            xSemaphoreTake(h.coreMutex, portMAX_DELAY);
            h.core.onValidFrame(now, parser.type());
            hub_protocol::AckStatus ack{};
            if (isCommand) ack = h.core.onCommand(now, cmd);
            xSemaphoreGive(h.coreMutex);
            if (isCommand) sendFrame(hub_protocol::MessageType::ACK_STATUS, &ack, sizeof ack);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
