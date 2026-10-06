// ObdDriver, ELM327 fallback — built only with -DHUB_OBD_ELM327 (env:hub_elm327), for a pre-CAN
// vehicle. See ObdDriver.h. The ELM327 sits on USART3 (PC10/PC11, Config.h).
#ifdef HUB_OBD_ELM327
#include <Arduino.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "drivers/ObdDriver.h"
#include "hub/Config.h"

namespace {
Uart obd(HUB_OBD_RX,
         HUB_OBD_TX);  // STM32duino's concrete UART class (HardwareSerial is the abstract API)
constexpr uint32_t kBaud = 38400;  // most ELM327 clones
constexpr uint32_t kReplyTimeoutMs = 500;
constexpr uint32_t kInitTimeoutMs = 2000;  // ATZ (a full reset) takes about a second
const char* const kInit[] = {"ATZ", "ATE0", "ATL0", "ATS1",
                             "ATSP0"};  // reset, no echo/linefeeds, auto protocol

// One adapter, one driver instance: its state lives here rather than in the shared header.
void send(const char* cmd);
bool parse(const char* line);
enum class Step { INIT, QUERY_SPEED, WAIT_SPEED, QUERY_RPM, WAIT_RPM } step_ = Step::INIT;
int initIndex_ = 0;
char line_[64] = {};
int lineLen_ = 0;
uint32_t sentMs_ = 0;
ObdDriver::Reading r_;
}  // namespace

bool ObdDriver::init() {
    obd.begin(kBaud);
    step_ = Step::INIT;
    initIndex_ = 0;
    return true;
}

namespace {
void send(const char* cmd) {
    obd.print(cmd);
    obd.print('\r');
    sentMs_ = millis();
    lineLen_ = 0;
}

// "41 0D 32" -> 50 km/h; "41 0C 1A F8" -> (0x1A * 256 + 0xF8) / 4 = 1726 rpm.
bool parse(const char* s) {
    unsigned a = 0, b = 0, c = 0, d = 0;
    const int n = ::sscanf(s, "%x %x %x %x", &a, &b, &c, &d);
    if (n < 3 || a != 0x41) return false;
    if (b == 0x0D) {
        r_.speedKph = static_cast<float>(c);
        r_.valid = true;
        return true;
    }
    if (b == 0x0C && n == 4) {
        r_.rpm = static_cast<uint16_t>((c * 256 + d) / 4);
        return true;
    }
    return false;
}
}  // namespace

bool ObdDriver::read(Reading& out) {
    bool updated = false;
    const bool waiting =
        step_ == Step::WAIT_SPEED || step_ == Step::WAIT_RPM || step_ == Step::INIT;
    while (obd.available() > 0) {
        const char ch = static_cast<char>(obd.read());
        if (ch == '>') {  // the ELM's prompt: the reply is complete
            line_[lineLen_] = '\0';
            if (step_ == Step::INIT) {
                if (++initIndex_ >= static_cast<int>(sizeof(kInit) / sizeof(kInit[0])))
                    step_ = Step::QUERY_SPEED;
                else
                    send(kInit[initIndex_]);
            } else {
                updated |= parse(line_);
                step_ = step_ == Step::WAIT_SPEED ? Step::QUERY_RPM : Step::QUERY_SPEED;
            }
            lineLen_ = 0;
        } else if (ch != '\r' && ch != '\n' && lineLen_ < static_cast<int>(sizeof(line_)) - 1) {
            line_[lineLen_++] = ch;
        }
    }
    if (step_ == Step::INIT && initIndex_ == 0 && sentMs_ == 0) send(kInit[0]);
    const uint32_t limit = step_ == Step::INIT ? kInitTimeoutMs : kReplyTimeoutMs;
    if (waiting && millis() - sentMs_ > limit) {  // no answer: move on, mark stale
        r_.valid = false;
        step_ = step_ == Step::INIT ? Step::INIT : Step::QUERY_SPEED;
        if (step_ == Step::INIT) send(kInit[initIndex_]);
    }
    if (step_ == Step::QUERY_SPEED) {
        send("010D");
        step_ = Step::WAIT_SPEED;
    } else if (step_ == Step::QUERY_RPM) {
        send("010C");
        step_ = Step::WAIT_RPM;
    }
    out = r_;
    (void)updated;
    return r_.valid;
}
#endif  // HUB_OBD_ELM327
