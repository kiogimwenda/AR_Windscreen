// hub_sim — run a software hub on a pseudo-terminal and point the host at it. See HubSim.h.
//
//     build/host/hub_sim                       # prints e.g. "hub on /dev/pts/7"
//     build/host/hub_sim --replay <recording>/hub.csv   # a recording's reports (Recording.h)
//     build/host/ar_drive_assist host/config logs/session.log --serial /dev/pts/7
//
// The hub here is the firmware's own SafetyCore and frame parser: the host arms it with its
// heartbeats, its commands are judged exactly as the STM32 will judge them, and stopping the host
// trips the same 200 ms watchdog. One status line a second; type a command and Enter to change
// the simulated hardware:  k = toggle the kill switch,  b = toggle the power box,  q = quit.

#include <poll.h>
#include <unistd.h>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include "HubSim.h"

namespace {
std::atomic<bool> gStop{false};
void onSignal(int) {
    gStop = true;
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    ar_drive_assist::sim::HubSim hub;
    if (argc == 3 && std::string(argv[1]) == "--replay") {
        hub.setReplay(ar_drive_assist::sim::loadReplayReports(argv[2]));
        std::printf("replaying %s from the host's first frame\n", argv[2]);
    } else if (argc != 1) {
        std::fprintf(stderr, "usage: hub_sim [--replay <recording>/hub.csv]\n");
        return 2;
    }
    hub.start();
    std::printf("hub on %s  (k: kill switch, b: power box, q: quit)\n", hub.devicePath().c_str());
    std::fflush(stdout);
    bool kill = false, box = true, stdinOpen = true;
    int tick = 0;
    while (!gStop) {
        pollfd p{stdinOpen ? STDIN_FILENO : -1, POLLIN, 0};  // fd -1: poll just sleeps
        if (::poll(&p, 1, 100) > 0 && (p.revents & (POLLIN | POLLHUP))) {
            char c = 0;
            if (::read(STDIN_FILENO, &c, 1) <= 0) {
                stdinOpen = false;  // no terminal (run in the background): Ctrl+C / SIGTERM only
                continue;
            }
            if (c == 'q') break;
            if (c == 'k') hub.setKillSwitch(kill = !kill);
            if (c == 'b') hub.setPowerBoxPresent(box = !box);
        }
        if (++tick % 10 == 0) {
            const auto o = hub.outputs();
            std::printf(
                "armed=%d brake=%3u peak=%3u magnet=%d indicators=%u lights=%u fault=%u | "
                "frames=%llu "
                "errors=%llu commands=%llu reports=%llu | kill=%d box=%d\n",
                o.armed, o.brakeIntensity, o.peakBrake, o.cableMagnet, o.indicators, o.lights,
                o.faultCode, static_cast<unsigned long long>(hub.framesReceived()),
                static_cast<unsigned long long>(hub.frameErrors()),
                static_cast<unsigned long long>(hub.commandsReceived()),
                static_cast<unsigned long long>(hub.reportsSent()), kill, box);
            std::fflush(stdout);
        }
    }
    hub.stop();
    std::printf("hub_sim: peak brake intensity applied %u\n", hub.outputs().peakBrake);
    return 0;
}
