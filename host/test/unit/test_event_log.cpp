// EventLog unit tests — see docs/BUILD_GUIDE.md Part 11.6.
//
// Part 11.6's requirement is that the log "survive a crash, not just a clean shutdown". The most
// important test below therefore checks exactly that: a child process writes a line and then
// dies via abort(), with no destructor and no cleanup, and the parent confirms the line is on
// disk.

#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ar_drive_assist/system/EventLog.h"

using ar_drive_assist::ActuationRequest;
using ar_drive_assist::EventLog;
using ar_drive_assist::RequestType;

namespace {

std::string tempPath(const std::string& name) {
    std::string p = testing::TempDir() + "eventlog_" + name + "_" + std::to_string(::getpid());
    std::remove(p.c_str());
    return p;
}

std::vector<std::string> readLines(const std::string& path) {
    std::ifstream in(path);
    std::vector<std::string> lines;
    for (std::string l; std::getline(in, l);) lines.push_back(l);
    return lines;
}

}  // namespace

TEST(EventLog, FirstLineIsSessionStartWithWallClock) {
    const auto path = tempPath("start");
    {
        EventLog log(path);
    }
    const auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_TRUE(std::regex_search(
        lines[0],
        std::regex(R"(^\d+\.\d{3} #0 SESSION_START wall_utc=\d{4}-\d\d-\d\dT\S+Z pid=)")));
    EXPECT_NE(lines[1].find("#1 SESSION_END"), std::string::npos);
}

// Each line must be readable from the file while the EventLog is still open, which proves no
// userspace buffer is holding it back.
TEST(EventLog, LinesAreVisibleBeforeTheLogIsClosed) {
    const auto path = tempPath("visible");
    EventLog log(path);
    log.logGeneral("hello");
    auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_NE(lines[1].find("INFO msg=\"hello\""), std::string::npos);

    log.logFault("VehicleInterface", "bus full");
    lines = readLines(path);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_NE(lines[2].find("FAULT component=\"VehicleInterface\" desc=\"bus full\""),
              std::string::npos);
}

TEST(EventLog, ActuationRequestLineCarriesEveryField) {
    const auto path = tempPath("request");
    EventLog log(path);
    ActuationRequest req;
    req.timestamp_ms = 123456;
    req.type = RequestType::RequestType_BRAKE;
    req.intensity = 60;
    req.reason_code = 1;
    log.logActuationRequest(req, "ttc=1.4s lead_track=7");

    const auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_NE(lines[1].find("ACTUATION_REQUEST type=BRAKE intensity=60 reason=1 req_ts=123456 "
                            "ctx=\"ttc=1.4s lead_track=7\""),
              std::string::npos);
}

TEST(EventLog, AckStatusLineNamesTheFaultCode) {
    const auto path = tempPath("ack");
    EventLog log(path);
    hub_protocol::AckStatus ack{};
    ack.timestampMs = 999;
    ack.lastCommandAccepted = 0;
    ack.actuatorFaultCode = 2;
    ack.appliedBrakeIntensity = 0;
    log.logAckStatus(ack);
    ack.actuatorFaultCode = 200;
    log.logAckStatus(ack);

    const auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_NE(lines[1].find("ACK_STATUS hub_ts=999 accepted=0 fault=2 fault_name=KILL_SWITCH "
                            "applied=0"),
              std::string::npos);
    // An unknown code is still logged, with its number, rather than dropped.
    EXPECT_NE(lines[2].find("fault=200 fault_name=UNKNOWN"), std::string::npos);
}

TEST(EventLog, NewlinesAndQuotesCannotSplitOrBreakALine) {
    const auto path = tempPath("escape");
    EventLog log(path);
    log.logFault("x", "line one\nline two \"quoted\" back\\slash");
    const auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_NE(lines[1].find(R"(desc="line one\nline two \"quoted\" back\\slash")"),
              std::string::npos);
}

TEST(EventLog, AppendsAcrossSessionsInsteadOfOverwriting) {
    const auto path = tempPath("append");
    {
        EventLog log(path);
        log.logGeneral("first session");
    }
    {
        EventLog log(path);
        log.logGeneral("second session");
    }
    const auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 6u);  // START, INFO, END, twice
    EXPECT_NE(lines[1].find("first session"), std::string::npos);
    EXPECT_NE(lines[4].find("second session"), std::string::npos);
}

// The Part 11.6 requirement itself. The child logs a fault and then abort()s, which runs no
// destructors and flushes no buffers. If EventLog buffered anything in userspace, the line
// would be lost.
TEST(EventLog, LineSurvivesProcessCrash) {
    const auto path = tempPath("crash");
    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        EventLog log(path);
        log.logFault("test", "about to crash");
        std::abort();
    }
    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFSIGNALED(status));  // it really did crash, not exit cleanly

    const auto lines = readLines(path);
    ASSERT_EQ(lines.size(), 2u);  // START + FAULT, and no SESSION_END: the destructor never ran
    EXPECT_NE(lines[1].find("desc=\"about to crash\""), std::string::npos);
}

TEST(EventLog, ConcurrentWritersProduceWholeLinesWithGaplessOrderedSequence) {
    const auto path = tempPath("concurrent");
    constexpr int kThreads = 8;
    constexpr int kPerThread = 500;
    {
        EventLog log(path);
        std::vector<std::thread> threads;
        for (int t = 0; t < kThreads; ++t) {
            threads.emplace_back([&log, t] {
                for (int i = 0; i < kPerThread; ++i) {
                    log.logGeneral("t" + std::to_string(t) + "_" + std::to_string(i));
                }
            });
        }
        for (auto& th : threads) th.join();
    }

    const auto lines = readLines(path);
    ASSERT_EQ(lines.size(), static_cast<size_t>(kThreads * kPerThread + 2));

    const std::regex lineRe(R"(^(\d+\.\d{3}) #(\d+) (SESSION_START|SESSION_END|INFO)( .*)?$)");
    double lastMs = -1.0;
    std::set<std::string> messages;
    for (size_t i = 0; i < lines.size(); ++i) {
        std::smatch m;
        ASSERT_TRUE(std::regex_match(lines[i], m, lineRe)) << "malformed line: " << lines[i];
        EXPECT_EQ(std::stoull(m[2]), i) << "sequence gap or reorder at line " << i;
        const double ms = std::stod(m[1]);
        EXPECT_GE(ms, lastMs) << "timestamp went backwards at line " << i;
        lastMs = ms;
        if (m[3] == "INFO") messages.insert(m[4]);
    }
    EXPECT_EQ(messages.size(), static_cast<size_t>(kThreads * kPerThread));  // none lost or doubled
}

TEST(EventLog, ThrowsWhenTheFileCannotBeOpened) {
    EXPECT_THROW(EventLog("/nonexistent-directory/session.log"), std::runtime_error);
}

// /dev/full accepts open() and fails every write() with ENOSPC. That makes it a convenient way
// to exercise the "disk full" path without filling a real disk.
TEST(EventLog, WriteFailureLatchesUnhealthy) {
    EventLog log("/dev/full");
    EXPECT_FALSE(log.healthy());
    log.logGeneral("still failing");
    EXPECT_FALSE(log.healthy());
}

TEST(EventLog, HealthyWhenWritesSucceed) {
    EventLog log(tempPath("healthy"));
    log.logActuationRequest(ActuationRequest{}, "");
    EXPECT_TRUE(log.healthy());
}
