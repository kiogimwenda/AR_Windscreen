// The Mid-360's point packets (LivoxPackets.h): decoding, per-point times, scans and the mapping
// of the LiDAR's clock onto the host's. Packets are built byte by byte to the SDK's layout.
#include <gtest/gtest.h>

#include <cstring>
#include <random>

#include "ar_drive_assist/lidar/LivoxPackets.h"

using namespace ar_drive_assist;

namespace {

template <typename T>
void put(std::vector<std::uint8_t>& b, std::size_t at, T v) {
    std::memcpy(b.data() + at, &v, sizeof v);
}

// A packet of `n` points, the first at `firstNs`, spanning `intervalTenthUs` (0.1 us units).
// Point i is at (1 + i, -2, 0.5) m, reflectivity 10 + i.
std::vector<std::uint8_t> packet(int n, std::int64_t firstNs, std::uint16_t intervalTenthUs = 1000,
                                 int type = 1) {
    const std::size_t size = type == 1 ? 14 : 8;
    std::vector<std::uint8_t> b(36 + size * n, 0);
    b[0] = 0;
    put<std::uint16_t>(b, 1, static_cast<std::uint16_t>(b.size()));
    put<std::uint16_t>(b, 3, intervalTenthUs);
    put<std::uint16_t>(b, 5, static_cast<std::uint16_t>(n));
    b[10] = static_cast<std::uint8_t>(type);
    put<std::uint64_t>(b, 28, static_cast<std::uint64_t>(firstNs));
    for (int i = 0; i < n; ++i) {
        const std::size_t at = 36 + size * i;
        if (type == 1) {
            put<std::int32_t>(b, at, 1000 * (1 + i));
            put<std::int32_t>(b, at + 4, -2000);
            put<std::int32_t>(b, at + 8, 500);
            b[at + 12] = static_cast<std::uint8_t>(10 + i);
        } else {
            put<std::int16_t>(b, at, static_cast<std::int16_t>(100 * (1 + i)));
            put<std::int16_t>(b, at + 2, -200);
            put<std::int16_t>(b, at + 4, 50);
            b[at + 6] = static_cast<std::uint8_t>(10 + i);
        }
    }
    return b;
}

}  // namespace

TEST(LivoxPacket, DecodesMillimetrePointsWithTheirOwnTimes) {
    const auto b = packet(96, 5'000'000'000LL, 1000);  // 96 points over 100 us
    LivoxPacketInfo info;
    std::vector<LivoxPoint> pts;
    ASSERT_TRUE(decodeLivoxPacket(b.data(), b.size(), info, pts));
    ASSERT_EQ(pts.size(), 96u);
    EXPECT_EQ(info.dataType, 1);
    EXPECT_NEAR(pts[0].point.p.x(), 1.0f, 1e-6);
    EXPECT_NEAR(pts[0].point.p.y(), -2.0f, 1e-6);
    EXPECT_NEAR(pts[0].point.p.z(), 0.5f, 1e-6);
    EXPECT_EQ(pts[5].point.intensity, 15);
    EXPECT_EQ(pts[0].lidarNs, 5'000'000'000LL);
    EXPECT_EQ(pts[48].lidarNs, 5'000'000'000LL + 100'000 * 48 / 96);  // halfway through 100 us
}

TEST(LivoxPacket, DecodesCentimetrePoints) {
    const auto b = packet(4, 1000, 100, 2);
    LivoxPacketInfo info;
    std::vector<LivoxPoint> pts;
    ASSERT_TRUE(decodeLivoxPacket(b.data(), b.size(), info, pts));
    ASSERT_EQ(pts.size(), 4u);
    EXPECT_NEAR(pts[3].point.p.x(), 4.0f, 1e-6);
    EXPECT_NEAR(pts[3].point.p.y(), -2.0f, 1e-6);
}

TEST(LivoxPacket, NoReturnsAreDroppedAndBadPacketsRefused) {
    auto b = packet(3, 0);
    std::memset(b.data() + 36 + 14, 0, 12);  // point 1 at (0, 0, 0): no return
    LivoxPacketInfo info;
    std::vector<LivoxPoint> pts;
    ASSERT_TRUE(decodeLivoxPacket(b.data(), b.size(), info, pts));
    EXPECT_EQ(pts.size(), 2u);
    pts.clear();
    EXPECT_FALSE(decodeLivoxPacket(b.data(), b.size() - 1, info, pts)) << "truncated";
    auto imu = packet(3, 0);
    imu[10] = 0;  // IMU data, not points
    EXPECT_FALSE(decodeLivoxPacket(imu.data(), imu.size(), info, pts));
    EXPECT_FALSE(decodeLivoxPacket(b.data(), 20, info, pts)) << "shorter than a header";
    EXPECT_TRUE(pts.empty());
}

// Packets every 1 ms (the Mid-360 sends ~200,000 points/s in 96-point packets: ~2 ms apart);
// network delays of 0.2-5 ms. Scans close every 100 ms of LiDAR time; the host times of the
// points come out within the smallest delay of the truth, whatever the jitter.
TEST(ScanAssembler, ScansEvery100MsWithTheClockMappedByTheLeastDelayedPacket) {
    ScanAssembler a;
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> delayMs(0.2, 5.0);
    const std::int64_t trueOffsetUs = 7'000'000'000LL;  // host clock = LiDAR clock + 7000 s
    std::vector<LidarFrame> scans;
    for (int k = 0; k < 1000; ++k) {  // 1 s of LiDAR time
        const std::int64_t ns = 1'000'000'000LL + k * 1'000'000LL;
        std::vector<LivoxPoint> pts(1);
        pts[0].point.p = {5, 0, 0};
        pts[0].lidarNs = ns;
        const std::int64_t host =
            ns / 1000 + trueOffsetUs + static_cast<std::int64_t>(delayMs(rng) * 1000);
        if (auto s = a.add(pts, ns, host)) scans.push_back(*s);
    }
    ASSERT_EQ(scans.size(), 9u);  // 10 periods, the last still open
    for (const auto& s : scans) EXPECT_EQ(s.points.size(), 100u);
    // the mapped offset is the true one plus the smallest delay seen (>= 0.2 ms, small)
    EXPECT_GE(a.offsetUs(), trueOffsetUs + 200);
    EXPECT_LE(a.offsetUs(), trueOffsetUs + 400);
    // a point's host time is its LiDAR time + the offset (so within 0.4 ms of the truth)
    const auto& last = scans.back().points.back();
    EXPECT_NEAR(
        static_cast<double>(last.tUs - (1'000'000'000LL + 899 * 1'000'000LL) / 1000 - trueOffsetUs),
        300, 200);
}

TEST(ScanAssembler, ALidarRestartResetsTheClock) {
    ScanAssembler a;
    std::vector<LivoxPoint> one(1);
    one[0].point.p = {1, 0, 0};
    for (int k = 0; k < 50; ++k) {
        one[0].lidarNs = 9'000'000'000LL + k * 1'000'000LL;
        a.add(one, one[0].lidarNs, 100'000'000 + k * 1000);
    }
    one[0].lidarNs = 5'000'000LL;  // the LiDAR rebooted: its clock starts again near zero
    a.add(one, one[0].lidarNs, 100'060'000);
    EXPECT_EQ(a.clockResets(), 1u);
    EXPECT_EQ(a.offsetUs(), 100'060'000 - 5'000);
}
