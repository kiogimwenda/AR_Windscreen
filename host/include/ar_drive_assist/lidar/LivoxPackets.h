#pragma once
// LivoxPackets — the Livox Mid-360's point packets to timestamped points, and points to scans.
// Pure code, no SDK: the SDK's callback hands over the packet's bytes (LivoxCapture), and these
// are tested in CI with synthetic packets. See docs/BUILD_GUIDE.md Part 8.1.
//
// ---------------------------------------------------------------------------------------------
// The packet (Livox SDK2, livox_lidar_def.h, LivoxLidarEthernetPacket, #pragma pack(1)):
//   offset  0  version        u8
//           1  length         u16   bytes in the packet
//           3  time_interval  u16   time spanned by the packet's points, units of 0.1 us
//           5  dot_num        u16   points in the packet
//           7  udp_cnt        u16
//           9  frame_cnt      u8
//          10  data_type      u8    1 = Cartesian, int32 mm (14 B/point); 2 = int16 cm (8 B)
//          11  time_type      u8    0 = the LiDAR's own clock (ns since power-on); others synced
//          12  rsvd[12]
//          24  crc32          u32
//          28  timestamp      u64   ns, the FIRST point's time
//          36  data
// All little-endian. Point i was measured at timestamp + i * time_interval / dot_num. A point at
// (0, 0, 0) is "no return" and is dropped.
//
// ---------------------------------------------------------------------------------------------
// Two clocks. The Mid-360 stamps points with its own clock (unless synchronised by PTP or a GNSS
// PPS, which this build does not do); everything in the host runs on the steady clock. The scan
// assembler maps one onto the other: offset = host arrival time - LiDAR time of a packet, and the
// SMALLEST offset over a sliding window is the packet that waited least in the network and the
// scheduler, so it is the best estimate of the true offset (one late packet never moves it). The
// window keeps up with the two oscillators' drift. A LiDAR clock that jumps (a reboot) resets it.
// ---------------------------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "ar_drive_assist/lidar/LidarFrame.h"

namespace ar_drive_assist {

struct LivoxPacketInfo {
    std::uint8_t dataType = 0, timeType = 0;
    std::uint16_t dotNum = 0;
    std::int64_t firstNs = 0;  // LiDAR clock
    std::int64_t spanNs = 0;   // first to last point
};

// One decoded point, still on the LiDAR clock.
struct LivoxPoint {
    LidarPoint point;  // metres, intensity = reflectivity; point.tUs not yet set
    std::int64_t lidarNs = 0;
};

// Decodes a packet. False (and nothing appended) for a truncated or unsupported packet.
bool decodeLivoxPacket(const std::uint8_t* bytes, std::size_t len, LivoxPacketInfo& info,
                       std::vector<LivoxPoint>& out);

class ScanAssembler {
public:
    // periodMs: scan length (the Mid-360 publishes 10 Hz). offsetWindowMs: how far back the
    // smallest-offset estimate looks.
    explicit ScanAssembler(std::uint64_t periodMs = 100, std::uint64_t offsetWindowMs = 2000)
        : periodNs_(static_cast<std::int64_t>(periodMs) * 1000000),
          windowNs_(static_cast<std::int64_t>(offsetWindowMs) * 1000000) {}

    // A decoded packet, which arrived at host time `hostUs`. Returns a finished scan when this
    // packet starts the next period (points carry host-clock times; timestampMs = the scan's end).
    std::optional<LidarFrame> add(const std::vector<LivoxPoint>& pts, std::int64_t firstNs,
                                  std::int64_t hostUs);

    std::int64_t offsetUs() const { return offsetUs_; }  // host = LiDAR + offset
    std::uint64_t clockResets() const { return resets_; }

private:
    std::int64_t periodNs_, windowNs_;
    std::deque<std::pair<std::int64_t, std::int64_t>> offsets_;  // (lidarNs, host - lidar us)
    std::int64_t offsetUs_ = 0;
    std::int64_t scanStartNs_ = -1;
    std::int64_t lastNs_ = -1;
    LidarFrame current_;
    std::uint64_t resets_ = 0;
};

}  // namespace ar_drive_assist
