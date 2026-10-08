// LivoxPackets — see include/ar_drive_assist/lidar/LivoxPackets.h.
#include "ar_drive_assist/lidar/LivoxPackets.h"

#include <algorithm>
#include <cstring>

namespace ar_drive_assist {

namespace {

template <typename T>
T le(const std::uint8_t* p) {  // little-endian, any alignment
    T v;
    std::memcpy(&v, p, sizeof v);
    return v;
}

constexpr std::size_t kHeader = 36;

}  // namespace

bool decodeLivoxPacket(const std::uint8_t* b, std::size_t len, LivoxPacketInfo& info,
                       std::vector<LivoxPoint>& out) {
    if (len < kHeader) return false;
    info.dataType = b[10];
    info.timeType = b[11];
    info.dotNum = le<std::uint16_t>(b + 5);
    const std::uint16_t interval = le<std::uint16_t>(b + 3);  // 0.1 us
    info.firstNs = static_cast<std::int64_t>(le<std::uint64_t>(b + 28));
    info.spanNs = static_cast<std::int64_t>(interval) * 100;
    const std::size_t size = info.dataType == 1 ? 14 : info.dataType == 2 ? 8 : 0;
    if (size == 0 || info.dotNum == 0 || len < kHeader + size * info.dotNum) return false;
    const std::uint8_t* d = b + kHeader;
    for (std::size_t i = 0; i < info.dotNum; ++i, d += size) {
        float x, y, z;
        std::uint8_t refl;
        if (info.dataType == 1) {
            const std::int32_t xi = le<std::int32_t>(d), yi = le<std::int32_t>(d + 4),
                               zi = le<std::int32_t>(d + 8);
            if (xi == 0 && yi == 0 && zi == 0) continue;  // no return
            x = xi * 1e-3f;
            y = yi * 1e-3f;
            z = zi * 1e-3f;
            refl = d[12];
        } else {
            const std::int16_t xi = le<std::int16_t>(d), yi = le<std::int16_t>(d + 2),
                               zi = le<std::int16_t>(d + 4);
            if (xi == 0 && yi == 0 && zi == 0) continue;
            x = xi * 1e-2f;
            y = yi * 1e-2f;
            z = zi * 1e-2f;
            refl = d[6];
        }
        LivoxPoint p;
        p.point.p = {x, y, z};
        p.point.intensity = refl;
        p.lidarNs =
            info.firstNs +
            (info.dotNum > 1 ? info.spanNs * static_cast<std::int64_t>(i) / info.dotNum : 0);
        out.push_back(p);
    }
    return true;
}

std::optional<LidarFrame> ScanAssembler::add(const std::vector<LivoxPoint>& pts,
                                             std::int64_t firstNs, std::int64_t hostUs) {
    // A LiDAR clock that goes backwards, or jumps a second ahead, has restarted: start over.
    if (lastNs_ >= 0 && (firstNs < lastNs_ - 1000000 || firstNs > lastNs_ + 1000000000LL)) {
        offsets_.clear();
        scanStartNs_ = -1;
        current_ = LidarFrame{};
        ++resets_;
    }
    lastNs_ = firstNs;
    // Clock mapping: the smallest (host - LiDAR) offset in the window.
    offsets_.emplace_back(firstNs, hostUs - firstNs / 1000);
    while (!offsets_.empty() && offsets_.front().first < firstNs - windowNs_) offsets_.pop_front();
    offsetUs_ = offsets_.front().second;
    for (const auto& o : offsets_) offsetUs_ = std::min(offsetUs_, o.second);

    std::optional<LidarFrame> done;
    if (scanStartNs_ < 0) scanStartNs_ = firstNs;
    if (firstNs >= scanStartNs_ + periodNs_) {
        // This packet opens the next scan: the current one is complete.
        current_.timestampMs =
            static_cast<std::uint64_t>((scanStartNs_ + periodNs_) / 1000 + offsetUs_) / 1000;
        if (!current_.points.empty()) done = std::move(current_);
        current_ = LidarFrame{};
        scanStartNs_ += periodNs_ * ((firstNs - scanStartNs_) / periodNs_);
    }
    for (const LivoxPoint& lp : pts) {
        LidarPoint p = lp.point;
        p.tUs = lp.lidarNs / 1000 + offsetUs_;
        current_.points.push_back(p);
    }
    return done;
}

}  // namespace ar_drive_assist
