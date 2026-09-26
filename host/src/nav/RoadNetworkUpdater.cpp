#include "ar_drive_assist/nav/RoadNetworkUpdater.h"

#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>

namespace ar_drive_assist {

namespace fs = std::filesystem;

RoadNetworkUpdater::RoadNetworkUpdater(std::string baseOsrm, std::string workRoot)
    : baseOsrm_(std::move(baseOsrm)), workRoot_(std::move(workRoot)) {}

std::string RoadNetworkUpdater::apply(const RoadStatus& status, std::uint64_t nowMs) {
    if (status.activeOverrides(nowMs).empty()) return baseOsrm_;  // pristine map, nothing to copy
    try {
        const fs::path base(baseOsrm_);
        const fs::path baseDir = fs::canonical(base.parent_path());
        fs::create_directories(workRoot_);
        const fs::path work = fs::path(workRoot_) / ("reweighted-" + std::to_string(::getpid()) +
                                                     "-" + std::to_string(++counter_));
        fs::remove_all(work);
        fs::create_directories(work);
        for (const auto& e : fs::directory_iterator(baseDir)) {
            if (e.path().filename().string().rfind(base.filename().string(), 0) == 0) {
                fs::copy_file(e.path(), work / e.path().filename());
            }
        }
        const fs::path csv = work / "segment_speeds.csv";
        if (!status.writeSegmentSpeedCsv(csv.string(), nowMs)) {
            lastError_ = "cannot write " + csv.string();
            return "";
        }
        const fs::path osrm = work / base.filename();
        const std::string cmd = "osrm-customize --segment-speed-file '" + csv.string() + "' '" +
                                osrm.string() + "' > '" + (work / "customize.log").string() +
                                "' 2>&1";
        if (std::system(cmd.c_str()) != 0) {
            lastError_ = "osrm-customize failed; see " + (work / "customize.log").string();
            return "";
        }
        // Keep the two newest copies.
        std::vector<fs::path> copies;
        for (const auto& e : fs::directory_iterator(workRoot_)) {
            if (e.path().filename().string().rfind("reweighted-", 0) == 0)
                copies.push_back(e.path());
        }
        std::sort(copies.begin(), copies.end(), [](const fs::path& a, const fs::path& b) {
            return fs::last_write_time(a) < fs::last_write_time(b);
        });
        for (std::size_t i = 0; i + 2 < copies.size(); ++i) fs::remove_all(copies[i]);
        return osrm.string();
    } catch (const std::exception& e) {
        lastError_ = e.what();
        return "";
    }
}

}  // namespace ar_drive_assist
