// MapMatcher's OSRM backend: the real Match service (Part 11.3). Separate from MapMatcher.cpp so
// the unit tests link MapMatcher's logic without OSRM.

#include <cmath>
#include <osrm/match_parameters.hpp>

#include "OsrmEngine.h"
#include "ar_drive_assist/nav/MapMatcher.h"

namespace ar_drive_assist {
namespace {

using namespace osrm_detail;

class OsrmMatchBackend : public MatchBackend {
public:
    explicit OsrmMatchBackend(std::shared_ptr<osrm::OSRM> o) : osrm_(std::move(o)) {}

    BackendMatch matchLast(const std::vector<TracePoint>& trace) override {
        BackendMatch out;
        if (trace.empty()) return out;
        osrm::MatchParameters params;
        for (const TracePoint& p : trace) {
            params.coordinates.push_back(coord(p.p));
            params.radiuses.push_back(p.radiusM);
            // Whole seconds (MapMatcher spaces committed points >= 1 s apart).
            params.timestamps.push_back(static_cast<unsigned>(p.timestampMs / 1000));
        }
        // A single point cannot be matched as a trace; Match needs two. Duplicate it one second
        // earlier, which asks OSRM for the nearest plausible road.
        if (trace.size() == 1) {
            params.coordinates.insert(params.coordinates.begin(), params.coordinates.front());
            params.radiuses.insert(params.radiuses.begin(), params.radiuses.front());
            params.timestamps.insert(params.timestamps.begin(), params.timestamps.front() - 1);
        }
        // Keep timestamps strictly increasing even if two poses land in the same second.
        for (std::size_t i = 1; i < params.timestamps.size(); ++i) {
            if (params.timestamps[i] <= params.timestamps[i - 1])
                params.timestamps[i] = params.timestamps[i - 1] + 1;
        }
        params.gaps = osrm::MatchParameters::GapsType::Ignore;
        params.overview = osrm::MatchParameters::OverviewType::False;

        osrm::engine::api::ResultT result = osrm::json::Object();
        const auto status = osrm_->Match(params, result);
        const auto& json = std::get<osrm::json::Object>(result);
        try {
            if (status != osrm::Status::Ok || str(at(json, "code")) != "Ok") return out;
            const auto& tps = arr(at(json, "tracepoints")).values;
            const auto& last = tps.back();
            if (std::holds_alternative<osrm::json::Null>(last)) return out;  // newest not matched
            const auto& tp = obj(last);
            out.matched = true;
            out.p = lonLat(at(tp, "location"));
            const auto idx = static_cast<std::size_t>(num(at(tp, "matchings_index")));
            const auto& matchings = arr(at(json, "matchings")).values;
            if (idx < matchings.size()) out.confidence = num(at(obj(matchings[idx]), "confidence"));
        } catch (const std::exception&) {
            out = BackendMatch{};
        }
        return out;
    }

private:
    std::shared_ptr<osrm::OSRM> osrm_;
};

}  // namespace

bool MapMatcher::init(const std::string& osrmDataPath) {
    try {
        backend_ = std::make_unique<OsrmMatchBackend>(open(osrmDataPath));
        return true;
    } catch (const std::exception&) {
        backend_.reset();
        return false;
    }
}

}  // namespace ar_drive_assist
